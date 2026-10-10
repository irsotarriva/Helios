#include "simulation_host.hpp"

#include "helios/core/logging.hpp"

#include <algorithm>
#include <chrono>
#include <utility>

namespace helios::app {

SimulationHost::SimulationHost(sim::Simulation simulation, sim::Focus focus)
    : simulation_(std::move(simulation)), focus_(focus) {
    tick(0.0, 0.0); // publish a first snapshot immediately
}

SimulationHost::~SimulationHost() {
    stop_ = true;
    if (thread_.joinable()) {
        thread_.join();
    }
}

void SimulationHost::start(double tick_rate_hz) {
    thread_ = std::thread([this, tick_rate_hz] {
        using Clock = std::chrono::steady_clock;
        const auto period =
            std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / tick_rate_hz));
        // Rationale: a tick advances the clock by the wall time since the last one. If a tick is
        // slow (high warp, heavy propagation), passing that whole time on makes the next tick
        // cover even more simulated time and run slower still, until one tick spans months and
        // leaps straight onto the next event. Capping it lets the simulation fall behind the
        // wall clock instead.
        const double max_tick_wall_s = 2.0 / tick_rate_hz;
        auto previous = Clock::now();
        auto next = previous + period;
        while (!stop_) {
            std::this_thread::sleep_until(next);
            const auto now = Clock::now();
            const double elapsed_s = std::chrono::duration<double>(now - previous).count();
            tick(std::min(elapsed_s, max_tick_wall_s), elapsed_s);
            previous = now;
            next += period;
            if (next < now) {
                next = now + period; // fell behind (debugger, sleep): do not try to catch up
            }
        }
    });
}

void SimulationHost::step(double wall_dt_s) {
    tick(wall_dt_s, wall_dt_s);
}

void SimulationHost::post(Command command) {
    const std::scoped_lock lock(mutex_);
    pending_.push_back(std::move(command));
}

void SimulationHost::set_focus(sim::Focus focus) {
    const std::scoped_lock lock(mutex_);
    focus_ = focus;
}

void SimulationHost::tick(double wall_dt_s, double elapsed_wall_s) {
    std::vector<Command> commands;
    sim::Focus focus;
    {
        const std::scoped_lock lock(mutex_);
        commands.swap(pending_);
        focus = focus_;
    }
    // The pilot going through an airlock changes vessel without the player choosing one: out
    // into the suit, or back in from it. The player's eyes go with the pilot.
    const auto pilot_vessel = [&]() -> std::optional<sim::VesselId> {
        const auto& pilot = simulation_.pilot();
        return pilot.has_value() ? std::optional(pilot->vessel) : std::nullopt;
    };
    const auto follow_pilot = [&](std::optional<sim::VesselId>& before) {
        const std::optional<sim::VesselId> after = pilot_vessel();
        if (before.has_value() && after.has_value() && *after != *before
            && focus == sim::Focus::vessel(*before)) {
            focus = sim::Focus::vessel(*after);
            boarding_tried_ = after;
            const std::scoped_lock lock(mutex_);
            focus_ = focus;
        }
        before = after;
    };
    std::optional<sim::VesselId> pilot_before = pilot_vessel();
    for (const Command& command : commands) {
        command(simulation_);
    }
    follow_pilot(pilot_before);
    // The vessel the player looks at is the one they fly; looking at a body changes nothing.
    if (focus.kind == sim::Focus::Kind::Vessel && simulation_.active_vessel() != sim::VesselId{focus.index}) {
        if (core::VoidResult activated = simulation_.set_active_vessel(sim::VesselId{focus.index});
            !activated) {
            LOG_WARN("cannot fly that vessel: {}", core::describe(activated.error())).tag("subsystem", "sim");
        }
    }
    // The pilot goes where the player looks: into the seat of the focused vessel, if it has
    // one. Asked once per change of focus; most vessels have no seat.
    if (focus.kind == sim::Focus::Kind::Vessel && boarding_tried_ != sim::VesselId{focus.index}) {
        boarding_tried_ = sim::VesselId{focus.index};
        const auto& pilot = simulation_.pilot();
        if (!pilot.has_value() || pilot->vessel != *boarding_tried_) {
            if (core::VoidResult boarded = simulation_.board(*boarding_tried_); !boarded) {
                LOG_DEBUG("no boarding: {}", core::describe(boarded.error())).tag("subsystem", "sim");
            }
        }
    }
    pilot_before = pilot_vessel();
    if (core::VoidResult advanced = simulation_.advance(wall_dt_s); !advanced) {
        LOG_ERROR("simulation step failed: {}", core::describe(advanced.error())).tag("subsystem", "sim");
    }
    follow_pilot(pilot_before);
    auto snapshot = sim::build_snapshot(simulation_, focus);
    if (!snapshot) {
        LOG_WARN("cannot build a snapshot: {}", core::describe(snapshot.error())).tag("subsystem", "sim");
        return;
    }
    // Report simulated seconds per real second: below the warp factor when the simulation
    // cannot keep up.
    if (elapsed_wall_s > wall_dt_s) {
        snapshot->warp_factor *= wall_dt_s / elapsed_wall_s;
    }
    exchange_.publish(std::make_shared<const sim::SceneSnapshot>(std::move(*snapshot)));
}

} // namespace helios::app
