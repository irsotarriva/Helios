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
    for (const Command& command : commands) {
        command(simulation_);
    }
    if (core::VoidResult advanced = simulation_.advance(wall_dt_s); !advanced) {
        LOG_ERROR("simulation step failed: {}", core::describe(advanced.error())).tag("subsystem", "sim");
    }
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
