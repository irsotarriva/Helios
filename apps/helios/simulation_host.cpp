#include "simulation_host.hpp"

#include "helios/core/logging.hpp"

#include <chrono>
#include <utility>

namespace helios::app {

SimulationHost::SimulationHost(sim::Simulation simulation, sim::Focus focus)
    : simulation_(std::move(simulation)), focus_(focus) {
    tick(0.0); // publish a first snapshot immediately
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
        auto previous = Clock::now();
        auto next = previous + period;
        while (!stop_) {
            std::this_thread::sleep_until(next);
            const auto now = Clock::now();
            tick(std::chrono::duration<double>(now - previous).count());
            previous = now;
            next += period;
            if (next < now) {
                next = now + period; // fell behind (debugger, sleep): do not try to catch up
            }
        }
    });
}

void SimulationHost::step(double wall_dt_s) {
    tick(wall_dt_s);
}

void SimulationHost::post(Command command) {
    const std::scoped_lock lock(mutex_);
    pending_.push_back(std::move(command));
}

void SimulationHost::set_focus(sim::Focus focus) {
    const std::scoped_lock lock(mutex_);
    focus_ = focus;
}

void SimulationHost::tick(double wall_dt_s) {
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
    exchange_.publish(std::make_shared<const sim::SceneSnapshot>(std::move(*snapshot)));
}

} // namespace helios::app
