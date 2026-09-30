#ifndef HELIOS_APPS_HELIOS_SIMULATION_HOST_HPP
#define HELIOS_APPS_HELIOS_SIMULATION_HOST_HPP

#include "helios/sim/scene_snapshot.hpp"
#include "helios/sim/simulation.hpp"

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace helios::app {

// Runs the Simulation and publishes a SceneSnapshot after every tick. The renderer only reads
// snapshots; everything that changes the simulation is posted as a command and runs between
// ticks on the simulation's thread, so the Simulation itself needs no locking.
//
// Two modes: start() runs ticks on a background thread at a fixed wall-clock rate (the game),
// step() runs one tick on the caller's thread (deterministic runs: screenshots, tests).
class SimulationHost {
public:
    using Command = std::function<void(sim::Simulation&)>;

    SimulationHost(sim::Simulation simulation, sim::Focus focus);
    SimulationHost(const SimulationHost&) = delete;
    SimulationHost& operator=(const SimulationHost&) = delete;
    SimulationHost(SimulationHost&&) = delete;
    SimulationHost& operator=(SimulationHost&&) = delete;
    ~SimulationHost();

    void start(double tick_rate_hz);
    void step(double wall_dt_s);

    void post(Command command);
    void set_focus(sim::Focus focus);
    [[nodiscard]] std::shared_ptr<const sim::SceneSnapshot> latest() const { return exchange_.latest(); }

private:
    void tick(double wall_dt_s);

    sim::Simulation simulation_;
    sim::SnapshotExchange exchange_;
    mutable std::mutex mutex_; // guards pending_ and focus_
    std::vector<Command> pending_;
    sim::Focus focus_;
    std::atomic<bool> stop_{false};
    std::thread thread_;
};

} // namespace helios::app

#endif // HELIOS_APPS_HELIOS_SIMULATION_HOST_HPP
