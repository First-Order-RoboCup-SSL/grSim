#include <QApplication>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <future>
#include <stdexcept>
#include "configwidget.h"
#include "sslworld.h"

struct Result {
    std::vector<double> state;
    double milliseconds;
};

// Measure collision detection, dWorldStep, and robot motors without rendering,
// packet generation, timers, or real-time throttling. Default ball-ground
// contact resistance is included; the legacy custom ball force is not applied.
static Result simulate(SimulationSettings settings, unsigned helpers, int steps, bool crowded, bool prepareSerially = false)
{
    RobotsFormation formation(FORMATION_INSIDE_1, &settings);
    SSLWorld world(nullptr, &settings, &formation, &formation);
    if (!world.p->setThreadCount(prepareSerially ? 0 : helpers)) throw std::runtime_error("Requested ODE threading is unavailable");
    if (dSpaceGetManualCleanup(world.p->space) != 1)
        throw std::runtime_error("Collision space is missing manual TLS cleanup");
    for (int i = 0; i < settings.Robots_Count() * 2; ++i) {
        if (crowded) world.robots[i]->setXY((i % 4) * 0.185 - 0.28, (i / 4) * 0.185 - 0.28);
        world.robots[i]->setSpeed(0.15, (i % 2 ? 1 : -1) * 0.05, 0.1);
    }
    auto tick = [&]() {
        for (int substep = 0; substep < 5; ++substep)
            if (!world.p->step(settings.DeltaTime() / 5)) throw std::runtime_error("ODE physics step failed");
        for (int i = 0; i < settings.Robots_Count() * 2; ++i) world.robots[i]->step();
    };
    for (int i = 0; i < 60; ++i) tick();
    if (prepareSerially && !world.p->setThreadCount(helpers))
        throw std::runtime_error("Requested ODE threading is unavailable");
    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < steps; ++i) tick();
    Result result;
    result.milliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    for (const auto& primitive : world.p->renderPrimitives()) {
        result.state.insert(result.state.end(), primitive.position.begin(), primitive.position.end());
        result.state.insert(result.state.end(), primitive.rotation.begin(), primitive.rotation.end());
    }
    auto appendVelocity = [&](dBodyID body) {
        const auto* v = dBodyGetLinearVel(body);
        const auto* w = dBodyGetAngularVel(body);
        result.state.insert(result.state.end(), v, v + 3);
        result.state.insert(result.state.end(), w, w + 3);
        for (int axis = 0; axis < 3; ++axis)
            if (std::abs(v[axis]) > 20 || std::abs(w[axis]) > 200)
                throw std::runtime_error("Unbounded body velocity");
    };
    appendVelocity(world.ball->body);
    for (int i = 0; i < settings.Robots_Count() * 2; ++i) appendVelocity(world.robots[i]->chassis->body);
    for (int i = 0; i < settings.Robots_Count() * 2; ++i) {
        const auto* position = dBodyGetPosition(world.robots[i]->chassis->body);
        if (position[2] < -0.05 || position[2] > 1)
            throw std::runtime_error("Robot escaped the ground contact");
    }
    for (double value : result.state)
        if (!std::isfinite(value)) throw std::runtime_error("Non-finite physics state");
    return result;
}

static bool equivalent(const Result& expected, const Result& actual, const char* description)
{
    double error = expected.state.size() == actual.state.size() ? 0 : INFINITY;
    for (size_t i = 0; i < std::min(expected.state.size(), actual.state.size()); ++i)
        error = std::max(error, std::abs(expected.state[i] - actual.state[i]));
    const bool passed = error < 1e-8;
    std::printf("%s %s max_state_difference=%.12g\n", passed ? "PASS" : "FAIL", description, error);
    return passed;
}

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    ConfigWidget config(false);
    bool benchmark = false, crowded = false;
    unsigned helpers = 0;
    int steps = 180, robots = 11;
    for (int i = 1; i < argc; ++i) {
        const QString option = QString::fromLocal8Bit(argv[i]);
        if (option == "--benchmark") benchmark = true;
        else if (option == "--crowded") crowded = true;
        else if (i + 1 < argc && (option == "--threads" || option == "--steps" || option == "--robots")) {
            bool valid = false;
            const int value = QString::fromLocal8Bit(argv[++i]).toInt(&valid);
            if (!valid || value < 0) return 2;
            if (option == "--threads") helpers = unsigned(value);
            else if (option == "--steps") steps = value;
            else robots = value;
        } else return 2;
    }
    if (helpers > 16 || robots < 1 || robots > MAX_ROBOT_COUNT || steps < 1) return 2;
    config.v_Robots_Count->setInt(robots);
    SimulationSettings settings = config.simulationSettings();
    settings.value_DeltaTime = 1.0 / 60.0;
    if (!dInitODE2(dInitFlagManualThreadCleanup)) return 2;
    if (!dCheckConfiguration("ODE_double_precision") || !dAllocateODEDataForThread(dAllocateMaskAll)) return 2;
    bool passed = true;
    try {
        if (benchmark) {
            const auto result = simulate(settings, helpers, steps, crowded);
            QJsonObject output;
            output["ode_version"] = QString::fromLatin1(dODE_VERSION);
            output["configuration"] = QString::fromLatin1(dGetConfiguration());
            output["helpers"] = int(helpers);
            output["robots_per_team"] = robots;
            output["steps"] = steps;
            output["crowded"] = crowded;
            output["wall_ms"] = result.milliseconds;
            output["ms_per_tick"] = result.milliseconds / steps;
            QJsonArray state;
            for (double value : result.state) state.append(value);
            output["state"] = state;
            std::puts(QJsonDocument(output).toJson(QJsonDocument::Compact).constData());
        } else {
            for (bool contacts : {false, true}) {
                // Threaded long crowded trajectories diverge. Check agreement
                // for one tick from identical serial preparation, then bounded states;
                // later contact ordering can diverge; boundedness is not equivalence.
                const int comparisonSteps = contacts ? 1 : steps;
                const auto serial = simulate(settings, 0, comparisonSteps, contacts, contacts);
                if (dCheckConfiguration("ODE_EXT_threading") && dCheckConfiguration("ODE_THR_builtin_impl")) {
                    passed = equivalent(serial, simulate(settings, 2, comparisonSteps, contacts, contacts),
                        contacts ? "two helpers agree for one crowded tick from identical serial preparation" : "two helpers agree in match formation") && passed;
                    passed = equivalent(serial, simulate(settings, 4, comparisonSteps, contacts, contacts),
                        contacts ? "four helpers agree for one crowded tick from identical serial preparation" : "four helpers agree in match formation") && passed;
                    if (contacts) {
                        for (unsigned count : {0U, 2U, 4U}) simulate(settings, count, steps, true);
                        std::puts("PASS long crowded runs retain finite, bounded body states");
                    }
                } else std::puts("SKIP optional ODE helper threads unavailable in this library");
            }
            if (dCheckConfiguration("ODE_EXT_mt_collisions") && dCheckConfiguration("ODE_EXT_threading")) {
                const auto expected = simulate(settings, 0, steps, false);
                auto run = [settings, steps]() {
                    if (!dAllocateODEDataForThread(dAllocateMaskAll)) throw std::runtime_error("Worker TLS allocation failed");
                    try {
                        auto result = simulate(settings, 0, steps, false);
                        dCleanupODEAllDataForThread();
                        return result;
                    } catch (...) { dCleanupODEAllDataForThread(); throw; }
                };
                auto first = std::async(std::launch::async, run);
                auto second = std::async(std::launch::async, run);
                passed = equivalent(expected, first.get(), "first concurrent world is independent") && passed;
                passed = equivalent(expected, second.get(), "second concurrent world is independent") && passed;
            } else std::puts("SKIP concurrent worlds require ODE collision TLS and per-world schedulers");
        }
    } catch (const std::exception& error) {
        std::fprintf(stderr, "FAIL %s\n", error.what());
        passed = false;
    }
    dCleanupODEAllDataForThread();
    dCloseODE();
    return passed ? 0 : 1;
}
