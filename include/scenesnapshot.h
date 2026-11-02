#ifndef SCENESNAPSHOT_H
#define SCENESNAPSHOT_H

#include <array>
#include <atomic>
#include <memory>
#include <vector>
#include <QtGlobal>
#include "simulationsettings.h"

enum class PrimitiveKind { None, Sphere, Box, Cylinder, Ground };

struct RenderPrimitive {
    PrimitiveKind kind = PrimitiveKind::None;
    std::array<double, 3> position{{0, 0, 0}};
    std::array<double, 12> rotation{{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0}};
    std::array<double, 3> dimensions{{0, 0, 0}};
    std::array<double, 3> color{{0, 0, 0}};
    int texture = -1;
    int robotIndex = -1;
    bool robotChassis = false;
    bool visible = true;
};

struct RobotSnapshot {
    int id = 0;
    int team = 0;
    bool on = true;
    double x = 0, y = 0, direction = 0;
    std::array<double, 3> velocity{{0, 0, 0}};
};

struct SceneSnapshot {
    quint64 generation = 0;
    quint64 tick = 0;
    double simulationTime = 0;
    double physicsHz = 0;
    double pendingTime = 0;
    bool lagging = false;
    SimulationSettings settings;
    std::vector<RenderPrimitive> primitives;
    std::vector<RobotSnapshot> robots;
    std::array<double, 3> ballPosition{{0, 0, 0}};
    unsigned odeHelpers = 0;
};

using SceneSnapshotPtr = std::shared_ptr<const SceneSnapshot>;

// Only pointer exchange is synchronized. Neither thread holds a rendering lock.
class SnapshotMailbox {
public:
    SceneSnapshotPtr latest() const { return std::atomic_load(&snapshot); }
    void publish(SceneSnapshotPtr value) { std::atomic_store(&snapshot, std::move(value)); }
private:
    SceneSnapshotPtr snapshot;
};

enum class SimulationActionKind {
    ResetRobot, SetRobotEnabled, PlaceRobot, PlaceBall, KickBall,
    WheelDelta, ResetSpeeds, BallForce, StopBall, SetBallVelocity,
    RobotKick, ToggleRoller, Formation, EnableTeam
};

struct SimulationAction {
    SimulationActionKind kind = SimulationActionKind::StopBall;
    quint64 generation = 0;
    int robot = -1;
    int team = 0;
    int formation = 0;
    bool enabled = true;
    bool reset = false;
    double x = 0, y = 0, z = 0;
    double direction = 0;
    std::array<double, 4> wheels{{0, 0, 0, 0}};
};

Q_DECLARE_METATYPE(SimulationAction)
#endif
