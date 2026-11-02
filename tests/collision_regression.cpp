#include <QApplication>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdarg>
#include <atomic>
#include "configwidget.h"
#include "sslworld.h"

static std::atomic<unsigned> lcpErrors{0};
static void recordOdeMessage(int code, const char* format, va_list arguments)
{
    if (code == d_ERR_LCP) ++lcpErrors;
    std::vfprintf(stderr, format, arguments); std::fputc('\n', stderr);
}

static bool check(bool passed, const char* description)
{
    std::printf("%s %s\n", passed ? "PASS" : "FAIL", description);
    return passed;
}

static void placeBall(SSLWorld& world, double x, double y, double z)
{
    for (auto* robot : world.robots) if (robot) robot->kicker->unholdBall();
    world.ball->setBodyPosition(x, y, z);
    dBodySetLinearVel(world.ball->body, 0, 0, 0);
    dBodySetAngularVel(world.ball->body, 0, 0, 0);
    dBodySetForce(world.ball->body, 0, 0, 0);
    dBodySetTorque(world.ball->body, 0, 0, 0);
}

static bool checkRobot(SimulationSettings settings, int team, double heading)
{
    // Isolate hull/ball geometry from rolling constraints against a kinematic
    // chassis. Dynamic dribble/rolling and robot-pair tests use full defaults.
    settings.value_BallRollingDeceleration = 0;
    settings.value_BallFriction = 0;
    const auto& teamSettings = team ? settings.yellowSettings : settings.blueSettings;
    std::printf("CHECK team=%d heading=%.0f radius=%.1fmm dribbler=%.1fmm\n", team, heading,
                teamSettings.RobotRadius*1000, teamSettings.RobotCenterFromKicker*1000);
    RobotsFormation formation(FORMATION_INSIDE_1, &settings);
    SSLWorld world(nullptr, &settings, &formation, &formation);
    auto* robot = world.robots[team];
    robot->setXY(0, 0);
    robot->setDir(heading);
    placeBall(world, 2, 2, 0.2);
    for (int i = 0; i < 200; ++i) world.p->step(1.0/300.0);
    // Hold the chassis still to measure ball penetration rather than robot pushback.
    dBodySetKinematic(robot->chassis->body);
    // A held chassis does not need driven wheels constrained against the floor.
    for (auto* wheel : robot->wheels) {
        dJointSetAMotorParam(wheel->motor, dParamFMax, 0);
        dGeomDisable(wheel->cyl->geom);
        dBodySetGravityMode(wheel->cyl->body, 0);
    }

    dBodySetLinearVel(robot->chassis->body,0,0,0);
    dBodySetAngularVel(robot->chassis->body,0,0,0);
    world.p->setGravity(0);
    const auto* position = dBodyGetPosition(robot->chassis->body);
    const double centerX = position[0], centerY = position[1];
    const double radius = robot->settings.RobotRadius, ballRadius = settings.BallRadius();
    bool passed = check(world.p->findSurface(robot->ballHull, world.ball) != nullptr &&
                        dGeomGetBody(robot->ballHull->geom) == robot->chassis->body,
                        "ball contact is registered on the visible chassis hull");

    auto* wall = world.walls[2];
    const auto* original = dGeomGetPosition(wall->geom);
    const dVector3 oldWall = {original[0], original[1], original[2]};
    dVector3 dimensions;
    dGeomBoxGetLengths(wall->geom, dimensions);
    dGeomSetPosition(wall->geom, centerX+radius+dimensions[0]/2-0.001, centerY, position[2]);
    dContactGeom wallContacts[8];
    passed = check(world.p->findSurface(robot->chassis, wall) != nullptr &&
                   dCollide(robot->chassis->geom, wall->geom, 8, wallContacts, sizeof(dContactGeom)) > 0,
                   "existing chassis-to-wall contact remains available") && passed;
    dGeomSetPosition(wall->geom, oldWall[0], oldWall[1], oldWall[2]);

    auto sideImpacts = [&](double height) {
        bool blockedAll = true;
        for (double side : {90.0, 180.0, 270.0}) {
            const double angle = (heading + side) * M_PI / 180;
            const double x = std::cos(angle), y = std::sin(angle);
            const double start = radius + ballRadius + 0.03;
                placeBall(world, centerX + x*start, centerY + y*start, height);
            dBodySetLinearVel(world.ball->body, -0.6*x, -0.6*y, 0);
            double closest = start;
            for (int i = 0; i < 90; ++i) {
                world.p->step(1.0/300.0);
                const auto* ball = dBodyGetPosition(world.ball->body);
                closest = std::min(closest, (ball[0]-centerX)*x + (ball[1]-centerY)*y);
            }
            const bool blocked = closest >= radius + ballRadius - 0.002;
            std::printf("%s team=%d heading=%.0f side=%.0f height=%.2fmm closest=%.2fmm expected=%.2fmm\n",
                        blocked ? "PASS" : "FAIL", team, heading, side, height*1000, closest*1000, (radius+ballRadius)*1000);
            blockedAll = blocked && blockedAll;
        }

        return blockedAll;
    };
    // Separate the geometry-only fixture from an unloaded rolling constraint.
    passed = sideImpacts(position[2]) && passed;
    // Retain ground-level impacts under a real normal load as well.
    world.p->setGravity(settings.Gravity());
    passed = sideImpacts(ballRadius) && passed;
    world.p->setGravity(0);

    const double yaw = heading * M_PI / 180, x = std::cos(yaw), y = std::sin(yaw);
    const auto* kicker = dBodyGetPosition(robot->kicker->box->body);
    const double front = robot->settings.KickerThickness/2 + ballRadius;
    const double ballX = kicker[0]+x*front, ballY = kicker[1]+y*front;
    placeBall(world, ballX-x*0.00005, ballY-y*0.00005, ballRadius);
    dContactGeom contacts[8];
    passed = check(dCollide(robot->ballHull->geom, world.ball->geom, 8, contacts, sizeof(dContactGeom)) == 0 &&
                   dCollide(robot->kicker->box->geom, world.ball->geom, 8, contacts, sizeof(dContactGeom)) > 0,
                   "front cutout lets the ball reach the dribbler face") && passed;

    placeBall(world, ballX+x*0.00005, ballY+y*0.00005, ballRadius);
    robot->kicker->setRoller(1);
    robot->kicker->step();
    bool retained = dAreConnected(robot->kicker->box->body, world.ball->body);
    for (int i = 0; i < 60; ++i) {
        world.p->step(1.0/300.0);
        robot->kicker->step();
        retained = retained && dAreConnected(robot->kicker->box->body, world.ball->body);
    }
    passed = check(retained && robot->kicker->isTouchingBall(),
                   "roller retains the ball against the front for repeated physics steps") && passed;
    dBodySetLinearVel(robot->chassis->body, 0.2*x, 0.2*y, 0);
    for (int i = 0; i < 90; ++i) {
        world.p->step(1.0/300.0);
        robot->kicker->step();
        retained = retained && dAreConnected(robot->kicker->box->body, world.ball->body);
    }
    dBodySetLinearVel(robot->chassis->body, 0, 0, 0);
    const auto* moved = dBodyGetPosition(robot->chassis->body);
    passed = check(retained && robot->kicker->isTouchingBall() &&
                   (moved[0]-centerX)*x + (moved[1]-centerY)*y > 0.04,
                   "roller retains the ball while the robot translates") && passed;
    const auto* kickStart = dBodyGetPosition(world.ball->body);
    const double kickX = kickStart[0], kickY = kickStart[1];
    robot->kicker->kick(3, 0);
    const auto* flat = dBodyGetLinearVel(world.ball->body);
    passed = check(!dAreConnected(robot->kicker->box->body, world.ball->body) && flat[0]*x+flat[1]*y > 2.5,
                   "flat kick releases the retained ball forwards") && passed;
    for (int i = 0; i < 60; ++i) world.p->step(1.0/300.0);
    const auto* kicked = dBodyGetPosition(world.ball->body);
    passed = check((kicked[0]-kickX)*x + (kicked[1]-kickY)*y > 0.2,
                   "kicked ball clears the chassis without being trapped") && passed;
    const auto* finalKicker = dBodyGetPosition(robot->kicker->box->body);
    placeBall(world, finalKicker[0]+x*(front+0.00005), finalKicker[1]+y*(front+0.00005), ballRadius);
    robot->kicker->kick(3, 3);
    const auto* chip = dBodyGetLinearVel(world.ball->body);
    passed = check(chip[0]*x+chip[1]*y > 2.5 && chip[2] > 2.5,
                   "front cutout preserves chip kicks") && passed;

    const auto* chassis = dBodyGetPosition(robot->chassis->body);
    const double approach = radius + ballRadius + 0.03;
    placeBall(world, chassis[0]+x*approach, chassis[1]+y*approach, ballRadius);
    dBodySetLinearVel(world.ball->body, -0.6*x, -0.6*y, 0);
    robot->kicker->setRoller(1);
    bool acquired = false;
    for (int i = 0; i < 120 && !acquired; ++i) {
        world.p->step(1.0/300.0);
        robot->kicker->step();
        acquired = dAreConnected(robot->kicker->box->body, world.ball->body);
    }
    passed = check(acquired && robot->kicker->isTouchingBall(),
                   "dribbler acquires a ball approaching from outside the chassis") && passed;
    robot->kicker->kick(3, 0);
    const auto* acquiredKick = dBodyGetLinearVel(world.ball->body);
    passed = check(acquired && !dAreConnected(robot->kicker->box->body, world.ball->body) &&
                   acquiredKick[0]*x+acquiredKick[1]*y > 2.5,
                   "dribbler can kick the ball acquired through the hull opening") && passed;
    return passed;
}

// Use the production pair registration, motors and five-substep integrator.
static bool checkRobotPair(SimulationSettings settings, double heading, double side, unsigned helpers)
{
    settings.value_Robots_Count = 1;
    settings.value_DeltaTime = 1.0/60;
    RobotsFormation formation(FORMATION_INSIDE_1, &settings);
    SSLWorld world(nullptr, &settings, &formation, &formation);
    if (!check(world.p->setThreadCount(helpers), "robot-pair ODE helpers are available")) return false;
    auto* first = world.robots[0];
    auto* second = world.robots[1];
    bool preservedMass = true;
    for (auto* robot : {first, second}) {
        dMass mass;
        dBodyGetMass(robot->chassis->body, &mass);
        const auto& shape = robot->settings;
        const double cylinder = shape.BodyMass*0.99f, proxy = shape.BodyMass*0.01f;
        const double sphereInertia = 0.4*proxy*shape.RobotCenterFromKicker*shape.RobotCenterFromKicker;
        const double transverse = cylinder*(3*shape.RobotRadius*shape.RobotRadius+
                                            shape.RobotHeight*shape.RobotHeight)/12+sphereInertia;
        const double axial = 0.5*cylinder*shape.RobotRadius*shape.RobotRadius+sphereInertia;
        preservedMass = preservedMass && std::abs(mass.mass-cylinder-proxy) < 1e-12 &&
            std::abs(mass.I[0]-transverse) < 1e-12 && std::abs(mass.I[5]-transverse) < 1e-12 &&
            std::abs(mass.I[10]-axial) < 1e-12;
    }
    if (!check(preservedMass, "compound robot preserves total chassis/proxy mass and inertia")) return false;
    first->step(); second->step();
    const double angle = (heading+side)*M_PI/180;
    const double x = std::cos(angle), y = std::sin(angle);
    const double diameter = first->settings.RobotRadius+second->settings.RobotRadius;
    const double start = diameter+0.02;
    first->setXY(-x*start/2, -y*start/2);
    second->setXY(x*start/2, y*start/2);
    first->setDir(heading); second->setDir(heading);
    placeBall(world, 2, 2, settings.BallRadius());
    for (int i=0; i<60; ++i) world.stepPhysics(1.0/60);

    unsigned pairContacts = 0;
    PWorld::StepStatistics statistics;
    statistics.contactObserver = [&](const dContact& contact) {
        if ((contact.geom.g1 == first->dummy->geom && contact.geom.g2 == second->dummy->geom) ||
            (contact.geom.g2 == first->dummy->geom && contact.geom.g1 == second->dummy->geom))
            ++pairContacts;
    };
    const double local = side*M_PI/180;
    double closest = start;
    for (int i=0; i<120; ++i) {
        first->setSpeed(0.25*std::cos(local), 0.25*std::sin(local), 0);
        second->setSpeed(-0.25*std::cos(local), -0.25*std::sin(local), 0);
        world.stepPhysics(1.0/60, PhysicsStepOptions(), &statistics);
        const auto* a = dBodyGetPosition(first->chassis->body);
        const auto* b = dBodyGetPosition(second->chassis->body);
        const double distance = std::hypot(a[0]-b[0], a[1]-b[1]);
        if (!std::isfinite(distance)) return check(false, "robot-pair positions stay finite");
        closest = std::min(closest, distance);
    }
    const bool blocked = pairContacts > 0 && closest >= diameter-0.002;
    first->resetRobot();
    first->setXY(1, -1);
    first->setDir(heading+45);
    const auto* center = dGeomGetPosition(first->dummy->geom);
    const bool followsPose = std::hypot(center[0]-1, center[1]+1) < 1e-12;
    const bool passed = blocked && followsPose;
    std::printf("%s robot-pair radii=%.1f/%.1fmm heading=%.0f side=%.0f helpers=%u "
                "closest=%.2fmm expected=%.2fmm contacts=%u\n",
                passed ? "PASS" : "FAIL", first->settings.RobotRadius*1000,
                second->settings.RobotRadius*1000, heading, side, helpers,
                closest*1000, diameter*1000, pairContacts);
    return passed;
}

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    ConfigWidget config(false);
    config.v_Robots_Count->setInt(1);
    auto settings = config.simulationSettings();
    // Exercise separate team geometry, including the smaller ParsianNew radius.
    settings.yellowSettings.RobotRadius = 0.0793;
    settings.yellowSettings.RobotCenterFromKicker = 0.068;
    settings.yellowSettings.RobotHeight = 0.138;
    if (!dInitODE2(dInitFlagManualThreadCleanup) ||
        !dAllocateODEDataForThread(dAllocateMaskAll)) return 2;
    const auto previousMessageHandler = dGetMessageHandler();
    dSetMessageHandler(recordOdeMessage);
    bool passed = true;
    for (int team : {0, 1})
        for (double heading : {0.0, 90.0})
            passed = checkRobot(settings, team, heading) && passed;
    settings.blueSettings.RobotRadius = 0.09;
    // Include a cut between vertices and one at the original 60-degree vertex.
    for (double distance : {0.04, 0.045}) {
        settings.blueSettings.RobotCenterFromKicker = distance;
        for (double heading : {0.0, 90.0})
            passed = checkRobot(settings, 0, heading) && passed;
    }
    // Reverse the team sizes to catch either team's collider using shared settings.
    for (int sizes=0; sizes<3; ++sizes) {
        settings.blueSettings.RobotRadius = sizes == 2 ? 0.0793 : 0.09;
        settings.yellowSettings.RobotRadius = sizes == 1 ? 0.0793 : 0.09;
        settings.blueSettings.RobotCenterFromKicker = sizes == 2 ? 0.068 : 0.073;
        settings.yellowSettings.RobotCenterFromKicker = sizes == 1 ? 0.068 : 0.073;
        for (double heading : {0.0, 90.0}) for (double side : {90.0, 180.0}) {
            passed = checkRobotPair(settings, heading, side, 0) && passed;
            if (dCheckConfiguration("ODE_EXT_threading") && dCheckConfiguration("ODE_THR_builtin_impl"))
                passed = checkRobotPair(settings, heading, side, 3) && passed;
        }
    }
    passed = check(lcpErrors == 0, "contact fixtures complete without LCP solver errors") && passed;
    dSetMessageHandler(previousMessageHandler);
    dCleanupODEAllDataForThread();
    dCloseODE();
    return passed ? 0 : 1;
}
