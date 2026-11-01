#include <QApplication>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdarg>
#include "configwidget.h"
#include "sslworld.h"

static unsigned lcpErrors = 0;
static void recordOdeMessage(int code, const char* format, va_list arguments)
{
    if (code == d_ERR_LCP) ++lcpErrors;
    else { std::vfprintf(stderr, format, arguments); std::fputc('\n', stderr); }
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
    dInitODE2(0);
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
    passed = check(lcpErrors == 0, "contact fixtures complete without LCP solver errors") && passed;
    dSetMessageHandler(previousMessageHandler);
    dCloseODE();
    return passed ? 0 : 1;
}
