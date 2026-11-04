#include <QApplication>
#include <QFile>
#include <QTemporaryDir>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include "configwidget.h"
#include "simulation.h"
#include "sslworld.h"
#include "grSim_Packet.pb.h"

namespace {
constexpr double tick = 1.0 / 60;

void require(bool condition, const char* description)
{
    if (!condition) throw std::runtime_error(description);
}

void close(double actual, double expected, const char* description)
{
    require(std::isfinite(actual) && std::abs(actual - expected) < 1e-12, description);
}

void checkCylinder(PCylinder* cylinder, double scale = 1)
{
    dReal radius, length;
    dGeomCylinderGetParams(cylinder->geom, &radius, &length);
    dMass mass;
    dBodyGetMass(cylinder->body, &mass);
    cylinder->setMass(mass.mass * scale);
    dBodyGetMass(cylinder->body, &mass);
    const double transverse = mass.mass * (radius*radius/4 + length*length/12);
    close(mass.I[0], transverse, "cylinder Ixx must match local-Z geometry");
    close(mass.I[5], transverse, "cylinder Iyy must match local-Z geometry");
    close(mass.I[10], mass.mass*radius*radius/2, "cylinder Izz must match local-Z geometry");
    for (int row = 0; row < 3; ++row)
        for (int column = 0; column < 3; ++column)
            if (row != column) close(mass.I[4*row+column], 0, "principal inertia must be diagonal");
}

void settingsMigration()
{
    QTemporaryDir directory;
    require(directory.isValid(), "temporary settings directory unavailable");
    const QString path = directory.filePath("settings.xml");
    auto writeLegacy = [&](const char* values) {
        QFile file(path);
        require(file.open(QIODevice::WriteOnly | QIODevice::Truncate), "temporary settings file unavailable");
        const QByteArray xml = QByteArray("<VarXML><Var name=\"Physics\" type=\"list\"><Var name=\"Ball\" type=\"list\">") +
            values + "</Var></Var></VarXML>";
        require(file.write(xml) == xml.size(), "temporary settings write failed");
    };
    writeLegacy("<Var name=\"Ball-ground slip\" type=\"double\">1</Var>"
                "<Var name=\"Ball linear damping\" type=\"double\">0.004</Var>"
                "<Var name=\"Ball angular damping\" type=\"double\">0.004</Var>"
                "<Var name=\"Models\" type=\"list\"><Var name=\"Straight-two-phase rolling acceleration\" type=\"double\">-0.7</Var></Var>");
    {
        ConfigWidget migrated(true, path);
        close(migrated.BallSlip(), 0, "legacy default slip must migrate to zero");
        close(migrated.BallLinearDamp(), 0, "legacy default linear damping must migrate to zero");
        close(migrated.BallAngularDamp(), 0, "legacy default angular damping must migrate to zero");
        close(migrated.BallRollingDeceleration(), 0.35, "older settings must acquire the new rolling default");
        close(migrated.BallModelTwoPhaseAccRoll(), -0.35, "old advertised acceleration must follow actual rolling");
        migrated.v_BallRollingDeceleration->setDouble(0.42);
        migrated.v_BallSlip->setDouble(1);
        migrated.v_BallLinearDamp->setDouble(0.004);
        migrated.v_BallAngularDamp->setDouble(0.004);
    }
    {
        ConfigWidget reloaded(true, path);
        close(reloaded.BallSlip(), 1, "migration must not repeat after user selects slip");
        close(reloaded.BallLinearDamp(), 0.004, "migration must not repeat after user selects linear damping");
        close(reloaded.BallAngularDamp(), 0.004, "migration must not repeat after user selects angular damping");
        close(reloaded.BallRollingDeceleration(), 0.42, "custom rolling deceleration must persist");
        close(reloaded.BallModelTwoPhaseAccRoll(), -0.42, "reloaded model must match custom rolling deceleration");
    }
    writeLegacy("<Var name=\"Ball-ground slip\" type=\"double\">0.02</Var>"
                "<Var name=\"Ball linear damping\" type=\"double\">0.003</Var>"
                "<Var name=\"Ball angular damping\" type=\"double\">0.008</Var>");
    {
        ConfigWidget custom(true, path);
        close(custom.BallSlip(), 0.02, "custom legacy slip must survive migration");
        close(custom.BallLinearDamp(), 0.003, "custom legacy linear damping must survive migration");
        close(custom.BallAngularDamp(), 0.008, "custom legacy angular damping must survive migration");
    }
    QFile file(path);
    require(file.open(QIODevice::ReadOnly), "read-only settings fixture unavailable");
    const auto before = file.readAll();
    file.close();
    {
        ConfigWidget readOnly(false, path);
        close(readOnly.BallSlip(), 0.02, "explicit read-only settings must load custom values");
        readOnly.v_BallRollingDeceleration->setDouble(0.5);
        readOnly.queueSettingsEdited();
        close(readOnly.BallModelTwoPhaseAccRoll(), -0.5, "live model row must follow rolling edits");
        close(readOnly.simulationSettings().BallModelTwoPhaseAccRoll(), -0.5, "settings copy must use actual rolling acceleration");
    }
    require(file.open(QIODevice::ReadOnly) && file.readAll() == before, "read-only settings load must not rewrite the file");
    std::puts("PASS persisted settings migration runs once and preserves other custom values");
}

void checkBallSettings(SSLWorld& world, const SimulationSettings& settings)
{
    const auto* contact = world.p->findSurface(world.ball, world.ground);
    const auto& material = contact->surface;
    const int base = dContactBounce | dContactApprox1 | dContactRolling | dContactMu2;
    const int flags = settings.BallSlip() > 0 ? base | dContactSlip1 | dContactSlip2 : base;
    require(material.mode == flags && !contact->usefdir1, "free ball contact flags must survive settings edits/release");
    close(material.mu, 1, "ground sliding traction must retain its tested value");
    close(material.mu2, 1, "ground sliding traction must be isotropic");
    const double rho = settings.Gravity() > 0 ?
        1.4*settings.BallRollingDeceleration()*settings.BallRadius()/settings.Gravity() : 0;
    close(material.rho, rho, "rolling torque must produce the requested deceleration");
    close(material.rho2, rho, "rolling resistance must be isotropic");
    close(material.rhoN, 2*settings.BallFriction()*settings.BallRadius(), "spin resistance must stay independent of rolling");
    close(material.bounce, settings.BallBounce(), "ball bounce must follow settings");
    close(material.bounce_vel, settings.BallBounceVel(), "bounce threshold must follow settings");
    close(material.slip1, settings.BallSlip(), "ball slip1 must follow settings");
    close(material.slip2, settings.BallSlip(), "ball slip2 must follow settings");
    close(dBodyGetLinearDamping(world.ball->body), settings.BallLinearDamp(), "linear damping must follow settings");
    close(dBodyGetAngularDamping(world.ball->body), settings.BallAngularDamp(), "angular damping must follow settings");
    dMass mass;
    dBodyGetMass(world.ball->body, &mass);
    close(mass.mass, settings.BallMass(), "ball mass must follow settings");
    dVector3 gravity;
    dWorldGetGravity(world.p->world, gravity);
    close(gravity[2], -settings.Gravity(), "gravity must follow settings at startup and live edit");
    world.sendGeomCount = 0;
    std::unique_ptr<SSL_WrapperPacket> packet(world.generatePacket());
    close(packet->geometry().models().straight_two_phase().acc_roll(), -settings.BallRollingDeceleration(),
          "published rolling acceleration must agree with actual physics settings");
}

void rollingMotion(SimulationSettings settings, double speed, unsigned substeps)
{
    settings.value_Robots_Count = 0;
    RobotsFormation formation(FORMATION_INSIDE_1, &settings);
    SSLWorld world(nullptr, &settings, &formation, &formation);
    const double radius = settings.BallRadius(), a = settings.BallRollingDeceleration();
    dBodySetPosition(world.ball->body, -2, 0, radius);
    dBodySetLinearVel(world.ball->body, speed, 0, 0);
    dBodySetAngularVel(world.ball->body, 0, speed/radius, 0);
    PhysicsStepOptions options;
    options.substeps = substeps;
    double previousEnergy = 0.7*settings.BallMass()*speed*speed;
    const int steps = int(std::ceil((speed/a+0.1)/tick));
    for (int i = 1; i <= steps; ++i) {
        world.stepPhysics(tick, options);
        const double v = dBodyGetLinearVel(world.ball->body)[0];
        const double w = dBodyGetAngularVel(world.ball->body)[1];
        const double expected = std::max(0.0, speed-a*i*tick);
        require(std::abs(v-expected) < 1e-8, "pure rolling speed must follow the requested deceleration");
        require(std::abs(v-radius*w) < 1e-8, "rolling must satisfy the no-slip constraint");
        const double energy = 0.5*settings.BallMass()*(v*v+0.4*radius*radius*w*w);
        require(energy <= previousEnergy+1e-12, "rolling resistance must not add kinetic energy");
        previousEnergy = energy;
    }
    const double distance = dBodyGetPosition(world.ball->body)[0]+2;
    const double idealDistance = speed*speed/(2*a);
    require(std::abs(distance-idealDistance) < speed*tick/(2*substeps)+1e-5,
            "rolling range must agree with energy balance within Euler integration error");
    std::printf("PASS rolling v0=%.1f, a=%.2f, substeps=%u, range=%.6f m; energy and no-slip constraints\n",
                speed, a, substeps, distance);
}

void materialsAndInertia(SimulationSettings settings)
{
    settings.value_Robots_Count = 1;
    RobotsFormation formation(FORMATION_INSIDE_1, &settings);
    SSLWorld world(nullptr, &settings, &formation, &formation);
    require(!PhysicsStepOptions().legacyBallFriction, "production must use contact resistance");
    checkBallSettings(world, settings);
    require(world.ball->tag == -1, "new world must not assign a phantom ball holder");
    auto* contact = world.p->findSurface(world.ball, world.ground);
    contact->callback(contact->id1, contact->id2, contact, 1);
    checkBallSettings(world, settings);
    for (int i = 0; i < 2; ++i) {
        checkCylinder(world.robots[i]->chassis);
        checkCylinder(world.robots[i]->chassis, 1.3);
        for (auto* wheel : world.robots[i]->wheels) {
            checkCylinder(wheel->cyl);
            checkCylinder(wheel->cyl, 0.7);
        }
    }
    std::puts("PASS production chassis/wheel inertia at creation and mass updates");

    world.ball->tag = 0;
    require(contact->callback(contact->id1, contact->id2, contact, 1), "held contact callback failed");
    require(contact->usefdir1 && (contact->surface.mode & dContactFDir1), "held contact must use holder direction");
    world.ball->tag = -1;
    contact->callback(contact->id1, contact->id2, contact, 1);
    checkBallSettings(world, settings);
    world.ball->tag = MAX_ROBOT_COUNT*2;
    contact->callback(contact->id1, contact->id2, contact, 1);
    checkBallSettings(world, settings);
    std::puts("PASS free contact restored after held contact and invalid holder ignored");
}

// Exercise the real worker configuration entry point in its owning thread.
// No event-loop iterations occur here, so the timer cannot race the assertions.
void workerSettings(SimulationSettings settings)
{
    settings.value_Robots_Count = 1;
    settings.value_CommandListenPort = settings.value_SimControlListenPort = 0;
    settings.value_BlueControlListenPort = settings.value_YellowControlListenPort = 0;
    auto mailbox = std::make_shared<SnapshotMailbox>();
    SimulationWorker worker(mailbox);
    worker.start(settings);
    auto configure = [&](const SimulationSettings& next, bool rebuild) {
        SimulationSettingsEdit edit;
        edit.before = edit.after = next;
        edit.rebuild = rebuild;
        worker.configure(edit);
    };
    try {
        auto* original = worker.findChild<SSLWorld*>();
        require(original, "worker must own its physics world");
        checkBallSettings(*original, settings);
        settings.value_BallFriction = 0.08;
        settings.value_BallRollingDeceleration = 0.4;
        settings.value_BallSlip = 0.002;
        settings.value_BallBounce = 0.65;
        settings.value_BallBounceVel = 0.15;
        settings.value_BallMass = 0.05;
        settings.value_Gravity = 8.5;
        settings.value_BallLinearDamp = 0.0001;
        settings.value_BallAngularDamp = 0.0002;
        configure(settings, false);
        require(worker.findChild<SSLWorld*>() == original, "material edits must preserve the world");
        checkBallSettings(*original, settings);
        configure(settings, true);
        checkBallSettings(*worker.findChild<SSLWorld*>(), settings);
        settings.value_BallRadius = 0.025;
        configure(settings, false);
        checkBallSettings(*worker.findChild<SSLWorld*>(), settings);
        settings.value_BallSlip = settings.value_BallLinearDamp = settings.value_BallAngularDamp = 0;
        settings.value_BallFriction = 0;
        configure(settings, false);
        checkBallSettings(*worker.findChild<SSLWorld*>(), settings);
        settings.value_BallRollingDeceleration = 0;
        configure(settings, false);
        checkBallSettings(*worker.findChild<SSLWorld*>(), settings);
        std::puts("PASS startup/live/rebuilt ball settings, radius changes and zero resistance");
        auto* world = worker.findChild<SSLWorld*>();
        auto oneTick = [&]() {
            PWorld::StepStatistics statistics;
            world->stepPhysics(tick, PhysicsStepOptions(), &statistics);
            return statistics.integrationSteps;
        };
        auto settle = [&]() {
            SimulationAction park;
            park.kind = SimulationActionKind::PlaceBall;
            park.generation = mailbox->latest()->generation;
            park.x = 4; park.y = -3; park.z = settings.BallRadius();
            worker.act(park);
            for (int i = 0; i < 150; ++i) oneTick();
            require(oneTick() == 2, "placement guard fixture must first reach two settled steps");
        };
        // These edits run through the real GUI/worker action entry point.
        for (auto kind : {SimulationActionKind::PlaceRobot, SimulationActionKind::ResetRobot,
                          SimulationActionKind::PlaceBall, SimulationActionKind::Formation}) {
            settle();
            SimulationAction action;
            action.generation = mailbox->latest()->generation;
            action.kind = kind;
            action.robot = 0; action.team = 0; action.formation = FORMATION_INSIDE_1;
            action.x = -2; action.y = 0; action.z = settings.BallRadius();
            worker.act(action);
            require(oneTick() >= 5, "GUI placement/reset must restart fine settling");
        }
        settle();
        SimulatorCommand command;
        auto* robot = command.mutable_control()->add_teleport_robot();
        robot->mutable_id()->set_id(0); robot->mutable_id()->set_team(BLUE);
        robot->set_x(-2); robot->set_y(0);
        SimulatorResponse response;
        world->processSimControl(command, response);
        require(response.errors_size() == 0 && oneTick() >= 5, "modern robot teleport must restart fine settling");
        for (bool ball : {false, true}) {
            settle();
            grSim_Packet packet;
            if (ball) {
                packet.mutable_replacement()->mutable_ball()->set_x(4);
                packet.mutable_replacement()->mutable_ball()->set_y(-2);
            } else {
                auto* replacement = packet.mutable_replacement()->add_robots();
                replacement->set_id(0); replacement->set_yellowteam(false);
                replacement->set_x(-1.8); replacement->set_y(0); replacement->set_dir(0);
            }
            QUdpSocket sender;
            const auto data = packet.SerializeAsString();
            require(sender.writeDatagram(data.data(), int(data.size()), QHostAddress::LocalHost,
                                         world->commandSocket->localPort()) == qint64(data.size()),
                    "legacy placement datagram must be sent");
            world->commandSocket->waitForReadyRead(1000);
            // readyRead normally drains synchronously through the worker; drain
            // explicitly as well if the platform deferred signal delivery.
            if (world->commandSocket->hasPendingDatagrams()) world->recvActions();
            const auto* position = dBodyGetPosition(ball ? world->ball->body : world->robots[0]->chassis->body);
            require(std::abs(position[ball ? 1 : 0] - (ball ? -2 : -1.8)) < 1e-6,
                    "legacy placement must actually reach the world");
            require(oneTick() >= 5, "legacy placement must restart fine settling");
        }
        std::puts("PASS modern/legacy and GUI placement/reset restart fine integration after settled motion");
    } catch (...) { worker.stop(); throw; }
    worker.stop();
}

void rollerAndKick(SimulationSettings settings, double chipSpeed)
{
    settings.value_Robots_Count = 1;
    RobotsFormation formation(FORMATION_INSIDE_1, &settings);
    SSLWorld world(nullptr, &settings, &formation, &formation);
    auto* robot = world.robots[0];
    robot->setXY(0, 0);
    world.robots[1]->setXY(3, 3);
    dBodySetPosition(world.ball->body, -3, -3, settings.BallRadius());
    for (int i = 0; i < 60; ++i) world.stepPhysics(tick);
    dReal dx, dy, dz, x, y, z;
    robot->chassis->getBodyDirection(dx, dy, dz);
    robot->kicker->box->getBodyPosition(x, y, z);
    const double offset = robot->settings.KickerThickness/2 + settings.BallRadius() + 0.001;
    dBodySetPosition(world.ball->body, x+dx*offset, y+dy*offset, settings.BallRadius());
    dBodySetLinearVel(world.ball->body, 0, 0, 0);
    dBodySetAngularVel(world.ball->body, 0, 0, 0);
    require(robot->kicker->isTouchingBall(), "roller fixture must touch the actual kicker");
    const int freeJoints = dBodyGetNumJoints(world.ball->body);
    robot->kicker->setRoller(1);
    robot->kicker->step();
    require(dBodyGetNumJoints(world.ball->body) == freeJoints+1, "roller must create a ball hold");
    const double chassisStartX = dBodyGetPosition(robot->chassis->body)[0];
    robot->setSpeed(0.3, 0, 0);
    for (int i = 0; i < 30; ++i) world.stepPhysics(tick);
    require(dBodyGetPosition(robot->chassis->body)[0] > chassisStartX+0.05, "dribble fixture must move forward");
    require(robot->kicker->isTouchingBall(), "moving roller must retain the ball for half a second");
    require(dBodyGetNumJoints(world.ball->body) == freeJoints+1, "roller hold must remain active");
    robot->kicker->setRoller(0);
    robot->kicker->step();
    require(dBodyGetNumJoints(world.ball->body) == freeJoints, "roller off must release the ball");
    robot->kicker->setRoller(1);
    robot->kicker->step();
    require(dBodyGetNumJoints(world.ball->body) == freeJoints+1, "roller must reacquire the ball");
    robot->kicker->kick(4, chipSpeed);
    require(dBodyGetNumJoints(world.ball->body) == freeJoints, "kick must release the roller hold");
    require(robot->kicker->isKicking() == (chipSpeed > 0 ? CHIP_KICK : FLAT_KICK), "kick status must match command");
    const auto* velocity = dBodyGetLinearVel(world.ball->body);
    close(velocity[2], chipSpeed, "kicker must apply requested launch height velocity");
    require(std::hypot(velocity[0], velocity[1]) > 3.5, "kicker must launch the ball forward");
    const auto* initial = dBodyGetPosition(world.ball->body);
    const double startX = initial[0], startY = initial[1];
    for (int i = 0; i < 10; ++i) world.stepPhysics(tick);
    const auto* position = dBodyGetPosition(world.ball->body);
    require(std::hypot(position[0]-startX, position[1]-startY) > 0.3, "ball must travel away after kicking");
    if (chipSpeed > 0) require(position[2] > settings.BallRadius()+0.2, "chip must lift ball clear of ground");
    checkBallSettings(world, settings);
    std::printf("PASS production roller hold/release/reacquire and %s kick\n", chipSpeed > 0 ? "chip" : "flat");
}
} // namespace

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    ConfigWidget config(false);
    auto settings = config.simulationSettings();
    if (!dInitODE2(dInitFlagManualThreadCleanup) || !dAllocateODEDataForThread(dAllocateMaskAll)) return 2;
    bool passed = true;
    try {
        settingsMigration();
        close(settings.BallRollingDeceleration(), 0.35, "default rolling deceleration must be 0.35 m/s^2");
        close(settings.BallSlip(), 0, "default ball slip must be zero");
        close(settings.BallLinearDamp(), 0, "default linear damping must be zero");
        close(settings.BallAngularDamp(), 0, "default angular damping must be zero");
        materialsAndInertia(settings);
        for (double speed : {0.5, 1.0, 2.0}) rollingMotion(settings, speed, 5);
        rollingMotion(settings, 2, 30);
        auto alteredBall = settings;
        alteredBall.value_BallMass = 0.03;
        alteredBall.value_BallRadius = 0.03;
        alteredBall.value_Gravity = 4;
        rollingMotion(alteredBall, 2, 5);
        rollerAndKick(settings, 0);
        rollerAndKick(settings, 3);
    } catch (const std::exception& error) {
        std::fprintf(stderr, "FAIL %s\n", error.what());
        passed = false;
    }
    dCleanupODEAllDataForThread();
    dCloseODE();
    try { workerSettings(settings); }
    catch (const std::exception& error) { std::fprintf(stderr, "FAIL %s\n", error.what()); passed = false; }
    return passed ? 0 : 1;
}
