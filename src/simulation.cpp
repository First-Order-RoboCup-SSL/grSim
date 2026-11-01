#include "simulation.h"
#include "sslworld.h"
#include "logger.h"
#include <QMutex>
#include <QMutexLocker>
#include <QUdpSocket>
#include <cmath>
#include <QtMath>

namespace {
QMutex odeMutex;
int odeUsers = 0;

int physicsInterval(double dt)
{
    return qMax(1, qFloor(qMin(4.0, dt * 250.0)));
}

bool requiresRebuild(const SimulationSettings& a, const SimulationSettings& b)
{
    return a.Robots_Count() != b.Robots_Count() || a.BallRadius() != b.BallRadius() ||
        a.ColorRobotBlue() != b.ColorRobotBlue() || a.ColorRobotYellow() != b.ColorRobotYellow() ||
        !(a.blueSettings == b.blueSettings) || !(a.yellowSettings == b.yellowSettings) ||
        a.Field_Line_Width() != b.Field_Line_Width() ||
        a.Field_Length() != b.Field_Length() ||
        a.Field_Width() != b.Field_Width() ||
        a.Field_Rad() != b.Field_Rad() ||
        a.Field_Free_Kick() != b.Field_Free_Kick() ||
        a.Field_Penalty_Width() != b.Field_Penalty_Width() ||
        a.Field_Penalty_Depth() != b.Field_Penalty_Depth() ||
        a.Field_Penalty_Point() != b.Field_Penalty_Point() ||
        a.Field_Margin_Touch_Line() != b.Field_Margin_Touch_Line() ||
        a.Field_Margin_Goal_Line() != b.Field_Margin_Goal_Line() ||
        a.Field_Referee_Margin() != b.Field_Referee_Margin() ||
        a.Wall_Thickness() != b.Wall_Thickness() ||
        a.Goal_Thickness() != b.Goal_Thickness() ||
        a.Goal_Depth() != b.Goal_Depth() ||
        a.Goal_Width() != b.Goal_Width() ||
        a.Goal_Height() != b.Goal_Height();
}

}

void SimulationSettingsEdit::applyRobotChanges(SimulationSettings& settings) const
{
    if (before.BlueTeam() != after.BlueTeam()) settings.blueSettings = after.blueSettings;
    else settings.blueSettings.applyChanges(before.blueSettings, after.blueSettings);
    if (before.YellowTeam() != after.YellowTeam()) settings.yellowSettings = after.yellowSettings;
    else settings.yellowSettings.applyChanges(before.yellowSettings, after.yellowSettings);
}

bool LagMonitor::update(double pending, double step, qint64 now)
{
    const bool behind = pending - step >= 0.050;
    if (behind == visible) transition = -1;
    else if (transition < 0) transition = now;
    else if (now - transition >= 500000000) {
        visible = behind;
        transition = -1;
    }
    return visible;
}

void LagMonitor::reset() { visible = false; transition = -1; }

SimulationWorker::SimulationWorker(std::shared_ptr<SnapshotMailbox> value)
    : mailbox(std::move(value)) {}

void SimulationWorker::start(SimulationSettings value)
{
    Q_ASSERT(QThread::currentThread() == QObject::thread());
    {
        QMutexLocker lock(&odeMutex);
        if (odeUsers++ == 0) dInitODE2(dInitFlagManualThreadCleanup);
    }
    dAllocateODEDataForThread(dAllocateMaskAll);
    initialized = true;
    settings = value;
    if (!std::isfinite(settings.DeltaTime()) || settings.DeltaTime() <= 0)
        settings.value_DeltaTime = 1.0 / 60.0;
    settings.value_Robots_Count = qBound(0, settings.Robots_Count(), int(MAX_ROBOT_COUNT));
    settings.value_sendGeometryEvery = qMax(4, settings.sendGeometryEvery());
    connectSockets();
    rebuildWorld();
    timer = new QTimer(this);
    timer->setTimerType(Qt::PreciseTimer);
    connect(timer, &QTimer::timeout, this, &SimulationWorker::tick);
    timer->start(physicsInterval(settings.DeltaTime()));
}

void SimulationWorker::connectSockets(const SimulationSettings* previous)
{
    auto bind = [this](QUdpSocket*& socket, int port, int oldPort,
                       const char* name, void (SSLWorld::*receive)()) {
        if (!socket) {
            socket = new QUdpSocket(this);
            connect(socket, &QUdpSocket::readyRead, this, [this, receive]() {
                if (!world) return;
                (world->*receive)();
                if (world->restartRequired) {
                    rebuildWorld();
                    emit settingsChanged(settings, appliedRevision);
                }
            });
        } else if (port == oldPort) return;
        socket->close();
        if (!socket->bind(QHostAddress::Any, quint16(port)))
            logStatus(QString("Could not bind %1 port %2: %3").arg(name).arg(port)
                      .arg(socket->errorString()), QColor("red"));
    };
    bind(commandSocket, settings.CommandListenPort(), previous ? previous->CommandListenPort() : -1,
         "commands", &SSLWorld::recvActions);
    bind(simControlSocket, settings.SimControlListenPort(), previous ? previous->SimControlListenPort() : -1,
         "simulation control", &SSLWorld::simControlSocketReady);
    bind(blueControlSocket, settings.BlueControlListenPort(), previous ? previous->BlueControlListenPort() : -1,
         "blue control", &SSLWorld::blueControlSocketReady);
    bind(yellowControlSocket, settings.YellowControlListenPort(), previous ? previous->YellowControlListenPort() : -1,
         "yellow control", &SSLWorld::yellowControlSocketReady);
    if (!blueStatusSocket) blueStatusSocket = new QUdpSocket(this);
    if (!yellowStatusSocket) yellowStatusSocket = new QUdpSocket(this);
    if (!visionServer) visionServer = new RoboCupSSLServer(this);
    visionServer->change_address(settings.VisionMulticastAddr());
    visionServer->change_port(settings.VisionMulticastPort());
}

void SimulationWorker::rebuildWorld()
{
    delete world;
    RobotsFormation form(FORMATION_INSIDE_1, &settings);
    world = new SSLWorld(this, &settings, &form, &form);
    world->visionServer = visionServer;
    world->commandSocket = commandSocket;
    world->simControlSocket = simControlSocket;
    world->blueControlSocket = blueControlSocket;
    world->yellowControlSocket = yellowControlSocket;
    world->blueStatusSocket = blueStatusSocket;
    world->yellowStatusSocket = yellowStatusSocket;
    ++generation;
    accumulator = 0;
    lastTick = 0;
    stepsThisSecond = 0;
    physicsHz = 0;
    lag.reset();
    clock.start();
    rateClock.start();
    publish();
}

void SimulationWorker::configure(SimulationSettingsEdit edit)
{
    if (!world) return;
    appliedRevision = edit.revision;
    SimulationSettings value = edit.after;
    if (!std::isfinite(value.DeltaTime()) || value.DeltaTime() <= 0 ||
        value.Robots_Count() < 0 || value.Robots_Count() > MAX_ROBOT_COUNT) {
        logStatus("Invalid simulation timestep or robot count", QColor("red"));
        emit settingsChanged(settings, appliedRevision);
        return;
    }
    value.value_sendGeometryEvery = qMax(4, value.sendGeometryEvery());
    const SimulationSettings previous = settings;
    // Only robot fields can be edited remotely. Keep their current worker values
    // unless this GUI edit changed the field or explicitly selected a preset.
    value.blueSettings = settings.blueSettings;
    value.yellowSettings = settings.yellowSettings;
    edit.applyRobotChanges(value);
    settings = value;
    connectSockets(&previous);
    if (edit.rebuild || requiresRebuild(previous, settings)) rebuildWorld();
    else {
        if (previous.Gravity() != settings.Gravity())
            dWorldSetGravity(world->p->world, 0, 0, -settings.Gravity());
        if (previous.BallMass() != settings.BallMass()) world->ball->setMass(settings.BallMass());
        if (previous.BallLinearDamp() != settings.BallLinearDamp() || previous.BallAngularDamp() != settings.BallAngularDamp()) {
            dBodySetLinearDampingThreshold(world->ball->body, 0.001);
            dBodySetLinearDamping(world->ball->body, settings.BallLinearDamp());
            dBodySetAngularDampingThreshold(world->ball->body, 0.001);
            dBodySetAngularDamping(world->ball->body, settings.BallAngularDamp());
        }
        if (previous.BallFriction() != settings.BallFriction() ||
            previous.BallSlip() != settings.BallSlip() || previous.BallBounce() != settings.BallBounce() ||
            previous.BallBounceVel() != settings.BallBounceVel()) {
            auto* surface = world->p->findSurface(world->ball, world->ground);
            surface->surface.mode = dContactBounce | dContactApprox1 | dContactSlip1 | dContactSlip2;
            surface->surface.mu = fric(settings.BallFriction());
            surface->surface.bounce = settings.BallBounce();
            surface->surface.bounce_vel = settings.BallBounceVel();
            surface->surface.slip1 = surface->surface.slip2 = settings.BallSlip();
        }
        publish();
    }
    if (timer) timer->setInterval(physicsInterval(settings.DeltaTime()));
    emit settingsChanged(settings, appliedRevision);
}

void SimulationWorker::expireCommands()
{
    auto expire = [this](QElapsedTimer& elapsed, int team) {
        if (elapsed.nsecsElapsed() <= 1000000000) return;
        for (int i = 0; i < settings.Robots_Count(); ++i)
            world->robots[world->robotIndex(i, team)]->resetSpeeds();
    };
    expire(world->elapsedLastPackageBlue, 0);
    expire(world->elapsedLastPackageYellow, 1);
}

void SimulationWorker::tick()
{
    if (!world) return;
    const qint64 now = clock.nsecsElapsed();
    accumulator += (now - lastTick) / 1e9;
    lastTick = now;
    expireCommands();
    const double step = settings.DeltaTime();
    QElapsedTimer budget;
    budget.start();
    int steps = 0;
    while (accumulator >= step && steps < 4 &&
           (steps == 0 || budget.nsecsElapsed() < 4000000)) {
        world->stepSimulation(step);
        accumulator -= step;
        ++steps;
        ++stepsThisSecond;
    }
    const qint64 rate = rateClock.nsecsElapsed();
    if (rate >= 1000000000) {
        physicsHz = stepsThisSecond / (rate / 1e9);
        stepsThisSecond = 0;
        rateClock.restart();
    }
    if (steps) publish();
    const int interval = accumulator >= step ? 0 : physicsInterval(step);
    if (timer->interval() != interval) timer->setInterval(interval);
}

void SimulationWorker::publish()
{
    auto snapshot = world->snapshot(generation);
    snapshot->physicsHz = physicsHz;
    snapshot->pendingTime = accumulator + (clock.nsecsElapsed() - lastTick) / 1e9;
    snapshot->lagging = lag.update(snapshot->pendingTime, settings.DeltaTime(), clock.nsecsElapsed());
    mailbox->publish(std::move(snapshot));
}

void SimulationWorker::act(SimulationAction action)
{
    if (!world || action.generation != generation) return;
    const int count = settings.Robots_Count();
    Robot* robot = action.robot >= 0 && action.robot < count * 2 ? world->robots[action.robot] : nullptr;
    auto releaseBall = [this]() {
        for (int i = 0; i < settings.Robots_Count() * 2; ++i) world->robots[i]->kicker->unholdBall();
    };
    switch (action.kind) {
    case SimulationActionKind::ResetRobot: if (robot) robot->resetRobot(); break;
    case SimulationActionKind::SetRobotEnabled: if (robot) robot->on = action.enabled; break;
    case SimulationActionKind::PlaceRobot:
        if (robot) {
            robot->setXY(action.x, action.y);
            if (action.reset) robot->resetRobot();
            if (action.reset) robot->setDir(action.direction);
        }
        break;
    case SimulationActionKind::PlaceBall:
        releaseBall();
        world->ball->setBodyPosition(action.x, action.y, action.z);
        dBodySetForce(world->ball->body, 0, 0, 0);
        dBodySetTorque(world->ball->body, 0, 0, 0);
        dBodySetLinearVel(world->ball->body, 0, 0, 0);
        dBodySetAngularVel(world->ball->body, 0, 0, 0);
        break;
    case SimulationActionKind::KickBall: {
        const auto* position = dBodyGetPosition(world->ball->body);
        double x = action.x - position[0], y = action.y - position[1];
        const double length = std::hypot(x, y);
        if (length < 1e-9) break;
        x *= action.direction / length; y *= action.direction / length;
        dBodySetLinearVel(world->ball->body, x, y, action.z);
        dBodySetAngularVel(world->ball->body, -y / settings.BallRadius(), x / settings.BallRadius(), action.z);
        break;
    }
    case SimulationActionKind::WheelDelta:
        if (robot) {
            (action.robot < count ? world->elapsedLastPackageBlue : world->elapsedLastPackageYellow).restart();
            for (int i = 0; i < 4; ++i) robot->incSpeed(i, action.wheels[i]);
        }
        break;
    case SimulationActionKind::ResetSpeeds: if (robot) robot->resetSpeeds(); break;
    case SimulationActionKind::BallForce: dBodyAddForce(world->ball->body, action.x, action.y, action.z); break;
    case SimulationActionKind::StopBall:
        dBodySetLinearVel(world->ball->body, 0, 0, 0);
        dBodySetAngularVel(world->ball->body, 0, 0, 0);
        break;
    case SimulationActionKind::SetBallVelocity:
        dBodySetLinearVel(world->ball->body, action.x, action.y, action.z);
        dBodySetAngularVel(world->ball->body, -action.y / settings.BallRadius(), action.x / settings.BallRadius(), 0);
        break;
    case SimulationActionKind::RobotKick: if (robot) robot->kicker->kick(action.x, action.z); break;
    case SimulationActionKind::ToggleRoller: if (robot) robot->kicker->toggleRoller(); break;
    case SimulationActionKind::Formation: {
        if (action.team < 0 || action.team > 1 || action.formation < 0 || action.formation > 3) break;
        RobotsFormation formation(static_cast<E_FORMATION>(action.formation), &settings);
        formation.resetRobots(world->robots, action.team);
        break;
    }
    case SimulationActionKind::EnableTeam:
        if (action.team >= 0 && action.team < 2)
            for (int i = 0; i < count; ++i) world->robots[world->robotIndex(i, action.team)]->on = action.enabled;
        break;
    }
    publish();
}

void SimulationWorker::stop()
{
    if (timer) timer->stop();
    delete world; world = nullptr;
    delete visionServer; visionServer = nullptr;
    delete commandSocket; commandSocket = nullptr;
    delete simControlSocket; simControlSocket = nullptr;
    delete blueControlSocket; blueControlSocket = nullptr;
    delete yellowControlSocket; yellowControlSocket = nullptr;
    delete blueStatusSocket; blueStatusSocket = nullptr;
    delete yellowStatusSocket; yellowStatusSocket = nullptr;
    if (initialized) {
        dCleanupODEAllDataForThread();
        QMutexLocker lock(&odeMutex);
        if (--odeUsers == 0) dCloseODE();
        initialized = false;
    }
}

SimulationController::SimulationController(const SimulationSettings& settings, QObject* parent)
    : QObject(parent), mailbox(std::make_shared<SnapshotMailbox>()),
      worker(new SimulationWorker(mailbox)), guiSettings(settings)
{
    qRegisterMetaType<SimulationSettings>();
    qRegisterMetaType<SimulationSettingsEdit>();
    qRegisterMetaType<SimulationAction>();
    worker->moveToThread(&thread);
    connect(&thread, &QThread::finished, worker, &QObject::deleteLater);
    connect(this, &SimulationController::configureRequested, worker, &SimulationWorker::configure);
    connect(this, &SimulationController::actionRequested, worker, &SimulationWorker::act);
    connect(worker, &SimulationWorker::settingsChanged, this, &SimulationController::receiveSettings);
    thread.setObjectName("grSim physics");
    thread.start();
    QMetaObject::invokeMethod(worker, "start", Qt::BlockingQueuedConnection, Q_ARG(SimulationSettings, settings));
}

SimulationController::~SimulationController() { stop(); }
void SimulationController::configure(const SimulationSettings& settings, bool rebuild)
{
    if (!thread.isRunning()) return;
    SimulationSettingsEdit edit;
    edit.before = guiSettings;
    edit.after = settings;
    edit.revision = ++nextRevision;
    edit.rebuild = rebuild;
    pendingEdits.push_back(edit);
    guiSettings = settings;
    emit configureRequested(edit);
}

void SimulationController::receiveSettings(SimulationSettings settings, quint64 appliedRevision)
{
    while (!pendingEdits.empty() && pendingEdits.front().revision <= appliedRevision)
        pendingEdits.pop_front();
    // Rebase still-pending GUI edits before notifying widgets. Otherwise an older
    // worker notification could erase a newer edit before the worker applies it.
    for (const auto& edit : pendingEdits) edit.applyRobotChanges(settings);
    guiSettings.blueSettings = settings.blueSettings;
    guiSettings.yellowSettings = settings.yellowSettings;
    emit remoteSettingsChanged(guiSettings);
}
void SimulationController::submit(const SimulationAction& action)
{
    if (thread.isRunning()) emit actionRequested(action);
}
void SimulationController::stop()
{
    if (!thread.isRunning()) return;
    QMetaObject::invokeMethod(worker, "stop", Qt::BlockingQueuedConnection);
    thread.quit();
    thread.wait();
}
