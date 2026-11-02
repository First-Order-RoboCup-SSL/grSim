#include <QApplication>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QThread>
#include <QUdpSocket>
#include <QTimer>
#include <QDir>
#include <QWidget>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <functional>
#include "mainwindow.h"
#include "ssl_simulation_control.pb.h"
#include "ssl_gc_common.pb.h"
#include "grSim_Packet.pb.h"

static std::atomic<bool> threadViolation{false};
static void captureQtMessages(QtMsgType, const QMessageLogContext&, const QString& text)
{
    if (text.contains("different thread", Qt::CaseInsensitive) ||
        text.contains("Timers cannot", Qt::CaseInsensitive) ||
        text.contains("QSocketNotifier")) threadViolation.store(true);
    std::fprintf(stderr, "%s\n", text.toLocal8Bit().constData());
}

static bool waitFor(const std::function<bool()>& predicate, int timeout = 2000, bool deliverGuiEvents = true)
{
    QElapsedTimer clock;
    clock.start();
    while (clock.elapsed() < timeout) {
        if (deliverGuiEvents) QCoreApplication::processEvents(QEventLoop::AllEvents, 5);
        if (predicate()) return true;
        QThread::msleep(1);
    }
    return predicate();
}

static bool check(bool passed, const char* description)
{
    std::printf("%s %s\n", passed ? "PASS" : "FAIL", description);
    std::fflush(stdout);
    return passed;
}


static bool checkOdeWorkerPolicy()
{
    const bool hadOverride = qEnvironmentVariableIsSet("GRSIM_ODE_THREADS");
    const QByteArray originalOverride = qgetenv("GRSIM_ODE_THREADS");
    qunsetenv("GRSIM_ODE_THREADS");
    QUdpSocket sink;
    if (!sink.bind(QHostAddress(QHostAddress::LocalHost), 0)) return false;
    QObject::connect(&sink, &QUdpSocket::readyRead, [&]() {
        while (sink.hasPendingDatagrams()) {
            QByteArray bytes(int(sink.pendingDatagramSize()), 0);
            sink.readDatagram(bytes.data(), bytes.size());
        }
    });
    ConfigWidget config(false);
    config.v_Robots_Count->setInt(6);
    config.v_CommandListenPort->setInt(0);
    config.v_SimControlListenPort->setInt(0);
    config.v_BlueControlListenPort->setInt(0);
    config.v_YellowControlListenPort->setInt(0);
    config.v_VisionMulticastAddr->setString("127.0.0.1");
    config.v_VisionMulticastPort->setInt(sink.localPort());
    auto settings = config.simulationSettings();
    const int available = QThread::idealThreadCount();
    const unsigned automatic = available >= 3 ? unsigned(qMin(3, available-1)) : 0;
    bool passed = true;
    {
        SimulationController controller(settings);
        auto verify = [&](unsigned expected, int robots, quint64 oldGeneration, const char* text) {
            return check(waitFor([&]() {
                const auto scene = controller.latest();
                return scene && scene->generation > oldGeneration && scene->tick >= 2 &&
                    scene->robots.size() == unsigned(2*robots) && scene->odeHelpers == expected &&
                    std::isfinite(scene->ballPosition[0]);
            }), text);
        };
        passed = verify(automatic, 6, 0, "large game starts with automatic island workers") && passed;
        auto rebuild = [&](int robots, unsigned expected, const char* text) {
            const quint64 generation = controller.latest()->generation;
            settings.value_Robots_Count = robots;
            controller.configure(settings, true);
            return verify(expected, robots, generation, text);
        };
        passed = rebuild(1, 0, "small game releases automatic island workers") && passed;
        qputenv("GRSIM_ODE_THREADS", "2");
        passed = rebuild(1, 2, "explicit worker override applies to small games") && passed;
        qputenv("GRSIM_ODE_THREADS", "0");
        passed = rebuild(6, 0, "explicit zero keeps large games serial") && passed;
        qputenv("GRSIM_ODE_THREADS", "invalid");
        passed = rebuild(6, 0, "invalid override retains safe serial fallback") && passed;
        qunsetenv("GRSIM_ODE_THREADS");
        passed = rebuild(6, automatic, "removing override restores automatic workers") && passed;
    }
    if (hadOverride) qputenv("GRSIM_ODE_THREADS", originalOverride);
    else qunsetenv("GRSIM_ODE_THREADS");
    passed = check(!threadViolation.load(), "worker policy rebuilds and shutdown keep Qt ownership valid") && passed;
    return passed;
}

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    qInstallMessageHandler(captureQtMessages);
    if (argc > 1 && QByteArray(argv[1]) == "--ode-worker-policy") {
        const bool passed = checkOdeWorkerPolicy();
        qInstallMessageHandler(nullptr);
        return passed ? 0 : 1;
    }
    bool passed = true;
    LagMonitor monitor;
    const double step = 1.0/60.0;
    bool hysteresis = !monitor.update(0.1, step, 0) &&
        !monitor.update(0.1, step, 499999999) && monitor.update(0.1, step, 500000000) &&
        monitor.update(0, step, 600000000) && monitor.update(0, step, 1099999999) &&
        !monitor.update(0, step, 1100000000);
    passed = check(hysteresis, "lag warning requires 500 ms to appear and clear") && passed;

    QUdpSocket vision, reserve, reserveLegacy;
    if (!vision.bind(QHostAddress(QHostAddress::LocalHost), 0) ||
        !reserve.bind(QHostAddress(QHostAddress::LocalHost), 0) ||
        !reserveLegacy.bind(QHostAddress(QHostAddress::LocalHost), 0)) return 2;
    const quint16 simPort = reserve.localPort();
    const quint16 legacyPort = reserveLegacy.localPort();
    reserve.close();
    reserveLegacy.close();
    QObject::connect(&vision, &QUdpSocket::readyRead, [&]() {
        while (vision.hasPendingDatagrams()) {
            QByteArray bytes(vision.pendingDatagramSize(), 0);
            vision.readDatagram(bytes.data(), bytes.size());
        }
    });
    auto* config = new ConfigWidget(false);
    config->v_Robots_Count->setInt(1);
    config->v_CommandListenPort->setInt(legacyPort);
    config->v_SimControlListenPort->setInt(simPort);
    config->v_BlueControlListenPort->setInt(0);
    config->v_YellowControlListenPort->setInt(0);
    config->v_VisionMulticastAddr->setString("127.0.0.1");
    config->v_VisionMulticastPort->setInt(vision.localPort());
    auto* window = new MainWindow(nullptr, config);
    window->hide();
    window->setIsGlEnabled(false);
    auto* view = window->findChild<GLWidget*>();
    auto* controller = view->simulation;
    auto scene = controller->latest();
    passed = check(scene && scene->robots.size() == 2, "initial snapshot contains both teams") && passed;
    if (!scene) return 2;
    const SceneSnapshotPtr retained = scene;
    const quint64 generation = scene->generation;
    SimulationAction action;
    action.generation = generation;
    action.kind = SimulationActionKind::PlaceRobot;
    action.robot = 0; action.x = 1; action.y = 0.2; action.direction = 45; action.reset = true;
    controller->submit(action);
    passed = check(waitFor([&]() {
        const auto next = controller->latest();
        return std::abs(next->robots[0].x-1) < 0.01 && std::abs(next->robots[0].y-0.2) < 0.01;
    }), "queued robot placement") && passed;
    passed = check(std::abs(retained->robots[0].x-1) > 0.1, "published snapshots remain immutable") && passed;
    action.kind = SimulationActionKind::SetRobotEnabled;
    action.enabled = false;
    controller->submit(action);
    passed = check(waitFor([&]() { return !controller->latest()->robots[0].on; }), "queued robot on/off") && passed;
    action.enabled = true;
    controller->submit(action);
    action.kind = SimulationActionKind::PlaceBall;
    action.x = 0.4; action.y = -0.3; action.z = 0.3;
    controller->submit(action);
    passed = check(waitFor([&]() { return std::abs(controller->latest()->ballPosition[0]-0.4) < 0.01; }), "queued ball teleport") && passed;
    action.kind = SimulationActionKind::SetBallVelocity;
    action.x = 2; action.y = 0; action.z = 0;
    controller->submit(action);
    passed = check(waitFor([&]() { return controller->latest()->ballPosition[0] > 0.45; }), "queued manual ball kick") && passed;
    action.kind = SimulationActionKind::Formation;
    action.team = 0; action.formation = 1;
    controller->submit(action);
    passed = check(waitFor([&]() { return std::abs(controller->latest()->robots[0].x + 1.5) < 0.01; }), "queued formation change") && passed;

    QUdpSocket control;
    control.bind(QHostAddress(QHostAddress::LocalHost), 0);
    SimulatorCommand command;
    auto* specification = command.mutable_config()->add_robot_specs();
    specification->mutable_id()->set_id(0);
    specification->mutable_id()->set_team(BLUE);
    specification->set_radius(0.10);
    const std::string data = command.SerializeAsString();
    control.writeDatagram(data.data(), int(data.size()), QHostAddress::LocalHost, simPort);
    passed = check(waitFor([&]() {
        const auto next = controller->latest();
        return next->generation > generation && std::abs(config->blueSettings.RobotRadius-0.10) < 1e-6;
    }), "remote robot configuration rebuilds in the worker and updates the GUI") && passed;
    scene = controller->latest();
    passed = check(std::abs(scene->settings.blueSettings.RobotRadius-0.10) < 1e-6 &&
                   std::abs(scene->settings.yellowSettings.RobotRadius-0.10) > 1e-3,
                   "blue and yellow settings remain independent") && passed;
    const auto settingsGeneration = scene->generation;
    config->v_noise->setBool(true);
    config->v_noise->wasEdited(config->v_noise);
    passed = check(waitFor([&]() { return controller->latest()->settings.noise(); }),
                   "widget edits copy runtime settings to the worker") && passed;
    passed = check(controller->latest()->generation == settingsGeneration,
                   "runtime settings changes preserve the world") && passed;
    config->v_noise->setBool(false);
    config->v_noise->wasEdited(config->v_noise);
    action.kind = SimulationActionKind::PlaceBall;
    action.generation = generation;
    action.x = 999; action.y = 999;
    controller->submit(action);
    const auto tick = scene->tick;
    waitFor([&]() { return controller->latest()->tick > tick + 2; });
    passed = check(std::abs(controller->latest()->ballPosition[0]) < 10, "stale actions are rejected after a reset") && passed;

    // Commands arriving through the legacy protocol must also renew the
    // one-second command timeout, even long after startup.
    QElapsedTimer idle;
    idle.start();
    waitFor([&]() { return idle.elapsed() > 1100; });
    grSim_Packet legacy;
    auto* commands = legacy.mutable_commands();
    commands->set_timestamp(0);
    commands->set_isteamyellow(false);
    auto* robotCommand = commands->add_robot_commands();
    robotCommand->set_id(0);
    robotCommand->set_kickspeedx(0); robotCommand->set_kickspeedz(0);
    robotCommand->set_veltangent(1); robotCommand->set_velnormal(0); robotCommand->set_velangular(0);
    robotCommand->set_spinner(false); robotCommand->set_wheelsspeed(false);
    const std::string legacyData = legacy.SerializeAsString();
    QTimer legacyCommands;
    legacyCommands.setInterval(20);
    QObject::connect(&legacyCommands, &QTimer::timeout, [&]() {
        control.writeDatagram(legacyData.data(), int(legacyData.size()), QHostAddress::LocalHost, legacyPort);
    });
    legacyCommands.start();
    passed = check(waitFor([&]() {
        const auto robot = controller->latest()->robots[0];
        return std::hypot(robot.velocity[0], robot.velocity[1]) > 0.05;
    }), "legacy commands remain active after startup timeout") && passed;
    legacyCommands.stop();
    idle.restart();
    waitFor([&]() { return idle.elapsed() > 1200; });
    passed = check(waitFor([&]() {
        const auto robot = controller->latest()->robots[0];
        return std::hypot(robot.velocity[0], robot.velocity[1]) < 0.05;
    }), "robots stop after one second without commands") && passed;

    auto* panel = window->findChild<RobotWidget*>();
    config->v_Robots_Count->setInt(6);
    window->changeRobotCount();
    waitFor([&]() { return controller->latest()->robots.size() == 12; });
    panel->teamCombo->setCurrentIndex(1);
    panel->robotCombo->setCurrentIndex(0);
    window->changeCurrentRobot();
    passed = check(panel->id == 6, "yellow robot zero initially uses the six-robot team offset") && passed;
    config->v_Robots_Count->setInt(5);
    window->changeRobotCount();
    waitFor([&]() { return controller->latest()->robots.size() == 10; }, 2000, false);
    window->updateUI();
    passed = check(panel->id == 5, "robot panel follows the selected robot after an asynchronous count reset") && passed;
    view->switchRobotOnOff();
    waitFor([&]() { return !controller->latest()->robots[5].on; }, 2000, false);
    window->updateUI();
    passed = check(panel->onOffBtn->text() == "Turn on", "robot panel button reflects on/off after the count reset") && passed;

    config->v_Robots_Count->setInt(0);
    window->changeRobotCount();
    passed = check(waitFor([&]() { return controller->latest()->robots.empty(); }), "zero-robot reset") && passed;
    window->updateUI();
    passed = check(panel->id == -1, "zero-robot reset clears the panel ID") && passed;
    config->v_Robots_Count->setInt(2);
    window->changeRobotCount();
    passed = check(waitFor([&]() { return controller->latest()->robots.size() == 4; }), "robot-count reset") && passed;
    window->updateUI();
    passed = check(panel->robotCombo->currentIndex() == 0 && view->Current_robot == 0,
                   "increasing the count from zero restores robot zero in the selector and view") && passed;
    for (int team : {0, 1}) {
        panel->teamCombo->setCurrentIndex(team);
        window->updateUI();
        const int index = team * 2;
        passed = check(panel->robotCombo->currentIndex() == 0 && panel->id == index,
                       "team switching preserves the restored robot selection") && passed;
        view->switchRobotOnOff();
        passed = check(waitFor([&]() { return !controller->latest()->robots[index].on; }),
                       "restored selection can turn the selected team robot off") && passed;
        window->updateUI();
        passed = check(panel->onOffBtn->text() == "Turn on",
                       "restored selection updates the robot panel button") && passed;
        const double x = team ? -1.2 : 1.2;
        panel->getPoseWidget->x->setText(QString::number(x));
        panel->getPoseWidget->y->setText("0.4");
        panel->getPoseWidget->a->setText("30");
        window->setCurrentRobotPosition();
        passed = check(waitFor([&]() {
            const auto robot = controller->latest()->robots[index];
            return std::abs(robot.x-x) < 0.01 && std::abs(robot.y-0.4) < 0.01 &&
                std::abs(robot.direction-30) < 1;
        }), "restored selection can set the selected robot's pose") && passed;
        view->resetCurrentRobot();
        passed = check(waitFor([&]() {
            const auto robot = controller->latest()->robots[index];
            return std::abs(std::remainder(robot.direction-(team ? 180 : 0), 360)) < 1;
        }), "restored selection can reset the selected robot") && passed;
    }
    config->v_Robots_Count->setInt(1);
    window->changeRobotCount();
    waitFor([&]() { return controller->latest()->robots.size() == 2; });
    config->v_DeltaTime->setDouble(1e-6);
    window->changePhysicsTimer();
    passed = check(waitFor([&]() { return controller->latest()->lagging; }, 3000), "CPU overload produces a lag warning") && passed;
    waitFor([&]() { return !view->findChild<QWidget*>("simulationLagWarning")->isHidden(); });
    passed = check(!view->findChild<QWidget*>("simulationLagWarning")->isHidden(), "corner warning follows worker lag state") && passed;
    auto* warning = view->findChild<QWidget*>("simulationLagWarning");
    passed = check(warning->y() == 8 && warning->x() + warning->width() == view->width() - 8,
                   "lag warning stays eight pixels from the top-right corner") && passed;
    const QString artifacts = QString::fromLocal8Bit(qgetenv("GRSIM_TEST_ARTIFACT_DIR"));
    if (!artifacts.isEmpty()) { QDir().mkpath(artifacts); warning->grab().save(artifacts + "/lag-warning.png"); }
    scene = controller->latest();
    const double pending = scene->pendingTime;
    const double before = scene->simulationTime;
    const quint64 overloadedGeneration = scene->generation;
    config->v_DeltaTime->setDouble(step);
    window->changePhysicsTimer();
    passed = check(waitFor([&]() { return !controller->latest()->lagging && controller->latest()->pendingTime < 0.05; }, 3000),
                   "warning clears after backlog recovery") && passed;
    scene = controller->latest();
    passed = check(scene->generation == overloadedGeneration && scene->simulationTime-before >= pending-0.05,
                   "overload recovery retains pending simulation time") && passed;
    waitFor([&]() { return view->findChild<QWidget*>("simulationLagWarning")->isHidden(); });
    passed = check(view->findChild<QWidget*>("simulationLagWarning")->isHidden(), "corner warning hides after recovery") && passed;

    for (int i = 0; i < 3; ++i) {
        const auto oldGeneration = controller->latest()->generation;
        window->restartSimulator();
        passed = check(waitFor([&]() { return controller->latest()->generation > oldGeneration; }), "repeat world reset") && passed;
    }
    delete window;
    passed = check(!threadViolation.load(), "shutdown and socket/timer ownership produce no Qt thread warnings") && passed;
    qInstallMessageHandler(nullptr);
    return passed ? 0 : 1;
}
