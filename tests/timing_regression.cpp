#include <QApplication>
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QJsonDocument>
#include <QJsonObject>
#include <QNetworkDatagram>
#include <QProcess>
#include <QThread>
#include <QTimer>
#include <QUdpSocket>
#include <cmath>
#include <cstdio>
#include "mainwindow.h"
#include "framepacer.h"
#include "timingstats.h"
#include <limits>
#include "ssl_vision_wrapper.pb.h"
#include "ssl_simulation_robot_control.pb.h"
#include "ssl_simulation_robot_feedback.pb.h"

// This process has its own event loop: blocking the GUI must not also block
// the observer and accidentally hide a pause in network publication.
static int observe(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    const quint16 controlPort = QString::fromLocal8Bit(argv[2]).toUShort();
    QUdpSocket vision, feedback;
    if (!vision.bind(QHostAddress(QHostAddress::LocalHost), 0) ||
        !feedback.bind(QHostAddress(QHostAddress::LocalHost), 0)) return 2;
    std::printf("READY %u\n", vision.localPort());
    std::fflush(stdout);
    QElapsedTimer clock;
    clock.start();
    qint64 firstArrival = -1, lastArrival = 0, longestGap = 0;
    qint64 lastHeartbeat = 0, longestObserverPause = 0;
    QTimer heartbeat;
    heartbeat.setTimerType(Qt::PreciseTimer);
    heartbeat.setInterval(5);
    QObject::connect(&heartbeat, &QTimer::timeout, [&]() {
        const qint64 now = clock.nsecsElapsed();
        longestObserverPause = qMax(longestObserverPause, now-lastHeartbeat);
        lastHeartbeat = now;
    });
    heartbeat.start();
    qint64 sent = -1, feedbackLatency = 0;
    int frames = 0, responses = 0, sentCommands = 0;
    quint32 previousFrame = 0;
    double firstTime = 0, lastTime = 0;
    bool fixedSteps = true;
    QObject::connect(&vision, &QUdpSocket::readyRead, [&]() {
        while (vision.hasPendingDatagrams()) {
            QByteArray data(vision.pendingDatagramSize(), 0);
            vision.readDatagram(data.data(), data.size());
            SSL_WrapperPacket packet;
            if (!packet.ParseFromArray(data.data(), data.size()) || !packet.has_detection() ||
                packet.detection().camera_id() != 0) continue;
            const auto& detection = packet.detection();
            const qint64 now = clock.nsecsElapsed();
            if (frames) {
                longestGap = qMax(longestGap, now - lastArrival);
                fixedSteps = fixedSteps && detection.frame_number() == previousFrame + 1 &&
                    std::abs(detection.t_capture() - lastTime - 1.0 / 60.0) < 1e-9;
            } else {
                firstArrival = now;
                firstTime = detection.t_capture();
            }
            previousFrame = detection.frame_number();
            lastArrival = now;
            lastTime = detection.t_capture();
            ++frames;
        }
    });
    QObject::connect(&feedback, &QUdpSocket::readyRead, [&]() {
        while (feedback.hasPendingDatagrams()) {
            QByteArray data(feedback.pendingDatagramSize(), 0);
            feedback.readDatagram(data.data(), data.size());
            RobotControlResponse response;
            if (!response.ParseFromArray(data.data(), data.size()) || response.feedback_size() == 0) continue;
            if (sent >= 0) feedbackLatency = qMax(feedbackLatency, clock.nsecsElapsed() - sent);
            sent = -1;
            ++responses;
        }
    });
    QTimer commands;
    commands.setTimerType(Qt::PreciseTimer);
    commands.setInterval(20);
    QObject::connect(&commands, &QTimer::timeout, [&]() {
        if (sent >= 0) return;
        RobotControl command;
        auto* robot = command.add_robot_commands();
        robot->set_id(0);
        auto* velocity = robot->mutable_move_command()->mutable_local_velocity();
        velocity->set_forward(0); velocity->set_left(0); velocity->set_angular(0);
        const std::string data = command.SerializeAsString();
        sent = clock.nsecsElapsed();
        ++sentCommands;
        feedback.writeDatagram(data.data(), int(data.size()), QHostAddress::LocalHost, controlPort);
    });
    commands.start();
    // Leave time to receive the last response, then require no commands outstanding.
    QTimer::singleShot(1300, Qt::PreciseTimer, &commands, &QTimer::stop);
    QTimer::singleShot(1400, Qt::PreciseTimer, &app, &QCoreApplication::quit);
    app.exec();
    const qint64 finalVisionAge = clock.nsecsElapsed() - lastArrival;
    longestGap = qMax(longestGap, finalVisionAge);
    const int unanswered = sentCommands - responses;
    const double elapsed = firstArrival >= 0 ? (lastArrival-firstArrival)/1e9 : 0;
    const double simulated = lastTime-firstTime;
    // A shared native Mac runner can pause the observer for >100 ms. Keep the
    // strict Linux/default gate and give native CI an explicit bounded budget;
    // both silence and missing-feedback negative controls still run unchanged.
    const int requestedBudget = qEnvironmentVariableIntValue("GRSIM_TEST_TIMING_BUDGET_MS");
    const int budgetMs = requestedBudget > 0 ? qBound(75, requestedBudget, 250) : 75;
    const qint64 latencyBudget = qint64(budgetMs)*1000000;
    const bool passed = frames > 50 && responses > 20 && fixedSteps &&
        longestGap < latencyBudget && feedbackLatency < latencyBudget && unanswered == 0 && sent < 0 &&
        std::abs(simulated-elapsed) < std::max(0.08, budgetMs/1000.0);
    QJsonObject result;
    result["passed"] = passed;
    result["frames"] = frames;
    result["responses"] = responses;
    result["max_gap_ms"] = longestGap/1e6;
    result["max_feedback_ms"] = feedbackLatency/1e6;
    result["max_observer_pause_ms"] = longestObserverPause/1e6;
    result["latency_budget_ms"] = budgetMs;
    result["final_vision_age_ms"] = finalVisionAge/1e6;
    result["unanswered_commands"] = unanswered;
    result["simulation_s"] = simulated;
    result["wall_s"] = elapsed;
    const QByteArray json = QJsonDocument(result).toJson(QJsonDocument::Compact);
    std::printf("%s\n", json.constData());
    return passed ? 0 : 1;
}

// Check vision silence and missing feedback independently so either regression
// fails even when the other channel remains healthy throughout the observation.
static int checkObserver(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    const bool freezeVision = argc > 2 && QByteArray(argv[2]) == "vision";
    QUdpSocket control, vision;
    if (!control.bind(QHostAddress(QHostAddress::LocalHost), 0)) return 2;
    QProcess observer;
    observer.start(QCoreApplication::applicationFilePath(), {"--observe", QString::number(control.localPort())});
    if (!observer.waitForStarted(5000) || !observer.waitForReadyRead(5000)) return 2;
    const QByteArray ready = observer.readLine().trimmed();
    if (!ready.startsWith("READY ")) return 2;
    const quint16 port = ready.mid(6).toUShort();
    QElapsedTimer clock;
    clock.start();
    QObject::connect(&control, &QUdpSocket::readyRead, [&]() {
        while (control.hasPendingDatagrams()) {
            const auto datagram = control.receiveDatagram();
            if (!freezeVision && clock.elapsed() >= 900) continue;
            RobotControlResponse response;
            response.add_feedback()->set_id(0);
            const std::string data = response.SerializeAsString();
            control.writeDatagram(data.data(), int(data.size()), datagram.senderAddress(), datagram.senderPort());
        }
    });
    int frame = 0;
    QTimer packets;
    packets.setTimerType(Qt::PreciseTimer);
    packets.setInterval(17);
    QObject::connect(&packets, &QTimer::timeout, [&]() {
        if (freezeVision && clock.elapsed() >= 900) return;
        SSL_WrapperPacket packet;
        auto* detection = packet.mutable_detection();
        detection->set_camera_id(0);
        detection->set_frame_number(frame);
        detection->set_t_capture(frame++ / 60.0);
        detection->set_t_sent(detection->t_capture());
        const std::string data = packet.SerializeAsString();
        vision.writeDatagram(data.data(), int(data.size()), QHostAddress::LocalHost, port);
    });
    packets.start();
    QEventLoop loop;
    QObject::connect(&observer, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), &loop, &QEventLoop::quit);
    QTimer::singleShot(5000, &loop, &QEventLoop::quit);
    loop.exec();
    if (observer.state() != QProcess::NotRunning) { observer.kill(); observer.waitForFinished(); return 2; }
    const QByteArray output = observer.readAllStandardOutput().trimmed();
    const auto result = QJsonDocument::fromJson(output).object();
    const bool passed = observer.exitStatus() == QProcess::NormalExit && observer.exitCode() == 1 &&
        (freezeVision ? result["final_vision_age_ms"].toDouble() >= 400 &&
                        result["unanswered_commands"].toInt() == 0
                      : result["final_vision_age_ms"].toDouble() < 75 &&
                        result["unanswered_commands"].toInt() > 0);
    std::printf("%s observer rejects %s: %s\n", passed ? "PASS" : "FAIL",
                freezeVision ? "a final 500 ms vision freeze" : "unanswered commands with healthy vision",
                output.constData());
    return passed ? 0 : 1;
}

static bool scenario(MainWindow& window, GLWidget& view, quint16 controlPort, double fps, int stall,
                     bool rendering)
{
    QProcess observer;
    observer.start(QCoreApplication::applicationFilePath(), {"--observe", QString::number(controlPort)});
    if (!observer.waitForStarted(5000)) return false;
    QElapsedTimer timeout;
    timeout.start();
    while (!observer.canReadLine() && timeout.elapsed() < 5000) observer.waitForReadyRead(100);
    const QByteArray ready = observer.readLine().trimmed();
    if (!ready.startsWith("READY ")) { observer.kill(); observer.waitForFinished(); return false; }
    view.cfg->v_VisionMulticastPort->setInt(ready.mid(6).toUShort());
    view.cfg->v_DesiredFPS->setDouble(fps);
    window.reconnectVisionSocket();
    window.changeTimer();
    int rendered = 0;
    const auto connection = QObject::connect(&view, &GLWidget::frameRendered, [&]() {
        ++rendered;
        if (rendering) {
            view.makeCurrent();
            glFinish();
            QThread::msleep(90);
        }
    });
    QEventLoop loop;
    QObject::connect(&observer, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), &loop, &QEventLoop::quit);
    if (stall) QTimer::singleShot(200, [&]() { QThread::msleep(stall); });
    QTimer::singleShot(10000, &loop, &QEventLoop::quit);
    loop.exec();
    QObject::disconnect(connection);
    if (observer.state() != QProcess::NotRunning) { observer.kill(); observer.waitForFinished(); return false; }
    const QByteArray output = observer.readAllStandardOutput().trimmed();
    const bool passed = observer.exitStatus() == QProcess::NormalExit && observer.exitCode() == 0 &&
        (!rendering || rendered >= (fps >= 5 ? 2 : 1));
    std::printf("%s fps=%.0f gui_stall=%dms rendering=%d drawn_frames=%d injected_render_cost=%dms %s\n",
                passed ? "PASS" : "FAIL", fps, stall, rendering, rendered, rendering ? 90 : 0, output.constData());
    if (observer.exitCode()) std::fprintf(stderr, "%s", observer.readAllStandardError().constData());
    return passed;
}

static bool checkSchedule()
{
    bool passed = true;
    for (double fps : {1.0, 5.0, 59.94, 60.0, 120.0, 1000.0}) {
        FramePacer schedule;
        schedule.reset(0, fps);
        qint64 now = 0;
        unsigned frames = 0;
        // Drive the same deadlines used by MainWindow with a synthetic clock.
        while (now < 60000000000LL) {
            const int delay = schedule.delayMilliseconds(now);
            passed = delay >= 1 && delay <= 1000 && passed;
            now += qint64(delay) * 1000000;
            if (schedule.due(now)) { ++frames; schedule.advance(now); }
        }
        const bool count = std::abs(double(frames) - 60 * fps) <= 1;
        passed = count && passed;
        std::printf("%s deadline count %.2f FPS over 60 seconds: %u\n",
                    count ? "PASS" : "FAIL", fps, frames);
        // A blocked GUI drops missed render deadlines without a catch-up burst.
        schedule.advance(now + 5000000000LL);
        passed = !schedule.due(now + 5000000000LL) && passed;
    }
    for (double invalid : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
        FramePacer schedule;
        schedule.reset(0, invalid);
        passed = schedule.delayMilliseconds(0) == 17 && passed;
    }
    TimingWindow samples;
    for (unsigned i = 1; i <= 240; ++i) samples.add(i);
    samples.add(-1); samples.add(std::numeric_limits<double>::quiet_NaN());
    const auto full = samples.summary();
    passed = full.samples == 240 && full.medianMs == 120 && full.p95Ms == 228 &&
             full.p99Ms == 238 && full.maximumMs == 240 && passed;
    for (unsigned i = 0; i < 240; ++i) samples.add(2);
    const auto wrapped = samples.summary();
    passed = wrapped.samples == 240 && wrapped.p99Ms == 2 && passed;
    std::printf("%s bounded timing histories, invalid FPS and stalled deadlines\n",
                passed ? "PASS" : "FAIL");
    return passed;
}

// Stop the producer before deliberately ageing its last completed snapshot.
// This exercises the real renderTick policy without racing a running worker.
static bool checkCadence(MainWindow& window, GLWidget& view)
{
    view.simulation->stop();
    auto frozen = std::const_pointer_cast<SceneSnapshot>(view.scene());
    if (!frozen) return false;
    frozen->pendingTime = 0.5;
    view.refreshSnapshot();
    view.updateGL();
    QCoreApplication::processEvents();
    const auto tick = frozen->tick;
    int frames = 0;
    bool fractionalInterval = false, boundedTimer = true;
    const auto frameConnection = QObject::connect(&view, &GLWidget::frameRendered,
                                                  [&]() { ++frames; });
    QTimer* render = nullptr;
    for (auto* timer : window.findChildren<QTimer*>(QString(), Qt::FindDirectChildrenOnly))
        if (timer->timerType() == Qt::PreciseTimer) render = timer;
    if (!render) return false;
    const auto timerConnection = QObject::connect(render, &QTimer::timeout, [&]() {
        fractionalInterval = fractionalInterval || render->interval() < 17;
        boundedTimer = boundedTimer && render->interval() > 0;
    });
    QEventLoop loop;
    QTimer::singleShot(600, &loop, &QEventLoop::quit);
    loop.exec();
    QObject::disconnect(frameConnection);
    QObject::disconnect(timerConnection);
    const bool drawing = frames >= 5 && view.scene()->tick == tick;
    std::printf("%s 60 FPS uses fractional deadlines without a zero-timer spin\n",
                fractionalInterval && boundedTimer ? "PASS" : "FAIL");
    std::printf("%s drawing continues from completed snapshots during physics lag (%d frames)\n",
                drawing ? "PASS" : "FAIL", frames);
    const auto timings = view.frameTimings();
    const bool measured = timings.frames >= unsigned(frames) && timings.interval.samples > 0 &&
        timings.scene.samples > 0 && timings.labels.samples > 0 && timings.paintAndSwap.samples > 0 &&
        (!timings.gpuRequested || !timings.gpuSupported || timings.gpu.samples > 0);
    std::printf("%s frame measurements; GPU requested=%d supported=%d samples=%u\n",
                measured ? "PASS" : "FAIL", timings.gpuRequested, timings.gpuSupported, timings.gpu.samples);
    return fractionalInterval && boundedTimer && drawing && measured;
}

int main(int argc, char** argv)
{
    if (argc > 1 && QByteArray(argv[1]) == "--schedule") return checkSchedule() ? 0 : 1;
    if (argc > 2 && QByteArray(argv[1]) == "--observe") return observe(argc, argv);
    if (argc > 1 && QByteArray(argv[1]) == "--check-observer") return checkObserver(argc, argv);
    const bool cadence = argc > 1 && QByteArray(argv[1]) == "--cadence";
    const bool rendering = cadence || (argc > 1 && QByteArray(argv[1]) == "--rendering");
    QApplication app(argc, argv);
    QUdpSocket reserve;
    if (!reserve.bind(QHostAddress(QHostAddress::LocalHost), 0)) return 2;
    const quint16 controlPort = reserve.localPort();
    reserve.close();
    auto* config = new ConfigWidget(false);
    config->v_DeltaTime->setDouble(1.0/60.0);
    config->v_DesiredFPS->setDouble(60);
    config->v_Robots_Count->setInt(1);
    config->v_sendDelay->setInt(0);
    config->v_sendGeometryEvery->setInt(120);
    config->v_VisionMulticastAddr->setString("127.0.0.1");
    config->v_CommandListenPort->setInt(0);
    config->v_SimControlListenPort->setInt(0);
    config->v_BlueControlListenPort->setInt(controlPort);
    config->v_YellowControlListenPort->setInt(0);
    auto* window = new MainWindow(nullptr, config);
    if (!rendering) { window->hide(); window->setIsGlEnabled(false); }
    else { window->showNormal(); window->resize(800, 600); app.processEvents(); }
    auto* view = window->findChild<GLWidget*>();
    if (!view) return 2;
    if (rendering && !view->isValid()) return 2;
    if (cadence) {
        const bool passed = checkCadence(*window, *view);
        delete window;
        return passed ? 0 : 1;
    }
    bool passed = scenario(*window, *view, controlPort, 60, 0, rendering);
    passed = scenario(*window, *view, controlPort, 5, 0, rendering) && passed;
    passed = scenario(*window, *view, controlPort, 1, 0, rendering) && passed;
    passed = scenario(*window, *view, controlPort, 60, 500, rendering) && passed;
    delete window;
    return passed ? 0 : 1;
}
