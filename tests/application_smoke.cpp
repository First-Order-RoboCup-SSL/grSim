#include <QApplication>
#include <QElapsedTimer>
#include <QImage>
#include <QLabel>
#include <QThread>
#include <QSet>
#include <QTimer>
#include <QUdpSocket>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include "mainwindow.h"
#include "ssl_simulation_robot_control.pb.h"
#include "ssl_simulation_robot_feedback.pb.h"
#include "ssl_vision_wrapper.pb.h"

// Run through the real executable and its installed resources; never read or
// write personal settings. CTest and the relocated macOS artifact use this path.
int runApplicationSmokeTest(QApplication& app, const QString& settingsPath)
{
    QUdpSocket vision, feedback, reserve;
    if (!vision.bind(QHostAddress(QHostAddress::LocalHost), 0) ||
        !feedback.bind(QHostAddress(QHostAddress::LocalHost), 0) ||
        !reserve.bind(QHostAddress(QHostAddress::LocalHost), 0)) return 2;
    const quint16 controlPort = reserve.localPort();
    reserve.close();
    auto* config = new ConfigWidget(false, settingsPath);
    config->v_BlueTeam->setString("Parsian");
    config->v_YellowTeam->setString("Parsian");
    config->loadRobotsSettings();
    std::printf("Settings: rolling_deceleration=%.2f\n", config->BallRollingDeceleration());
    const auto model = config->simulationSettings().blueSettings;
    const bool resources = std::abs(model.RobotHeight - 0.147) < 1e-9 &&
        std::abs(model.WheelRadius - 0.027) < 1e-9;
    config->v_Robots_Count->setInt(1);
    config->v_VisionMulticastAddr->setString("127.0.0.1");
    config->v_VisionMulticastPort->setInt(vision.localPort());
    config->v_CommandListenPort->setInt(0);
    config->v_SimControlListenPort->setInt(0);
    config->v_BlueControlListenPort->setInt(controlPort);
    config->v_YellowControlListenPort->setInt(0);
    config->v_sendDelay->setInt(0);
    MainWindow window(nullptr, config);
    const bool initiallyHidden = !window.isVisible();
    window.resize(800, 600);
    window.show();
    auto* view = window.findChild<GLWidget*>();
    if (!view) return 2;

    std::atomic<bool> renderOnGuiThread{true};
    QObject::connect(view, &GLWidget::frameRendered, [&]() {
        if (QThread::currentThread() != app.thread()) renderOnGuiThread.store(false);
    });
    int frames = 0, responses = 0;
    double firstTime = -1, lastTime = -1;
    bool monotonic = true, havePosition = false;
    double initialX = 0, initialY = 0, distance = 0;
    QObject::connect(&vision, &QUdpSocket::readyRead, [&]() {
        while (vision.hasPendingDatagrams()) {
            QByteArray data(int(vision.pendingDatagramSize()), 0);
            vision.readDatagram(data.data(), data.size());
            SSL_WrapperPacket packet;
            if (!packet.ParseFromArray(data.data(), data.size()) || !packet.has_detection()) continue;
            const auto& detection = packet.detection();
            for (const auto& robot : detection.robots_blue()) {
                if (robot.robot_id() != 0) continue;
                if (!havePosition) {
                    initialX = robot.x(); initialY = robot.y(); havePosition = true;
                }
                distance = std::max(distance, std::hypot(robot.x()-initialX, robot.y()-initialY));
            }
            if (detection.camera_id() == 0) {
            if (firstTime < 0) firstTime = detection.t_capture();
            else monotonic = monotonic && detection.t_capture() > lastTime;
            lastTime = detection.t_capture();
            ++frames;
            }
        }
    });
    QObject::connect(&feedback, &QUdpSocket::readyRead, [&]() {
        while (feedback.hasPendingDatagrams()) {
            QByteArray data(int(feedback.pendingDatagramSize()), 0);
            feedback.readDatagram(data.data(), data.size());
            RobotControlResponse response;
            if (response.ParseFromArray(data.data(), data.size()) &&
                response.errors_size() == 0 && response.feedback_size() > 0) ++responses;
        }
    });
    QTimer commands;
    commands.setInterval(100);
    QObject::connect(&commands, &QTimer::timeout, [&]() {
        RobotControl command;
        auto* robot = command.add_robot_commands();
        robot->set_id(0);
        auto* velocity = robot->mutable_move_command()->mutable_local_velocity();
        velocity->set_forward(0.3f); velocity->set_left(0); velocity->set_angular(0);
        const auto data = command.SerializeAsString();
        feedback.writeDatagram(data.data(), int(data.size()), QHostAddress::LocalHost, controlPort);
    });
    commands.start();
    int result = 1;
    QTimer::singleShot(4000, &app, [&]() {
        commands.stop();
        view->makeCurrent();
        const QImage normal = view->grabFrameBuffer();
        config->v_LowSpecMode->setBool(true);
        view->updateGL();
        const QImage low = view->grabFrameBuffer();
        auto nonblank = [](const QImage& image) {
            QSet<QRgb> colors;
            for (int y = 0; y < image.height(); y += 3)
                for (int x = 0; x < image.width(); x += 3) colors.insert(image.pixel(x, y));
            return !image.isNull() && colors.size() > 16;
        };
        const bool textures = !QImage(":/y0.png").isNull() && !QImage(":/b0.png").isNull();
        const auto* graphicsStatus = window.findChild<QLabel*>("graphicsDeviceStatus");
        const auto* physicsStatus = window.findChild<QLabel*>("physicsProcessorStatus");
        const QString device = view->graphicsRendererName();
        const bool deviceStatus = graphicsStatus && physicsStatus && !device.isEmpty() &&
            graphicsStatus->toolTip().contains(device) && physicsStatus->text().contains("CPU");
        const bool rendering = initiallyHidden && view->isValid() && renderOnGuiThread.load() &&
            nonblank(normal) && nonblank(low);
        const bool network = frames > 30 && responses > 5 && monotonic && lastTime-firstTime > 0.5;
        result = resources && textures && rendering && deviceStatus && network && distance > 10 ? 0 : 1;
        std::printf("%s application smoke: config=%d textures=%d OpenGL=%d vision=%d feedback=%d moved=%.1fmm\n",
                    result == 0 ? "PASS" : "FAIL", resources, textures, rendering, frames, responses, distance);
        std::printf("Renderer: %s; status=%d; rendering_on_gui_thread=%d\n",
                    device.toUtf8().constData(), deviceStatus, renderOnGuiThread.load());
        std::fflush(stdout);
        app.exit(result);
    });
    app.exec();
    return result;
}
