#include <QApplication>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QThread>
#include <QUdpSocket>
#include <cmath>
#include <cstdio>
#include <functional>
#include "mainwindow.h"
#include "ssl_vision_wrapper.pb.h"

static bool waitFor(const std::function<bool()>& predicate)
{
    QElapsedTimer clock;
    clock.start();
    while (clock.elapsed() < 2000) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 5);
        if (predicate()) return true;
        QThread::msleep(1);
    }
    return predicate();
}

static bool check(bool passed, const char* description)
{
    std::printf("%s %s\n", passed ? "PASS" : "FAIL", description);
    return passed;
}

static bool equal(double a, double b) { return std::abs(a - b) < 1e-6; }

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    QUdpSocket vision;
    if (!vision.bind(QHostAddress(QHostAddress::LocalHost), 0)) return 2;
    SSL_GeometryFieldSize advertised;
    QObject::connect(&vision, &QUdpSocket::readyRead, [&]() {
        while (vision.hasPendingDatagrams()) {
            QByteArray bytes(vision.pendingDatagramSize(), 0);
            vision.readDatagram(bytes.data(), bytes.size());
            SSL_WrapperPacket packet;
            if (packet.ParseFromArray(bytes.data(), bytes.size()) && packet.has_geometry())
                advertised = packet.geometry().field();
        }
    });
    auto* config = new ConfigWidget(false);
    config->v_Robots_Count->setInt(0);
    config->v_CommandListenPort->setInt(0);
    config->v_SimControlListenPort->setInt(0);
    config->v_BlueControlListenPort->setInt(0);
    config->v_YellowControlListenPort->setInt(0);
    config->v_VisionMulticastAddr->setString("127.0.0.1");
    config->v_VisionMulticastPort->setInt(vision.localPort());
    config->v_sendGeometryEvery->setInt(4);
    config->v_DivA_Field_Referee_Margin->setDouble(0.1);
    MainWindow window(nullptr, config);
    window.hide();
    window.setIsGlEnabled(false);
    auto* simulation = window.findChild<GLWidget*>()->simulation;
    auto packetMatches = [&](int length, int touch, int goal, int substitution) {
        return advertised.field_length() == length && advertised.boundary_width() == touch &&
            advertised.has_boundary_width_goal_line() && advertised.boundary_width_goal_line() == goal &&
            advertised.has_goal_substitution_area_width() &&
            advertised.goal_substitution_area_width() == substitution;
    };
    auto wallsMatch = [&]() {
        const auto scene = simulation->latest();
        if (!scene) return false;
        const auto& settings = scene->settings;
        const double goalBoundary = settings.Field_Length()/2 + settings.Field_Margin_Goal_Line() +
            settings.Field_Referee_Margin() + settings.Wall_Thickness()/2;
        const double touchBoundary = settings.Field_Width()/2 + settings.Field_Margin_Touch_Line() +
            settings.Field_Referee_Margin() + settings.Wall_Thickness()/2;
        int goalWalls = 0, touchWalls = 0, goalSides = 0;
        for (const auto& primitive : scene->primitives) {
            if (primitive.kind != PrimitiveKind::Box) continue;
            if (equal(primitive.dimensions[0], settings.Wall_Thickness()) &&
                equal(std::abs(primitive.position[0]), goalBoundary)) ++goalWalls;
            if (equal(primitive.dimensions[1], settings.Wall_Thickness()) &&
                equal(std::abs(primitive.position[1]), touchBoundary)) ++touchWalls;
            if (equal(primitive.dimensions[0], settings.Field_Margin_Goal_Line() + settings.Field_Referee_Margin()) &&
                equal(primitive.dimensions[1], settings.Goal_Thickness())) ++goalSides;
        }
        return goalWalls == 2 && touchWalls == 2 && goalSides == 4;
    };
    bool passed = check(waitFor([&]() { return packetMatches(12000, 300, 600, 300); }),
                        "Division A advertises the 2026 boundary and substitution widths in millimetres");
    passed = check(wallsMatch(), "Division A walls use the separate touch and goal margins") && passed;
    const auto initial = simulation->latest();
    if (!initial) return 2;
    auto generation = initial->generation;
    config->v_DivA_Field_Margin_Goal_Line->setDouble(0.8);
    config->v_DivA_Field_Margin_Goal_Line->wasEdited(config->v_DivA_Field_Margin_Goal_Line);
    passed = check(waitFor([&]() {
        return simulation->latest()->generation > generation && packetMatches(12000, 300, 800, 300);
    }) && wallsMatch(), "editing the goal margin rebuilds the walls including the referee margin") && passed;
    generation = simulation->latest()->generation;
    config->v_DivA_Field_Margin_Touch_Line->setDouble(0.4);
    config->v_DivA_Field_Margin_Touch_Line->wasEdited(config->v_DivA_Field_Margin_Touch_Line);
    passed = check(waitFor([&]() {
        return simulation->latest()->generation > generation && packetMatches(12000, 400, 800, 300);
    }) && wallsMatch(), "editing the touch margin updates the worker and boundary walls") && passed;
    config->v_DivA_Field_Goal_Substitution_Area_Width->setDouble(0.2);
    config->v_DivA_Field_Goal_Substitution_Area_Width->wasEdited(config->v_DivA_Field_Goal_Substitution_Area_Width);
    passed = check(waitFor([&]() { return packetMatches(12000, 400, 800, 200); }),
                   "editing the substitution width reaches outgoing vision packets") && passed;
    config->v_Division->setString("Division B");
    config->v_Division->wasEdited(config->v_Division);
    passed = check(waitFor([&]() { return packetMatches(9000, 300, 300, 0); }) && wallsMatch(),
                   "switching to Division B restores its independent geometry defaults") && passed;
    return passed ? 0 : 1;
}
