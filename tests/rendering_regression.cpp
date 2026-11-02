#include <QApplication>
#include <QTimer>
#include <QDir>
#include <QElapsedTimer>
#include <QThread>
#include <QWindow>
#include <cstdio>
#include <cstdlib>
#include <set>
#include <functional>
#include <QOpenGLContext>
#include <QOpenGLFunctions>
#include "mainwindow.h"

class TestView : public GLWidget {
public:
    TestView(ConfigWidget* config, SimulationController* simulation)
        : GLWidget(nullptr, config, simulation) {}
    bool allowBlit = true;
    using GLWidget::paintGL;
protected:
    bool hasFramebufferBlit() const override { return allowBlit && GLWidget::hasFramebufferBlit(); }
};

// These checks detect blank or missing captures, not texture or field fidelity.
static bool checkNonblankImage(const QImage& image, bool low, const QSize& size)
{
    std::set<QRgb> colors;
    int opaque = 0;
    for (int y = 0; y < image.height(); ++y) {
        for (int x = 0; x < image.width(); ++x) {
            const QRgb pixel = image.pixel(x, y);
            colors.insert(pixel);
            if (qAlpha(pixel) > 0) ++opaque;
        }
    }
    const bool passed = image.size() == size && colors.size() > 16 &&
        opaque > image.width() * image.height() / 2;
    std::printf("%s nonblank capture low=%d size=%dx%d colors=%zu visible_pixels=%d\n",
                passed ? "PASS" : "FAIL", low, size.width(), size.height(),
                colors.size(), opaque);
    return passed;
}

static bool checkCapture(GLWidget& view, bool low, const QSize& size)
{
    view.cfg->v_LowSpecMode->setBool(low);
    view.updateGL();
    const QImage image = view.renderPixmap(size.width(), size.height(), false).toImage();
    const QString artifacts = QString::fromLocal8Bit(qgetenv("GRSIM_TEST_ARTIFACT_DIR"));
    if (!artifacts.isEmpty()) {
        QDir().mkpath(artifacts);
        image.save(artifacts + QString("/snapshot-%1-%2x%3.png").arg(low ? "low" : "normal").arg(size.width()).arg(size.height()));
    }
    return checkNonblankImage(image, low, size);
}

static bool checkTexturedCylinderDetail(GLWidget& view)
{
    QGLWidget context;
    context.makeCurrent();
    if (!context.isValid()) return false;
    const auto scene = view.scene();
    SceneRenderer renderer(&context);
    renderer.initialize(*scene);
    renderer.graphics.setViewpoint(0, -1, 0.5, 90, -20, 0);
    bool passed = true;
    for (bool chassis : {false, true}) {
        SceneSnapshot fixture = *scene;
        fixture.primitives.clear();
        for (const auto& primitive : scene->primitives) {
            if (primitive.kind == PrimitiveKind::Cylinder && primitive.texture >= 0 &&
                primitive.robotChassis == chassis) {
                fixture.primitives.push_back(primitive);
                break;
            }
        }
        if (fixture.primitives.empty()) return false;
        auto& cylinder = fixture.primitives.front();
        cylinder.position = {{0, 0, 0}};
        cylinder.rotation = {{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0}};
        cylinder.dimensions = {{0.2, 0.3, 0}};
        cylinder.visible = true;
        auto vertices = [&](bool low) {
            GLfloat feedback[16384];
            glFeedbackBuffer(16384, GL_3D, feedback);
            glRenderMode(GL_FEEDBACK);
            auto quality = RenderQuality::forLowSpec(low);
            quality.showSkybox = false;
            renderer.draw(fixture, 512, 512, quality, -1, false, {{0, 0, 0}}, 0);
            return glRenderMode(GL_RENDER);
        };
        const int normal = vertices(false), low = vertices(true), repeated = vertices(true);
        const bool reduced = normal > 0 && low > 0 && low < normal && repeated == low;
        std::printf("%s textured %s geometry low=%d normal=%d feedback values\n",
                    reduced ? "PASS" : "FAIL", chassis ? "chassis" : "wheel", low, normal);
        passed = reduced && passed;
    }
    renderer.graphics.clearTextures();
    return passed;
}

static bool checkPicking(TestView& view)
{
    view.updateGL();
    const auto scene = view.scene();
    const RenderPrimitive* robot = nullptr;
    for (const auto& primitive : scene->primitives)
        if (primitive.robotChassis && primitive.robotIndex == 0) { robot = &primitive; break; }
    if (!robot) return false;
    view.makeCurrent();
    GLdouble model[16], projection[16];
    GLint viewport[4];
    glGetDoublev(GL_MODELVIEW_MATRIX, model);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);
    glGetIntegerv(GL_VIEWPORT, viewport);
    const double point[4] = {robot->position[0], robot->position[1], robot->position[2], 1};
    double eye[4] = {}, clip[4] = {};
    for (int row = 0; row < 4; ++row)
        for (int col = 0; col < 4; ++col) eye[row] += model[col*4+row]*point[col];
    for (int row = 0; row < 4; ++row)
        for (int col = 0; col < 4; ++col) clip[row] += projection[col*4+row]*eye[col];
    const double ratio = view.devicePixelRatioF();
    const int x = qRound((viewport[0]+(clip[0]/clip[3]+1)*viewport[2]/2)/ratio);
    const int y = qRound(view.height()-(viewport[1]+(clip[1]/clip[3]+1)*viewport[3]/2)/ratio);
    view.update3DCursor(x, y);
    const bool picked = view.selected == 0;
    view.updateGL();
    const bool passed = picked && view.selected == 0;
    std::printf("%s snapshot picking and selection survive repeated drawing\n", passed ? "PASS" : "FAIL");
    return passed;
}

static bool checkCameraTracking(TestView& view)
{
    auto matrix = [&]() {
        view.updateGL();
        view.makeCurrent();
        std::array<double, 16> result;
        glGetDoublev(GL_MODELVIEW_MATRIX, result.data());
        return result;
    };
    auto waitFor = [&](const std::function<bool()>& predicate) {
        QElapsedTimer timer;
        timer.start();
        while (!predicate() && timer.elapsed() < 2000) {
            QCoreApplication::processEvents(); QThread::msleep(1);
        }
        return predicate();
    };
    view.lockCameraToRobot();
    const auto beforeRobot = matrix();
    SimulationAction action;
    action.generation = view.scene()->generation;
    action.kind = SimulationActionKind::PlaceRobot;
    action.robot = 0; action.x = 1; action.y = 0.2;
    view.simulation->submit(action);
    bool passed = waitFor([&]() { return std::abs(view.scene()->robots[0].x-1) < 0.01; });
    passed = matrix() != beforeRobot && passed;
    view.lockCameraToBall();
    const auto beforeBall = matrix();
    action.kind = SimulationActionKind::PlaceBall;
    action.x = 2; action.y = 0.5; action.z = 0.3;
    view.simulation->submit(action);
    passed = waitFor([&]() { return std::abs(view.scene()->ballPosition[0]-2) < 0.01; }) && passed;
    passed = matrix() != beforeBall && passed;
    std::set<std::array<double, 16>> modes;
    for (int i = 0; i < 6; ++i) { view.changeCameraMode(); modes.insert(matrix()); }
    passed = modes.size() == 6 && passed;
    std::printf("%s cameras follow changed snapshots and switch modes\n", passed ? "PASS" : "FAIL");
    return passed;
}

static bool checkNoBlitUpdates(TestView& view)
{
    view.updateGL();
    const QImage before = view.grabFrameBuffer();
    SimulationAction action;
    action.generation = view.scene()->generation;
    action.kind = SimulationActionKind::PlaceBall;
    action.x = 1; action.y = 1; action.z = 0.2;
    view.simulation->submit(action);
    QElapsedTimer wait;
    wait.start();
    while (std::abs(view.scene()->ballPosition[0]-1) > 0.01 && wait.elapsed() < 2000) {
        QCoreApplication::processEvents(); QThread::msleep(1);
    }
    view.updateGL();
    const bool passed = std::abs(view.scene()->ballPosition[0]-1) < 0.01 &&
        view.grabFrameBuffer() != before;
    std::printf("%s direct Low Spec rendering keeps updating without framebuffer blitting\n", passed ? "PASS" : "FAIL");
    return passed;
}

int main(int argc, char** argv)
{
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QApplication app(argc, argv);
    auto* config = new ConfigWidget(false);
    config->v_Robots_Count->setInt(1);
    config->v_CommandListenPort->setInt(0);
    config->v_SimControlListenPort->setInt(0);
    config->v_BlueControlListenPort->setInt(0);
    config->v_YellowControlListenPort->setInt(0);
    config->v_VisionMulticastAddr->setString("127.0.0.1");
    auto* window = new MainWindow(nullptr, config);
    window->hide();
    for (auto* timer : window->findChildren<QTimer*>()) timer->stop();
    auto* view = window->findChild<GLWidget*>();
    if (!view || !view->isValid()) {
        std::fprintf(stderr, "No valid OpenGL context\n");
        std::_Exit(2);
    }
    view->resize(512, 512);
    view->makeCurrent();
    std::printf("OpenGL renderer: %s; display scale: %.1f\n", glGetString(GL_RENDERER), view->devicePixelRatioF());
    if (qgetenv("QT_SCALE_FACTOR") == "2" && view->devicePixelRatioF() < 1.9) return 2;
    bool passed = true;
    passed = checkTexturedCylinderDetail(*view) && passed;
    for (const QSize size : {QSize(512, 512), QSize(320, 240)}) {
        passed = checkCapture(*view, false, size) && passed;
        passed = checkCapture(*view, true, size) && passed;
    }
    {
        TestView fallback(config, view->simulation);
        fallback.resize(512, 512);
        fallback.show();
        QElapsedTimer exposed;
        exposed.start();
        while ((!fallback.windowHandle() || !fallback.windowHandle()->isExposed()) &&
               exposed.elapsed() < 2000) {
            app.processEvents(); QThread::msleep(1);
        }
        app.processEvents();
        fallback.allowBlit = false;
        config->v_LowSpecMode->setBool(true);
        fallback.updateGL();
        const QImage displayed = fallback.grabFrameBuffer();
        passed = checkNonblankImage(displayed, true, displayed.size()) && passed;
        passed = checkNoBlitUpdates(fallback) && passed;
        passed = checkCapture(fallback, true, QSize(320, 240)) && passed;
        passed = checkPicking(fallback) && passed;

        const auto generation = fallback.scene()->generation;
        view->simulation->configure(config->simulationSettings(), true);
        QElapsedTimer wait;
        wait.start();
        while (fallback.scene()->generation == generation && wait.elapsed() < 2000) {
            app.processEvents(); QThread::msleep(1);
        }
        const bool reset = fallback.scene()->generation > generation;
        std::printf("%s scene reset while rendering\n", reset ? "PASS" : "FAIL");
        passed = reset && passed;
        passed = checkCapture(fallback, true, QSize(320, 240)) && passed;
        passed = checkCameraTracking(fallback) && passed;

        fallback.makeCurrent();
        QGLFramebufferObject target(QSize(377, 211), QGLFramebufferObject::Depth);
        target.bind();
        glViewport(0, 0, 377, 211);
        fallback.paintGL();
        GLint binding = 0, viewport[4];
        glGetIntegerv(GL_FRAMEBUFFER_BINDING, &binding);
        glGetIntegerv(GL_VIEWPORT, viewport);
        const bool restored = GLuint(binding) == target.handle() && viewport[2] == 377 && viewport[3] == 211;
        std::printf("%s capture framebuffer and viewport preserved\n", restored ? "PASS" : "FAIL");
        passed = restored && passed;
        passed = checkNonblankImage(target.toImage(), true, QSize(377, 211)) && passed;
        target.release();
    }
    delete window;
    return passed ? 0 : 1;
}
