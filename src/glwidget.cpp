/*
grSim - RoboCup Small Size Soccer Robots Simulator
Copyright (C) 2011, Parsian Robotic Center (eew.aut.ac.ir/~parsian/grsim)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "config.h"
#include <QtGui>

#include <QPainter>
#include "glwidget.h"
#include "logger.h"
#include <QLabel>

#include <iostream>
#include <QHBoxLayout>
#include <QStyle>
#include <QOpenGLContext>
#include <QOpenGLFunctions>
#include <QResizeEvent>
#include <limits>
#include <cmath>

GLWidget::GLWidget(QWidget *parent, ConfigWidget* _cfg, SimulationController* controller)
    : QGLWidget(parent), cfg(_cfg), simulation(controller), renderer(this)
{
    Q_INIT_RESOURCE(textures);
    state = CursorMode::STEADY;
    m_fbo = nullptr;
    selected = -1;
    clicked_robot = -1;
    moving_robot_id = -1;
    renderer.graphics.setViewpoint(0, -(cfg->Field_Width() + cfg->Field_Margin_Touch_Line() * 2) / 2, 3, 90, -45, 0);
    Current_robot = 0;
    Current_team = 0;
    cammode = CameraMode::BIRDS_EYE_FROM_TOUCH_LINE;
    setMouseTracking(true);

    blueRobotsMenu = new QMenu("&Blue Robots");
    yellowRobotsMenu = new QMenu("&Yellow Robots");
    allRobotsMenu = new QMenu("&All Robots");
    blueRobotsMenu->addAction(tr("Put all inside with formation 1"));
    blueRobotsMenu->addAction(tr("Put all inside with formation 2"));
    blueRobotsMenu->addAction(tr("Put all outside"));
    blueRobotsMenu->addAction(tr("Put all out of field"));
    blueRobotsMenu->addAction(tr("Turn all off"));
    blueRobotsMenu->addAction(tr("Turn all on"));
    yellowRobotsMenu->addAction(tr("Put all inside with formation 1"));
    yellowRobotsMenu->addAction(tr("Put all inside with formation 2"));
    yellowRobotsMenu->addAction(tr("Put all outside"));
    yellowRobotsMenu->addAction(tr("Put all out of field"));
    yellowRobotsMenu->addAction(tr("Turn all off"));
    yellowRobotsMenu->addAction(tr("Turn all on"));
    allRobotsMenu->addAction(tr("Put all inside with formation 1"));
    allRobotsMenu->addAction(tr("Put all inside with formation 2"));
    allRobotsMenu->addAction(tr("Put all outside"));
    allRobotsMenu->addAction(tr("Put all out of field"));
    allRobotsMenu->addAction(tr("Turn all off"));
    allRobotsMenu->addAction(tr("Turn all on"));
    robpopup = new QMenu(this);
    moveRobotAct = new QAction(tr("&Locate robot"),this);
    selectRobotAct = new QAction(tr("&Select robot"),this);
    resetRobotAct = new QAction(tr("&Reset robot"),this);
    onOffRobotAct = new QAction(tr("Turn &off"),this);
    lockToRobotAct = new QAction(tr("Loc&k camera to this robot"),this);    
    robpopup->addAction(moveRobotAct);
    robpopup->addAction(resetRobotAct);
    robpopup->addAction(onOffRobotAct);
    robpopup->addAction(lockToRobotAct);
    robpopup->addMenu(blueRobotsMenu);
    robpopup->addMenu(yellowRobotsMenu);
    robpopup->addMenu(allRobotsMenu);

    moveBallAct = new QAction(tr("&Locate ball"),this);
    lockToBallAct = new QAction(tr("Loc&k camera to ball"),this);
    ballpopup = new QMenu(this);
    ballpopup->addAction(moveBallAct);
    ballpopup->addAction(lockToBallAct);

    moveRobotHereAct = new QAction(tr("Locate current &robot here"),this);
    moveBallHereAct = new QAction(tr("Locate &ball here"),this);
    mainpopup = new QMenu(this);
    mainpopup->addAction(moveBallHereAct);
    mainpopup->addAction(moveRobotHereAct);
    cameraMenu = new QMenu("&Camera");
    changeCamModeAct=new QAction(tr("Change &mode"),this);
    changeCamModeAct->setShortcut(QKeySequence("C"));
    cameraMenu->addAction(changeCamModeAct);
    cameraMenu->addAction(lockToRobotAct);
    cameraMenu->addAction(lockToBallAct);

    mainpopup->addMenu(cameraMenu);
    mainpopup->addMenu(blueRobotsMenu);
    mainpopup->addMenu(yellowRobotsMenu);
    mainpopup->addMenu(allRobotsMenu);


    connect(moveRobotAct, SIGNAL(triggered()), this, SLOT(moveRobot()));
    connect(selectRobotAct, SIGNAL(triggered()), this, SLOT(selectRobot()));
    connect(resetRobotAct, SIGNAL(triggered()), this, SLOT(resetCurrentRobot()));
    connect(moveBallAct, SIGNAL(triggered()), this, SLOT(moveBall()));
    connect(onOffRobotAct, SIGNAL(triggered()), this, SLOT(switchRobotOnOff()));
    connect(yellowRobotsMenu,SIGNAL(triggered(QAction*)),this,SLOT(yellowRobotsMenuTriggered(QAction*)));
    connect(blueRobotsMenu,SIGNAL(triggered(QAction*)),this,SLOT(blueRobotsMenuTriggered(QAction*)));
    connect(allRobotsMenu,SIGNAL(triggered(QAction*)),this,SLOT(yellowRobotsMenuTriggered(QAction*)));
    connect(allRobotsMenu,SIGNAL(triggered(QAction*)),this,SLOT(blueRobotsMenuTriggered(QAction*)));
    connect(moveBallHereAct, SIGNAL(triggered()),this , SLOT(moveBallHere()));
    connect(moveRobotHereAct, SIGNAL(triggered()),this , SLOT(moveRobotHere()));
    connect(lockToRobotAct, SIGNAL(triggered()), this, SLOT(lockCameraToRobot()));
    connect(lockToBallAct, SIGNAL(triggered()), this, SLOT(lockCameraToBall()));
    connect(changeCamModeAct,SIGNAL(triggered()),this,SLOT(changeCameraMode()));
    setFocusPolicy(Qt::StrongFocus);
    ctrl = false;
    alt = false;
    kickingball = false;
    kickpower = 3.0;
    altTrigger = false;
    chipAngle = M_PI/4;
    chiping = false;
    auto* warning = new QFrame(this);
    lagWarning = warning;
    warning->setObjectName("simulationLagWarning");
    warning->setToolTip(tr("Simulation is falling behind real time."));
    warning->setStyleSheet("QFrame#simulationLagWarning { background: #ffe0a3; border: 1px solid #b77900; border-radius: 3px; } QLabel { color: #6b4500; }");
    auto* layout = new QHBoxLayout(warning);
    layout->setContentsMargins(6, 3, 6, 3);
    layout->setSpacing(4);
    auto* icon = new QLabel(warning);
    icon->setPixmap(style()->standardIcon(QStyle::SP_MessageBoxWarning).pixmap(16, 16));
    layout->addWidget(icon);
    layout->addWidget(new QLabel(tr("Lag"), warning));
    warning->adjustSize();
    warning->hide();
    refreshSnapshot();

}

GLWidget::~GLWidget()
{
    makeCurrent();
    delete m_fbo;
    renderer.graphics.clearTextures();
    doneCurrent();
}

SceneSnapshotPtr GLWidget::refreshSnapshot()
{
    auto latest = scene();
    if (!latest) return snapshot;
    if (!snapshot || latest->generation != snapshot->generation) {
        selected = clicked_robot = moving_robot_id = -1;
        showCursor = false;
        state = CursorMode::STEADY;
        const int count = latest->settings.Robots_Count();
        Current_robot = count ? qBound(0, Current_robot, count - 1) : -1;
    }
    snapshot = std::move(latest);
    lagWarning->setVisible(snapshot->lagging);
    if (snapshot->lagging) {
        lagWarning->adjustSize();
        lagWarning->move(qMax(8, width() - lagWarning->width() - 8), 8);
    }
    return snapshot;
}

int GLWidget::robotIndex(int id, int team) const
{
    const int count = snapshot ? snapshot->settings.Robots_Count() : cfg->Robots_Count();
    return id >= 0 && id < count && team >= 0 && team < 2 ? id + team * count : -1;
}

const RobotSnapshot* GLWidget::robotSnapshot(int index) const
{
    return snapshot && index >= 0 && index < int(snapshot->robots.size()) ? &snapshot->robots[index] : nullptr;
}

QImage GLWidget::robotImage(int index) const
{
    return snapshot && index >= 0 && index < int(snapshot->robots.size())
        ? SceneRenderer::robotImage(index, snapshot->settings) : QImage();
}

void GLWidget::sendAction(SimulationAction action)
{
    if (!snapshot) return;
    action.generation = snapshot->generation;
    simulation->submit(action);
}

void GLWidget::resizeEvent(QResizeEvent* event)
{
    QGLWidget::resizeEvent(event);
    lagWarning->move(qMax(8, width() - lagWarning->width() - 8), 8);
}

void GLWidget::moveRobot()
{
    moveCurrentRobot();
    moving_robot_id = clicked_robot;
}

void GLWidget::unselectRobot()
{
    showCursor = false;
    state = CursorMode::STEADY;
    moving_robot_id = robotIndex(Current_robot, Current_team);
}

void GLWidget::selectRobot()
{
    const auto* robot = robotSnapshot(clicked_robot);
    if (!robot) return;
    Current_robot = robot->id;
    Current_team = robot->team;
    emit selectedRobot();
}

void GLWidget::resetCurrentRobot()
{
    SimulationAction action;
    action.kind = SimulationActionKind::ResetRobot;
    action.robot = robotIndex(Current_robot, Current_team);
    sendAction(action);
}

void GLWidget::switchRobotOnOff()
{
    const int index = robotIndex(Current_robot, Current_team);
    const auto* robot = robotSnapshot(index);
    if (!robot) return;
    SimulationAction action;
    action.kind = SimulationActionKind::SetRobotEnabled;
    action.robot = index;
    action.enabled = !robot->on;
    sendAction(action);
}

void GLWidget::moveCurrentRobot()
{
    if (!snapshot) return;
    showCursor = true;
    cursorRadius = (Current_team ? snapshot->settings.yellowSettings : snapshot->settings.blueSettings).RobotRadius;
    state = CursorMode::PLACE_ROBOT;
    moving_robot_id = robotIndex(Current_robot, Current_team);
}

void GLWidget::moveBall()
{
    if (!snapshot) return;
    showCursor = true;
    cursorRadius = snapshot->settings.BallRadius();
    state = CursorMode::PLACE_BALL;
}

void GLWidget::mousePressEvent(QMouseEvent* event)
{
    if (!graphicsEnabled || !snapshot) return;
    lastPos = event->pos();
    update3DCursor(event->x(), event->y());
    if (event->buttons() & Qt::LeftButton) {
        SimulationAction action;
        action.x = cursor[0]; action.y = cursor[1];
        if (state == CursorMode::PLACE_ROBOT) {
            action.kind = SimulationActionKind::PlaceRobot;
            action.robot = moving_robot_id;
            sendAction(action);
            state = CursorMode::STEADY; showCursor = false;
        } else if (state == CursorMode::PLACE_BALL) {
            action.kind = SimulationActionKind::PlaceBall;
            action.z = snapshot->settings.BallRadius() * 1.1 * 20;
            sendAction(action);
            state = CursorMode::STEADY; showCursor = false;
        } else {
            if (selected >= 0) { clicked_robot = selected; selectRobot(); }
            if (kickingball || chiping) {
                action.kind = SimulationActionKind::KickBall;
                action.direction = kickpower;
                action.z = chiping ? kickpower * std::tan(chipAngle) : 0;
                sendAction(action);
            }
        }
    }
    if (event->buttons() & Qt::RightButton) {
        if (selected >= 0) {
            clicked_robot = selected;
            selectRobot();
            const auto* robot = robotSnapshot(selected);
            onOffRobotAct->setText(robot && robot->on ? tr("Turn &off") : tr("Turn &on"));
            robpopup->exec(event->globalPos());
        } else {
            clicked_robot = -1;
            if (selected == -2) ballpopup->exec(event->globalPos());
            else mainpopup->exec(event->globalPos());
        }
    }
}

void GLWidget::wheelEvent(QWheelEvent* event)
{
    if (!graphicsEnabled) return;
    renderer.graphics.zoomCamera(-event->angleDelta().y() * 0.002);
    update3DCursor(event->x(), event->y());
}

namespace {
double intersectPrimitive(const RenderPrimitive& primitive, const dReal* origin, const dReal* direction)
{
    double o[3] = {}, d[3] = {};
    for (int axis = 0; axis < 3; ++axis)
        for (int row = 0; row < 3; ++row) {
            o[axis] += primitive.rotation[row * 4 + axis] * (origin[row] - primitive.position[row]);
            d[axis] += primitive.rotation[row * 4 + axis] * direction[row];
        }
    double best = std::numeric_limits<double>::infinity();
    const double radius = primitive.dimensions[0];
    const bool sphere = primitive.kind == PrimitiveKind::Sphere;
    const double a = d[0]*d[0] + d[1]*d[1] + (sphere ? d[2]*d[2] : 0);
    const double b = o[0]*d[0] + o[1]*d[1] + (sphere ? o[2]*d[2] : 0);
    const double c = o[0]*o[0] + o[1]*o[1] + (sphere ? o[2]*o[2] : 0) - radius*radius;
    const double disc = b*b - a*c;
    if (a > 1e-12 && disc >= 0) {
        for (const double t : {(-b - std::sqrt(disc))/a, (-b + std::sqrt(disc))/a})
            if (t >= 0 && (sphere || std::abs(o[2] + t*d[2]) <= primitive.dimensions[1]/2)) best = qMin(best, t);
    }
    if (!sphere && std::abs(d[2]) > 1e-12) {
        for (const double cap : {-primitive.dimensions[1]/2, primitive.dimensions[1]/2}) {
            const double t = (cap-o[2])/d[2];
            const double x = o[0]+t*d[0], y = o[1]+t*d[1];
            if (t >= 0 && x*x+y*y <= radius*radius) best = qMin(best, t);
        }
    }
    return best;
}
}

void GLWidget::update3DCursor(int mouseX, int mouseY)
{
    if (!graphicsEnabled || !snapshot || !hasRendered || width() <= 0 || height() <= 0) return;
    auto& graphics = renderer.graphics;
    dReal xyz[3], hpr[3], f[3], r[3];
    graphics.getViewpoint(xyz, hpr);
    graphics.getCameraForward(f[0], f[1], f[2]);
    graphics.getCameraRight(r[0], r[1], r[2]);
    dReal u[3] = {r[1]*f[2]-r[2]*f[1], r[2]*f[0]-r[0]*f[2], r[0]*f[1]-r[1]*f[0]};
    dReal xx, yy, near;
    graphics.getFrustum(xx, yy, near);
    const double x = (1 - 2.0*mouseX/width())*xx;
    const double y = (1 - 2.0*mouseY/height())*yy;
    dReal direction[3];
    for (int i = 0; i < 3; ++i) direction[i] = -u[i]*y-r[i]*x-near*f[i];
    if (std::abs(direction[2]) > 1e-12) {
        const double t = -xyz[2]/direction[2];
        if (t >= 0) cursor = {{xyz[0]+t*direction[0], xyz[1]+t*direction[1], 0}};
    }
    selected = -1;
    double nearest = std::numeric_limits<double>::infinity();
    for (const auto& primitive : snapshot->primitives) {
        if (primitive.kind != PrimitiveKind::Sphere && !primitive.robotChassis) continue;
        if (primitive.robotIndex < 0 && !primitive.visible) continue;
        const double t = intersectPrimitive(primitive, xyz, direction);
        if (t < nearest) { nearest = t; selected = primitive.robotIndex < 0 ? -2 : primitive.robotIndex; }
    }
}

void GLWidget::mouseMoveEvent(QMouseEvent* event)
{
    if (!graphicsEnabled) return;
    const int dx = -(event->x()-lastPos.x()), dy = -(event->y()-lastPos.y());
    if (event->buttons() & Qt::LeftButton)
        renderer.graphics.cameraMotion(ctrl ? CameraMotionMode::MOVE_POSITION_FREELY : CameraMotionMode::ROTATE_VIEW_POINT, dx, dy);
    else if (event->buttons() & Qt::MidButton)
        renderer.graphics.cameraMotion(CameraMotionMode::MOVE_POSITION_LR, dx, dy);
    lastPos = event->pos();
    update3DCursor(event->x(), event->y());
}

void GLWidget::mouseReleaseEvent(QMouseEvent*) { emit clicked(); }
void GLWidget::initializeGL() { if (snapshot) renderer.initialize(*snapshot); }
void GLWidget::resizeGL(int width, int height) { glViewport(0, 0, width, height); }
bool GLWidget::hasFramebufferObjects() const { return QGLFramebufferObject::hasOpenGLFramebufferObjects(); }
bool GLWidget::hasFramebufferBlit() const { return QGLFramebufferObject::hasOpenGLFramebufferBlit(); }

void GLWidget::paintGL()
{
    if (!graphicsEnabled) return;
    refreshSnapshot();
    if (!snapshot) return;
    auto& graphics = renderer.graphics;
    if (cammode == CameraMode::CURRENT_ROBOT_VIEW) {
        const auto* robot = robotSnapshot(robotIndex(Current_robot, Current_team));
        if (robot) graphics.setViewpoint(robot->x, robot->y, 0.3, robot->direction, -25, 0);
    } else if (cammode == CameraMode::LOCK_TO_ROBOT) {
        const auto* robot = robotSnapshot(lockedIndex);
        if (robot) graphics.lookAt(robot->x, robot->y, 0.1);
    } else if (cammode == CameraMode::LOCK_TO_BALL) {
        const auto& ball = snapshot->ballPosition;
        graphics.lookAt(ball[0], ball[1], ball[2]);
    }
    const auto quality = RenderQuality::forLowSpec(cfg->LowSpecMode());
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    const int fullW = qMax(1, viewport[2]), fullH = qMax(1, viewport[3]);
    const bool fboSupported = hasFramebufferObjects();
    const bool blitSupported = fboSupported && hasFramebufferBlit();
    GLint target = 0, readTarget = 0;
    if (fboSupported) glGetIntegerv(GL_FRAMEBUFFER_BINDING, &target);
    if (blitSupported) glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &readTarget);
    bool scaled = false;
    if (quality.lowSpec && blitSupported && target == 0) {
        const int w = qMax(1, qRound(fullW*0.75)), h = qMax(1, qRound(fullH*0.75));
        if (!m_fbo || m_fbo->size() != QSize(w, h)) {
            delete m_fbo;
            QGLFramebufferObjectFormat format;
            format.setAttachment(QGLFramebufferObject::Depth);
            m_fbo = new QGLFramebufferObject(w, h, format);
        }
        if (m_fbo->isValid() && m_fbo->bind()) {
            renderer.draw(*snapshot, w, h, quality, selected, showCursor, cursor, cursorRadius);
            m_fbo->release();
            QGLFramebufferObject::blitFramebuffer(nullptr, QRect(0, 0, fullW, fullH), m_fbo, QRect(0, 0, w, h));
            scaled = true;
        }
    }
    if (!scaled) {
        if (fboSupported) QOpenGLContext::currentContext()->functions()->glBindFramebuffer(GL_FRAMEBUFFER, target);
        renderer.draw(*snapshot, fullW, fullH, quality, selected, showCursor, cursor, cursorRadius);
    }
    if (quality.showRobotLabels) {
        QFont font;
        for (const auto& robot : snapshot->robots) {
            qglColor(robot.team ? Qt::yellow : Qt::cyan);
            renderText(robot.x, robot.y, 0.3, QString::number(robot.id), font);
            if (!robot.on) {
                qglColor(Qt::red); font.setBold(true);
                renderText(robot.x, robot.y, 0.4, "Off", font);
                font.setBold(false);
            }
        }
    }
    if (fboSupported) {
        auto* functions = QOpenGLContext::currentContext()->functions();
        functions->glBindFramebuffer(GL_FRAMEBUFFER, target);
        if (blitSupported) functions->glBindFramebuffer(GL_READ_FRAMEBUFFER, readTarget);
    }
    glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
    hasRendered = true;
    emit frameRendered();
}

void GLWidget::changeCameraMode()
{
    if (static_cast<int>(cammode) < 0) cammode = CameraMode::BIRDS_EYE_FROM_TOUCH_LINE;
    cammode = static_cast<CameraMode>((static_cast<int>(cammode)+1) %
        (static_cast<int>(CameraMode::MAX_ACTIVE_MODE_FOR_CHANGEMODE)+1));
    auto& graphics = renderer.graphics;
    if (cammode == CameraMode::BIRDS_EYE_FROM_TOUCH_LINE)
        graphics.setViewpoint(0, -(cfg->Field_Width()+cfg->Field_Margin_Touch_Line()*2)/2, 3, 90, -45, 0);
    else if (cammode == CameraMode::TOP_VIEW) graphics.setViewpoint(0, 0, 5, 0, -90, 0);
    else if (cammode == CameraMode::BIRDS_EYE_FROM_OPPOSITE_TOUCH_LINE)
        graphics.setViewpoint(0, (cfg->Field_Width()+cfg->Field_Margin_Touch_Line()*2)/2, 3, 270, -45, 0);
    else if (cammode == CameraMode::BIRDS_EYE_FROM_BLUE)
        graphics.setViewpoint(-(cfg->Field_Length()+cfg->Field_Margin_Goal_Line()*2)/2, 0, 3, 0, -45, 0);
    else if (cammode == CameraMode::BIRDS_EYE_FROM_YELLOW)
        graphics.setViewpoint((cfg->Field_Length()+cfg->Field_Margin_Goal_Line()*2)/2, 0, 3, 180, -45, 0);
}

void GLWidget::putBall(dReal x, dReal y)
{
    SimulationAction action;
    action.kind = SimulationActionKind::PlaceBall;
    action.x = x; action.y = y; action.z = 0.3;
    sendAction(action);
}

void GLWidget::keyReleaseEvent(QKeyEvent* event)
{
    if (event->key() == Qt::Key_Control) ctrl = false;
    if (event->key() == Qt::Key_Alt) {
        if (!showCursor) moveCurrentRobot(); else unselectRobot();
        alt = false;
    }
}

void GLWidget::keyPressEvent(QKeyEvent* event)
{
    if (event->key() == Qt::Key_Control) ctrl = true;
    if (event->key() == Qt::Key_Alt) alt = true;
    SimulationAction action;
    action.robot = robotIndex(Current_robot, Current_team);
    switch (event->key()) {
    case Qt::Key_T: action.kind = SimulationActionKind::WheelDelta; action.wheels = {{-1, 1, -1, 1}}; break;
    case Qt::Key_G: action.kind = SimulationActionKind::WheelDelta; action.wheels = {{1, -1, 1, -1}}; break;
    case Qt::Key_F: action.kind = SimulationActionKind::WheelDelta; action.wheels = {{1, 1, 1, 1}}; break;
    case Qt::Key_H: action.kind = SimulationActionKind::WheelDelta; action.wheels = {{-1, -1, -1, -1}}; break;
    case Qt::Key_W: action.kind = SimulationActionKind::BallForce; action.y = 2; break;
    case Qt::Key_S: action.kind = SimulationActionKind::BallForce; action.y = -2; break;
    case Qt::Key_D: action.kind = SimulationActionKind::BallForce; action.x = 2; break;
    case Qt::Key_A: action.kind = SimulationActionKind::BallForce; action.x = -2; break;
    case Qt::Key_K: action.kind = SimulationActionKind::RobotKick; action.x = 8; break;
    case Qt::Key_L: action.kind = SimulationActionKind::RobotKick; action.x = 3; action.z = 3; break;
    case Qt::Key_J: action.kind = SimulationActionKind::ToggleRoller; break;
    case Qt::Key_I: action.kind = SimulationActionKind::SetBallVelocity; action.x = 2; break;
    case Qt::Key_Space: action.kind = SimulationActionKind::ResetSpeeds; break;
    case Qt::Key_QuoteLeft: action.kind = SimulationActionKind::StopBall; break;
    case Qt::Key_Semicolon:
        kickingball = !kickingball; chiping = false;
        logStatus(kickingball ? "Kick mode On" : "Kick mode Off", QColor(kickingball ? "blue" : "red"));
        return;
    case Qt::Key_Apostrophe:
        chiping = !chiping; kickingball = false;
        logStatus(chiping ? "Chip mode On" : "Chip mode Off", QColor(chiping ? "blue" : "red"));
        return;
    case Qt::Key_BracketRight: kickpower += 0.1; logStatus(QString("Kick power = %1").arg(kickpower), QColor("orange")); return;
    case Qt::Key_BracketLeft: kickpower -= 0.1; logStatus(QString("Kick power = %1").arg(kickpower), QColor("cyan")); return;
    default: return;
    }
    sendAction(action);
}

void GLWidget::closeEvent(QCloseEvent*) { emit closeSignal(false); }
void GLWidget::blueRobotsMenuTriggered(QAction* action) { reform(0, action->text()); }
void GLWidget::yellowRobotsMenuTriggered(QAction* action) { reform(1, action->text()); }

void GLWidget::reform(int team, const QString& text)
{
    SimulationAction action;
    action.team = team;
    action.kind = SimulationActionKind::Formation;
    if (text == tr("Put all inside with formation 1")) action.formation = 1;
    else if (text == tr("Put all inside with formation 2")) action.formation = 2;
    else if (text == tr("Put all outside")) action.formation = 0;
    else if (text == tr("Put all out of field")) action.formation = 3;
    else if (text == tr("Turn all off") || text == tr("Turn all on")) {
        action.kind = SimulationActionKind::EnableTeam;
        action.enabled = text == tr("Turn all on");
    } else return;
    sendAction(action);
}

void GLWidget::moveBallHere()
{
    SimulationAction action;
    action.kind = SimulationActionKind::PlaceBall;
    action.x = cursor[0]; action.y = cursor[1];
    action.z = snapshot ? snapshot->settings.BallRadius()*2 : 0.3;
    sendAction(action);
}
void GLWidget::lockCameraToRobot() { cammode = CameraMode::LOCK_TO_ROBOT; lockedIndex = robotIndex(Current_robot, Current_team); }
void GLWidget::lockCameraToBall() { cammode = CameraMode::LOCK_TO_BALL; }
void GLWidget::moveRobotHere()
{
    SimulationAction action;
    action.kind = SimulationActionKind::PlaceRobot;
    action.robot = robotIndex(Current_robot, Current_team);
    action.x = cursor[0]; action.y = cursor[1]; action.reset = true;
    const auto* robot = robotSnapshot(action.robot);
    action.direction = robot && robot->team ? 180 : 0;
    sendAction(action);
}
