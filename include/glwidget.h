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

#ifndef GLWIDGET_H
#define GLWIDGET_H

#define GL_SILENCE_DEPRECATION
#include <QGLWidget>
#include <QGLFramebufferObject>
#include <QOpenGLTimerQuery>
#include <QOpenGLContext>
#include <QPointer>
#include <memory>
#include "timingstats.h"
#include <QLabel>
#include <QFrame>
#include "simulation.h"
#include "scenerenderer.h"
#include <QMenu>

#include "configwidget.h"


class GLWidget : public QGLWidget {

    Q_OBJECT
public:
    enum class CursorMode {
        STEADY = 0,
        PLACE_ROBOT = 1,
        PLACE_BALL = 2,
    };
    enum class CameraMode {
        BIRDS_EYE_FROM_TOUCH_LINE = 0,
        CURRENT_ROBOT_VIEW = 1,
        TOP_VIEW = 2,
        BIRDS_EYE_FROM_OPPOSITE_TOUCH_LINE = 3,
        BIRDS_EYE_FROM_BLUE = 4,
        BIRDS_EYE_FROM_YELLOW = 5,
        MAX_ACTIVE_MODE_FOR_CHANGEMODE=BIRDS_EYE_FROM_YELLOW,
        // non-avaliable modes when "toggle camera mode" called
        LOCK_TO_ROBOT = -1,
        LOCK_TO_BALL = -2,
    };
    GLWidget(QWidget *parent, ConfigWidget* cfg, SimulationController* simulation);
    ~GLWidget();
    ConfigWidget* cfg;   
    SimulationController* simulation;
    SceneSnapshotPtr scene() const { return simulation->latest(); }
    SceneSnapshotPtr refreshSnapshot();
    int robotIndex(int id, int team) const;
    const RobotSnapshot* robotSnapshot(int index) const;
    QImage robotImage(int index) const;
    void sendAction(SimulationAction action);
    QString graphicsRendererName() const { return graphicsRenderer; }
    QString graphicsRendererVendor() const { return graphicsVendor; }
    QString graphicsRendererVersion() const { return graphicsVersion; }
    bool usesSoftwareRendering() const { return softwareRendering; }
    bool graphicsEnabled = true;
    int selected = -1;
    std::array<double, 3> cursor{{0, 0, 0}};
    QMenu* robpopup,*ballpopup,*mainpopup;
    QMenu *blueRobotsMenu,*yellowRobotsMenu,*allRobotsMenu;
    QAction* moveRobotAct;
    QAction* selectRobotAct;
    QAction* resetRobotAct;
    QAction* moveBallAct;        
    QAction* lockToBallAct;
    QAction* onOffRobotAct;
    QAction* lockToRobotAct;
    QAction* moveBallHereAct;
    QAction* moveRobotHereAct;
    QAction* changeCamModeAct;
    QMenu *cameraMenu;
    int Current_robot,Current_team;
    int lockedIndex;
    FrameTimings frameTimings() const;
    bool ctrl,alt,kickingball,altTrigger;
    bool chiping;
    double kickpower, chipAngle;
    void update3DCursor(int mouse_x,int mouse_y);
    void putBall(dReal x,dReal y);
    void reform(int team,const QString& act);    
public slots:
    void moveRobot();
    void selectRobot();
    void unselectRobot();    
    void moveCurrentRobot();
    void resetCurrentRobot();
    void moveBall();
    void changeCameraMode();
    void yellowRobotsMenuTriggered(QAction* act);
    void blueRobotsMenuTriggered(QAction* act);
    void switchRobotOnOff();
    void moveRobotHere();
    void moveBallHere();
    void lockCameraToRobot();
    void lockCameraToBall();
signals:
    void frameRendered();
    void frameSubmitted();
    void clicked();
    void selectedRobot();
    void closeSignal(bool);
    void robotTurnedOnOff(int,bool);
protected:
    void paintEvent(QPaintEvent* event) override;
    virtual bool hasFramebufferObjects() const;
    virtual bool hasFramebufferBlit() const;
    void resizeEvent(QResizeEvent* event) override;
    void resizeGL(int width, int height) override;
    void paintGL ();
    void initializeGL ();        

    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);    
    void wheelEvent(QWheelEvent *event);
    void keyPressEvent(QKeyEvent *event);
    void keyReleaseEvent(QKeyEvent* event);
    void closeEvent(QCloseEvent *event);    
private:
    CursorMode state;
    CameraMode cammode;
    int moving_robot_id,clicked_robot;
    SceneSnapshotPtr snapshot;
    SceneRenderer renderer;
    QString graphicsRenderer, graphicsVendor, graphicsVersion;
    bool softwareRendering = false;
    TimingWindow intervalTimings, sceneTimings, labelTimings, paintTimings, gpuTimings;
    QElapsedTimer frameClock;
    qint64 previousFrame = -1;
    quint64 submittedFrames = 0;
    bool gpuTimingRequested = false, gpuTimingSupported = false;
    bool gpuTimingInitialized = false, paintingWindow = false;
    QPointer<QOpenGLContext> gpuContext;
    QPointer<QObject> gpuSurfaceOwner;
    QSurface* gpuSurface = nullptr;
    QMetaObject::Connection gpuCleanupConnection;
    void initializeGpuTiming();
    void cleanupGpuTiming();
    std::array<std::unique_ptr<QOpenGLTimerQuery>, 4> gpuQueries;
    std::array<bool, 4> gpuPending{{false, false, false, false}};
    int activeGpuQuery = -1;

    QWidget* lagWarning;
    bool showCursor = false;
    double cursorRadius = 0;
    bool hasRendered = false;
    QPoint lastPos;

    QGLFramebufferObject* m_fbo;
};

#endif // WIDGET_H
