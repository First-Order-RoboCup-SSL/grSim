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

#include <QtGui>

#include <QApplication>
#include <QMenuBar>
#include <QGroupBox>
#include <QGridLayout>
#include <QSlider>
#include <QTimer>
#include <QToolBar>
#include <QDockWidget>
#include <QVBoxLayout>
#include <QFileDialog>
#include <QApplication>
#include <QDir>
#include <QClipboard>
#include <QShortcut>

#include <QStatusBar>
#include <QMessageBox>
#include <cmath>

#include "mainwindow.h"
#include "config.h"
#include "logger.h"

void MainWindow::customFPS(int fps)
{
    Q_UNUSED(fps);
    changeTimer();
    logStatus(QString("new FPS set by user: %1").arg(fps),"red");
}

MainWindow::MainWindow(QWidget *parent, ConfigWidget *config)
    : QMainWindow(parent)
{
    QDir dir = qApp->applicationDirPath();
    dir.cdUp();
    current_dir = dir.path();
    /* Status Logger */
    printer = new CStatusPrinter(this);
    statusWidget = new CStatusWidget(printer);
    statusWidget->setObjectName("CStatusWidget");
    initLogger((void*)printer);

    /* Widgets */

    configwidget = config ? config : new ConfigWidget();
    dockconfig = new ConfigDockWidget(this,configwidget);
    dockconfig->setObjectName("ConfigDockWidget");
    dockconfig->setWindowTitle("Configuration");

    simulation = new SimulationController(configwidget->simulationSettings(), this);
    connect(simulation, &SimulationController::remoteSettingsChanged,
            configwidget, &ConfigWidget::applyRemoteSettings);
    glwidget = new GLWidget(this,configwidget,simulation);
    glwidget->setWindowTitle(tr("Simulator"));
    glwidget->resize(512,512);    

    robotwidget = new RobotWidget(this, configwidget);
    robotwidget->setObjectName("RobotWidget");
    /* Status Bar */
    fpslabel = new QLabel(this);
    cursorlabel = new QLabel(this);
    selectinglabel = new QLabel(this);
    vanishlabel = new QLabel("Vanishing",this);
    noiselabel = new QLabel("Gaussian noise",this);
    fpslabel->setFrameStyle(QFrame::Panel);
    cursorlabel->setFrameStyle(QFrame::Panel);
    selectinglabel->setFrameStyle(QFrame::Panel);
    vanishlabel->setFrameStyle(QFrame::Panel);
    noiselabel->setFrameStyle(QFrame::Panel);
    statusBar()->addWidget(fpslabel);
    statusBar()->addWidget(cursorlabel);
    statusBar()->addWidget(selectinglabel);
    statusBar()->addWidget(vanishlabel);
    statusBar()->addWidget(noiselabel);
    graphicsDeviceLabel = new QLabel(tr("Graphics: initializing..."), this);
    graphicsDeviceLabel->setObjectName("graphicsDeviceStatus");
    graphicsDeviceLabel->setTextFormat(Qt::PlainText);
    graphicsDeviceLabel->setMinimumWidth(150);
    graphicsDeviceLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    statusBar()->addPermanentWidget(graphicsDeviceLabel, 1);
    auto* physicsDeviceLabel = new QLabel(tr("Physics: CPU"), this);
    physicsDeviceLabel->setObjectName("physicsProcessorStatus");
    physicsDeviceLabel->setToolTip(tr("Physics runs on the CPU, separately from rendering."));
    statusBar()->addPermanentWidget(physicsDeviceLabel);

    /* Menus */

    QMenu *fileMenu = new QMenu("&File");
    menuBar()->addMenu(fileMenu);    
    QAction *takeSnapshotAct = new QAction("&Save snapshot to file", fileMenu);
    takeSnapshotAct->setShortcut(QKeySequence("F3"));
    QAction *takeSnapshotToClipboardAct = new QAction("&Copy snapshot to clipboard", fileMenu);
    takeSnapshotToClipboardAct->setShortcut(QKeySequence("F4"));    
    QAction *exit = new QAction("E&xit", fileMenu);
    exit->setShortcut(QKeySequence("Ctrl+X"));
    fileMenu->addAction(takeSnapshotAct);
    fileMenu->addAction(takeSnapshotToClipboardAct);
    fileMenu->addAction(exit);

    QObject::connect(new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_Q), this),
                     &QShortcut::activated,
                     this,
                     &MainWindow::close);

    QMenu *viewMenu = new QMenu("&View");
    menuBar()->addMenu(viewMenu);
    showsimulator = new QAction("&Simulator", viewMenu);
    showsimulator->setCheckable(true);
    showsimulator->setChecked(true);
    viewMenu->addAction(showsimulator);
    showconfig = dockconfig->toggleViewAction();
    viewMenu->addAction(showconfig);

    QMenu *simulatorMenu = new QMenu("&Simulator");
    menuBar()->addMenu(simulatorMenu);
    QMenu *robotMenu = new QMenu("&Robot");
    QMenu *ballMenu = new QMenu("&Ball");
    simulatorMenu->addMenu(robotMenu);
    simulatorMenu->addMenu(ballMenu);

    QMenu *helpMenu = new QMenu("&Help");
    QAction* aboutMenu = new QAction("&About", helpMenu);
    menuBar()->addMenu(helpMenu);
    helpMenu->addAction(aboutMenu);

    ballMenu->addAction(tr("Put on Center"))->setShortcut(QKeySequence("Ctrl+0"));
    ballMenu->addAction(tr("Put on Corner 1"))->setShortcut(QKeySequence("Ctrl+1"));
    ballMenu->addAction(tr("Put on Corner 2"))->setShortcut(QKeySequence("Ctrl+2"));
    ballMenu->addAction(tr("Put on Corner 3"))->setShortcut(QKeySequence("Ctrl+3"));
    ballMenu->addAction(tr("Put on Corner 4"))->setShortcut(QKeySequence("Ctrl+4"));
    ballMenu->addAction(tr("Put on Penalty 1"))->setShortcut(QKeySequence("Ctrl+5"));
    ballMenu->addAction(tr("Put on Penalty 2"))->setShortcut(QKeySequence("Ctrl+6"));

    robotMenu->addMenu(glwidget->blueRobotsMenu);
    robotMenu->addMenu(glwidget->yellowRobotsMenu);
    robotMenu->addMenu(glwidget->allRobotsMenu);

    fullScreenAct = new QAction(tr("&Full screen"),simulatorMenu);
    fullScreenAct->setShortcut(QKeySequence("F2"));
    fullScreenAct->setCheckable(true);
    fullScreenAct->setChecked(false);
    simulatorMenu->addAction(fullScreenAct);

    showrobot = robotwidget->toggleViewAction();
    viewMenu->addAction(showrobot);
    viewMenu->addMenu(glwidget->cameraMenu);

    addDockWidget(Qt::LeftDockWidgetArea,dockconfig);
    addDockWidget(Qt::BottomDockWidgetArea, statusWidget);
    addDockWidget(Qt::LeftDockWidgetArea, robotwidget);

    setCentralWidget(glwidget);

    renderTimer = new QTimer(this);
    renderTimer->setTimerType(Qt::PreciseTimer);
    renderTimer->setSingleShot(true);
    renderClock.start();
    framePacer.reset(renderClock.nsecsElapsed(), configwidget->DesiredFPS());

    uiTimer = new QTimer(this);
    uiTimer->setInterval(33);

    m_simHz = 0;
    m_renderFps = 0;
    m_renderFramesThisSecond = 0;
    m_rateTimer.start();

    QObject::connect(renderTimer, SIGNAL(timeout()), this, SLOT(renderTick()));
    QObject::connect(glwidget, SIGNAL(frameSubmitted()), this, SLOT(recordRenderedFrame()));
    QObject::connect(uiTimer, SIGNAL(timeout()), this, SLOT(updateUI()));
    QObject::connect(takeSnapshotAct, SIGNAL(triggered(bool)), this, SLOT(takeSnapshot()));
    QObject::connect(takeSnapshotToClipboardAct, SIGNAL(triggered(bool)), this, SLOT(takeSnapshotToClipboard()));
    QObject::connect(exit, SIGNAL(triggered(bool)), this, SLOT(close()));
    QObject::connect(showsimulator, SIGNAL(triggered(bool)), this, SLOT(showHideSimulator(bool)));
    QObject::connect(glwidget, SIGNAL(closeSignal(bool)), this, SLOT(showHideSimulator(bool)));    
    QObject::connect(glwidget, SIGNAL(selectedRobot()), this, SLOT(updateRobotLabel()));
    QObject::connect(robotwidget->robotCombo,SIGNAL(currentIndexChanged(int)),this,SLOT(changeCurrentRobot()));
    QObject::connect(robotwidget->teamCombo,SIGNAL(currentIndexChanged(int)),this,SLOT(changeCurrentTeam()));
    QObject::connect(robotwidget->resetBtn,SIGNAL(clicked()),glwidget,SLOT(resetCurrentRobot()));
    QObject::connect(robotwidget->locateBtn,SIGNAL(clicked()),glwidget,SLOT(moveCurrentRobot()));
    QObject::connect(robotwidget->onOffBtn,SIGNAL(clicked()),glwidget,SLOT(switchRobotOnOff()));
    QObject::connect(robotwidget->getPoseWidget->okBtn,SIGNAL(clicked()),this,SLOT(setCurrentRobotPosition()));
    QObject::connect(glwidget,SIGNAL(robotTurnedOnOff(int,bool)),robotwidget,SLOT(changeRobotOnOff(int,bool)));
    QObject::connect(ballMenu,SIGNAL(triggered(QAction*)),this,SLOT(ballMenuTriggered(QAction*)));
    QObject::connect(fullScreenAct,SIGNAL(triggered(bool)),this,SLOT(toggleFullScreen(bool)));
    QObject::connect(aboutMenu, SIGNAL(triggered()), this, SLOT(showAbout()));
    connect(configwidget, &ConfigWidget::settingsEdited, this, [this]() {
        changeTimer();
        if (configwidget->Robots_Count() != robotwidget->robotCombo->count()) changeRobotCount();
        else simulation->configure(configwidget->simulationSettings());
    });
    renderTimer->start(framePacer.delayMilliseconds(renderClock.nsecsElapsed()));
    uiTimer->start();


    this->setWindowTitle("grSim");

    robotwidget->teamCombo->setCurrentIndex(0);
    robotwidget->robotCombo->setCurrentIndex(0);
    changeCurrentRobot();
    robotwidget->id = 0;
}

MainWindow::~MainWindow()
{
    renderTimer->stop();
    uiTimer->stop();
    simulation->stop();
    initLogger(nullptr);
    delete glwidget;
}

void MainWindow::showHideSimulator(bool v)
{
    if (v) {glwidget->show();showsimulator->setChecked(true);}
    else {glwidget->hide();showsimulator->setChecked(false);}
}

void MainWindow::changeCurrentRobot()
{
    glwidget->Current_robot = robotwidget->robotCombo->currentIndex();
    glwidget->refreshSnapshot();
    const int index = robotIndex(glwidget->Current_robot, glwidget->Current_team);
    robotwidget->id = index;
    const auto* robot = glwidget->robotSnapshot(index);
    if (!robot) return;
    QImage image = glwidget->robotImage(index);
    robotwidget->setPicture(&image);
    robotwidget->changeRobotOnOff(index, robot->on);
}

void MainWindow::changeCurrentTeam()
{
    glwidget->Current_team = robotwidget->teamCombo->currentIndex();
    changeCurrentRobot();
}

void MainWindow::changeRobotCount()
{
    auto newRobotCount = configwidget->Robots_Count();

    if(newRobotCount < 0 || newRobotCount > MAX_ROBOT_COUNT) {
        newRobotCount = std::min(std::max(0, newRobotCount), static_cast<int>(MAX_ROBOT_COUNT));
        configwidget->v_Robots_Count->setInt(newRobotCount);
    }

    robotwidget->changeRobotCount(newRobotCount);
    
    const int newCurrentRobot = newRobotCount ? qBound(0, glwidget->Current_robot, newRobotCount - 1) : -1;
    robotwidget->changeCurrentRobot(newCurrentRobot);
    restartSimulator();
    changeCurrentRobot();
}

void MainWindow::changeGravity()
{
    simulation->configure(configwidget->simulationSettings());
}

int MainWindow::robotIndex(int robot, int team)
{
    return glwidget->robotIndex(robot, team);
}

void MainWindow::changeTimer()
{
    framePacer.reset(renderClock.nsecsElapsed(), configwidget->DesiredFPS());
    renderTimer->start(framePacer.delayMilliseconds(renderClock.nsecsElapsed()));
}

void MainWindow::changePhysicsTimer()
{
    simulation->configure(configwidget->simulationSettings());
}

QString dRealToStr(dReal a)
{
    QString s;
    s.setNum(a,'f',3);
    if (a>=0) return QString('+') + s;
    return s;
}



void MainWindow::renderTick()
{
    const qint64 now = renderClock.nsecsElapsed();
    if (framePacer.due(now)) {
        framePacer.advance(now);
        // The mailbox contains completed immutable states. Physics backlog
        // changes their age, not their safety to draw.
        if (glwidget->graphicsEnabled && glwidget->isVisible() && simulation->latest())
            glwidget->update();
    }
    renderTimer->start(framePacer.delayMilliseconds(renderClock.nsecsElapsed()));
}

void MainWindow::recordRenderedFrame()
{
    ++m_renderFramesThisSecond;
}

void MainWindow::updateGraphicsDeviceStatus()
{
    const QString renderer = glwidget->graphicsRendererName();
    QString label;
    if (!glwidget->graphicsEnabled) label = tr("Graphics: disabled");
    else if (renderer.isEmpty()) label = tr("Graphics: initializing...");
    else if (glwidget->usesSoftwareRendering()) label = tr("Graphics: Software (CPU)");
    else label = tr("Graphics: %1").arg(renderer);
    graphicsDeviceLabel->setText(graphicsDeviceLabel->fontMetrics().elidedText(
        label, Qt::ElideMiddle, graphicsDeviceLabel->width()));
    graphicsDeviceLabel->setToolTip(
        tr("Renderer: %1\nVendor: %2\nOpenGL: %3\n%4")
            .arg(renderer.isEmpty() ? tr("Not initialized") : renderer,
                 glwidget->graphicsRendererVendor(), glwidget->graphicsRendererVersion(),
                 glwidget->usesSoftwareRendering()
                     ? tr("Software rendering and physics run separately on the CPU.")
                     : tr("Physics runs on the CPU, separately from rendering.")));
}

void MainWindow::updateUI()
{
    updateGraphicsDeviceStatus();
    const auto snapshot = glwidget->refreshSnapshot();
    if (!snapshot) return;
    const int index = robotIndex(glwidget->Current_robot, glwidget->Current_team);
    robotwidget->id = index;
    const auto* robot = glwidget->robotSnapshot(index);
    if (robot) {
        if (previousGeneration != snapshot->generation || previousRobot != index) {
            QImage image = glwidget->robotImage(index);
            robotwidget->setPicture(&image);
        }
        double velocity = 0, acceleration = 0;
        const double dt = snapshot->simulationTime - previousSimTime;
        for (int i = 0; i < 3; ++i) {
            velocity += robot->velocity[i] * robot->velocity[i];
            if (dt > 0 && previousGeneration == snapshot->generation && previousRobot == index) {
                const double a = (robot->velocity[i] - previousVelocity[i]) / dt;
                acceleration += a * a;
            }
        }
        robotwidget->vellabel->setText(QString::number(std::sqrt(velocity), 'f', 3));
        robotwidget->acclabel->setText(QString::number(std::sqrt(acceleration), 'f', 3));
        robotwidget->changeRobotOnOff(index, robot->on);
        previousVelocity = robot->velocity;
    }
    previousSimTime = snapshot->simulationTime;
    previousGeneration = snapshot->generation;
    previousRobot = index;
    m_simHz = snapshot->physicsHz;
    const double elapsed = m_rateTimer.nsecsElapsed() / 1e9;
    if (elapsed >= 1) {
        m_renderFps = m_renderFramesThisSecond / elapsed;
        m_renderFramesThisSecond = 0;
        m_rateTimer.restart();
        const auto timings = glwidget->frameTimings();
        const auto gpu = timings.gpu.samples
            ? tr("%1 ms").arg(timings.gpu.p95Ms, 0, 'f', 2)
            : (timings.gpuRequested ? tr("unavailable or pending") : tr("profiling disabled"));
        fpslabel->setToolTip(tr(
            "Submitted frames; display presentation is controlled by the compositor.\n"
            "Frame interval p50 / p95 / p99: %1 / %2 / %3 ms\n"
            "Scene / labels / paint + swap p95: %4 / %5 / %6 ms\n"
            "GPU drawing p95: %7\n"
            "Simulation tick p95 / p99: %8 / %9 ms; snapshot: %10 ms\n"
            "Physics backlog: %11 ms; ODE helpers: %12")
            .arg(timings.interval.medianMs, 0, 'f', 2)
            .arg(timings.interval.p95Ms, 0, 'f', 2)
            .arg(timings.interval.p99Ms, 0, 'f', 2)
            .arg(timings.scene.p95Ms, 0, 'f', 2)
            .arg(timings.labels.p95Ms, 0, 'f', 2)
            .arg(timings.paintAndSwap.p95Ms, 0, 'f', 2)
            .arg(gpu)
            .arg(snapshot->stepTimings.p95Ms, 0, 'f', 2)
            .arg(snapshot->stepTimings.p99Ms, 0, 'f', 2)
            .arg(snapshot->snapshotMs, 0, 'f', 2)
            .arg(snapshot->pendingTime * 1000, 0, 'f', 2)
            .arg(snapshot->odeHelpers));
    }
    fpslabel->setText(QString("Render: %1 fps | Sim: %2 Hz")
        .arg(QString::asprintf("%06.2f", m_renderFps))
        .arg(QString::asprintf("%06.2f", m_simHz)));
    const int selected = glwidget->selected;
    selectinglabel->setVisible(selected != -1);
    if (selected == -2) selectinglabel->setText("Ball");
    else if (const auto* selectedRobot = glwidget->robotSnapshot(selected))
        selectinglabel->setText(QString("%1:%2").arg(selectedRobot->id).arg(selectedRobot->team ? "Yellow" : "Blue"));
    vanishlabel->setVisible(configwidget->vanishing());
    noiselabel->setVisible(configwidget->noise());
    cursorlabel->setText(QString("Cursor: [X=%1;Y=%2;Z=%3]")
        .arg(dRealToStr(glwidget->cursor[0])).arg(dRealToStr(glwidget->cursor[1])).arg(dRealToStr(glwidget->cursor[2])));
    statusWidget->update();
}

void MainWindow::updateRobotLabel()
{
    robotwidget->teamCombo->setCurrentIndex(glwidget->Current_team);
    robotwidget->robotCombo->setCurrentIndex(glwidget->Current_robot);
    changeCurrentRobot();
}


void MainWindow::changeBallMass()
{
    simulation->configure(configwidget->simulationSettings());
}


void MainWindow::changeBallGroundSurface()
{
    simulation->configure(configwidget->simulationSettings());
}

void MainWindow::changeBallDamping()
{
    simulation->configure(configwidget->simulationSettings());
}

void MainWindow::restartSimulator()
{
    simulation->configure(configwidget->simulationSettings(), true);
}

void MainWindow::ballMenuTriggered(QAction* act)
{
    dReal l = configwidget->Field_Length()/2.0;
    dReal w = configwidget->Field_Width()/2.0;
    dReal p = l - configwidget->Field_Penalty_Point();
    if (act->text()==tr("Put on Center")) glwidget->putBall(0,0);
    else if (act->text()==tr("Put on Corner 1")) glwidget->putBall(-l,-w);
    else if (act->text()==tr("Put on Corner 2")) glwidget->putBall(-l, w);
    else if (act->text()==tr("Put on Corner 3")) glwidget->putBall( l,-w);
    else if (act->text()==tr("Put on Corner 4")) glwidget->putBall( l, w);
    else if (act->text()==tr("Put on Penalty 1")) glwidget->putBall( p, 0);
    else if (act->text()==tr("Put on Penalty 2")) glwidget->putBall(-p, 0);
}

void MainWindow::toggleFullScreen(bool a)
{
    if (a)
    {
        prevState = saveState();
        for (auto dockwidget : findChildren<QDockWidget*>())
        {
            dockwidget->hide();
        }
        showconfig->setEnabled(false);
        showrobot->setEnabled(false);
        showFullScreen();
    }
    else {
        showNormal();
        restoreState(prevState);
        showconfig->setEnabled(true);
        showrobot->setEnabled(true);
    }
}

void MainWindow::setCurrentRobotPosition()
{
    bool okX = false, okY = false, okAngle = false;
    SimulationAction action;
    action.kind = SimulationActionKind::PlaceRobot;
    action.robot = robotIndex(glwidget->Current_robot, glwidget->Current_team);
    action.x = robotwidget->getPoseWidget->x->text().toDouble(&okX);
    action.y = robotwidget->getPoseWidget->y->text().toDouble(&okY);
    action.direction = robotwidget->getPoseWidget->a->text().toDouble(&okAngle);
    action.reset = true;
    if (!okX || !okY || !okAngle) { logStatus("Invalid robot position", QColor("red")); return; }
    glwidget->sendAction(action);
    robotwidget->getPoseWidget->close();
}

void MainWindow::takeSnapshot()
{
    QPixmap p(glwidget->renderPixmap(glwidget->size().width(),glwidget->size().height(),false));
    p.save(QFileDialog::getSaveFileName(this, tr("Save Snapshot"), current_dir, tr("Images (*.png)")),"PNG",100);
}

void MainWindow::takeSnapshotToClipboard()
{
    QPixmap p(glwidget->renderPixmap(glwidget->size().width(),glwidget->size().height(),false));
    QClipboard* b = QApplication::clipboard();
    b->setPixmap(p);
}

void MainWindow::showAbout()
{
    QString title = QString("grSim v0.9 - Build r1240");
    QString text = QString("grSim - RoboCup Small Size Soccer Robots Simulator\n\n(C) 2011 - Parsian Robotic Center\nhttp://eew.aut.ac.ir/~parsian/grsim\n\ngrSim is free software released under the terms of GNU GPL v3");
    QMessageBox::about(this, title, text);
}



void MainWindow::reconnectBlueStatusSocket()
{
    simulation->configure(configwidget->simulationSettings());
}

void MainWindow::reconnectYellowStatusSocket()
{
    simulation->configure(configwidget->simulationSettings());
}

void MainWindow::reconnectCommandSocket()
{
    simulation->configure(configwidget->simulationSettings());
}

void MainWindow::reconnectSimControlSocket()
{
    simulation->configure(configwidget->simulationSettings());
}

void MainWindow::reconnectBlueControlSocket()
{
    simulation->configure(configwidget->simulationSettings());
}

void MainWindow::reconnectYellowControlSocket()
{
    simulation->configure(configwidget->simulationSettings());
}

void MainWindow::reconnectVisionSocket()
{
    simulation->configure(configwidget->simulationSettings());
}









void MainWindow::setIsGlEnabled(bool value)
{
    glwidget->graphicsEnabled = value;
}
