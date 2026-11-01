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

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMdiArea>
#include <QElapsedTimer>

#include <QLabel>

#include "glwidget.h"
#include "configwidget.h"
#include "statuswidget.h"
#include "robotwidget.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0, ConfigWidget *config = nullptr);
    ~MainWindow() override;
public slots:
    void renderTick();
    void recordRenderedFrame();
    void updateUI();
    void updateRobotLabel();
    void showHideSimulator(bool v);
    void changeCurrentRobot();
    void changeCurrentTeam();

    void changeRobotCount();

    void changeBallMass();   
    void changeBallGroundSurface();    
    void changeBallDamping();
    void changeGravity();
    void changeTimer();
    void changePhysicsTimer();

    void restartSimulator();
    void ballMenuTriggered(QAction* act);
    void toggleFullScreen(bool);
    void setCurrentRobotPosition();
    void takeSnapshot();
    void takeSnapshotToClipboard();

    void customFPS(int fps);
    void showAbout();
    void reconnectCommandSocket();
    void reconnectYellowStatusSocket();
    void reconnectBlueStatusSocket();
    void reconnectVisionSocket();
    void reconnectSimControlSocket();
    void reconnectBlueControlSocket();
    void reconnectYellowControlSocket();
    void setIsGlEnabled(bool value);

    int robotIndex(int robot,int team);
private:
    int getRenderInterval();
    void updateGraphicsDeviceStatus();
    SimulationController* simulation;
    QTimer *renderTimer;
    QTimer *uiTimer;
    GLWidget *glwidget;

    QElapsedTimer m_rateTimer;
    dReal m_simHz;
    dReal m_renderFps;
    int m_renderFramesThisSecond;
    ConfigWidget *configwidget;
    ConfigDockWidget *dockconfig;
    RobotWidget *robotwidget;        
    QByteArray prevState;

    CStatusPrinter *printer;
    CStatusWidget *statusWidget;

    QAction *showsimulator, *showconfig, *showrobot;
    QAction* fullScreenAct;
    QLabel *fpslabel,*cursorlabel,*selectinglabel,*vanishlabel,*noiselabel;
    QLabel* graphicsDeviceLabel;
    QString current_dir;

    std::array<double, 3> previousVelocity{{0, 0, 0}};
    double previousSimTime = 0;
    quint64 previousGeneration = 0;
    int previousRobot = -1;

};

#endif // MAINWINDOW_H
