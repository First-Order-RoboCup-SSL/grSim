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

#ifndef SSLWORLD_H
#define SSLWORLD_H


#include <QObject>
#include <QElapsedTimer>
#include <QUdpSocket>
#include <QList>


#include "physics/pworld.h"
#include "physics/pball.h"
#include "physics/pground.h"
#include "physics/pfixedbox.h"
#include "physics/pray.h"

#include "net/robocup_ssl_server.h"

#include "robot.h"
#include "simulationsettings.h"

#include "config.h"

#include "grSim_Robotstatus.pb.h"
#include "ssl_simulation_config.pb.h"
#include "ssl_simulation_control.pb.h"
#include "ssl_simulation_robot_control.pb.h"
#include "ssl_simulation_robot_feedback.pb.h"

#define WALL_COUNT 10

class RobotsFormation;
// Alternate values support controlled offline experiments without changing
// networking or personal settings. Legacy friction is only a comparison mode.
struct PhysicsStepOptions {
    unsigned substeps = 5;
    unsigned quickStepIterations = 0;
    bool legacyBallFriction = false;
    bool finiteWheelRotation = false;
};

class SendingPacket {
    public:
    SendingPacket(SSL_WrapperPacket* _packet,int _t);
    SSL_WrapperPacket* packet;
    int t;
};

class SSLWorld : public QObject
{
    Q_OBJECT
private:
    int frame_num;
    dReal last_dt;
    dReal sim_time = 0;
    dSurfaceParameters ballGroundSurface{};
    bool legacyBallFrictionActive = false;
    static bool ballGroundContact(dGeomID, dGeomID, PSurface*, int);
    QList<SendingPacket*> sendQueue;
    bool lastInfraredState[TEAM_COUNT][MAX_ROBOT_COUNT]{};
    KickStatus lastKickState[TEAM_COUNT][MAX_ROBOT_COUNT]{};
    void processSimControl(const SimulatorCommand &simulatorCommand, SimulatorResponse &simulatorResponse);
    void processRobotControl(const RobotControl &robotControl, RobotControlResponse &robotControlResponse, Team team);
    void processRobotSpec(const RobotSpecs &robotSpec) const;
    static void processRobotLimits(const RobotSpecs &robotSpec, RobotSettings *settings);
    static void processMoveCommand(RobotControlResponse &robotControlResponse, const RobotMoveCommand &robotCommand,
                            Robot *robot) ;
    void releaseBallHolds() const;
    void processTeleportBall(SimulatorResponse &simulatorResponse, const TeleportBall &teleBall) const;
    static void processTeleportRobot(const TeleportRobot &teleBot, Robot *robot);
public:    
    dReal customDT;
    SSLWorld(QObject* parent, SimulationSettings* _cfg, RobotsFormation *form1, RobotsFormation *form2);
    ~SSLWorld() override;
    void stepSimulation(dReal dt);
    void stepPhysics(dReal dt, const PhysicsStepOptions& options = PhysicsStepOptions(),
                     PWorld::StepStatistics* statistics = nullptr);
    // Apply the same ball material, mass and optional damping on startup/edit.
    void updateBallPhysics();
    std::shared_ptr<SceneSnapshot> snapshot(quint64 generation) const;
    SSL_WrapperPacket* generatePacket(int cam_id=0);
    void addFieldLinesArcs(SSL_GeometryFieldSize *field);
    static void addFieldLine(SSL_GeometryFieldSize *field, const std::string &name, float p1_x, float p1_y, float p2_x, float p2_y, float thickness);
    static void addFieldArc(SSL_GeometryFieldSize *field, const string &name, float c_x, float c_y, float radius, float a1, float a2, float thickness);
    void sendVisionBuffer();
    static bool visibleInCam(int id, double x, double y);
    QPair<float, float> cameraPosition(int id);
    int  robotIndex(int robot,int team);
    static void addRobotStatus(Robots_Status& robotsPacket, int robotID, bool infrared, KickStatus kickStatus);
    void sendRobotStatus(Robots_Status& robotsPacket, const QHostAddress& sender, int team);

    SimulationSettings* cfg;
    PWorld* p;
    PBall* ball;
    PGround* ground;
    PFixedBox* walls[WALL_COUNT]{};
    RoboCupSSLServer *visionServer{};
    QUdpSocket *commandSocket{};
    QUdpSocket *blueStatusSocket{},*yellowStatusSocket{};
    QUdpSocket *simControlSocket;
    QUdpSocket *blueControlSocket;
    QUdpSocket *yellowControlSocket;

    QElapsedTimer elapsedLastPackageBlue;
    QElapsedTimer elapsedLastPackageYellow;

    Robot* robots[MAX_ROBOT_COUNT*2]{};
    int sendGeomCount;
    bool restartRequired;
public slots:
    void recvActions();
    void simControlSocketReady();
    void blueControlSocketReady();
    void yellowControlSocketReady();
signals:
    void fpsChanged(int newFPS);
};


enum E_FORMATION {
    FORMATION_OUTSIDE = 0,
    FORMATION_INSIDE_1 = 1,
    FORMATION_INSIDE_2 = 2,
    FORMATION_OUTSIDE_FIELD = 3
};

class RobotsFormation {
    public:
        dReal x[MAX_ROBOT_COUNT]{};
        dReal y[MAX_ROBOT_COUNT]{};
        RobotsFormation(E_FORMATION type, SimulationSettings* _cfg);
        void setAll(const dReal *xx,const dReal *yy);
        void loadFromFile(const QString& filename);
        void resetRobots(Robot** r,int team);
    private:
        SimulationSettings* cfg;
};

dReal fric(dReal f);
int robotIndex(int robot,int team);

#endif // SSLWORLD_H
