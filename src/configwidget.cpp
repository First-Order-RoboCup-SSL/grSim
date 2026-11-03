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

#include "configwidget.h"

#include <memory>
#include <QTimer>
#include <QFile>
#include <QFileInfo>
#include <QXmlStreamReader>
#include <stdexcept>
#include <cmath>

#define ADD_ENUM(type,name,Defaultvalue,namestring) \
    v_##name = std::shared_ptr<VarTypes::Var##type>(new VarTypes::Var##type(namestring,Defaultvalue)); \
    connect(v_##name.get(), SIGNAL(wasEdited(VarPtr)), this, SLOT(queueSettingsEdited()));
#define ADD_VALUE(parent,type,name,defaultvalue,namestring) \
    v_##name = std::shared_ptr<VarTypes::Var##type>(new VarTypes::Var##type(namestring,defaultvalue)); \
    parent->addChild(v_##name); \
    connect(v_##name.get(), SIGNAL(wasEdited(VarPtr)), this, SLOT(queueSettingsEdited()));

#define END_ENUM(parents, name) \
    parents->addChild(v_##name);
#define ADD_TO_ENUM(name,str) \
    v_##name->addItem(str);


// Resolve resources relative to the executable, without changing the process CWD.
static QString robotConfigDirectory()
{
    const QDir executable(QCoreApplication::applicationDirPath());
    for (const QString& relative : {QString("../Resources/config"), QString("../config"),
                                    QString("../share/grSim/config")}) {
        const QDir candidate(executable.absoluteFilePath(relative));
        if (!candidate.entryList(QStringList() << "*.ini", QDir::Files).isEmpty())
            return candidate.absolutePath();
    }
    qWarning("Robot configuration files were not found beside the application");
    return executable.absoluteFilePath("../config");
}

// Explicit settings are configuration input, not a request to silently use
// defaults. The implicit personal settings path keeps its historical fallback.
static void validateSettingsFile(const QString& path)
{
    QFile input(path);
    if (!QFileInfo(path).isFile() || !input.open(QIODevice::ReadOnly))
        throw std::runtime_error("Requested settings must be an existing readable regular file");
    QXmlStreamReader xml(&input);
    if (!xml.readNextStartElement() || xml.qualifiedName() != QLatin1String("VarXML") ||
        !xml.namespaceUri().isEmpty())
        throw std::runtime_error("Requested settings must have a VarXML root");
    while (!xml.atEnd()) xml.readNext();
    if (xml.hasError())
        throw std::runtime_error("Requested settings contain invalid XML");
}

ConfigWidget::ConfigWidget(bool persist, const QString& settingsPath)
    : persistSettings(persist), settingsFile(settingsPath.isEmpty()
          ? QDir::homePath() + QString("/.grsim.xml") : settingsPath) {
  if (!settingsPath.isEmpty()) validateSettingsFile(settingsPath);
  tmodel=new VarTreeModel();
  this->setModel(tmodel);  
  geo_vars = std::make_shared<VarList>("Geometry");
  world.push_back(geo_vars);  
  robot_settings = new QSettings;

  VarListPtr game_vars(new VarList("Game"));
  geo_vars->addChild(game_vars);
  ADD_ENUM(StringEnum, Division, "Division A", "Division")
  ADD_TO_ENUM(Division, "Division A");
  ADD_TO_ENUM(Division, "Division B");
  END_ENUM(game_vars, Division);
  ADD_VALUE(game_vars,Int, Robots_Count, 11, "Robots Count")
  ADD_VALUE(game_vars, String, ColorRobotBlue, "#0000ff", "Color Robot Blue")
  ADD_VALUE(game_vars, String, ColorRobotYellow, "#ffff00", "Color Robot Yellow")

  VarListPtr fields_vars(new VarList("Field"));
  VarListPtr div_a_vars(new VarList("Division A"));
  VarListPtr div_b_vars(new VarList("Division B"));
  geo_vars->addChild(fields_vars);
  fields_vars->addChild(div_a_vars);
  fields_vars->addChild(div_b_vars);

  ADD_VALUE(div_a_vars, Double, DivA_Field_Line_Width,0.010,"Line Thickness")
  ADD_VALUE(div_a_vars, Double, DivA_Field_Length,12.000,"Length")
  ADD_VALUE(div_a_vars, Double, DivA_Field_Width,9.000,"Width")
  ADD_VALUE(div_a_vars, Double, DivA_Field_Rad,0.500,"Radius")
  ADD_VALUE(div_a_vars, Double, DivA_Field_Free_Kick,0.700,"Free Kick Distance From Defense Area")
  ADD_VALUE(div_a_vars, Double, DivA_Field_Penalty_Width,3.60,"Penalty width")
  ADD_VALUE(div_a_vars, Double, DivA_Field_Penalty_Depth,1.80,"Penalty depth")
  ADD_VALUE(div_a_vars, Double, DivA_Field_Penalty_Point,8.00,"Penalty point")
  ADD_VALUE(div_a_vars, Double, DivA_Field_Margin_Touch_Line,0.3,"Margin along the touch lines")
  ADD_VALUE(div_a_vars, Double, DivA_Field_Margin_Goal_Line,0.6,"Margin behind the goal lines")
  ADD_VALUE(div_a_vars, Double, DivA_Field_Goal_Substitution_Area_Width,0.3,"Substitution area width behind the goal lines")
  ADD_VALUE(div_a_vars, Double, DivA_Field_Referee_Margin,0.0,"Referee margin")
  ADD_VALUE(div_a_vars, Double, DivA_Wall_Thickness,0.050,"Wall thickness")
  ADD_VALUE(div_a_vars, Double, DivA_Goal_Thickness,0.020,"Goal thickness")
  ADD_VALUE(div_a_vars, Double, DivA_Goal_Depth,0.18,"Goal depth")
  ADD_VALUE(div_a_vars, Double, DivA_Goal_Width,1.8,"Goal width")
  ADD_VALUE(div_a_vars, Double, DivA_Goal_Height,0.160,"Goal height")

  ADD_VALUE(div_b_vars, Double, DivB_Field_Line_Width,0.010,"Line Thickness")
  ADD_VALUE(div_b_vars, Double, DivB_Field_Length,9.000,"Length")
  ADD_VALUE(div_b_vars, Double, DivB_Field_Width,6.000,"Width")
  ADD_VALUE(div_b_vars, Double, DivB_Field_Rad,0.500,"Radius")
  ADD_VALUE(div_b_vars, Double, DivB_Field_Free_Kick,0.700,"Free Kick Distance From Defense Area")
  ADD_VALUE(div_b_vars, Double, DivB_Field_Penalty_Width,2.00,"Penalty width")
  ADD_VALUE(div_b_vars, Double, DivB_Field_Penalty_Depth,1.0,"Penalty depth")
  ADD_VALUE(div_b_vars, Double, DivB_Field_Penalty_Point,6.00,"Penalty point")
  ADD_VALUE(div_b_vars, Double, DivB_Field_Margin_Touch_Line,0.3,"Margin along the touch lines")
  ADD_VALUE(div_b_vars, Double, DivB_Field_Margin_Goal_Line,0.3,"Margin behind the goal lines")
  ADD_VALUE(div_b_vars, Double, DivB_Field_Goal_Substitution_Area_Width,0.0,"Substitution area width behind the goal lines")
  ADD_VALUE(div_b_vars, Double, DivB_Field_Referee_Margin,0.0,"Referee margin")
  ADD_VALUE(div_b_vars, Double, DivB_Wall_Thickness,0.050,"Wall thickness")
  ADD_VALUE(div_b_vars, Double, DivB_Goal_Thickness,0.020,"Goal thickness")
  ADD_VALUE(div_b_vars, Double, DivB_Goal_Depth,0.18,"Goal depth")
  ADD_VALUE(div_b_vars, Double, DivB_Goal_Width,1.000,"Goal width")
  ADD_VALUE(div_b_vars, Double, DivB_Goal_Height,0.160,"Goal height")

  ADD_ENUM(StringEnum,YellowTeam,"Parsian","Yellow Team");
  END_ENUM(geo_vars,YellowTeam)
  ADD_ENUM(StringEnum,BlueTeam,"Parsian","Blue Team");
  END_ENUM(geo_vars,BlueTeam)
  VarListPtr camera_vars(new VarList("Cameras"));
  geo_vars->addChild(camera_vars);
  ADD_VALUE(camera_vars,Double,Camera_Height,4,"Height of all cameras")
  ADD_VALUE(camera_vars,Double,Camera_Focal_Length,390,"Focal length")
  ADD_VALUE(camera_vars,Double,Camera_Scaling_Limit,0.9,"Camera scaling limit")


    VarListPtr ballg_vars(new VarList("Ball"));
    geo_vars->addChild(ballg_vars);
        ADD_VALUE(ballg_vars,Double,BallRadius,0.0215,"Radius")
  VarListPtr phys_vars(new VarList("Physics"));
  world.push_back(phys_vars);
    VarListPtr worldp_vars(new VarList("World"));
    phys_vars->addChild(worldp_vars);  
        // VarXML uses these names as persistent keys.
        ADD_VALUE(worldp_vars,Double,DesiredFPS,60,"Desired FPS")
        ADD_VALUE(worldp_vars,Double,DeltaTime,1.0/60,"ODE time step")
        ADD_VALUE(worldp_vars,Double,Gravity,9.81,"Gravity")
        ADD_VALUE(worldp_vars,Bool,ResetTurnOver,true,"Auto reset turn-over")
        ADD_VALUE(worldp_vars,Bool,LowSpecMode,false,"Low Spec Mode")
  VarListPtr ballp_vars(new VarList("Ball"));
    phys_vars->addChild(ballp_vars);
        ADD_VALUE(ballp_vars,Double,BallMass,0.043,"Ball mass");
        ADD_VALUE(ballp_vars,Double,BallFriction,0.05,"Ball-ground friction")
        ADD_VALUE(ballp_vars,Double,BallRollingDeceleration,0.35,"Ball rolling deceleration (m/s^2)")
        ADD_VALUE(ballp_vars,Double,BallSlip,0,"Ball-ground slip")
        ADD_VALUE(ballp_vars,Double,BallBounce,0.5,"Ball-ground bounce factor")
        ADD_VALUE(ballp_vars,Double,BallBounceVel,0.1,"Ball-ground bounce min velocity")
        ADD_VALUE(ballp_vars,Double,BallLinearDamp,0,"Ball linear damping")
        ADD_VALUE(ballp_vars,Double,BallAngularDamp,0,"Ball angular damping")
        ADD_VALUE(ballp_vars,Bool,BallProjectAirborne,true,"Project airborne balls")
    auto contactBallDefaults = std::make_shared<VarBool>("Contact ball physics defaults", false);
    contactBallDefaults->addFlags(VARTYPE_FLAG_HIDDEN);
    ballp_vars->addChild(contactBallDefaults);
  VarListPtr ball_model_vars(new VarList("Models"));
  ballp_vars->addChild(ball_model_vars);
    ADD_VALUE(ball_model_vars,Double,BallModelTwoPhaseAccSlide,-14.0,"Straight-two-phase sliding acceleration")
    ADD_VALUE(ball_model_vars,Double,BallModelTwoPhaseAccRoll,-0.35,"Straight-two-phase rolling acceleration")
    ADD_VALUE(ball_model_vars,Double,BallModelTwoPhaseKSwitch,0.7,"Straight-two-phase switch factor k")
    ADD_VALUE(ball_model_vars,Double,BallModelChipFixedLossDampingXyFirstHop,0.6,"Chip-fixed-loss damping in xy-direction for first hop")
    ADD_VALUE(ball_model_vars,Double,BallModelChipFixedLossDampingXyOtherHops,0.96,"Chip-fixed-loss damping in xy-direction for other hops")
    ADD_VALUE(ball_model_vars,Double,BallModelChipFixedLossDampingZ,0.42,"Chip-fixed-loss damping in z-direction")
  VarListPtr comm_vars(new VarList("Communication"));
  world.push_back(comm_vars);
    ADD_VALUE(comm_vars,String,VisionMulticastAddr,"224.5.23.2","Vision multicast address")  //SSL Vision: "224.5.23.2"
    ADD_VALUE(comm_vars,Int,VisionMulticastPort,10020,"Vision multicast port")
    ADD_VALUE(comm_vars,Int,CommandListenPort,20011,"Command listen port")
    ADD_VALUE(comm_vars,Int,BlueStatusSendPort,30011,"Blue Team status send port")
    ADD_VALUE(comm_vars,Int,YellowStatusSendPort,30012,"Yellow Team status send port")
    ADD_VALUE(comm_vars,Int,SimControlListenPort,10300,"Simulation control port")
    ADD_VALUE(comm_vars,Int,BlueControlListenPort,10301,"Blue team control port")
    ADD_VALUE(comm_vars,Int,YellowControlListenPort,10302,"Yellow team control port")
    ADD_VALUE(comm_vars,Int,sendDelay,0,"Sending delay (milliseconds)")
    ADD_VALUE(comm_vars,Int,sendGeometryEvery,120,"Send geometry every X frames")
    VarListPtr gauss_vars(new VarList("Gaussian noise"));
        comm_vars->addChild(gauss_vars);
        ADD_VALUE(gauss_vars,Bool,noise,false,"Noise")
        ADD_VALUE(gauss_vars,Double,noiseDeviation_x,3,"Deviation for x values")
        ADD_VALUE(gauss_vars,Double,noiseDeviation_y,3,"Deviation for y values")
        ADD_VALUE(gauss_vars,Double,noiseDeviation_angle,2,"Deviation for angle values")
    VarListPtr vanishing_vars(new VarList("Vanishing probability"));
        comm_vars->addChild(vanishing_vars);
        ADD_VALUE(gauss_vars,Bool,vanishing,false,"Vanishing")
        ADD_VALUE(vanishing_vars,Double,blue_team_vanishing,0,"Blue team")
        ADD_VALUE(vanishing_vars,Double,yellow_team_vanishing,0,"Yellow team")
        ADD_VALUE(vanishing_vars,Double,ball_vanishing,0,"Ball")

    bool fpsLoaded = false, stepLoaded = false;
    const auto fpsRead = connect(v_DesiredFPS.get(), &VarType::XMLwasRead, this,
                                 [&]() { fpsLoaded = true; });
    const auto stepRead = connect(v_DeltaTime.get(), &VarType::XMLwasRead, this,
                                  [&]() { stepLoaded = true; });
    if (persistSettings || !settingsPath.isEmpty())
        world=VarXML::read(world,settingsFile.toStdString());
    disconnect(fpsRead);
    disconnect(stepRead);

    // Also accept files saved by the intermediate PR version. Remove aliases
    // and the retired synchronization option so they cannot appear as controls.
    // When both keys are present, keep the value saved under the stable name.
    auto migrateAlias = [&](const char* name, const std::shared_ptr<VarDouble>& setting, bool loaded) {
        auto saved = std::dynamic_pointer_cast<VarDouble>(worldp_vars->findChild(name));
        if (saved) {
            if (!loaded) setting->setDouble(saved->getDouble());
            worldp_vars->removeChild(saved);
        }
    };
    migrateAlias("Render FPS Cap", v_DesiredFPS, fpsLoaded);
    migrateAlias("Physics Time Step (s)", v_DeltaTime, stepLoaded);
    const auto synchronization = worldp_vars->findChild("Synchronize ODE with OpenGL");
    if (synchronization) worldp_vars->removeChild(synchronization);

    // Old startup ignored these stored defaults. Migrate them once, retaining
    // other custom values and allowing users to select the old numbers again.
    if (!contactBallDefaults->getBool()) {
        if (BallSlip() == 1) v_BallSlip->setDouble(0);
        if (BallLinearDamp() == 0.004) v_BallLinearDamp->setDouble(0);
        if (BallAngularDamp() == 0.004) v_BallAngularDamp->setDouble(0);
        contactBallDefaults->setBool(true);
    }
    v_BallRollingDeceleration->setMin(0);
    v_BallModelTwoPhaseAccRoll->setDouble(-BallRollingDeceleration());
    v_BallModelTwoPhaseAccRoll->addFlags(VARTYPE_FLAG_READONLY);


    std::string blueteam = v_BlueTeam->getString();
    geo_vars->removeChild(v_BlueTeam);

            std::string yellowteam = v_YellowTeam->getString();
    geo_vars->removeChild(v_YellowTeam);

    ADD_ENUM(StringEnum,BlueTeam,blueteam,"Blue Team");
    ADD_ENUM(StringEnum,YellowTeam,yellowteam,"Yellow Team");

    QDir dir(robotConfigDirectory());
    dir.setNameFilters(QStringList() << "*.ini");
    dir.setSorting(QDir::Size | QDir::Reversed);
    QFileInfoList list = dir.entryInfoList();

    for (const auto& fileInfo : list) {
        QStringList s = fileInfo.fileName().split(".");
        QString str;
        if (s.count() > 0) str = s[0];
        ADD_TO_ENUM(BlueTeam,str.toStdString())
        ADD_TO_ENUM(YellowTeam,str.toStdString())
    }

    END_ENUM(geo_vars,BlueTeam)
    END_ENUM(geo_vars,YellowTeam)

  v_BlueTeam->setString(blueteam);
  v_YellowTeam->setString(yellowteam);

  tmodel->setRootItems(world);

  this->expandAndFocus(geo_vars);
  this->expandAndFocus(phys_vars);
  this->expandAndFocus(comm_vars);

  this->fitColumns();

  resize(320,400);
  connect(v_BlueTeam.get(), &VarType::wasEdited, this, [this]() {
      loadRobotSettings(BlueTeam().c_str());
      blueSettings = robotSettings;
  });
  connect(v_YellowTeam.get(), &VarType::wasEdited, this, [this]() {
      loadRobotSettings(YellowTeam().c_str());
      yellowSettings = robotSettings;
  });
  loadRobotsSettings();
}

ConfigWidget::~ConfigWidget() {
    delete robot_settings;
    if (persistSettings)
        VarXML::write(world,settingsFile.toStdString());
}


ConfigDockWidget::ConfigDockWidget(QWidget* _parent,ConfigWidget* _conf){
    parent=_parent;conf=_conf;
    setWidget(conf);
    conf->move(0,20);
}  
void ConfigDockWidget::closeEvent(QCloseEvent* event)
{
    emit closeSignal(false);
}


void ConfigWidget::loadRobotsSettings()
{
    loadRobotSettings(YellowTeam().c_str());
    yellowSettings = robotSettings;
    loadRobotSettings(BlueTeam().c_str());
    blueSettings = robotSettings;
}

void ConfigWidget::loadRobotSettings(QString team)
{
    const QString ss = QDir(robotConfigDirectory()).filePath(QString("%1.ini").arg(team));
    robot_settings = new QSettings(ss, QSettings::IniFormat);
    robotSettings.RobotCenterFromKicker = robot_settings->value("Geometery/CenterFromKicker", 0.073).toDouble();
    robotSettings.RobotRadius = robot_settings->value("Geometery/Radius", 0.09).toDouble();
    robotSettings.RobotHeight = robot_settings->value("Geometery/Height", 0.13).toDouble();
    robotSettings.BottomHeight = robot_settings->value("Geometery/RobotBottomZValue", 0.02).toDouble();
    robotSettings.KickerZ = robot_settings->value("Geometery/KickerZValue", 0.005).toDouble();
    robotSettings.KickerThickness = robot_settings->value("Geometery/KickerThickness", 0.005).toDouble();
    robotSettings.KickerWidth = robot_settings->value("Geometery/KickerWidth", 0.08).toDouble();
    robotSettings.KickerHeight = robot_settings->value("Geometery/KickerHeight", 0.04).toDouble();
    robotSettings.WheelRadius = robot_settings->value("Geometery/WheelRadius", 0.0325).toDouble();
    robotSettings.WheelThickness = robot_settings->value("Geometery/WheelThickness", 0.005).toDouble();
    robotSettings.Wheel1Angle = robot_settings->value("Geometery/Wheel1Angle", 60).toDouble();
    robotSettings.Wheel2Angle = robot_settings->value("Geometery/Wheel2Angle", 135).toDouble();
    robotSettings.Wheel3Angle = robot_settings->value("Geometery/Wheel3Angle", 225).toDouble();
    robotSettings.Wheel4Angle = robot_settings->value("Geometery/Wheel4Angle", 300).toDouble();

    robotSettings.BodyMass = robot_settings->value("Physics/BodyMass", 2).toDouble();
    robotSettings.WheelMass = robot_settings->value("Physics/WheelMass", 0.2).toDouble();
    robotSettings.KickerMass = robot_settings->value("Physics/KickerMass", 0.02).toDouble();
    robotSettings.KickerDampFactor = robot_settings->value("Physics/KickerDampFactor", 0.2f).toDouble();
    robotSettings.RollerTorqueFactor = robot_settings->value("Physics/RollerTorqueFactor", 0.06f).toDouble();
    robotSettings.RollerPerpendicularTorqueFactor = robot_settings->value("Physics/RollerPerpendicularTorqueFactor", 0.005f).toDouble();
    robotSettings.Kicker_Friction = robot_settings->value("Physics/KickerFriction", 0.8f).toDouble();
    robotSettings.WheelTangentFriction = robot_settings->value("Physics/WheelTangentFriction", 0.8f).toDouble();
    robotSettings.WheelPerpendicularFriction = robot_settings->value("Physics/WheelPerpendicularFriction", 0.05f).toDouble();
    robotSettings.Wheel_Motor_FMax = robot_settings->value("Physics/WheelMotorMaximumApplyingTorque", 0.2f).toDouble();
    
    robotSettings.MaxLinearKickSpeed = robot_settings->value("Physics/MaxLinearKickSpeed", 10).toDouble();
    robotSettings.MaxChipKickSpeed = robot_settings->value("Physics/MaxChipKickSpeed", 10).toDouble();
    robotSettings.AccSpeedupAbsoluteMax = robot_settings->value("Physics/AccSpeedupAbsoluteMax", 4).toDouble();
    robotSettings.AccSpeedupAngularMax = robot_settings->value("Physics/AccSpeedupAngularMax", 50).toDouble();
    robotSettings.AccBrakeAbsoluteMax = robot_settings->value("Physics/AccBrakeAbsoluteMax", 4).toDouble();
    robotSettings.AccBrakeAngularMax = robot_settings->value("Physics/AccBrakeAngularMax", 50).toDouble();
    robotSettings.VelAbsoluteMax = robot_settings->value("Physics/VelAbsoluteMax", 5).toDouble();
    robotSettings.VelAngularMax = robot_settings->value("Physics/VelAngularMax", 20).toDouble();
    auto velocityScale = [&](const char* name) {
        const double value = robot_settings->value(QString("Actuation/") + name, 1.0).toDouble();
        if (std::isfinite(value) && value > 0) return value;
        qWarning("Invalid %s in robot model; using unit response", name);
        return 1.0;
    };
    robotSettings.ForwardVelocityScale = velocityScale("ForwardVelocityScale");
    robotSettings.LeftVelocityScale = velocityScale("LeftVelocityScale");
    robotSettings.AngularVelocityScale = velocityScale("AngularVelocityScale");
    const double slope = robot_settings->value("Actuation/AngularVelocityScaleSlope", 0.0).toDouble();
    robotSettings.AngularVelocityScaleSlope = std::isfinite(slope) && slope >= 0 ? slope : 0;
}

SimulationSettings ConfigWidget::simulationSettings()
{
    SimulationSettings result;
    result.value_Robots_Count = Robots_Count();
    result.value_Camera_Height = Camera_Height();
    result.value_Camera_Focal_Length = Camera_Focal_Length();
    result.value_Camera_Scaling_Limit = Camera_Scaling_Limit();
    result.value_BallRadius = BallRadius();
    result.value_BallMass = BallMass();
    result.value_BallFriction = BallFriction();
    result.value_BallRollingDeceleration = BallRollingDeceleration();
    result.value_BallSlip = BallSlip();
    result.value_BallBounce = BallBounce();
    result.value_BallBounceVel = BallBounceVel();
    result.value_BallLinearDamp = BallLinearDamp();
    result.value_BallAngularDamp = BallAngularDamp();
    result.value_BallProjectAirborne = BallProjectAirborne();
    result.value_BallModelTwoPhaseAccSlide = BallModelTwoPhaseAccSlide();
    result.value_BallModelTwoPhaseKSwitch = BallModelTwoPhaseKSwitch();
    result.value_BallModelChipFixedLossDampingXyFirstHop = BallModelChipFixedLossDampingXyFirstHop();
    result.value_BallModelChipFixedLossDampingXyOtherHops = BallModelChipFixedLossDampingXyOtherHops();
    result.value_BallModelChipFixedLossDampingZ = BallModelChipFixedLossDampingZ();
    result.value_DesiredFPS = DesiredFPS();
    result.value_DeltaTime = DeltaTime();
    result.value_LowSpecMode = LowSpecMode();
    result.value_sendGeometryEvery = sendGeometryEvery();
    result.value_Gravity = Gravity();
    result.value_ResetTurnOver = ResetTurnOver();
    result.value_VisionMulticastAddr = VisionMulticastAddr();
    result.value_VisionMulticastPort = VisionMulticastPort();
    result.value_CommandListenPort = CommandListenPort();
    result.value_BlueStatusSendPort = BlueStatusSendPort();
    result.value_YellowStatusSendPort = YellowStatusSendPort();
    result.value_SimControlListenPort = SimControlListenPort();
    result.value_BlueControlListenPort = BlueControlListenPort();
    result.value_YellowControlListenPort = YellowControlListenPort();
    result.value_sendDelay = sendDelay();
    result.value_noise = noise();
    result.value_noiseDeviation_x = noiseDeviation_x();
    result.value_noiseDeviation_y = noiseDeviation_y();
    result.value_noiseDeviation_angle = noiseDeviation_angle();
    result.value_vanishing = vanishing();
    result.value_ball_vanishing = ball_vanishing();
    result.value_blue_team_vanishing = blue_team_vanishing();
    result.value_yellow_team_vanishing = yellow_team_vanishing();
    result.value_ColorRobotBlue = ColorRobotBlue();
    result.value_ColorRobotYellow = ColorRobotYellow();
    result.value_Field_Line_Width = Field_Line_Width();
    result.value_Field_Length = Field_Length();
    result.value_Field_Width = Field_Width();
    result.value_Field_Rad = Field_Rad();
    result.value_Field_Free_Kick = Field_Free_Kick();
    result.value_Field_Penalty_Width = Field_Penalty_Width();
    result.value_Field_Penalty_Depth = Field_Penalty_Depth();
    result.value_Field_Penalty_Point = Field_Penalty_Point();
    result.value_Field_Margin_Touch_Line = Field_Margin_Touch_Line();
    result.value_Field_Margin_Goal_Line = Field_Margin_Goal_Line();
    result.value_Field_Goal_Substitution_Area_Width = Field_Goal_Substitution_Area_Width();
    result.value_Field_Referee_Margin = Field_Referee_Margin();
    result.value_Wall_Thickness = Wall_Thickness();
    result.value_Goal_Thickness = Goal_Thickness();
    result.value_Goal_Depth = Goal_Depth();
    result.value_Goal_Width = Goal_Width();
    result.value_Goal_Height = Goal_Height();
    result.value_Division = Division();
    result.value_YellowTeam = YellowTeam();
    result.value_BlueTeam = BlueTeam();
    result.blueSettings = blueSettings;
    result.yellowSettings = yellowSettings;
    result.robotSettings = robotSettings;
    return result;
}

void ConfigWidget::applyRemoteSettings(const SimulationSettings& settings)
{
    // A preset selection may still be waiting for its coalesced settingsEdited
    // notification. Keep it until the controller has received the new team name.
    if (settings.BlueTeam() == BlueTeam()) blueSettings = settings.blueSettings;
    if (settings.YellowTeam() == YellowTeam()) yellowSettings = settings.yellowSettings;
}

void ConfigWidget::queueSettingsEdited()
{
    v_BallModelTwoPhaseAccRoll->setDouble(-BallRollingDeceleration());
    if (settingsNotificationPending) return;
    settingsNotificationPending = true;
    QTimer::singleShot(0, this, [this]() {
        settingsNotificationPending = false;
        emit settingsEdited();
    });
}
