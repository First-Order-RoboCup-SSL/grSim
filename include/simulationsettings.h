#ifndef SIMULATIONSETTINGS_H
#define SIMULATIONSETTINGS_H

#include <array>
#include <string>
#include <QMetaType>

class RobotSettings {
public:
    //geometeric settings
    double RobotCenterFromKicker;
    double RobotRadius;
    double RobotHeight;
    double BottomHeight;
    double KickerZ;
    double KickerThickness;
    double KickerWidth;
    double KickerHeight;
    double WheelRadius;
    double WheelThickness;
    double Wheel1Angle;
    double Wheel2Angle;
    double Wheel3Angle;
    double Wheel4Angle;
    //physical settings
    double BodyMass;
    double WheelMass;
    double KickerMass;
    double KickerDampFactor;
    double RollerTorqueFactor;
    double RollerPerpendicularTorqueFactor;
    double Kicker_Friction;
    double WheelTangentFriction;
    double WheelPerpendicularFriction;
    double Wheel_Motor_FMax;
    double MaxLinearKickSpeed;
    double MaxChipKickSpeed;
    double AccSpeedupAbsoluteMax;
    double AccSpeedupAngularMax;
    double AccBrakeAbsoluteMax;
    double AccBrakeAngularMax;
    double VelAbsoluteMax;
    double VelAngularMax;
    bool operator==(const RobotSettings& other) const {
        for (auto member : members())
            if (this->*member != other.*member) return false;
        return true;
    }

    void applyChanges(const RobotSettings& before, const RobotSettings& after) {
        for (auto member : members())
            if (before.*member != after.*member) this->*member = after.*member;
    }

private:
    static const std::array<double RobotSettings::*, 32>& members() {
        static const std::array<double RobotSettings::*, 32> values{{
            &RobotSettings::RobotCenterFromKicker, &RobotSettings::RobotRadius,
            &RobotSettings::RobotHeight, &RobotSettings::BottomHeight,
            &RobotSettings::KickerZ, &RobotSettings::KickerThickness,
            &RobotSettings::KickerWidth, &RobotSettings::KickerHeight,
            &RobotSettings::WheelRadius, &RobotSettings::WheelThickness,
            &RobotSettings::Wheel1Angle, &RobotSettings::Wheel2Angle,
            &RobotSettings::Wheel3Angle, &RobotSettings::Wheel4Angle,
            &RobotSettings::BodyMass, &RobotSettings::WheelMass, &RobotSettings::KickerMass,
            &RobotSettings::KickerDampFactor, &RobotSettings::RollerTorqueFactor,
            &RobotSettings::RollerPerpendicularTorqueFactor, &RobotSettings::Kicker_Friction,
            &RobotSettings::WheelTangentFriction, &RobotSettings::WheelPerpendicularFriction,
            &RobotSettings::Wheel_Motor_FMax, &RobotSettings::MaxLinearKickSpeed,
            &RobotSettings::MaxChipKickSpeed, &RobotSettings::AccSpeedupAbsoluteMax,
            &RobotSettings::AccSpeedupAngularMax, &RobotSettings::AccBrakeAbsoluteMax,
            &RobotSettings::AccBrakeAngularMax, &RobotSettings::VelAbsoluteMax,
            &RobotSettings::VelAngularMax
        }};
        return values;
    }
};



// Values copied on the GUI thread; the simulation owns its copy.
struct SimulationSettings {
    RobotSettings robotSettings{}, blueSettings{}, yellowSettings{};
    int value_Robots_Count{};
    int Robots_Count() const { return value_Robots_Count; }
    double value_Camera_Height{};
    double Camera_Height() const { return value_Camera_Height; }
    double value_Camera_Focal_Length{};
    double Camera_Focal_Length() const { return value_Camera_Focal_Length; }
    double value_Camera_Scaling_Limit{};
    double Camera_Scaling_Limit() const { return value_Camera_Scaling_Limit; }
    double value_BallRadius{};
    double BallRadius() const { return value_BallRadius; }
    double value_BallMass{};
    double BallMass() const { return value_BallMass; }
    double value_BallFriction{};
    double BallFriction() const { return value_BallFriction; }
    double value_BallSlip{};
    double BallSlip() const { return value_BallSlip; }
    double value_BallBounce{};
    double BallBounce() const { return value_BallBounce; }
    double value_BallBounceVel{};
    double BallBounceVel() const { return value_BallBounceVel; }
    double value_BallLinearDamp{};
    double BallLinearDamp() const { return value_BallLinearDamp; }
    double value_BallAngularDamp{};
    double BallAngularDamp() const { return value_BallAngularDamp; }
    bool value_BallProjectAirborne{};
    bool BallProjectAirborne() const { return value_BallProjectAirborne; }
    double value_BallModelTwoPhaseAccSlide{};
    double BallModelTwoPhaseAccSlide() const { return value_BallModelTwoPhaseAccSlide; }
    double value_BallModelTwoPhaseAccRoll{};
    double BallModelTwoPhaseAccRoll() const { return value_BallModelTwoPhaseAccRoll; }
    double value_BallModelTwoPhaseKSwitch{};
    double BallModelTwoPhaseKSwitch() const { return value_BallModelTwoPhaseKSwitch; }
    double value_BallModelChipFixedLossDampingXyFirstHop{};
    double BallModelChipFixedLossDampingXyFirstHop() const { return value_BallModelChipFixedLossDampingXyFirstHop; }
    double value_BallModelChipFixedLossDampingXyOtherHops{};
    double BallModelChipFixedLossDampingXyOtherHops() const { return value_BallModelChipFixedLossDampingXyOtherHops; }
    double value_BallModelChipFixedLossDampingZ{};
    double BallModelChipFixedLossDampingZ() const { return value_BallModelChipFixedLossDampingZ; }
    double value_DesiredFPS{};
    double DesiredFPS() const { return value_DesiredFPS; }
    double value_DeltaTime{};
    double DeltaTime() const { return value_DeltaTime; }
    bool value_LowSpecMode{};
    bool LowSpecMode() const { return value_LowSpecMode; }
    int value_sendGeometryEvery{};
    int sendGeometryEvery() const { return value_sendGeometryEvery; }
    double value_Gravity{};
    double Gravity() const { return value_Gravity; }
    bool value_ResetTurnOver{};
    bool ResetTurnOver() const { return value_ResetTurnOver; }
    std::string value_VisionMulticastAddr{};
    std::string VisionMulticastAddr() const { return value_VisionMulticastAddr; }
    int value_VisionMulticastPort{};
    int VisionMulticastPort() const { return value_VisionMulticastPort; }
    int value_CommandListenPort{};
    int CommandListenPort() const { return value_CommandListenPort; }
    int value_BlueStatusSendPort{};
    int BlueStatusSendPort() const { return value_BlueStatusSendPort; }
    int value_YellowStatusSendPort{};
    int YellowStatusSendPort() const { return value_YellowStatusSendPort; }
    int value_SimControlListenPort{};
    int SimControlListenPort() const { return value_SimControlListenPort; }
    int value_BlueControlListenPort{};
    int BlueControlListenPort() const { return value_BlueControlListenPort; }
    int value_YellowControlListenPort{};
    int YellowControlListenPort() const { return value_YellowControlListenPort; }
    int value_sendDelay{};
    int sendDelay() const { return value_sendDelay; }
    bool value_noise{};
    bool noise() const { return value_noise; }
    double value_noiseDeviation_x{};
    double noiseDeviation_x() const { return value_noiseDeviation_x; }
    double value_noiseDeviation_y{};
    double noiseDeviation_y() const { return value_noiseDeviation_y; }
    double value_noiseDeviation_angle{};
    double noiseDeviation_angle() const { return value_noiseDeviation_angle; }
    bool value_vanishing{};
    bool vanishing() const { return value_vanishing; }
    double value_ball_vanishing{};
    double ball_vanishing() const { return value_ball_vanishing; }
    double value_blue_team_vanishing{};
    double blue_team_vanishing() const { return value_blue_team_vanishing; }
    double value_yellow_team_vanishing{};
    double yellow_team_vanishing() const { return value_yellow_team_vanishing; }
    std::string value_ColorRobotBlue{};
    std::string ColorRobotBlue() const { return value_ColorRobotBlue; }
    std::string value_ColorRobotYellow{};
    std::string ColorRobotYellow() const { return value_ColorRobotYellow; }
    double value_Field_Line_Width{};
    double Field_Line_Width() const { return value_Field_Line_Width; }
    double value_Field_Length{};
    double Field_Length() const { return value_Field_Length; }
    double value_Field_Width{};
    double Field_Width() const { return value_Field_Width; }
    double value_Field_Rad{};
    double Field_Rad() const { return value_Field_Rad; }
    double value_Field_Free_Kick{};
    double Field_Free_Kick() const { return value_Field_Free_Kick; }
    double value_Field_Penalty_Width{};
    double Field_Penalty_Width() const { return value_Field_Penalty_Width; }
    double value_Field_Penalty_Depth{};
    double Field_Penalty_Depth() const { return value_Field_Penalty_Depth; }
    double value_Field_Penalty_Point{};
    double Field_Penalty_Point() const { return value_Field_Penalty_Point; }
    double value_Field_Margin_Touch_Line{};
    double Field_Margin_Touch_Line() const { return value_Field_Margin_Touch_Line; }
    double value_Field_Margin_Goal_Line{};
    double Field_Margin_Goal_Line() const { return value_Field_Margin_Goal_Line; }
    double value_Field_Goal_Substitution_Area_Width{};
    double Field_Goal_Substitution_Area_Width() const { return value_Field_Goal_Substitution_Area_Width; }
    double value_Field_Referee_Margin{};
    double Field_Referee_Margin() const { return value_Field_Referee_Margin; }
    double value_Wall_Thickness{};
    double Wall_Thickness() const { return value_Wall_Thickness; }
    double value_Goal_Thickness{};
    double Goal_Thickness() const { return value_Goal_Thickness; }
    double value_Goal_Depth{};
    double Goal_Depth() const { return value_Goal_Depth; }
    double value_Goal_Width{};
    double Goal_Width() const { return value_Goal_Width; }
    double value_Goal_Height{};
    double Goal_Height() const { return value_Goal_Height; }
    std::string value_Division{};
    std::string Division() const { return value_Division; }
    std::string value_YellowTeam{};
    std::string YellowTeam() const { return value_YellowTeam; }
    std::string value_BlueTeam{};
    std::string BlueTeam() const { return value_BlueTeam; }
};

Q_DECLARE_METATYPE(SimulationSettings)
#endif
