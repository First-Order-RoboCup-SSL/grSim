#include <QApplication>
#include <QFile>
#include <QSaveFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QElapsedTimer>
#include <QTimer>
#include <QSysInfo>
#include <QUdpSocket>
#include <QNetworkDatagram>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <memory>
#include <vector>
#include <stdexcept>
#include "configwidget.h"
#include "sslworld.h"
#ifdef GRSIM_ODE_USE_BLAS
#include "grsim_ode_math.h"
#endif
#ifndef GRSIM_REPLAY_REVISION
#define GRSIM_REPLAY_REVISION "unknown"
#endif
#ifndef GRSIM_ODE_MATH_BACKEND
#define GRSIM_ODE_MATH_BACKEND "ODE"
#endif

#define ROBOT_FIELDS(X) \
    X(RobotCenterFromKicker) \
    X(RobotRadius) \
    X(RobotHeight) \
    X(BottomHeight) \
    X(KickerZ) \
    X(KickerThickness) \
    X(KickerWidth) \
    X(KickerHeight) \
    X(WheelRadius) \
    X(WheelThickness) \
    X(Wheel1Angle) \
    X(Wheel2Angle) \
    X(Wheel3Angle) \
    X(Wheel4Angle) \
    X(BodyMass) \
    X(WheelMass) \
    X(KickerMass) \
    X(KickerDampFactor) \
    X(RollerTorqueFactor) \
    X(RollerPerpendicularTorqueFactor) \
    X(Kicker_Friction) \
    X(WheelTangentFriction) \
    X(WheelPerpendicularFriction) \
    X(Wheel_Motor_FMax) \
    X(MaxLinearKickSpeed) \
    X(MaxChipKickSpeed) \
    X(AccSpeedupAbsoluteMax) \
    X(AccSpeedupAngularMax) \
    X(AccBrakeAbsoluteMax) \
    X(AccBrakeAngularMax) \
    X(VelAbsoluteMax) \
    X(VelAngularMax)
#define ROBOT_RESPONSE_FIELDS(X) \
    X(ForwardVelocityScale) \
    X(LeftVelocityScale) \
    X(AngularVelocityScale) \
    X(AngularVelocityScaleSlope)
#define SIM_INT_FIELDS(X) \
    X(Robots_Count) \
    X(sendGeometryEvery) \
    X(VisionMulticastPort) \
    X(CommandListenPort) \
    X(BlueStatusSendPort) \
    X(YellowStatusSendPort) \
    X(SimControlListenPort) \
    X(BlueControlListenPort) \
    X(YellowControlListenPort) \
    X(sendDelay)
#define SIM_DOUBLE_FIELDS(X) \
    X(Camera_Height) \
    X(Camera_Focal_Length) \
    X(Camera_Scaling_Limit) \
    X(BallRadius) \
    X(BallMass) \
    X(BallFriction) \
    X(BallRollingDeceleration) \
    X(BallSlip) \
    X(BallBounce) \
    X(BallBounceVel) \
    X(BallLinearDamp) \
    X(BallAngularDamp) \
    X(BallModelTwoPhaseAccSlide) \
    X(BallModelTwoPhaseKSwitch) \
    X(BallModelChipFixedLossDampingXyFirstHop) \
    X(BallModelChipFixedLossDampingXyOtherHops) \
    X(BallModelChipFixedLossDampingZ) \
    X(DesiredFPS) \
    X(DeltaTime) \
    X(Gravity) \
    X(noiseDeviation_x) \
    X(noiseDeviation_y) \
    X(noiseDeviation_angle) \
    X(ball_vanishing) \
    X(blue_team_vanishing) \
    X(yellow_team_vanishing) \
    X(Field_Line_Width) \
    X(Field_Length) \
    X(Field_Width) \
    X(Field_Rad) \
    X(Field_Free_Kick) \
    X(Field_Penalty_Width) \
    X(Field_Penalty_Depth) \
    X(Field_Penalty_Point) \
    X(Field_Margin_Touch_Line) \
    X(Field_Margin_Goal_Line) \
    X(Field_Goal_Substitution_Area_Width) \
    X(Field_Referee_Margin) \
    X(Wall_Thickness) \
    X(Goal_Thickness) \
    X(Goal_Depth) \
    X(Goal_Width) \
    X(Goal_Height)
#define SIM_BOOL_FIELDS(X) \
    X(BallProjectAirborne) \
    X(LowSpecMode) \
    X(ResetTurnOver) \
    X(noise) \
    X(vanishing)
#define SIM_STRING_FIELDS(X) \
    X(VisionMulticastAddr) \
    X(ColorRobotBlue) \
    X(ColorRobotYellow) \
    X(Division) \
    X(YellowTeam) \
    X(BlueTeam)

namespace {
double number(const QJsonObject& object, const char* name)
{
    const auto value = object.value(name);
    if (!value.isDouble() || !std::isfinite(value.toDouble()))
        throw std::runtime_error(std::string("Missing or nonfinite number: ") + name);
    return value.toDouble();
}
unsigned integer(const QJsonObject& object, const char* name, unsigned maximum)
{
    const double value = number(object, name);
    if (value < 0 || value > maximum || value != std::floor(value))
        throw std::runtime_error(std::string("Invalid integer: ") + name);
    return unsigned(value);
}
QJsonObject saveRobot(const RobotSettings& settings)
{
    QJsonObject result;
#define SAVE(name) result[#name] = settings.name;
    ROBOT_FIELDS(SAVE) ROBOT_RESPONSE_FIELDS(SAVE)
#undef SAVE
    return result;
}
RobotSettings loadRobot(const QJsonObject& object)
{
    RobotSettings result{};
#define LOAD(name) result.name = number(object, #name);
    ROBOT_FIELDS(LOAD)
#undef LOAD
    // Older v1 recordings have unit response, before these optional fields
    // existed. Preserve their exact command/physics behaviour.
#define LOAD(name) if (object.contains(#name)) result.name = number(object, #name);
    ROBOT_RESPONSE_FIELDS(LOAD)
#undef LOAD
    if (result.ForwardVelocityScale <= 0 || result.LeftVelocityScale <= 0 ||
        result.AngularVelocityScale <= 0 || result.AngularVelocityScaleSlope < 0)
        throw std::runtime_error("Invalid robot velocity response");
    if (result.BodyMass <= 0 || result.WheelMass <= 0 || result.KickerMass <= 0 ||
        result.RobotRadius <= 0 || result.RobotHeight <= 0 ||
        result.WheelRadius <= 0 || result.WheelThickness <= 0)
        throw std::runtime_error("Invalid robot dimensions or mass");
    const auto known = saveRobot(result);
    for (auto field = object.begin(); field != object.end(); ++field)
        if (!known.contains(field.key())) throw std::runtime_error("Unknown robot setting");
    return result;
}
QJsonObject saveSettings(const SimulationSettings& settings)
{
    QJsonObject result;
#define SAVE(name) result[#name] = settings.value_##name;
    SIM_INT_FIELDS(SAVE) SIM_DOUBLE_FIELDS(SAVE) SIM_BOOL_FIELDS(SAVE)
#undef SAVE
#define SAVE(name) result[#name] = QString::fromStdString(settings.value_##name);
    SIM_STRING_FIELDS(SAVE)
#undef SAVE
    result["robotSettings"] = saveRobot(settings.robotSettings);
    result["blueSettings"] = saveRobot(settings.blueSettings);
    result["yellowSettings"] = saveRobot(settings.yellowSettings);
    return result;
}
SimulationSettings loadSettings(const QJsonObject& object)
{
    SimulationSettings result;
#define LOAD(name) { const double value = number(object, #name); \
    if (value < std::numeric_limits<int>::min() || value > std::numeric_limits<int>::max() || value != std::floor(value)) \
        throw std::runtime_error("Invalid integer setting: " #name); result.value_##name = int(value); }
    SIM_INT_FIELDS(LOAD)
#undef LOAD
#define LOAD(name) result.value_##name = number(object, #name);
    SIM_DOUBLE_FIELDS(LOAD)
#undef LOAD
#define LOAD(name) if (!object[#name].isBool()) throw std::runtime_error("Invalid boolean: " #name); result.value_##name = object[#name].toBool();
    SIM_BOOL_FIELDS(LOAD)
#undef LOAD
#define LOAD(name) if (!object[#name].isString()) throw std::runtime_error("Invalid string: " #name); result.value_##name = object[#name].toString().toStdString();
    SIM_STRING_FIELDS(LOAD)
#undef LOAD
    result.robotSettings = loadRobot(object["robotSettings"].toObject());
    result.blueSettings = loadRobot(object["blueSettings"].toObject());
    result.yellowSettings = loadRobot(object["yellowSettings"].toObject());
    if (result.Robots_Count() < 0 || result.Robots_Count() > MAX_ROBOT_COUNT ||
        result.DeltaTime() <= 0 || result.DeltaTime() > 0.1 ||
        result.BallRadius() <= 0 || result.BallMass() <= 0 || result.Gravity() < 0 ||
        result.BallRollingDeceleration() < 0)
        throw std::runtime_error("Invalid physical settings");
    if (saveSettings(result).size() != object.size()) throw std::runtime_error("Unknown setting");
    return result;
}
QJsonArray values(const dReal* value, unsigned count)
{
    QJsonArray result;
    for (unsigned i = 0; i < count; ++i) {
        if (!std::isfinite(value[i])) throw std::runtime_error("Nonfinite body state");
        result.append(value[i]);
    }
    return result;
}
QJsonObject bodyState(dBodyID body)
{
    return {{"position", values(dBodyGetPosition(body), 3)},
            {"quaternion", values(dBodyGetQuaternion(body), 4)},
            {"velocity", values(dBodyGetLinearVel(body), 3)},
            {"angular_velocity", values(dBodyGetAngularVel(body), 3)},
            {"force", values(dBodyGetForce(body), 3)},
            {"torque", values(dBodyGetTorque(body), 3)}};
}
QJsonObject state(const SSLWorld& world, unsigned tick)
{
    // Explicit construction order avoids pointer/hash-space traversal order.
    QJsonArray bodies, robots;
    bodies.append(bodyState(world.ball->body));
    for (int i = 0; i < world.cfg->Robots_Count() * 2; ++i) {
        auto* robot = world.robots[i];
        bodies.append(bodyState(robot->chassis->body));
        QJsonArray speeds;
        for (auto* wheel : robot->wheels) {
            bodies.append(bodyState(wheel->cyl->body));
            speeds.append(wheel->speed);
        }
        bodies.append(bodyState(robot->kicker->box->body));
        robots.append(QJsonObject{{"on", robot->on}, {"wheel_commands", speeds},
                                  {"roller", robot->kicker->getRoller()}});
    }
    return {{"tick", int(tick)}, {"bodies", bodies}, {"robots", robots},
            {"simulation_time", world.snapshot(0)->simulationTime}};
}
bool finiteMessage(const google::protobuf::Message& message)
{
    const auto* reflection = message.GetReflection();
    std::vector<const google::protobuf::FieldDescriptor*> fields;
    reflection->ListFields(message, &fields);
    for (const auto* field : fields) {
        const int count = field->is_repeated() ? reflection->FieldSize(message, field) : 1;
        for (int i = 0; i < count; ++i) {
            using F = google::protobuf::FieldDescriptor;
            if (field->cpp_type() == F::CPPTYPE_FLOAT) {
                const auto value = field->is_repeated() ? reflection->GetRepeatedFloat(message, field, i) : reflection->GetFloat(message, field);
                if (!std::isfinite(value)) return false;
            } else if (field->cpp_type() == F::CPPTYPE_DOUBLE) {
                const auto value = field->is_repeated() ? reflection->GetRepeatedDouble(message, field, i) : reflection->GetDouble(message, field);
                if (!std::isfinite(value)) return false;
            } else if (field->cpp_type() == F::CPPTYPE_MESSAGE) {
                const auto& nested = field->is_repeated() ? reflection->GetRepeatedMessage(message, field, i) : reflection->GetMessage(message, field);
                if (!finiteMessage(nested)) return false;
            }
        }
    }
    return true;
}
template<class Message> Message decode(const QJsonObject& event)
{
    if (!event["data"].isString()) throw std::runtime_error("Missing command payload");
    const auto encoded = event["data"].toString().toLatin1();
    const auto data = QByteArray::fromBase64(encoded);
    Message message;
    if (data.toBase64() != encoded || !message.ParseFromArray(data.data(), data.size()) ||
        !message.IsInitialized() || !finiteMessage(message))
        throw std::runtime_error("Invalid protobuf command");
    return message;
}
QByteArray apply(SSLWorld& world, const QJsonObject& event)
{
    const auto kind = event["kind"].toString();
    if (kind == "robot_control") {
        const unsigned team = integer(event, "team", 1);
        RobotControlResponse response;
        const auto command = decode<RobotControl>(event);
        for (const auto& robot : command.robot_commands())
            if (robot.has_move_command() && !robot.move_command().has_local_velocity() &&
                !robot.move_command().has_global_velocity() && !robot.move_command().has_wheel_velocity())
                throw std::runtime_error("Unsupported robot movement");
        world.processRobotControl(command, response, team ? YELLOW : BLUE);
        if (response.errors_size()) throw std::runtime_error("Unsupported robot command");
        const auto data = response.SerializeAsString();
        return QByteArray(data.data(), int(data.size()));
    } else if (kind == "simulator_control") {
        const auto command = decode<SimulatorCommand>(event);
        if (command.has_config()) throw std::runtime_error("Replay does not support world configuration changes");
        if (command.has_control() && command.control().has_teleport_ball()) {
            const auto& ball = command.control().teleport_ball();
            // The live handler can apply these teleports before reporting an
            // unsupported field. Reject the entire capture/replay batch first,
            // including explicitly present false flags and sibling robot moves.
            if (ball.has_teleport_safely() || ball.has_roll())
                throw std::runtime_error("Replay does not support safe or rolling ball teleports");
        }
        SimulatorResponse response;
        world.processSimControl(command, response);
        if (response.errors_size()) throw std::runtime_error("Unsupported simulator command");
        const auto data = response.SerializeAsString();
        return QByteArray(data.data(), int(data.size()));
    } else if (kind == "expire") {
        const unsigned team = integer(event, "team", 1);
        for (int id = 0; id < world.cfg->Robots_Count(); ++id)
            world.robots[world.robotIndex(id, team)]->resetSpeeds();
    } else throw std::runtime_error("Unknown replay event");
    return {};
}
QJsonObject commandEvent(unsigned tick, const char* kind,
                         const google::protobuf::Message& command, int team = -1)
{
    if (!command.IsInitialized() || !finiteMessage(command)) throw std::runtime_error("Invalid command");
    const auto data = command.SerializeAsString();
    QJsonObject result{{"tick", int(tick)}, {"kind", QString::fromLatin1(kind)},
        {"data", QString::fromLatin1(QByteArray(data.data(), int(data.size())).toBase64())}};
    if (team >= 0) result["team"] = team;
    return result;
}
bool sameInitialState(const QJsonValue& left, const QJsonValue& right)
{
    if (left.type() != right.type()) return false;
    if (left.isDouble())
        return std::isfinite(right.toDouble()) && std::abs(left.toDouble() - right.toDouble()) <= 1e-12;
    if (left.isArray()) {
        const auto a = left.toArray(), b = right.toArray();
        if (a.size() != b.size()) return false;
        for (int i = 0; i < a.size(); ++i) if (!sameInitialState(a[i], b[i])) return false;
        return true;
    }
    if (left.isObject()) {
        const auto a = left.toObject(), b = right.toObject();
        if (a.keys() != b.keys()) return false;
        for (auto i = a.begin(); i != a.end(); ++i)
            if (!sameInitialState(i.value(), b[i.key()])) return false;
        return true;
    }
    return left == right;
}
const char* triangularBackend()
{
#ifdef GRSIM_ODE_USE_BLAS
    return grsim_ode::triangularBackend();
#else
    return "ODE";
#endif
}
QJsonObject readJson(const QString& filename)
{
    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly)) throw std::runtime_error("Could not open replay");
    QJsonParseError error;
    const auto document = QJsonDocument::fromJson(file.readAll(), &error);
    if (error.error != QJsonParseError::NoError || !document.isObject()) throw std::runtime_error("Invalid replay JSON");
    return document.object();
}
void writeJson(const QString& filename, const QJsonObject& object)
{
    QSaveFile file(filename);
    const auto data = QJsonDocument(object).toJson(QJsonDocument::Indented);
    if (!file.open(QIODevice::WriteOnly) || file.write(data) != data.size() || !file.commit())
        throw std::runtime_error("Could not write replay");
}
}

int main(int argc, char** argv)
{
    if (qEnvironmentVariableIsEmpty("QT_QPA_PLATFORM")) qputenv("QT_QPA_PLATFORM", "offscreen");
    QApplication app(argc, argv);
    QString recordFile, replayFile, settingsFile, fixture;
    unsigned helpers = 0, seed = 166, ticks = 180, durationMs = 0, checkpointEvery = 30;
    unsigned ports[3] = {0, 0, 0};
    try {
        for (int i = 1; i < argc; ++i) {
            const QString option = QString::fromLocal8Bit(argv[i]);
            if (++i == argc) throw std::runtime_error("Expected an option value");
            const QString value = QString::fromLocal8Bit(argv[i]);
            if (option == "--record") recordFile = value;
            else if (option == "--replay") replayFile = value;
            else if (option == "--settings") settingsFile = value;
            else if (option == "--fixture") fixture = value;
            else {
                bool valid = false;
                const auto number = value.toUInt(&valid);
                if (!valid) throw std::runtime_error("Invalid numeric option");
                if (option == "--helpers" && number <= 16) helpers = number;
                else if (option == "--seed") seed = number;
                else if (option == "--ticks" && number > 0 && number <= 1000000) ticks = number;
                else if (option == "--duration-ms" && number >= 50 && number <= 120000) durationMs = number;
                else if (option == "--checkpoint-every" && number > 0 && number <= 1000000) checkpointEvery = number;
                else if (option == "--blue-port" && number <= 65535) ports[0] = number;
                else if (option == "--yellow-port" && number <= 65535) ports[1] = number;
                else if (option == "--sim-port" && number <= 65535) ports[2] = number;
                else throw std::runtime_error("Unknown or out-of-range option");
            }
        }
        if (recordFile.isEmpty() == replayFile.isEmpty() ||
            (!fixture.isEmpty() && fixture != "spaced" && fixture != "pushing" && fixture != "pushing-turning" && fixture != "pushing-turning-fast") ||
            (!recordFile.isEmpty() && fixture.isEmpty() && !durationMs) ||
            (!fixture.isEmpty() && durationMs) ||
            (!replayFile.isEmpty() && (!fixture.isEmpty() || durationMs || !settingsFile.isEmpty())))
            throw std::runtime_error("Use --record FILE with --fixture spaced|pushing|pushing-turning|pushing-turning-fast or --duration-ms N, or --replay FILE");
        QJsonObject recording;
        SimulationSettings settings;
        PhysicsStepOptions physicsOptions;
        QString integrationPolicy = "adaptive-v1";
        if (!replayFile.isEmpty()) {
            recording = readJson(replayFile);
            const bool historical = recording["format"] == "grsim-physics-replay-v1";
            if ((!historical && recording["format"] != "grsim-physics-replay-v2") ||
                recording["ode_version"] != QString::fromLatin1(dODE_VERSION))
                throw std::runtime_error("Incompatible replay format or ODE version");
            // Missing policy is the historical fixed-five integrator.
            const unsigned recordedSteps = integer(recording, "substeps", 32);
            if (recordedSteps == 0) throw std::runtime_error("Invalid integration step count");
            const auto policy = recording.value("integration_policy");
            if (historical) {
                if (recordedSteps != 5 || (!policy.isUndefined() && policy != "fixed-5-v1"))
                    throw std::runtime_error("Unsupported historical integration policy");
                integrationPolicy = "fixed-5-v1";
                physicsOptions.integrationPolicy = IntegrationPolicy::Fixed;
            } else {
                if (policy != "fixed-5-v1" && policy != "near-shell-6-v1" && policy != "fixed-v1" && policy != "adaptive-v1")
                    throw std::runtime_error("Missing or unsupported integration policy");
                if (policy != "fixed-v1" && policy != "adaptive-v1" && recordedSteps != 5)
                    throw std::runtime_error("Legacy integration policy requires five steps");
                integrationPolicy = policy.toString();
                physicsOptions.integrationPolicy = policy == "near-shell-6-v1" ? IntegrationPolicy::NearShellSix :
                    (policy == "adaptive-v1" ? IntegrationPolicy::Adaptive : IntegrationPolicy::Fixed);
            }
            physicsOptions.substeps = recordedSteps;
            seed = integer(recording, "seed", std::numeric_limits<unsigned>::max());
            ticks = integer(recording, "ticks", 1000000);
            if (!recording["events"].isArray() || !recording["initial"].isObject())
                throw std::runtime_error("Missing replay timeline");
            settings = loadSettings(recording["settings"].toObject());
        } else {
            ConfigWidget config(false, settingsFile);
            if (settingsFile.isEmpty()) {
                config.v_BlueTeam->setString("Parsian"); config.v_YellowTeam->setString("Parsian");
                config.loadRobotsSettings();
                config.v_Robots_Count->setInt(6);
                config.v_DeltaTime->setDouble(1.0 / 60);
            }
            settings = loadSettings(saveSettings(config.simulationSettings()));
        }
        if (!dInitODE2(dInitFlagManualThreadCleanup) || !dAllocateODEDataForThread(dAllocateMaskAll))
            throw std::runtime_error("Could not initialize ODE");
        if (!dCheckConfiguration("ODE_double_precision")) {
            dCleanupODEAllDataForThread(); dCloseODE();
            throw std::runtime_error("Replay requires double-precision ODE");
        }
        QJsonObject result;
        try {
            dRandSetSeed(seed);
            std::srand(seed);
            RobotsFormation formation(FORMATION_INSIDE_1, &settings);
            SSLWorld world(nullptr, &settings, &formation, &formation);
            if (!world.p->setThreadCount(helpers)) throw std::runtime_error("Could not start ODE workers");
            const auto initial = state(world, 0);
            if (!replayFile.isEmpty() && !sameInitialState(initial, recording["initial"]))
                throw std::runtime_error("Initial world differs from the recorded configuration");
            if (!recordFile.isEmpty()) {
                recording = {{"format", "grsim-physics-replay-v2"},
                    {"build_revision", QString::fromLatin1(GRSIM_REPLAY_REVISION)},
                    {"cpu_architecture", QSysInfo::currentCpuArchitecture()}, {"recorded_helpers", int(helpers)},
                    {"ode_version", QString::fromLatin1(dODE_VERSION)},
                    {"ode_configuration", QString::fromLatin1(dGetConfiguration())},
                    {"math_backend", QString::fromLatin1(GRSIM_ODE_MATH_BACKEND)},
                    {"triangular_backend", QString::fromLatin1(triangularBackend())},
                    {"seed", double(seed)}, {"substeps", int(physicsOptions.substeps)}, {"settings", saveSettings(settings)}, {"initial", initial},
                    {"integration_policy", integrationPolicy}};
            }
            QJsonArray events = recording["events"].toArray(), checkpoints;
            checkpoints.append(initial);
            unsigned tick = 0;
            double maximumShellPenetration = 0, maximumBallSolidPenetration = 0, maximumRobotWallPenetration = 0;
            PWorld::StepStatistics statistics;
            statistics.contactObserver = [&](const dContact& contact) {
                if (dGeomGetClass(contact.geom.g1) == dSphereClass &&
                    dGeomGetClass(contact.geom.g2) == dSphereClass &&
                    contact.geom.g1 != world.ball->geom && contact.geom.g2 != world.ball->geom)
                    maximumShellPenetration = std::max(maximumShellPenetration, double(contact.geom.depth));
                const bool firstBall = contact.geom.g1 == world.ball->geom;
                const bool secondBall = contact.geom.g2 == world.ball->geom;
                if ((firstBall && contact.geom.g2 != world.ground->geom) ||
                    (secondBall && contact.geom.g1 != world.ground->geom))
                    maximumBallSolidPenetration = std::max(maximumBallSolidPenetration, double(contact.geom.depth));
                if (!firstBall && !secondBall &&
                    (dGeomGetClass(contact.geom.g1) == dBoxClass || dGeomGetClass(contact.geom.g2) == dBoxClass)) {
                    for (auto* wall : world.walls)
                        if (contact.geom.g1 == wall->geom || contact.geom.g2 == wall->geom)
                            maximumRobotWallPenetration = std::max(maximumRobotWallPenetration, double(contact.geom.depth));
                }
            };
            auto step = [&]() {
                world.stepPhysics(settings.DeltaTime(), physicsOptions, &statistics);
                ++tick;
                if (tick % checkpointEvery == 0) checkpoints.append(state(world, tick));
            };
            auto accept = [&](const QJsonObject& event) {
                const auto response = apply(world, event);
                events.append(event);
                return response;
            };
            if (!fixture.isEmpty()) {
                const bool dense = fixture != "spaced";
                SimulatorCommand placement;
                auto* ball = placement.mutable_control()->mutable_teleport_ball();
                ball->set_x(4); ball->set_y(-3); ball->set_z(settings.BallRadius());
                ball->set_vx(0); ball->set_vy(0); ball->set_vz(0);
                for (int team = 0; team < 2; ++team)
                    for (int id = 0; id < settings.Robots_Count(); ++id) {
                        auto* robot = placement.mutable_control()->add_teleport_robot();
                        robot->mutable_id()->set_id(id); robot->mutable_id()->set_team(team ? YELLOW : BLUE);
                        const int col = dense ? (team ? 2 + id % 2 : 1 - id % 2) : team * 3 + id % 3;
                        robot->set_x((col - 2.5) * (dense ? .185 : .7));
                        robot->set_y((id / (dense ? 2 : 3) - 1) * (dense ? .185 : .7));
                        robot->set_orientation(team ? M_PI : 0);
                    }
                accept(commandEvent(tick, "simulator_control", placement));
                while (tick < ticks) {
                    // Fixture inputs have their own fixed schedule. Changing
                    // observation density must never alter the accepted commands.
                    if (tick % 30 == 0) {
                        for (int team = 0; team < 2; ++team) {
                            RobotControl command;
                            for (int id = 0; id < settings.Robots_Count(); ++id) {
                                auto* robot = command.add_robot_commands(); robot->set_id(id);
                                auto* velocity = robot->mutable_move_command()->mutable_local_velocity();
                                velocity->set_left(0);
                                if (fixture == "pushing-turning" || fixture == "pushing-turning-fast") {
                                    velocity->set_forward(.5);
                                    const double turn = fixture == "pushing-turning-fast" ? 4 : 2;
                                    velocity->set_angular(id % 2 ? -turn : turn);
                                } else {
                                    velocity->set_forward(tick < ticks * 2 / 3 ? (dense ? .5 : .15) : 0);
                                    velocity->set_angular(dense ? 0 : .2);
                                }
                            }
                            accept(commandEvent(tick, "robot_control", command, team));
                        }
                    }
                    step();
                }
            } else if (durationMs) {
                QUdpSocket sockets[3];
                QElapsedTimer commandClock[2];
                bool expired[2] = {false, false};
                for (int i = 0; i < 3; ++i) {
                    if (!sockets[i].bind(QHostAddress(QHostAddress::LocalHost), quint16(ports[i])))
                        throw std::runtime_error("Could not bind replay capture port");
                    QObject::connect(&sockets[i], &QUdpSocket::readyRead, [&, i]() {
                        while (sockets[i].hasPendingDatagrams()) {
                            const auto datagram = sockets[i].receiveDatagram();
                            try {
                                if (i < 2) {
                                    RobotControl command;
                                    if (!command.ParseFromArray(datagram.data().data(), datagram.data().size())) continue;
                                    const auto response = accept(commandEvent(tick, "robot_control", command, i));
                                    sockets[i].writeDatagram(response, datagram.senderAddress(), datagram.senderPort());
                                    commandClock[i].restart(); expired[i] = false;
                                } else {
                                    SimulatorCommand command;
                                    if (!command.ParseFromArray(datagram.data().data(), datagram.data().size())) continue;
                                    const auto response = accept(commandEvent(tick, "simulator_control", command));
                                    sockets[i].writeDatagram(response, datagram.senderAddress(), datagram.senderPort());
                                }
                            } catch (const std::exception& error) {
                                std::fprintf(stderr, "Rejected capture command: %s\n", error.what());
                            }
                        }
                    });
                }
                std::fprintf(stderr, "READY blue=%u yellow=%u simulator=%u\n",
                             sockets[0].localPort(), sockets[1].localPort(), sockets[2].localPort());
                std::fflush(stderr);
                QElapsedTimer clock;
                clock.start();
                for (auto& elapsed : commandClock) elapsed.start();
                QTimer timer;
                timer.setTimerType(Qt::PreciseTimer); timer.setInterval(1);
                QObject::connect(&timer, &QTimer::timeout, [&]() {
                    try {
                        for (int team = 0; team < 2; ++team)
                            if (!expired[team] && commandClock[team].elapsed() > 1000) {
                                accept({{"tick", int(tick)}, {"kind", "expire"}, {"team", team}});
                                expired[team] = true;
                            }
                        unsigned budget = 0;
                        while ((tick + 1) * settings.DeltaTime() <= clock.nsecsElapsed() / 1e9 && budget++ < 4)
                            step();
                    } catch (const std::exception& error) {
                        std::fprintf(stderr, "Capture failed: %s\n", error.what()); app.exit(1);
                    }
                });
                timer.start();
                QTimer::singleShot(durationMs, &app, &QCoreApplication::quit);
                if (app.exec()) throw std::runtime_error("Capture failed");
                ticks = tick;
            } else {
                unsigned previous = 0;
                for (const auto& value : events) {
                    if (!value.isObject()) throw std::runtime_error("Invalid replay event");
                    const unsigned at = integer(value.toObject(), "tick", ticks);
                    if (at > ticks || at < previous) throw std::runtime_error("Unordered or out-of-range command tick");
                    previous = at;
                }
                unsigned index = 0;
                while (tick < ticks) {
                    while (index < unsigned(events.size()) &&
                           integer(events[int(index)].toObject(), "tick", ticks) == tick)
                        apply(world, events[int(index++)].toObject());
                    step();
                }
                while (index < unsigned(events.size()))
                    apply(world, events[int(index++)].toObject());
            }
            if (tick % checkpointEvery == 0) checkpoints.removeLast();
            checkpoints.append(state(world, tick));
            recording["ticks"] = int(tick);
            recording["events"] = events;
            if (!recordFile.isEmpty()) writeJson(recordFile, recording);
            result = {{"success", true}, {"ticks", int(tick)}, {"helpers", int(helpers)},
                {"step_seconds", settings.DeltaTime()}, {"substeps", int(physicsOptions.substeps)},
                {"ode_version", QString::fromLatin1(dODE_VERSION)},
                {"build_revision", QString::fromLatin1(GRSIM_REPLAY_REVISION)},
                {"cpu_architecture", QSysInfo::currentCpuArchitecture()},
                {"math_backend", QString::fromLatin1(GRSIM_ODE_MATH_BACKEND)},
                    {"triangular_backend", QString::fromLatin1(triangularBackend())},
                {"checkpoints", checkpoints}, {"max_shell_penetration_m", maximumShellPenetration},
                {"max_ball_solid_penetration_m", maximumBallSolidPenetration},
                {"max_robot_wall_penetration_m", maximumRobotWallPenetration},
                {"contacts", double(statistics.contacts)}, {"integration_steps", double(statistics.integrationSteps)},
                {"integration_policy", integrationPolicy}};
        } catch (...) {
            dCleanupODEAllDataForThread(); dCloseODE(); throw;
        }
        dCleanupODEAllDataForThread(); dCloseODE();
        std::puts(QJsonDocument(result).toJson(QJsonDocument::Compact).constData());
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "%s\n", error.what());
        return 1;
    }
}
