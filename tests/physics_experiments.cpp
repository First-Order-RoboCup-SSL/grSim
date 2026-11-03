#include <QApplication>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include "configwidget.h"
#include "sslworld.h"

namespace {
constexpr double tickSize = 1.0 / 60.0;
struct Variant {
    QString name;
    double rollingScale = 0;
    bool correctInertia = false;
    bool finiteWheels = false;
    double wheelCfm = -1;
    double layer = 0;
    double correctionSpeed = -1;
    unsigned iterations = 0;
    bool systemDefaults = false;
};

Variant variantFor(const QString& name)
{
    Variant v;
    v.name = name;
    if (name == "system") {
        v.systemDefaults = true;
        v.correctInertia = true;
        v.rollingScale = 1;
    }
    else if (name == "baseline") {}
    else if (name == "ball-native-half") v.rollingScale = 0.5;
    else if (name == "ball-native") v.rollingScale = 1;
    else if (name == "ball-native-double") v.rollingScale = 2;
    else if (name == "inertia-z") v.correctInertia = true;
    else if (name == "wheel-finite") v.finiteWheels = true;
    else if (name == "wheel-cfm-half") v.wheelCfm = 0.001;
    else if (name == "wheel-cfm-double") v.wheelCfm = 0.004;
    else if (name == "contact-layer") v.layer = 0.0001;
    else if (name == "contact-correction") v.correctionSpeed = 0.5;
    else if (name == "quick-20") v.iterations = 20;
    else if (name == "quick-80") v.iterations = 80;
    else if (name == "quick-160") v.iterations = 160;
    else throw std::invalid_argument("Unknown variant");
    return v;
}

QJsonArray vectorJson(const dReal* values, int count=3)
{
    QJsonArray result;
    for (int i=0; i<count; ++i) {
        if (!std::isfinite(values[i])) throw std::runtime_error("Non-finite body state");
        result.append(values[i]);
    }
    return result;
}

struct WheelContact {
    PSurfaceCallback* original;
    void* originalData;
    double cfm;
};

bool experimentalWheelContact(dGeomID first, dGeomID second, PSurface* surface, int count)
{
    auto* context = static_cast<WheelContact*>(surface->callbackData);
    surface->callbackData = context->originalData;
    const bool result = context->original(first, second, surface, count);
    surface->callbackData = context;
    surface->surface.soft_cfm = context->cfm;
    return result;
}

// The old callback retained its held-ball settings after tag reset. Robot cases
// start with a ground ball and the former default tag of zero, so reproducing
// this behavior is necessary even though their passive ball is far from robots.
bool historicalBallContact(dGeomID, dGeomID, PSurface* surface, int)
{
    auto* world = static_cast<SSLWorld*>(surface->callbackData);
    if (world->ball->tag >= 0 && world->ball->tag < world->cfg->Robots_Count()*2) {
        dReal x, y, z;
        world->robots[world->ball->tag]->chassis->getBodyDirection(x, y, z);
        surface->fdir1[0] = x;
        surface->fdir1[1] = y;
        surface->fdir1[2] = surface->fdir1[3] = 0;
        surface->usefdir1 = true;
        surface->surface.mode = dContactMu2 | dContactFDir1 | dContactSoftCFM;
        surface->surface.mu = world->cfg->BallFriction();
        surface->surface.mu2 = 0.5;
        surface->surface.soft_cfm = 0.002;
    }
    return true;
}

double applyVariant(SSLWorld& world, const Variant& variant,
                  std::vector<std::unique_ptr<WheelContact>>& contexts,
                  unsigned substeps, bool normalizeCompliance)
{
    double complianceScale = 1;
    if (normalizeCompliance && substeps != 5) {
        const double erp = dWorldGetERP(world.p->world);
        const double timestepRatio = 5.0/substeps;
        complianceScale = 1/(erp*timestepRatio + 1-erp);
        dWorldSetERP(world.p->world, erp*timestepRatio*complianceScale);
        dWorldSetCFM(world.p->world, dWorldGetCFM(world.p->world)*complianceScale);
    }
    // Preserve the historical experiments explicitly as production defaults
    // evolve. The system variant exercises the unmodified live configuration.
    if (!variant.systemDefaults) {
        world.p->setGravity(9.81f);
        world.ball->tag = 0;
        dBodySetLinearDamping(world.ball->body, 0);
        dBodySetAngularDamping(world.ball->body, 0);
        for (int i=0; i<dSpaceGetNumGeoms(world.p->space); ++i) {
            const auto geom = dSpaceGetGeom(world.p->space, i);
            if (dGeomGetClass(geom) != dCylinderClass) continue;
            dReal radius, length;
            dGeomCylinderGetParams(geom, &radius, &length);
            const auto body = dGeomGetBody(geom);
            dMass mass;
            dBodyGetMass(body, &mass);
            dMassSetCylinderTotal(&mass, mass.mass, variant.correctInertia ? 3 : 1, radius, length);
            dBodySetMass(body, &mass);
        }
        auto* surface = world.p->findSurface(world.ball, world.ground);
        surface->callback = variant.rollingScale > 0 ? nullptr : historicalBallContact;
        surface->surface = {};
        surface->surface.mode = dContactBounce | dContactApprox1;
        surface->surface.mu = 1;
        surface->surface.bounce = world.cfg->BallBounce();
        surface->surface.bounce_vel = world.cfg->BallBounceVel();
        if (variant.rollingScale > 0) {
            surface->surface.mode |= dContactRolling | dContactMu2;
            surface->surface.mu2 = surface->surface.mu;
            const double rho = variant.rollingScale * 2 * world.cfg->BallFriction() * world.cfg->BallRadius();
            surface->surface.rho = surface->surface.rho2 = surface->surface.rhoN = rho;
        }
    }
    if (variant.layer > 0) dWorldSetContactSurfaceLayer(world.p->world, variant.layer);
    if (variant.correctionSpeed > 0)
        dWorldSetContactMaxCorrectingVel(world.p->world, variant.correctionSpeed);
    if (variant.wheelCfm > 0 || complianceScale != 1) {
        for (int i=0; i<world.cfg->Robots_Count()*2; ++i) {
            for (auto* wheel : world.robots[i]->wheels) {
                auto* surface = world.p->findSurface(wheel->cyl, world.ground);
                const double cfm = (variant.wheelCfm > 0 ? variant.wheelCfm : 0.002)*complianceScale;
                contexts.emplace_back(new WheelContact{surface->callback, surface->callbackData, cfm});
                surface->callbackData = contexts.back().get();
                surface->callback = experimentalWheelContact;
            }
        }
    }
    return complianceScale;
}

struct Diagnostics {
    double maxJoint = 0, sumJointSquared = 0, maxAxisDegrees = 0;
    double maxLinearSpeed = 0, maxAngularSpeed = 0, minChassisZ = 1e9;
    unsigned long jointSamples = 0;
    void observe(const SSLWorld& world)
    {
        for (int i=0; i<dSpaceGetNumGeoms(world.p->space); ++i) {
            const auto geom = dSpaceGetGeom(world.p->space, i);
            const auto body = dGeomGetBody(geom);
            if (!body) continue;
            vectorJson(dBodyGetPosition(body));
            vectorJson(dBodyGetRotation(body), 12);
            const auto* v = dBodyGetLinearVel(body);
            const auto* w = dBodyGetAngularVel(body);
            vectorJson(v); vectorJson(w);
            maxLinearSpeed = std::max(maxLinearSpeed, std::sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]));
            maxAngularSpeed = std::max(maxAngularSpeed, std::sqrt(w[0]*w[0]+w[1]*w[1]+w[2]*w[2]));
        }
        for (int i=0; i<world.cfg->Robots_Count()*2; ++i) {
            minChassisZ = std::min(minChassisZ, double(dBodyGetPosition(world.robots[i]->chassis->body)[2]));
            for (auto* wheel : world.robots[i]->wheels) {
                dVector3 first, second, axis;
                dJointGetHingeAnchor(wheel->joint, first);
                dJointGetHingeAnchor2(wheel->joint, second);
                double squared = 0;
                for (int j=0; j<3; ++j) squared += (first[j]-second[j])*(first[j]-second[j]);
                maxJoint = std::max(maxJoint, std::sqrt(squared));
                sumJointSquared += squared;
                ++jointSamples;
                dJointGetHingeAxis(wheel->joint, axis);
                const auto* rotation = dBodyGetRotation(wheel->cyl->body);
                const double dot = std::abs(axis[0]*rotation[2]+axis[1]*rotation[6]+axis[2]*rotation[10]);
                maxAxisDegrees = std::max(maxAxisDegrees, std::acos(std::min(1.0, dot))*180/M_PI);
            }
        }
    }
};

QJsonArray record(const SSLWorld& world, double time)
{
    QJsonArray row;
    row.append(time);
    for (double value : {dBodyGetPosition(world.ball->body)[0], dBodyGetPosition(world.ball->body)[1],
                        dBodyGetPosition(world.ball->body)[2], dBodyGetLinearVel(world.ball->body)[0],
                        dBodyGetLinearVel(world.ball->body)[1], dBodyGetLinearVel(world.ball->body)[2],
                        dBodyGetAngularVel(world.ball->body)[0], dBodyGetAngularVel(world.ball->body)[1],
                        dBodyGetAngularVel(world.ball->body)[2]}) row.append(value);
    if (world.cfg->Robots_Count()) {
        auto* robot = world.robots[0];
        const auto* p = dBodyGetPosition(robot->chassis->body);
        const auto* r = dBodyGetRotation(robot->chassis->body);
        row.append(p[0]); row.append(p[1]); row.append(p[2]);
        row.append(std::atan2(r[4], r[0]));
        for (int i=0; i<3; ++i) row.append(dBodyGetLinearVel(robot->chassis->body)[i]);
        row.append(dBodyGetAngularVel(robot->chassis->body)[2]);
    }
    return row;
}

QJsonObject torqueExperiment(const SimulationSettings& settings, const Variant& variant,
                            bool wheel, unsigned substeps)
{
    const double mass = wheel ? settings.blueSettings.WheelMass : settings.blueSettings.BodyMass*0.99;
    const double radius = wheel ? settings.blueSettings.WheelRadius : settings.blueSettings.RobotRadius;
    const double length = wheel ? settings.blueSettings.WheelThickness : settings.blueSettings.RobotHeight;
    const double torque = 0.02, duration = 0.2;
    const double expected[3] = {mass*(radius*radius/4+length*length/12),
                               mass*(radius*radius/4+length*length/12), mass*radius*radius/2};
    QJsonArray inertia, observedOmega, expectedOmega;
    double maxError = 0, milliseconds = 0;
    for (int axis=0; axis<3; ++axis) {
        PWorld world(tickSize/substeps, 0, 0);
        auto* cylinder = new PCylinder(0, 0, 1, radius, length, mass, 1, 1, 1);
        world.addObject(cylinder);
        world.initAllObjects();
        dMass model;
        dBodyGetMass(cylinder->body, &model);
        if (!variant.systemDefaults) {
            dMassSetCylinderTotal(&model, mass, variant.correctInertia ? 3 : 1, radius, length);
            dBodySetMass(cylinder->body, &model);
        }
        inertia.append(model.I[axis*4+axis]);
        const auto start = std::chrono::steady_clock::now();
        for (unsigned step=0; step<12*substeps; ++step) {
            dVector3 applied = {0,0,0,0};
            applied[axis] = torque;
            dBodyAddTorque(cylinder->body, applied[0], applied[1], applied[2]);
            if (!world.step(tickSize/substeps, variant.iterations)) throw std::runtime_error("Torque step failed");
        }
        milliseconds += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-start).count();
        const double actual = dBodyGetAngularVel(cylinder->body)[axis];
        const double reference = torque*duration/expected[axis];
        observedOmega.append(actual); expectedOmega.append(reference);
        maxError = std::max(maxError, std::abs(actual/reference-1));
    }
    return {{"physics_ms", milliseconds}, {"ms_per_tick", milliseconds/36},
            {"steps",36}, {"duration_s",3*duration}, {"warmup_ticks",0},
            {"metrics", QJsonObject{{"max_relative_angular_velocity_error",maxError},
            {"inertia_xyz_kg_m2",inertia}, {"omega_xyz_rad_s",observedOmega},
            {"expected_omega_xyz_rad_s",expectedOmega}}}};
}

QJsonObject runScenario(SimulationSettings settings, const Variant& variant, const QString& scenario,
                        unsigned substeps, int requestedSteps, bool normalizeCompliance, double initialSpeed)
{
    if (scenario == "torque-chassis" || scenario == "torque-wheel")
        return torqueExperiment(settings, variant, scenario == "torque-wheel", substeps);
    const bool ballCase = scenario.startsWith("ball-");
    const bool performance = scenario.startsWith("perf-");
    const bool crowded = scenario == "crowded-contact" || scenario == "perf-crowded";
    const int robots = ballCase ? 0 : (performance || crowded ? (scenario == "perf-16" ? 16 : 11) : 1);
    settings.value_Robots_Count = robots;
    settings.value_DeltaTime = tickSize;
    RobotsFormation formation(FORMATION_INSIDE_1, &settings);
    SSLWorld world(nullptr, &settings, &formation, &formation);
    std::vector<std::unique_ptr<WheelContact>> contexts;
    const double complianceScale=applyVariant(world, variant, contexts, substeps, normalizeCompliance);
    PhysicsStepOptions options;
    options.refineShellImpacts = variant.systemDefaults;
    options.substeps = substeps;
    options.quickStepIterations = variant.iterations;
    options.legacyBallFriction = variant.rollingScale == 0;
    options.finiteWheelRotation = variant.finiteWheels;
    if (crowded) {
        for (int i=0; i<robots*2; ++i)
            world.robots[i]->setXY((i%4)*0.185-0.28, (i/4)*0.185-0.28);
    } else if (!performance && robots) {
        world.robots[0]->setXY(-2, 0);
        world.robots[1]->setXY(3, 3);
    }
    // Establish the initial heading before the recorded command response.
    for (int i=0; i<robots*2; ++i) world.robots[i]->step();
    if (!ballCase) dBodySetPosition(world.ball->body, 4, -3, settings.BallRadius());
    int warmup = performance ? 60 : (crowded || scenario == "ball-flight" || scenario == "ball-drop" ? 0 : 60);
    for (int i=0; i<warmup; ++i) world.stepPhysics(tickSize, options);
    auto command = [&]() {
        if (performance || crowded) {
            for (int i=0; i<robots*2; ++i)
                world.robots[i]->setSpeed(0.15, (i%2 ? 1 : -1)*0.05, 0.1);
        } else if (scenario == "robot-drive") world.robots[0]->setSpeed(1.5, 0, 0);
        else if (scenario == "robot-turn") world.robots[0]->setSpeed(3.0, 0, 8.0);
    };
    int steps = performance ? 300 : (crowded || scenario == "ball-flight" ? 30 : 180);
    if (scenario == "ball-spin" || scenario == "ball-drop") steps = 90;
    if (requestedSteps > 0) steps = requestedSteps;
    const double radius = settings.BallRadius();
    if (ballCase) {
        dBodySetPosition(world.ball->body, -2, 0,
                         scenario == "ball-flight" || scenario == "ball-drop" ? 1 : radius);
        dBodySetLinearVel(world.ball->body, 0, 0, 0);
        dBodySetAngularVel(world.ball->body, 0, 0, 0);
        if (scenario == "ball-flight") dBodySetLinearVel(world.ball->body, 3, 0, 4);
        else if (scenario == "ball-rolling" || scenario == "ball-sliding") {
            dBodySetLinearVel(world.ball->body, initialSpeed, 0, 0);
            if (scenario == "ball-rolling") dBodySetAngularVel(world.ball->body, 0, initialSpeed/radius, 0);
        } else if (scenario == "ball-spin") dBodySetAngularVel(world.ball->body, 0, 0, 100);
    }
    dVector3 gravity;
    dWorldGetGravity(world.p->world, gravity);
    Diagnostics diagnostics;
    PWorld::StepStatistics contacts;
    QJsonArray trajectory;
    trajectory.append(record(world, 0));
    double physicsMs = 0, stopTime = -1, firstBounceHeight = 0;
    double peakBallSpeed = 0, rollingSum = 0;
    unsigned rollingSamples = 0;
    bool bounced = false, reboundComplete = false;
    double previousVz = dBodyGetLinearVel(world.ball->body)[2];
    double previousVx = dBodyGetLinearVel(world.ball->body)[0];
    for (int frame=1; frame<=steps; ++frame) {
        command();
        const auto start = std::chrono::steady_clock::now();
        world.stepPhysics(tickSize, options, &contacts);
        physicsMs += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-start).count();
        diagnostics.observe(world);
        const double time = frame*tickSize;
        trajectory.append(record(world, time));
        const auto* velocity = dBodyGetLinearVel(world.ball->body);
        const auto* angular = dBodyGetAngularVel(world.ball->body);
        const auto* position = dBodyGetPosition(world.ball->body);
        const double speed = std::hypot(velocity[0],velocity[1]);
        peakBallSpeed = std::max(peakBallSpeed,speed);
        const bool stopped = scenario == "ball-spin" ? std::abs(angular[2])<0.1 : speed<0.02;
        if (stopped && stopTime<0 && std::abs(position[2]-radius)<0.001) stopTime=time;
        if (scenario == "ball-rolling" && time>=0.2 && time<=1.0) {
            rollingSum += (velocity[0]-previousVx)/tickSize;
            ++rollingSamples;
        }
        if (scenario == "ball-drop") {
            if (!bounced && previousVz<0 && velocity[2]>0) bounced=true;
            if (bounced && !reboundComplete) {
                firstBounceHeight=std::max(firstBounceHeight,double(position[2]));
                if (previousVz>0 && velocity[2]<=0) reboundComplete=true;
            }
        }
        previousVz=velocity[2]; previousVx=velocity[0];
    }
    const auto* position = dBodyGetPosition(world.ball->body);
    const double duration=steps*tickSize;
    QJsonObject metrics{{"max_joint_anchor_error_m", diagnostics.maxJoint},
        {"rms_joint_anchor_error_m", diagnostics.jointSamples ? std::sqrt(diagnostics.sumJointSquared/diagnostics.jointSamples) : 0},
        {"max_wheel_axis_error_deg", diagnostics.maxAxisDegrees},
        {"max_contact_penetration_m", contacts.maxPenetration}, {"detected_contacts",double(contacts.contacts)},
        {"max_body_linear_speed_m_s", diagnostics.maxLinearSpeed}, {"max_body_angular_speed_rad_s",diagnostics.maxAngularSpeed},
        {"min_chassis_z_m",robots ? diagnostics.minChassisZ : 0}, {"ball_runout_x_m",position[0]+2},
        {"ball_stop_time_s",stopTime}, {"ball_first_rebound_height_m",firstBounceHeight},
        {"mean_rolling_acceleration_m_s2",rollingSamples ? rollingSum/rollingSamples : 0},
        {"expected_matched_rolling_acceleration_m_s2",2*settings.BallFriction()*gravity[2]/1.4},
        {"expected_native_spin_stop_s",variant.rollingScale > 0 ? 100*0.4*radius*radius/
            (variant.rollingScale*2*settings.BallFriction()*radius*(-gravity[2])) : -1},
        {"peak_ball_horizontal_speed_m_s",peakBallSpeed}};
    if (scenario == "ball-flight") {
        metrics["flight_horizontal_error_m"]=std::abs(position[0]-(-2+3*duration));
        metrics["flight_vertical_error_m"]=std::abs(position[2]-(1+4*duration+0.5*gravity[2]*duration*duration));
    }
    QJsonObject parameters{{"robots_per_team",robots}, {"ball_radius_m",radius},
        {"ball_friction",settings.BallFriction()}, {"ball_bounce",settings.BallBounce()},
        {"rolling_scale",variant.rollingScale}, {"rolling_rho_m",variant.rollingScale*2*settings.BallFriction()*radius},
        {"cylinder_inertia_axis",variant.correctInertia ? 3 : 1},
        {"wheel_finite_rotation",variant.finiteWheels}, {"wheel_cfm",variant.wheelCfm>0 ? variant.wheelCfm : 0.002},
        {"effective_wheel_cfm",(variant.wheelCfm>0 ? variant.wheelCfm : 0.002)*complianceScale},
        {"world_erp",dWorldGetERP(world.p->world)}, {"world_cfm",dWorldGetCFM(world.p->world)},
        {"contact_layer_m",variant.layer}, {"contact_correction_m_s",variant.correctionSpeed},
        {"quickstep_iterations",int(variant.iterations)}, {"gravity_m_s2",-gravity[2]}};
    if (variant.systemDefaults) {
        const auto& surface = world.p->findSurface(world.ball, world.ground)->surface;
        parameters["rolling_rho_m"] = surface.rho;
        parameters["spin_rho_m"] = surface.rhoN;
        parameters["ball_rolling_deceleration_m_s2"] = settings.BallRollingDeceleration();
        parameters["advertised_rolling_acceleration_m_s2"] = settings.BallModelTwoPhaseAccRoll();
        parameters["ball_linear_damping"] = dBodyGetLinearDamping(world.ball->body);
        parameters["ball_angular_damping"] = dBodyGetAngularDamping(world.ball->body);
        parameters["ball_slip"] = settings.BallSlip();
        parameters["ball_mass_kg"] = settings.BallMass();
        parameters["field_length_m"] = settings.Field_Length();
        parameters["field_width_m"] = settings.Field_Width();
        parameters["initial_ball_speed_m_s"] = initialSpeed;
    }
    return {{"parameters",parameters}, {"steps",steps}, {"warmup_ticks",warmup},
        {"duration_s",duration}, {"physics_ms",physicsMs}, {"ms_per_tick",physicsMs/steps},
        {"metrics",metrics}, {"trajectory_columns",QJsonArray{"t_s","ball_x_m","ball_y_m","ball_z_m",
         "ball_vx_m_s","ball_vy_m_s","ball_vz_m_s","ball_wx_rad_s","ball_wy_rad_s","ball_wz_rad_s",
         "robot_x_m","robot_y_m","robot_z_m","robot_yaw_rad","robot_vx_m_s","robot_vy_m_s","robot_vz_m_s","robot_wz_rad_s"}},
        {"trajectory",trajectory}};
}
} // namespace

int main(int argc,char** argv)
{
    QApplication app(argc,argv);
    QString variantName="baseline", scenario="ball-flight";
    QString settingsFile;
    double rollingDeceleration = -1, initialSpeed = 2;
    unsigned substeps=5, seed=166;
    bool normalizeCompliance=false;
    int steps=0;
    for (int i=1; i<argc; ++i) {
        if (i+1>=argc) return 2;
        const QString option=QString::fromLocal8Bit(argv[i]);
        const QString value=QString::fromLocal8Bit(argv[++i]);
        if (option=="--variant") variantName=value;
        else if (option=="--scenario") scenario=value;
        else if (option=="--settings-file") settingsFile=value;
        else if (option=="--rolling-deceleration" || option=="--ball-speed") {
            bool valid = false;
            const double number = value.toDouble(&valid);
            if (!valid || !std::isfinite(number) || number < 0) return 2;
            if (option=="--rolling-deceleration") rollingDeceleration=number;
            else initialSpeed=number;
        }
        else {
            bool valid=false;
            const unsigned number=value.toUInt(&valid);
            if (!valid) return 2;
            if (option=="--substeps") substeps=number;
            else if (option=="--steps") steps=number;
            else if (option=="--seed") seed=number;
            else if (option=="--normalize-contact-stiffness") normalizeCompliance=number != 0;
            else return 2;
        }
    }
    const QStringList scenarios={"ball-flight","ball-rolling","ball-sliding","ball-spin","ball-drop",
        "torque-chassis","torque-wheel","robot-drive","robot-turn","robot-rest","crowded-contact",
        "perf-11","perf-16","perf-crowded"};
    if (!substeps || substeps>120 || !scenarios.contains(scenario)) return 2;
    ConfigWidget config(false, settingsFile);
    auto settings = config.simulationSettings();
    if (rollingDeceleration >= 0) settings.value_BallRollingDeceleration = rollingDeceleration;
    if (!dInitODE2(dInitFlagManualThreadCleanup) || !dAllocateODEDataForThread(dAllocateMaskAll)) return 2;
    int status=0;
    QJsonObject output{{"collision_model","chassis-hull-2026"}, {"variant",variantName}, {"scenario",scenario}, {"seed",int(seed)},
        {"substeps",int(substeps)}, {"normalized_contact_stiffness",normalizeCompliance},
        {"ode_version",QString::fromLatin1(dODE_VERSION)}};
    try {
        dRandSetSeed(seed);
        const auto result=runScenario(settings, variantFor(variantName), scenario, substeps, steps,
                                      normalizeCompliance, initialSpeed);
        for (auto i=result.begin(); i!=result.end(); ++i) output[i.key()]=i.value();
        output["success"]=true;
    } catch (const std::exception& error) {
        output["success"]=false;
        output["error"]=QString::fromLocal8Bit(error.what());
        status=1;
    }
    dCleanupODEAllDataForThread(); dCloseODE();
    std::puts(QJsonDocument(output).toJson(QJsonDocument::Compact).constData());
    return status;
}
