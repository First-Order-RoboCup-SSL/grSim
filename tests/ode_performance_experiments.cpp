// Offline experiments using the production masks and optional threading controls.
#include <QApplication>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <numeric>
#include <set>
#include <stdexcept>
#include "configwidget.h"
#include "sslworld.h"

namespace {
using Clock = std::chrono::steady_clock;
constexpr double tick = 1.0/60;
struct Options {
    QString scenario = "spaced-6", traceFile, perfControl, perfAck;
    unsigned helpers = 0, islands = 1, preparation = 1;
    int steps = 300, warmup = 60;
    bool masks = true, profile = false, audit = false, replay = false;
};
struct Object { PObject* object; unsigned category; };

std::vector<Object> objectsOf(SSLWorld& world)
{
    std::vector<Object> objects{{world.ball,0}, {world.ground,1}};
    for (auto* wall : world.walls) objects.push_back({wall,2});
    for (int i=0; i<2*world.cfg->Robots_Count(); ++i) {
        auto* robot = world.robots[i];
        objects.push_back({robot->chassis,3});
        objects.push_back({robot->ballHull,7});
        objects.push_back({robot->dummy,4});
        objects.push_back({robot->kicker->box,5});
        for (auto* wheel : robot->wheels) objects.push_back({wheel->cyl,6});
    }
    std::sort(objects.begin(), objects.end(), [](const Object& a, const Object& b) {
        return a.object->id < b.object->id;
    });
    if (objects.size() != size_t(dSpaceGetNumGeoms(world.p->space)))
        throw std::runtime_error("Unclassified collision geometry");
    return objects;
}

QJsonObject configureMasks(SSLWorld& world, const std::vector<Object>& objects, bool enable)
{
    std::array<unsigned long,8> allowed{};
    unsigned registered = 0, rejected = 0, unsupportedPassing = 0;
    world.p->setCollisionFiltering(enable);
    for (const auto& entry : objects) {
        if (enable && dGeomGetCategoryBits(entry.object->geom) != (1UL << entry.category))
            throw std::runtime_error("Production collision category is missing or incorrect");
        allowed[entry.category] = dGeomGetCollideBits(entry.object->geom);
    }
    // Exhaust the actual object-pair graph, not just the eight-class table.
    for (size_t i=0; i<objects.size(); ++i) for (size_t j=i+1; j<objects.size(); ++j) {
        const auto first = objects[i].object->geom, second = objects[j].object->geom;
        const bool passes = (dGeomGetCategoryBits(first)&dGeomGetCollideBits(second)) ||
                            (dGeomGetCategoryBits(second)&dGeomGetCollideBits(first));
        const bool surface = world.p->findSurface(objects[i].object, objects[j].object);
        if (surface) {
            ++registered;
            if (!passes) throw std::runtime_error("Mask removed an allowed surface pair");
        } else if (passes) ++unsupportedPassing;
        else ++rejected;
    }
    QJsonArray classMasks;
    for (auto mask : allowed) classMasks.append(int(mask));
    return {{"objects",int(objects.size())}, {"registered_pairs",int(registered)},
        {"unsupported_pairs_rejected",int(rejected)},
        {"unsupported_pairs_passing_masks",int(unsupportedPassing)},
        {"class_collide_masks",classMasks}};
}

void verifyBroadPhase(const std::vector<Object>& objects)
{
    // Probe the actual ODE mask path for every object pair. Overlapping spheres
    // make the outcome depend only on filtering, not on fixture placement.
    auto space=dSimpleSpaceCreate(nullptr);
    dSpaceSetManualCleanup(space,1);
    auto first=dCreateSphere(space,1), second=dCreateSphere(space,1);
    unsigned callbacks=0;
    auto callback=[](void* context,dGeomID,dGeomID) { ++*static_cast<unsigned*>(context); };
    for (size_t i=0; i<objects.size(); ++i) for (size_t j=i+1; j<objects.size(); ++j) {
        const auto a=objects[i].object->geom, b=objects[j].object->geom;
        dGeomSetCategoryBits(first,dGeomGetCategoryBits(a));
        dGeomSetCollideBits(first,dGeomGetCollideBits(a));
        dGeomSetCategoryBits(second,dGeomGetCategoryBits(b));
        dGeomSetCollideBits(second,dGeomGetCollideBits(b));
        const bool expected=(dGeomGetCategoryBits(a)&dGeomGetCollideBits(b)) ||
                            (dGeomGetCategoryBits(b)&dGeomGetCollideBits(a));
        callbacks=0; dSpaceCollide(space,&callbacks,callback);
        if (callbacks!=unsigned(expected)) {
            dSpaceDestroy(space);
            throw std::runtime_error("ODE broad-phase mask probe disagrees with object pair graph");
        }
    }
    // A negative control demonstrates that this probe reaches mask rejection.
    dGeomSetCollideBits(first,0); dGeomSetCollideBits(second,0);
    callbacks=0; dSpaceCollide(space,&callbacks,callback);
    dSpaceDestroy(space);
    if (callbacks) throw std::runtime_error("ODE mask rejection negative control failed");
}

void hashBytes(uint64_t& hash, const void* data, size_t count)
{
    const auto* bytes = static_cast<const unsigned char*>(data);
    for (size_t i=0; i<count; ++i) { hash ^= bytes[i]; hash *= UINT64_C(1099511628211); }
}
void hashReal(uint64_t& hash, dReal value) { hashBytes(hash, &value, sizeof(value)); }
void hashContact(uint64_t& hash, const dContact& contact)
{
    const auto& geom = contact.geom;
    for (auto id : {geom.g1, geom.g2}) {
        const int index = *static_cast<const int*>(dGeomGetData(id));
        hashBytes(hash, &index, sizeof(index));
    }
    hashReal(hash, geom.depth);
    for (int i=0; i<3; ++i) { hashReal(hash,geom.pos[i]); hashReal(hash,geom.normal[i]); }
    const auto& s = contact.surface;
    hashBytes(hash,&s.mode,sizeof(s.mode));
    for (double value : {s.mu,s.mu2,s.rho,s.rho2,s.rhoN,s.bounce,s.bounce_vel,
                         s.soft_erp,s.soft_cfm,s.motion1,s.motion2,s.motionN,s.slip1,s.slip2})
        hashReal(hash,value);
    if (s.mode & dContactFDir1) for (int i=0; i<3; ++i) hashReal(hash,contact.fdir1[i]);
}

struct Diagnostics {
    std::vector<dBodyID> bodies;
    std::map<dBodyID,size_t> index;
    double maxAnchor = 0, anchorSquares = 0, maxAxisDegrees = 0;
    double maxSpeed = 0, maxSpin = 0, minChassisZ = 1e9;
    uint64_t anchorSamples = 0, islandObservations = 0;
    size_t maxIsland = 0;
    double sumLargestIsland = 0, sumIslandCount = 0;
    std::ofstream trace;
    explicit Diagnostics(const std::vector<Object>& objects, const QString& filename)
    {
        for (const auto& entry : objects) if (entry.object->body) {
            index[entry.object->body] = bodies.size();
            bodies.push_back(entry.object->body);
        }
        if (!filename.isEmpty()) {
            trace.open(filename.toStdString(), std::ios::binary);
            if (!trace) throw std::runtime_error("Cannot create trace file");
        }
    }
    void observeIslands()
    {
        std::vector<size_t> parent(bodies.size()), size(bodies.size(),1);
        std::iota(parent.begin(),parent.end(),0);
        auto root = [&](size_t i) { while (parent[i]!=i) i=parent[i]; return i; };
        for (size_t i=0; i<bodies.size(); ++i) {
            if (!dBodyIsEnabled(bodies[i])) throw std::runtime_error("Unexpected disabled body");
            for (int j=0; j<dBodyGetNumJoints(bodies[i]); ++j) {
                const auto joint = dBodyGetJoint(bodies[i],j);
                if (!dJointIsEnabled(joint)) continue;
                const auto first = index.find(dJointGetBody(joint,0));
                const auto second = index.find(dJointGetBody(joint,1));
                // Static ground/walls do not connect otherwise independent islands.
                if (first==index.end() || second==index.end()) continue;
                auto a=root(first->second), b=root(second->second);
                if (a!=b) { parent[b]=a; size[a]+=size[b]; }
            }
        }
        size_t count=0, largest=0;
        for (size_t i=0; i<bodies.size(); ++i) if (root(i)==i) {
            ++count; largest=std::max(largest,size[i]);
        }
        ++islandObservations;
        maxIsland=std::max(maxIsland,largest);
        sumLargestIsland+=largest; sumIslandCount+=count;
    }
    void state()
    {
        for (auto body : bodies) {
            const auto* p=dBodyGetPosition(body);
            const auto* q=dBodyGetQuaternion(body);
            const auto* v=dBodyGetLinearVel(body);
            const auto* w=dBodyGetAngularVel(body);
            const double values[13]={p[0],p[1],p[2],q[0],q[1],q[2],q[3],v[0],v[1],v[2],w[0],w[1],w[2]};
            for (auto value : values) if (!std::isfinite(value)) throw std::runtime_error("Nonfinite body state");
            if (trace.is_open()) trace.write(reinterpret_cast<const char*>(values),sizeof(values));
            maxSpeed=std::max(maxSpeed,std::sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]));
            maxSpin=std::max(maxSpin,std::sqrt(w[0]*w[0]+w[1]*w[1]+w[2]*w[2]));
        }
        if (trace.is_open() && !trace) throw std::runtime_error("Failed to write body trace");
    }
    void observe(SSLWorld& world)
    {
        state();
        for (int i=0; i<world.cfg->Robots_Count()*2; ++i) {
            minChassisZ=std::min(minChassisZ,double(dBodyGetPosition(world.robots[i]->chassis->body)[2]));
            for (auto* wheel : world.robots[i]->wheels) {
                dVector3 a,b,axis;
                dJointGetHingeAnchor(wheel->joint,a); dJointGetHingeAnchor2(wheel->joint,b);
                double squared=0;
                for (int j=0; j<3; ++j) squared+=(a[j]-b[j])*(a[j]-b[j]);
                maxAnchor=std::max(maxAnchor,std::sqrt(squared)); anchorSquares+=squared; ++anchorSamples;
                dJointGetHingeAxis(wheel->joint,axis);
                const auto* rotation=dBodyGetRotation(wheel->cyl->body);
                const double dot=std::abs(axis[0]*rotation[2]+axis[1]*rotation[6]+axis[2]*rotation[10]);
                maxAxisDegrees=std::max(maxAxisDegrees,std::acos(std::min(1.0,dot))*180/3.141592653589793);
            }
        }
    }
    QJsonObject json() const
    {
        return {{"body_count",int(bodies.size())}, {"state_values_per_body",13},
            {"max_hinge_anchor_error_m",maxAnchor},
            {"rms_hinge_anchor_error_m",anchorSamples ? std::sqrt(anchorSquares/anchorSamples) : 0},
            {"max_wheel_axis_error_deg",maxAxisDegrees}, {"max_linear_speed_m_s",maxSpeed},
            {"max_angular_speed_rad_s",maxSpin}, {"min_chassis_z_m",bodies.size()>1 ? minChassisZ : 0},
            {"max_island_bodies",int(maxIsland)},
            {"mean_largest_island_bodies",islandObservations ? sumLargestIsland/islandObservations : 0},
            {"mean_island_count",islandObservations ? sumIslandCount/islandObservations : 0}};
    }
};

QJsonObject run(SimulationSettings settings, const Options& options)
{
    const bool ballCase=options.scenario.startsWith("ball-");
    const bool skill=options.scenario=="dribble-flat" || options.scenario=="dribble-chip";
    const bool pushing=options.scenario=="pushing-6";
    const bool crowded=pushing || options.scenario.startsWith("crowded-");
    const bool group=options.scenario.startsWith("group-");
    const int robots=ballCase ? 0 : skill ? 1 : group ? options.scenario.section('-',1).toInt() :
        options.scenario.endsWith("16") ? 16 : options.scenario.endsWith("-6") ? 6 : 11;
    settings.value_Robots_Count=robots; settings.value_DeltaTime=tick;
    RobotsFormation formation(FORMATION_INSIDE_1,&settings);
    SSLWorld world(nullptr,&settings,&formation,&formation);
    auto configureThreads=[&]() {
        if (!world.p->setThreadCount(options.helpers)) throw std::runtime_error("Requested helpers unavailable");
        dWorldSteppingThreadingParameters threading={};
        threading.param_set=dWSTP_WorldIslandsIterationMaxThreads|dWSTP_IslandSteppingMaxThreads|dWSTP_LCPSolvingMaxThreads;
        threading.world_islands_iteration_max_threads=options.islands;
        threading.island_stepping_max_threads=options.preparation;
        threading.lcp_solving_max_threads=1;
        dWorldSetSteppingThreadingParameters(world.p->world,&threading);
    };
    // Equal-input replays prepare their snapshot with the serial reference.
    if (!options.replay) configureThreads();
    const auto objects=objectsOf(world);
    const bool defaultFiltering=world.p->collisionFilteringEnabled();
    const auto maskAudit=configureMasks(world,objects,options.masks);
    if (options.audit) verifyBroadPhase(objects);
    Diagnostics diagnostics(objects,options.traceFile);
    // Resolve first-step team headings before capturing any fixed link pose.
    for (int i=0; i<robots*2; ++i) world.robots[i]->step();
    if (crowded || group) for (int i=0; i<robots*2; ++i) {
        if (pushing || (crowded && robots==6)) {
            // Six per team in a non-overlapping 4x3 grid. In the pushing case
            // each team faces the other and keeps driving into the shared front.
            const int team=i/robots, id=i%robots;
            const int column=pushing ? (team ? 2+id%2 : 1-id%2) : i%4;
            const int row=pushing ? id/2 : i/4;
            world.robots[i]->setXY((column-1.5)*0.185,(row-1)*0.185);
            if (pushing) world.robots[i]->setDir(team ? 180 : 0);
        } else {
            world.robots[i]->setXY((i%4)*(group ? 0.35 : 0.185)-0.6,
                                  (i/4)*(group ? 0.35 : 0.185)-0.6);
        }
    }
    if (skill) {
        world.robots[0]->setXY(0,0); world.robots[1]->setXY(3,3);
    }
    std::vector<dJointID> links;
    if (group) for (int i=1; i<robots*2; ++i) {
        auto joint=dJointCreateFixed(world.p->world,nullptr);
        dJointAttach(joint,world.robots[i-1]->chassis->body,world.robots[i]->chassis->body);
        dJointSetFixed(joint); links.push_back(joint);
    }
    dBodySetPosition(world.ball->body,4,-3,settings.BallRadius());
    PhysicsStepOptions physics;
    for (int i=0; i<options.warmup; ++i) world.stepPhysics(tick,physics);
    const double radius=settings.BallRadius();
    if (ballCase) {
        dBodySetPosition(world.ball->body,-2,0,options.scenario=="ball-flight" ? 1 : radius);
        if (options.scenario=="ball-flight") dBodySetLinearVel(world.ball->body,3,0,4);
        if (options.scenario=="ball-rolling") {
            dBodySetLinearVel(world.ball->body,2,0,0); dBodySetAngularVel(world.ball->body,0,2/radius,0);
        }
        if (options.scenario=="ball-spin") dBodySetAngularVel(world.ball->body,0,0,100);
    }
    if (skill) {
        auto* robot=world.robots[0];
        dReal dx,dy,dz,x,y,z;
        robot->chassis->getBodyDirection(dx,dy,dz);
        robot->kicker->box->getBodyPosition(x,y,z);
        const double offset=robot->settings.KickerThickness/2+radius+0.001;
        dBodySetPosition(world.ball->body,x+dx*offset,y+dy*offset,radius);
        dBodySetLinearVel(world.ball->body,0,0,0); dBodySetAngularVel(world.ball->body,0,0,0);
        if (!robot->kicker->isTouchingBall()) throw std::runtime_error("Dribble fixture misses kicker");
        robot->kicker->setRoller(1); robot->kicker->step();
        if (dBodyGetNumJoints(world.ball->body)!=1) throw std::runtime_error("Dribble hold not created");
    }
    auto command=[&](int frame) {
        for (int i=0; i<robots*2; ++i) {
            const bool idle=options.scenario.startsWith("idle-") ||
                (options.scenario.startsWith("stopstart-") && (frame/30)%2==0);
            world.robots[i]->setSpeed(idle ? 0 : pushing ? 0.5 : skill ? 0.3 : group && i>=robots ? -0.15 : 0.15,
                                     idle || skill || group || pushing ? 0 : (i%2 ? 1 : -1)*0.05,
                                     idle || skill || group || pushing ? 0 : 0.1);
        }
    };
    struct FrozenBody {
        dBodyID body;
        std::array<dReal,3> position, velocity, spin, force, torque;
        std::array<dReal,4> quaternion;
    };
    std::vector<FrozenBody> frozen;
    int replayPreparationTicks=0;
    if (options.replay) {
        replayPreparationTicks=group ? 2 : 120;
        for (int i=0; i<replayPreparationTicks; ++i) {
            command(i); world.stepPhysics(tick,physics);
        }
        command(replayPreparationTicks);
        for (int i=0; i<robots*2; ++i) world.robots[i]->step();
        for (auto body : diagnostics.bodies) {
            dBodySetAutoDisableFlag(body,0); dBodyEnable(body);
            FrozenBody state; state.body=body;
            std::copy_n(dBodyGetPosition(body),3,state.position.begin());
            std::copy_n(dBodyGetQuaternion(body),4,state.quaternion.begin());
            std::copy_n(dBodyGetLinearVel(body),3,state.velocity.begin());
            std::copy_n(dBodyGetAngularVel(body),3,state.spin.begin());
            std::copy_n(dBodyGetForce(body),3,state.force.begin());
            std::copy_n(dBodyGetTorque(body),3,state.torque.begin());
            frozen.push_back(state);
        }
        configureThreads();
    }
    PWorld::StepStatistics statistics;
    statistics.measureTimings=options.profile;
    uint64_t hash=UINT64_C(14695981039346656037);
    if (options.audit) {
        statistics.contactObserver=[&](const dContact& contact) { hashContact(hash,contact); };
        statistics.beforeSolve=[&]() { diagnostics.observeIslands(); };
    }
    QJsonArray timings,contactHashes,contactCounts,ballTrajectory;
    diagnostics.state();
    std::ofstream perfControl;
    std::ifstream perfAck;
    auto perfCommand=[&](const char* command) {
        perfControl << command << '\n' << std::flush;
        std::string acknowledgement;
        if (!perfControl || !std::getline(perfAck,acknowledgement) || acknowledgement!="ack")
            throw std::runtime_error("perf control acknowledgement failed");
    };
    if (!options.perfControl.isEmpty()) {
        perfControl.open(options.perfControl.toStdString());
        perfAck.open(options.perfAck.toStdString());
        perfCommand("enable");
    }
    double elapsed=0;
    for (int frame=0; frame<options.steps; ++frame) {
        if (options.replay) {
            // Body updates leave geometry dirty in worker-dependent order.
            // Clean first, then dirty every body in the same order, untimed.
            dSpaceClean(world.p->space);
            for (const auto& s : frozen) {
                dBodySetPosition(s.body,s.position[0],s.position[1],s.position[2]);
                dBodySetQuaternion(s.body,s.quaternion.data());
                dBodySetLinearVel(s.body,s.velocity[0],s.velocity[1],s.velocity[2]);
                dBodySetAngularVel(s.body,s.spin[0],s.spin[1],s.spin[2]);
                dBodySetForce(s.body,s.force[0],s.force[1],s.force[2]);
                dBodySetTorque(s.body,s.torque[0],s.torque[1],s.torque[2]);
            }
            hash=UINT64_C(14695981039346656037);
        } else {
            command(frame);
        }
        if (skill && frame==30) {
            auto* kicker=world.robots[0]->kicker;
            if (!kicker->isTouchingBall() || dBodyGetNumJoints(world.ball->body)!=1)
                throw std::runtime_error("Dribble lost hold before kick");
            kicker->kick(4,options.scenario=="dribble-chip" ? 3 : 0);
            if (dBodyGetNumJoints(world.ball->body)!=0) throw std::runtime_error("Kick did not release hold");
            kicker->setRoller(0);
        }
        const unsigned contactsBefore=statistics.createdContacts;
        const auto start=Clock::now();
        auto* probes=options.profile || options.audit ? &statistics : nullptr;
        if (options.replay) {
            if (!world.p->step(tick/5,0,probes)) throw std::runtime_error("Frozen ODE substep failed");
        } else world.stepPhysics(tick,physics,probes);
        const double ms=std::chrono::duration<double,std::milli>(Clock::now()-start).count();
        elapsed+=ms; timings.append(ms);
        if (options.audit) {
            diagnostics.observe(world);
            contactHashes.append(QString::number(hash,16));
            contactCounts.append(int(statistics.createdContacts-contactsBefore));
            QJsonArray row;
            for (int i=0; i<3; ++i) row.append(dBodyGetPosition(world.ball->body)[i]);
            for (int i=0; i<3; ++i) row.append(dBodyGetLinearVel(world.ball->body)[i]);
            for (int i=0; i<3; ++i) row.append(dBodyGetAngularVel(world.ball->body)[i]);
            ballTrajectory.append(row);
        }
    }
    // No later physics depends on the disable acknowledgement. Flush before
    // serializing results so only a small stop-command boundary can be sampled.
    if (perfControl.is_open()) perfControl << "disable\n" << std::flush;
    for (auto link : links) dJointDestroy(link);
    QJsonObject output{{"collision_model","chassis-hull-2026"}, {"ode_version",QString::fromLatin1(dODE_VERSION)},
        {"configuration",QString::fromLatin1(dGetConfiguration())},
        {"scenario",options.scenario}, {"robots_per_team",robots}, {"masks",options.masks},
        {"default_filtering_enabled",defaultFiltering},
        {"helpers",int(options.helpers)}, {"island_limit",int(options.islands)},
        {"preparation_limit",int(options.preparation)}, {"lcp_limit",1},
        {"steps",options.steps}, {"warmup_ticks",options.warmup}, {"substeps",options.replay ? 1 : 5},
        {"replay",options.replay}, {"replay_preparation_ticks",replayPreparationTicks},
        {"seed",166}, {"profile",options.profile}, {"audit",options.audit},
        {"ms_per_tick",elapsed/options.steps}, {"tick_ms",timings}, {"mask_audit",maskAudit},
        {"callbacks",double(statistics.callbacks)}, {"narrow_phase_calls",double(statistics.narrowPhaseCalls)},
        {"detected_contacts",double(statistics.contacts)}, {"created_contacts",double(statistics.createdContacts)},
        {"max_penetration_m",statistics.maxPenetration},
        {"collision_ms_per_tick",1000*statistics.collisionSeconds/options.steps},
        {"narrow_phase_ms_per_tick",1000*statistics.narrowPhaseSeconds/options.steps},
        {"contact_creation_ms_per_tick",1000*statistics.contactCreationSeconds/options.steps},
        {"solver_ms_per_tick",1000*statistics.solverSeconds/options.steps}};
    if (options.audit) {
        output["diagnostics"]=diagnostics.json(); output["contact_hashes"]=contactHashes;
        output["contact_counts"]=contactCounts; output["ball_trajectory"]=ballTrajectory;
        output["trace_file"]=options.traceFile;
    }
    return output;
}
} // namespace

int main(int argc,char** argv)
{
    QApplication app(argc,argv);
    Options options;
    for (int i=1; i<argc; ++i) {
        if (i+1>=argc) return 2;
        const QString key=QString::fromLocal8Bit(argv[i]), value=QString::fromLocal8Bit(argv[++i]);
        if (key=="--scenario") options.scenario=value;
        else if (key=="--trace-file") options.traceFile=value;
        else if (key=="--perf-control") options.perfControl=value;
        else if (key=="--perf-ack") options.perfAck=value;
        else {
            bool valid=false;
            const unsigned number=value.toUInt(&valid);
            if (!valid) return 2;
            if (key=="--helpers") options.helpers=number;
            else if (key=="--islands") options.islands=number;
            else if (key=="--prepare") options.preparation=number;
            else if (key=="--steps") { if (number>10000) return 2; options.steps=int(number); }
            else if (key=="--warmup") { if (number>10000) return 2; options.warmup=int(number); }
            else if (key=="--masks") options.masks=number!=0;
            else if (key=="--profile") options.profile=number!=0;
            else if (key=="--audit") options.audit=number!=0;
            else if (key=="--replay") options.replay=number!=0;
            else return 2;
        }
    }
    const QStringList scenarios={"spaced-6","crowded-6","pushing-6","spaced-11","spaced-16","crowded-11","crowded-16",
        "idle-11","stopstart-11","group-4","group-8","group-11","group-16","ball-flight","ball-rolling",
        "ball-spin","dribble-flat","dribble-chip"};
    if (!scenarios.contains(options.scenario) || options.helpers>16 || !options.islands ||
        options.islands>16 || !options.preparation || options.preparation>16 || options.steps<1 ||
        (!options.traceFile.isEmpty() && !options.audit) ||
        (options.replay && !options.scenario.contains('-')) ||
        (options.replay && (options.scenario.startsWith("ball-") || options.scenario.startsWith("dribble-"))) ||
        (options.perfControl.isEmpty()!=options.perfAck.isEmpty())) return 2;
    ConfigWidget config(false);
    if (options.scenario.endsWith("-6")) config.v_Division->setString("Division B");
    auto settings=config.simulationSettings();
    if (!dInitODE2(dInitFlagManualThreadCleanup) || !dAllocateODEDataForThread(dAllocateMaskAll)) return 2;
    int status=0;
    QJsonObject output;
    try {
        dRandSetSeed(166);
        output=run(settings,options); output["success"]=true;
    } catch (const std::exception& error) {
        output["success"]=false; output["error"]=QString::fromLocal8Bit(error.what()); status=1;
    }
    dCleanupODEAllDataForThread(); dCloseODE();
    std::puts(QJsonDocument(output).toJson(QJsonDocument::Compact).constData());
    return status;
}
