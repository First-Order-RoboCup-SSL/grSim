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

#include "pworld.h"
#include <QMap>
#include <ode/threading_impl.h>
#include <condition_variable>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <chrono>

static_assert(sizeof(dReal) == sizeof(double), "grSim requires double-precision ODE headers");

// Each world owns its scheduler, including in serial mode. ODE's implicit
// scheduler is shared globally and cannot serve concurrently stepped worlds.
class OdeThreading {
public:
    OdeThreading(dWorldID world, unsigned helpers) : world(world), helpers(helpers)
    {
        if (!dCheckConfiguration("ODE_EXT_threading")) {
            if (helpers) throw std::runtime_error("ODE has no threading interface");
            return;
        }
        try {
            if (helpers && !dCheckConfiguration("ODE_THR_builtin_impl"))
                throw std::runtime_error("ODE has no multithreaded implementation");
            implementation = helpers ? dThreadingAllocateMultiThreadedImplementation()
                                     : dThreadingAllocateSelfThreadedImplementation();
            if (!implementation) throw std::runtime_error("Could not allocate ODE scheduler");
            threads.reserve(helpers);
            for (unsigned i = 0; i < helpers; ++i) {
                threads.emplace_back([this]() {
                    if (!dAllocateODEDataForThread(dAllocateFlagBasicData)) {
                        std::lock_guard<std::mutex> lock(mutex);
                        ++failed;
                        ready.notify_all();
                        return;
                    }
                    dExternalThreadingServeMultiThreadedImplementation(implementation,
                        [](void* context) {
                            auto* self = static_cast<OdeThreading*>(context);
                            std::lock_guard<std::mutex> lock(self->mutex);
                            ++self->registered;
                            self->ready.notify_all();
                        }, this);
                    dCleanupODEAllDataForThread();
                });
            }
            {
                std::unique_lock<std::mutex> lock(mutex);
                ready.wait(lock, [this]() { return registered + failed == this->helpers; });
                if (failed) throw std::runtime_error("Could not allocate ODE helper data");
            }
            dWorldSetStepThreadingImplementation(world,
                dThreadingImplementationGetFunctions(implementation), implementation);
            attached = true;
            dWorldSteppingThreadingParameters parameters = {};
            parameters.param_set = dWSTP_WorldIslandsIterationMaxThreads |
                                   dWSTP_IslandSteppingMaxThreads | dWSTP_LCPSolvingMaxThreads;
            parameters.world_islands_iteration_max_threads = helpers ? helpers : 1;
            parameters.island_stepping_max_threads = 1;
            parameters.lcp_solving_max_threads = 1;
            dWorldSetSteppingThreadingParameters(world, &parameters);
        } catch (...) {
            stop();
            throw;
        }
    }
    ~OdeThreading() { stop(); }
    unsigned threadCount() const { return helpers; }
private:
    void stop()
    {
        if (!implementation) return;
        dThreadingImplementationShutdownProcessing(implementation);
        for (auto& thread : threads) if (thread.joinable()) thread.join();
        // Detachment frees the world's cached wait object through this scheduler.
        if (attached) dWorldSetStepThreadingImplementation(world, nullptr, nullptr);
        dThreadingFreeImplementation(implementation);
        implementation = nullptr;
    }
    dWorldID world;
    unsigned helpers;
    dThreadingImplementationID implementation = nullptr;
    bool attached = false;
    std::vector<std::thread> threads;
    std::mutex mutex;
    std::condition_variable ready;
    unsigned registered = 0, failed = 0;
};

bool PWorld::setThreadCount(unsigned helpers)
{
    if (helpers > 16) return false;
    if (threading && threading->threadCount() == helpers) return true;
    threading.reset();
    try {
        threading.reset(new OdeThreading(world, helpers));
        return true;
    } catch (const std::exception&) {
        threading.reset(new OdeThreading(world, 0));
        return false;
    }
}

unsigned PWorld::threadCount() const { return threading ? threading->threadCount() : 0; }

PSurface::PSurface()
{
  callback = NULL;
  usefdir1 = false;
  surface.mode = dContactApprox1;
  surface.mu = 0.5;
}
bool PSurface::isIt(dGeomID i1,dGeomID i2)
{
    return ((i1==id1) && (i2==id2)) || ((i1==id2) && (i2==id1));
}


void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
  ((PWorld*) data)->handleCollisions(o1,o2);
}


PWorld::PWorld(dReal dt,dReal gravity, int _robot_count)
{
    robot_count = _robot_count;
    world = dWorldCreate();
    space = dHashSpaceCreate (0);
    dSpaceSetManualCleanup(space, 1);
    contactgroup = dJointGroupCreate (0);
    dWorldSetGravity (world,0,0,-gravity);
    objects_count = 0;
    sur_matrix = NULL;
    delta_time = dt;
    try {
        threading.reset(new OdeThreading(world, 0));
    } catch (...) {
        dJointGroupDestroy(contactgroup);
        dSpaceDestroy(space);
        dWorldDestroy(world);
        throw;
    }
}

PWorld::~PWorld()
{
  threading.reset();
  dJointGroupDestroy (contactgroup);
  qDeleteAll(objects);
  qDeleteAll(surfaces);
  for (int i = 0; i < objects_count; ++i) delete[] sur_matrix[i];
  delete[] sur_matrix;
  dSpaceDestroy (space);
  dWorldDestroy (world);
}

void PWorld::setGravity(dReal gravity)
{
    dWorldSetGravity (world,0,0,-gravity);
}

void PWorld::handleCollisions(dGeomID o1, dGeomID o2)
{   
    using Clock = std::chrono::steady_clock;
    const bool measure = activeStatistics && activeStatistics->measureTimings;
    if (activeStatistics) ++activeStatistics->callbacks;
    PSurface* sur;
    int j=sur_matrix[*((int*)(dGeomGetData(o1)))][*((int*)(dGeomGetData(o2)))];
    if (j!=-1)
    {
        const int N = 10;
        dContact contact[N];
        const auto narrowStart = measure ? Clock::now() : Clock::time_point();
        int n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
        if (activeStatistics) {
            ++activeStatistics->narrowPhaseCalls;
            if (measure) activeStatistics->narrowPhaseSeconds +=
                std::chrono::duration<double>(Clock::now()-narrowStart).count();
            activeStatistics->contacts += n;
            for (int i = 0; i < n; ++i)
                activeStatistics->maxPenetration = std::max(activeStatistics->maxPenetration,
                                                           double(contact[i].geom.depth));
        }
        if (n > 0) {
          sur = surfaces[j];
          sur->contactPos   [0] = contact[0].geom.pos[0];
          sur->contactPos   [1] = contact[0].geom.pos[1];
          sur->contactPos   [2] = contact[0].geom.pos[2];
          sur->contactNormal[0] = contact[0].geom.normal[0];
          sur->contactNormal[1] = contact[0].geom.normal[1];
          sur->contactNormal[2] = contact[0].geom.normal[2];
          bool flag=true;
          if (sur->callback!=NULL) flag = sur->callback(o1,o2,sur,robot_count);
          const auto creationStart = measure ? Clock::now() : Clock::time_point();
          if (flag)
          for (int i=0; i<n; i++) {
              contact[i].surface = sur->surface;
              if (sur->usefdir1)
              {
                  contact[i].fdir1[0] = sur->fdir1[0];
                  contact[i].fdir1[1] = sur->fdir1[1];
                  contact[i].fdir1[2] = sur->fdir1[2];
                  contact[i].fdir1[3] = sur->fdir1[3];
              }
              if (activeStatistics) {
                  ++activeStatistics->createdContacts;
                  if (activeStatistics->contactObserver) activeStatistics->contactObserver(contact[i]);
              }
              dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);

              dJointAttach (c,
                            dGeomGetBody(contact[i].geom.g1),
                            dGeomGetBody(contact[i].geom.g2));
            }
          if (measure) activeStatistics->contactCreationSeconds +=
              std::chrono::duration<double>(Clock::now()-creationStart).count();
        }
    }

}

void PWorld::addObject(PObject* o)
{      
    int id = objects.count();
    o->id = id;
    if (o->world==NULL) o->world = world;
    if (o->space==NULL) o->space = space;
    o->init();
    dGeomSetData(o->geom,(void*)(&(o->id)));
    objects.append(o);
    collisionCategories.append(0);
    collisionMasksDirty = true;
}

void PWorld::initAllObjects()
{
    if (sur_matrix!=NULL)
    {
        for (int i=0;i<objects_count;i++)
            delete[] sur_matrix[i];
        delete[] sur_matrix;
    }
    objects_count = objects.count();
    const int c = objects_count;
    sur_matrix = new int* [c];
    for (int i=0;i<c;i++)
    {
        sur_matrix[i] = new int [c];    
        for (int j=0;j<c;j++)
            sur_matrix[i][j] = -1;
    }
    for (int i=0;i<surfaces.count();i++)
        sur_matrix[(*(int*)(dGeomGetData(surfaces[i]->id1)))][*((int*)(dGeomGetData(surfaces[i]->id2)))] =
        sur_matrix[(*(int*)(dGeomGetData(surfaces[i]->id2)))][*((int*)(dGeomGetData(surfaces[i]->id1)))] = i;
    collisionMasksDirty = true;
}

PSurface* PWorld::createSurface(PObject* o1,PObject* o2)
{
    PSurface *s = new PSurface();
    s->id1 = o1->geom;
    s->id2 = o2->geom;
    surfaces.append(s);
    sur_matrix[o1->id][o2->id] =
    sur_matrix[o2->id][o1->id] = surfaces.count() - 1;
    collisionMasksDirty = true;
    return s;
}

void PWorld::setCollisionCategory(PObject* object, unsigned long category)
{
    const int index = objects.indexOf(object);
    if (index < 0) throw std::invalid_argument("Collision category requires an owned object");
    collisionCategories[index] = category;
    collisionMasksDirty = true;
}

void PWorld::setCollisionFiltering(bool enabled)
{
    if (collisionFiltering != enabled) {
        collisionFiltering = enabled;
        collisionMasksDirty = true;
    }
    updateCollisionMasks();
}

void PWorld::updateCollisionMasks()
{
    if (!collisionMasksDirty) return;
    QMap<unsigned long, unsigned long> allowed;
    if (collisionFiltering) for (const auto* surface : surfaces) {
        const auto first = collisionCategories[*static_cast<int*>(dGeomGetData(surface->id1))];
        const auto second = collisionCategories[*static_cast<int*>(dGeomGetData(surface->id2))];
        if (first && second) {
            allowed[first] |= second;
            allowed[second] |= first;
        }
    }
    for (int i = 0; i < objects.count(); ++i) {
        const auto category = collisionCategories[i];
        const bool classified = collisionFiltering && category;
        dGeomSetCategoryBits(objects[i]->geom, classified ? category : ~0UL);
        dGeomSetCollideBits(objects[i]->geom, classified ? allowed.value(category) : ~0UL);
    }
    // ODE admits a pair when either direction matches. Unclassified objects
    // keep both masks open, so future object types cannot lose contacts.
    collisionMasksDirty = false;
}

PSurface* PWorld::findSurface(PObject* o1,PObject* o2)
{
    for (int i=0;i<surfaces.count();i++)
    {
        if (surfaces[i]->isIt(o1->geom,o2->geom)) return (surfaces[i]);
    }
    return NULL;
}

bool PWorld::step(dReal dt, unsigned quickStepIterations, StepStatistics* statistics)
{
    activeStatistics = statistics;
    if (statistics) ++statistics->integrationSteps;
    bool success = false;
    try {
        updateCollisionMasks();
        using Clock = std::chrono::steady_clock;
        const bool measure = statistics && statistics->measureTimings;
        const auto collisionStart = measure ? Clock::now() : Clock::time_point();
        dSpaceCollide (space,this,&nearCallback);
        if (measure) statistics->collisionSeconds +=
            std::chrono::duration<double>(Clock::now()-collisionStart).count();
        if (statistics && statistics->beforeSolve) statistics->beforeSolve();
        const auto solverStart = measure ? Clock::now() : Clock::time_point();
        const dReal stepSize = (dt<0) ? delta_time : dt;
        if (quickStepIterations) {
            dWorldSetQuickStepNumIterations(world, quickStepIterations);
            success = dWorldQuickStep(world, stepSize) != 0;
        } else success = dWorldStep(world, stepSize) != 0;
        if (measure) statistics->solverSeconds +=
            std::chrono::duration<double>(Clock::now()-solverStart).count();
    }
    catch (...) { success = false; }
    dJointGroupEmpty(contactgroup);
    activeStatistics = nullptr;
    return success;
}

std::vector<RenderPrimitive> PWorld::renderPrimitives() const
{
    std::vector<RenderPrimitive> result;
    result.reserve(objects.size());
    for (const auto* object : objects) result.push_back(object->renderPrimitive());
    return result;
}
