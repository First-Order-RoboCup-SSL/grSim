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

#ifndef PWORLD_H
#define PWORLD_H

#include "pobject.h"
#include <QVector>
#include <memory>
#include <functional>

class PSurface;
class OdeThreading;
class PWorld
{
public:
    struct StepStatistics {
        unsigned contacts = 0;
        unsigned integrationSteps = 0;
        double maxPenetration = 0;
        unsigned callbacks = 0, narrowPhaseCalls = 0, createdContacts = 0;
        bool measureTimings = false;
        double collisionSeconds = 0, narrowPhaseSeconds = 0;
        double contactCreationSeconds = 0, solverSeconds = 0;
        // Optional offline probes. Timing runs should leave these empty.
        std::function<void(const dContact&)> contactObserver;
        std::function<void()> beforeSolve;
    };
private:
    dJointGroupID contactgroup;
    QVector<PObject*> objects;
    QVector<PSurface*> surfaces;
    QVector<unsigned long> collisionCategories;
    bool collisionFiltering = true;
    bool collisionMasksDirty = true;
    void updateCollisionMasks();
    dReal delta_time;
    int **sur_matrix;
    int objects_count;
    std::unique_ptr<OdeThreading> threading;
    StepStatistics* activeStatistics = nullptr;
public:
    PWorld(dReal dt,dReal gravity, int robot_count);
    ~PWorld();
    void setGravity(dReal gravity);
    void addObject(PObject* o);
    void initAllObjects();
    PSurface* createSurface(PObject* o1,PObject* o2);
    PSurface* findSurface(PObject* o1,PObject* o2);
    // Configure between steps. Zero leaves unclassified objects permissive.
    // Masks conservatively admit every registered surface; the matrix still
    // decides which individual object pairs may create contacts.
    void setCollisionCategory(PObject* object, unsigned long category);
    void setCollisionFiltering(bool enabled);
    bool collisionFilteringEnabled() const { return collisionFiltering; }
    // Statistics accumulate across calls. Zero iterations selects dWorldStep.
    bool step(dReal dt=-1, unsigned quickStepIterations=0, StepStatistics* statistics=nullptr);
    // Zero runs on the caller. Each island uses one helper by default.
    bool setThreadCount(unsigned helpers);
    unsigned threadCount() const;
    std::vector<RenderPrimitive> renderPrimitives() const;
    void handleCollisions(dGeomID o1, dGeomID o2);    
    dWorldID world;
    dSpaceID space;
    int robot_count;
};

typedef bool PSurfaceCallback(dGeomID o1,dGeomID o2,PSurface* s,int robot_count);
class PSurface
{
public:
    PSurface();
    dSurfaceParameters surface{};
    bool isIt(dGeomID i1,dGeomID i2);
    dGeomID id1,id2;
    bool usefdir1;   //if true use fdir1 instead of ODE value
    dVector3 fdir1;  //fdir1 is a normalized vector tangent to friction force vector
    dVector3 contactPos,contactNormal;
    PSurfaceCallback* callback;
    void* callbackData = nullptr;
};
#endif // PWORLD_H
