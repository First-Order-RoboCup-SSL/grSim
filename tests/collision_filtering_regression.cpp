#include "physics/pball.h"
#include "physics/pworld.h"
#include <cstdio>
#include <stdexcept>

namespace {
void require(bool condition, const char* description)
{
    if (!condition) throw std::runtime_error(description);
}

unsigned callbacks(PWorld& world)
{
    unsigned count = 0;
    dSpaceCollide(world.space, &count, [](void* data, dGeomID, dGeomID) {
        ++*static_cast<unsigned*>(data);
    });
    return count;
}

void reset(PBall* ball, double x)
{
    dBodySetPosition(ball->body, x, 0, 0);
    dBodySetLinearVel(ball->body, 0, 0, 0);
    dBodySetAngularVel(ball->body, 0, 0, 0);
}

void checkFiltering()
{
    PWorld world(1.0 / 300, 0, 0);
    auto* first = new PBall(0, 0, 0, 0.02, 1, 1, 1, 1);
    auto* second = new PBall(0.02, 0, 0, 0.02, 1, 1, 1, 1);
    world.addObject(first);
    world.addObject(second);
    world.initAllObjects();
    world.setCollisionCategory(first, 1);
    world.setCollisionCategory(second, 2);
    world.setCollisionFiltering(true);
    require(callbacks(world) == 0, "unsupported pair must be rejected before the callback");

    auto* original = world.createSurface(first, second);
    PWorld::StepStatistics lateSurface;
    require(world.step(-1, 0, &lateSurface), "step failed after adding a surface");
    require(lateSurface.narrowPhaseCalls == 1 && lateSurface.createdContacts > 0,
            "new surfaces must refresh masks before the next collision pass");

    auto* unknown = new PBall(0.01, 0, 0, 0.02, 1, 1, 1, 1);
    world.addObject(unknown);
    world.initAllObjects();
    require(world.findSurface(first, second) == original, "matrix rebuild lost an existing surface");
    world.createSurface(first, unknown);
    world.setCollisionFiltering(true);
    reset(first, 0);
    reset(second, 0.02);
    require(dGeomGetCategoryBits(unknown->geom) == ~0UL &&
            dGeomGetCollideBits(unknown->geom) == ~0UL,
            "unclassified objects must remain permissive");
    require(callbacks(world) == 3, "unknown objects must pass either direction of ODE filtering");
    PWorld::StepStatistics unclassified;
    require(world.step(-1, 0, &unclassified), "step failed with unclassified object");
    require(unclassified.callbacks == 3 && unclassified.narrowPhaseCalls == 2,
            "surface matrix must still reject unsupported object pairs");

    world.setCollisionCategory(unknown, 4);
    world.setCollisionFiltering(true);
    reset(first, 0);
    reset(second, 0.02);
    reset(unknown, 0.01);
    require(callbacks(world) == 2, "classification must reject the remaining unsupported pair");
    world.setCollisionFiltering(false);
    require(callbacks(world) == 3, "disabling filtering must restore all callbacks");
    world.setCollisionFiltering(true);
    require(callbacks(world) == 2, "reenabling filtering must restore derived masks");

    world.createSurface(second, unknown);
    PWorld::StepStatistics allPairs;
    require(world.step(-1, 0, &allPairs), "step failed after expanding the graph");
    require(allPairs.narrowPhaseCalls == 3 && allPairs.createdContacts == 3,
            "late surface graph changes must preserve every allowed contact");

    bool rejected = false;
    try { world.setCollisionCategory(nullptr, 1); }
    catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "categories must reject objects outside this world");
}
} // namespace

int main()
{
    if (!dInitODE2(dInitFlagManualThreadCleanup)) return 1;
    if (!dAllocateODEDataForThread(dAllocateMaskAll)) { dCloseODE(); return 1; }
    int status = 0;
    try {
        checkFiltering();
        std::puts("PASS collision filtering, late surfaces, matrix growth and unknown objects");
    } catch (const std::exception& error) {
        std::fprintf(stderr, "FAIL %s\n", error.what());
        status = 1;
    }
    dCleanupODEAllDataForThread();
    dCloseODE();
    return status;
}
