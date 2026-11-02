#include "pcollisionsphere.h"

PCollisionSphere::PCollisionSphere(dBodyID attachedBody, dReal radius)
    : PObject(0, 0, 0, 0, 0, 0, 0), attachedBody(attachedBody), radius(radius)
{
    setVisibility(false);
}

void PCollisionSphere::init()
{
    geom = dCreateSphere(nullptr, radius);
    dGeomSetBody(geom, attachedBody);
    dSpaceAdd(space, geom);
}
