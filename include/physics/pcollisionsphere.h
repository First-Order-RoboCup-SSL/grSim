#ifndef PCOLLISIONSPHERE_H
#define PCOLLISIONSPHERE_H

#include "pobject.h"

// Geometry only: the attached chassis owns the ODE body.
class PCollisionSphere : public PObject {
public:
    PCollisionSphere(dBodyID attachedBody, dReal radius);
    void init() override;
private:
    dBodyID attachedBody;
    dReal radius;
};

#endif
