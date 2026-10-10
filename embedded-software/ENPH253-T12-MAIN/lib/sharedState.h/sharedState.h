#pragma once

#include <types.h>

// turret angle float plus its semaphorehandle_t mutex
// for phase 2, homing onto beacon

void sharedState_init(void);

namespace Turret {
void setTurretAngle(float angle);
float getTurretAngle(void);
} // namespace Turret