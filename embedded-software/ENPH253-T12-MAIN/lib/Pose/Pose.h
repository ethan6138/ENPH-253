#pragma once

#include <types.h>

void pose_init(void);
void setPose(void); 

Pose::PoseState getPose(void);
void resetPose(Pose::ResetPoint point);

bool getRampBottomDetected(void);
bool getRampTopDetected(void); 