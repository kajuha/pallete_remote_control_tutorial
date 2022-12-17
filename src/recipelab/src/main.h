#pragma once

#include <ros/ros.h>
// #include <mecanum/MotorPosition.h>

#ifdef GAIN_TUNING
#include "AGVConfig.h"

#include <dynamic_reconfigure/server.h>
#include <recipelab/GainTuningConfig.h>

extern AGVModels LBox;
#endif

void queueString(std::string format, std::string node, std::string msg, double tsnow, double tscall, double tsdiff);

// extern int getMotorPosition(mecanum::MotorPosition& motorPosition);
// extern int setMotorPosition(mecanum::MotorPosition& motorPosition);