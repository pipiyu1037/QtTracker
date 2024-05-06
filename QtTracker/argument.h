#pragma once
#include <Kinect.h>
#include <utility>
#include <array>
#include <chrono>
#include <vector>
#include "Utils.hpp"

const int bodyCount = 6;
class Argument
{
public:
	//last tracked postion
	std::array<std::array<vec2f, JointType::JointType_Count>, bodyCount> lastPos;
	std::vector<std::vector<bool>> posInitialized = std::vector<std::vector<bool>>(bodyCount, std::vector<bool>(JointType::JointType_Count, false));

	//last tracked velocity
	std::array<std::array<vec2f, JointType::JointType_Count>, bodyCount> lastVelocity;
	std::vector<std::vector<bool>> velocityInitialied = std::vector<std::vector<bool>>(bodyCount, std::vector<bool>(JointType::JointType_Count, false));

	//last acceleration
	std::array<std::array<vec2f, JointType::JointType_Count>, bodyCount> lastA;

	//last tracked time
	std::array<std::array<std::chrono::steady_clock::time_point, JointType::JointType_Count>, bodyCount> lastJointTime;

	//last tracked frame
	std::array<std::array<int64_t, JointType::JointType_Count>, bodyCount> lastJointFrame;

	float lastLeftAngle = 0.f;
	float lastRightAngle = 0.f;
	bool lastLeftAngleInit = false;
	bool lastRightAngleInit = false;

	float leftAngleV;
	float rightAngleV;

	std::chrono::steady_clock::time_point lastLeftAngleTime;
	std::chrono::steady_clock::time_point lastRightAngleTime;
};

