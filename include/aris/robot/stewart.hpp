﻿#ifndef ARIS_ROBOT_STEWART_H_
#define ARIS_ROBOT_STEWART_H_

#include <memory>

#include <aris/dynamic/dynamic.hpp>
#include <aris/control/control.hpp>
#include <aris/plan/plan.hpp>

/// \brief 机器人命名空间
/// \ingroup aris
/// 
///
///
namespace aris::robot
{
	auto ARIS_API createModelStewart(const double *robot_pm = nullptr)->std::unique_ptr<aris::dynamic::Model>;
	auto ARIS_API createControllerStewart()->std::unique_ptr<aris::control::Controller>;
	auto ARIS_API createPlanRootStewart()->std::unique_ptr<aris::plan::PlanRoot>;
}


#endif