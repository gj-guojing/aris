﻿#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <limits>
#include <sstream>
#include <regex>
#include <limits>
#include <type_traits>
#include <array>

#include "aris/core/reflection.hpp"

#include "aris/dynamic/model.hpp"
#include "aris/dynamic/model_solver.hpp"
#include "aris/dynamic/mechanism_wafer_machine.hpp"

namespace aris::dynamic{
	struct WaferMachineInverseKinematicSolver::Imp {
	};
	auto WaferMachineInverseKinematicSolver::allocateMemory()->void {
		InverseKinematicSolver::allocateMemory();
	}
	auto WaferMachineInverseKinematicSolver::kinPos()->int {
		double output[3], input[3];
		model()->getOutputPos(output);
		kinPosPure(output, input, whichRoot(), nullptr);
		model()->setInputPos(input);

		// 每个关节 makI 相对于 makJ 的pe
		double joint_pe[5][6]{
			{0,0,input[0],0,0,0},
			{0,0,0,0,0,input[1]},
			{0,0,0,0,0,input[2]},
			{0,0,0,0,0,-2*input[2]},
			{0,0,0,0,0,input[2]},
		};

		for (int i = 0; i < 5; ++i) {
			model()->jointPool()[i].makI()->setPe(*model()->jointPool()[i].makJ(), joint_pe[i], "123");
		}

		for (auto& mot : model()->motionPool()) {
			mot.updP();
		}
		

		return 0;
	}
	auto WaferMachineInverseKinematicSolver::kinPosPure(const double* output, double* input, int which_root, const double* current_answer)->int {
		auto dh = dynamic_cast<aris::dynamic::MatrixVariable*>(model()->findVariable("dh"))->data().data();
		
		double a = dh[0];
		double b = dh[1];
		double c = dh[2];

		double x = output[0];
		double y = output[1];
		double z = output[2];

		

		// rtz //
		double r = std::sqrt(x * x + y * y);              // 理想的r 
		double theta = std::atan2(y, x) - aris::PI / 2;   // 理想的theta

		// 判断 r 的符号
		auto diff = theta - model()->motionPool()[2].mp();
		if (std::abs(aris::dynamic::s_put_into_period(diff, 0.0, 2 * aris::PI)) > aris::PI / 2) {
			r = -r;
			theta = std::abs(r) < 1e-10 ? model()->motionPool()[1].mp() : theta + aris::PI; // r 过小时，就用当前角度
		}
		else {
			theta = std::abs(r) < 1e-10 ? model()->motionPool()[1].mp() : theta; // r 过小时，就用当前角度
		}

		// 将theta 置入 【-pi，pi】中
		theta = aris::dynamic::s_put_into_period(theta, 0.0, 2 * aris::PI);

		


		input[0] = z - a;
		input[1] = theta;
		input[2] = (aris::PI / 2 - std::acos(std::min(std::max((r*r+b*b-c*c)/(2*std::abs(r)*b), -1.0), 1.0))) * aris::dynamic::s_sgn2(r);




		return 0;
	}
	WaferMachineInverseKinematicSolver::~WaferMachineInverseKinematicSolver() = default;
	WaferMachineInverseKinematicSolver::WaferMachineInverseKinematicSolver() :InverseKinematicSolver(1, 0.0), imp_(new Imp) {
		setWhichRoot(0);
		setRootNumber(2);
	}
	ARIS_DEFINE_BIG_FOUR_CPP(WaferMachineInverseKinematicSolver);

	struct WaferMachineForwardKinematicSolver::Imp {
	};
	auto WaferMachineForwardKinematicSolver::allocateMemory()->void {
		ForwardKinematicSolver::allocateMemory();
	}
	auto WaferMachineForwardKinematicSolver::kinPos()->int {
		double output[4], input[4];
		model()->getInputPos(input);
		kinPosPure(input, output, whichRoot(), nullptr);
		model()->setOutputPos(output);

		double joint_pe[5][6]{
			{0,0,input[0],0,0,0},
			{0,0,0,0,0,input[1]},
			{0,0,0,0,0,input[2]},
			{0,0,0,0,0,-2 * input[2]},
			{0,0,0,0,0,input[2]},
		};
		for (int i = 0; i < 5; ++i) {
			model()->jointPool()[i].makI()->setPe(*model()->jointPool()[i].makJ(), joint_pe[i], "123");
		}
		model()->generalMotionPool()[0].updP();

		//double pe[6];
		//for (auto& prt : model()->partPool()) {
		//	prt.getPe(pe, "123");
		//	dsp(1, 6, pe);
		//}


		//model()->getOutputPos(output);
		//dsp(1, 3, output);

		return 0;
	}
	auto WaferMachineForwardKinematicSolver::kinPosPure(const double* input, double* output, int which_root, const double *current_answer)->int {
		auto dh = dynamic_cast<aris::dynamic::MatrixVariable*>(model()->findVariable("dh"))->data().data();

		double a = dh[0];
		double b = dh[1];
		double c = dh[2];

		double z = a + input[0];
		double theta = input[1];

		// 已知一条临边，一条对边
		//
		// 余弦定理
		// cos(t) = (a*a+b*b-c*c)/(2*a*b)
		//    
		// 现在需要求 a
		// 
		// 2*ct*b*a = a*a + b*b -c*c
		// 
		// a*a - 2*ct*b*a + b*b-c*c == 0
		// 
		// 等效成 2 次方程

		double A = 1;
		double B = -2 * std::cos(aris::PI/2 - std::abs(input[2])) * b;
		double C = b * b - c * c;

		double r = (-B + std::sqrt(B * B - 4 * C)) / 2 * aris::dynamic::s_sgn2(input[2]);

		double x = r * std::cos(theta + aris::PI / 2);
		double y = r * std::sin(theta + aris::PI / 2);

		output[0] = x;
		output[1] = y;
		output[2] = z;

		return 0;
	}
	WaferMachineForwardKinematicSolver::~WaferMachineForwardKinematicSolver() = default;
	WaferMachineForwardKinematicSolver::WaferMachineForwardKinematicSolver() :ForwardKinematicSolver(1, 0.0), imp_(new Imp) {
		setWhichRoot(0);
		setRootNumber(1);
	}
	ARIS_DEFINE_BIG_FOUR_CPP(WaferMachineForwardKinematicSolver);



	struct WaferMachineInverseKinematicSolverTwoArm::Imp {
	};
	auto WaferMachineInverseKinematicSolverTwoArm::allocateMemory()->void {
		InverseKinematicSolver::allocateMemory();
	}
	auto WaferMachineInverseKinematicSolverTwoArm::kinPos()->int {
		double output[3], input[3];
		model()->getOutputPos(output);
		kinPosPure(output, input, whichRoot());
		model()->setInputPos(input);

		// 每个关节 makI 相对于 makJ 的pe
		double joint_pe[5][6]{
			{0,0,input[0],0,0,0},
			{0,0,0,0,0,input[1]},
			{0,0,0,0,0,input[2]},
			{0,0,0,0,0,-2 * input[2]},
			{0,0,0,0,0,input[2]},
		};

		for (int i = 0; i < 5; ++i) {
			model()->jointPool()[i].makI()->setPe(*model()->jointPool()[i].makJ(), joint_pe[i], "123");
		}

		for (auto& mot : model()->motionPool()) {
			mot.updP();
		}


		return 0;
	}
	auto WaferMachineInverseKinematicSolverTwoArm::kinPosPure(const double* output, double* input, int which_root, const double* current_answer)->int {
		auto dh = dynamic_cast<aris::dynamic::MatrixVariable*>(model()->findVariable("dh"))->data().data();

		double a = dh[0];
		double b = dh[1];
		double c = dh[2];

		double x = output[0];
		double y = output[1];
		double z = output[2];

		double x2 = output[3];
		double y2 = output[4];
		double z2 = output[5];

		// rtz //
		double r1 = std::sqrt(x * x + y * y);              // 理想的r1
		double r2 = std::sqrt(x2 * x2 + y2 * y2);              // 理想的r1
		double theta = std::atan2(y, x) - aris::PI / 2;   // 理想的theta

		// 判断 r 的符号
		auto diff = theta - model()->motionPool()[2].mp();
		if (std::abs(aris::dynamic::s_put_into_period(diff, 0.0, 2 * aris::PI)) > aris::PI / 2) {
			r1 = -r1;
			theta = std::abs(r1) < 1e-10 ? model()->motionPool()[1].mp() : theta + aris::PI; // r 过小时，就用当前角度
		}
		else {
			theta = std::abs(r1) < 1e-10 ? model()->motionPool()[1].mp() : theta; // r 过小时，就用当前角度
		}

		// 将theta 置入 【-pi，pi】中
		theta = aris::dynamic::s_put_into_period(theta, 0.0, 2 * aris::PI);




		input[0] = z - a;
		input[1] = theta;
		input[2] = (aris::PI / 2 - std::acos(std::min(std::max((r1 * r1 + b * b - c * c) / (2 * std::abs(r1) * b), -1.0), 1.0))) * aris::dynamic::s_sgn2(r1);




		return 0;
	}
	WaferMachineInverseKinematicSolverTwoArm::~WaferMachineInverseKinematicSolverTwoArm() = default;
	WaferMachineInverseKinematicSolverTwoArm::WaferMachineInverseKinematicSolverTwoArm() :InverseKinematicSolver(1, 0.0), imp_(new Imp) {
		setWhichRoot(0);
		setRootNumber(2);
	}
	ARIS_DEFINE_BIG_FOUR_CPP(WaferMachineInverseKinematicSolverTwoArm);

	struct WaferMachineForwardKinematicSolverTwoArm::Imp {
	};
	auto WaferMachineForwardKinematicSolverTwoArm::allocateMemory()->void {
		ForwardKinematicSolver::allocateMemory();
	}
	auto WaferMachineForwardKinematicSolverTwoArm::kinPos()->int {
		double output[4], input[4];
		model()->getInputPos(input);
		kinPosPure(input, output, whichRoot());
		model()->setOutputPos(output);

		double joint_pe[8][6]{
			{0,0,input[0],0,0,0},
			{0,0,0,0,0,input[1]},
			{0,0,0,0,0,input[2]},
			{0,0,0,0,0,-2 * input[2]},
			{0,0,0,0,0,input[2]},
			{0,0,0,0,0,input[3]},
			{0,0,0,0,0,-2 * input[3]},
			{0,0,0,0,0,input[3]},
		};
		for (int i = 0; i < 5; ++i) {
			model()->jointPool()[i].makI()->setPe(*model()->jointPool()[i].makJ(), joint_pe[i], "123");
		}
		model()->generalMotionPool()[0].updP();

		//double pe[6];
		//for (auto& prt : model()->partPool()) {
		//	prt.getPe(pe, "123");
		//	dsp(1, 6, pe);
		//}


		//model()->getOutputPos(output);
		//dsp(1, 3, output);

		return 0;
	}
	auto WaferMachineForwardKinematicSolverTwoArm::kinPosPure(const double* input, double* output, int which_root, const double* current_answer)->int {
		auto dh = dynamic_cast<aris::dynamic::MatrixVariable*>(model()->findVariable("dh"))->data().data();

		double a = dh[0];
		double b = dh[1];
		double c = dh[2];

		double z = a + input[0];
		double theta = input[1];

		// 已知一条临边，一条对边
		//
		// 余弦定理
		// cos(t) = (a*a+b*b-c*c)/(2*a*b)
		//    
		// 现在需要求 a
		// 
		// 2*ct*b*a = a*a + b*b -c*c
		// 
		// a*a - 2*ct*b*a + b*b-c*c == 0
		// 
		// 等效成 2 次方程

		double A = 1;
		double B = -2 * std::cos(aris::PI / 2 - std::abs(input[2])) * b;
		double C = b * b - c * c;

		double r = (-B + std::sqrt(B * B - 4 * C)) / 2 * aris::dynamic::s_sgn2(input[2]);

		double x = r * std::cos(theta + aris::PI / 2);
		double y = r * std::sin(theta + aris::PI / 2);

		output[0] = x;
		output[1] = y;
		output[2] = z;

		// arm2 
		B = -2 * std::cos(aris::PI / 2 - std::abs(input[3])) * b;
		r = (-B + std::sqrt(B * B - 4 * C)) / 2 * aris::dynamic::s_sgn2(input[2]);
		x = r * std::cos(theta + aris::PI / 2 + aris::PI);
		y = r * std::sin(theta + aris::PI / 2 + aris::PI);

		output[3] = x;
		output[4] = y;
		output[5] = z;

		return 0;
	}
	WaferMachineForwardKinematicSolverTwoArm::~WaferMachineForwardKinematicSolverTwoArm() = default;
	WaferMachineForwardKinematicSolverTwoArm::WaferMachineForwardKinematicSolverTwoArm() :ForwardKinematicSolver(1, 0.0), imp_(new Imp) {
		setWhichRoot(0);
		setRootNumber(1);
	}
	ARIS_DEFINE_BIG_FOUR_CPP(WaferMachineForwardKinematicSolverTwoArm);



	auto createModelWaferMachine(const WaferMachineParam&param)->std::unique_ptr<aris::dynamic::Model>{
		std::unique_ptr<aris::dynamic::Model> model = std::make_unique<aris::dynamic::Model>();
		
		model->setName("WaferMachineModel");

		////////////////////////////  DH  /////////////////////////////
		model->variablePool().add<aris::dynamic::MatrixVariable>("dh", aris::core::Matrix({ param.a, param.b, param.c }));
		model->variablePool().add<aris::dynamic::MatrixVariable>("tool0_pe", aris::core::Matrix(1, 6, param.tool0_pe));
		model->variablePool().add<aris::dynamic::StringVariable>("tool0_pe_type", param.tool0_pe_type.empty() ? std::string("321"): param.tool0_pe_type);
		model->variablePool().add<aris::dynamic::MatrixVariable>("base_pe", aris::core::Matrix(1, 6, param.base2ref_pe));
		model->variablePool().add<aris::dynamic::StringVariable>("base_pe_type", param.base2ref_pe_type.empty() ? std::string("321") : param.base2ref_pe_type);
		model->variablePool().add<aris::dynamic::MatrixVariable>("axis_range", aris::core::Matrix(1, 6, param.axis_range));
		model->variablePool().add<aris::dynamic::MatrixVariable>("install_method", aris::core::Matrix(1, 1, param.install_method));

		////////////////////////////  ENVIRONMENTS  /////////////////////////////
		const double gravity[6]{ 0.0, 0.0, -9.8, 0.0, 0.0, 0.0 };
		model->environment().setGravity(gravity);

		////////////////////////////  EE  /////////////////////////////
		const double axis_6_pe[6]{ param.b - param.c, 0, param.a, 0.0, 0.0, 0.0 };
		double axis_6_pm[16];
		double ee_i_pm[16], ee_i_wrt_axis_6_pm[16];
		double ee_j_pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };

		s_pe2pm(axis_6_pe, axis_6_pm, "321");
		s_pe2pm(param.tool0_pe, ee_i_wrt_axis_6_pm, param.tool0_pe_type.empty() ? "321" : param.tool0_pe_type.c_str());
		s_pm2pm(axis_6_pm, ee_i_wrt_axis_6_pm, ee_i_pm);

		////////////////////////////  PARTS  /////////////////////////////
		const double default_iv[10]{ 1,0,0,0,0,0,0,0,0,0 };
		auto &p1 = model->partPool().add<Part>("L1", param.iv_vec.size() == 6 ? param.iv_vec[0].data() : default_iv);
		auto &p2 = model->partPool().add<Part>("L2", param.iv_vec.size() == 6 ? param.iv_vec[1].data() : default_iv);
		auto &p3 = model->partPool().add<Part>("L3", param.iv_vec.size() == 6 ? param.iv_vec[2].data() : default_iv);
		auto &p4 = model->partPool().add<Part>("L4", param.iv_vec.size() == 6 ? param.iv_vec[3].data() : default_iv);
		auto &p5 = model->partPool().add<Part>("EE", param.iv_vec.size() == 6 ? param.iv_vec[4].data() : default_iv, ee_i_pm);

		////////////////////////////  JOINTS  /////////////////////////////
		const double j1_pos[3]{     0.0,                 0.0, param.a };
		const double j2_pos[3]{     0.0,                 0.0, param.a };
		const double j3_pos[3]{     0.0,                 0.0, param.a };
		const double j4_pos[3]{ param.b,                 0.0, param.a };
		const double j5_pos[3]{ param.b - param.c,       0.0, param.a };

		const double j1_axis[6]{ 0.0, 0.0, 1.0 };
		const double j2_axis[6]{ 0.0, 0.0, 1.0 };
		const double j3_axis[6]{ 0.0, 0.0, 1.0 };
		const double j4_axis[6]{ 0.0, 0.0, 1.0 };
		const double j5_axis[6]{ 0.0, 0.0, 1.0 };

		auto &j1 = model->addPrismaticJoint(p1, model->ground(), j1_pos, j1_axis);
		auto &j2 = model->addRevoluteJoint(p2, p1, j2_pos, j2_axis);
		auto &j3 = model->addRevoluteJoint(p3, p2, j3_pos, j3_axis);
		auto &j4 = model->addRevoluteJoint(p4, p3, j4_pos, j4_axis);
		auto &j5 = model->addRevoluteJoint(p5, p4, j5_pos, j5_axis);

		////////////////////////////  MOTIONS  /////////////////////////////
		auto &m1 = model->addMotion(j1);
		auto &m2 = model->addMotion(j2);
		auto &m3 = model->addMotion(j3);

		m1.setRotateRange(param.axis_range[0]);
		m2.setRotateRange(param.axis_range[1]);
		m3.setRotateRange(param.axis_range[2]);

		const double default_mot_frc[3]{0.0, 0.0, 0.0};

		m1.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[0].data() : default_mot_frc);
		m2.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[1].data() : default_mot_frc);
		m3.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[2].data() : default_mot_frc);

		////////////////////////////  EES  /////////////////////////////
		auto &makI = p5.addMarker("tool0");
		auto &makJ = model->ground().addMarker("wobj0", ee_j_pm);
		auto &ee = model->generalMotionPool().add<aris::dynamic::PointMotion>("ee", &makI, &makJ, false);

		////////////////////////////  INSTALL METHODS  /////////////////////////////
		double install_pm_relative[16];
		switch (param.install_method) {
		case 0:
			s_eye(4, install_pm_relative);
			break;
		case 1:
			s_eye(4, install_pm_relative);
			s_rmx(aris::PI, install_pm_relative, 4);
			break;
		case 2: {
			double pe[6]{ 0,0,0,aris::PI , aris::PI / 2 , 0, };
			s_pe2pm(pe, install_pm_relative, "123");
			break;
		}
		case 3: {
			s_eye(4, install_pm_relative);
			s_rmy(aris::PI / 2, install_pm_relative, 4);
			break;
		}
		default:
			THROW_FILE_LINE("INVALID value for install method");
		}
		double install_pm[16];
		s_pm_dot_pm(install_pm_relative, *j1.makJ()->prtPm(), install_pm);
		j1.makJ()->setPrtPm(install_pm);


		////////////////////////////  ROBOT POSITION  /////////////////////////////
		double robot_pm[16];
		s_pe2pm(param.base2ref_pe, robot_pm, param.base2ref_pe_type.empty() ? "321" : param.base2ref_pe_type.c_str());

		p1.setPm(s_pm_dot_pm(robot_pm, *p1.pm()));
		p2.setPm(s_pm_dot_pm(robot_pm, *p2.pm()));
		p3.setPm(s_pm_dot_pm(robot_pm, *p3.pm()));
		p4.setPm(s_pm_dot_pm(robot_pm, *p4.pm()));
		p5.setPm(s_pm_dot_pm(robot_pm, *p5.pm()));
		j1.makJ()->setPrtPm(s_pm_dot_pm(robot_pm, *j1.makJ()->prtPm()));
		ee.makJ()->setPrtPm(s_pm_dot_pm(robot_pm, *ee.makJ()->prtPm()));

		////////////////////////////  TOOLS WOBJS  /////////////////////////////
		for (int i = 1; i < 17; ++i)
			p5.addMarker("tool" + std::to_string(i), *ee.makI()->prtPm());
		for (int i = 1; i < 33; ++i) 
			model->ground().markerPool().add<aris::dynamic::Marker>("wobj" + std::to_string(i), *ee.makJ()->prtPm());

		////////////////////////////  SOLVERS  /////////////////////////////
		auto &inverse_kinematic = model->solverPool().add<aris::dynamic::WaferMachineInverseKinematicSolver>();
		auto &forward_kinematic = model->solverPool().add<aris::dynamic::WaferMachineForwardKinematicSolver>();
		auto &inverse_dynamic = model->solverPool().add<aris::dynamic::InverseDynamicSolver>();
		auto &forward_dynamic = model->solverPool().add<aris::dynamic::ForwardDynamicSolver>();

		////////////////////////////  TOPOLOGY  /////////////////////////////
		for (auto &m : model->motionPool())m.activate(true);
		for (auto &gm : model->generalMotionPool())gm.activate(false);
		for (auto &f : model->forcePool())f.activate(false);

		model->init();
		return model;
	}

	// 临时搭的，所有参数都不对，仅仅加了一根输入轴 //
	auto createModelWaferMachineTwoArms(const WaferMachineParam& param) -> std::unique_ptr<aris::dynamic::Model> {
		std::unique_ptr<aris::dynamic::Model> model = std::make_unique<aris::dynamic::Model>();

		model->setName("WaferMachineModelTwoArms");

		////////////////////////////  DH  /////////////////////////////
		model->variablePool().add<aris::dynamic::MatrixVariable>("dh", aris::core::Matrix({ param.a, param.b, param.c }));
		model->variablePool().add<aris::dynamic::MatrixVariable>("tool0_pe", aris::core::Matrix(1, 6, param.tool0_pe));
		model->variablePool().add<aris::dynamic::StringVariable>("tool0_pe_type", param.tool0_pe_type.empty() ? std::string("321") : param.tool0_pe_type);
		model->variablePool().add<aris::dynamic::MatrixVariable>("base_pe", aris::core::Matrix(1, 6, param.base2ref_pe));
		model->variablePool().add<aris::dynamic::StringVariable>("base_pe_type", param.base2ref_pe_type.empty() ? std::string("321") : param.base2ref_pe_type);
		model->variablePool().add<aris::dynamic::MatrixVariable>("axis_range", aris::core::Matrix(1, 6, param.axis_range));
		model->variablePool().add<aris::dynamic::MatrixVariable>("install_method", aris::core::Matrix(1, 1, param.install_method));

		////////////////////////////  ENVIRONMENTS  /////////////////////////////
		const double gravity[6]{ 0.0, 0.0, -9.8, 0.0, 0.0, 0.0 };
		model->environment().setGravity(gravity);

		////////////////////////////  EE  /////////////////////////////
		const double axis_6_pe[6]{ param.b - param.c, 0, param.a, 0.0, 0.0, 0.0 };
		double axis_6_pm[16];
		double ee_i_pm[16], ee_i_wrt_axis_6_pm[16];
		double ee_j_pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };

		s_pe2pm(axis_6_pe, axis_6_pm, "321");
		s_pe2pm(param.tool0_pe, ee_i_wrt_axis_6_pm, param.tool0_pe_type.empty() ? "321" : param.tool0_pe_type.c_str());
		s_pm2pm(axis_6_pm, ee_i_wrt_axis_6_pm, ee_i_pm);

		////////////////////////////  PARTS  /////////////////////////////
		const double default_iv[10]{ 1,0,0,0,0,0,0,0,0,0 };
		auto& p1 = model->partPool().add<Part>("L1", param.iv_vec.size() == 6 ? param.iv_vec[0].data() : default_iv);
		auto& p2 = model->partPool().add<Part>("L2", param.iv_vec.size() == 6 ? param.iv_vec[1].data() : default_iv);
		auto& p3 = model->partPool().add<Part>("L3", param.iv_vec.size() == 6 ? param.iv_vec[2].data() : default_iv);
		auto& p4 = model->partPool().add<Part>("L4", param.iv_vec.size() == 6 ? param.iv_vec[3].data() : default_iv);
		auto& p5 = model->partPool().add<Part>("EE", param.iv_vec.size() == 6 ? param.iv_vec[4].data() : default_iv, ee_i_pm);

		// arm2
		auto& p6 = model->partPool().add<Part>("L32", param.iv_vec.size() == 6 ? param.iv_vec[2].data() : default_iv);
		auto& p7 = model->partPool().add<Part>("L42", param.iv_vec.size() == 6 ? param.iv_vec[3].data() : default_iv);
		auto& p8 = model->partPool().add<Part>("EE2", param.iv_vec.size() == 6 ? param.iv_vec[4].data() : default_iv, ee_i_pm);

		////////////////////////////  JOINTS  /////////////////////////////
		const double j1_pos[3]{ 0.0,                 0.0, param.a };
		const double j2_pos[3]{ 0.0,                 0.0, param.a };
		const double j3_pos[3]{ 0.0,                 0.0, param.a };
		const double j4_pos[3]{ param.b,                 0.0, param.a };
		const double j5_pos[3]{ param.b - param.c,       0.0, param.a };

		// arm2
		const double j6_pos[3]{ 0.0,                 0.0, param.a };
		const double j7_pos[3]{ -param.b,                 0.0, param.a };
		const double j8_pos[3]{ -param.b + param.c,       0.0, param.a };

		const double j1_axis[6]{ 0.0, 0.0, 1.0 };
		const double j2_axis[6]{ 0.0, 0.0, 1.0 };
		const double j3_axis[6]{ 0.0, 0.0, 1.0 };
		const double j4_axis[6]{ 0.0, 0.0, 1.0 };
		const double j5_axis[6]{ 0.0, 0.0, 1.0 };

		// arm2
		const double j6_axis[6]{ 0.0, 0.0, 1.0 };
		const double j7_axis[6]{ 0.0, 0.0, 1.0 };
		const double j8_axis[6]{ 0.0, 0.0, 1.0 };

		auto& j1 = model->addPrismaticJoint(p1, model->ground(), j1_pos, j1_axis);
		auto& j2 = model->addRevoluteJoint(p2, p1, j2_pos, j2_axis);
		auto& j3 = model->addRevoluteJoint(p3, p2, j3_pos, j3_axis);
		auto& j4 = model->addRevoluteJoint(p4, p3, j4_pos, j4_axis);
		auto& j5 = model->addRevoluteJoint(p5, p4, j5_pos, j5_axis);

		// arm2
		auto& j6 = model->addRevoluteJoint(p6, p2, j6_pos, j6_axis);
		auto& j7 = model->addRevoluteJoint(p7, p6, j7_pos, j7_axis);
		auto& j8 = model->addRevoluteJoint(p8, p7, j8_pos, j8_axis);

		////////////////////////////  MOTIONS  /////////////////////////////
		auto& m1 = model->addMotion(j1);
		auto& m2 = model->addMotion(j2);
		auto& m3 = model->addMotion(j3);
		auto& m4 = model->addMotion(j6); // arm2

		m1.setRotateRange(param.axis_range[0]);
		m2.setRotateRange(param.axis_range[1]);
		m3.setRotateRange(param.axis_range[2]);
		m4.setRotateRange(param.axis_range[2]); // arm2

		const double default_mot_frc[3]{ 0.0, 0.0, 0.0 };

		m1.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[0].data() : default_mot_frc);
		m2.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[1].data() : default_mot_frc);
		m3.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[2].data() : default_mot_frc);
		m4.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[2].data() : default_mot_frc); // arm2

		////////////////////////////  EES  /////////////////////////////
		auto& makI = p5.addMarker("tool0");
		auto& makJ = model->ground().addMarker("wobj0", ee_j_pm);
		auto& ee = model->generalMotionPool().add<aris::dynamic::PointMotion>("ee", &makI, &makJ, false);
		
		auto& makI2 = p8.addMarker("tool0"); // arm2
		auto& ee2  = model->generalMotionPool().add<aris::dynamic::PointMotion>("ee2", &makI2, &makJ, false); // arm2

		////////////////////////////  INSTALL METHODS  /////////////////////////////
		double install_pm_relative[16];
		switch (param.install_method) {
		case 0:
			s_eye(4, install_pm_relative);
			break;
		case 1:
			s_eye(4, install_pm_relative);
			s_rmx(aris::PI, install_pm_relative, 4);
			break;
		case 2: {
			double pe[6]{ 0,0,0,aris::PI , aris::PI / 2 , 0, };
			s_pe2pm(pe, install_pm_relative, "123");
			break;
		}
		case 3: {
			s_eye(4, install_pm_relative);
			s_rmy(aris::PI / 2, install_pm_relative, 4);
			break;
		}
		default:
			THROW_FILE_LINE("INVALID value for install method");
		}
		double install_pm[16];
		s_pm_dot_pm(install_pm_relative, *j1.makJ()->prtPm(), install_pm);
		j1.makJ()->setPrtPm(install_pm);


		////////////////////////////  ROBOT POSITION  /////////////////////////////
		double robot_pm[16];
		s_pe2pm(param.base2ref_pe, robot_pm, param.base2ref_pe_type.empty() ? "321" : param.base2ref_pe_type.c_str());

		p1.setPm(s_pm_dot_pm(robot_pm, *p1.pm()));
		p2.setPm(s_pm_dot_pm(robot_pm, *p2.pm()));
		p3.setPm(s_pm_dot_pm(robot_pm, *p3.pm()));
		p4.setPm(s_pm_dot_pm(robot_pm, *p4.pm()));
		p5.setPm(s_pm_dot_pm(robot_pm, *p5.pm()));
		j1.makJ()->setPrtPm(s_pm_dot_pm(robot_pm, *j1.makJ()->prtPm()));
		ee.makJ()->setPrtPm(s_pm_dot_pm(robot_pm, *ee.makJ()->prtPm()));

		////////////////////////////  TOOLS WOBJS  /////////////////////////////
		for (int i = 1; i < 17; ++i)
			p5.addMarker("tool" + std::to_string(i), *ee.makI()->prtPm());
		for (int i = 1; i < 33; ++i)
			model->ground().markerPool().add<aris::dynamic::Marker>("wobj" + std::to_string(i), *ee.makJ()->prtPm());

		for (int i = 1; i < 17; ++i)
			p8.addMarker("tool" + std::to_string(i), *ee.makI()->prtPm()); // arm2

		////////////////////////////  SOLVERS  /////////////////////////////
		auto& inverse_kinematic = model->solverPool().add<aris::dynamic::WaferMachineInverseKinematicSolverTwoArm>();
		auto& forward_kinematic = model->solverPool().add<aris::dynamic::WaferMachineForwardKinematicSolverTwoArm>();
		auto& inverse_dynamic = model->solverPool().add<aris::dynamic::InverseDynamicSolver>();
		auto& forward_dynamic = model->solverPool().add<aris::dynamic::ForwardDynamicSolver>();

		////////////////////////////  TOPOLOGY  /////////////////////////////
		for (auto& m : model->motionPool())m.activate(true);
		for (auto& gm : model->generalMotionPool())gm.activate(false);
		for (auto& f : model->forcePool())f.activate(false);

		model->init();
		return model;
	}






	ARIS_REGISTRATION{
		aris::core::class_<WaferMachineInverseKinematicSolver>("WaferMachineInverseKinematicSolver")
			.inherit<InverseKinematicSolver>()
			;

		aris::core::class_<WaferMachineForwardKinematicSolver>("WaferMachineForwardKinematicSolver")
			.inherit<ForwardKinematicSolver>()
			;

		aris::core::class_<WaferMachineInverseKinematicSolverTwoArm>("WaferMachineInverseKinematicSolverTwoArm")
			.inherit<InverseKinematicSolver>()
			;

		aris::core::class_<WaferMachineForwardKinematicSolverTwoArm>("WaferMachineForwardKinematicSolverTwoArm")
			.inherit<ForwardKinematicSolver>()
			;
	}
}
