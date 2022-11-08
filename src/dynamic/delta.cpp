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
#include "aris/dynamic/delta.hpp"

namespace aris::dynamic {

	auto deltaInverse(const double *param, const double *ee_xyza, int which_root, double *input)->int {
		for (int i = 0; i < 3; ++i) {
			// 尺寸 //
			const double a = param[0 + i * 6];
			const double b = param[1 + i * 6];
			const double c = param[2 + i * 6];
			const double d = param[3 + i * 6];
			const double e = param[4 + i * 6];
			const double theta = param[5 + i * 6];

			// 此处将点转回到原x z平面，因此与构造delta的地方不太一样 //
			auto x =  ee_xyza[0] * std::cos(theta) + ee_xyza[1] * std::sin(theta);
			auto y = -ee_xyza[0] * std::sin(theta) + ee_xyza[1] * std::cos(theta);
			auto z =  ee_xyza[2];

			if (std::abs(d) < std::abs(y)) return -1;

			auto d1 = std::sqrt(d * d - y * y);
			auto k = x + e - a;
			auto l = std::sqrt(k * k + z * z);

			if (b + l < d1 || b + d1 < l || d1 + l < b)
				return -2;

			input[i] =
				((0x01 << i) & which_root) ?
				-std::atan2(z, k) + std::acos((b * b + l * l - d1 * d1) / b / l / 2.0) :
				-std::atan2(z, k) - std::acos((b * b + l * l - d1 * d1) / b / l / 2.0);
		}

		input[3] = ee_xyza[3];
		return 0;
	}
	auto deltaForward(const double *param, const double *input, int which_root, double *ee_xyza)->int {
		// 记p1 为 S1 S2 的中点位置，p2为末端到 S3 S4 中点的向量
		//
		// 于是对其中某一根支联，应有以下方程：
		//  || ee + p2 - p1 || = d
		// 
		// 若记：k = p2 - p1
		//
		// 继而：
		//   (x + k0)^2 + (y + k1)^2 + (z + k2)^2 = d^2
		//
		// 展开：
		//   x^2 + 2k0 x + k0^2 + y^2 + 2k1 x + k1^2 + z^2 + 2k2 z + k2^2 = d^2
		// 
		// 若记: t = x^2 + y^2 + z^2
		//       s = d^2 - k0^2 - k1^2 - k2^2
		//       k = 2k
		//
		// 于是有：
		//   [k0 k1 k2] * [x; y; z] = s - t
		//
		// 对3根支联列方程：
		//   [ k00 k01 k02 ]   [ x ]   [ s1 - t ]
		//   | k10 k11 k12 | * | y | = | s2 - t |
		//   [ k20 k21 k22 ]   [ z ]   [ s3 - t ]
		//
		// 【CASE 1】：K矩阵不奇异
		// 记做：
		//   K * x = b - t
		// 其中 x^T * x = t
		//
		// 于是：
		//    x = K^-1 * (b - t) = [ g1 + h1 * t ]
		//                         | g2 + h2 * t |
		//                         [ g3 + h3 * t ]
		//
		// 继而有：
		//    t = (h1^2 + h2^2 + h3^2)t^2 + (2 h1 g1 + 2 h1 g1 + 2 h1 g1)t + (g1^2 + g2^2 + g3^2)
		//
		// 于是有：
		//    A t^2 + B t + C = 0
		// 其中：
		//    A：(h1^2 + h2^2 + h3^2)
		//    B: (2 h1 g1 + 2 h1 g1 + 2 h1 g1) - 1
		//    C: (g1^2 + g2^2 + g3^2)
		//
		// 用以上1元2次方程求解出t
		//
		// 进一步的，有：
		// [x; y; z] = g + h * t
		// 
		// 
		// 
		// 【CASE 2】：K奇异
		// 此时先对 K 做 svd 分解，有：
		// U * S * V^T * x = s - t
		// 
		// 两侧同乘 QT：
		// S * V^T * x = UT * (s - t)
		// 
		// 令 V^T * x = y 则有：
		// S * y = UT * s - UT * [1;1;1] * t
		// 
		// 由于 K 奇异，因此将 R写成：
		// [ d1      ]
		// [    d2   ]
		// [       0 ]
		//
		// 令 UT * s = b
		//    UT * [1;1;1] = [u1;u2;u3]
		// 
		// 于是方程变为：
		// [ d1     u1 ] * [ y1 ] = b
		// [    d2  u2 ]   | y2 |
		// [        u3 ]   [ t  ]
		// 
		// 其中 y3 = +- sqrt(t - y1*y1 - y2*y2) 
		// 
		// 根据 which_root 选择其符号，一般来说，在 which_root 为 0 时，希望原值 x3 为负，因此根据which_root 与 V[3,3]的符号共同确定
		// 
		// 
		// 
		
		// 根据 p1 & p2 计算k
		double p1[9], p2[9], k[9], s[3];
		for (int i = 0; i < 3; ++i) {
			const double &a = param[0 + i * 6];
			const double &b = param[1 + i * 6];
			const double &c = param[2 + i * 6];
			const double &d = param[3 + i * 6];
			const double &e = param[4 + i * 6];
			const double &theta = param[5 + i * 6];

			p1[0 + i * 3] = (a + std::cos(input[i]) * b) * std::cos(theta);
			p1[1 + i * 3] = (a + std::cos(input[i]) * b) * std::sin(theta);
			p1[2 + i * 3] = -std::sin(input[i]) * b;

			p2[0 + i * 3] = e * std::cos(theta);
			p2[1 + i * 3] = e * std::sin(theta);
			p2[2 + i * 3] = 0.0;

			k[0 + i * 3] = p2[0 + i * 3] - p1[0 + i * 3];
			k[1 + i * 3] = p2[1 + i * 3] - p1[1 + i * 3];
			k[2 + i * 3] = p2[2 + i * 3] - p1[2 + i * 3];

			s[i] = d * d - k[0 + i * 3] * k[0 + i * 3] - k[1 + i * 3] * k[1 + i * 3] - k[2 + i * 3] * k[2 + i * 3];
		}

		// 随后k乘以2
		s_nv(9, 2.0, k);

		// 计算k的稚
		double inv_k[9], u[9], tau[3], tau2[3];
		aris::Size p[3], rank;
		s_householder_utp(3, 3, k, u, tau, p, rank);

		// 在所有的输入都为0的时候，k矩阵奇异，因此需要特殊处理：
		if (rank == 3) {
			// 计算 k 的逆
			s_householder_utp2pinv(3, 3, rank, u, tau, p, inv_k, tau2);
			
			// 计算 g&h
			double g[3], h[3]{ -inv_k[0] - inv_k[1] - inv_k[2], -inv_k[3] - inv_k[4] - inv_k[5], -inv_k[6] - inv_k[7] - inv_k[8], };
			s_mm(3, 1, 3, inv_k, s, g);

			// 计算 A,B,C
			double A = h[0] * h[0] + h[1] * h[1] + h[2] * h[2];
			double B = 2.0 * (h[0] * g[0] + h[1] * g[1] + h[2] * g[2]) - 1.0;
			double C = g[0] * g[0] + g[1] * g[1] + g[2] * g[2];

			// 此时有2个根 //
			double t;
			if (auto lambda = B * B - 4 * A * C; lambda < 0) return -1;
			else {
				// 这里用异或，找到z < 0的根
				t = ((which_root == 0) != (h[2] < 0)) ? (-B - std::sqrt(lambda)) / (2 * A) : (-B + std::sqrt(lambda)) / (2 * A);
				if (t < 0)return -2;
			}

			ee_xyza[0] = g[0] + h[0] * t;
			ee_xyza[1] = g[1] + h[1] * t;
			ee_xyza[2] = g[2] + h[2] * t;
			ee_xyza[3] = input[3];
			return 0;
		}
		else {
			// 计算针对 y 的方程
			double U[9], S[9], V[9];
			s_svd(3, 3, k, U, S, V);

			double ones[3]{ 1,1,1 }, b[3];
			s_mm(3, 1, 3, U, T(3), ones, 1, S + 2, 3);
			s_mm(3, 1, 3, U, T(3), s, 1, b, 1);

			// 求解关于 y 的方程
			double y[3], x[3];
			aris::Size p[3], rank;

			s_householder_utp(3, 3, S, u, tau, p, rank);
			s_householder_utp_sov(3, 3, 1, rank, u, tau, p, b, y);

			y[2] = ((which_root == 0) != (V[8] < 0) ? -1.0 : 1.0) * std::sqrt(y[2] - y[0] * y[0] - y[1] * y[1]);

			// 求解 x
			s_mm(3, 1, 3, V, 3, y, 1, x, 1);

			ee_xyza[0] = x[0];
			ee_xyza[1] = x[1];
			ee_xyza[2] = x[2];
			ee_xyza[3] = input[3];
			return 0;
		}




	}

	class DeltaInverseKinematicSolver :public aris::dynamic::InverseKinematicSolver {
	public:
		auto virtual kinPos()->int override {
			double input[4], output[4];

			auto dh = dynamic_cast<aris::dynamic::MatrixVariable*>(model()->findVariable("dh"))->data().data();

			model()->getOutputPos(output);
			if (auto ret = deltaInverse(dh, output, 0, input))
				return ret;

			// ee //
			double pe[6]{ output[0],output[1],output[2],output[3], 0.0, 0.0 }, pp[3];
			model()->generalMotionPool()[0].makI()->setPe(*model()->generalMotionPool()[0].makJ(), pe, "321");

			// up //
			s_fill(1, 3, 0.0, pe);
			pe[3] = -pe[3];
			model()->jointPool()[15].makJ()->setPe(*model()->jointPool()[15].makI(), pe, "321");

			// link1 //
			s_fill(1, 6, 0.0, pe);
			pe[5] = input[0];
			model()->jointPool()[0].makI()->setPe(*model()->jointPool()[0].makJ(), pe, "123");

			s_fill(1, 6, 0.0, pe);
			pe[5] = input[1];
			model()->jointPool()[5].makI()->setPe(*model()->jointPool()[5].makJ(), pe, "123");

			s_fill(1, 6, 0.0, pe);
			pe[5] = input[2];
			model()->jointPool()[10].makI()->setPe(*model()->jointPool()[10].makJ(), pe, "123");

			// link2&3 //
			s_fill(1, 6, 0.0, pe);
			pe[5] = PI / 2;
			model()->jointPool()[3].makI()->getPp(*model()->jointPool()[1].makJ(), pp);
			s_sov_ab(pp, pe + 3, "312");
			model()->jointPool()[1].makI()->setPe(*model()->jointPool()[1].makJ(), pe, "312");

			model()->jointPool()[4].makI()->getPp(*model()->jointPool()[2].makJ(), pp);
			s_sov_ab(pp, pe + 3, "312");
			model()->jointPool()[2].makI()->setPe(*model()->jointPool()[2].makJ(), pe, "312");

			model()->jointPool()[8].makI()->getPp(*model()->jointPool()[6].makJ(), pp);
			s_sov_ab(pp, pe + 3, "312");
			model()->jointPool()[6].makI()->setPe(*model()->jointPool()[6].makJ(), pe, "312");

			model()->jointPool()[9].makI()->getPp(*model()->jointPool()[7].makJ(), pp);
			s_sov_ab(pp, pe + 3, "312");
			model()->jointPool()[7].makI()->setPe(*model()->jointPool()[7].makJ(), pe, "312");

			model()->jointPool()[13].makI()->getPp(*model()->jointPool()[11].makJ(), pp);
			s_sov_ab(pp, pe + 3, "312");
			model()->jointPool()[11].makI()->setPe(*model()->jointPool()[11].makJ(), pe, "312");

			model()->jointPool()[14].makI()->getPp(*model()->jointPool()[12].makJ(), pp);
			s_sov_ab(pp, pe + 3, "312");
			model()->jointPool()[12].makI()->setPe(*model()->jointPool()[12].makJ(), pe, "312");

			for (auto &m : model()->motionPool()) m.updP();
			return 0;
		}

		DeltaInverseKinematicSolver() = default;
	};
	class DeltaForwardKinematicSolver :public aris::dynamic::ForwardKinematicSolver {
	public:
		auto virtual kinPos()->int override {
			double input[4], output[4];

			auto dh = dynamic_cast<aris::dynamic::MatrixVariable*>(model()->findVariable("dh"))->data().data();

			model()->getInputPos(input);
			if (auto ret = deltaForward(dh, input, 0, output))
				return ret;

			// ee //
			double pe[6]{ output[0],output[1],output[2],output[3], 0.0, 0.0 }, pp[3];
			model()->generalMotionPool()[0].makI()->setPe(*model()->generalMotionPool()[0].makJ(), pe, "321");

			// up //
			s_fill(1, 3, 0.0, pe);
			pe[3] = -pe[3];
			model()->jointPool()[15].makJ()->setPe(*model()->jointPool()[15].makI(), pe, "321");

			// link1 //
			s_fill(1, 6, 0.0, pe);
			pe[5] = input[0];
			model()->jointPool()[0].makI()->setPe(*model()->jointPool()[0].makJ(), pe, "123");

			s_fill(1, 6, 0.0, pe);
			pe[5] = input[1];
			model()->jointPool()[5].makI()->setPe(*model()->jointPool()[5].makJ(), pe, "123");

			s_fill(1, 6, 0.0, pe);
			pe[5] = input[2];
			model()->jointPool()[10].makI()->setPe(*model()->jointPool()[10].makJ(), pe, "123");

			// link2&3 //
			s_fill(1, 6, 0.0, pe);
			pe[5] = PI / 2;
			model()->jointPool()[3].makI()->getPp(*model()->jointPool()[1].makJ(), pp);
			s_sov_ab(pp, pe + 3, "312");
			model()->jointPool()[1].makI()->setPe(*model()->jointPool()[1].makJ(), pe, "312");

			model()->jointPool()[4].makI()->getPp(*model()->jointPool()[2].makJ(), pp);
			s_sov_ab(pp, pe + 3, "312");
			model()->jointPool()[2].makI()->setPe(*model()->jointPool()[2].makJ(), pe, "312");

			model()->jointPool()[8].makI()->getPp(*model()->jointPool()[6].makJ(), pp);
			s_sov_ab(pp, pe + 3, "312");
			model()->jointPool()[6].makI()->setPe(*model()->jointPool()[6].makJ(), pe, "312");

			model()->jointPool()[9].makI()->getPp(*model()->jointPool()[7].makJ(), pp);
			s_sov_ab(pp, pe + 3, "312");
			model()->jointPool()[7].makI()->setPe(*model()->jointPool()[7].makJ(), pe, "312");

			model()->jointPool()[13].makI()->getPp(*model()->jointPool()[11].makJ(), pp);
			s_sov_ab(pp, pe + 3, "312");
			model()->jointPool()[11].makI()->setPe(*model()->jointPool()[11].makJ(), pe, "312");

			model()->jointPool()[14].makI()->getPp(*model()->jointPool()[12].makJ(), pp);
			s_sov_ab(pp, pe + 3, "312");
			model()->jointPool()[12].makI()->setPe(*model()->jointPool()[12].makJ(), pe, "312");

			for (auto &m : model()->generalMotionPool()) m.updP();
			return 0;
		}

		DeltaForwardKinematicSolver() = default;
	};

	auto ARIS_API createModelDelta(const DeltaParam &param)->std::unique_ptr<aris::dynamic::Model> {
		DeltaFullParam full_param;
		full_param.a1 = full_param.a2 = full_param.a3 = param.a;
		full_param.b1 = full_param.b2 = full_param.b3 = param.b;
		full_param.c1 = full_param.c2 = full_param.c3 = param.c;
		full_param.d1 = full_param.d2 = full_param.d3 = param.d;
		full_param.e1 = full_param.e2 = full_param.e3 = param.e;

		full_param.theta1 = 0.0;
		full_param.theta2 = aris::PI * 2 / 3;
		full_param.theta3 = -aris::PI * 2 / 3;

		s_vc(6, param.tool0_pe, full_param.tool0_pe);
		full_param.tool0_pe_type = param.tool0_pe_type;

		s_vc(6, param.base2ref_pe, full_param.base2ref_pe);
		full_param.base2ref_pe_type = param.base2ref_pe_type;

		full_param.iv_vec = param.iv_vec;

		full_param.mot_frc_vec = param.mot_frc_vec;

		return createModelDelta(full_param);
	}
	auto ARIS_API createModelDelta(const DeltaFullParam &param)->std::unique_ptr<aris::dynamic::Model> {
		std::unique_ptr<aris::dynamic::Model> model(new aris::dynamic::Model);

		model->variablePool().add<aris::dynamic::MatrixVariable>("dh", aris::core::Matrix({ 
			param.a1, param.b1, param.c1, param.d1, param.e1, param.theta1,
			param.a2, param.b2, param.c2, param.d2, param.e2, param.theta2,
			param.a3, param.b3, param.c3, param.d3, param.e3, param.theta3,
		}));

		// 设置重力 //
		const double gravity[6]{ 0.0,0.0,-9.8,0.0,0.0,0.0 };
		model->environment().setGravity(gravity);

		// add part //
		const double default_iv[10]{ 1,0,0,0,0,0,0,0,0,0 };
		auto &p11 = model->partPool().add<Part>("L11", param.iv_vec.size() == 4 ? param.iv_vec[0].data() : default_iv);
		auto &p12 = model->partPool().add<Part>("L12", param.iv_vec.size() == 4 ? param.iv_vec[1].data() : default_iv);
		auto &p13 = model->partPool().add<Part>("L13", param.iv_vec.size() == 4 ? param.iv_vec[2].data() : default_iv);
		auto &p21 = model->partPool().add<Part>("L21", param.iv_vec.size() == 4 ? param.iv_vec[0].data() : default_iv);
		auto &p22 = model->partPool().add<Part>("L22", param.iv_vec.size() == 4 ? param.iv_vec[1].data() : default_iv);
		auto &p23 = model->partPool().add<Part>("L23", param.iv_vec.size() == 4 ? param.iv_vec[2].data() : default_iv);
		auto &p31 = model->partPool().add<Part>("L31", param.iv_vec.size() == 4 ? param.iv_vec[0].data() : default_iv);
		auto &p32 = model->partPool().add<Part>("L32", param.iv_vec.size() == 4 ? param.iv_vec[1].data() : default_iv);
		auto &p33 = model->partPool().add<Part>("L33", param.iv_vec.size() == 4 ? param.iv_vec[2].data() : default_iv);
		auto &pup = model->partPool().add<Part>("UP", param.iv_vec.size() == 4 ? param.iv_vec[3].data() : default_iv);
		auto &pee = model->partPool().add<Part>("EE", param.iv_vec.size() == 4 ? param.iv_vec[3].data() : default_iv);

		// add geometry //
		//model->ground().geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\base.xmt_txt");
		//pup.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\up.xmt_txt");
		//pee.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\ee.xmt_txt");
		//double geo_p13_prt_pm[16]{ 1,0,0,0,0,1,0,-param.c,0,0,1,0,0,0,0,1 };
		//double geo_rot_pm[16], geo_local_pm[16];
		//p11.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p1.xmt_txt");
		//p12.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p2.xmt_txt");
		//p13.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p2.xmt_txt", geo_p13_prt_pm);
		//s_eye(4, geo_rot_pm);
		//s_rmz(PI * 2 / 3, geo_rot_pm, 4);
		//s_pm_dot_pm(geo_rot_pm, geo_p13_prt_pm, geo_local_pm);
		//p21.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p1.xmt_txt", geo_rot_pm);
		//p22.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p2.xmt_txt", geo_rot_pm);
		//p23.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p2.xmt_txt", geo_local_pm);
		//s_eye(4, geo_rot_pm);
		//s_rmz(-PI * 2 / 3, geo_rot_pm, 4);
		//s_pm_dot_pm(geo_rot_pm, geo_p13_prt_pm, geo_local_pm);
		//p31.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p1.xmt_txt", geo_rot_pm);
		//p32.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p2.xmt_txt", geo_rot_pm);
		//p33.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p2.xmt_txt", geo_local_pm);

		// add joint //
		double r11_pos_1[3]{ param.a1           ,          0.0, 0.0 };
		double s12_pos_1[3]{ param.a1 + param.b1,  param.c1 / 2, 0.0 };
		double s13_pos_1[3]{ param.a1 + param.b1, -param.c1 / 2, 0.0 };
		double s14_pos_1[3]{ param.e1           ,  param.c1 / 2, -std::sqrt(param.d1*param.d1 - (param.a1 + param.b1 - param.e1)*(param.a1 + param.b1 - param.e1)) };
		double s15_pos_1[3]{ param.e1           , -param.c1 / 2, -std::sqrt(param.d1*param.d1 - (param.a1 + param.b1 - param.e1)*(param.a1 + param.b1 - param.e1)) };
		
		double r11_axis_1[6]{ 0.0, 1.0, 0.0 };
		double u12_first_axis_1[3]{ 0.0, 1.0, 0.0 };
		double u12_second_axis_1[3]{ s14_pos_1[2] - s12_pos_1[2], 0.0, -s14_pos_1[0] + s12_pos_1[0] };

		double r21_pos_1[3]{ param.a2           ,           0.0, 0.0 };
		double s22_pos_1[3]{ param.a2 + param.b2,  param.c2 / 2, 0.0 };
		double s23_pos_1[3]{ param.a2 + param.b2, -param.c2 / 2, 0.0 };
		double s24_pos_1[3]{ param.e2           ,  param.c2 / 2, -std::sqrt(param.d2*param.d2 - (param.a2 + param.b2 - param.e2)*(param.a2 + param.b2 - param.e2)) };
		double s25_pos_1[3]{ param.e2           , -param.c2 / 2, -std::sqrt(param.d2*param.d2 - (param.a2 + param.b2 - param.e2)*(param.a2 + param.b2 - param.e2)) };

		double r21_axis_1[6]{ 0.0, 1.0, 0.0 };
		double u22_first_axis_1[3]{ 0.0, 1.0, 0.0 };
		double u22_second_axis_1[3]{ s24_pos_1[2] - s22_pos_1[2], 0.0, -s24_pos_1[0] + s22_pos_1[0] };

		double r31_pos_1[3]{ param.a3           ,          0.0, 0.0 };
		double s32_pos_1[3]{ param.a3 + param.b3,  param.c3 / 2, 0.0 };
		double s33_pos_1[3]{ param.a3 + param.b3, -param.c3 / 2, 0.0 };
		double s34_pos_1[3]{ param.e3           ,  param.c3 / 2, -std::sqrt(param.d3*param.d3 - (param.a3 + param.b3 - param.e3)*(param.a3 + param.b3 - param.e3)) };
		double s35_pos_1[3]{ param.e3           , -param.c3 / 2, -std::sqrt(param.d3*param.d3 - (param.a3 + param.b3 - param.e3)*(param.a3 + param.b3 - param.e3)) };

		double r31_axis_1[6]{ 0.0, 1.0, 0.0 };
		double u32_first_axis_1[3]{ 0.0, 1.0, 0.0 };
		double u32_second_axis_1[3]{ s34_pos_1[2] - s32_pos_1[2], 0.0, -s34_pos_1[0] + s32_pos_1[0] };
		
		double r11_pos[3], s12_pos[3], s13_pos[3], s14_pos[3], s15_pos[3], r11_axis[3], u12_first_axis[3], u12_second_axis[3];
		double r21_pos[3], s22_pos[3], s23_pos[3], s24_pos[3], s25_pos[3], r21_axis[3], u22_first_axis[3], u22_second_axis[3];
		double r31_pos[3], s32_pos[3], s33_pos[3], s34_pos[3], s35_pos[3], r31_axis[3], u32_first_axis[3], u32_second_axis[3];

		double rm1[9], rm2[9], rm3[9];
		s_rmz(param.theta1, rm1);
		s_rmz(param.theta2, rm2);
		s_rmz(param.theta3, rm3);

		s_mm(3, 1, 3, rm1, r11_pos_1,         r11_pos);
		s_mm(3, 1, 3, rm1, s12_pos_1,         s12_pos);
		s_mm(3, 1, 3, rm1, s13_pos_1,         s13_pos);
		s_mm(3, 1, 3, rm1, s14_pos_1,         s14_pos);
		s_mm(3, 1, 3, rm1, s15_pos_1,         s15_pos);
		s_mm(3, 1, 3, rm1, r11_axis_1,        r11_axis);
		s_mm(3, 1, 3, rm1, u12_first_axis_1,  u12_first_axis);
		s_mm(3, 1, 3, rm1, u12_second_axis_1, u12_second_axis);

		s_mm(3, 1, 3, rm2, r21_pos_1,         r21_pos);
		s_mm(3, 1, 3, rm2, s22_pos_1,         s22_pos);
		s_mm(3, 1, 3, rm2, s23_pos_1,         s23_pos);
		s_mm(3, 1, 3, rm2, s24_pos_1,         s24_pos);
		s_mm(3, 1, 3, rm2, s25_pos_1,         s25_pos);
		s_mm(3, 1, 3, rm2, r21_axis_1,        r21_axis);
		s_mm(3, 1, 3, rm2, u22_first_axis_1,  u22_first_axis);
		s_mm(3, 1, 3, rm2, u22_second_axis_1, u22_second_axis);

		s_mm(3, 1, 3, rm3, r31_pos_1,         r31_pos);
		s_mm(3, 1, 3, rm3, s32_pos_1,         s32_pos);
		s_mm(3, 1, 3, rm3, s33_pos_1,         s33_pos);
		s_mm(3, 1, 3, rm3, s34_pos_1,         s34_pos);
		s_mm(3, 1, 3, rm3, s35_pos_1,         s35_pos);
		s_mm(3, 1, 3, rm3, r31_axis_1,        r31_axis);
		s_mm(3, 1, 3, rm3, u32_first_axis_1,  u32_first_axis);
		s_mm(3, 1, 3, rm3, u32_second_axis_1, u32_second_axis);

		auto &r11 = model->addRevoluteJoint(p11, model->ground(), r11_pos, r11_axis);
		auto &u12 = model->addUniversalJoint(p12, p11, s12_pos, u12_second_axis, u12_first_axis);
		auto &u13 = model->addUniversalJoint(p13, p11, s13_pos, u12_second_axis, u12_first_axis);
		auto &s14 = model->addSphericalJoint(pup, p12, s14_pos);
		auto &s15 = model->addSphericalJoint(pup, p13, s15_pos);

		auto &r21 = model->addRevoluteJoint(p21, model->ground(), r21_pos, r21_axis);
		auto &u22 = model->addUniversalJoint(p22, p21, s22_pos, u22_second_axis, u22_first_axis);
		auto &u23 = model->addUniversalJoint(p23, p21, s23_pos, u22_second_axis, u22_first_axis);
		auto &s24 = model->addSphericalJoint(pup, p22, s24_pos);
		auto &s25 = model->addSphericalJoint(pup, p23, s25_pos);

		auto &r31 = model->addRevoluteJoint(p31, model->ground(), r31_pos, r31_axis);
		auto &u32 = model->addUniversalJoint(p32, p31, s32_pos, u32_second_axis, u32_first_axis);
		auto &u33 = model->addUniversalJoint(p33, p31, s33_pos, u32_second_axis, u32_first_axis);
		auto &s34 = model->addSphericalJoint(pup, p32, s34_pos);
		auto &s35 = model->addSphericalJoint(pup, p33, s35_pos);

		// 求正解计算末端位置 //
		double input_0[4]{ 0,0,0,0 };
		double output_0[4]{ 0,0,0,0 };
		deltaForward(dynamic_cast<aris::dynamic::MatrixVariable&>(model->variablePool()[0]).data().data(), input_0, 0, output_0);

		// 得到末端
		double re_pos[3]{ output_0[0], output_0[1], output_0[2]};
		double re_axis[3]{ 0,0,1 };
		auto &re = model->addRevoluteJoint(pee, pup, re_pos, re_axis);
		// add actuation //
		auto &m1 = model->addMotion(r11);
		auto &m2 = model->addMotion(r21);
		auto &m3 = model->addMotion(r31);
		auto &m4 = model->addMotion(re);

		const double default_mot_frc[3]{ 0.0, 0.0, 0.0 };
		m1.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[0].data() : default_mot_frc);
		m2.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[1].data() : default_mot_frc);
		m3.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[2].data() : default_mot_frc);
		m4.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[3].data() : default_mot_frc);

		// add ee general motion //
		double ee_i_pm[16]{ 
			1,0,0,output_0[0],
			0,1,0,output_0[1],
			0,0,1,output_0[2],
			0,0,0,1 };
		double ee_j_pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };

		auto &makI = pee.addMarker("tool0", ee_i_pm);
		auto &makJ = model->ground().addMarker("wobj0", ee_j_pm);
		model->variablePool().add<aris::dynamic::MatrixVariable>("tool0_axis_home", aris::core::Matrix(1, 6, 0.0));
		auto &ee = model->generalMotionPool().add<aris::dynamic::XyztMotion>("tool", &makI, &makJ, false);

		// change robot pose wrt ground //
		double robot_pm[16];
		s_pe2pm(param.base2ref_pe, robot_pm, param.base2ref_pe_type.empty() ? "321" : param.base2ref_pe_type.c_str());

		p11.setPm(s_pm_dot_pm(robot_pm, *p11.pm()));
		p12.setPm(s_pm_dot_pm(robot_pm, *p12.pm()));
		p13.setPm(s_pm_dot_pm(robot_pm, *p13.pm()));
		p21.setPm(s_pm_dot_pm(robot_pm, *p21.pm()));
		p22.setPm(s_pm_dot_pm(robot_pm, *p22.pm()));
		p23.setPm(s_pm_dot_pm(robot_pm, *p23.pm()));
		p31.setPm(s_pm_dot_pm(robot_pm, *p31.pm()));
		p32.setPm(s_pm_dot_pm(robot_pm, *p32.pm()));
		p33.setPm(s_pm_dot_pm(robot_pm, *p33.pm()));
		pup.setPm(s_pm_dot_pm(robot_pm, *pup.pm()));
		pee.setPm(s_pm_dot_pm(robot_pm, *pee.pm()));
		r11.makJ()->setPrtPm(s_pm_dot_pm(robot_pm, *r11.makJ()->prtPm()));
		r21.makJ()->setPrtPm(s_pm_dot_pm(robot_pm, *r21.makJ()->prtPm()));
		r31.makJ()->setPrtPm(s_pm_dot_pm(robot_pm, *r31.makJ()->prtPm()));
		ee.makJ()->setPrtPm(s_pm_dot_pm(robot_pm, *ee.makJ()->prtPm()));

		// add tools and wobj //
		for (int i = 1; i < 17; ++i) {
			pee.addMarker("tool" + std::to_string(i), ee_i_pm);
		}
		for (int i = 1; i < 33; ++i) model->ground().markerPool().add<aris::dynamic::Marker>("wobj" + std::to_string(i), ee_j_pm);

		// add solver
		auto &inverse_kinematic = model->solverPool().add<aris::dynamic::DeltaInverseKinematicSolver>();
		auto &forward_kinematic = model->solverPool().add<aris::dynamic::DeltaForwardKinematicSolver>();
		auto &inverse_dynamic = model->solverPool().add<aris::dynamic::InverseDynamicSolver>();
		auto &forward_dynamic = model->solverPool().add<aris::dynamic::ForwardDynamicSolver>();

		// make topology correct // 
		for (auto &m : model->motionPool())m.activate(true);
		for (auto &gm : model->generalMotionPool())gm.activate(false);
		for (auto &f : model->forcePool())f.activate(false);

		model->init();
		return model;
	}

	auto planarDeltaInverse(const double *param, const double *ee_xya, int which_root, double *input)->int {
		// Planar Delta
		//---------------------------------------------------------------------------------------------
		// 包含4个自由度。共4个杆件  link1~4
		// 尺寸示意：
		//                   上平台 
		//                     
		//                     o------o      
		//                    /         \    
		//                   /            \   
		//                   o              o   
		//                     \           /  
		//                       \        /  
		//                         \     /     
		//                           \  /        
		//                             o
		//                       ee(长度e)
		//---------------------------------------------------------------------------------------------
		// 零位示意：
		//                         y 
		//                         ^
		//                         | a |<- b ->|
		//             o-------o---*---o-------o   ->  x             
		//            / \                     / 
		//                \                 /   
		//                  \             /        
		//                 c  \         /       
		//                      \     /        
		//                        \ /     
		//                         o         
		//                        /|
		//                            
		//                       ee(长度e)
		//---------------------------------------------------------------------------------------------
		
		// 输入输出关系：
		// 
		// [ x ] = [ a ] + [ c1 -s1 ] * [ b ] + [ c2 -s2 ] * [ c ] 
		// [ y ]   [ 0 ]   [ s1  c1 ]   [ 0 ]   [ s2  c2 ]   [ 0 ]
		//
		// 其中 theta1 为电机的转角，theta2 为转动副2的转角减去电机的转角
		//  
		// 进一步的：
		// [ x ] = [ a + b * c1 + c * c2 ] 
		// [ y ]   [     b * s1 + c * s2 ]
		//
		// 消除未知数 theta2：
		//
		// x - a - b * c1 = c * c2  
		// y     - b * s1 = c * s2 
		//
		// -> : c^2 = (x - a - b * c1)^2 + (y - b * s1)^2
		//
		// -> : c^2 - b^2 = (x - a)^2 - 2 (x - a) b c1 + y^2 - 2 y b s1  
		// -> : c^2 - b^2 - (x - a)^2 - y^2 = - 2 y b s1 - 2 (x - a) b c1
		
		for (int i = 0; i < 2; ++i) {
			//尺寸
			auto a = param[0 + i * 3];
			auto b = param[1 + i * 3];
			auto c = param[2 + i * 3];
			auto d = param[6];

			auto x = ee_xya[0];
			auto y = ee_xya[1] + d;

			auto k1 = -2 * y * b;
			auto k2 = -2 * (x - a) * b;
			auto right_side = c * c - b * b - (x - a)*(x - a) - y * y;

			double result[2];
			auto ret = s_sov_theta(k1, k2, right_side, result);

			input[i] = result[(0x01 << i) & which_root];
		}

		input[2] = ee_xya[2];
		return 0;
	}
	auto planarDeltaForward(const double *param, const double *input, int which_root, double *ee_xya)->int {
		// 
		//            ----* p2  
		//    p1 *----    |
		//         \     /
		//           \  /
		//             *  ee
		//
		// p1:[x1 y1]
		// p2:[x2 y2]
		// ee:[x  y ]
		//
		// 根据余弦公式，求得   p2 - p1 - ee 之间的夹角theta，再求出ee 与 p1的夹角
		//
		//
		auto d = param[6];

		double x[2], y[2];
		for (int i = 0; i < 2; ++i) {
			//尺寸
			auto a = param[0 + i * 3];
			auto b = param[1 + i * 3];
			auto c = param[2 + i * 3];
			

			auto c1 = std::cos(input[i]);
			auto s1 = std::sin(input[i]);
			
			x[i] = a + b * c1;
			y[i] = b * s1;
		}
		
		auto c1 = param[2];
		auto c2 = param[5];
		
		auto d_p1_p2 = std::sqrt((x[1] - x[0]) * (x[1] - x[0]) + (y[1] - y[0])* (y[1] - y[0]));

		auto theta = std::acos((c1 * c1 + d_p1_p2 * d_p1_p2 - c2 * c2) / (2.0* c1 * d_p1_p2));
		if (which_root != 0)theta = -theta;


		double p1_p2[2]{ (x[1] - x[0])/ d_p1_p2, (y[1] - y[0])/ d_p1_p2 };
		double c1_axis[2]{p1_p2[0] * std::cos(theta) - p1_p2[1] * std::sin(theta), p1_p2[1] * std::cos(theta) + p1_p2[0] * std::sin(theta) };

		ee_xya[0] = x[0] + c1_axis[0] * c1;
		ee_xya[1] = y[0] + c1_axis[1] * c1 - d;
		ee_xya[2] = input[2];

		return 0;
	}

	class PlanarDeltaInverseKinematicSolver :public aris::dynamic::InverseKinematicSolver {
	public:
		auto virtual kinPos()->int override {
			double input[4], output[4];

			auto dh = dynamic_cast<aris::dynamic::MatrixVariable*>(model()->findVariable("dh"))->data().data();

			model()->getOutputPos(output);
			if (auto ret = planarDeltaInverse(dh, output, 0, input))
				return ret;

			//// ee //
			//double pe[6]{ output[0],output[1],output[2],output[3], 0.0, 0.0 }, pp[3];
			//model()->generalMotionPool()[0].makI()->setPe(*model()->generalMotionPool()[0].makJ(), pe, "321");

			//// up //
			//s_fill(1, 3, 0.0, pe);
			//pe[3] = -pe[3];
			//model()->jointPool()[15].makJ()->setPe(*model()->jointPool()[15].makI(), pe, "321");

			//// link1 //
			//s_fill(1, 6, 0.0, pe);
			//pe[5] = input[0];
			//model()->jointPool()[0].makI()->setPe(*model()->jointPool()[0].makJ(), pe, "123");

			//s_fill(1, 6, 0.0, pe);
			//pe[5] = input[1];
			//model()->jointPool()[5].makI()->setPe(*model()->jointPool()[5].makJ(), pe, "123");

			//s_fill(1, 6, 0.0, pe);
			//pe[5] = input[2];
			//model()->jointPool()[10].makI()->setPe(*model()->jointPool()[10].makJ(), pe, "123");

			//// link2&3 //
			//s_fill(1, 6, 0.0, pe);
			//pe[5] = PI / 2;
			//model()->jointPool()[3].makI()->getPp(*model()->jointPool()[1].makJ(), pp);
			//s_sov_ab(pp, pe + 3, "312");
			//model()->jointPool()[1].makI()->setPe(*model()->jointPool()[1].makJ(), pe, "312");

			//model()->jointPool()[4].makI()->getPp(*model()->jointPool()[2].makJ(), pp);
			//s_sov_ab(pp, pe + 3, "312");
			//model()->jointPool()[2].makI()->setPe(*model()->jointPool()[2].makJ(), pe, "312");

			//model()->jointPool()[8].makI()->getPp(*model()->jointPool()[6].makJ(), pp);
			//s_sov_ab(pp, pe + 3, "312");
			//model()->jointPool()[6].makI()->setPe(*model()->jointPool()[6].makJ(), pe, "312");

			//model()->jointPool()[9].makI()->getPp(*model()->jointPool()[7].makJ(), pp);
			//s_sov_ab(pp, pe + 3, "312");
			//model()->jointPool()[7].makI()->setPe(*model()->jointPool()[7].makJ(), pe, "312");

			//model()->jointPool()[13].makI()->getPp(*model()->jointPool()[11].makJ(), pp);
			//s_sov_ab(pp, pe + 3, "312");
			//model()->jointPool()[11].makI()->setPe(*model()->jointPool()[11].makJ(), pe, "312");

			//model()->jointPool()[14].makI()->getPp(*model()->jointPool()[12].makJ(), pp);
			//s_sov_ab(pp, pe + 3, "312");
			//model()->jointPool()[12].makI()->setPe(*model()->jointPool()[12].makJ(), pe, "312");

			model()->motionPool()[0].setMp(input[0]);
			model()->motionPool()[1].setMp(input[1]);
			model()->motionPool()[2].setMp(input[2]);
			return 0;
		}

		PlanarDeltaInverseKinematicSolver() = default;
	};
	class PlanarDeltaForwardKinematicSolver :public aris::dynamic::ForwardKinematicSolver {
	public:
		auto virtual kinPos()->int override {
			double input[4], output[4];

			auto dh = dynamic_cast<aris::dynamic::MatrixVariable*>(model()->findVariable("dh"))->data().data();

			model()->getInputPos(input);
			if (auto ret = planarDeltaForward(dh, input, 0, output))
				return ret;

			// ee //
			double pe[6]{ output[0], output[1], 0.0, output[2], 0.0, 0.0 };
			model()->generalMotionPool()[0].makI()->setPe(*model()->generalMotionPool()[0].makJ(), pe, "321");

			//// up //
			//s_fill(1, 3, 0.0, pe);
			//pe[3] = -pe[3];
			//model()->jointPool()[15].makJ()->setPe(*model()->jointPool()[15].makI(), pe, "321");

			//// link1 //
			//s_fill(1, 6, 0.0, pe);
			//pe[5] = input[0];
			//model()->jointPool()[0].makI()->setPe(*model()->jointPool()[0].makJ(), pe, "123");

			//s_fill(1, 6, 0.0, pe);
			//pe[5] = input[1];
			//model()->jointPool()[5].makI()->setPe(*model()->jointPool()[5].makJ(), pe, "123");

			//s_fill(1, 6, 0.0, pe);
			//pe[5] = input[2];
			//model()->jointPool()[10].makI()->setPe(*model()->jointPool()[10].makJ(), pe, "123");

			//// link2&3 //
			//s_fill(1, 6, 0.0, pe);
			//pe[5] = PI / 2;
			//model()->jointPool()[3].makI()->getPp(*model()->jointPool()[1].makJ(), pp);
			//s_sov_ab(pp, pe + 3, "312");
			//model()->jointPool()[1].makI()->setPe(*model()->jointPool()[1].makJ(), pe, "312");

			//model()->jointPool()[4].makI()->getPp(*model()->jointPool()[2].makJ(), pp);
			//s_sov_ab(pp, pe + 3, "312");
			//model()->jointPool()[2].makI()->setPe(*model()->jointPool()[2].makJ(), pe, "312");

			//model()->jointPool()[8].makI()->getPp(*model()->jointPool()[6].makJ(), pp);
			//s_sov_ab(pp, pe + 3, "312");
			//model()->jointPool()[6].makI()->setPe(*model()->jointPool()[6].makJ(), pe, "312");

			//model()->jointPool()[9].makI()->getPp(*model()->jointPool()[7].makJ(), pp);
			//s_sov_ab(pp, pe + 3, "312");
			//model()->jointPool()[7].makI()->setPe(*model()->jointPool()[7].makJ(), pe, "312");

			//model()->jointPool()[13].makI()->getPp(*model()->jointPool()[11].makJ(), pp);
			//s_sov_ab(pp, pe + 3, "312");
			//model()->jointPool()[11].makI()->setPe(*model()->jointPool()[11].makJ(), pe, "312");

			//model()->jointPool()[14].makI()->getPp(*model()->jointPool()[12].makJ(), pp);
			//s_sov_ab(pp, pe + 3, "312");
			//model()->jointPool()[12].makI()->setPe(*model()->jointPool()[12].makJ(), pe, "312");

			for (auto &m : model()->generalMotionPool()) m.updP();
			return 0;
		}

		PlanarDeltaForwardKinematicSolver() = default;
	};

	/*auto ARIS_API createModelPlanarDelta(const PlanarDeltaParam &param)->std::unique_ptr<aris::dynamic::Model> {
		DeltaFullParam full_param;
		full_param.a1 = full_param.a2 = full_param.a3 = param.a;
		full_param.b1 = full_param.b2 = full_param.b3 = param.b;
		full_param.c1 = full_param.c2 = full_param.c3 = param.c;
		full_param.d1 = full_param.d2 = full_param.d3 = param.d;
		full_param.e1 = full_param.e2 = full_param.e3 = param.e;

		full_param.theta1 = 0.0;
		full_param.theta2 = aris::PI * 2 / 3;
		full_param.theta3 = -aris::PI * 2 / 3;

		s_vc(6, param.tool0_pe, full_param.tool0_pe);
		full_param.tool0_pe_type = param.tool0_pe_type;

		s_vc(6, param.base2ref_pe, full_param.base2ref_pe);
		full_param.base2ref_pe_type = param.base2ref_pe_type;

		full_param.iv_vec = param.iv_vec;

		full_param.mot_frc_vec = param.mot_frc_vec;

		return createModelDelta(full_param);
	}*/
	auto ARIS_API createModelPlanarDelta(const PlanarDeltaFullParam &param)->std::unique_ptr<aris::dynamic::Model> {
		std::unique_ptr<aris::dynamic::Model> model(new aris::dynamic::Model);

		model->variablePool().add<aris::dynamic::MatrixVariable>("dh", aris::core::Matrix({
			param.a1, param.b1, param.c1,
			param.a2, param.b2, param.c2, 
			param.d
			}));

		// 设置重力 //
		const double gravity[6]{ 0.0,0.0,-9.8,0.0,0.0,0.0 };
		model->environment().setGravity(gravity);

		// add part //
		const double default_iv[10]{ 1,0,0,0,0,0,0,0,0,0 };
		auto &p11 = model->partPool().add<Part>("L11", param.iv_vec.size() == 4 ? param.iv_vec[0].data() : default_iv);
		auto &p12 = model->partPool().add<Part>("L12", param.iv_vec.size() == 4 ? param.iv_vec[1].data() : default_iv);
		auto &p21 = model->partPool().add<Part>("L31", param.iv_vec.size() == 4 ? param.iv_vec[0].data() : default_iv);
		auto &p22 = model->partPool().add<Part>("L32", param.iv_vec.size() == 4 ? param.iv_vec[1].data() : default_iv);
		auto &pup = model->partPool().add<Part>("UP", param.iv_vec.size() == 4 ? param.iv_vec[3].data() : default_iv);
		auto &pee = model->partPool().add<Part>("EE", param.iv_vec.size() == 4 ? param.iv_vec[3].data() : default_iv);

		// add geometry //

		// add joint //
		double axis[3]{ 0,0,1 };
		double r11_pos[3]{ param.a1           ,          0.0, 0.0 };
		double r12_pos[3]{ param.a1 + param.b1,  param.c1 / 2, 0.0 };
		double r13_pos[3]{ param.a1 + param.b1, -param.c1 / 2, 0.0 };
		
		double r21_pos[3]{ param.a2           ,           0.0, 0.0 };
		double r22_pos[3]{ param.a2 + param.b2,  param.c2 / 2, 0.0 };
		double r23_pos[3]{ param.a2 + param.b2, -param.c2 / 2, 0.0 };

		auto &r11 = model->addRevoluteJoint(p11, model->ground(), r11_pos, axis);
		auto &r12 = model->addRevoluteJoint(p12, p11, r12_pos, axis);
		auto &r13 = model->addRevoluteJoint(pup, p11, r13_pos, axis);

		auto &r21 = model->addRevoluteJoint(p21, model->ground(), r21_pos, axis);
		auto &r22 = model->addRevoluteJoint(p22, p21, r22_pos, axis);
		auto &r23 = model->addRevoluteJoint(pup, p21, r23_pos, axis);


		// 求正解计算末端位置 //
		double input_0[4]{ 0,0,0,0 };
		double output_0[4]{ 0,0,0,0 };
		planarDeltaForward(dynamic_cast<aris::dynamic::MatrixVariable&>(model->variablePool()[0]).data().data(), input_0, 0, output_0);

		// 得到末端
		double re_pos[3]{ output_0[0], output_0[1], output_0[2] };
		double re_axis[3]{ 0,0,1 };
		auto &re = model->addRevoluteJoint(pee, pup, re_pos, re_axis);
		// add actuation //
		auto &m1 = model->addMotion(r11);
		auto &m2 = model->addMotion(r21);
		auto &m3 = model->addMotion(re);

		const double default_mot_frc[3]{ 0.0, 0.0, 0.0 };
		m1.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[0].data() : default_mot_frc);
		m2.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[1].data() : default_mot_frc);
		m3.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[2].data() : default_mot_frc);

		// add ee general motion //
		double ee_i_pm[16]{
			1,0,0,output_0[0],
			0,1,0,output_0[1],
			0,0,1,output_0[2],
			0,0,0,1 };
		double ee_j_pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };

		auto &makI = pee.addMarker("tool0", ee_i_pm);
		auto &makJ = model->ground().addMarker("wobj0", ee_j_pm);
		model->variablePool().add<aris::dynamic::MatrixVariable>("tool0_axis_home", aris::core::Matrix(1, 6, 0.0));
		auto &ee = model->generalMotionPool().add<aris::dynamic::PlanarMotion>("tool", &makI, &makJ, false);

		// change robot pose wrt ground //

		// add tools and wobj //
		for (int i = 1; i < 17; ++i) {
			pee.addMarker("tool" + std::to_string(i), ee_i_pm);
		}
		for (int i = 1; i < 33; ++i) model->ground().markerPool().add<aris::dynamic::Marker>("wobj" + std::to_string(i), ee_j_pm);

		// add solver
		auto &inverse_kinematic = model->solverPool().add<aris::dynamic::PlanarDeltaInverseKinematicSolver>();
		auto &forward_kinematic = model->solverPool().add<aris::dynamic::PlanarDeltaForwardKinematicSolver>();
		auto &inverse_dynamic = model->solverPool().add<aris::dynamic::InverseDynamicSolver>();
		auto &forward_dynamic = model->solverPool().add<aris::dynamic::ForwardDynamicSolver>();

		// make topology correct // 
		for (auto &m : model->motionPool())m.activate(true);
		for (auto &gm : model->generalMotionPool())gm.activate(false);
		for (auto &f : model->forcePool())f.activate(false);

		model->init();
		return model;
	}

	ARIS_REGISTRATION{
		aris::core::class_<DeltaInverseKinematicSolver>("DeltaInverseKinematicSolver")
			.inherit<InverseKinematicSolver>()
			;

		aris::core::class_<DeltaForwardKinematicSolver>("DeltaForwardKinematicSolver")
			.inherit<ForwardKinematicSolver>()
			;

		aris::core::class_<PlanarDeltaInverseKinematicSolver>("PlanarDeltaInverseKinematicSolver")
			.inherit<InverseKinematicSolver>()
			;

		aris::core::class_<PlanarDeltaForwardKinematicSolver>("PlanarDeltaForwardKinematicSolver")
			.inherit<ForwardKinematicSolver>()
			;
	}
}
