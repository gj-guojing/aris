#include <iostream>
#include <aris/core/core.hpp>
#include <aris/plan/plan.hpp>
#include <aris/robot/rokae.hpp>

#include <random>
using namespace aris::plan;

auto test_singular_processor_1()->void {
	// 构造 TG //
	aris::plan::TrajectoryGenerator tg;

	const int PE_SIZE = 6;
	const int EE_NUM = 1;
	const int A_NUM = 0;

	//  INIT TG //
	tg.setEeTypes({ aris::dynamic::EEType::PE321 });
	double init_pe[EE_NUM * 6]{ 0.45, 0, 0.75,   aris::PI, 1.0,   aris::PI };
	double init_vel[EE_NUM * 2 + A_NUM]{ 1,1 };
	tg.insertLinePos(1, init_pe, init_vel, init_vel, init_vel, init_vel);

	//  MAKE PQS ... //
	double pes[PE_SIZE][6 * EE_NUM]{
		{ 0.45, 0.4, 0.75,   aris::PI, 1.0,   aris::PI},
		{ 0.45, 0.0, 0.75,   aris::PI, 1.0,   aris::PI},
		{ 0.45, 0.0, 0.65,   aris::PI, 1.0,   aris::PI},
		{ 0.45, 0.0, 0.65,   aris::PI / 4, aris::PI / 2,   aris::PI / 4},
		{ 0.45, 0.0, 0.75,   aris::PI / 4, aris::PI / 2,   aris::PI / 4},
		{ 0.45, 0.1, 0.75,   aris::PI / 4, aris::PI / 2,   aris::PI / 4},
	};

	//  MAKE VELS ACCS JERKS ZONES ... //
	double vels[PE_SIZE][2 * EE_NUM + A_NUM]{
		{ 10, 10 },
		{ 0.1, 0.1 },
		{ 0.1, 0.1 },
		{ 1000, 1000 },
		{ 1000, 1000 },
		{ 1000, 1000 },
	};
	double accs[PE_SIZE][2 * EE_NUM + A_NUM]{
		{ 10, 10 },
		{ 0.1, 0.1 },
		{ 0.1, 0.1 },
		{ 10000, 10000 },
		{ 10000, 10000 },
		{ 10000, 10000 },
	};
	double jerks[PE_SIZE][2 * EE_NUM + A_NUM]{
		{ 100, 100 },
		{ 0.1, 0.1 },
		{ 0.1, 0.1 },
		{ 10000000, 1000000 },
		{ 10000000, 1000000 },
		{ 10000000, 1000000 },
	};
	double zones[PE_SIZE][2 * EE_NUM + A_NUM]{
		{ 0.2, 0.2 },
		{ 0.0, 0.0 },
		{ 0.1, 0.1 },
		{ 0.0, 0.0 },
		{ 0.0, 0.0 },
		{ 0.0, 0.0 },
	};

	for (int i = 0; i < PE_SIZE; ++i) {
		tg.insertLinePos(i + 10, pes[i % PE_SIZE], vels[i % PE_SIZE], accs[i % PE_SIZE], jerks[i % PE_SIZE], zones[i % PE_SIZE]);
	}

	double out_pe[7 * EE_NUM + A_NUM];

	// 构造模型 //
	aris::dynamic::PumaParam puma_param;
	puma_param.d1 = 0.3;
	puma_param.a1 = 0.1;
	puma_param.a2 = 0.4;
	puma_param.a3 = 0.05;
	puma_param.d3 = 0.0;
	puma_param.d4 = 0.35;
	puma_param.install_method = 0;
	auto puma = aris::dynamic::createModelPuma(puma_param);

	puma->setOutputPos(init_pe);
	puma->inverseKinematics();

	dynamic_cast<aris::dynamic::GeneralMotion&>(puma->generalMotionPool()[0]).setPoseType(aris::dynamic::GeneralMotion::PoseType::EULER321);
	double input_init[6]{ 0,0,0,0,0,0 };
	puma->setInputPos(input_init);
	puma->forwardKinematics();
	double pm[16];
	puma->getOutputPos(pm);
	aris::dynamic::dsp(1, 16, pm);


	//  这里处理 //
	aris::plan::SingularProcessor sp;

	// 最大速度、加速度 //
	std::vector<double> max_vels{ 3.14, 3.14, 3.14, 3.14, 3.14, 3.14 };
	std::vector<double> max_accs{ 31.4, 31.4, 31.4, 31.4, 31.4, 31.4 };
	std::vector<double> max_jerks{ 314, 314, 314, 314, 314, 314 };

	// 设置模型等参数 //
	sp.setModel(*puma);
	sp.setMaxVels(max_vels.data());
	sp.setMaxAccs(max_accs.data());
	sp.setMaxJerks(max_jerks.data());
	sp.setTrajectoryGenerator(tg);
	//sp.setInverseKinematicMethod([](aris::dynamic::ModelBase &model, const double *output) {
	//	model
	//
	//
	//	});

	sp.init();

	// 设置速度百分比 //
	//sp.setDs(0.5);

	// 打印数据 //
	std::vector<double> vec, v_vec, a_vec;
	int m = 0;
	double out_vel[16]{}, out_acc[16]{};
	while (sp.setModelPosAndMoveDt()) {
		m++;

		if (m > 3000 && m < 6000)
			sp.setTargetDs(0.0);
		else
			sp.setTargetDs(1.0);

		if (m == 6000) {
			std::cout << "debug" << std::endl;
		}



		vec.resize(m * (6 * EE_NUM + A_NUM), 0.0);
		v_vec.resize(m * (6 * EE_NUM + A_NUM), 0.0);
		a_vec.resize(m * (6 * EE_NUM + A_NUM), 0.0);
		
		puma->getInputPos(vec.data() + (6 * EE_NUM + A_NUM) * (m - 1));
	}

	aris::dynamic::dlmwrite(m, (6 * EE_NUM + A_NUM), vec.data(), "C:\\Users\\py033\\Desktop\\test_data\\pes.txt");
	//aris::dynamic::dlmwrite(m, (7 * EE_NUM + A_NUM), v_vec.data(), "C:\\Users\\py033\\Desktop\\test_data\\vpes.txt");
	//aris::dynamic::dlmwrite(m, (7 * EE_NUM + A_NUM), a_vec.data(), "C:\\Users\\py033\\Desktop\\test_data\\apes.txt");

}

auto test_smooth2() -> void {
	auto func = [](double s, double* p)->int {
		p[0] = s * s;
		return 0;
		};

	const double dt = 0.001;
	const int dim = 1;

	double min_p[dim]{ -100 };
	double max_p[dim]{ 100 };
	double min_dp[dim]{ -0.25 };
	double max_dp[dim]{ 1.0 };
	double min_d2p[dim]{ -2.0 };
	double max_d2p[dim]{ 2.0 };
	double min_d3p[dim]{ -10.0 };
	double max_d3p[dim]{ 10.0 };

	double p0[dim];
	double p1[dim];
	double p2[dim];
	double p3[dim];

	double s0 = 0.5 * dt * 0, s1 = 0.5 * dt * 1, s2 = 0.5 * dt * 2, s3 = 0.5 * dt * 3;

	func(s0, p0);
	func(s1, p1);
	func(s2, p2);
	func(s3, p3);

	std::vector<double> poss{p0[0], p1[0], p2[0], p3[0]};
	int n = 200;
	for (int i = 0; i < n; ++i) {
		SmoothParam p{
			dt,
			dim,
			min_p, max_p, min_dp, max_dp, min_d2p, max_d2p, min_d3p, max_d3p,
			0.005, 1.0, -1000, 1000, -100000, 1000000,
			(s1-s0)/dt, (s2-s1)/dt, (s3-s2)/dt,
			p0, p1, p2, p3,
			1.0
		};
		SmoothRet ret;
		s_smooth_curve2(p, ret);
		
		std::swap(p0[0], p1[0]);
		std::swap(p1[0], p2[0]);
		std::swap(p2[0], p3[0]);

		std::swap(s0, s1);
		std::swap(s1, s2);
		std::swap(s2, s3);
		
		if (i == 100)
			std::cout << "debug" << std::endl;	

		s3 = s2 + ret.next_ds * dt;
		func(s3, p3);

		poss.push_back(p3[0]);
	}

	aris::dynamic::dlmwrite(n+4, 1, poss.data(), "C:\\Users\\py033\\Desktop\\test_data\\poss.txt");
}

// 
auto test_smooth_cond_3_1() -> void {
	auto func = [](double s, double* p)->int {
		double a = 1.0;
		double b = 2.0;
		// 
		// p   = a * sin(b*s)
		// dp  = a*b * cos(b*s)
		// d2p = -a*b*b* sin(b*s)
		// d3p = -a*b*b*b*cos(b*s)
		p[0] = a*std::sin(b*s - aris::PI/2);


		return 0;
		};

	const double dt = 0.001;
	const int dim = 1;

	double min_p[dim]{ -100 };
	double max_p[dim]{ 100 };
	double min_dp[dim]{ -3.0 };
	double max_dp[dim]{ 3.0 };
	double min_d2p[dim]{ -3.0 };
	double max_d2p[dim]{ 3.0 };
	double min_d3p[dim]{ -10.0 };
	double max_d3p[dim]{ 10.0 };

	double p0[dim];
	double p1[dim];
	double p2[dim];
	double p3[dim];

	double s0 = dt * 0, s1 = dt * 1, s2 = dt * 2, s3 = dt * 3;

	func(s0, p0);
	func(s1, p1);
	func(s2, p2);
	func(s3, p3);

	std::vector<double> poss{ p0[0], p1[0], p2[0], p3[0] };
	int n = 6000;
	for (int i = 0; i < n; ++i) {
		SmoothParam p{
			dt,
			dim,
			min_p, max_p, min_dp, max_dp, min_d2p, max_d2p, min_d3p, max_d3p,
			0.005, 1.0, -1000, 1000, -100000, 1000000,
			(s1 - s0) / dt, (s2 - s1) / dt, (s3 - s2) / dt,
			p0, p1, p2, p3,
			1.0
		};
		SmoothRet ret;
		s_smooth_curve3(p, ret);

		if (ret.state != 2) {
			std::cout << "count:" << i << "  state:" << ret.state <<"  ds:" <<ret.next_ds << std::endl;
		
		}

		std::swap(p0[0], p1[0]);
		std::swap(p1[0], p2[0]);
		std::swap(p2[0], p3[0]);

		std::swap(s0, s1);
		std::swap(s1, s2);
		std::swap(s2, s3);


			

		s3 = s2 + ret.next_ds * dt;	
		func(s3, p3);

		if (i < 1600 && i >= 1270) {
			//std::cout << "debug" << std::endl;
			//std::cout << s2 << "  " << ret.next_ds << std::endl;
			
			auto dp3 = (p3[0] - p2[0]) / dt;
			auto dp2 = (p2[0] - p1[0]) / dt;
			auto dp1 = (p1[0] - p0[0]) / dt;

			auto d2p2 = (dp2 - dp1) / dt;
			auto d2p3 = (dp3 - dp2) / dt;

			auto d3p3 = (d2p3 - d2p2) / dt;

			auto ds1 = s1 - s0;
			auto ds2 = s2 - s1;
			auto ds3 = s3 - s2;

			auto d2s2 = (ds2 - ds1) / dt;
			auto d2s3 = (ds3 - ds2) / dt;

			auto d3s3 = (d2s3 - d2s2) / dt;


			auto dp_t15 = dp2;
			auto d2p_t15 = (d2p2 + d2p3) / 2;
			auto d3p_t15 = d3p3;

			double ds_t15 = ds2;
			double d2s_t15 = (d2s2 + d2s3) / 2;
			double d3s_t15 = d3s3;
			double ds_t25 = ds3;

			auto dp_ds_t15 = dp_t15 / ds_t15;
			auto d2p_ds2_t15 = (d2p_t15 - dp_ds_t15 * d2s_t15) / ds_t15 / ds_t15;
			auto d3p_ds3_t15 = (d3p_t15 - 3 * d2p_ds2_t15 * ds_t15 * d2s_t15 - dp_ds_t15 * d3s_t15) / ds_t15 / ds_t15 / ds_t15;

			auto s25_s15 = (ds2 + ds3) / 2 * dt;
			auto d3p_ds3_t25 = d3p_ds3_t15;
			auto d2p_ds2_t25 = d2p_ds2_t15 + d3p_ds3_t15 * s25_s15;
			auto dp_ds_t25 = dp_ds_t15 + d2p_ds2_t15 * s25_s15 + 0.5 * d3p_ds3_t15 * s25_s15 * s25_s15;

			//std::cout << i << std::setprecision(15) << "  p:  " << p0[0] <<"  " << p1[0] << "  " << p2[0] << "  " << p3[0] << std::endl;
			//std::cout << i << std::setprecision(15) << "  s:  " << s0 << "  " << s1 << "  " << s2 << "  " << s3 << std::endl;
			//std::cout << i << std::setprecision(15) <<"  d2p_ds2_t15:   " << d2p_ds2_t15 << std::endl;
		}


		poss.push_back(p3[0]);
	}

	aris::dynamic::dlmwrite(n + 4, 1, poss.data(), "C:\\Users\\py033\\Desktop\\test_data\\poss.txt");
}

auto test_smooth() -> void {
	




	const double dt = 0.001;

	double a0 = 5.0;
	double v0 = 1.0 - a0 * dt * 2;
	double v1 = v0 + a0 * dt;
	double v2 = v1 + a0 * dt;

	double pos0 = 0.0;
	double pos1 = pos0 + v0 * dt;
	double pos2 = pos1 + v1 * dt;
	double pos3 = pos2 + v2 * dt;


	const int dim = 1;

	double min_p[dim]{ -100 };
	double max_p[dim]{ 100 };
	double min_dp[dim]{ -0.25 };
	double max_dp[dim]{ 2.25 };
	double min_d2p[dim]{ -5.0 };
	double max_d2p[dim]{ 5.0 };
	double min_d3p[dim]{ -10.0 };
	double max_d3p[dim]{ 10.0 };
	
	double p0[dim]{ pos0 };
	double p1[dim]{ pos1 };
	double p2[dim]{ pos2 };
	double p3[dim]{ pos3 };

	SmoothParam p{
		dt,
		dim,
		min_p, max_p, min_dp, max_dp, min_d2p, max_d2p, min_d3p, max_d3p,
		0.005, 1.0, -10, 10, -10000, 10000,
		0.1, 0.1, 0.1,
		p0, p1, p2, p3,
		1.0
	};

	SmoothRet ret;

	s_smooth_curve2(p, ret);

}


void test_singular_processor(){
	std::cout << std::endl << "-----------------test processor---------------------" << std::endl;

	test_smooth_cond_3_1();
	//test_singular_processor_1();

	std::cout << "-----------------test processor finished------------" << std::endl << std::endl;
}

