//#define ARIS_DEBUG

#include "test_dynamic_kinematics.h"
#include <iostream>
#include <aris/dynamic/dynamic.hpp>
#include <aris/dynamic/kinematics.hpp>
#include <random>


using namespace aris::dynamic;


namespace aris::dynamic {
	//auto s_eye_in_hand_calib(int n, const double* pq_obj_in_eye, const double* pq_tool_in_base, double* eye_in_tool) -> void {
	//	std::cout << "aaa" << std::endl;
	//}

	//auto s_put_into_period(double value, double which_period, double period) -> double {
	//	// 获取当前值的
	//	auto ret = std::fmod(value, period);

	//	// 整数周期 
	//	auto t = std::trunc(which_period);
	//	// 取余的周期
	//	auto mod = which_period - t;

	//	while (ret > (mod + 0.5) * period) ret -= period;
	//	while (ret < (mod - 0.5) * period) ret += period;

	//	ret += t * period;


	//	std::cout << "ccc" << std::endl;
	//	return ret;
	//}

}

void test_eye_in_hand()
{
	const double pe_eye_in_tool[6]{ 0.1,0.2,0.3,0.111,0.221,0.832 };
	double pq_eye_in_tool[7];
	s_pe2pq(pe_eye_in_tool, pq_eye_in_tool, "321");

	std::cout << "pq_eye_in tool:" << std::endl;
	aris::dynamic::dsp(1, 7, pq_eye_in_tool);

	const double pe_obj_in_base[6]{ 1.23,2.2,4.3,0.511,0.321,0.932 };
	double pq_obj_in_base[7];
	s_pe2pq(pe_obj_in_base, pq_obj_in_base, "321");

	// make data
	const int n = 10;
	double pe_tool_in_base[n][6]{
		{0.1,0.2,0.1,0.1,0.2,0.3},
		{0.2,0.4,-0.3,0.5,0.2,0.3},
		{0.3,0.6,0.3,0.8,0.2,0.3},
		{0.3,0.8,-0.3,0.1,0.2,0.3},
		{0.2,0.9,0.3,0.1,0.5,0.3},
		{0.1,0.9,-0.3,0.1,0.8,0.3},
		{0.1,0.8,0.3,0.1,0.2,0.3},
		{0.2,0.7,-0.3,0.1,0.2,0.6},
		{0.3,0.6,0.3,0.1,0.2,0.9},
	};


	double pq_tool_in_base[n][7];
	double pq_obj_in_eye[n][7];

	for (int i= 0; i < n; ++i) {
		s_pe2pq(pe_tool_in_base[i], pq_tool_in_base[i], "123");


		double pq_eye_in_base[7];
		s_pq_dot_pq(pq_tool_in_base[i], pq_eye_in_tool, pq_eye_in_base);

		double pm_eye_in_base[16];
		s_pq2pm(pq_eye_in_base, pm_eye_in_base);
		
		s_inv_pq2pq(pm_eye_in_base, pq_obj_in_base, pq_obj_in_eye[i]);
	}


	//dsp(1, 4, plane2);
	double pq_eye_in_tool_result[7];
	double mem[16 * n * n];
	aris::dynamic::s_eye_in_hand_calib(n, *pq_obj_in_eye, *pq_tool_in_base, pq_eye_in_tool_result, mem);

	aris::dynamic::dsp(1, 7, pq_eye_in_tool_result);
	aris::dynamic::dsp(1, 7, pq_eye_in_tool);

	std::cout << "wait" << std::endl;
}

void test_eye_in_hand2()
{
	const double pe_eye_in_tool[6]{ 0.1,0.2,0.3,0.111,0.221,0.832 };
	double pq_eye_in_tool[7];
	s_pe2pq(pe_eye_in_tool, pq_eye_in_tool, "321");

	std::cout << "pq_eye_in tool:" << std::endl;
	aris::dynamic::dsp(1, 7, pq_eye_in_tool);

	const double pe_obj_in_base[6]{ 1.23,2.2,4.3,0.511,0.321,0.932 };
	double pq_obj_in_base[7];
	s_pe2pq(pe_obj_in_base, pq_obj_in_base, "321");

	// make data
	const int n = 8;
	double pe_tool_in_base[n][6]{
		{805.274911,315.296968,622.923222,270.089615,7.576354,143.475021},
		{807.528099,192.358122,533.766378,252.773708,16.775748,139.864493 },
		{810.656227,193.447522,555.544276,252.160947,17.216653,139.284136},
		{703.055386,393.973543,635.753282,297.275858,-2.776476,146.597051},
		{736.764133,407.004243,576.898704,297.385539,-2.511494,144.171319},
		{774.181753,376.340377,550.681197,294.633840,-2.338603,142.601433},
		{708.394648,560.365043,546.019583,327.438039,-15.554671,127.257024},
		{788.483623,369.193844,556.525055,291.617520,0.320245,141.774271}
	};
	double pe_obj_in_eye[n][6]{
		{ -8.54, 14.123, 516.975, 1.64404, 0.083299, 3.1057},
		{24.097, 4.539, 440.932, 1.3541, 0.096654, 2.90711},
		{13.153, 9.99, 461.617, 1.34519, 0.102088, 2.89634},
		{-13.778, 29.961, 544.16, 2.10252, - 0.057346, - 3.00314},
		{-22.242, 17.293, 483.206, 2.10872, - 0.018245, - 2.98404},
		{17.09, 15.964, 449.284, 2.06448, 0.016135, - 2.97464},
		{9.191, 26.467, 499.036, 2.55876, - 0.138528, - 2.63453},
		{3.022, 18.44, 452.709, 2.01848, 0.057736, - 3.01188},
	};




	double pq_tool_in_base[n][7];
	double pq_obj_in_eye[n][7];

	for (int i = 0; i < n; ++i) {
		pe_tool_in_base[i][3] *= aris::PI / 180;
		pe_tool_in_base[i][4] *= aris::PI / 180;
		pe_tool_in_base[i][5] *= aris::PI / 180;

		s_pe2pq(pe_tool_in_base[i], pq_tool_in_base[i], "321");
		s_pe2pq(pe_obj_in_eye[i], pq_obj_in_eye[i], "321");

	}
	
	double pq_eye_in_tool_result[7];
	double mem[16 * n * n];
	aris::dynamic::s_eye_in_hand_calib(n, *pq_obj_in_eye, *pq_tool_in_base, pq_eye_in_tool_result, mem);
	aris::dynamic::dsp(1, 7, pq_eye_in_tool_result);

	for (int i = 0; i < n; ++i) {
		double pq_eye_wrt_tool[7]{ 26.4478, - 143.284, 81.5701, 0.0514753, - 0.263611, 0.963143, -0.0146969 };
		//double pq_eye_wrt_tool[7]{ 27.924852, - 141.300530,   82.107110, - 0.049961,   0.265327, - 0.962756,   0.014378 };
		aris::dynamic::s_vc(7, pq_eye_in_tool_result, pq_eye_wrt_tool);
		
		double pq1[7], pq2[7];
		s_pq_dot_pq(pq_eye_wrt_tool, pq_obj_in_eye[i], pq1);
		s_pq_dot_pq(pq_tool_in_base[i], pq1,pq2);
		aris::dynamic::dsp(1, 7, pq2);
	}


	std::cout << "wait" << std::endl;
}

void test_kinematics()
{
	std::cout << std::endl << "-----------------test kinematics--------------------" << std::endl;

	test_eye_in_hand2();
	//test_interp_plane();

	


	std::cout << "-----------------test kinematics finished-----------" << std::endl << std::endl;
}