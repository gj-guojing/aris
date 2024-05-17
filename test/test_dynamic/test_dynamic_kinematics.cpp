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
	s_pe2pq(pe_eye_in_tool, pq_eye_in_tool, "123");

	std::cout << "pq_eye_in tool:" << std::endl;
	aris::dynamic::dsp(1, 7, pq_eye_in_tool);

	const double pe_obj_in_base[6]{ 1.23,2.2,4.3,0.511,0.321,0.932 };
	double pq_obj_in_base[7];
	s_pe2pq(pe_obj_in_base, pq_obj_in_base, "123");

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


void test_kinematics()
{
	std::cout << std::endl << "-----------------test kinematics--------------------" << std::endl;

	test_eye_in_hand();
	//test_interp_plane();

	std::cout << "-----------------test kinematics finished-----------" << std::endl << std::endl;
}