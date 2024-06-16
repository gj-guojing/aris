#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <limits>
#include <sstream>
#include <cstddef>
#include <array>
#include <list>


#include "aris/dynamic/math_optimization.hpp"
#include "aris/dynamic/math_matrix.hpp"

//#define ARIS_DEBUG_QP

namespace aris::dynamic {
	auto ARIS_API s_optimize(OptimizationParam& param)->double {
		const double zero_check = 1e-14;
		const double ds = 1e-2;
		const double iter_max_step = 0.1;
		
		Size m = param.observe_size * param.r_size;
		Size n = param.param_size;
		
		std::vector<double> J_vec(m * n);
		double* J = J_vec.data();

		std::vector<double> p0_vec(n), p1_vec(n), gradient_vec(std::max(m, n));  // 求解s_householder_sov 时，x的大小为 max(m,n)
		double* p0= p0_vec.data(), * p1 = p1_vec.data(), *gradient = gradient_vec.data();

		std::copy_n(param.init_param, n, p0);

		std::vector<double> r_vec(m), r1_vec(param.r_size), r2_vec(param.r_size);
		double* r = r_vec.data(), *r1 = r1_vec.data(), *r2 = r2_vec.data();

		std::vector<double> U_vec(m * n);
		double* U = U_vec.data();
		std::vector<Size> p_vec(std::max(m, n));
		auto p = p_vec.data();

		for (;;) {
			// compute J and residual（r） //
			for (int i = 0; i < param.observe_size; ++i) {
				param.cost_func(p0, param.x + i * param.x_size, r + i * param.r_size);

				for (int j = 0; j < n; ++j) {
					double p_reserve = p0[j];
					p0[j] = p_reserve + ds;
					param.cost_func(p0, param.x + i * param.x_size, r1);
					p0[j] = p_reserve - ds;
					param.cost_func(p0, param.x + i * param.x_size, r2);
					p0[j] = p_reserve;

					aris::dynamic::s_vs(param.r_size, r2, r1);
					aris::dynamic::s_nv(param.r_size, 1.0 / (2 * ds), r1);
					s_mc(param.r_size, 1, r1, 1, J + at(i * param.r_size, j, n), n);
				}
			}

			// 残差为0，在最优点 //
			double r_norm = s_norm(m, r);
			if (r_norm < zero_check) {
				s_vc(n, p0, param.param);
				return r_norm;
			}

			// compute gradient = J^-1 * residual //
			aris::Size rank;
			s_householder_up(m, n, J, U, p, rank, zero_check);
			s_householder_up_sov(m, n, 1, rank, U, p, r, gradient, zero_check);

			// Jacobi 为 0，在最优点 //
			if (rank == 0) {
				s_vc(n, p0, param.param);
				return r_norm;
			}

			// 梯度为 0，在最优点 //
			double gradient_norm = s_norm(n, gradient);
			if (gradient_norm < zero_check) {
				s_vc(n, p0, param.param);
				return r_norm;
			}

			// 将梯度归一化 //
			s_nv(n, 1.0 / gradient_norm, gradient);
			
			// 计算步长，并更新新的最优点 p1 //
			double step = std::min(gradient_norm, iter_max_step);
			for (;;) {
				// 计算 residual_mid
				s_vc(n, p0, p1);
				s_va(n, -step*0.5, gradient, p1);
				for (int i = 0; i < param.observe_size; ++i) {
					param.cost_func(p1, param.x + i * param.x_size, r + i*param.r_size);
				}
				double r_mid_norm = s_norm(m, r);

				// 计算 residual_next
				s_vc(n, p0, p1);
				s_va(n, -step, gradient, p1);
				for (int i = 0; i < param.observe_size; ++i) {
					param.cost_func(p1, param.x + i * param.x_size, r + i * param.r_size);
				}
				double r1_norm = s_norm(m, r);
				
				if ((r1_norm < r_norm && r1_norm < r_mid_norm) || step < zero_check) {
					r_norm = r1_norm;
					break;
				}
				step = step * std::min(r_norm / r1_norm, 0.5);
			}

			// 步长过小，返回 //
			if (step < zero_check) {
				std::copy_n(p0, n, param.param);
				return r_norm;
			}

			// move p1 to p0 //
			s_vc(n, p1, p0);
		}

		

		return 0;
	}
}
