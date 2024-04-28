#ifndef ARIS_PLAN_FUNCTION_H_
#define ARIS_PLAN_FUNCTION_H_


#include <aris/core/basic_type.hpp>

#include <string>
#include <map>
#include <cmath>
#include <aris/core/core.hpp>

namespace aris::plan
{
	auto inline acc_up(double n, double i)noexcept->double { return (-1.0 / 2 / n / n / n * i*i*i + 3.0 / 2.0 / n / n * i*i); }
	auto inline acc_down(double n, double i)noexcept->double { return (-1.0*i*i*i / 2.0 / n / n / n + 3.0 * i*i / 2.0 / n / n); }
	auto inline dec_up(double n, double i)noexcept->double { return 1.0 - (-1.0 / 2.0 / n / n / n * (n - i)*(n - i)*(n - i) + 3.0 / 2.0 / n / n * (n - i)*(n - i)); }
	auto inline dec_down(double n, double i)noexcept->double { return 1.0 - (-1.0*(n - i)*(n - i)*(n - i) / 2.0 / n / n / n + 3.0 * (n - i)*(n - i) / 2.0 / n / n); }

	auto inline acc_even(double n, double i)noexcept->double { return 1.0 / n / n * i * i; }
	auto inline dec_even(double n, double i)noexcept->double { return 1.0 - 1.0 / n / n * (n - i)*(n - i); }
	auto inline even(double n, double i)noexcept->double { return 1.0 / n * i; }

	auto inline s_p2p(double n, double i, double begin_pos, double end_pos)noexcept->double
	{
		double a = 4.0 * (end_pos - begin_pos) / n / n;
		return i <= n / 2.0 ? 0.5*a*i*i + begin_pos : end_pos - 0.5*a*(n - i)*(n - i);
	}
	auto inline s_v2v(int n, int i, double begin_vel, double end_vel)noexcept->double
	{
		double s = static_cast<double>(i) / n;
		double m = 1.0 - s;

		return (s*s*s - s * s)*end_vel*n + (m*m - m * m*m)*begin_vel*n;
	}
	auto inline s_interp(int n, int i, double begin_pos, double end_pos, double begin_vel, double end_vel)noexcept->double{
		double s = static_cast<double>(i) / n;

		double a, b, c, d;

		c = begin_vel * n;
		d = begin_pos;
		a = end_vel * n - 2.0 * end_pos + c + 2.0 * d;
		b = end_pos - c - d - a;

		return a * s*s*s + b * s*s + c * s + d;
	}

	auto ARIS_API moveAbsolute(double i, double begin_pos, double end_pos, double vel, double acc, double dec, double &current_pos, double &current_vel, double &current_acc, Size& total_count)->void;

	// pa : pos actual value
	// va : vel actual value
	// aa : acc actual value
	// pt : pos target value
	// vt : vel target value
	// at : acc target value
	// vm : vel max permitted value, always positive
	// am : acc max permitted value, always positive
	// dm : dec max permitted value, always positive
	// dt : time inteval, set to 1e-3
	// zero_check : set to 1e-10
	// pc : next planned pos
	// vc : next planned vel
	// ac : next planned acc
	// total_count : tbd, not finished yet
	auto ARIS_API moveAbsolute2(double pa, double va, double aa, double pt, double vt, double at, double vm, double am, double dm, double dt, double zero_check, double &pc, double &vc, double &ac, Size& total_count)->int;


	auto inline ARIS_API s_is_in_vavg_boundage(double v0, double v1, double vavg, double a_max, double a_min, double dt, double zero_check = 1e-10)->bool {
		if (v1 < v0)
			return s_is_in_vavg_boundage(v1, v0, vavg, -a_min, -a_max, dt, zero_check);

		if (a_max < 0 || a_min > 0)
			return false;

		// 
		// 从 v0 加速到 v1 所需时间为 t1，其他时间为 dt - t1
		// 其他时间可以在 v0 到 v1 的任何数值上进行加速减速，设若该加速所产生的平均速度偏移是 r
		// 
		// 其他时间的加速时间为 Ta，减速为 Tb
		// 有 Ta + Tb = dt - t1
		// r = Ta * a_max/2 = - Tb*a_min/2
		// Ta = -a_min/a_max * Tb
		// 
		// => Ta + Tb = (a_max - a_min)/a_max * Tb = dt - t1
		// => Tb = a_max/(a_max - a_min) * (dt - t1)
		// => r  = -Tb*a_min/2
		//       = -a_max*a_min/(a_max - a_min)*(dt - t1)/2
		// 


		// cpt r //
		double t1 = (v1 - v0) / a_max;
		double r = -a_max * a_min / (a_max - a_min) * (dt - t1)/2;

		// too small margin //
		if (std::abs(dt - t1) < zero_check) {
			double dis = std::abs(vavg - (v1 + v0) / 2);
			return dis <= r + zero_check;
		}

		// normal check //
		// 
		// 若 va 是从 v0 加速到 v1 的平均速度，vb 是其他过程的平均速度
		// 于是针对平均速度 v_avg，有
		// v_avg * dt = va *t1 + vb*(dt - t1)
		//
		// 继而：
		// vb = （v_avg * dt - va *t1）/ (dt - t1)
		//
		double vavg_left = (vavg * dt - (v0 + v1) / 2 * t1) / (dt - t1);

		// 正常情况需求 vb 偏离  v0 -> v1 这根轴的距离

		double dis_d;
		double *dis = &dis_d;

		if (vavg_left < v0) {
			*dis = v0 - vavg_left;
		}
		else if (vavg_left > v1) {
			*dis = vavg_left - v1;
		}
		else {
			*dis = 0.0;
		}

		return *dis < r;
	};


	//          pa : pos actual value
	//          va : vel actual value
	//          pt : pos target value
	//       v_max : vel max permitted value
	//       v_min : vel min permitted value
	//       a_max : acc max permitted value
	//       a_min : acc min permitted value
	//          dt : time inteval
	//  zero_check : set to 1e-10
	//          pc : next planned pos
	//          vc : next planned vel
	//          ac : next planned acc
	// total_count : tbd, not finished yet
	auto ARIS_API inline s_follow_x(double pa, double va, double pt, double v_max, double v_min, double a_max, double a_min, double dt, double zero_check, double& pc, double& vc, double& ac, Size& total_count)->int {
		
		// 如果位置方向为负
		if (pt - pa < 0) {
			double pa2 = -pa;
			double pt2 = -pt;
			double va2 = -va;
			double v_max2 = -v_min;
			double v_min2 = -v_max;
			double a_max2 = -a_min;
			double a_min2 = -a_max;
			double pc2, vc2, ac2;

			s_follow_x(pa2, va2, pt2, v_max2, v_min2, a_max2, a_min2, dt, zero_check, pc2, vc2, ac2, total_count);

			pc = -pc2;
			vc = -vc2;
			ac = -ac2;
			return 0;
		}

		// 当前速度超过速度上下限 //
		{
			if (va + a_min * dt > v_max) {
				ac = a_min;
				goto return_flag;
			}
			if (va + a_max * dt < v_min) {
				ac = a_max;
				goto return_flag;
			}
		}

		// 查看当前速度是否过快 //
		{
			// a_min_real, a_max_real 还需考虑速度不超限
			double a_min_real = std::max((v_min - va) / dt, a_min);
			a_min_real = std::min(a_max, a_min_real);
			double a_max_real = std::min((v_max - va) / dt, a_max);
			a_max_real = std::max(a_min, a_max_real);

			//if (va > 0) {
				// 【CASE 0】没有减速能力
				if (va > 0 && a_min > -zero_check) {
					ac = a_min;
					goto return_flag;
				}
				if (va < 0 && a_max < zero_check) {
					ac = a_max;
					goto return_flag;
				}

				// 【CASE 1】已经到达目标位置
				if (s_is_in_vavg_boundage(va, 0.0, (pt - pa) / dt, a_max, a_min, dt, zero_check)) {
					ac = (0.0 - va) / dt;
					vc = 0.0;
					pc = pt;
					total_count = 1;
					return 0;
				}

				// 【CASE 2】最短的减速段距离，大于当前还剩的距离，最短减速距离还需要考虑圆整
				const double vdec = va;
				const double ndec = std::ceil(-vdec / a_min / dt);
				const double tdec = ndec * dt;
				const double sdec = vdec * tdec / 2;
				if (vdec > 0 && pt - pa - sdec < zero_check) {
					ac = -va / ndec / dt;
					goto return_flag;
				}

				// 【CASE 3】当前还可以用 a_max_real 加速
				const double vdec1 = va + a_max_real * dt;
				const double ndec1 = std::ceil(-vdec1 / a_min / dt);
				const double tdec1 = ndec1 * dt;
				const double sdec1 = vdec1 * tdec1 / 2;
				if (vdec1 < 0 || sdec1 < pt - (pa + va * dt + 0.5 * a_max_real * dt * dt)) {
					ac = a_max_real;
					goto return_flag;
				}

				// 【CASE 4】当前加速度需要小于 a_max 大于 a_min，减速的步数在 ndec 与 ndec1 之间
				// v_next = -ndec * dt * a_min
				// 
				// pt - pa = (va + v_next)/2*dt + v_next * ndec * dt /2
				// => va*dt/2 - a_min*dt^2/2*ndec - (a_min*dt^2*ndec^2)/2 - (pt - pa) == 0
				// => ndec^2 + ndec - (va * dt/2 - (pt - pa))/(a_min * dt * dt)
				// =>
				// => ndec 应取大值，即：(-B + std::sqrt(B * B - 4 * A * C)) / (2 * A)
				//
				// => 即
				//double A = 1;
				//double B = 1;
				//double C = (va * dt / 2 - (pt - pa)) / (-a_min * dt * dt / 2);
				//double n_ideal = (-B + std::sqrt(B * B - 4 * A * C)) / (2 * A);
				//const double ndec2 = std::ceil(n_ideal);
				// 实际将 A, B 带入可得：
				double C = (va * dt / 2 - (pt - pa)) / (-a_min * dt * dt / 2);
				double n_ideal = (-1.0 + std::sqrt(std::max(1.0 - 4.0 * C, 0.0))) / 2.0;
				const double ndec2 = std::ceil(n_ideal);

				// pt - pa = (va + v_next)/2*dt + v_next * ndec * dt /2
				// 
				// => pt - pa == (va + v_next * (ndec + 1)) * dt/2
				// => pt - pa == (ndec + 1)*dt/2 * v_next + va * dt/2
				// 
				// => v_next = (2*(pt-pa)-dt*va)/(dt*(ndec+1))
				//
				// 但是如此计算出来的 v_next 可能会导致 ac 过大，这是因为 ndec 相比于 CASE2 有增加
				//
				double v_next = (2 * (pt - pa) - dt * va) / (dt * (ndec2 + 1));
				ac = std::max((v_next - va) / dt, a_min_real);
				goto return_flag;
			//}

			
			// 计算完全减速所产生的位移，vdec和sdec有方向 //
			//const double vdec = va;
			//const double ndec = std::abs(std::ceil(std::abs(vdec) / dm / dt - zero_check));
			//const double tdec = ndec * dt;
			//const double sdec = vdec * tdec / 2;

			// 当前速度需要完全减速到零，之后可能还需要反转 //
			//if ((va > zero_check && (pt - pa) < sdec + zero_check) || (va < -zero_check && (pt - pa) > sdec - zero_check))
			//{
			//	ac = -va / ndec / dt;
			//	goto return_flag;
			//}
		}

	return_flag:
		vc = va + ac * dt;
		pc = pa + 0.5 * (va + vc) * dt;
		total_count = 1;
		return std::abs(pt - pc) < zero_check ? 0 : 1;
	}


}


#endif