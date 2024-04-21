#include"aris/plan/singular_processor.hpp"
#include"aris/plan/function.hpp"

//#define ARIS_DEBUG_SINGULAR_PROCESSOR

namespace aris::plan {
	struct ThirdPolynomialParam {
		double a, b, c, d;
	};
	auto s_third_polynomial_compute_Tmin(double pb, double pe, double vb, double ve, double vmax, double amax)->double {
		// 根据速度算 T 的最小值
		double A1 = (-vb * vb - vb * ve - 3 * vmax * vb - ve * ve - 3 * vmax * ve);
		double B1 = (6 * pe * vb - 6 * pb * ve - 6 * pb * vb + 6 * pe * ve - 6 * pb * vmax + 6 * pe * vmax);
		double C1 = (-9 * pb * pb + 18 * pb * pe - 9 * pe * pe);

		double A2 = (-vb * vb - vb * ve + 3 * vmax * vb - ve * ve + 3 * vmax * ve);
		double B2 = (6 * pe * vb - 6 * pb * ve - 6 * pb * vb + 6 * pe * ve + 6 * pb * vmax - 6 * pe * vmax);
		double C2 = (-9 * pb * pb + 18 * pb * pe - 9 * pe * pe);

		double T_min_candidate[4]{ -1,-1,-1,-1 };

		if (B1 * B1 - 4 * A1 * C1 > 0) {
			T_min_candidate[0] = (-B1 - std::sqrt(B1 * B1 - 4 * A1 * C1)) / (2 * A1);
			T_min_candidate[1] = (-B1 + std::sqrt(B1 * B1 - 4 * A1 * C1)) / (2 * A1);
		}

		if (B2 * B2 - 4 * A2 * C2 > 0) {
			T_min_candidate[2] = (-B2 - std::sqrt(B2 * B2 - 4 * A2 * C2)) / (2 * A2);
			T_min_candidate[3] = (-B2 + std::sqrt(B2 * B2 - 4 * A2 * C2)) / (2 * A2);
		}

		std::sort(T_min_candidate, T_min_candidate + 4);

		double Tmin = 0.0;
		for (int i = 0; i < 4; ++i) {
			double T = T_min_candidate[3 - i];
			double m = (T * (3 * pb - 3 * pe + 2 * T * vb + T * ve)) / (3 * (2 * pb - 2 * pe + T * vb + T * ve));
			if ((0 < m && m < T) || T < 0.0) {
				Tmin = T;
				break;
			}
		}

		// 根据加速度算 T 最小值
		double coes[4][3]{
			{-amax, (2 * vb + 4 * ve), 6 * pb - 6 * pe},
			{amax, (2 * vb + 4 * ve), 6 * pb - 6 * pe},
			{-amax, -(4 * vb + 2 * ve), -6 * pb + 6 * pe},
			{amax, -(4 * vb + 2 * ve), -6 * pb + 6 * pe}
		};

		for (int i = 0; i < 4; ++i) {
			double A = coes[i][0];
			double B = coes[i][1];
			double C = coes[i][2];

			if (B * B - 4 * A * C > 0) {

#ifdef ARIS_DEBUG_SINGULAR_PROCESSOR
				std::cout << "Tmin1:" << (-B + std::sqrt(B * B - 4 * A * C)) / (2 * A) << std::endl;
				std::cout << "Tmin1:" << (-B - std::sqrt(B * B - 4 * A * C)) / (2 * A) << std::endl;
#endif
				Tmin = std::max(Tmin, (-B + std::sqrt(B * B - 4 * A * C)) / (2 * A));
				Tmin = std::max(Tmin, (-B - std::sqrt(B * B - 4 * A * C)) / (2 * A));
			}
		}
#ifdef ARIS_DEBUG_SINGULAR_PROCESSOR
		std::cout << "Tmin:" << Tmin << std::endl;
#endif
		return Tmin;
	}
	auto s_third_polynomial_compute_param(double pb, double pe, double vb, double ve, double T)->ThirdPolynomialParam {
		ThirdPolynomialParam param;

		param.a = (2 * pb - 2 * pe + T * vb + T * ve) / T / T / T;
		param.b = -(3 * pb - 3 * pe + 2 * T * vb + T * ve) / T / T;
		param.c = vb;
		param.d = pb;

		return param;
	}
	auto s_third_polynomial_compute_value(const ThirdPolynomialParam& param, double t)->double {
		return param.a * t * t * t + param.b * t * t + param.c * t + param.d;
	}

	struct TcurveParam {
		double pb, pe, vb, ve, vmax, amax;
		double T, Ta, Tb, v, a;
		int mode;
	};
	// 计算可行的时间域
	// (T1 T2), (T3,inf)
	auto s_tcurve_T_range(const TcurveParam& param, double& T1, double& T2, double& T3)->void {
		// 
		const double pb = param.pb;
		const double pe = param.pe;
		const double vb = param.vb;
		const double ve = param.ve;
		const double vmax = param.vmax;
		const double amax = param.amax;

		// 
		const double pt = pe - pb;
		const double Tb2e = std::abs(vb - ve) / amax;
		const double pb2e = (vb + ve) / 2 * Tb2e;

		double v;
		if (pb2e < pt) {
			if ((vmax + vb) / 2 * (vmax - vb) / amax + (vmax + ve) / 2 * (vmax - ve) / amax < pt) {
				v = vmax;
				double Ta = (v - vb) / amax;
				double Tb = (v - ve) / amax;
				T1 = (pt - Ta * (vb + v) / 2 - Tb * (ve + v) / 2) / v + Ta + Tb;
			}
			else {
				v = std::sqrt(std::abs(pt * amax + (vb * vb + ve * ve) / 2));
				T1 = (v - vb) / amax + (v - ve) / amax;
			}
		}
		else {
			if ((-vmax + vb) / 2 * (vb + vmax) / amax + (-vmax + ve) / 2 * (ve + vmax) / amax > pt) {
				v = -vmax;
				double Ta = (vb - v) / amax;
				double Tb = (ve - v) / amax;
				T1 = (pt - Ta * (vb + v) / 2 - Tb * (ve + v) / 2) / v + Ta + Tb;
			}
			else {
				v = -std::sqrt(std::abs(-pt * amax + (vb * vb + ve * ve) / 2));
				T1 = (vb - v) / amax + (ve - v) / amax;
			}
		}

		T2 = T1;
		T3 = T1;
		if (v > 0 && vb > 0 && ve > 0) {
			if (vb * vb / 2 / amax + ve * ve / 2 / amax > pt) {
				v = std::sqrt(std::abs(-pt * amax + (vb * vb + ve * ve) / 2));
				T2 = std::abs(vb - v) / amax + std::abs(ve - v) / amax;
				T3 = std::abs(vb + v) / amax + std::abs(ve + v) / amax;
			}
		}
		else if (v < 0 && vb < 0 && ve < 0) {
			if (-vb * vb / 2 / amax - ve * ve / 2 / amax < pt) {
				v = -std::sqrt(std::abs(pt * amax + (vb * vb + ve * ve) / 2));
				T2 = std::abs(vb - v) / amax + std::abs(ve - v) / amax;
				T3 = std::abs(vb + v) / amax + std::abs(ve + v) / amax;
			}
		}
	}
	// 计算param
	auto s_tcurve_param(TcurveParam& param)->void {
		const auto pb = param.pb;
		const auto pe = param.pe;
		const auto vb = param.vb;
		const auto ve = param.ve;
		const auto vmax = param.vmax;
		const auto amax = param.amax;
		const auto T = param.T;

		auto& a = param.a;
		auto& v = param.v;
		auto& Ta = param.Ta;
		auto& Tb = param.Tb;
		auto& mode = param.mode;

		double pt = pe - pb;
		double Tb2e = std::abs(vb - ve) / amax;
		double pb2e = (vb + ve) / 2 * Tb2e;


		if ((pt - pb2e) > (T - Tb2e) * std::max(vb, ve)) {
			param.mode = 0;
			param.a = amax;

			double A = 1;
			double B = -T * a - vb - ve;
			double C = (vb * vb + ve * ve) / 2 + pt * a;

			param.v = (-B - std::sqrt(std::abs(B * B - 4 * A * C))) / (2 * A);
			param.Ta = (v - vb) / a;
			param.Tb = (v - ve) / a;
		}

		else if ((pt - pb2e) > (T - Tb2e) * std::min(vb, ve)) {
			param.mode = 1;
			param.v = (pt - pb2e) / (T - Tb2e);
			Ta = (pt - pb2e) / (vb - ve) - ve / (vb - ve) * (T - Tb2e);
			Ta = std::max(0.0, Ta);
			param.Ta = std::min(Ta, T - Tb2e);
			param.Tb = T - Tb2e - Ta;
			param.a = aris::dynamic::s_sgn2(ve - vb) * amax;
		}

		else {
			param.mode = 0;
			param.a = -amax;
			double A = 1;
			double B = -T * a - vb - ve;
			double C = (vb * vb + ve * ve) / 2 + pt * a;

			param.v = (-B + std::sqrt(std::abs(B * B - 4 * A * C))) / (2 * A);
			param.Ta = (v - vb) / a;
			param.Tb = (v - ve) / a;
		}
	}
	// 计算值
	auto s_tcurve_value(const TcurveParam& param, double t)->double {
		const auto pb = param.pb;
		const auto pe = param.pe;
		const auto vb = param.vb;
		const auto ve = param.ve;
		const auto vmax = param.vmax;
		const auto amax = param.amax;
		const auto T = param.T;

		const auto a = param.a;
		const auto v = param.v;
		const auto Ta = param.Ta;
		const auto Tb = param.Tb;
		const auto mode = param.mode;


		if (mode == 0)
			if (t < Ta)
				return pb + vb * t + a * t * t / 2;
			else if (t < T - Tb)
				return pb + vb * Ta + a * Ta * Ta / 2 + v * (t - Ta);
			else
				return pe - ve * (T - t) - a * (T - t) * (T - t) / 2;
		else
			if (t < Ta)
				return pb + vb * t;
			else if (t < T - Tb)
				return pb + vb * t + a * (t - Ta) * (t - Ta) / 2;
			else
				return  pe - ve * (T - t);
	}

	struct SmoothParam {
		double dt;
		int dim;
		const double *min_dp, * max_dp, *min_d2p, * max_d2p, *min_d3p, * max_d3p;
		double ds1, ds2, ds3;
		double* p0, *p1, *p2, *p3;

		double target_ds;
	};
	struct SmoothRet {
		double next_ds;
	};
	auto s_smooth_curve(const SmoothParam &param, SmoothRet &ret) -> int {
		
		///////////////////////////// PART 1 计算 d3s 的可选范围 ///////////////////////////////// 
		// 
		// 
		// 
		// 
		// % 以下考虑约束条件
		//% 【COND 1】经过 dt 时间后，速度不超过上下限
		//%  
		//%  dp_min < dp3 + (d2p3 + d3p25*dt)*dt < dp_max
		//%  =>
		//%  (dp_min - dp3 - d2p3*dt - g*dt*dt)/k/dt/dt 
		//%  <
		//%    d3s
		//%  <
		//%  (dp_max - dp3 - d2p3*dt - g*dt*dt)/k/dt/dt 
		//% 【COND 2】经过 dt 时间后，加速度不超过上下限
		//%
		//%  d2p_min < d2p3 + d3p_25 * dt < d2p_max
		//%
		//%  (d2p_min - d2p3 - g*dt)/k/dt
		//%
		//% 【COND 3】jerk 不超过上下限
		//%
		//%  d3p_min < d3p_25 < d3p_max
		//% 
		//% 【COND 4】jerk 不超过考虑速度与加速度的上下限
		//%
		//%  d2p_min2 < d2p3 + d3p_25 * dt < d2p_max2
		//%
		//%
		//% 【COND 4】推导如下：
		//%
		//%  将加速度减为0，所需时间：T = abs(d2p)/d3p_max
		//%  减为0时的速度：dp + T*d2p/2
		//%  dp_min < dp + T*d2p/2 < dp_max
		//%  =>
		//%  dp_min-dp < T*d2p/2 < dp_max-dp
		//%  2*(dp_min-dp)*d3p_max < sig(d2p)*d2p*d2p < 2*(dp_max-dp)*d3p_max
		//%  => 
		//%  d2p < 0时
		//%    d2p > -sqrt(2*(dp-dp_min)*d3p_max)
		//%  d2p > 0时
		//%    d2p < sqrt(2*(dp_max-dp)*d3p_max)
		//% 
		//%  考虑经过dt，则应有：
		//%  
		//%  d2p < 0时
		//%    d2p > -sqrt(2*(dp-dp_min-d2p_max*dt)*d3p_max)
		//%  d2p > 0时
		//%    d2p < sqrt(2*(dp_max-dp-d2p_max*dt)*d3p_max)
		// 
		// 应有以下等式：
		// dp_min  < dp  < dp_max
		// d2p_min < d2p < d2p_max
		// d3p_min < d3p < d3p_max
		//
		// dp  = dp_ds * ds
		// d2p = d2p_ds2 * ds^2 + dp_ds * d2s
		// d3p = d3p_ds3 * ds^3 + 2 * d2p_ds2 * ds * d2s + d2p_ds2 * ds * d2s + dp_ds * d3s
		//     = d3p_ds3 * ds^3 + 3 * d2p_ds2 * ds * d2s + dp_ds * d3s
		//
		// 带入上述不等式，应有：
		//                    dp_min/dp_ds < ds  < dp_max/dp_ds
		//  (d2p_max-d2p_ds2 * ds^2)/dp_ds < d2s < (d2p_max-d2p_ds2 * ds^2)/dp_ds
		//               (d3p_min-f)/dp_ds < d3s < (d3p_max-f)/dp_ds
		//
		// 其中 f = d3p_ds2 * ds^3 + 3 * d2p_ds * ds * d2s
		// 
		// 若 dp_ds < 0，则上式左右相反。
		//
		// 上式仅考虑了电机端的输入，没有考虑加速度将为 0 也需要时间。此时：
		//
		//       dp_min  <  dp + d2p * T + 0.5 * d3p * T^2  <  dp_max
		//                                  
		// 中间式子在 T = -d2p/d3p 时取到极值，大小为：
		//    
		//       dp - 0.5*d2p*d2p/d3p
		//
		// 若 d2p < 0 && dp_ds < 0，则应有 d3p > 0：
		//       dp_min < dp - 0.5*d2p*d2p/d3p
		// =>    d2p*d2p/d3p < 2*(dp-dp_min)
		// =>    d3p > d2p*d2p/(2(dp-dp_min))
		// =>    d3s < (d2p*d2p/(2(dp-dp_min)) - f)/dp_ds
		// 		
		// 若 d2p > 0 && dp_ds > 0，则应有 d3p > 0：
		//
		//       d3p < d2p*d2p/(2(dp_max-dp))
		// =>    d3s < (d2p*d2p/(2(dp-dp_max)) - f)/dp_ds
		//
		// 考虑加速度不超限，应有：
		// 
		//                           d2p_min  < d2p + d3p*dt < d2p_max
		// =>             (d2p_min - d2p)/dt  <      d3p     <  (d2p_max - d2p)/dt
		// =>  ((d2p_min - d2p)/dt - f)/dp_ds <      d3s     <  ((d2p_max - d2p)/dt - f)/dp_ds
		// 
		///////////////////////////// PART 2 计算其中的数据 ///////////////////////////////// 
		// t0        t1   t15   t2   t25   t3
		//
		// p0        p1         p2         p3
		//     dp1        dp2        dp3
		//          d2p2       d2p3
		//               d3p3
		//  
		// s0        s1         s2         s3
		//     ds1        ds2        ds3
		//          d2s2       d2s3
		//               d3s3
		//
		// 
		//
		// at point t=1.5:
		//
		// dp_ds_t15   = dp_t15/ds_t15
		// d2p_ds2_t15 = (d2p_t15 - dp_ds_t15 * d2s_t15)/ds_t15^2
		// d3p_ds3_t15 = (d3p_t15 - 3 * d2p_ds2_t15 * ds_t15 * d2s_t15 - dp_ds_t15 * d3s_t15)/ds_t15^3
		// 
		// at point t=2.5:
		// 
		// ds_t25 = ds3
		// d2s_25 = d2s_t15 + (d3s_t15+d3s_25)/2*dt
		// 
		// d3p_ds3_t25 = d3p_ds3_t15
		// d2p_ds2_t25 = d2p_ds2_t15 + d3p_ds3_t15 * (s_25 - s_15)
		// dp_ds_t25   = dp_ds_t15 + d2p_ds2_t15 * (s_25 - s_15) + 0.5 * d3p_ds3_t15 * (s_25 - s_15)^2
		// 
		// f_25 = d3p_ds3_t25 * ds_t25^3 + 3 * d2p_ds2_t25 * ds_t25 * d2s_25
		//      = d3p_ds3_t25 * ds_t25^3 + 3 * d2p_ds2_t25 * ds_t25 * (d2s_t15 + (d3s_t15+d3s_25)/2*dt)
		//      = d3p_ds3_t25 * ds_t25^3 + 3 * d2p_ds2_t25 * ds_t25 * (d2s15 + d3s_t15*dt/2) + g*dp_ds_t25* d3s_25
		//      = k + g*dp_ds_t25* d3s_25
		// 
		// 其中:
		//    k = d3p_ds3_t25 * ds_t25^3 + 3 * d2p_ds2_t25 * ds_t25 * (d2s15 + d3s_t15*dt/2)
		//    g = 3 * d2p_ds2_t25 * ds_t25 /2 * dt / dp_ds_t25
		// 
		// 根据以下3个不等式，可求出 d3s 的许可范围：
		//                 (d3p_min-f_25)/dp_ds_t25 < d3s_25 < (d3p_max-f_25)/dp_ds_t25
		// ((d2p_min - d2p_25)/dt - f_25)/dp_ds_t25 < d3s_25 < ((d2p_max - d2p_25)/dt - f_25)/dp_ds_t25
		//                                           d3s_25 < (d2p_25*d2p_25/(2(dp_25-dp_max)) - f_25)/dp_ds_t25
		// 
		// => 
		// 
		// 			                (d3p_min-k)/r < d3s_25 < (d3p_max-k)/r
		//          ((d2p_min - d2p_25)/dt - k)/r < d3s_25 < ((d2p_max - d2p_25)/dt - k)/r
		//                                          d3s_25 < (d2p_25*d2p_25/(2(dp_25-dp_max)) - k)/r
		// 
		// 其中 r = (1+g) * dp_ds_t25 = dp_ds_t25 + 3 * d2p_ds2_t25 * ds_t25 /2 * dt
		// 
		// 其中 不等式1 和 2 左右需要根据dp_ds_t25 的符号进行切换
		
		double zero_check = 1e-10;
		
		const double MAX_DS = 1.0;
		const double MIN_DS = std::min(0.005, param.target_ds);
		const double MAX_D2S = 10.0;
		const double MIN_D2S = -10.0;
		const double MAX_D3S = 10000.0;
		const double MIN_D3S = -10000.0;

		const double VEL_BOUND_RATIO = 0.99;
		const double ACC_BOUND_RATIO = 0.9;
		const double JERK_BOUND_RATIO = 0.999;
		

		double dt = param.dt;
		auto dim = param.dim;

		auto target_ds = param.target_ds;
		auto ds1 = param.ds1;
		auto ds2 = param.ds2;
		auto ds3 = param.ds3;

		auto p3 = param.p3;
		auto p2 = param.p2;
		auto p1 = param.p1;
		auto p0 = param.p0;
		
		auto dp_max = param.max_dp;
		auto dp_min = param.min_dp;
		auto d2p_max = param.max_d2p;
		auto d2p_min = param.min_d2p;
		auto d3p_max = param.max_d3p;
		auto d3p_min = param.min_d3p;

		auto d2s2 = (ds2 - ds1) / dt;
		auto d2s3 = (ds3 - ds2) / dt;
		auto d3s3 = (d2s3 - d2s2) / dt;

		double ds_t15 = ds2;
		double d2s_t15 = (d2s2 + d2s3) / 2;
		double d3s_t15 = d3s3;
		double ds_t25 = ds3;

		double d3s_min_1 = -1e10;
		double d3s_max_1 = 1e10;
		double d3s_min_2 = -1e10;
		double d3s_max_2 = 1e10;
		double d3s_min_3 = -1e10;
		double d3s_max_3 = 1e10;
		double d3s_min_4 = -1e10;
		double d3s_max_4 = 1e10;

		for (int i = 0; i < dim; ++i) {
			auto dp3 = (p3[i] - p2[i]) / dt;
			auto dp2 = (p2[i] - p1[i]) / dt;
			auto dp1 = (p1[i] - p0[i]) / dt;

			auto d2p3 = (dp3 - dp2) / dt;
			auto d2p2 = (dp2 - dp1) / dt;
			
			auto d3p3 = (d2p3 - d2p2) / dt;

			auto dp_t15 = dp2;
			auto d2p_t15 = (d2p2 + d2p3) / 2;
			auto d3p_t15 = d3p3;

			auto dp_ds_t15 = dp_t15 / ds_t15;
			auto d2p_ds2_t15 = (d2p_t15 - dp_ds_t15 * d2s_t15) / ds_t15 / ds_t15;
			auto d3p_ds3_t15 = (d3p_t15 - 3 * d2p_ds2_t15 * ds_t15 * d2s_t15 - dp_ds_t15 * d3s_t15) / ds_t15 / ds_t15 / ds_t15;

			auto s25_s15 = (ds2 + ds3) / 2 * dt;
			auto d3p_ds3_t25 = d3p_ds3_t15;
			auto d2p_ds2_t25 = d2p_ds2_t15 + d3p_ds3_t15 * s25_s15;
			auto dp_ds_t25 = dp_ds_t15 + d2p_ds2_t15 * s25_s15 + 0.5 * d3p_ds3_t15 * s25_s15 * s25_s15;

			double k = (dp_ds_t25 + (3 * d2p_ds2_t25 * ds_t25 * dt) / 2);
			double g = d3p_ds3_t25 * ds_t25 * ds_t25 * ds_t25 + 3 * d2p_ds2_t25 * (d2s2 / 2 + d2s3 / 2 + (d3s3 * dt) / 2) * ds_t25;

			if (std::abs(k) > zero_check) {
				auto d3s_l1 = (-dp_max[i] * VEL_BOUND_RATIO - dp3 - d2p3 * dt - g * dt * dt) / k / dt / dt;
				auto d3s_r1 = (dp_max[i] * VEL_BOUND_RATIO - dp3 - d2p3 * dt - g * dt * dt) / k / dt / dt;
				
				auto d3s_l2 = (-d2p_max[i] * ACC_BOUND_RATIO - d2p3 - g * dt) / k / dt;
				auto d3s_r2 = (d2p_max[i] * ACC_BOUND_RATIO - d2p3 - g * dt) / k / dt;

				auto d3s_l3 = (-d3p_max[i] * JERK_BOUND_RATIO - g) / k;
				auto d3s_r3 = (d3p_max[i] * JERK_BOUND_RATIO - g) / k;

				auto d2p_min2 = -std::sqrt(std::max(2 * (dp3 + dp_max[i] * VEL_BOUND_RATIO + d2p3 * dt) * d3p_max[i] * JERK_BOUND_RATIO, 0.0));
				auto d2p_max2 = std::sqrt(std::max(2 * (dp_max[i] * VEL_BOUND_RATIO - dp3 - d2p3 * dt) * d3p_max[i] * JERK_BOUND_RATIO, 0.0));
				auto d3s_l4 = (d2p_min2 - d2p3 - g * dt) / k / dt;
				auto d3s_r4 = (d2p_max2 - d2p3 - g * dt) / k / dt;

				if (k < 0) {
					std::swap(d3s_l1, d3s_r1);
					std::swap(d3s_l2, d3s_r2);
					std::swap(d3s_l3, d3s_r3);
					std::swap(d3s_l4, d3s_r4);
				}

				d3s_min_1 = std::max(d3s_l1, d3s_min_1);
				d3s_max_1 = std::min(d3s_r1, d3s_max_1);
				d3s_min_2 = std::max(d3s_l2, d3s_min_2);
				d3s_max_2 = std::min(d3s_r2, d3s_max_2);
				d3s_min_3 = std::max(d3s_l3, d3s_min_3);
				d3s_max_3 = std::min(d3s_r3, d3s_max_3);
				d3s_min_4 = std::max(d3s_l4, d3s_min_4);
				d3s_max_4 = std::min(d3s_r4, d3s_max_4);

			}
		}

		double d3s_min_12 = std::max({
			d3s_min_1,
			d3s_min_2,
			});
		double d3s_max_12 = std::min({
			d3s_max_1,
			d3s_max_2,
			});
		double d3s_min_124 = std::max({
			d3s_min_1,
			d3s_min_2,
			d3s_min_4,
			});
		double d3s_max_124 = std::min({
			d3s_max_1,
			d3s_max_2,
			d3s_max_4,
			});
		double d3s_min_all = std::max({
			d3s_min_1,
			d3s_min_2,
			d3s_min_3,
			d3s_min_4,
			});
		double d3s_max_all = std::min({
			d3s_max_1,
			d3s_max_2,
			d3s_max_3,
			d3s_max_4,
			});

		double next_ds, next_d2s, next_d3s{ 0.0 };

		// 0. ds 过小、无法通过ds 判断出原始轨迹的曲率等
		if (ds3 <= zero_check || ds2 < zero_check || ds1 < zero_check) {
			aris::Size total_count;
			s_follow_x(ds3, d2s3, target_ds, MAX_D2S, MIN_D2S, MAX_D3S, MIN_D3S, dt, zero_check, next_ds, next_d2s, next_d3s, total_count);
			goto next_step;
		}

		// 1. 原始轨迹在起始点，可以直接加速到想要的 ds
		if (d3s_max_all >= 1e9 && d3s_min_all < -1e9 && ds3 > zero_check) {
			next_ds = ds2 = ds3 = target_ds;
			goto next_step;
		}

		// 2. 可以在COND 1234不超限的情况下，完成规划
		auto d3s_max = d3s_max_all;
		auto d3s_min = d3s_min_all;
		if (d3s_max > d3s_min) {
			bool is_avalable = false;
			if (ds3 > 0.5) {
				if (d2s3 > 0.0) {
					// diff_ds = 0.5* d3s_min * T^2
					// 
					// T = d2s / d3s_min
					// 
					// => diff_ds = 0.5* d2s^2 / d3s_min
					auto d3s_desired = ds3 < MAX_DS ? -d2s3 * d2s3 / ((MAX_DS - (ds3)) * 2.0) : 0.0;
					is_avalable = d3s_desired > d3s_min;

					//auto d2s_desired = std::sqrt(std::max(-d3s_min * (MAX_DS - (ds3 + d2s3 * dt + 0.5 * d3s_max * dt*dt))*2.0, 0.0));
					//d3s_max = std::min(d3s_max, (d2s_desired - d2s3) / dt);
				}
				else {
					is_avalable = true;
				}
			}
			else {
				if (d2s3 > 0.0) {
					is_avalable = true;
				}
				else {
					auto d3s_desired = ds3 > MIN_DS ? -d2s3 * d2s3 / ((MIN_DS - ds3) * 2.0) : 0.0;
					is_avalable = d3s_desired < d3s_max;

					//auto d2s_desired = -std::sqrt(std::max(-d3s_max * (MIN_DS - (ds3 + d2s3 * dt + 0.5 * d3s_min * dt * dt)) * 2.0, 0.0));
					//d3s_min = std::max(d3s_min, (d2s_desired - d2s3) / dt);
				}
			}
			if (is_avalable) {
				aris::Size total_count;
				s_follow_x(ds3, d2s3, target_ds, MAX_D2S, MIN_D2S, d3s_max, d3s_min, dt, zero_check, next_ds, next_d2s, next_d3s, total_count);
				goto next_step;
			}
		}

		// 3. 可以在COND 124不超限的情况下，完成规划
		d3s_max = d3s_max_124;
		d3s_min = d3s_min_124;
		if (d3s_max > d3s_min) {
			bool is_avalable = false;
			if (ds3 > 0.5) {
				if (d2s3 > 0.0) {
					auto d3s_desired = ds3 < MAX_DS ? -d2s3 * d2s3 / ((MAX_DS - ds3) * 2.0) : 0.0;
					is_avalable = d3s_desired > d3s_min;
					d3s_max = std::min(d3s_desired, d3s_max);
					//d3s_min = d3s_desired;
				}
				else {
					is_avalable = true;
				}
			}
			else {
				if (d2s3 > 0.0) {
					is_avalable = true;
				}
				else {
					auto d3s_desired = ds3 > MIN_DS ? -d2s3 * d2s3 / ((MIN_DS - ds3) * 2.0) : 0.0;
					is_avalable = d3s_desired < d3s_max;
					//d3s_max = d3s_desired;
					d3s_min = std::max(d3s_desired, d3s_min);
				}
			}
			if (is_avalable) {
				aris::Size total_count;
				s_follow_x(ds3, d2s3, target_ds, MAX_D2S, MIN_D2S, d3s_max, d3s_min, dt, zero_check, next_ds, next_d2s, next_d3s, total_count);
				goto next_step;
			}
		}

		// 4. 可以在COND 12不超限的情况下，完成规划
		d3s_max = d3s_max_12;
		d3s_min = d3s_min_12;
		if (d3s_max > d3s_min) {
			bool is_avalable = false;
			if (ds3 > 0.5) {
				if (d2s3 > 0.0) {
					auto d3s_desired = ds3 < MAX_DS ? -d2s3 * d2s3 / ((MAX_DS - ds3) * 2.0) : 0.0;
					is_avalable = d3s_desired > d3s_min;
					d3s_max = std::min(d3s_desired, d3s_max);
				}
				else {
					is_avalable = true;
				}
			}
			else {
				if (d2s3 > 0.0) {
					is_avalable = true;
				}
				else {
					auto d3s_desired = ds3 > MIN_DS ? -d2s3 * d2s3 / ((MIN_DS - ds3) * 2.0) : 0.0;
					is_avalable = d3s_desired < d3s_max;
					d3s_min = std::max(d3s_desired, d3s_min);
				}
			}
			if (is_avalable) {
				aris::Size total_count;
				s_follow_x(ds3, d2s3, target_ds, 1e9, -1e9, d3s_max, d3s_min, dt, zero_check, next_ds, next_d2s, next_d3s, total_count);
				goto next_step;
			}
		}

		// 5. 只考虑减速环节
		{
			aris::Size total_count;
			s_follow_x(ds3, d2s3, MIN_DS, MAX_D2S, MIN_D2S, MAX_D3S, MIN_D3S, dt, zero_check, next_ds, next_d2s, next_d3s, total_count);
		}






	next_step:

#ifdef ARIS_DEBUG_SINGULAR_PROCESSOR
		static int count_{ 0 };
		//std::cout << "count: " << count_++ << std::endl;

		if (count_ > 380 && count_ < 400) {
			std::cout << "bound:" << d3s_max_all << "  " << d3s_min_all << std::endl;
			std::cout << "next_d3s:" << next_d3s << std::endl;
			//aris::dynamic::dsp(1, 6, d3s_l1.data());
			//aris::dynamic::dsp(1, 6, d3s_r1.data());
			//aris::dynamic::dsp(1, 6, d3s_l2.data());
			//aris::dynamic::dsp(1, 6, d3s_r2.data());
			//aris::dynamic::dsp(1, 6, d3s_l3.data());
			//aris::dynamic::dsp(1, 6, d3s_r3.data());
			//aris::dynamic::dsp(1, 6, d3s_l4.data());
			//aris::dynamic::dsp(1, 6, d3s_r4.data());

			if (count_ == 180) {
				std::cout << "debug" << std::endl;
			}
			//return 0;
		}

#endif // ARIS_DEBUG_SINGULAR_PROCESSOR

		//next_d2s = d2s3 + next_d3s * dt;
		//next_ds = ds3 + next_d2s * dt;

		next_ds = std::min(next_ds, MAX_DS);
		next_ds = std::max(next_ds, std::min(MIN_DS, target_ds));

		ret.next_ds = next_ds;
		return 0;
	};


	struct SingularProcessor::Imp {
		using CurveParam = TcurveParam;

		InverseKinematicMethod inv_func_;

		aris::Size input_size_{ 0 };

		std::vector<char> mem_;
		double* max_vels_,
			* max_accs_,
			* max_jerks_,
			* min_vels_,
			* min_accs_,
			* min_jerks_,
			* input_pos_begin_,// 奇异状态的起始值
			* input_vel_begin_,
			* input_acc_begin_,
			* input_pos_end_,  // 奇异状态的终止值
			* input_vel_end_,
			* input_acc_end_,
			* input_pos_last_, // 真实状态值
			* input_vel_last_,
			* input_pos_this_,
			* input_vel_this_,
			* input_acc_this_,
			* input_acc_ratio_,
			* input_acc_max_consider_ratio_,
			* input_acc_min_consider_ratio_,
			* output_pos_,
			* p0_,             // 理想的位置值，仅仅在 move_in_tg 中改变
			* p1_,
			* p2_,
			* p3_;

		std::int64_t singular_ret_{ 0 };
		aris::Size total_singular_count_{ 0 }, current_singular_count_{ 0 };
		CurveParam* curve_params_;



		double T;
		double check_rate_{ 0.99 };
		double max_vel_ratio_{ 0.95 };
		double max_acc_ratio_{ 0.9 };
		double target_ds_{ 1.0 };

		double ds1_{ 1.0 }, ds2_{ 1.0 }, ds3_{ 1.0 };
		
		aris::dynamic::ModelBase* model_{ nullptr };
		aris::plan::TrajectoryGenerator* tg_{ nullptr };

		// 是否已经处于奇异状态，或者是否还在继续准备处理奇异情况 //
		int singular_idx;

		enum class SingularState {
			SINGULAR,
			SINGULAR_PREPARE,
			NORMAL
		};

		SingularState state_{ SingularState::NORMAL };

	};
	auto SingularProcessor::setMaxVels(const double* max_vels)->void {
		std::copy(max_vels, max_vels + imp_->input_size_, imp_->max_vels_);
		std::copy(max_vels, max_vels + imp_->input_size_, imp_->min_vels_);

		for (int i = 0; i < imp_->input_size_; ++i)
			imp_->min_vels_[i] = -imp_->min_vels_[i];
	}
	auto SingularProcessor::setMaxAccs(const double* max_accs)->void {
		std::copy(max_accs, max_accs + imp_->input_size_, imp_->max_accs_);
		std::copy(max_accs, max_accs + imp_->input_size_, imp_->min_accs_);

		for (int i = 0; i < imp_->input_size_; ++i)
			imp_->min_accs_[i] = -imp_->min_accs_[i];
	}
	auto SingularProcessor::setMaxJerks(const double* max_jerks) -> void {
		std::copy(max_jerks, max_jerks + imp_->input_size_, imp_->max_jerks_);
		std::copy(max_jerks, max_jerks + imp_->input_size_, imp_->min_jerks_);

		for (int i = 0; i < imp_->input_size_; ++i)
			imp_->min_jerks_[i] = -imp_->min_jerks_[i];
	}
	auto SingularProcessor::setMaxVelRatio(double vel_ratio)->void {
		imp_->max_vel_ratio_ = vel_ratio;
	}
	auto SingularProcessor::setMaxAccRatio(double acc_ratio)->void {
		imp_->max_acc_ratio_ = acc_ratio;
	}
	auto SingularProcessor::setModel(aris::dynamic::ModelBase& model)->void {
		imp_->model_ = &model;
		imp_->input_size_ = model.inputPosSize();

		Size mem_size = 0;
		core::allocMem(mem_size, imp_->max_vels_, imp_->input_size_);
		core::allocMem(mem_size, imp_->max_accs_, imp_->input_size_);
		core::allocMem(mem_size, imp_->max_jerks_, imp_->input_size_);
		core::allocMem(mem_size, imp_->min_vels_, imp_->input_size_);
		core::allocMem(mem_size, imp_->min_accs_, imp_->input_size_);
		core::allocMem(mem_size, imp_->min_jerks_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_pos_begin_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_vel_begin_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_pos_end_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_vel_end_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_pos_last_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_vel_last_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_pos_this_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_vel_this_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_acc_this_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_acc_ratio_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_acc_max_consider_ratio_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_acc_min_consider_ratio_, imp_->input_size_);
		core::allocMem(mem_size, imp_->output_pos_, imp_->input_size_);
		core::allocMem(mem_size, imp_->curve_params_, imp_->input_size_);
		core::allocMem(mem_size, imp_->p0_, imp_->input_size_);
		core::allocMem(mem_size, imp_->p1_, imp_->input_size_);
		core::allocMem(mem_size, imp_->p2_, imp_->input_size_);
		core::allocMem(mem_size, imp_->p3_, imp_->input_size_);

		imp_->mem_.resize(mem_size, char(0));

		imp_->max_vels_ = core::getMem(imp_->mem_.data(), imp_->max_vels_);
		imp_->max_accs_ = core::getMem(imp_->mem_.data(), imp_->max_accs_);
		imp_->max_jerks_ = core::getMem(imp_->mem_.data(), imp_->max_jerks_);
		imp_->min_vels_ = core::getMem(imp_->mem_.data(), imp_->min_vels_);
		imp_->min_accs_ = core::getMem(imp_->mem_.data(), imp_->min_accs_);
		imp_->min_jerks_ = core::getMem(imp_->mem_.data(), imp_->min_jerks_);
		imp_->input_pos_begin_ = core::getMem(imp_->mem_.data(), imp_->input_pos_begin_);
		imp_->input_vel_begin_ = core::getMem(imp_->mem_.data(), imp_->input_vel_begin_);
		imp_->input_acc_begin_ = core::getMem(imp_->mem_.data(), imp_->input_acc_begin_);
		imp_->input_pos_end_ = core::getMem(imp_->mem_.data(), imp_->input_pos_end_);
		imp_->input_vel_end_ = core::getMem(imp_->mem_.data(), imp_->input_vel_end_);
		imp_->input_acc_end_ = core::getMem(imp_->mem_.data(), imp_->input_acc_end_);
		imp_->input_pos_last_ = core::getMem(imp_->mem_.data(), imp_->input_pos_last_);
		imp_->input_vel_last_ = core::getMem(imp_->mem_.data(), imp_->input_vel_last_);
		imp_->input_pos_this_ = core::getMem(imp_->mem_.data(), imp_->input_pos_this_);
		imp_->input_vel_this_ = core::getMem(imp_->mem_.data(), imp_->input_vel_this_);
		imp_->input_acc_this_ = core::getMem(imp_->mem_.data(), imp_->input_acc_this_);
		imp_->input_acc_ratio_ = core::getMem(imp_->mem_.data(), imp_->input_acc_ratio_);
		imp_->input_acc_max_consider_ratio_ = core::getMem(imp_->mem_.data(), imp_->input_acc_max_consider_ratio_);
		imp_->input_acc_min_consider_ratio_ = core::getMem(imp_->mem_.data(), imp_->input_acc_min_consider_ratio_);
		imp_->output_pos_ = core::getMem(imp_->mem_.data(), imp_->output_pos_);
		imp_->curve_params_ = core::getMem(imp_->mem_.data(), imp_->curve_params_);
		imp_->p0_ = core::getMem(imp_->mem_.data(), imp_->p0_);
		imp_->p1_ = core::getMem(imp_->mem_.data(), imp_->p1_);
		imp_->p2_ = core::getMem(imp_->mem_.data(), imp_->p2_);
		imp_->p3_ = core::getMem(imp_->mem_.data(), imp_->p3_);
	}
	auto SingularProcessor::setTrajectoryGenerator(TrajectoryGenerator& tg)->void {
		imp_->tg_ = &tg;
	}
	auto SingularProcessor::init()->void {
		imp_->model_->getInputPos(imp_->input_pos_this_);
		std::fill_n(imp_->input_vel_this_, imp_->input_size_, 0.0);
		std::fill_n(imp_->input_acc_this_, imp_->input_size_, 0.0);
		std::fill_n(imp_->input_vel_last_, imp_->input_size_, 0.0);

		imp_->model_->getInputPos(imp_->p0_);
		aris::dynamic::s_vc(imp_->input_size_, imp_->p0_, imp_->p1_);
		aris::dynamic::s_vc(imp_->input_size_, imp_->p0_, imp_->p2_);
		aris::dynamic::s_vc(imp_->input_size_, imp_->p0_, imp_->p3_);

		imp_->ds1_ = imp_->ds2_ = imp_->ds3_ = 1.0;

		imp_->state_ = Imp::SingularState::NORMAL;
	}
	auto SingularProcessor::setDs(double ds)->void {
		imp_->ds3_ = imp_->ds2_ = imp_->ds1_ = ds;
	}
	auto SingularProcessor::currentDs() -> double {
		return imp_->ds3_;
	}
	auto SingularProcessor::setTargetDs(double ds)->void {
		imp_->target_ds_ = ds;
	}
	auto SingularProcessor::setModelPosAndMoveDt()->std::int64_t {

#ifdef ARIS_DEBUG_SINGULAR_PROCESSOR
		static int count{ 0 };
		count++;
		if (count % 1000 == 0)
			std::cout << "count: " << count++ << std::endl;
		if (count == 125)
			std::cout << "debug" << std::endl;
#endif

		// 获得最大的速度比或加速度比
		auto get_max_ratio = [](aris::Size input_size, const double* max_value, const double* value)->double {
			double max_ratio = 0.0;
			for (auto idx = 0; idx < input_size; ++idx)
				max_ratio = std::max(max_ratio, std::abs(value[idx]) / max_value[idx]);
			return max_ratio;
		};

		// move tg step //
		// max_vel_ratio 和 max_acc_ratio 会触发正常降速
		auto move_tg_step = [this]()->std::int64_t {
			// 当前处于非奇异状态，正常求反解 //
			auto ret = imp_->tg_->getEePosAndMoveDt(imp_->output_pos_);
			if (imp_->inv_func_) {
				if (auto ret = imp_->inv_func_(*imp_->model_, imp_->output_pos_); ret)
					return ret;
			}
			else {
				imp_->model_->setOutputPos(imp_->output_pos_);
				if (auto ret = imp_->model_->inverseKinematics(); ret)
					return ret;
			}
			
			// update ds //
			imp_->ds1_ = imp_->ds2_;
			imp_->ds2_ = imp_->ds3_;
			imp_->ds3_ = imp_->tg_->currentDs();

			// update input pos //
			std::swap(imp_->p0_, imp_->p1_);
			std::swap(imp_->p1_, imp_->p2_);
			std::swap(imp_->p2_, imp_->p3_);
			imp_->model_->getInputPos(imp_->p3_);

			SmoothParam param{
				imp_->tg_->dt(),
				imp_->input_size_,
				imp_->min_vels_, imp_->max_vels_, imp_->min_accs_, imp_->max_accs_, imp_->min_jerks_, imp_->max_jerks_,
				imp_->ds1_, imp_->ds2_, imp_->ds3_,
				imp_->p0_, imp_->p1_, imp_->p2_, imp_->p3_,
				imp_->target_ds_
			};
			SmoothRet smooth_ret;
			s_smooth_curve(param, smooth_ret);

			imp_->tg_->setCurrentDs(smooth_ret.next_ds);
			imp_->tg_->setCurrentDds(0.0);
			imp_->tg_->setTargetDs(smooth_ret.next_ds);

			return ret;
		};

		// move in singular state
		// 不考虑 max_vel_ratio 与 max_acc_ratio，奇异降速时，只考虑真实电机限制
		auto move_in_singular = [this]()->std::int64_t {
			auto dt = imp_->tg_->dt();
			
			// 取出位置
			std::swap(imp_->input_pos_this_, imp_->input_pos_last_);
			std::swap(imp_->input_vel_this_, imp_->input_vel_last_);
			for (int i = 0; i < imp_->input_size_; ++i) {
				imp_->input_pos_this_[i] = s_tcurve_value(imp_->curve_params_[i], imp_->current_singular_count_ * dt);
			}

			imp_->model_->setInputPos(imp_->input_pos_this_);
			imp_->model_->forwardKinematics();

			// 计算速度
			aris::dynamic::s_vc(imp_->input_size_, imp_->input_pos_this_, imp_->input_vel_this_);
			aris::dynamic::s_vs(imp_->input_size_, imp_->input_pos_last_, imp_->input_vel_this_);
			aris::dynamic::s_nv(imp_->input_size_, 1.0 / dt, imp_->input_vel_this_);

			// 计算加速度
			aris::dynamic::s_vc(imp_->input_size_, imp_->input_vel_this_, imp_->input_acc_this_);
			aris::dynamic::s_vs(imp_->input_size_, imp_->input_vel_last_, imp_->input_acc_this_);
			aris::dynamic::s_nv(imp_->input_size_, 1.0 / dt, imp_->input_acc_this_);

			imp_->current_singular_count_++;
			if (imp_->current_singular_count_ > imp_->total_singular_count_)
				imp_->state_ = Imp::SingularState::NORMAL;

			return imp_->singular_ret_;
		};

		// check if singular //
		auto check_if_singular = [](aris::Size input_size, double dt, const double* max_vel, const double* max_acc, const double* p1, const double* p2, const double *p3)->int {
			// here is condition //
			int idx = 0;
			for (idx = 0; idx < input_size; ++idx) {
				double v2 = (p3[idx] - p2[idx]) / dt;
				double v1 = (p2[idx] - p1[idx]) / dt;
				double a = (v2 - v1) / dt;

				if (v2 > max_vel[idx] || v2 < -max_vel[idx] || a > max_acc[idx] || a < -max_acc[idx]) {
#ifdef ARIS_DEBUG_SINGULAR_PROCESSOR
					std::cout << "singular idx" << idx << " vel:" << vel[idx] << "  max_vel:" << max_vel[idx] << "  acc:" << acc[idx] << "  max_acc:" << max_acc[idx] << std::endl;
#endif
					return idx;
				}
			}
			return input_size;
		};

		auto prepare_singular = [&]()->std::int64_t {
#ifdef ARIS_DEBUG_SINGULAR_PROCESSOR
			std::cout << "singular parpare" << std::endl;
#endif
			imp_->state_ = Imp::SingularState::SINGULAR_PREPARE;

			// 将当前值置为起始值
			aris::dynamic::s_vc(imp_->input_size_, imp_->input_pos_this_, imp_->input_pos_begin_);
			aris::dynamic::s_vc(imp_->input_size_, imp_->input_vel_this_, imp_->input_vel_begin_);


			// 尝试迭代到下一个非奇异的时刻，最多迭代 10 次 //
			const int MAX_ITER_COUNT = 2;

			int idx;
			int while_count{ 0 };
			auto dt = imp_->tg_->dt();
			do {
				while_count++;
				imp_->singular_ret_ = move_tg_step();
				idx = check_if_singular(imp_->input_size_, dt, imp_->max_vels_, imp_->max_accs_, imp_->p1_, imp_->p2_, imp_->p3_);

				// 尝试处理奇异情况 //
				if (idx == imp_->input_size_ || while_count >= MAX_ITER_COUNT) {
					// 保存终止时刻的位置与速度 //
					aris::dynamic::s_vc(imp_->input_size_, imp_->p3_, imp_->input_pos_end_);

					aris::dynamic::s_vc(imp_->input_size_, imp_->p3_, imp_->input_vel_end_);
					aris::dynamic::s_vs(imp_->input_size_, imp_->p2_, imp_->input_vel_end_);
					aris::dynamic::s_nv(imp_->input_size_, 1.0 / imp_->tg_->dt(), imp_->input_vel_end_);

					// 考虑结束条件可能是循环次数到了，因此将终止速度保护
					for (int i = 0; i < imp_->input_size_; ++i) {
						imp_->input_vel_end_[i] = std::max(-imp_->max_vels_[i], imp_->input_vel_end_[i]);
						imp_->input_vel_end_[i] = std::min(imp_->max_vels_[i], imp_->input_vel_end_[i]);
					}

					// 计算每根轴所需要的时间 //
					std::int64_t Ts_count[100];
					for (int i = 0; i < imp_->input_size_; ++i) {
						auto& tcurve_param = imp_->curve_params_[i];

						tcurve_param.pb = imp_->input_pos_begin_[i];
						tcurve_param.pe = imp_->input_pos_end_[i];
						tcurve_param.vb = imp_->input_vel_begin_[i];
						tcurve_param.ve = imp_->input_vel_end_[i];
						tcurve_param.vmax = imp_->max_vels_[i];
						tcurve_param.amax = imp_->max_accs_[i];

						double T1, T2, T3;
						s_tcurve_T_range(tcurve_param, T1, T2, T3);
						if (T1 == T3) {
							Ts_count[i * 3] = (std::int64_t)std::ceil(T1 / dt);
							Ts_count[i * 3 + 1] = (std::int64_t)std::ceil(T2 / dt);
							Ts_count[i * 3 + 2] = (std::int64_t)std::ceil(T3 / dt);
						}
						else {
							Ts_count[i * 3] = (std::int64_t)std::ceil(T1 / dt);
							Ts_count[i * 3 + 1] = (std::int64_t)std::floor(T2 / dt);
							Ts_count[i * 3 + 2] = (std::int64_t)std::ceil(T3 / dt);
						}
					}

					// 寻找最小可行时间
					auto Tmin_count = *std::max_element(Ts_count, Ts_count + imp_->input_size_ * 3);
					for (int i = 0; i < imp_->input_size_; ++i) {
						if (Ts_count[i * 3 + 2] < Tmin_count) {
							auto candidate_count = Ts_count[i * 3 + 2];
							bool candidate_failed = false;
							for (int j = 0; j < imp_->input_size_; ++j) {
								if ((candidate_count < Ts_count[j * 3]
									|| ((candidate_count > Ts_count[j * 3 + 1]) && candidate_count < Ts_count[j * 3 + 2]))) {
									candidate_failed = true;
									break;
								}
							}
							if (!candidate_failed)
								Tmin_count = candidate_count;
						}

						if (Ts_count[i * 3 + 1] < Tmin_count && Ts_count[i * 3] <= Ts_count[i * 3 + 1]) {
							auto candidate_count = Ts_count[i * 3 + 1];
							bool candidate_failed = false;
							for (int j = 0; j < imp_->input_size_; ++j) {
								if ((candidate_count < Ts_count[j * 3]
									|| ((candidate_count > Ts_count[j * 3 + 1]) && candidate_count < Ts_count[j * 3 + 2]))) {
									candidate_failed = true;
									break;
								}
							}
							if (!candidate_failed)
								Tmin_count = candidate_count;
						}

						if (Ts_count[i * 3] < Tmin_count && Ts_count[i * 3] <= Ts_count[i * 3 + 1]) {
							auto candidate_count = Ts_count[i * 3];
							bool candidate_failed = false;
							for (int j = 0; j < imp_->input_size_; ++j) {
								if ((candidate_count < Ts_count[j * 3]
									|| ((candidate_count > Ts_count[j * 3 + 1]) && candidate_count < Ts_count[j * 3 + 2]))) {
									candidate_failed = true;
									break;
								}
							}
							if (!candidate_failed)
								Tmin_count = candidate_count;
						}
					}

					// 根据 Tmin 生成曲线
					for (int i = 0; i < imp_->input_size_; ++i) {
						imp_->curve_params_[i].T = Tmin_count * dt;
						s_tcurve_param(imp_->curve_params_[i]);
					}

					imp_->total_singular_count_ = Tmin_count;
					imp_->current_singular_count_ = 1;

					// 判断是否满足完全修复条件
					auto& singular_param = imp_->curve_params_[imp_->singular_idx];
					if (idx == imp_->input_size_ && 
						((singular_param.vb * singular_param.ve < 0.0) || (singular_param.v * singular_param.vb >= 0)))
					{
						imp_->state_ = Imp::SingularState::SINGULAR;
					}
				}
			} while (imp_->state_ != Imp::SingularState::SINGULAR && while_count < MAX_ITER_COUNT);

			// 移动一步
			return move_in_singular();
		};

		if (imp_->state_ == Imp::SingularState::SINGULAR) {
			// 当前已经处于奇异状态 //
			return move_in_singular();
		}
		else if (imp_->state_ == Imp::SingularState::SINGULAR_PREPARE) {
#ifdef ARIS_DEBUG_SINGULAR_PROCESSOR
			std::cout << "singular prepare" << std::endl;
#endif
			return prepare_singular();
		}
		else {
			imp_->singular_ret_ = move_tg_step();

			if ((imp_->singular_idx = check_if_singular(imp_->input_size_, imp_->tg_->dt(), imp_->max_vels_, imp_->max_accs_, imp_->p1_, imp_->p2_, imp_->p3_)) == imp_->input_size_) {
				// 正常保存位置 //
				std::swap(imp_->input_pos_this_, imp_->input_pos_last_);
				aris::dynamic::s_vc(imp_->input_size_, imp_->p3_, imp_->input_pos_this_);

				// 保存速度 //
				std::swap(imp_->input_vel_this_, imp_->input_vel_last_);
				aris::dynamic::s_vc(imp_->input_size_, imp_->input_pos_this_, imp_->input_vel_this_);
				aris::dynamic::s_vs(imp_->input_size_, imp_->input_pos_last_, imp_->input_vel_this_);
				aris::dynamic::s_nv(imp_->input_size_, 1.0 / imp_->tg_->dt(), imp_->input_vel_this_);

				// 保存加速度 //
				aris::dynamic::s_vc(imp_->input_size_, imp_->input_vel_this_, imp_->input_acc_this_);
				aris::dynamic::s_vs(imp_->input_size_, imp_->input_vel_last_, imp_->input_acc_this_);
				aris::dynamic::s_nv(imp_->input_size_, 1.0 / imp_->tg_->dt(), imp_->input_acc_this_);
				
				return imp_->singular_ret_;
			}
			else {
#ifdef ARIS_DEBUG_SINGULAR_PROCESSOR
				std::cout << "singular" << std::endl;
#endif
				return prepare_singular();
			}
		}

		return 0;
	}
	auto SingularProcessor::setInverseKinematicMethod(InverseKinematicMethod func)->void {
		imp_->inv_func_ = func;
	}
	SingularProcessor::~SingularProcessor() = default;
	SingularProcessor::SingularProcessor() :imp_(new Imp) {

	}






}
