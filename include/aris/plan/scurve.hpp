﻿#ifndef ARIS_PLAN_SCURVE_H_
#define ARIS_PLAN_SCURVE_H_

#include <list>
#include <cmath>
#include <iostream>
#include <functional>
#include <map>
#include <any>

#include <aris/dynamic/dynamic.hpp>

/// \brief 轨迹规划命名空间
/// \ingroup aris
/// 
///
///
namespace aris::plan{
	struct LargeNum {
		std::int64_t count_;
		double value_;

		LargeNum() {
			count_ = 0;
			value_ = 0.0;
		};
		LargeNum(std::int64_t count, double value) {
			count_ = count;
			value_ = value;
		};
		LargeNum(double value) {
			double r = std::fmod(value, 1000.0);
			count_ = std::lround((value - r) / 1000.0);
			value_ = r;
		};

		operator double()const { return count_ * 1000.0 + value_; }

		friend auto operator+(LargeNum left, LargeNum right)->LargeNum {
			double v = left.value_ + right.value_;
			double r = std::fmod(v, 1000.0);
			return LargeNum{
				left.count_ + right.count_ + std::lround((v - r) / 1000.0),
				r
			};
		}
		friend auto operator+(LargeNum left, double right)->LargeNum {
			double v = left.value_ + right;
			double r = std::fmod(v, 1000.0);
			return LargeNum{
				left.count_ + std::lround((v - r) / 1000.0),
				r
			};
		}
		friend auto operator+(double left, LargeNum right)->LargeNum {
			double v = left + right.value_;
			double r = std::fmod(v, 1000.0);
			return LargeNum{
				right.count_ + std::lround((v - r) / 1000.0),
				r
			};
		}
		friend auto operator-(LargeNum left, LargeNum right)->LargeNum {
			double v = left.value_ - right.value_;
			double r = std::fmod(v, 1000.0);
			return LargeNum{
				left.count_ - right.count_ + std::lround((v - r) / 1000.0),
				r
			};
		}
		friend auto operator-(LargeNum left, double right)->LargeNum {
			double v = left.value_ - right;
			double r = std::fmod(v, 1000.0);
			return LargeNum{
				left.count_ + std::lround((v - r) / 1000.0),
				r
			};
		}
		friend auto operator-(double left, LargeNum right)->LargeNum {
			double v = left - right.value_;
			double r = std::fmod(v, 1000.0);
			return LargeNum{
				-right.count_ + std::lround((v - r) / 1000.0),
				r
			};
		}
	};

	struct ARIS_API SCurveParam {
		LargeNum pb_;     // 结束位置
		double vc_max_;   // 允许的最大速度
		double vb_max_;   // 允许的最大末端速度
		double a_;        // 过程中最大加速度
		double j_;        // 过程中最大加加速度

		LargeNum pa_;     // 起始位置
		double va_;       // 起始速度
		double T_;        // 总时长，T = Ta + Tb + Tc，Tc是匀速段时长

		double vb_;       // 结束速度
		double vc_;       // 匀速段速度
		double Ta_;       // 起始段加速时长
		double Tb_;       // 结束段加速时长
		int    mode_;     // A or B 模式

		LargeNum t0_;     // 起始时间
	};

	struct ARIS_API SCurveNode {
		std::vector<SCurveParam> params_;
	};

	// 计算指定时间处的 p v a j
	auto ARIS_API s_compute_scurve(std::list<SCurveNode>::iterator begin_iter, std::list<SCurveNode>::iterator end_iter)noexcept->void;

	// 计算指定时间处的 p v a j
	auto ARIS_API s_scurve_at(const SCurveParam& param, LargeNum t, LargeNum *p_out, double* v_out = nullptr, double* a_out = nullptr, double* j_out = nullptr)noexcept->void;


}

#endif