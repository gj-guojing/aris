#ifndef ARIS_DYNAMIC_MATH_OPTIMIZATION_H_
#define ARIS_DYNAMIC_MATH_OPTIMIZATION_H_

#include <vector>
#include <numeric>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <iterator>
#include <functional>


#include <aris_lib_export.h>
#include <aris/core/basic_type.hpp>

namespace aris::dynamic {


	using CostFunc = std::function<int(const double *param, const double *x, double *residual)>;

	//
	// 
	// observe_size : 观测次数
	//       x_size : 单次观测的数据维度
	//       r_size : 单次观测的残差维度
	//   param_size : 需要优化的参数维度
	//    cost_func : 计算残差的函数
	//            x : 测试数据，大小为 x_size * observe_size
	//   init_param : 需要优化的参数初值，大小为 param_size
	//        param : 优化结果，大小为 param_size
	struct OptimizationParam {
		Size observe_size, x_size, r_size, param_size;
		CostFunc cost_func;
		const double* x, *init_param;
		double* param;
	};

	auto ARIS_API s_optimize(OptimizationParam &param)->double;

}

#endif
