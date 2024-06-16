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

	struct OptimizationParam {
		Size observe_size, x_size, r_size, param_size;
		CostFunc cost_func;
		const double* x, *init_param;
		double* param;
	};

	auto ARIS_API s_optimize(OptimizationParam &param)->double;

}

#endif
