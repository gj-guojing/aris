﻿#include"aris/plan/scurve.hpp"

//#define DEBUG_ARIS_PLAN_TRAJECTORY


namespace aris::plan {
    // 给定时间，计算加加速段的终止速度
    auto s_acc_vend(double va, double a, double j, double T)noexcept->double {
        return a / j > T / 2.0 ? va + j * T * T / 4.0 : va + T * a - a * a / j;
    };
    // 给定速度，计算加加速段的终止速度
    auto s_acc_time(double va, double vb, double a, double j)noexcept->double {
        double v_diff = std::abs(vb - va);
        return v_diff > a * a / j ? v_diff / a + a / j : 2 * std::sqrt(v_diff / j);
    };

    auto newton_raphson_binary_search(std::function<double(double)> f, double x_below, double x_upper, double tol = std::numeric_limits<double>::epsilon() * 10)->double {
        double f_upper = f(x_upper);
        double f_below = f(x_below);

        double fsig = aris::dynamic::s_sgn2(f_upper - f_below);
        double xsig = aris::dynamic::s_sgn2(x_upper - x_below);

        if (aris::dynamic::s_sgn2(f_upper * f_below) >= 0)
            return std::abs(f_upper) < std::abs(f_below) ? x_upper : x_below;

        double diff = std::abs(x_upper - x_below);
        double diff_last = 10*diff;
        
        while (diff < diff_last) {
            diff_last = diff;

            double x_mid = x_below + (x_upper - x_below) / 2;
            double f_mid = f(x_mid);

            if (aris::dynamic::s_sgn2(f_mid) == fsig) {
                x_upper = x_mid;
                f_upper = f_mid;
            }
            else {
                x_below = x_mid;
                f_below = f_mid;
            }

            double x1 = (x_mid * f_below - x_below * f_mid) / (f_below - f_mid);
            if (xsig * x1 <= xsig * x_upper && xsig * x1 >= xsig * x_below) {
                double fx1 = f(x1);
                if (aris::dynamic::s_sgn2(fx1) == fsig) {
                    x_upper = x1;
                    f_upper = fx1;
                }
                else{
                    x_below = x1;
                    f_below = fx1;
                }
            }

            double x2 = (x_mid * f_upper - x_upper * f_mid) / (f_upper - f_mid);
            if (xsig * x2 <= xsig * x_upper && xsig * x2 >= xsig * x_below) {
                double fx2 = f(x2);
                if (aris::dynamic::s_sgn2(fx2) == fsig) {
                    x_upper = x2;
                    f_upper = fx2;
                }
                else{
                    x_below = x2;
                    f_below = fx2;
                }
            
            }
            
            diff = std::abs(x_upper - x_below);
        }
        return (x_below + x_upper) / 2;
    }

    // 根据起始条件，终止位置，总时间来计算 S 曲线
    //
    // param 中的以下信息为输入：
    // - pb
    // - vc_max_
    // - vb_max_
    // - a
    // - j
    // - pa
    // - va
    // - T
    // 
    // 以下信息会改变：
    // - vb_
    // - vc_
    // - Ta_
    // - Tb_
    // - mode_
	auto s_compute_scurve_node(SCurveParam& param)->void {
        const double va     = std::max(param.va_, 0.0);
        const double vb_max = param.vb_max_;
        const double vc_max = param.vc_max_;
        const double a      = param.a_;
        const double j      = param.j_;
        const double pt     = param.pb_ - param.pa_;
        const double T      = param.T_;

        const double Z1 = a*a/j;
        const double Z2 = T*T*j;

        const double l_cons = 2000000.0 * std::max(pt, 1.0) * std::numeric_limits<double>::epsilon();
        const double t_cons = 2000000.0 * std::max(T, 1.0) * std::numeric_limits<double>::epsilon();

        double T_va_to_max_v = s_acc_time(va, vc_max, a, j);
        double T_0_to_va = s_acc_time(va, 0, a, j);

        if (pt < std::numeric_limits<double>::epsilon() * 100) {
            param.vb_ = 0.0;
            param.vc_ = 0.0;
            param.mode_ = 1;
            param.Ta_ = param.T_ / 2.0;
            param.Tb_ = param.T_ / 2.0;
            return;
        }



        double& Ta = param.Ta_;
        double& Tb = param.Tb_;
        double& vb = param.vb_;
        double& vc = param.vc_;
        int& mode = param.mode_;
        double Tc, l, la, lb, lc, v_upper, v_below;

#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
        //% ------------------ l1 --------------------- %
        //% 全力减速
        Ta = std::min(T, T_0_to_va);
        vb = std::max(s_acc_vend(va,-a,-j,Ta), 0.0);
        l  = Ta * (va + vb) / 2.0;

        double l1 = l;

        if (pt < l - 1e-10) {
            THROW_FILE_LINE("failed to compute trajectory : l1");
        }
#endif // DEBUG_ARIS_PLAN_TRAJECTORY

        //% ------------------ l2 --------------------- %
        //% vb为0，无需加速即可完成
        //% 此时 0 < v < va, 因此 Ta 段匀速va，Tb段匀速0，Tc段减速到0 
        l = -1;
        if (T >= T_0_to_va) {
            Tc = T_0_to_va;
            Ta = T - Tc;
            l = Ta * va + Tc * va / 2;
        }
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
        double l2 = l;
#endif
        if (pt < l) {
            Tc = T_0_to_va;
            param.Ta_ = (pt - Tc * va / 2) / va;
            param.Tb_ = T - Ta - Tc;
            param.vc_ = (T - Tc) > t_cons ? Ta / (T - Tc) * va : 0.0;
            param.vb_ = 0;
            param.mode_ = 1;

#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
            if (Ta < -t_cons || Tb < -t_cons || T - Ta - Tb < -t_cons
                || std::abs(Ta * va + (T - Ta - Tb) * va /2 - pt) > l_cons
                ) {
                THROW_FILE_LINE("ERROR l2");
            }
#endif
            return;
        }

        //% ------------------ l3 --------------------- %
        //% vb为0，a段达不到最大加速度，b段达不到最大加速度
        //%
        //% 此时需要满足3个条件：
        //% A. Ta + Tb <= T
        //% B. v - va  <= a^2/j && v <= a^2/j && v <= max_v && v >= va
        //% C. pt <= l
        //%
        //% 首先根据条件 A 和 B 确定 v 的取值范围，然后再计算 l
        //% Ta = 2 * sqrt((v-va)/j)
        //% Tb = 2 * sqrt(v/j)
        //% 带入条件 A，以下为推导对 v 的表达式：
        //% eq = (Ta+Tb)^2
        //% expand(eq)
        //% >> 8*(v/j - va/j)^(1/2)*(v/j)^(1/2) + (8*v)/j - (4*va)/j
        //% left  = 8*(v/j - va/j)^(1/2)*(v/j)^(1/2)
        //% right = T^2 + (4*va)/j - (8*v)/j
        //%
        //% 原不等式等效于 left^2 - right^2 <= 0 && right >= 0
        //%
        //% expand(left^2 - right^2)
        //% collect(left^2 - right^2,v)
        //% >> ((16*(T^2 + (4*va)/j))/j - (64*va)/j^2)*v - (T^2 + (4*va)/j)^2
        //% solve(left^2 - right^2,v)
        //% >> (T^4*j^2 + 8*T^2*j*va + 16*va^2)/(16*T^2*j)
        //% 对于不等式 left^2 - right^2 <= 0，应有：
        //% v <= (T^4*j^2 + 8*T^2*j*va + 16*va^2)/(16*T^2*j)
        //% && v <= (j*T^2)/8 + va/2
        //% 考虑其他条件应有：
        //% v_upper = min(max_v, a^2/j 
        //%               ,(T^4*j^2 + 8*T^2*j*va + 16*va^2)/(16*T^2*j)
        //%               ,(j*T^2)/8 + va/2)
        //% v_below = va
        v_upper = std::min(std::min(std::min(vc_max, Z1), Z2/16.0 + va/2.0 + va*va/Z2), Z2/8.0 + va/2.0);
        v_below = va;
        l = -1;
        if (v_below <= v_upper) {
            vc = v_upper;
            Ta = s_acc_time(va, vc, a, j);
            Tb = s_acc_time(0, vc, a, j);
            Tc = T - Ta - Tb;
            l = (va + vc) / 2 * Ta + vc / 2 * Tb + vc * Tc;
        }
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
        double l3 = l;
#endif
        if (pt < l) {
            //% syms va j Ta pt T f(Ta)
            //% v  = va + j*Ta*Ta/4
            //% la = (v+va)/2*Ta
            //% Tb = sqrt(4*va/j + Ta*Ta)
            //% lb = Tb*v/2
            //% lc = (T - Ta - Tb)*v
            //%
            //% l = la+lb+lc
            //%   = la + T*v - Ta*v -Tb*v/2
            param.Ta_ = newton_raphson_binary_search([va, j, pt, T](double Ta)->double {
                return T * va - (Ta * Ta * Ta * j) / 8
                    - (va / 2 + Ta * Ta * j / 8) * std::sqrt(Ta * Ta + 4 * va / j)
                    + (T * Ta*Ta * j) / 4 - pt;
                }
            , 0, std::min(std::min(s_acc_time(va, vc, a, j), T / 2.0), T - T_0_to_va));
            param.Tb_ = std::sqrt(4.0 * va / j + Ta * Ta); 
            param.vc_  = va + j*Ta*Ta/4.0;
            param.vb_ = 0;
            param.mode_ = 0;
            
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
            //% error %
            if (Ta < -t_cons || Tb < -t_cons || T - Ta - Tb < -t_cons
                || std::abs(Ta * (va + vc) / 2 + Tb * vc / 2 + (T - Ta - Tb) * vc - pt) > l_cons
                ) {

                v_upper = std::min(std::min(std::min(vc_max, Z1), Z2 / 16.0 + va / 2.0 + va * va / Z2), Z2 / 8.0 + va / 2.0);
                v_below = va;
                l = -1;
                if (v_below <= v_upper) {
                    vc = v_upper;
                    Ta = s_acc_time(va, vc, a, j);
                    Tb = s_acc_time(0, vc, a, j);
                    Tc = T - Ta - Tb;
                    l = (va + vc) / 2 * Ta + vc / 2 * Tb + vc * Tc;
                }

                param.Ta_ = newton_raphson_binary_search([va, j, pt, T](double Ta)->double {
                    return T * va - (Ta * Ta * Ta * j) / 8
                        - (va / 2 + Ta * Ta * j / 8) * std::sqrt(Ta * Ta + 4 * va / j)
                        + (T * Ta * Ta * j) / 4 - pt;
                    //return T * va - (Ta * Ta * Ta * j) / 8 - (va * std::sqrt(Ta * Ta + (4 * va) / j)) / 2
                    //    - (Ta * Ta * j * std::sqrt(Ta * Ta + (4 * va) / j)) / 8 + (T * Ta * Ta * j) / 4 - pt;
                    }
                , 0, std::min(std::min(s_acc_time(va, vc, a, j), T / 2.0), T - T_0_to_va));

                THROW_FILE_LINE("ERROR l3");
            }
#endif
            return;
        }

        //% ------------------ l4 --------------------- %
        //% vb为0，a段达不到最大加速度，b段可达到最大加速度
        //%
        //% 此时需要满足3个条件：
        //% A. Ta + Tb <= T
        //% B. v - va  <= a^2/j && v >= a^2/j && v <= max_v && v >= va
        //% C. pt <= l
        //%
        //% 首先根据条件 A 和 B 确定 v 的取值范围，然后再计算 l
        //% Ta = 2 * sqrt((v-va)/j)
        //% Tb = v/a + a/j
        //% 带入条件 A, 有：
        //% Ta^2 - (T-Tb)^2 <= 0 && Tb <= T
        //% 展开有：
        //% -v^2 + k1*v + k0 <=0 && v  <= T*a-a^2/j 
        //% 其中：
        //% k1 = (2*T*a + 2*a^2/j)
        //% k0 = -a^2*((T - a/j)^2 + (4*va)/j)
        //%
        //% 可以看出，其极值点为 a^2/j + T*a，然而若v 大于此值，必有 Tb > T
        //% 故而取其较小的根 x1
        //% 因此有 v <= x1
        //% 进一步的，根据条件 B，可以得到v的取值范围：
        //%
        //% v_upper = min(x1, va + a^2/j, max_v, T*a-a^2/j)
        //% v_below = a^2/j
        {
            double k1 = 2 * (T * a + Z1);
            double k0 = -a * a * ((4 * va) / j + T * T + Z1 / j - (2 * T * a) / j);
            double x1 = (k1 - std::sqrt(k1 * k1 + 4 * k0)) / 2; // k0为 - 1

            v_upper = std::min(std::min(std::min(x1, va + Z1), vc_max), T * a - Z1);
            v_below = std::max(Z1, va);
        }
        l = -1;
        if (v_upper >= v_below) {
            vc = v_upper;
            Ta = s_acc_time(va, vc, a, j);
            Tb = s_acc_time(0, vc, a, j);
            Tc = T - Ta - Tb;
            l = (va + vc) / 2 * Ta + vc / 2 * Tb + vc * Tc;
        }
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
        double l4 = l;
#endif
        if (pt < l) {
            //% clear
            //% syms va j Ta pt T a f(Ta)
            //% v  = va + j*Ta*Ta/4
            //% la = (v+va)/2*Ta
            //% Tb = (v + a^2/j)/a
            //% lb = Tb*v/2
            //% lc = (T - Ta - Tb)*v
            //%
            //% l = la+lb+lc
            //%
            //% collect(l-pt,Ta)
            //%
            //% k4 = -j^2/32
            //% k3 = -j/8
            //% k2 = (j*(a^2/j + va))/8 - (j*va)/8 - (j*(a^2/j - T + va))/4
            //% k0 = (va*(a^2/j + va))/2 - va*(a^2/j - T + va) - pt
            //%
            //% 考虑4次方程过于复杂，这里用迭代方法
            double k4 = -j*j / (32.0 * a);
            double k3 = -j / 8.0;
            double k2 = (T * j) / 4.0 - a / 8.0 - (j * va) / (4.0 * a);
            double k0 = T * va - pt - va * va / (2.0 * a) - (a * va) / (2.0 * j);
            //% 求其上下界
            //% syms f(Ta) g(Ta)
            //% f(Ta) = k4*Ta^4 + k3*Ta^3 + k2*Ta^2 + k0
            //% g(Ta) = 4*k4*Ta^3 + 3*k3*Ta^2 + 2*k2*Ta
            //% solve(g,Ta)
            //%
            //% 可得 f 的三个极值点：
            //% r0 = 0
            //% r1 = (-3*a + (a^2 + (T*j*a)*16 - (j*va)*16)^(1/2))/(2*j);
            //% r2 = (-3*a - (a^2 + (T*j*a)*16 - (j*va)*16)^(1/2))/(2*j);
            //% 
            //% 分析极值点分布，有：
            //% r1 = (-3*a + (a^2 + (T*j*a)*16 - (j*va)*16)^(1/2))/(2*j)
            //%    = (-3*a + (a^2 + 16*j*(T*a-va))^(1/2))/(2*j)
            //%
            //% 由于b段必定可达加速度a，因此有 T >= Tb = v/a+a/j >= va/a+a/j
            //% 因此有：
            //% r1 =  (-3*a + (a^2 + 16*j*(T*a-va))^(1/2))/(2*j)
            //%    >= (-3*a + (17*a^2)^(1/2))/(2*j)
            //%    >  0
            //% 而显然 r2 < 0
            //% 因此其极值点分布为：
            //% r2 < r0 = 0 < r1
            //%
            //% 下求 f(0)：
            //% f(0) =  T*va - pt - va^2/(2*a) - (a*va)/(2*j)
            //%      =  -pt + va*(Ta + Tb + Tc - (va/a + a/j)/2)
            //%      =  -pt + va * Ta + va * (Tb-(va/a + a/j)/2) + va * Tc
            //%      <= -pt + (va+v)/2*Ta + va*(v/a-va/(2*a)+a/(2*j)) + v*Tc
            //%      =  -pt + la + lc + 1/(2*a)*(2*va*v - va*va + va*a*a/j)
            //%      =  -pt + la + lc + 1/(2*a)*(v*v-(v*v-2*v*va +va*va) + va*a*a/j)
            //%      =  -pt + la + lc + 1/(2*a)*(v*v + va*a*a/j)
            //%      <= -pt + la + lc + 1/(2*a)*(v*v + v*a*a/j)
            //%      =  -pt + la + lc + v*(v/a + a/j)/2
            //%      =  -pt + la + lc + lb
            //%      =  0
            //%
            //% 因此可用 newton_raphson 方法搜索 [0,r1] 内的取值
            double Ta_below = 0;
            double Ta_upper = (-3.0*a + std::sqrt(a*a + T*j*a*16.0 - j*va*16.0))/(2.0*j);
            param.Ta_ = newton_raphson_binary_search([k4,k3,k2,k0](double x)->double {return x*x*(x*(x*k4 + k3) + k2) + k0; }, Ta_below, Ta_upper);
            param.vc_ = va + j * Ta * Ta / 4.0;
            param.Tb_ = (vc + Z1) / a;
            param.vb_ = 0.0;
            param.mode_ = 0;

            // adjust //
            param.Ta_ = std::max(0.0, param.Ta_);
            param.Tb_ = std::max(0.0, param.Tb_);
            if (param.Ta_ + param.Tb_ > T + t_cons) {
                double ratio = T / (param.Ta_ + param.Tb_);
                param.Ta_ *= ratio;
                param.Tb_ *= ratio;
            
            }

#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
            //% error %
            if (Ta < -t_cons || Tb < -t_cons || T - Ta - Tb < -t_cons
                || std::abs(Ta * (va + vc) / 2 + Tb * vc / 2 + (T - Ta - Tb) * vc - pt) > l_cons
                ) {
                double l = k4 * Ta * Ta * Ta * Ta + k3 * Ta * Ta * Ta + k2 * Ta * Ta + k0;
                auto [Tmax, Tmin] = s_compute_s_curve_Tmax_Tmin(param);
                THROW_FILE_LINE("ERROR l4");
            }
#endif

            return;
        }

        //% ------------------ l5 --------------------- %
        //% vb为0，a段达不到最大加速度，b段可达到最大加速度
        //%
        //% 此时需要满足3个条件：
        //% A. Ta + Tb <= T
        //% B. v - va  >= a^2/j && v >= a^2/j && v <= max_v && v >= va
        //% C. pt <= l
        //%
        //% 首先根据条件 A 和 B 确定 v 的取值范围，然后再计算 l
        //% Ta = (v-va)/a + a/j
        //% Tb = v/a + a/j
        //% 带入条件 A, 有：
        //% v <= - a^2/j + (T*a)/2 + va/2
        //%
        //% v_upper = min(max_v, - a^2/j + (T*a)/2 + va/2)
        //% v_below = va + a^2/j
        v_upper = std::min(vc_max, - Z1 + (T*a)/2 + va/2);
        v_below = va + Z1;
        l = -1;
        if (v_below <= v_upper) {
            vc = v_upper;
            Ta = (vc - va) / a + a / j;
            Tb = vc / a + a / j;
            Tc = T - Ta - Tb;
            l = Ta * (vc + va) / 2 + Tb * vc / 2 + Tc * vc;
        }
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
        double l5 = l;
#endif
        if (pt < l) {
            //% clear
            //% syms va j Ta pt T a f(Ta)
            //% v  = va + Ta*a - a^2/j
            //% la = (v+va)/2*Ta
            //% Tb = (v + a^2/j)/a
            //% lb = Tb*v/2
            //% lc = (T - Ta - Tb)*v
            //%
            //% l  = la+lb+lc-pt
            //%
            //% collect(l,Ta)
            //%
            //% k2 = -a
            //% k1 = (a*(T - va/a) + a^2/j)
            //% k0 = (T - va/a)*(va - a^2/j) + (va*(va - a^2/j))/(2*a)- pt
        
            double k2 = -a;
            double k1 = (a*T - va + Z1);
            double k0 = (va - Z1)*(T - va/a/2)- pt;
    
            param.Ta_ = (-k1 + std::sqrt(k1*k1 - 4*k0*k2))/(2*k2);
            param.vc_ = va + Ta*a - Z1;
            param.vb_ = 0;
            param.Tb_ = (vc + Z1)/a;
            param.mode_ = 0;
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
            //% error %
            if (Ta < -t_cons || Tb < -t_cons || T - Ta - Tb < -t_cons
                || std::abs(Ta * (va + vc) / 2 + Tb * vc / 2 + (T-Ta-Tb)*vc - pt) > l_cons
                ) {
                THROW_FILE_LINE("ERROR l5");
            }
#endif
            return;
        }

        //% ------------------ l6 --------------------- % 
        //% vb不为0，达不到max_v，a段达不到最大加速度，b段可达到最大加速度
        //%
        //% 此时需要满足3个条件：
        //% A. Ta + Tb =  T
        //% B. v - va  <= a^2/j && v - vb >= a^2/j && v <= max_v && v >= va
        //% C. pt <= l
        //%
        //% 条件 A   可得 Ta  = T - Tb
        //%            => Ta <= T - 2*a/j   (因为 Tb >= 2*a/j)
        //%
        //% 条件 B.1 可得 Ta <= 2*a/j
        //% 条件 B.2 可得 va + j*Ta*Ta/4 >= a^2/j
        //%            => Ta >= 2*sqrt(max(0, a^2/j - va)/j)
        //% 条件 B.3 可得 Ta <= T_va_to_max_v
        //% 条件 B.4 可得 Ta >= 0  (包含在B.2中)
        double Ta_upper = std::min(std::min(T_va_to_max_v, 2.0*a/j), T - 2.0*a/j);
        double Ta_below = 2.0*std::sqrt(std::max(0.0, Z1 - va)/j);
        if (Ta_upper >= Ta_below) {
            Ta = Ta_upper;
            vc = va + j * Ta * Ta / 4.0;
            la = Ta * (vc + va) / 2.0;
            Tb = T - Ta;
            vb = vc - Tb * a + Z1;
            lb = Tb * (vc + vb) / 2.0;
            l = la + lb;
        }
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
        double l6 = l;
#endif
        if (pt < l) {
            //%   clear
            //%   syms va j Ta a T
            //%   v  = va + j*Ta*Ta/4
            //%   la = j/8*Ta^3 + va*Ta
            //%   Tb = T-Ta
            //%   vb = v - Tb*a + a^2/j;
            //%   lb = Tb*(v+vb)/2
            //%   l  = la + lb
            //%
            //%   【result】:
            //%   带入方程la + lb = pt
            //%   可得：
            //%   k3*Ta^3 + k2*Ta^2 + k1*Ta + k0
            //%   其中：
            //%   k3 = - j/8
            //%   k2 = (T*j)/4 - a/2
            //%   k1 = - a^2/(2*j) + T*a
            //%   k0 = (T*(a^2/j - T*a + 2*va))/2 - pt
            //%
            //%   【condition】:
            //%   Tb > 2*a/j
            //%   => Ta < T - 2*a/j
            //%   于是：
            //%   0 <= Ta <= min(T, 2*a/j, T - 2*a/j)
            //%     
            //%   计算Ta = min(T, 2*a/j, T - 2*a/j)
            //%   l3 = la + lb  
            double k3 = -j / 8;
            double k2 = (T * j) / 4 - a / 2;
            double k1 = -Z1 / 2 + T * a;
            double k0 = (T * (Z1 - T * a + 2 * va)) / 2 - pt;
            //% 选根
            //% syms f(Ta) g(Ta)
            //% f(Ta) = k3*Ta^3 + k2*Ta^2 +k1*Ta + k0
            //% g(Ta) = diff(f,Ta)
            //% solve(g,Ta)
            //% 
            //% 可得其极值：
            //% r1 = 4/3*T - 2/3*a/j
            //% r2 = -(2*a)/j
            //%
            //% 由于b段可达最大加速度，因此必有 T >= 2a/j
            //% 于是
            //% r1 >= 4/3*T - 1/3*T = T
            //% 因此其上下界为 [0,T]
            param.Ta_ = newton_raphson_binary_search([k0,k1,k2,k3](double x)->double {
                return x*(x*(k3 * x + k2) + k1) + k0;
                }, 0, T);
            param.vc_ = va + j * Ta * Ta / 4;
            param.Tb_ = T - Ta;
            param.vb_ = s_acc_vend(vc, -a, -j, Tb);
            param.mode_ = 0;

#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
            //% error %
            if (Ta < -t_cons || Tb < -t_cons || T - Ta - Tb < -t_cons
                || std::abs(Ta * (va + vc) / 2 + Tb * (vb + vc) / 2 + (T - Ta - Tb)*vc - pt) > l_cons
                || (vb - param.vb_max_) > std::max(param.vb_max_ * 1e-10, 1e-10)
                ) {
                auto [Tmax, Tmin] = s_compute_s_curve_Tmax_Tmin(param);
                THROW_FILE_LINE("ERROR l6");
            }
#endif
            return;
        }

        //
        //% ------------------ l8 -------------------- 
        //% vb不为0，达不到max_v，a段可达到最大加速度，b段可达到最大加速度
        Ta_upper = std::min(T_va_to_max_v, T - 2*a/j);
        Ta_below = 2*a/j;
        l=-1;
        if (Ta_upper >= Ta_below) {
            Ta = Ta_upper;
            vc = va + Ta * a - Z1;
            la = (va + vc) * Ta / 2;
            Tb = T - Ta;
            vb = s_acc_vend(vc, -a, -j, Tb);
            lb = Tb * (vc + vb) / 2;
            l = la + lb;
        }
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
        double l8 = l;
#endif
        if (pt < l) {
            //    % syms va T a j pt Ta
            //    % v  = va + Ta*a - a^2/j
            //    % la = Ta*(va + v)/2
            //    % Tb = T-Ta
            //    % vb = v - Tb*a + a^2/j;
            //    % lb = Tb*(v+vb)/2
            //    %
            //    % 根据 la + lb = pt，有：
            //    % collect(la+lb-pt,Ta)
            //    % - a*Ta^2 + 2*T*a*Ta - pt - (T*(T*a - 2*va + a^2/j))/2
            //    %
            //    % 可得方程系数
            double k2 = -a;
            double k1 = 2.0 * T * a;
            double k0 = -pt - (T * (Z1 + T * a - 2.0 * va)) / 2.0;
            //
            //    % 选根
            //    % 其极值为：
            //    % r = k1 / (2*k2) = T
            //    % 应有 Ta < T
            //    % 故而选其较小的根
            param.Ta_ = (-k1 + std::sqrt(k1 * k1 - 4 * k0 * k2)) / 2 / k2;
            param.vc_ = va + Ta * a - Z1;
            param.Tb_ = T - Ta;
            param.vb_ = s_acc_vend(vc, -a, -j, Tb);
            param.mode_ = 0;
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
            //% error %
            if (Ta < -t_cons || Tb < -t_cons || T - Ta - Tb < -t_cons
                || std::abs(Ta * (va + vc) / 2 + Tb * (vb + vc) / 2 + (T - Ta - Tb) * vc - pt) > l_cons
                || (vb - param.vb_max_) > std::max(param.vb_max_ * 1e-10, 1e-10)
                ) {
                THROW_FILE_LINE("ERROR l8");
            }
#endif
            return;
        }

        //% ------------------ l10 -------------------- 
        //% vb不为0，可达max_v，b段可达到最大加速度
        Ta = std::min(s_acc_time(va, vc_max, a, j),T);
        vc = vc_max;
        Tb = std::min(std::min(T - Ta, s_acc_time(0, vc_max, a, j)), 2*a/j);
        l = -1;

        if (Tb >= 2 * a / j) {
            la = (va + vc_max) * Ta / 2;
            vb = s_acc_vend(vc, -a, -j, Tb);
            lb = Tb * (vc + vb) / 2;
            Tc = T - Ta - Tb;
            lc = Tc * vc_max;
            l = la + lb + lc;
        }
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
        double l10 = l;
#endif // DEBUG_ARIS_PLAN_TRAJECTORY
        if (pt < l) {
            double B = -a / j;
            double C = -(2.0 * la - 2.0 * pt + 2.0 * vc_max * (T - T_va_to_max_v)) / a;

            param.Tb_ = std::max((-B + std::sqrt(B*B - 4.0 * C)) / 2.0, 0.0);
            param.Ta_ = s_acc_time(va, vc_max, a, j);
            param.vb_ = vc_max - Tb * a + Z1;
            param.vc_ = vc_max;
            param.mode_ = 0;
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
            //% error %
            if (Ta < -t_cons || Tb < -t_cons || T - Ta - Tb < -t_cons
                || std::abs(Ta * (va + vc) / 2 + Tb * (vb + vc) / 2 + (T - Ta - Tb) * vc - pt) > l_cons
                || (vb - param.vb_max_) > std::max(param.vb_max_ * 1e-10, 1e-10)
                ) {

                

                THROW_FILE_LINE("ERROR l10");
            }
#endif
            return;
        }

        //% ------------------ l7 --------------------- 
//% vb不为0，达不到max_v，a段达不到最大加速度，b段达不到最大加速度
//%
//% 此时需要满足3个条件：
//% A. Ta + Tb =  T
//% B. v - va  <= a^2/j && v - vb <= a^2/j && v <= max_v && v >= va
//% C. pt <= l
//%
//% 条件 A   可得 Ta =  T-Tb
//%            => Ta >= T - 2*a/j   (因为Tb < 2*a/j)
//%
//% 条件 B.1 可得 Ta <= 2*a/j
//% 条件 B.2 无法得到有效等式，因为vb可以为为任意值
//% 条件 B.3 可得 Ta <= T_va_to_max_v
//% 条件 B.4 可得 Ta >= 0
        Ta_upper = std::min(std::min(T, T_va_to_max_v), 2.0 * a / j);
        Ta_below = std::max(0.0, T - 2.0 * a / j);
        l = -1;
        if (Ta_upper - Ta_below >= 0) {
            Ta = Ta_upper;
            vc = va + j * Ta * Ta / 4;
            Tb = T - Ta;
            vb = std::max(vc - j * Tb * Tb / 4, 0.0);
            l = Ta * (va + vc) / 2.0 + Tb * (vb + vc) / 2.0;
        }
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
        double l7 = l;
#endif
        //% 此处的cons为必须，因为有可能vb = v
        if (pt < l || T < std::min(T_va_to_max_v, 2.0*a/j)) {
            //    % v  = va + j*Ta*Ta/4
            //    % la = j/8*Ta^3 + va*Ta
            //    % Tb = T-Ta
            //    % lb = Tb*(v+vb)/2
            //    % 【result】:
            //    % 带入方程la + lb = pt
            //    % 可得：
            //    % k2*Ta^2 +k1*Ta + k0
            //    % 其中：
            //    % k2 = T*j/8
            //    % k1 = -3*T^2*j/8
            //    % k0 = pt - (T*(2*va - (T^2*j)/4))/2
            //    %
            //    % Ta = (-k1+sqrt(k1*k1-4*k0*k2))/2/k2
            double k2 = T * j / 8.0;
            double k1 = -3.0 * Z2 / 8.0;
            double k0 = pt - T * va + T * Z2 / 8.0;
            //
            //% 选根
            //% 其极值为(k1) / (2 * k2) = (3 * T) / 2
            //% 因此需选其较小的根
            param.Ta_ = (-k1 - std::sqrt(k1 * k1 - 4.0 * k0 * k2)) / 2.0 / k2;

            // adjust //
            param.Ta_ = std::max(param.Ta_, 0.0);
            // adjust finished //

            param.vc_ = va + j * Ta * Ta / 4.0;
            param.Tb_ = T - Ta;
            param.vb_ = s_acc_vend(vc, -a, -j, Tb);
            param.mode_ = 0;
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
            //% error %
            if (Ta < -t_cons || Tb < -t_cons || T - Ta - Tb < -t_cons
                || std::abs(Ta * (va + vc) / 2 + Tb * (vb + vc) / 2 + (T - Ta - Tb) * vc - pt) > l_cons
                || (vb - param.vb_max_) > std::max(param.vb_max_ * 1e-10, 1e-10)
                ) {
                //double l = k2 * Ta * Ta + k1 * Ta + k0;
                std::cout << "debug -----------------------------" << std::endl;
                auto [Tmax, Tmin] = s_compute_s_curve_Tmax_Tmin(param);
                THROW_FILE_LINE("ERROR l7");
            }
#endif
            return;
        }


        //
        //% ------------------ l9 -------------------- 
        //% vb不为0，达不到max_v，a段可达到最大加速度，b段不可达到最大加速度
        Ta_upper = std::min(T, T_va_to_max_v);
        Ta_below = 2 * a / j;
        l = -1;
        if (Ta_upper >= Ta_below) {
            Ta = Ta_upper;
            vc = s_acc_vend(va, a, j, Ta);
            la = (va + vc) * Ta / 2;
            Tb = T - Ta;
            vb = s_acc_vend(vc, -a, -j, Tb);
            lb = Tb * (vc + vb) / 2;
            l = la + lb;
        }
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
        double l9 = l;
#endif
        if (pt < l || T < T_va_to_max_v) {
            //    % syms va T a j pt Ta
            //    % v  = va + Ta*a - a^2/j
            //    % la = Ta*(va + v)/2
            //    % Tb = T-Ta
            //    % vb = v - j*Tb*Tb/4;
            //    % lb = Tb*(v+vb)/2
            //    %
            //    % 根据 la + lb = pt，有：
            //    % collect(la+lb-pt,Ta)
            //    % (j*Ta^3)/8 + (- a/2 - (3*T*j)/8)*Ta^2 + ((T^2*j)/8 + a^2/(2*j) + (T*(2*a + (T*j)/2))/2)*Ta - pt - (T*((T^2*j)/4 - 2*va + (2*a^2)/j))/2
            //    %
            //    % 可得方程系数
            double k3 = j / 8.0;
            double k2 = -a / 2.0 - 3.0 / 8.0 * T * j;
            double k1 = Z2 * 3.0 / 8.0 + Z1 / 2.0 + T * a;
            double k0 = -pt - (T * (Z2 / 8.0 + Z1 - va));
            //    
            //    % 选根
            //    % syms f(Ta) g(Ta)
            //    % f(Ta) = k3*Ta^3 + k2*Ta^2 +k1*Ta + k0
            //    % g(Ta) = diff(f,Ta)
            //    % solve(g,Ta)
            //    %
            //    % 得到:
            //    %
            //    % r1 = T + (2*a)/(3*j)
            //    % r2 = T + (2*a)/j
            //    %
            //    % 因为必有 2*a/j <= Ta <= T, 因此其上下界为：
            //    % T_below = 2*a/j
            //    % T_upper = T
            //    
            param.Ta_ = newton_raphson_binary_search([k0, k1, k2, k3](double x) ->double {
                return x * (x * (k3 * x + k2) + k1) + k0;
                }, 2.0 * a / j, T);
            param.vc_ = va + Ta * a - Z1;
            param.Tb_ = std::max(T - Ta, 0.0);
            param.vb_ = s_acc_vend(vc, -a, -j, Tb);
            param.mode_ = 0;
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
            //% error %
            if (Ta < -t_cons || Tb < -t_cons || T - Ta - Tb < -t_cons
                || std::abs(Ta * (va + vc) / 2 + Tb * (vb + vc) / 2 + (T - Ta - Tb) * vc - pt) > l_cons
                || (vb - param.vb_max_) > std::max(param.vb_max_ * 1e-10, 1e-10)
                ) {
                double l = Ta * (Ta * (k3 * Ta + k2) + k1) + k0;
                param.Ta_ = newton_raphson_binary_search([k0, k1, k2, k3](double x) ->double {
                    return x * (x * (k3 * x + k2) + k1) + k0;
                    }, 2.0 * a / j, T);
                THROW_FILE_LINE("ERROR l9");
            }
#endif
            return;
        }

        //% ------------------ l11 -------------------- 
        //% vb不为0，可达max_v，b段不可达到最大加速度
        
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
        Ta = T_va_to_max_v;
        la = (va + vc_max) * Ta / 2;
        Tb = 0;
        lb = 0;
        Tc = T - Ta - Tb;
        lc = Tc * vc_max;
        l = la + lb + lc;
        if (pt > l + l_cons) {
            THROW_FILE_LINE("ERROR l12");
        }
        double l11 = l;
#endif


        // 计算 //
        param.Ta_ = T_va_to_max_v;
        param.Tb_ = std::cbrt(std::max(((va + vc_max) * Ta / 2 + vc_max * (T - T_va_to_max_v) - pt) * 8 / j, 0.0));
        param.vb_ = vc_max - j * Tb * Tb / 4;
        param.vc_ = vc_max;
        param.mode_ = 0;
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
        //% error %
        if (Ta < -t_cons || Tb < -t_cons || T - Ta - Tb < -t_cons
            || std::abs(Ta * (va + vc) / 2 + Tb * (vb + vc) / 2 + (T - Ta - Tb) * vc - pt) > l_cons
            || (vb - param.vb_max_) > std::max(param.vb_max_ * 1e-10, 1e-10)
            ) {
            THROW_FILE_LINE("ERROR l11");
        }
#endif
        return;



        
	}

    // 计算 S 曲线的最大最小时间
    //
    // param 中的以下为输入：
    // - pb
    // - vc_max_
    // - vb_max_
    // - a
    // - j
    // - pa
    // - va
    // 
    // 以下不变：
    // - vb_
    // - vc_
    // - Ta_
    // - Tb_
    // - mode_
    // - T_
    //
    // 函数返回：
    // Tmax, Tmin
    auto s_compute_scurve_Tmax_Tmin(const SCurveParam& param)->std::tuple<double, double> {
        const double va     = param.va_;
        const double vb_max = param.vb_max_;
        const double vc_max = param.vc_max_;
        const double a      = param.a_;
        const double j      = param.j_;
        const double pt     = param.pb_ - param.pa_;
        double Tmin_max = 1e-3;
        if (pt < std::numeric_limits<double>::epsilon() * 100){
            if (va > std::numeric_limits<double>::epsilon() * 100) {
                return std::make_tuple<double, double>(-1.0, -1.0);
            }
            else {
                return std::make_tuple(std::numeric_limits<double>::infinity(), Tmin_max);
            }
        }

        
        double Tmax, Tmin, vb, l, v1, v2, v_upper, v_below;

        double Z1 = a * a / j;
        double T_va_to_vb = s_acc_time(va, vb_max, a, j);
        double l_va_to_vb = T_va_to_vb * (va + vb_max) / 2;

        // failed //
        if ((!std::isfinite(Z1)) || (!std::isfinite(T_va_to_vb)) || (!std::isfinite(l_va_to_vb)) || (va > vb_max && l_va_to_vb > pt))
            return std::make_tuple(-1, -1);

        double pacc = va - 1.5 * Z1 > 0 ? 0.5 * (Z1 / 2 + va) * (Z1 / 2 + va) / a : 4.0 / 3.0 * va * std::sqrt(2.0 / 3.0 * va / j);
        if (pacc <= pt)
            Tmax = std::numeric_limits<double>::infinity();
        else {
            //% 计算vb
            //% 【条件1】 加速度正好可以达到a时，所前进的长度
            //% 此时 vb = - a^2/j + va
            //% 前进时间 t = (va-vb)/a+a/j
            //% 前进长度 l = t*(va+vb)/2 = (2*a*va)/j - a*a*a/j/j
            if (va<a * a / j || (2 * a * va) / j - a * a * a / j / j > pt) {
                //% 此时无匀速段
                //% 前进时间为：t = 2 * sqrt( (va-vb) / j );
                //% 前进长度的平方为：
                //% p^2 = (t * (va+vb)/2)^2
                //%     =  va^3/j + (va^2*vb)/j - (va*vb^2)/j - vb^3/j
                //% vb 需求解一元三次方程 【1,va,-va^2,pb-pa-va^3】
                //%
                //% vb 取尽可能大的实数
                //% vb 的范围取自 【va/3，va】,
                //% 因为T的极值为 sqrt(va*8/3/j)，此时带入vb的公式，可得

                double vb = newton_raphson_binary_search([va, j, pt](double x) {return std::sqrt((va - x) / j) * (va + x) - pt; }
                , va / 3, va);
                Tmax = s_acc_time(va, vb, a, j);
            }

            else {
                //% 此时有匀速段
                //% 前进时间为：t = (va-vb)/a+a/j;
                //% 前进长度为：
                //% l = t*(va+vb)/2
                //%   = - vb^2/(2*a) + (a*vb)/(2*j) + (va*(a/j + va/a))/2
                //% vb 需求解一元二次方程 【
                //%       1,
                //%       (va - a*(a/j + va/a)),
                //%       2*(pb-pa)*a- a*va*(a/j + va/a)
                //% 】
                //% 对于根来说，应当取大值，这是因为Tmax应该尽可能的小
                double B = -Z1;
                double C = 2 * pt * a - va * Z1 - va * va;
                double vb = (-B + std::sqrt(B * B - 4 * C)) / 2;
                Tmax = s_acc_time(va, vb, a, j);
            }
        }

        //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 计算Tmin：%%%%%%

        //% ------------------ l1 --------------------
        //% 加速不到max_vb, 无法达到最大加速度a
        //% va < vb < max_vb, vb - va < a^2 / j
        vb = std::min(va + a * a / j, vb_max);
        l = std::min(s_acc_time(va, vb, a, j) * (va + vb) / 2, l_va_to_vb);
        if (va < vb_max && pt < l) {
            //%     %%%%%%%%%%%%%%%%% METHOD1 %%%%%%%%%%%%%%%%% 
            //%     % clear
            //%     % syms va vb a j pt
            //%     % T = 2 * sqrt((vb-va) / j )
            //%     % l = T * (va+vb)/2
            //%     % expand((T*(va + vb)/2)^2 *j - pt^2*j)
            //%     % coeffs(l^2*j - pt^2*j, vb)
            //%     %
            //%     % k3 = 1;
            //%     % k2 = va;
            //%     % k1 = -va^2;
            //%     % k0 = -j*pt^2 - va^3;
            //%     %
            //%     % f(vb) = k3*vb^3 + k2*vb^2 + k1*vb + k0 
            //%     % 
            //%     % 对f求导，可知
            //%     % df    = 3*k3*vb^2 + 2*k2*vb + k1
            //%     %
            //%     % 其极值为 -va 与 1/3va
            //%     % 将此2值带入，可以看出
            //%     % f(-va)  = -j*pt^2
            //%     % f(va/3) = - j*pt^2 - (32*va^3)/27
            //%     % 因此该三次方程在pt不为0时只有一个根，为0时有3个根，此时选择最大的
            //%     k3 = 1;
            //%     k2 = va;
            //%     k1 = -va^2;
            //%     k0 = -j*pt^2 - va^3;
            //%     
            //%     r = cubic_equation_solve(k3,k2,k1,k0);
            //%     % 选根 %
            //%     % 【注意】 pt为0时有3个根，两个为 -va，因此这里选择最大的
            //%     if(isreal(r(3)))
            //%         vb = r(3);
            //%     else
            //%         vb = r(1);
            //%     end

            //%%%%%%%%%%%%%%%%% METHOD2 %%%%%%%%%%%%%%%%% 
            //% newton raphson %
            vb = newton_raphson_binary_search([va, j, pt](double x) {return std::sqrt((x - va) / j) * (va + x) - pt; }
            , va, vb_max);
            Tmin = s_acc_time(va, vb, a, j);
            return std::make_tuple(Tmax, Tmin);
        }

        //% ------------------ l2 --------------------
        //% 加速不到max_vb, 可以达到最大加速度a
        //% va < vb < max_vb, vb - va >= a^2 / j
        l = l_va_to_vb;
        if (pt < l) {
            //% clear
            //% syms va vb a j pt;
            //% T = (vb-va) / a + a/j;
            //% l = T * (va+vb)/2;
            //% coeffs(l,vb)
            //% 
            //% k2 = 1/(2*a);
            //% k1 = a/(2*j);
            //% k0 = (va*(a/j - va/a))/2 - pt;
            //% 
            //% 其极值为 vb = -k1/2/k2 = -a^2/(2*j)
            //% 因此取其右侧大值
            double k2 = 1.0 / (2 * a);
            double k1 = a / (2 * j);
            double k0 = (va * (a / j - va / a)) / 2 - pt;

            //% 选根 %
            vb = (-k1 + std::sqrt(k1 * k1 - 4 * k2 * k0)) / (k2 * 2);
            Tmin = s_acc_time(va, vb, a, j);
            return std::make_tuple(Tmax, Tmin);
        }

        //% ------------------ l3 --------------------
        //% 可以加速到某个v，无匀速段，之后减速到max_vb，加减速过程，加速度均不超过a
        //% v1 = max(va,vb), v2 = min(va,vb), v1 < v < min(v2 + a^2/j, max_v)
        vb = vb_max;
        v1 = std::max(va, vb);
        v2 = std::min(va, vb);
        v_upper = std::min(v2 + Z1, vc_max);
        v_below = v1;
        l = -1;
        if (v_upper >= v_below) {
            l = s_acc_time(v1, v_upper, a, j) * (v_upper + v1) / 2 +
                s_acc_time(v2, v_upper, a, j) * (v_upper + v2) / 2;
        }
        if (pt < l) {
            //% 以下为方程求解
            //% syms v1 v2 a j pt v;
            //% T1 = 2 * sqrt((v-v1) / j )
            //% T2 = 2 * sqrt((v-v2) / j )
            //% l = T1 * (v1 + v) / 2 + T2 * (v2 + v) / 2
            //% 
            //% 其导数为：
            //% diff(l,v) == T1/2 + T2/2 + (v + v1)/(j*T1) + (v + v2)/(j*T2)
            //% 
            //
            //  考虑到sqrt在0附近数值求解的稳定性，因此设置 x = v-v1，以防在 v 接近 v1 的情况下
            //  ，sqrt(x - v1)精度不够
            // 
            double v_minus_v1 = newton_raphson_binary_search([v1,v2,j,pt](double x) {
                return std::sqrt(x / j) * (2.0*v1 + x) + std::sqrt((v1 - v2 + x) / j) * (v1 + v2 + x) - pt;
                }
                
                , 0.0, v_upper - v_below);

            double T1 = 2 * std::sqrt(v_minus_v1 / j);
            double T2 = 2 * std::sqrt((v1 - v2 + v_minus_v1) / j);
            Tmin = T1 + T2;
            return std::make_tuple(Tmax, Tmin);
        }

        //% ------------------ l4 --------------------
        //% 可以加速到某个v，无匀速段，之后减速到max_vb，加减速过程，有一段可以达到加速度a
        //% v1 = max(va,vb), v2 = min(va,vb), v1 < v < min(v2 + a^2/j, max_v)
        v1 = std::max(va, vb);
        v2 = std::min(va, vb);
        v_upper = std::min(v1 + Z1, vc_max);
        v_below = std::max(v2 + Z1, v1);
        l = -1;
        if (v_upper >= v_below) {
            l = s_acc_time(v1, v_upper, a, j) * (v_upper + v1) / 2 +
                s_acc_time(v2, v_upper, a, j) * (v_upper + v2) / 2;
        }
        if (pt < l){
            //% 以下为方程求解
            //% syms v1 v2 a j pt v;
            //% T1 = 2 * sqrt((v-v1) / j )
            //% T2 = (v-v2) / a + a/j
            //% l1 = T1 * (v1 + v) / 2
            //% l2 = T2 * (v2 + v) / 2
            //% l  = l1 + l2
            //% 
            //% 可以化成1元4次方程：
            //% eq = expand(l1^2 - (pt-l2)^2)
            //% 
            //% 受限于难以求解1元4次方程，因此还是使用牛顿法 
            //%
            //% eq = l - pt
            //% >> eq  = sqrt((x-v1)/j)*(v1+x) + ((x-v2)/a+a/j)*(v2+x)/2 - pt)
            //% deq = diff(eq,v)
            //% >> deq = T1/2 + T2/2 + (v + v2)/(2*a) + (v+v1)/(j*T1)
            //% 
            //  考虑到sqrt在0附近数值求解的稳定性，因此设置 x = v-v1，以防在 v 接近 v1 的情况下
            //  ，sqrt(x - v1)精度不够
            // 
            double v_minus_v1 = newton_raphson_binary_search([v1, v2, a, j, pt](double x)->double {
                return std::sqrt(x / j) * x + ((v1 - v2 + x) / a + a / j) * (v1 + v2 + x) / 2 - pt;
                }
                , v_below, v_upper);

            double T1 = 2 * std::sqrt(v_minus_v1 / j);
            double T2 = (v1 - v2 + v_minus_v1) / a + a / j;
            Tmin = T1 + T2;
            return std::make_tuple(Tmax, Tmin);
        }

        //% ------------------ l5 --------------------
        //% 可以加速到某个v，无匀速段，之后减速到max_vb，加减速过程，两段都可以达到加速度a
        //% v1 = max(va,vb), v2 = min(va,vb), v1 + a^2/j < v < max_v
        v1 = std::max(va, vb);
        v2 = std::min(va, vb);
        v_upper = vc_max;
        v_below = v1 + Z1;
        l = -1;
        if (v_upper >= v_below) {
            l = s_acc_time(v1, v_upper, a, j) * (v_upper + v1) / 2 + 
                s_acc_time(v2, v_upper, a, j) * (v_upper + v2) / 2;
        }
        if (pt < l) {
            //% 以下为方程求解
            //% syms v1 v2 a j pt v;
            //% T1 = (v-v1) / a + a/j
            //% T2 = (v-v2) / a + a/j
            //% l1 = T1 * (v1 + v) / 2
            //% l2 = T2 * (v2 + v) / 2
            //% l  = l1 + l2
            //% 
            //% 可以化成1元2次方程：
            //% f(v) = l-pt
            //% f(v) == 0
            //% 
            //% 其系数为：
            //% k2 = 1/a
            //% k1 = a/j
            //% k0 = (v1*(a/j - v1/a))/2 - pt + (v2*(a/j - v2/a))/2
            //%
            //% 分析 f 的极值，有：
            //% df = diff(f,v)
            //% solve(df,v)
            //%
            //% ans = -a^2/(2*j)
            //%
            //% 因此f的根位于上述结果两侧，应取较大值
            double k2 = 1 / a;
            double k1 = a / j;
            double k0 = (v1 * (a / j - v1 / a)) / 2 - pt + (v2 * (a / j - v2 / a)) / 2;

            double v = (-k1 + std::sqrt(k1 * k1 - 4 * k0 * k2)) / (2 * k2);

            double T1 = (v - v1) / a + a / j;
            double T2 = (v - v2) / a + a / j;
            Tmin = T1 + T2;

            Tmin = std::max(Tmin, Tmin_max);
            return std::make_tuple(Tmax,Tmin);
        }

        //% ------------------ l6 --------------------
        //% 可以加速到max_v，匀速运行一段时间，之后减速到max_vb
        //% v1 = max(va,vb), v2 = min(va,vb), v = max_v
        v1 = std::max(va,vb);
        v2 = std::min(va,vb);
        double v  = vc_max;
        double T1 = s_acc_time(v1,vc_max,a,j);
        double T2 = s_acc_time(v2,vc_max,a,j);
        double T3 = (pt - T1 * (v + v1)/2 - T2 * (v + v2)/2)/vc_max;
        Tmin = T1 + T2 + T3;
        return std::make_tuple(Tmax, Tmin);
	}

    // 以最大时间测试是否可能达到终止条件，即：
    // cond A: 所有末端速度均减为0
    // cond B: 达到最后一个节点，且结束速度满足要求
    auto test_curve_slow(std::list<SCurveNode>::iterator begin, std::list<SCurveNode>::iterator end)->bool {
        // 达到最后一个节点 //
        if (begin == std::prev(end))
            return true;

        std::vector<double> Tmaxs(begin->params_.size()), Tmins(begin->params_.size());
        for (Size i = 0; i < begin->params_.size(); ++i) {
            // 计算 vb
            s_compute_scurve_node(begin->params_[i]);

            // 更新 vb
            std::next(begin)->params_[i].va_ = begin->params_[i].vb_;

            // 计算 Tmax Tmin
            std::tie(Tmaxs[i], Tmins[i]) = s_compute_scurve_Tmax_Tmin(std::next(begin)->params_[i]);
        }

        double Tmin_all = *std::max_element(Tmins.begin(), Tmins.end());
        double Tmax_all = *std::min_element(Tmaxs.begin(), Tmaxs.end());

        if (Tmax_all == std::numeric_limits<double>::infinity()) {
            return true;
        }
        else if (Tmax_all < 0 || Tmax_all < Tmin_all) {
            return false;
        }
        else {
            for (auto& param : std::next(begin)->params_) {
                param.T_ = Tmax_all;
            }
            return test_curve_slow(std::next(begin), end);
        }
    }

    // 循环计算每个节点：
    auto ARIS_API s_compute_scurve(std::list<SCurveNode>::iterator begin_iter, std::list<SCurveNode>::iterator end_iter)noexcept->void {
        // 设置正确的 pa, 并检查 vc, a, j 等参数的合理性
        for (auto iter = std::next(begin_iter); iter != end_iter; ++iter) {
            // 设置正确的 pa
            for (Size i = 0; i < iter->params_.size(); ++i) {
                iter->params_[i].pa_ = std::prev(iter)->params_[i].pb_;
                
                if (iter->params_[i].pa_ > iter->params_[i].pb_)
                    THROW_FILE_LINE("POS SET NOT CORRECT");

                if (iter->params_[i].pb_ - iter->params_[i].pa_ >= 100 * std::numeric_limits<double>::epsilon()){
                    if(iter->params_[i].vc_max_ < 1e-10)
                        THROW_FILE_LINE("VEL SET NOT CORRECT");
                    if (iter->params_[i].a_ < 1e-10)
                        THROW_FILE_LINE("ACC SET NOT CORRECT");
                    if (iter->params_[i].j_ < 1e-10)
                        THROW_FILE_LINE("JERK SET NOT CORRECT");
                }
            }
        }

        // 设置正确的 vb_max,  vb_max 应小于上一个与这一个的 较小的 vc //
        for (auto iter = begin_iter; iter != end_iter && iter != std::prev(end_iter); ++iter) {
            // 设置正确的 vb_max
            for (Size i = 0; i < iter->params_.size(); ++i) {
                iter->params_[i].vb_max_ = std::min(iter->params_[i].vc_max_, iter->params_[i].vb_max_);
                iter->params_[i].vb_max_ = std::min(std::next(iter)->params_[i].vc_max_, iter->params_[i].vb_max_);
            }
        }

        // 设置最后的 vb_max 等于 0
        for (Size i = 0; i < std::prev(end_iter)->params_.size(); ++i) {
            std::prev(end_iter)->params_[i].vb_max_ = 0.0;
        }
        
        // 开始循环计算
        for (auto iter = begin_iter; iter != end_iter; ++iter) {
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
            static int count_ = 0;
            //std::cout << "trajectory count:" << count_ << std::endl;
            count_++;
#endif

            // STEP 1 : 计算全部末端的最大最小时间
            std::vector<double> Tmaxs(iter->params_.size()), Tmins(iter->params_.size());
            for (Size i = 0; i < iter->params_.size(); ++i) {
                // 设置正确的 pa 和 va
                if (iter != begin_iter) {
                    iter->params_[i].va_ = std::prev(iter)->params_[i].vb_;
                }
                
                // 计算 Tmax Tmin
                std::tie(Tmaxs[i], Tmins[i]) = s_compute_scurve_Tmax_Tmin(iter->params_[i]);
            }
            
            double Tmin_all = *std::max_element(Tmins.begin(), Tmins.end());
            double Tmax_all = *std::min_element(Tmaxs.begin(), Tmaxs.end());

            // STEP 2 : 基于 T_below 求得 T_upper 上限
            double T_upper = Tmax_all;
            double T_below = std::max(Tmin_all,0.001);

            if (T_upper == std::numeric_limits<double>::infinity()) {
                T_upper = std::max(T_below, 1.0) * 2.0; // in case T_below == 0.0
                for (auto& p : iter->params_)
                    p.T_ = T_upper;

                while (!test_curve_slow(iter, end_iter)) {
                    T_below = T_upper;
                    T_upper = 2.0 * T_upper;

                    for (auto& p : iter->params_)
                        p.T_ = T_upper;
                }
            }

            // STEP 3 : 二分法求解最优的可行时间
            double diff = std::abs(T_upper - T_below);
            double diff_last = 10 * diff;

            while (diff < diff_last) {
                diff_last = diff;
                double T_next = (T_upper + T_below) / 2;
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
                //std::cout << "  Tnext:" << T_next << std::endl;              
#endif
                for (auto& p : iter->params_)
                    p.T_ = T_next;

                if (test_curve_slow(iter, end_iter)) {
                    T_upper = T_next;
                } else {
                    T_below = T_next;
                }

                diff = std::abs(T_upper - T_below);
            }

            // STEP 4 : 设置可行时间并计算每段 s 曲线
            for (auto& p : iter->params_) {
                p.T_ = T_upper;
                s_compute_scurve_node(p);

                if (iter != begin_iter) {
                    LargeNum t0 = std::prev(iter)->params_[0].t0_ + std::prev(iter)->params_[0].T_;
                    for (auto& p : iter->params_) {
                        p.t0_ = t0;
                        //p.t0_count_ = std::prev(iter)->params_[0].t0_count_ + std::lround((t0 - std::fmod(t0, 1000.0))/1000.0);
                    }
                }
                
            }
        }
    }

    // 计算指定时间处的 p v a j
    auto ARIS_API s_scurve_at(const SCurveParam& param, LargeNum t, LargeNum* p_out, double* v_out, double* a_out, double* j_out)noexcept->void {
        const double va = std::max(param.va_, 0.0);
        const double vb_max = param.vb_max_;
        const double vc_max = param.vc_max_;
        const double a = param.a_;
        const double j = param.j_;
        const double pt = param.pb_ - param.pa_;
        const double T = param.T_;
        const double Ta = param.Ta_;
        const double Tb = param.Tb_;
        const LargeNum pa = param.pa_;
        const LargeNum pb = param.pa_ + pt;  // 注意，这里不能直接用 param.pb_，因为 count 有可能不对
        const double vb = param.vb_;
        const double vc = param.vc_;
        const int    mode = param.mode_;

        double t_ = t - param.t0_;

        LargeNum p_;
        double v_, a_, j_;
        // %CASE B
        if (mode == 1) {
            if (t_ < Ta) {
                p_ = pa + va * t_;
            }
            else if (t_ < T - Tb) {
                double Tacc = T - Ta - Tb;
                double si = aris::dynamic::s_sgn2(vb - va);
                t_ = t_ - Ta;
                if (Tacc >= 2.0 * a / j) {
                    if (t_ < a / j) {
                        p_ = pa + va * Ta + va * t_ + si / 6.0 * j * t_ * t_ * t_;
                    }
                    else if (t_ < (Tacc - a / j)) {
                        p_ = pa + va * Ta + va * a / j + si / 6.0 * a * a * a / j / j + (va + si / 2.0 * a * a / j) * (t_ - a / j) + si * a / 2.0 * (t_ - a / j) * (t_ - a / j);
                    }
                    else {
                        p_ = pa + va * Ta + (va + vb) / 2.0 * Tacc - (vb * (Tacc - t_) - si / 6.0 * j * (Tacc - t_) * (Tacc - t_) * (Tacc - t_));
                    }
                }
                else {
                    if (t_ < Tacc / 2.0) {
                        p_ = pa + va * Ta + va * t_ + si / 6.0 * j * t_ * t_ * t_;
                    }
                    else {
                        p_ = pa + va * Ta + (va + vb) / 2.0 * Tacc - (vb * (Tacc - t_) - si / 6.0 * j * (Tacc - t_) * (Tacc - t_) * (Tacc - t_));
                    }
                }
            }
            else {
                p_ = pb - vb * (T - t_);
            }
        }
        else {
            //%CASE A
            if (t_ < Ta) {
                double si = aris::dynamic::s_sgn2(vc - va);

                if (Ta >= 2.0 * a / j) {
                    if (t_ < a / j)
                        p_ = pa + va * t_ + si / 6.0 * j * t_ * t_ * t_;
                    else if (t_ < (Ta - a / j))
                        p_ = pa + va * a / j + si / 6.0 * a * a * a / j / j + (va + si / 2.0 * a * a / j) * (t_ - a / j) + si * a / 2 * (t_ - a / j) * (t_ - a / j);
                    else
                        p_ = pa + (va + vc) / 2.0 * Ta - (vc * (Ta - t_) - si / 6.0 * j * (Ta - t_) * (Ta - t_) * (Ta - t_));
                }
                else {
                    if (t_ < Ta / 2.0)
                        p_ = pa + va * t_ + si / 6.0 * j * t_ * t_ * t_;
                    else
                        p_ = pa + (va + vc) / 2.0 * Ta - (vc * (Ta - t_) - si / 6.0 * j * (Ta - t_) * (Ta - t_) * (Ta - t_));
                }
            }
            else if (t_ < T - Tb) {
                p_ = pa + (va + vc) / 2.0 * Ta + vc * (t_ - Ta);
            }
            else {
                double si = aris::dynamic::s_sgn2(vb - vc);
                if (Tb >= 2.0 * a / j) {
                    if (T - t_ < a / j)
                        p_ = pb - vb * (T - t_) + si / 6.0 * j * (T - t_) * (T - t_) * (T - t_);
                    else if (T - t_ < (Tb - a / j))
                        p_ = pb - vb * a / j + si / 6.0 * a * a * a / j / j - (vb - si / 2.0 * a * a / j) * (T - t_ - a / j) + si * a / 2.0 * (T - t_ - a / j) * (T - t_ - a / j);
                    else
                        p_ = pb - (vc + vb) / 2.0 * Tb + (vc * (Tb - T + t_) + si / 6.0 * j * (Tb - T + t_) * (Tb - T + t_) * (Tb - T + t_));
                }

                else {
                    if (T - t_ < Tb / 2.0)
                        p_ = pb - vb * (T - t_) + si / 6.0 * j * (T - t_) * (T - t_) * (T - t_);
                    else
                        p_ = pb - (vc + vb) / 2.0 * Tb + vc * (Tb - T + t_) + si / 6.0 * j * (Tb - T + t_) * (Tb - T + t_) * (Tb - T + t_);
                }
            }
        }

        //if (p_count) *p_count = param.pa_count_ + std::lround((p_ - std::fmod(p_, 1000.0)) / 1000.0);
        if (p_out) *p_out = p_;
    }
}