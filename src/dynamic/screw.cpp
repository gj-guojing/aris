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

#include "aris/dynamic/pose.hpp"
#include "aris/dynamic/screw.hpp"

namespace aris::dynamic{
	auto s_cm3(const double *a, double *cm_out) noexcept->void{
		cm_out[0] = 0;
		cm_out[1] = -a[2];
		cm_out[2] = a[1];
		cm_out[3] = a[2];
		cm_out[4] = 0;
		cm_out[5] = -a[0];
		cm_out[6] = -a[1];
		cm_out[7] = a[0];
		cm_out[8] = 0;
	}
	auto s_c3(const double *a, const double *b, double *c_out) noexcept->void{
		c_out[0] = -a[2] * b[1] + a[1] * b[2];
		c_out[1] = a[2] * b[0] - a[0] * b[2];
		c_out[2] = -a[1] * b[0] + a[0] * b[1];
	}
	auto s_c3(double alpha, const double *a, const double *b, double *c_out) noexcept->void{
		c_out[0] = alpha * (-a[2] * b[1] + a[1] * b[2]);
		c_out[1] = alpha * (a[2] * b[0] - a[0] * b[2]);
		c_out[2] = alpha * (-a[1] * b[0] + a[0] * b[1]);
	}
	auto s_c3i(const double *a, const double *b, double *c_out) noexcept->void{
		c_out[0] = a[2] * b[1] - a[1] * b[2];
		c_out[1] = -a[2] * b[0] + a[0] * b[2];
		c_out[2] = a[1] * b[0] - a[0] * b[1];
	}
	auto s_c3a(const double *a, const double *b, double *c_out) noexcept->void{
		c_out[0] += -a[2] * b[1] + a[1] * b[2];
		c_out[1] += a[2] * b[0] - a[0] * b[2];
		c_out[2] += -a[1] * b[0] + a[0] * b[1];
	}
	auto s_c3a(double alpha, const double *a, const double *b, double *c_out) noexcept->void{
		c_out[0] += alpha * (-a[2] * b[1] + a[1] * b[2]);
		c_out[1] += alpha * (a[2] * b[0] - a[0] * b[2]);
		c_out[2] += alpha * (-a[1] * b[0] + a[0] * b[1]);
	}
	auto s_c3s(const double *a, const double *b, double *c_out) noexcept->void{
		c_out[0] -= -a[2] * b[1] + a[1] * b[2];
		c_out[1] -= a[2] * b[0] - a[0] * b[2];
		c_out[2] -= -a[1] * b[0] + a[0] * b[1];
	}
	auto s_cmf(const double *vs_in, double *cmf_out) noexcept->void{
		std::fill_n(cmf_out, 36, 0);

		cmf_out[6] = vs_in[5];
		cmf_out[12] = -vs_in[4];
		cmf_out[1] = -vs_in[5];
		cmf_out[13] = vs_in[3];
		cmf_out[2] = vs_in[4];
		cmf_out[8] = -vs_in[3];

		cmf_out[27] = vs_in[5];
		cmf_out[33] = -vs_in[4];
		cmf_out[22] = -vs_in[5];
		cmf_out[34] = vs_in[3];
		cmf_out[23] = vs_in[4];
		cmf_out[29] = -vs_in[3];

		cmf_out[24] = vs_in[2];
		cmf_out[30] = -vs_in[1];
		cmf_out[19] = -vs_in[2];
		cmf_out[31] = vs_in[0];
		cmf_out[20] = vs_in[1];
		cmf_out[26] = -vs_in[0];
	}
	auto s_cf(const double *vs, const double *fs, double* vfs_out) noexcept->void{
		s_c3(vs + 3, fs, vfs_out);
		s_c3(vs + 3, fs + 3, vfs_out + 3);
		s_c3a(vs, fs, vfs_out + 3);
	}
	auto s_cf(double alpha, const double *vs, const double *fs, double* vfs_out) noexcept->void{
		s_cf(vs, fs, vfs_out);
		s_nv(6, alpha, vfs_out);
	}
	auto s_cfi(const double *vs, const double *fs, double* vfs_out) noexcept->void{
		s_c3i(vs + 3, fs, vfs_out);
		s_c3i(vs + 3, fs + 3, vfs_out + 3);
		s_c3s(vs, fs, vfs_out + 3);
	}
	auto s_cfa(const double *vs, const double *fs, double* vfs_out) noexcept->void{
		s_c3a(vs + 3, fs, vfs_out);
		s_c3a(vs + 3, fs + 3, vfs_out + 3);
		s_c3a(vs, fs, vfs_out + 3);
	}
	auto s_cfa(double alpha, const double *vs, const double *fs, double* vfs_out) noexcept->void{
		s_c3a(alpha, vs + 3, fs, vfs_out);
		s_c3a(alpha, vs + 3, fs + 3, vfs_out + 3);
		s_c3a(alpha, vs, fs, vfs_out + 3);
	}
	auto s_cfs(const double *vs, const double *fs, double* vfs_out) noexcept->void{
		s_c3s(vs + 3, fs, vfs_out);
		s_c3s(vs + 3, fs + 3, vfs_out + 3);
		s_c3s(vs, fs, vfs_out + 3);
	}
	auto s_cmv(const double *vs_in, double *cmv_out) noexcept->void{
		std::fill_n(cmv_out, 36, 0);

		cmv_out[6] = vs_in[5];
		cmv_out[12] = -vs_in[4];
		cmv_out[1] = -vs_in[5];
		cmv_out[13] = vs_in[3];
		cmv_out[2] = vs_in[4];
		cmv_out[8] = -vs_in[3];

		cmv_out[27] = vs_in[5];
		cmv_out[33] = -vs_in[4];
		cmv_out[22] = -vs_in[5];
		cmv_out[34] = vs_in[3];
		cmv_out[23] = vs_in[4];
		cmv_out[29] = -vs_in[3];

		cmv_out[9] = vs_in[2];
		cmv_out[15] = -vs_in[1];
		cmv_out[4] = -vs_in[2];
		cmv_out[16] = vs_in[0];
		cmv_out[5] = vs_in[1];
		cmv_out[11] = -vs_in[0];
	}
	auto s_cv(const double *vs, const double *vs2, double* vvs_out) noexcept->void{
		s_c3(vs + 3, vs2, vvs_out);
		s_c3(vs + 3, vs2 + 3, vvs_out + 3);
		s_c3a(vs, vs2 + 3, vvs_out);
	}
	auto s_cv(double alpha, const double *vs, const double *vs2, double* vvs_out) noexcept->void{
		s_cv(vs, vs2, vvs_out);
		s_nv(6, alpha, vvs_out);
	}
	auto s_cvi(const double *vs, const double *vs2, double* vvs_out) noexcept->void{
		s_c3i(vs + 3, vs2, vvs_out);
		s_c3i(vs + 3, vs2 + 3, vvs_out + 3);
		s_c3s(vs, vs2 + 3, vvs_out);
	}
	auto s_cva(const double *vs, const double *vs2, double* vvs_out) noexcept->void{
		s_c3a(vs + 3, vs2, vvs_out);
		s_c3a(vs + 3, vs2 + 3, vvs_out + 3);
		s_c3a(vs, vs2 + 3, vvs_out);
	}
	auto s_cva(double alpha, const double *vs, const double *vs2, double* vvs_out) noexcept->void{
		s_c3a(alpha, vs + 3, vs2, vvs_out);
		s_c3a(alpha, vs + 3, vs2 + 3, vvs_out + 3);
		s_c3a(alpha, vs, vs2 + 3, vvs_out);
	}
	auto s_cvs(const double *vs, const double *vs2, double* vvs_out) noexcept->void{
		s_c3s(vs + 3, vs2, vvs_out);
		s_c3s(vs + 3, vs2 + 3, vvs_out + 3);
		s_c3s(vs, vs2 + 3, vvs_out);
	}

	auto s_tmf(const double *pm_in, double *tmf_out) noexcept->void{
		std::fill_n(tmf_out + 3, 3, 0);
		std::fill_n(tmf_out + 9, 3, 0);
		std::fill_n(tmf_out + 15, 3, 0);

		std::copy_n(&pm_in[0], 3, &tmf_out[0]);
		std::copy_n(&pm_in[4], 3, &tmf_out[6]);
		std::copy_n(&pm_in[8], 3, &tmf_out[12]);
		std::copy_n(&pm_in[0], 3, &tmf_out[21]);
		std::copy_n(&pm_in[4], 3, &tmf_out[27]);
		std::copy_n(&pm_in[8], 3, &tmf_out[33]);

		tmf_out[18] = -pm_in[11] * pm_in[4] + pm_in[7] * pm_in[8];
		tmf_out[24] = pm_in[11] * pm_in[0] - pm_in[3] * pm_in[8];
		tmf_out[30] = -pm_in[7] * pm_in[0] + pm_in[3] * pm_in[4];
		tmf_out[19] = -pm_in[11] * pm_in[5] + pm_in[7] * pm_in[9];
		tmf_out[25] = pm_in[11] * pm_in[1] - pm_in[3] * pm_in[9];
		tmf_out[31] = -pm_in[7] * pm_in[1] + pm_in[3] * pm_in[5];
		tmf_out[20] = -pm_in[11] * pm_in[6] + pm_in[7] * pm_in[10];
		tmf_out[26] = pm_in[11] * pm_in[2] - pm_in[3] * pm_in[10];
		tmf_out[32] = -pm_in[7] * pm_in[2] + pm_in[3] * pm_in[6];
	}
	auto s_tf(const double *pm, const double *fs, double *fs_out) noexcept->void{
		s_pm_dot_v3(pm, fs, fs_out);
		s_pm_dot_v3(pm, fs + 3, fs_out + 3);
		s_c3a(pm + 3, 4, fs_out, 1, fs_out + 3, 1);
	}
	auto s_tf(double alpha, const double *pm, const double *fs, double *fs_out) noexcept->void{
		s_tf(pm, fs, fs_out);
		s_nv(6, alpha, fs_out);
	}
	auto s_tfa(const double *pm, const double *fs, double *fs_out) noexcept->void{
		double tem[6];
		s_tf(pm, fs, tem);
		s_va(6, tem, fs_out);
	}
	auto s_tfa(double alpha, const double *pm, const double *fs, double *fs_out) noexcept->void{
		double tem[6];
		s_tf(pm, fs, tem);
		s_va(6, alpha, tem, fs_out);
	}
	auto s_inv_tf(const double *inv_pm, const double *fs, double *fs_out) noexcept->void{
		s_c3i(inv_pm + 3, 4, fs, 1, fs_out, 1);
		s_va(3, fs + 3, fs_out);

		s_inv_pm_dot_v3(inv_pm, fs_out, fs_out + 3);
		s_inv_pm_dot_v3(inv_pm, fs, fs_out);
	}
	auto s_inv_tf(double alpha, const double *inv_pm, const double *fs, double *fs_out) noexcept->void{
		s_inv_tf(inv_pm, fs, fs_out);
		s_nv(6, alpha, fs_out);
	}
	auto s_inv_tfa(const double *inv_pm, const double *fs, double *fs_out) noexcept->void{
		double tem[6];
		s_inv_tf(inv_pm, fs, tem);
		s_va(6, tem, fs_out);
	}
	auto s_inv_tfa(double alpha, const double *inv_pm, const double *fs, double *fs_out) noexcept->void{
		double tem[6];
		s_inv_tf(inv_pm, fs, tem);
		s_va(6, alpha, tem, fs_out);
	}
	auto s_tmv(const double *pm, double *tmv_out) noexcept->void{
		std::fill_n(tmv_out + 18, 3, 0);
		std::fill_n(tmv_out + 24, 3, 0);
		std::fill_n(tmv_out + 30, 3, 0);

		std::copy_n(&pm[0], 3, &tmv_out[0]);
		std::copy_n(&pm[4], 3, &tmv_out[6]);
		std::copy_n(&pm[8], 3, &tmv_out[12]);
		std::copy_n(&pm[0], 3, &tmv_out[21]);
		std::copy_n(&pm[4], 3, &tmv_out[27]);
		std::copy_n(&pm[8], 3, &tmv_out[33]);

		tmv_out[3] = -pm[11] * pm[4] + pm[7] * pm[8];
		tmv_out[9] = pm[11] * pm[0] - pm[3] * pm[8];
		tmv_out[15] = -pm[7] * pm[0] + pm[3] * pm[4];
		tmv_out[4] = -pm[11] * pm[5] + pm[7] * pm[9];
		tmv_out[10] = pm[11] * pm[1] - pm[3] * pm[9];
		tmv_out[16] = -pm[7] * pm[1] + pm[3] * pm[5];
		tmv_out[5] = -pm[11] * pm[6] + pm[7] * pm[10];
		tmv_out[11] = pm[11] * pm[2] - pm[3] * pm[10];
		tmv_out[17] = -pm[7] * pm[2] + pm[3] * pm[6];
	}
	auto s_tv(const double *pm, const double *vs, double *vs_out) noexcept->void{
		s_pm_dot_v3(pm, vs, vs_out);
		s_pm_dot_v3(pm, vs + 3, vs_out + 3);
		s_c3a(pm + 3, 4, vs_out + 3, 1, vs_out, 1);
	}
	auto s_tv(double alpha, const double *pm, const double *vs, double *vs_out) noexcept->void{
		s_tv(pm, vs, vs_out);
		s_nv(6, alpha, vs_out);
	}
	auto s_tva(const double *pm, const double *vs, double *vs_out) noexcept->void{
		double tem[6];
		s_tv(pm, vs, tem);
		s_va(6, tem, vs_out);
	}
	auto s_tva(double alpha, const double *pm, const double *vs, double *vs_out) noexcept->void{
		double tem[6];
		s_tv(pm, vs, tem);
		s_va(6, alpha, tem, vs_out);
	}
	auto s_inv_tv(const double *inv_pm, const double *vs, double *vs_out) noexcept->void{
		s_c3i(inv_pm + 3, 4, vs + 3, 1, vs_out + 3, 1);
		s_va(3, vs, vs_out + 3);

		s_inv_pm_dot_v3(inv_pm, vs_out + 3, vs_out);
		s_inv_pm_dot_v3(inv_pm, vs + 3, vs_out + 3);
	}
	auto s_inv_tv(double alpha, const double *inv_pm, const double *vs, double *vs_out) noexcept->void{
		s_inv_tv(inv_pm, vs, vs_out);
		s_nv(6, alpha, vs_out);
	}
	auto s_inv_tva(const double *inv_pm, const double *vs, double *vs_out) noexcept->void{
		double tem[6];
		s_inv_tv(inv_pm, vs, tem);
		s_va(6, tem, vs_out);
	}
	auto s_inv_tva(const double *inv_pm, const double *vs, Size vs_ld, double *vs_out, Size vs_out_ld) noexcept->void{
		double tem[6];
		s_inv_tv(inv_pm, vs, vs_ld, tem, 1);
		s_va(6, tem, 1, vs_out, vs_out_ld);
	}
	auto s_inv_tva(double alpha, const double *inv_pm, const double *vs, double *vs_out) noexcept->void{
		double tem[6];
		s_inv_tv(inv_pm, vs, tem);
		s_va(6, alpha, tem, vs_out);
	}
	auto s_inv_tva(double alpha, const double *inv_pm, const double *vs, Size vs_ld, double *vs_out, Size vs_out_ld) noexcept->void{
		double tem[6];
		s_inv_tv(inv_pm, vs, vs_ld, tem, 1);
		s_va(6, alpha, tem, 1, vs_out, vs_out_ld);
	}
}
