#ifndef ARIS_DYNAMIC_SCREW_H_
#define ARIS_DYNAMIC_SCREW_H_

#include <cmath>

#include "aris/dynamic/math_matrix.hpp"
#include "aris/dynamic/kinematics.hpp"

namespace aris::dynamic{
	///
	/// 符号定义: \n
	/// pp  :  3x1 点位置(position of point)  \n
	/// re  :  3x1 欧拉角(eula angle)         \n
	/// rq  :  4x1 四元数(quaternions)        \n
	/// rm  :  3x3 旋转矩阵(rotation matrix)  \n
	/// pe  :  6x1 点位置与欧拉角(position and eula angle)\n
	/// pq  :  7x1 点位置与四元数(position and quaternions)\n
	/// pm  :  4x4 位姿矩阵(pose matrix)\n
	/// ra  :  3x1 绕固定轴的旋转的指数积（rotation around axis, exponential product）
	/// ps  :  6x1 位移螺旋，[v;w]的速度螺旋转过theta角，也就是ps = [v;w]*theta
	///
	/// vp  :  3x1 线速度(velocity of point)\n
	/// we  :  3x1 欧拉角导数(omega in term of eula angle)\n
	/// wq  :  4x1 四元数导数(omega in term of quternions)\n
	/// wm  :  3x3 旋转矩阵导数(omega in term of rotation matrix)\n
	/// ve  :  6x1 线速度与欧拉角导数（velocity and omega in term of eula angle）\n
	/// vq  :  7x1 线速度与四元数导数(velocity and omega in term of quternions)\n
	/// vm  :  4x4 位姿矩阵导数(velocity in term of pose matrix)\n
	/// wa  :  3x1 角速度(omega)\n
	/// va  :  6x1 线速度与角速度(velocity and omega)\n
	/// vs  :  6x1 螺旋速度(velocity screw)\n
	///
	/// ap  :  3x1 线加速度(acceleration of point)\n
	/// xe  :  3x1 欧拉角导导数(alpha in term of eula angle)\n
	/// xq  :  4x1 四元数导导数(alpha in term of quternions)\n
	/// xm  :  3x3 旋转矩阵导导数(alpha in term of rotation matrix)\n
	/// ae  :  6x1 线加速度与欧拉角导导数(acceleration and alpha in term of eula angle)\n
	/// aq  :  7x1 线加速度与四元数导导数(acceleration and alpha in term of quternions)\n
	/// am  :  4x4 位姿矩阵导导数(acceleration in term of pose matrix)\n
	/// xa  :  3x1 角加速度(alpha, acceleration of angle)\n
	/// aa  :  6x1 线加速度与角加速度(acceleration and alpha)\n
	/// as  :  6x1 螺旋加速度(acceleration screw)\n
	///
	/// i3  :  3x3 惯量矩阵
	/// im  :  6x6 空间惯量矩阵
	/// iv  :  10x1 惯量矩阵向量[m, cx, cy, cz, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]

	using double3x3 = double[3][3];

	/// \brief 计算三维向量叉乘矩阵
	///
	/// 用来计算：cm_out = \n
	/// [ 0  -z   y \n
	///   z   0  -x \n
	///  -y   x   0 ]
	/// 
	///
	auto ARIS_API s_cm3(const double *a, double *cm_out) noexcept->void;
	/// \brief 计算三维向量叉乘
	///
	/// 用来计算：c = a x b
	///
	///
	auto ARIS_API s_c3(const double *a, const double *b, double *c_out) noexcept->void;
	/// \brief 计算三维向量叉乘
	///
	/// 用来计算：c = a x b
	///
	///
	template<typename AType, typename BType, typename CType>
	auto s_c3(const double *a, AType a_t, const double *b, BType b_t, double *c_out, CType c_t) noexcept->void{
		const Size a0{ 0 }, a1{ next_r(a0, a_t) }, a2{ next_r(a1, a_t) };
		const Size b0{ 0 }, b1{ next_r(b0, b_t) }, b2{ next_r(b1, b_t) };
		const Size c0{ 0 }, c1{ next_r(c0, c_t) }, c2{ next_r(c1, c_t) };

		c_out[c0] = -a[a2] * b[b1] + a[a1] * b[b2];
		c_out[c1] = a[a2] * b[b0] - a[a0] * b[b2];
		c_out[c2] = -a[a1] * b[b0] + a[a0] * b[b1];
	}
	/// \brief 计算三维向量叉乘
	///
	/// 用来计算：c = alpha * a x b
	///
	///
	auto ARIS_API s_c3(double alpha, const double *a, const double *b, double *c_out) noexcept->void;
	/// \brief 计算三维向量叉乘
	///
	/// 用来计算：c = alpha * a x b
	///
	///
	template<typename AType, typename BType, typename CType>
	auto s_c3(double alpha, const double *a, AType a_t, const double *b, BType b_t, double *c_out, CType c_t) noexcept->void{
		const Size a0{ 0 }, a1{ next_r(a0, a_t) }, a2{ next_r(a1, a_t) };
		const Size b0{ 0 }, b1{ next_r(b0, b_t) }, b2{ next_r(b1, b_t) };
		const Size c0{ 0 }, c1{ next_r(c0, c_t) }, c2{ next_r(c1, c_t) };

		c_out[c0] = alpha * (-a[a2] * b[b1] + a[a1] * b[b2]);
		c_out[c1] = alpha * (a[a2] * b[b0] - a[a0] * b[b2]);
		c_out[c2] = alpha * (-a[a1] * b[b0] + a[a0] * b[b1]);
	}
	/// \brief 计算三维向量叉乘
	///
	/// 用来计算：c = -a x b
	///
	///
	auto ARIS_API s_c3i(const double *a, const double *b, double *c_out) noexcept->void;
	/// \brief 计算三维向量叉乘
	///
	/// 用来计算：c = a x b
	///
	///
	template<typename AType, typename BType, typename CType>
	auto s_c3i(const double *a, AType a_t, const double *b, BType b_t, double *c_out, CType c_t) noexcept->void{
		const Size a0{ 0 }, a1{ next_r(a0, a_t) }, a2{ next_r(a1, a_t) };
		const Size b0{ 0 }, b1{ next_r(b0, b_t) }, b2{ next_r(b1, b_t) };
		const Size c0{ 0 }, c1{ next_r(c0, c_t) }, c2{ next_r(c1, c_t) };

		c_out[c0] = a[a2] * b[b1] - a[a1] * b[b2];
		c_out[c1] = -a[a2] * b[b0] + a[a0] * b[b2];
		c_out[c2] = a[a1] * b[b0] - a[a0] * b[b1];
	}
	/// \brief 计算三维向量叉乘
	///
	/// 用来计算：c = a x b + c
	///
	///
	auto ARIS_API s_c3a(const double *a, const double *b, double *c_out) noexcept->void;
	/// \brief 计算三维向量叉乘
	///
	/// 用来计算：c = a x b + c
	///
	///
	template<typename AType, typename BType, typename CType>
	auto s_c3a(const double *a, AType a_t, const double *b, BType b_t, double *c_out, CType c_t) noexcept->void{
		const Size a0{ 0 }, a1{ next_r(a0, a_t) }, a2{ next_r(a1, a_t) };
		const Size b0{ 0 }, b1{ next_r(b0, b_t) }, b2{ next_r(b1, b_t) };
		const Size c0{ 0 }, c1{ next_r(c0, c_t) }, c2{ next_r(c1, c_t) };

		c_out[c0] += -a[a2] * b[b1] + a[a1] * b[b2];
		c_out[c1] += a[a2] * b[b0] - a[a0] * b[b2];
		c_out[c2] += -a[a1] * b[b0] + a[a0] * b[b1];
	}
	/// \brief 计算三维向量叉乘
	///
	/// 用来计算：c = alpha * a x b + beta * c
	///
	///
	auto ARIS_API s_c3a(double alpha, const double *a, const double *b, double *c_out) noexcept->void;
	/// \brief 计算三维向量叉乘
	///
	/// 用来计算：c = alpha * a x b + beta * c
	///
	///
	template<typename AType, typename BType, typename CType>
	auto s_c3a(double alpha, const double *a, AType a_t, const double *b, BType b_t, double *c_out, CType c_t) noexcept->void{
		const Size a0{ 0 }, a1{ next_r(a0, a_t) }, a2{ next_r(a1, a_t) };
		const Size b0{ 0 }, b1{ next_r(b0, b_t) }, b2{ next_r(b1, b_t) };
		const Size c0{ 0 }, c1{ next_r(c0, c_t) }, c2{ next_r(c1, c_t) };

		c_out[c0] += alpha * (-a[a2] * b[b1] + a[a1] * b[b2]);
		c_out[c1] += alpha * (a[a2] * b[b0] - a[a0] * b[b2]);
		c_out[c2] += alpha * (-a[a1] * b[b0] + a[a0] * b[b1]);
	}
	/// \brief 计算三维向量叉乘
	///
	/// 用来计算：c = a x b + c
	///
	///
	auto ARIS_API s_c3s(const double *a, const double *b, double *c_out) noexcept->void;
	/// \brief 计算三维向量叉乘
	///
	/// 用来计算：c = a x b + c
	///
	///
	template<typename AType, typename BType, typename CType>
	auto s_c3s(const double *a, AType a_t, const double *b, BType b_t, double *c_out, CType c_t) noexcept->void{
		const Size a0{ 0 }, a1{ next_r(a0, a_t) }, a2{ next_r(a1, a_t) };
		const Size b0{ 0 }, b1{ next_r(b0, b_t) }, b2{ next_r(b1, b_t) };
		const Size c0{ 0 }, c1{ next_r(c0, c_t) }, c2{ next_r(c1, c_t) };

		c_out[c0] -= -a[a2] * b[b1] + a[a1] * b[b2];
		c_out[c1] -= a[a2] * b[b0] - a[a0] * b[b2];
		c_out[c2] -= -a[a1] * b[b0] + a[a0] * b[b1];
	}
	/// \brief 构造6x6的力叉乘矩阵
	///
	///  其中,cmf = [w x,  O; v x, w x]
	///
	///
	auto ARIS_API s_cmf(const double *vs, double *cmf_out) noexcept->void;
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vfs = vs xf fs
	///
	///
	auto ARIS_API s_cf(const double *vs, const double *fs, double* vfs_out) noexcept->void;
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vfs = vs xf fs
	///
	///
	template<typename VType, typename FType, typename VFType>
	auto s_cf(const double *vs, VType v_t, const double *fs, FType f_t, double* vfs_out, VFType vf_t) noexcept->void{
		const Size v3_pos{ at(3, v_t) }, f3_pos{ at(3, f_t) }, vf3_pos{ at(3, vf_t) };

		s_c3(vs + v3_pos, v_t, fs, f_t, vfs_out, vf_t);
		s_c3(vs + v3_pos, v_t, fs + f3_pos, f_t, vfs_out + vf3_pos, vf_t);
		s_c3a(vs, v_t, fs, f_t, vfs_out + vf3_pos, vf_t);
	}
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vfs = alpha * vs xf fs
	///
	///
	auto ARIS_API s_cf(double alpha, const double *vs, const double *fs, double* vfs_out) noexcept->void;
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vfs = alpha * vs xf fs
	///
	///
	template<typename VType, typename FType, typename VFType>
	auto s_cf(double alpha, const double *vs, VType v_t, const double *fs, FType f_t, double* vfs_out, VFType vf_t) noexcept->void{
		s_cf(vs, v_t, fs, f_t, vfs_out, vf_t);
		s_nv(6, alpha, vfs_out, vf_t);
	}
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vfs = vs xf fs
	///
	///
	auto ARIS_API s_cfi(const double *vs, const double *fs, double* vfs_out) noexcept->void;
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vfs = vs xf fs
	///
	///
	template<typename VType, typename FType, typename VFType>
	auto s_cfi(const double *vs, VType v_t, const double *fs, FType f_t, double* vfs_out, VFType vf_t) noexcept->void{
		const Size v3_pos{ at(3, v_t) }, f3_pos{ at(3, f_t) }, vf3_pos{ at(3, vf_t) };

		s_c3i(vs + v3_pos, v_t, fs, f_t, vfs_out, vf_t);
		s_c3i(vs + v3_pos, v_t, fs + f3_pos, f_t, vfs_out + vf3_pos, vf_t);
		s_c3s(vs, v_t, fs, f_t, vfs_out + vf3_pos, vf_t);
	}
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vfs = vs xf fs
	///
	///
	auto ARIS_API s_cfa(const double *vs, const double *fs, double* vfs_out) noexcept->void;
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vfs = vs xf fs
	///
	///
	template<typename VType, typename FType, typename VFType>
	auto s_cfa(const double *vs, VType v_t, const double *fs, FType f_t, double* vfs_out, VFType vf_t) noexcept->void{
		const Size v3_pos{ at(3, v_t) }, f3_pos{ at(3, f_t) }, vf3_pos{ at(3, vf_t) };

		s_c3a(vs + v3_pos, v_t, fs, f_t, vfs_out, vf_t);
		s_c3a(vs + v3_pos, v_t, fs + f3_pos, f_t, vfs_out + vf3_pos, vf_t);
		s_c3a(vs, v_t, fs, f_t, vfs_out + vf3_pos, vf_t);
	}
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vfs = alpha * vs xf fs + beta * vfs
	///
	///
	auto ARIS_API s_cfa(double alpha, const double *vs, const double *fs, double* vfs_out) noexcept->void;
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vfs = alpha * vs xf fs + beta * vfs
	///
	///
	template<typename VType, typename FType, typename VFType>
	auto s_cfa(double alpha, const double *vs, VType v_t, const double *fs, FType f_t, double* vfs_out, VFType vf_t) noexcept->void{
		const Size v3_pos{ at(3, v_t) }, f3_pos{ at(3, f_t) }, vf3_pos{ at(3, vf_t) };

		s_c3a(alpha, vs + v3_pos, v_t, fs, f_t, vfs_out, vf_t);
		s_c3a(alpha, vs + v3_pos, v_t, fs + f3_pos, f_t, vfs_out + vf3_pos, vf_t);
		s_c3a(alpha, vs, v_t, fs, f_t, vfs_out + vf3_pos, vf_t);
	}
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vfs = vs xf fs
	///
	///
	auto ARIS_API s_cfs(const double *vs, const double *fs, double* vfs_out) noexcept->void;
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vfs = vs xf fs
	///
	///
	template<typename VType, typename FType, typename VFType>
	auto s_cfs(const double *vs, VType v_t, const double *fs, FType f_t, double* vfs_out, VFType vf_t) noexcept->void{
		const Size v3_pos{ at(3, v_t) }, f3_pos{ at(3, f_t) }, vf3_pos{ at(3, vf_t) };

		s_c3s(vs + v3_pos, v_t, fs, f_t, vfs_out, vf_t);
		s_c3s(vs + v3_pos, v_t, fs + f3_pos, f_t, vfs_out + vf3_pos, vf_t);
		s_c3s(vs, v_t, fs, f_t, vfs_out + vf3_pos, vf_t);
	}
	/// \brief 构造6x6的速度叉乘矩阵
	///
	///  其中,cmv = \n
	///  [w x,  v x \n
	///   O  ,  w x] \n
	///
	///
	auto ARIS_API s_cmv(const double *vs, double *cmv_out) noexcept->void;
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vvs = vs xv vs2
	///
	///
	auto ARIS_API s_cv(const double *vs, const double *vs2, double* vvs_out) noexcept->void;
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vvs = vs xv vs2
	///
	///
	template<typename VType, typename V2Type, typename VVType>
	auto s_cv(const double *vs, VType v_t, const double *vs2, V2Type v2_t, double* vvs_out, VVType vv_t) noexcept->void{
		const Size v3_pos{ at(3, v_t) }, v23_pos{ at(3, v2_t) }, vv3_pos{ at(3, vv_t) };

		s_c3(vs + v3_pos, v_t, vs2, v2_t, vvs_out, vv_t);
		s_c3(vs + v3_pos, v_t, vs2 + v23_pos, v2_t, vvs_out + vv3_pos, vv_t);
		s_c3a(vs, v_t, vs2 + v23_pos, v2_t, vvs_out, vv_t);
	}
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vvs = alpha * vs xv vs2
	///
	///
	auto ARIS_API s_cv(double alpha, const double *vs, const double *vs2, double* vvs_out) noexcept->void;
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vvs = alpha * vs xv vs2
	///
	///
	template<typename VType, typename V2Type, typename VVType>
	auto s_cv(double alpha, const double *vs, VType v_t, const double *vs2, V2Type v2_t, double* vvs_out, VVType vv_t) noexcept->void{
		s_cv(vs, v_t, vs2, v2_t, vvs_out, vv_t);
		s_nv(6, alpha, vvs_out, vv_t);
	}
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vvs = vs xv vs2
	///
	///
	auto ARIS_API s_cvi(const double *vs, const double *vs2, double* vvs_out) noexcept->void;
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vvs = vs xv vs2
	///
	///
	template<typename VType, typename V2Type, typename VVType>
	auto s_cvi(const double *vs, VType v_t, const double *vs2, V2Type v2_t, double* vvs_out, VVType vv_t) noexcept->void{
		const Size v3_pos{ at(3, v_t) }, v23_pos{ at(3, v2_t) }, vv3_pos{ at(3, vv_t) };

		s_c3i(vs + v3_pos, v_t, vs2, v2_t, vvs_out, vv_t);
		s_c3i(vs + v3_pos, v_t, vs2 + v23_pos, v2_t, vvs_out + vv3_pos, vv_t);
		s_c3s(vs, v_t, vs2 + v23_pos, v2_t, vvs_out, vv_t);
	}
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vvs = vs xv vs2 + vvs
	///
	///
	auto ARIS_API s_cva(const double *vs, const double *vs2, double* vvs_out) noexcept->void;
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vvs = vs xv vs2 + vvs
	///
	///
	template<typename VType, typename V2Type, typename VVType>
	auto s_cva(const double *vs, VType v_t, const double *vs2, V2Type v2_t, double* vvs_out, VVType vv_t) noexcept->void{
		const Size v3_pos{ at(3, v_t) }, v23_pos{ at(3, v2_t) }, vv3_pos{ at(3, vv_t) };

		s_c3a(vs + v3_pos, v_t, vs2, v2_t, vvs_out, vv_t);
		s_c3a(vs + v3_pos, v_t, vs2 + v23_pos, v2_t, vvs_out + vv3_pos, vv_t);
		s_c3a(vs, v_t, vs2 + v23_pos, v2_t, vvs_out, vv_t);
	}
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vvs = alpha * vs xv vs2 + beta * vvs
	///
	///
	auto ARIS_API s_cva(double alpha, const double *vs, const double *vs2, double* vvs_out) noexcept->void;
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vvs = alpha * vs xv vs2 + beta * vvs
	///
	///
	template<typename VType, typename V2Type, typename VVType>
	auto s_cva(double alpha, const double *vs, VType v_t, const double *vs2, V2Type v2_t, double* vvs_out, VVType vv_t) noexcept->void{
		const Size v3_pos{ at(3, v_t) }, v23_pos{ at(3, v2_t) }, vv3_pos{ at(3, vv_t) };

		s_c3a(alpha, vs + v3_pos, v_t, vs2, v2_t, vvs_out, vv_t);
		s_c3a(alpha, vs + v3_pos, v_t, vs2 + v23_pos, v2_t, vvs_out + vv3_pos, vv_t);
		s_c3a(alpha, vs, v_t, vs2 + v23_pos, v2_t, vvs_out, vv_t);
	}
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vvs = vs xv vs2 + vvs
	///
	///
	auto ARIS_API s_cvs(const double *vs, const double *vs2, double* vvs_out) noexcept->void;
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vvs = vs xv vs2 + vvs
	///
	///
	template<typename VType, typename V2Type, typename VVType>
	auto s_cvs(const double *vs, VType v_t, const double *vs2, V2Type v2_t, double* vvs_out, VVType vv_t) noexcept->void{
		const Size v3_pos{ at(3, v_t) }, v23_pos{ at(3, v2_t) }, vv3_pos{ at(3, vv_t) };

		s_c3s(vs + v3_pos, v_t, vs2, v2_t, vvs_out, vv_t);
		s_c3s(vs + v3_pos, v_t, vs2 + v23_pos, v2_t, vvs_out + vv3_pos, vv_t);
		s_c3s(vs, v_t, vs2 + v23_pos, v2_t, vvs_out, vv_t);
	}
	/// \brief 计算三维向量叉乘
	///
	/// 用来计算：c_mtx_out = a x b_mtx
	///
	///
	auto inline s_c3_n(Size n, const double *a, const double *b_mtx, double *c_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_c3(a, 1, b_mtx + i, n, c_mtx_out + i, n); }
	/// \brief 计算三维向量叉乘
	///
	/// 用来计算：c_mtx_out = a x b_mtx
	///
	///
	template<typename AType, typename BType, typename CType>
	auto inline s_c3_n(Size n, const double *a, AType a_t, const double *b_mtx, BType b_t, double *c_mtx_out, CType c_t) noexcept->void{
		for (Size i(-1), bid(0), cid(0); ++i < n; bid = next_c(bid, b_t), cid = next_c(cid, c_t))
			s_c3(a, a_t, b_mtx + bid, b_t, c_mtx_out + cid, c_t);
	}
	/// \brief 计算三维向量叉乘
	///
	/// 用来计算：c_mtx_out = alpha * a x b_mtx
	///
	///
	auto inline s_c3_n(Size n, double alpha, const double *a, const double *b_mtx, double *c_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_c3(alpha, a, 1, b_mtx + i, n, c_mtx_out + i, n); }
	/// \brief 计算三维向量叉乘
	///
	/// 用来计算：c_mtx_out = alpha * a x b_mtx
	///
	///
	template<typename AType, typename BType, typename CType>
	auto inline s_c3_n(Size n, double alpha, const double *a, AType a_t, const double *b_mtx, BType b_t, double *c_mtx_out, CType c_t) noexcept->void{
		for (Size i(-1), bid(0), cid(0); ++i < n; bid = next_c(bid, b_t), cid = next_c(cid, c_t))
			s_c3(alpha, a, a_t, b_mtx + bid, b_t, c_mtx_out + cid, c_t);
	}
	/// \brief 计算三维向量叉乘
	///
	/// 用来计算：c_mtx_out = a x b_mtx
	///
	///
	auto inline s_c3i_n(Size n, const double *a, const double *b_mtx, double *c_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_c3i(a, 1, b_mtx + i, n, c_mtx_out + i, n); }
	/// \brief 计算三维向量叉乘
	///
	/// 用来计算：c_mtx_out = a x b_mtx
	///
	///
	template<typename AType, typename BType, typename CType>
	auto inline s_c3i_n(Size n, const double *a, AType a_t, const double *b_mtx, BType b_t, double *c_mtx_out, CType c_t) noexcept->void{
		for (Size i(-1), bid(0), cid(0); ++i < n; bid = next_c(bid, b_t), cid = next_c(cid, c_t))
			s_c3i(a, a_t, b_mtx + bid, b_t, c_mtx_out + cid, c_t);
	}
	/// \brief 计算三维向量叉乘
	///
	/// 用来计算：c_mtx_out = alpha * a x b_mtx + beta * c_mtx_out
	///
	///
	auto inline s_c3a_n(Size n, const double *a, const double *b_mtx, double *c_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_c3a(a, 1, b_mtx + i, n, c_mtx_out + i, n); }
	/// \brief 计算三维向量叉乘
	///
	/// 用来计算：c_mtx_out = alpha * a x b_mtx + beta * c_mtx_out
	///
	///
	template<typename AType, typename BType, typename CType>
	auto inline s_c3a_n(Size n, const double *a, AType a_t, const double *b_mtx, BType b_t, double *c_mtx_out, CType c_t) noexcept->void{
		for (Size i(-1), bid(0), cid(0); ++i < n; bid = next_c(bid, b_t), cid = next_c(cid, c_t))
			s_c3a(a, a_t, b_mtx + bid, b_t, c_mtx_out + cid, c_t);
	}
	/// \brief 计算三维向量叉乘
	///
	/// 用来计算：c_mtx_out = alpha * a x b_mtx + beta * c_mtx_out
	///
	///
	auto inline s_c3a_n(Size n, double alpha, const double *a, const double *b_mtx, double *c_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_c3a(alpha, a, 1, b_mtx + i, n, c_mtx_out + i, n); }
	/// \brief 计算三维向量叉乘
	///
	/// 用来计算：c_mtx_out = alpha * a x b_mtx + beta * c_mtx_out
	///
	///
	template<typename AType, typename BType, typename CType>
	auto inline s_c3a_n(Size n, double alpha, const double *a, AType a_t, const double *b_mtx, BType b_t, double *c_mtx_out, CType c_t) noexcept->void{
		for (Size i(-1), bid(0), cid(0); ++i < n; bid = next_c(bid, b_t), cid = next_c(cid, c_t))
			s_c3a(alpha, a, a_t, b_mtx + bid, b_t, c_mtx_out + cid, c_t);
	}
	/// \brief 计算三维向量叉乘
	///
	/// 用来计算：c_mtx_out = alpha * a x b_mtx + beta * c_mtx_out
	///
	///
	auto inline s_c3s_n(Size n, const double *a, const double *b_mtx, double *c_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_c3s(a, 1, b_mtx + i, n, c_mtx_out + i, n); }
	/// \brief 计算三维向量叉乘
	///
	/// 用来计算：c_mtx_out = alpha * a x b_mtx + beta * c_mtx_out
	///
	///
	template<typename AType, typename BType, typename CType>
	auto inline s_c3s_n(Size n, const double *a, AType a_t, const double *b_mtx, BType b_t, double *c_mtx_out, CType c_t) noexcept->void{
		for (Size i(-1), bid(0), cid(0); ++i < n; bid = next_c(bid, b_t), cid = next_c(cid, c_t))
			s_c3s(a, a_t, b_mtx + bid, b_t, c_mtx_out + cid, c_t);
	}
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vfs = vs xf fs
	///
	///
	auto inline s_cf_n(Size n, const double *vs, const double *fs_mtx, double* vfs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_cf(vs, 1, fs_mtx + i, n, vfs_mtx_out + i, n); }
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vfs = vs xf fs
	///
	///
	template<typename VType, typename FType, typename VFType>
	auto inline s_cf_n(Size n, const double *vs, VType v_t, const double *fs_mtx, FType f_t, double* vfs_mtx_out, VFType vf_t) noexcept->void{
		for (Size i(-1), fid(0), vfid(0); ++i < n; fid = next_c(fid, f_t), vfid = next_c(vfid, vf_t))
			s_cf(vs, v_t, fs_mtx + fid, f_t, vfs_mtx_out + vfid, vf_t);
	}
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vfs = alpha * vs xf fs
	///
	///
	auto inline s_cf_n(Size n, double alpha, const double *vs, const double *fs_mtx, double* vfs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_cf(alpha, vs, 1, fs_mtx + i, n, vfs_mtx_out + i, n); }
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vfs = alpha * vs xf fs
	///
	///
	template<typename VType, typename FType, typename VFType>
	auto inline s_cf_n(Size n, double alpha, const double *vs, VType v_t, const double *fs_mtx, FType f_t, double* vfs_mtx_out, VFType vf_t) noexcept->void{
		for (Size i(-1), fid(0), vfid(0); ++i < n; fid = next_c(fid, f_t), vfid = next_c(vfid, vf_t))
			s_cf(alpha, vs, v_t, fs_mtx + fid, f_t, vfs_mtx_out + vfid, vf_t);
	}
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vfs = vs xf fs
	///
	///
	auto inline s_cfi_n(Size n, const double *vs, const double *fs_mtx, double* vfs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_cfi(vs, 1, fs_mtx + i, n, vfs_mtx_out + i, n); }
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vfs = vs xf fs
	///
	///
	template<typename VType, typename FType, typename VFType>
	auto inline s_cfi_n(Size n, const double *vs, VType v_t, const double *fs_mtx, FType f_t, double* vfs_mtx_out, VFType vf_t) noexcept->void{
		for (Size i(-1), fid(0), vfid(0); ++i < n; fid = next_c(fid, f_t), vfid = next_c(vfid, vf_t))
			s_cfi(vs, v_t, fs_mtx + fid, f_t, vfs_mtx_out + vfid, vf_t);
	}
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vfs = vs xf fs
	///
	///
	auto inline s_cfa_n(Size n, const double *vs, const double *fs_mtx, double* vfs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_cfa(vs, 1, fs_mtx + i, n, vfs_mtx_out + i, n); }
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vfs = vs xf fs
	///
	///
	template<typename VType, typename FType, typename VFType>
	auto inline s_cfa_n(Size n, const double *vs, VType v_t, const double *fs_mtx, FType f_t, double* vfs_mtx_out, VFType vf_t) noexcept->void{
		for (Size i(-1), fid(0), vfid(0); ++i < n; fid = next_c(fid, f_t), vfid = next_c(vfid, vf_t))
			s_cfa(vs, v_t, fs_mtx + fid, f_t, vfs_mtx_out + vfid, vf_t);
	}
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vfs = alpha * vs xf fs + beta * vfs
	///
	///
	auto inline s_cfa_n(Size n, double alpha, const double *vs, const double *fs_mtx, double* vfs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_cfa(alpha, vs, 1, fs_mtx + i, n, vfs_mtx_out + i, n); }
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vfs = alpha * vs xf fs + beta * vfs
	///
	///
	template<typename VType, typename FType, typename VFType>
	auto inline s_cfa_n(Size n, double alpha, const double *vs, VType v_t, const double *fs_mtx, FType f_t, double* vfs_mtx_out, VFType vf_t) noexcept->void{
		for (Size i(-1), fid(0), vfid(0); ++i < n; fid = next_c(fid, f_t), vfid = next_c(vfid, vf_t))
			s_cfa(alpha, vs, v_t, fs_mtx + fid, f_t, vfs_mtx_out + vfid, vf_t);
	}
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vfs = vs xf fs
	///
	///
	auto inline s_cfs_n(Size n, const double *vs, const double *fs_mtx, double* vfs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_cfs(vs, 1, fs_mtx + i, n, vfs_mtx_out + i, n); }
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vfs = vs xf fs
	///
	///
	template<typename VType, typename FType, typename VFType>
	auto inline s_cfs_n(Size n, const double *vs, VType v_t, const double *fs_mtx, FType f_t, double* vfs_mtx_out, VFType vf_t) noexcept->void{
		for (Size i(-1), fid(0), vfid(0); ++i < n; fid = next_c(fid, f_t), vfid = next_c(vfid, vf_t))
			s_cfs(vs, v_t, fs_mtx + fid, f_t, vfs_mtx_out + vfid, vf_t);
	}
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vvs = vs xv vs2
	///
	///
	auto inline s_cv_n(Size n, const double *vs, const double *vs_mtx, double* vvs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_cv(vs, 1, vs_mtx + i, n, vvs_mtx_out + i, n); }
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vvs = vs xv vs2
	///
	///
	template<typename VType, typename V2Type, typename VVType>
	auto inline s_cv_n(Size n, const double *vs, VType v_t, const double *vs_mtx, V2Type v2_t, double* vvs_mtx_out, VVType vv_t) noexcept->void{
		for (Size i(-1), vid(0), vvid(0); ++i < n; vid = next_c(vid, v2_t), vvid = next_c(vvid, vv_t))
			s_cv(vs, v_t, vs_mtx + vid, v2_t, vvs_mtx_out + vvid, vv_t);
	}
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vvs = alpha * vs xv vs2
	///
	///
	auto inline s_cv_n(Size n, double alpha, const double *vs, const double *vs_mtx, double* vvs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_cv(alpha, vs, 1, vs_mtx + i, n, vvs_mtx_out + i, n); }
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vvs = alpha * vs xv vs2
	///
	///
	template<typename VType, typename V2Type, typename VVType>
	auto inline s_cv_n(Size n, double alpha, const double *vs, VType v_t, const double *vs_mtx, V2Type v2_t, double* vvs_mtx_out, VVType vv_t) noexcept->void{
		for (Size i(-1), vid(0), vvid(0); ++i < n; vid = next_c(vid, v2_t), vvid = next_c(vvid, vv_t))
			s_cv(alpha, vs, v_t, vs_mtx + vid, v2_t, vvs_mtx_out + vvid, vv_t);
	}
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vvs = vs xv vs2
	///
	///
	auto inline s_cvi_n(Size n, const double *vs, const double *vs_mtx, double* vvs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_cvi(vs, 1, vs_mtx + i, n, vvs_mtx_out + i, n); }
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vvs = vs xv vs2
	///
	///
	template<typename VType, typename V2Type, typename VVType>
	auto inline s_cvi_n(Size n, const double *vs, VType v_t, const double *vs_mtx, V2Type v2_t, double* vvs_mtx_out, VVType vv_t) noexcept->void{
		for (Size i(-1), vid(0), vvid(0); ++i < n; vid = next_c(vid, v2_t), vvid = next_c(vvid, vv_t))
			s_cvi(vs, v_t, vs_mtx + vid, v2_t, vvs_mtx_out + vvid, vv_t);
	}
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vvs = vs xv vs2 + vvs
	///
	///
	auto inline s_cva_n(Size n, const double *vs, const double *vs_mtx, double* vvs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_cva(vs, 1, vs_mtx + i, n, vvs_mtx_out + i, n); }
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vvs = vs xv vs2 + vvs
	///
	///
	template<typename VType, typename V2Type, typename VVType>
	auto inline s_cva_n(Size n, const double *vs, VType v_t, const double *vs_mtx, V2Type v2_t, double* vvs_mtx_out, VVType vv_t) noexcept->void{
		for (Size i(-1), vid(0), vvid(0); ++i < n; vid = next_c(vid, v2_t), vvid = next_c(vvid, vv_t))
			s_cva(vs, v_t, vs_mtx + vid, v2_t, vvs_mtx_out + vvid, vv_t);
	}
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vvs = alpha * vs xv vs2 + beta * vvs
	///
	///
	auto inline s_cva_n(Size n, double alpha, const double *vs, const double *vs_mtx, double* vvs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_cva(alpha, vs, 1, vs_mtx + i, n, vvs_mtx_out + i, n); }
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vvs = alpha * vs xv vs2 + beta * vvs
	///
	///
	template<typename VType, typename V2Type, typename VVType>
	auto inline s_cva_n(Size n, double alpha, const double *vs, VType v_t, const double *vs_mtx, V2Type v2_t, double* vvs_mtx_out, VVType vv_t) noexcept->void{
		for (Size i(-1), vid(0), vvid(0); ++i < n; vid = next_c(vid, v2_t), vvid = next_c(vvid, vv_t))
			s_cva(alpha, vs, v_t, vs_mtx + vid, v2_t, vvs_mtx_out + vvid, vv_t);
	}
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vvs = vs xv vs2 + vvs
	///
	///
	auto inline s_cvs_n(Size n, const double *vs, const double *vs_mtx, double* vvs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_cvs(vs, 1, vs_mtx + i, n, vvs_mtx_out + i, n); }
	/// \brief 计算六维向量叉乘
	///
	/// 用来计算：vvs = vs xv vs2 + vvs
	///
	///
	template<typename VType, typename V2Type, typename VVType>
	auto inline s_cvs_n(Size n, const double *vs, VType v_t, const double *vs_mtx, V2Type v2_t, double* vvs_mtx_out, VVType vv_t) noexcept->void{
		for (Size i(-1), vid(0), vvid(0); ++i < n; vid = next_c(vid, v2_t), vvid = next_c(vvid, vv_t))
			s_cvs(vs, v_t, vs_mtx + vid, v2_t, vvs_mtx_out + vvid, vv_t);
	}
	/// \brief 构造6x6的力转换矩阵
	///
	///  其中,tmf = [rm (3x3),  pp x rm (3x3); O (3x3), rm (3x3)]
	///
	///
	auto ARIS_API s_tmf(const double *pm, double *tmf_out) noexcept->void;
	/// \brief 根据位姿矩阵转换六维力向量
	///
	/// 等同于： fs_out = tmf(pm) * fs
	///
	///
	auto ARIS_API s_tf(const double *pm, const double *fs, double *fs_out) noexcept->void;
	/// \brief 根据位姿矩阵转换六维力向量
	///
	/// 等同于： fs_out = tmf(pm) * fs
	///
	///
	template<typename F1Type, typename F2Type>
	auto s_tf(const double *pm, const double *fs, F1Type f1_t, double *fs_out, F2Type f2_t) noexcept->void{
		const Size f13_pos{ at(3, f1_t) }, f23_pos{ at(3,f2_t) };

		s_pm_dot_v3(pm, fs, f1_t, fs_out, f2_t);
		s_pm_dot_v3(pm, fs + f13_pos, f1_t, fs_out + f23_pos, f2_t);
		s_c3a(pm + 3, 4, fs_out, f2_t, fs_out + f23_pos, f2_t);
	}
	/// \brief 根据位姿矩阵转换六维力向量
	///
	/// 等同于： fs_out = alpha * tmf(pm) * fs
	///
	///
	auto ARIS_API s_tf(double alpha, const double *pm, const double *fs, double *fs_out) noexcept->void;
	/// \brief 根据位姿矩阵转换六维力向量
	///
	/// 等同于： fs_out = alpha * tmf(pm) * fs
	///
	///
	template<typename F1Type, typename F2Type>
	auto s_tf(double alpha, const double *pm, const double *fs, F1Type f1_t, double *fs_out, F2Type f2_t) noexcept->void{
		s_tf(pm, fs, f1_t, fs_out, f2_t);
		s_nv(6, alpha, fs_out, f2_t);
	}
	/// \brief 根据位姿矩阵转换六维力向量
	///
	/// 等同于： fs_out = tmf(pm) * fs + fs_out
	///
	///
	auto ARIS_API s_tfa(const double *pm, const double *fs, double *fs_out) noexcept->void;
	/// \brief 根据位姿矩阵转换六维力向量
	///
	/// 等同于： fs_out = tmf(pm) * fs + fs_out
	///
	///
	template<typename F1Type, typename F2Type>
	auto s_tfa(const double *pm, const double *fs, F1Type f1_t, double *fs_out, F2Type f2_t) noexcept->void{
		double tem[6];
		s_tf(pm, fs, f1_t, tem, 1);
		s_va(6, tem, 1, fs_out, f2_t);
	}
	/// \brief 根据位姿矩阵转换六维力向量
	///
	/// 等同于： fs_out = alpha * tmf(pm) * fs + beta * fs_out
	///
	///
	auto ARIS_API s_tfa(double alpha, const double *pm, const double *fs, double *fs_out) noexcept->void;
	/// \brief 根据位姿矩阵转换六维力向量
	///
	/// 等同于： fs_out = alpha * tmf(pm) * fs + beta * fs_out
	///
	///
	template<typename F1Type, typename F2Type>
	auto s_tfa(double alpha, const double *pm, const double *fs, F1Type f1_t, double *fs_out, F2Type f2_t) noexcept->void{
		double tem[6];
		s_tf(pm, fs, f1_t, tem, 1);
		s_va(6, alpha, tem, 1, fs_out, f2_t);
	}
	/// \brief 根据位姿矩阵的逆矩阵转换六维力向量
	///
	/// 等同于： fs_out = tmv(pm^-1) * fs
	///
	///
	auto ARIS_API s_inv_tf(const double *inv_pm, const double *fs, double *fs_out) noexcept->void;
	/// \brief 根据位姿矩阵的逆矩阵转换六维力向量
	///
	/// 等同于： fs_out = tmv(pm^-1) * fs
	///
	///
	template<typename F1Type, typename F2Type>
	auto s_inv_tf(const double *inv_pm, const double *fs, F1Type f1_t, double *fs_out, F2Type f2_t) noexcept->void{
		s_c3i(inv_pm + 3, 4, fs, f1_t, fs_out, f2_t);
		s_va(3, fs + at(3, f1_t), f1_t, fs_out, f2_t);

		s_inv_pm_dot_v3(inv_pm, fs_out, f2_t, fs_out + at(3, f2_t), f2_t);
		s_inv_pm_dot_v3(inv_pm, fs, f1_t, fs_out, f2_t);
	}
	/// \brief 根据位姿矩阵的逆矩阵转换六维力向量
	///
	/// 等同于： fs_out = alpha * tmv(pm^-1) * fs
	///
	///
	auto ARIS_API s_inv_tf(double alpha, const double *inv_pm, const double *fs, double *fs_out) noexcept->void;
	/// \brief 根据位姿矩阵的逆矩阵转换六维力向量
	///
	/// 等同于： fs_out = alpha * tmv(pm^-1) * fs
	///
	///
	template<typename F1Type, typename F2Type>
	auto s_inv_tf(double alpha, const double *inv_pm, const double *fs, F1Type f1_t, double *fs_out, F2Type f2_t) noexcept->void{
		s_inv_tf(inv_pm, fs, f1_t, fs_out, f2_t);
		s_nv(6, alpha, fs_out, f2_t);
	}
	/// \brief 根据位姿矩阵转换六维力向量
	///
	/// 等同于： fs_out = tmf(pm^-1) * fs + fs_out
	///
	///
	auto ARIS_API s_inv_tfa(const double *inv_pm, const double *fs, double *fs_out) noexcept->void;
	/// \brief 根据位姿矩阵转换六维力向量
	///
	/// 等同于： fs_out = tmf(pm^-1) * fs + fs_out
	///
	///
	template<typename F1Type, typename F2Type>
	auto s_inv_tfa(const double *inv_pm, const double *fs, F1Type f1_t, double *fs_out, F2Type f2_t) noexcept->void{
		double tem[6];
		s_inv_tf(inv_pm, fs, f1_t, tem, 1);
		s_va(6, tem, 1, fs_out, f2_t);
	}
	/// \brief 根据位姿矩阵转换六维力向量
	///
	/// 等同于： fs_out = alpha * tmf(pm^-1) * fs + beta * fs_out
	///
	///
	auto ARIS_API s_inv_tfa(double alpha, const double *inv_pm, const double *fs, double *fs_out) noexcept->void;
	/// \brief 根据位姿矩阵转换六维力向量
	///
	/// 等同于： fs_out = alpha * tmf(pm^-1) * fs + beta * fs_out
	///
	///
	template<typename F1Type, typename F2Type>
	auto s_inv_tfa(double alpha, const double *inv_pm, const double *fs, F1Type f1_t, double *fs_out, F2Type f2_t) noexcept->void{
		double tem[6];
		s_inv_tf(inv_pm, fs, f1_t, tem, 1);
		s_va(6, alpha, tem, 1, fs_out, f2_t);
	}
	/// \brief 构造6x6的速度转换矩阵
	///
	///  其中,tmv = [rm (3x3),  O (3x3); pp x rm (3x3), rm (3x3)]
	///
	///
	auto ARIS_API s_tmv(const double *pm, double *tmv_out) noexcept->void;
	/// \brief 根据位姿矩阵转换六维速度向量
	///
	/// 等同于： vec_out = tmv(pm_in) * vs_in
	///
	///
	auto ARIS_API s_tv(const double *pm, const double *vs, double *vs_out) noexcept->void;
	/// \brief 根据位姿矩阵转换六维速度向量
	///
	/// 等同于： vs_out = tmv(pm) * vs
	///
	///
	template<typename V1Type, typename V2Type>
	auto s_tv(const double *pm, const double *vs, V1Type v1_t, double *vs_out, V2Type v2_t) noexcept->void{
		s_pm_dot_v3(pm, vs, v1_t, vs_out, v2_t);
		s_pm_dot_v3(pm, vs + at(3, v1_t), v1_t, vs_out + at(3, v2_t), v2_t);
		s_c3a(pm + 3, 4, vs_out + at(3, v2_t), v2_t, vs_out, v2_t);
	}
	/// \brief 根据位姿矩阵转换六维速度向量
	///
	/// 等同于： vs_out = alpha * tmv(pm) * vs
	///
	///
	auto ARIS_API s_tv(double alpha, const double *pm, const double *vs, double *vs_out) noexcept->void;
	/// \brief 根据位姿矩阵转换六维速度向量
	///
	/// 等同于： vs_out = alpha * tmv(pm) * vs
	///
	///
	template<typename V1Type, typename V2Type>
	auto s_tv(double alpha, const double *pm, const double *vs, V1Type v1_t, double *vs_out, V2Type v2_t) noexcept->void{
		s_tv(pm, vs, v1_t, vs_out, v2_t);
		s_nv(6, alpha, vs_out, v2_t);
	}
	/// \brief 根据位姿矩阵转换六维速度向量
	///
	/// 等同于： vs_out = tmv(pm) * vs + vs_out
	///
	///
	auto ARIS_API s_tva(const double *pm, const double *vs, double *vs_out) noexcept->void;
	/// \brief 根据位姿矩阵转换六维速度向量
	///
	/// 等同于： vs_out = tmv(pm) * vs + vs_out
	///
	///
	template<typename V1Type, typename V2Type>
	auto s_tva(const double *pm, const double *vs, V1Type v1_t, double *vs_out, V2Type v2_t) noexcept->void{
		double tem[6];
		s_tv(pm, vs, v1_t, tem, 1);
		s_va(6, tem, 1, vs_out, v2_t);
	}
	/// \brief 根据位姿矩阵转换六维速度向量
	///
	/// 等同于： vs_out = alpha * tmv(pm) * vs + beta * vs_out
	///
	///
	auto ARIS_API s_tva(double alpha, const double *pm, const double *vs, double *vs_out) noexcept->void;
	/// \brief 根据位姿矩阵转换六维速度向量
	///
	/// 等同于： vs_out = alpha * tmv(pm) * vs + beta * vs_out
	///
	///
	template<typename V1Type, typename V2Type>
	auto s_tva(double alpha, const double *pm, const double *vs, V1Type v1_t, double *vs_out, V2Type v2_t) noexcept->void{
		double tem[6];
		s_tv(pm, vs, v1_t, tem, 1);
		s_va(6, alpha, tem, 1, vs_out, v2_t);
	}
	/// \brief 根据位姿矩阵的逆矩阵转换六维速度向量
	///
	/// 等同于： vs_out = tmv(pm^-1) * vs
	///
	///
	auto ARIS_API s_inv_tv(const double *inv_pm, const double *vs, double *vs_out) noexcept->void;
	/// \brief 根据位姿矩阵的逆矩阵转换六维速度向量
	///
	/// 等同于： vs_out = tmv(pm^-1) * vs
	///
	///
	template<typename V1Type, typename V2Type>
	auto s_inv_tv(const double *inv_pm, const double *vs, V1Type v1_t, double *vs_out, V2Type v2_t) noexcept->void{
		const double* vs3{ vs + at(3, v1_t) };
		double* vs_out3{ vs_out + at(3, v2_t) };

		s_c3i(inv_pm + 3, 4, vs3, v1_t, vs_out3, v2_t);
		s_va(3, vs, v1_t, vs_out3, v2_t);

		s_inv_pm_dot_v3(inv_pm, vs_out3, v2_t, vs_out, v2_t);
		s_inv_pm_dot_v3(inv_pm, vs3, v1_t, vs_out3, v2_t);
	}
	/// \brief 根据位姿矩阵的逆矩阵转换六维速度向量
	///
	/// 等同于： vs_out = alpha * tmv(pm^-1) * vs
	///
	///
	auto ARIS_API s_inv_tv(double alpha, const double *inv_pm, const double *vs, double *vs_out) noexcept->void;
	/// \brief 根据位姿矩阵的逆矩阵转换六维速度向量
	///
	/// 等同于： vs_out = alpha * tmv(pm^-1) * vs
	///
	///
	template<typename V1Type, typename V2Type>
	auto s_inv_tv(double alpha, const double *inv_pm, const double *vs, V1Type v1_t, double *vs_out, V2Type v2_t) noexcept->void{
		s_inv_tv(inv_pm, vs, v1_t, vs_out, v2_t);
		s_nv(6, alpha, vs_out, v2_t);
	}
	/// \brief 根据位姿矩阵转换六维速度向量
	///
	/// 等同于： vs_out = tmv(pm^-1) * vs + vs_out
	///
	///
	auto ARIS_API s_inv_tva(const double *inv_pm, const double *vs, double *vs_out) noexcept->void;
	/// \brief 根据位姿矩阵转换六维速度向量
	///
	/// 等同于： vs_out = tmv(pm^-1) * vs + vs_out
	///
	///
	template<typename V1Type, typename V2Type>
	auto s_inv_tva(const double *inv_pm, const double *vs, V1Type v1_t, double *vs_out, V2Type v2_t) noexcept->void{
		double tem[6];
		s_inv_tv(inv_pm, vs, v1_t, tem, 1);
		s_va(6, tem, 1, vs_out, v2_t);
	}
	/// \brief 根据位姿矩阵转换六维速度向量
	///
	/// 等同于： vs_out = alpha * tmv(pm^-1) * vs + beta * vs_out
	///
	///
	auto ARIS_API s_inv_tva(double alpha, const double *inv_pm, const double *vs, double *vs_out) noexcept->void;
	/// \brief 根据位姿矩阵转换六维速度向量
	///
	/// 等同于： vs_out = alpha * tmv(pm^-1) * vs + beta * vs_out
	///
	///
	template<typename V1Type, typename V2Type>
	auto s_inv_tva(double alpha, const double *inv_pm, const double *vs, V1Type v1_t, double *vs_out, V2Type v2_t) noexcept->void{
		double tem[6];
		s_inv_tv(inv_pm, vs, v1_t, tem, 1);
		s_va(6, alpha, tem, 1, vs_out, v2_t);
	}
	/// \brief 根据位姿矩阵转换六维力矩阵
	///
	/// 等同于： fs_mtx_out = tmf(pm) * fs_mtx
	///
	///
	auto inline s_tf_n(Size n, const double *pm, const double *fs_mtx, double *fs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_tf(pm, fs_mtx + i, n, fs_mtx_out + i, n); }
	/// \brief 根据位姿矩阵转换六维力矩阵
	///
	/// 等同于： fs_mtx_out = tmf(pm) * fs_mtx
	///
	///
	template<typename F1Type, typename F2Type>
	auto inline s_tf_n(Size n, const double *pm, const double *fs_mtx, F1Type f1_t, double *fs_mtx_out, F2Type f2_t) noexcept->void{
		for (Size i(-1), f1id(0), f2id(0); ++i < n; f1id = next_c(f1id, f1_t), f2id = next_c(f2id, f2_t))
			s_tf(pm, fs_mtx + f1id, f1_t, fs_mtx_out + f2id, f2_t);
	}
	/// \brief 根据位姿矩阵转换六维力矩阵
	///
	/// 等同于： fs_mtx_out = alpha * tmf(pm) * fs_mtx
	///
	///
	auto inline s_tf_n(Size n, double alpha, const double *pm, const double *fs_mtx, double *fs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_tf(alpha, pm, fs_mtx + i, n, fs_mtx_out + i, n); }
	/// \brief 根据位姿矩阵转换六维力矩阵
	///
	/// 等同于： fs_mtx_out = alpha * tmf(pm) * fs_mtx
	///
	///
	template<typename F1Type, typename F2Type>
	auto inline s_tf_n(Size n, double alpha, const double *pm, const double *fs_mtx, F1Type f1_t, double *fs_mtx_out, F2Type f2_t) noexcept->void{
		for (Size i(-1), f1id(0), f2id(0); ++i < n; f1id = next_c(f1id, f1_t), f2id = next_c(f2id, f2_t))
			s_tf(alpha, pm, fs_mtx + f1id, f1_t, fs_mtx_out + f2id, f2_t);
	}
	/// \brief 根据位姿矩阵转换六维力矩阵
	///
	/// 等同于： fs_mtx_out = tmf(pm) * fs_mtx + fs_mtx_out
	///
	///
	auto inline s_tfa_n(Size n, const double *pm, const double *fs_mtx, double *fs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_tfa(pm, fs_mtx + i, n, fs_mtx_out + i, n); }
	/// \brief 根据位姿矩阵转换六维力矩阵
	///
	/// 等同于： fs_mtx_out = tmf(pm) * fs_mtx + fs_mtx_out
	///
	///
	template<typename F1Type, typename F2Type>
	auto inline s_tfa_n(Size n, const double *pm, const double *fs_mtx, F1Type f1_t, double *fs_mtx_out, F2Type f2_t) noexcept->void{
		for (Size i(-1), f1id(0), f2id(0); ++i < n; f1id = next_c(f1id, f1_t), f2id = next_c(f2id, f2_t))
			s_tfa(pm, fs_mtx + f1id, f1_t, fs_mtx_out + f2id, f2_t);
	}
	/// \brief 根据位姿矩阵转换六维力矩阵
	///
	/// 等同于： fs_mtx_out = alpha * tmf(pm) * fs_mtx + beta * fs_mtx_out
	///
	///
	auto inline s_tfa_n(Size n, double alpha, const double *pm, const double *fs_mtx, double *fs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_tfa(alpha, pm, fs_mtx + i, n, fs_mtx_out + i, n); }
	/// \brief 根据位姿矩阵转换六维力矩阵
	///
	/// 等同于： fs_mtx_out = alpha * tmf(pm) * fs_mtx + beta * fs_mtx_out
	///
	///
	template<typename F1Type, typename F2Type>
	auto inline s_tfa_n(Size n, double alpha, const double *pm, const double *fs_mtx, F1Type f1_t, double *fs_mtx_out, F2Type f2_t) noexcept->void{
		for (Size i(-1), f1id(0), f2id(0); ++i < n; f1id = next_c(f1id, f1_t), f2id = next_c(f2id, f2_t))
			s_tfa(alpha, pm, fs_mtx + f1id, f1_t, fs_mtx_out + f2id, f2_t);
	}
	/// \brief 根据位姿矩阵转换六维力矩阵
	///
	/// 等同于： fs_mtx_out = tmf(inv_pm^-1) * fs_mtx
	///
	///
	auto inline s_inv_tf_n(Size n, const double *inv_pm, const double *fs_mtx, double *fs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_inv_tf(inv_pm, fs_mtx + i, n, fs_mtx_out + i, n); }
	/// \brief 根据位姿矩阵转换六维力矩阵
	///
	/// 等同于： fs_mtx_out = tmf(inv_pm^-1) * fs_mtx
	///
	///
	template<typename F1Type, typename F2Type>
	auto inline s_inv_tf_n(Size n, const double *inv_pm, const double *fs_mtx, F1Type f1_t, double *fs_mtx_out, F2Type f2_t) noexcept->void{
		for (Size i(-1), f1id(0), f2id(0); ++i < n; f1id = next_c(f1id, f1_t), f2id = next_c(f2id, f2_t))
			s_inv_tf(inv_pm, fs_mtx + f1id, f1_t, fs_mtx_out + f2id, f2_t);
	}
	/// \brief 根据位姿矩阵转换六维力矩阵
	///
	/// 等同于： fs_mtx_out = alpha * tmf(inv_pm^-1) * fs_mtx
	///
	///
	auto inline s_inv_tf_n(Size n, double alpha, const double *inv_pm, const double *fs_mtx, double *fs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_inv_tf(alpha, inv_pm, fs_mtx + i, n, fs_mtx_out + i, n); }
	/// \brief 根据位姿矩阵转换六维力矩阵
	///
	/// 等同于： fs_mtx_out = alpha * tmf(inv_pm^-1) * fs_mtx
	///
	///
	template<typename F1Type, typename F2Type>
	auto inline s_inv_tf_n(Size n, double alpha, const double *inv_pm, const double *fs_mtx, F1Type f1_t, double *fs_mtx_out, F2Type f2_t) noexcept->void{
		for (Size i(-1), f1id(0), f2id(0); ++i < n; f1id = next_c(f1id, f1_t), f2id = next_c(f2id, f2_t))
			s_inv_tf(alpha, inv_pm, fs_mtx + f1id, f1_t, fs_mtx_out + f2id, f2_t);
	}
	/// \brief 根据位姿矩阵转换六维力矩阵
	///
	/// 等同于： fs_mtx_out = tmf(pm^-1) * fs_mtx + fs_mtx_out
	///
	///
	auto inline s_inv_tfa_n(Size n, const double *inv_pm, const double *fs_mtx, double *fs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_inv_tfa(inv_pm, fs_mtx + i, n, fs_mtx_out + i, n); }
	/// \brief 根据位姿矩阵转换六维力矩阵
	///
	/// 等同于： fs_mtx_out = tmf(pm^-1) * fs_mtx + fs_mtx_out
	///
	///
	template<typename F1Type, typename F2Type>
	auto inline s_inv_tfa_n(Size n, const double *inv_pm, const double *fs_mtx, F1Type f1_t, double *fs_mtx_out, F2Type f2_t) noexcept->void{
		for (Size i(-1), f1id(0), f2id(0); ++i < n; f1id = next_c(f1id, f1_t), f2id = next_c(f2id, f2_t))
			s_inv_tfa(inv_pm, fs_mtx + f1id, f1_t, fs_mtx_out + f2id, f2_t);
	}
	/// \brief 根据位姿矩阵转换六维力矩阵
	///
	/// 等同于： fs_mtx_out = alpha * tmf(pm^-1) * fs_mtx + beta * fs_mtx_out
	///
	///
	auto inline s_inv_tfa_n(Size n, double alpha, const double *inv_pm, const double *fs_mtx, double *fs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_inv_tfa(alpha, inv_pm, fs_mtx + i, n, fs_mtx_out + i, n); }
	/// \brief 根据位姿矩阵转换六维力矩阵
	///
	/// 等同于： fs_mtx_out = alpha * tmf(pm^-1) * fs_mtx + beta * fs_mtx_out
	///
	///
	template<typename F1Type, typename F2Type>
	auto inline s_inv_tfa_n(Size n, double alpha, const double *inv_pm, const double *fs_mtx, F1Type f1_t, double *fs_mtx_out, F2Type f2_t) noexcept->void{
		for (Size i(-1), f1id(0), f2id(0); ++i < n; f1id = next_c(f1id, f1_t), f2id = next_c(f2id, f2_t))
			s_inv_tfa(alpha, inv_pm, fs_mtx + f1id, f1_t, fs_mtx_out + f2id, f2_t);
	}
	/// \brief 根据位姿矩阵转换六维力矩阵
	///
	/// 等同于： m_out = tmv(pm) * fces_in
	///
	///
	auto inline s_tv_n(Size n, const double *pm, const double *vs_mtx, double *vs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_tv(pm, vs_mtx + i, n, vs_mtx_out + i, n); }
	/// \brief 根据位姿矩阵转换六维力矩阵
	///
	/// 等同于： m_out = tmv(pm) * fces_in
	///
	///
	template<typename V1Type, typename V2Type>
	auto inline s_tv_n(Size n, const double *pm, const double *vs_mtx, V1Type v1_t, double *vs_mtx_out, V2Type v2_t) noexcept->void{
		for (Size i(-1), v1id(0), v2id(0); ++i < n; v1id = next_c(v1id, v1_t), v2id = next_c(v2id, v2_t))
			s_tv(pm, vs_mtx + v1id, v1_t, vs_mtx_out + v2id, v2_t);
	}
	/// \brief 根据位姿矩阵转换六维速度向量
	///
	/// 等同于： vs_out = alpha * tmv(pm) * vs
	///
	///
	auto inline s_tv_n(Size n, double alpha, const double *pm, const double *vs_mtx, double *vs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_tv(alpha, pm, vs_mtx + i, n, vs_mtx_out + i, n); }
	/// \brief 根据位姿矩阵转换六维速度向量
	///
	/// 等同于： vs_out = alpha * tmv(pm) * vs
	///
	///
	template<typename V1Type, typename V2Type>
	auto inline s_tv_n(Size n, double alpha, const double *pm, const double *vs_mtx, V1Type v1_t, double *vs_mtx_out, V2Type v2_t) noexcept->void{
		for (Size i(-1), v1id(0), v2id(0); ++i < n; v1id = next_c(v1id, v1_t), v2id = next_c(v2id, v2_t))
			s_tv(alpha, pm, vs_mtx + v1id, v1_t, vs_mtx_out + v2id, v2_t);
	}
	/// \brief 根据位姿矩阵转换六维力矩阵
	///
	/// 等同于： m_out = alpha * tmv(pm) * fces_in + beta * m_out
	///
	///
	auto inline s_tva_n(Size n, const double *pm, const double *vs_mtx, double *vs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_tva(pm, vs_mtx + i, n, vs_mtx_out + i, n); }
	/// \brief 根据位姿矩阵转换六维力矩阵
	///
	/// 等同于： m_out = alpha * tmv(pm) * fces_in + beta * m_out
	///
	///
	template<typename V1Type, typename V2Type>
	auto inline s_tva_n(Size n, const double *pm, const double *vs_mtx, V1Type v1_t, double *vs_mtx_out, V2Type v2_t) noexcept->void{
		for (Size i(-1), v1id(0), v2id(0); ++i < n; v1id = next_c(v1id, v1_t), v2id = next_c(v2id, v2_t))
			s_tva(pm, vs_mtx + v1id, v1_t, vs_mtx_out + v2id, v2_t);
	}
	/// \brief 根据位姿矩阵转换六维力矩阵
	///
	/// 等同于： m_out = alpha * tmv(pm) * fces_in + beta * m_out
	///
	///
	auto inline s_tva_n(Size n, double alpha, const double *pm, const double *vs_mtx, double *vs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_tva(alpha, pm, vs_mtx + i, n, vs_mtx_out + i, n); }
	/// \brief 根据位姿矩阵转换六维力矩阵
	///
	/// 等同于： m_out = alpha * tmv(pm) * fces_in + beta * m_out
	///
	///
	template<typename V1Type, typename V2Type>
	auto inline s_tva_n(Size n, double alpha, const double *pm, const double *vs_mtx, V1Type v1_t, double *vs_mtx_out, V2Type v2_t) noexcept->void{
		for (Size i(-1), v1id(0), v2id(0); ++i < n; v1id = next_c(v1id, v1_t), v2id = next_c(v2id, v2_t))
			s_tva(alpha, pm, vs_mtx + v1id, v1_t, vs_mtx_out + v2id, v2_t);
	}
	/// \brief 根据位姿矩阵转换六维速度矩阵
	///
	/// 等同于： vs_mtx_out = tmv(inv_pm^-1) * vs_mtx
	///
	///
	auto inline s_inv_tv_n(Size n, const double *inv_pm, const double *vs_mtx, double *vs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_inv_tv(inv_pm, vs_mtx + i, n, vs_mtx_out + i, n); }
	/// \brief 根据位姿矩阵转换六维速度矩阵
	///
	/// 等同于： vs_mtx_out = tmv(inv_pm^-1) * vs_mtx
	///
	///
	template<typename V1Type, typename V2Type>
	auto inline s_inv_tv_n(Size n, const double *inv_pm, const double *vs_mtx, V1Type v1_t, double *vs_mtx_out, V2Type v2_t) noexcept->void{
		for (Size i(-1), v1id(0), v2id(0); ++i < n; v1id = next_c(v1id, v1_t), v2id = next_c(v2id, v2_t))
			s_inv_tv(inv_pm, vs_mtx + v1id, v1_t, vs_mtx_out + v2id, v2_t);
	}
	/// \brief 根据位姿矩阵转换六维速度矩阵
	///
	/// 等同于： vs_mtx_out = alpha * tmv(inv_pm^-1) * vs_mtx
	///
	///
	auto inline s_inv_tv_n(Size n, double alpha, const double *inv_pm, const double *vs_mtx, double *vs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_inv_tv(alpha, inv_pm, vs_mtx + i, n, vs_mtx_out + i, n); }
	/// \brief 根据位姿矩阵转换六维速度矩阵
	///
	/// 等同于： vs_mtx_out = alpha * tmv(inv_pm^-1) * vs_mtx
	///
	///
	template<typename V1Type, typename V2Type>
	auto inline s_inv_tv_n(Size n, double alpha, const double *inv_pm, const double *vs_mtx, V1Type v1_t, double *vs_mtx_out, V2Type v2_t) noexcept->void{
		for (Size i(-1), v1id(0), v2id(0); ++i < n; v1id = next_c(v1id, v1_t), v2id = next_c(v2id, v2_t))
			s_inv_tv(alpha, inv_pm, vs_mtx + v1id, v1_t, vs_mtx_out + v2id, v2_t);
	}
	/// \brief 根据位姿矩阵转换六维速度矩阵
	///
	/// 等同于： vs_mtx_out = tmv(pm^-1) * vs_mtx + vs_mtx_out
	///
	///
	auto inline s_inv_tva_n(Size n, const double *inv_pm, const double *vs_mtx, double *vs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_inv_tva(inv_pm, vs_mtx + i, n, vs_mtx_out + i, n); }
	/// \brief 根据位姿矩阵转换六维速度矩阵
	///
	/// 等同于： vs_mtx_out = tmv(pm^-1) * vs_mtx + vs_mtx_out
	///
	///
	template<typename V1Type, typename V2Type>
	auto inline s_inv_tva_n(Size n, const double *inv_pm, const double *vs_mtx, V1Type v1_t, double *vs_mtx_out, V2Type v2_t) noexcept->void{
		for (Size i(-1), v1id(0), v2id(0); ++i < n; v1id = next_c(v1id, v1_t), v2id = next_c(v2id, v2_t))
			s_inv_tva(inv_pm, vs_mtx + v1id, v1_t, vs_mtx_out + v2id, v2_t);
	}
	/// \brief 根据位姿矩阵转换六维速度矩阵
	///
	/// 等同于： vs_mtx_out = alpha * tmv(pm^-1) * vs_mtx + beta * vs_mtx_out
	///
	///
	auto inline s_inv_tva_n(Size n, double alpha, const double *inv_pm, const double *vs_mtx, double *vs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_inv_tva(alpha, inv_pm, vs_mtx + i, n, vs_mtx_out + i, n); }
	/// \brief 根据位姿矩阵转换六维速度矩阵
	///
	/// 等同于： vs_mtx_out = alpha * tmv(pm^-1) * vs_mtx + beta * vs_mtx_out
	///
	///
	template<typename V1Type, typename V2Type>
	auto inline s_inv_tva_n(Size n, double alpha, const double *inv_pm, const double *vs_mtx, V1Type v1_t, double *vs_mtx_out, V2Type v2_t) noexcept->void{
		for (Size i(-1), v1id(0), v2id(0); ++i < n; v1id = next_c(v1id, v1_t), v2id = next_c(v2id, v2_t))
			s_inv_tva(alpha, inv_pm, vs_mtx + v1id, v1_t, vs_mtx_out + v2id, v2_t);
	}

}

#endif