/**
 * @file data_type.h
 * @brief 定义了本库中使用的所有数据类型
 * 
 * 大部分数据类型是Eigen库的别名。
 */

#include <stdio.h>  // 包含标准输入输出库
#include <math.h>   // 包含数学函数库
#include <limits>   // 包含数值极限库
#include <vector>   // 包含标准向量库
#include <Eigen/Geometry>  // 包含Eigen几何库
#include <Eigen/StdVector>  // 包含Eigen标准向量库

/// 在printf函数中设置红色字体
#ifndef ANSI_COLOR_RED
#define ANSI_COLOR_RED "\x1b[1;31m"
#endif
/// 在printf函数中设置绿色字体
#ifndef ANSI_COLOR_GREEN
#define ANSI_COLOR_GREEN "\x1b[1;32m"
#endif
/// 在printf函数中设置黄色字体
#ifndef ANSI_COLOR_YELLOW
#define ANSI_COLOR_YELLOW "\x1b[1;33m"
#endif
/// 在printf函数中设置蓝色字体
#ifndef ANSI_COLOR_BLUE
#define ANSI_COLOR_BLUE "\x1b[1;34m"
#endif
/// 在printf函数中设置洋红色字体
#ifndef ANSI_COLOR_MAGENTA
#define ANSI_COLOR_MAGENTA "\x1b[1;35m"
#endif
/// 在printf函数中设置青色字体
#ifndef ANSI_COLOR_CYAN
#define ANSI_COLOR_CYAN "\x1b[1;36m"
#endif
/// 在printf函数中重置字体颜色
#ifndef ANSI_COLOR_RESET
#define ANSI_COLOR_RESET "\x1b[0m"
#endif

#ifndef DATA_TYPE_H
#define DATA_TYPE_H
/*! \brief 重命名库中使用的浮点类型

    默认设置为double，但用户可以将其更改为float。
*/
typedef double decimal_t;

/// 预分配的std::vector，用于Eigen，使用vec_E
template <typename T>
using vec_E = std::vector<T, Eigen::aligned_allocator<T>>;
/// Eigen 1D浮点向量
template <int N>
using Vecf = Eigen::Matrix<decimal_t, N, 1>;
/// Eigen 1D整数向量
template <int N>
using Veci = Eigen::Matrix<int, N, 1>;
/// MxN的Eigen矩阵
template <int M, int N>
using Matf = Eigen::Matrix<decimal_t, M, N>;
/// MxN的Eigen矩阵，其中M未知
template <int N>
using MatDNf = Eigen::Matrix<decimal_t, Eigen::Dynamic, N>;
/// Eigen 1D浮点向量的向量
template <int N>
using vec_Vecf = vec_E<Vecf<N>>;
/// Eigen 1D整数向量的向量
template <int N>
using vec_Veci = vec_E<Veci<N>>;

/// 大小为2的Eigen 1D浮点向量
typedef Vecf<2> Vec2f;
/// 大小为2的Eigen 1D整数向量
typedef Veci<2> Vec2i;
/// 大小为3的Eigen 1D浮点向量
typedef Vecf<3> Vec3f;
/// 大小为3的Eigen 1D整数向量
typedef Veci<3> Vec3i;
/// 大小为4的Eigen 1D浮点向量
typedef Vecf<4> Vec4f;
/// 大小为6的浮点列向量
typedef Vecf<6> Vec6f;

/// 类型为Vec2f的向量
typedef vec_E<Vec2f> vec_Vec2f;
/// 类型为Vec2i的向量
typedef vec_E<Vec2i> vec_Vec2i;
/// 类型为Vec3f的向量
typedef vec_E<Vec3f> vec_Vec3f;
/// 类型为Vec3i的向量
typedef vec_E<Vec3i> vec_Vec3i;

/// 2x2的浮点矩阵
typedef Matf<2, 2> Mat2f;
/// 3x3的浮点矩阵
typedef Matf<3, 3> Mat3f;
/// 4x4的浮点矩阵
typedef Matf<4, 4> Mat4f;
/// 6x6的浮点矩阵
typedef Matf<6, 6> Mat6f;

/// 动态Nx1的Eigen浮点向量
typedef Vecf<Eigen::Dynamic> VecDf;
/// Nx2的Eigen浮点矩阵
typedef MatDNf<2> MatD2f;
/// Nx3的Eigen浮点矩阵
typedef MatDNf<3> MatD3f;
/// 动态MxN的Eigen浮点矩阵
typedef Matf<Eigen::Dynamic, Eigen::Dynamic> MatDf;

/// Eigen::Affine2d的别名
typedef Eigen::Transform<decimal_t, 2, Eigen::Affine> Aff2f;
/// Eigen::Affine3d的别名
typedef Eigen::Transform<decimal_t, 3, Eigen::Affine> Aff3f;
#endif

#ifndef EIGEN_QUAT
#define EIGEN_QUAT
/// Eigen::Quaterniond的别名
typedef Eigen::Quaternion<decimal_t> Quatf;
#endif

#ifndef EIGEN_EPSILON
#define EIGEN_EPSILON
/// 补偿数值误差
constexpr decimal_t epsilon_ = 1e-10; // 数值计算误差
#endif