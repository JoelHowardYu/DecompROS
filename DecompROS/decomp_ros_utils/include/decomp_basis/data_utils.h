/**
 * @file data_utils.h
 * @brief 提供一些广泛使用的基本类型函数
 */
#ifndef DATA_UTILS_H
#define DATA_UTILS_H

#include <decomp_basis/data_type.h>

/// 用于转换向量的模板函数
template <class T, class TF>
vec_E<T> transform_vec(const vec_E<T> &t, const TF &tf) {
  vec_E<T> new_t; // 创建一个新的向量来存储转换后的结果
  for (const auto &it : t) // 遍历输入向量中的每个元素
    new_t.push_back(tf * it); // 使用给定的变换函数TF对每个元素进行变换，并将结果添加到新向量中
  return new_t; // 返回转换后的新向量
}

/// 用于计算距离的模板函数
template <class T>
decimal_t total_distance(const vec_E<T>& vs){
  decimal_t dist = 0; // 初始化总距离为0
  for(unsigned int i = 1; i < vs.size(); i++) // 遍历向量中的每个元素，从第二个元素开始
    dist += (vs[i] - vs[i-1]).norm(); // 计算当前元素与前一个元素之间的距离，并累加到总距离中

  return dist; // 返回总距离
}


/// 使用给定的TF变换向量中的所有元素
#define transform_vec3 transform_vec<Vec3f, Aff3f>
/// 计算Vec3f类型向量的总距离
#define total_distance3f total_distance<Vec3f>
/// 计算Vec3i类型向量的总距离
#define total_distance3i total_distance<Vec3i>
#endif