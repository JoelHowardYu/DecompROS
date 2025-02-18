/**
 * @file iterative_decomp.h
 * @brief IterativeDecomp 类
 */
#ifndef ITERATIVE_DECOMP_H
#define ITERATIVE_DECOMP_H

#include <decomp_util/ellipsoid_decomp.h>

/**
 * @brief IterativeDecomp 类
 *
 * 迭代调用 ElliseDecomp 以形成一个更安全的飞行走廊，远离障碍物
 */
template <int Dim>
class IterativeDecomp : public EllipsoidDecomp<Dim>
{
  public:
    /// 简单的构造函数
    IterativeDecomp() {}
    /**
     * @brief 基本的构造函数
     * @param origin 全局包围盒的原点
     * @param dim 全局包围盒的维度
     */
    IterativeDecomp(const Vecf<Dim> &origin, const Vecf<Dim> &dim) :
        EllipsoidDecomp<Dim>(origin, dim) {}
    /**
     * @brief 分解线程
     * @param path_raw 需要膨胀的路径
     * @param iter_num 最大迭代次数
     * @param offset_x 添加到长半轴的偏移量，默认是 0
     * @param res 路径降采样的分辨率
     */
    void dilate_iter(const vec_Vecf<Dim> &path_raw, int iter_num = 5,
                decimal_t res = 0, decimal_t offset_x = 0) {
      vec_Vecf<Dim> path = res > 0 ? downsample(path_raw, res) : path_raw; // 如果分辨率大于0，则对路径进行降采样
      this->dilate(path, offset_x); // 对路径进行膨胀
      vec_Vecf<Dim> new_path = simplify(path); // 简化路径
      for (int i = 0; i < iter_num; i++) { // 迭代处理
        if (new_path.size() == path.size()) // 如果新路径和旧路径大小相同，则跳出循环
          break;
        else {
          path = new_path; // 更新路径
          this->dilate(path, offset_x); // 再次对路径进行膨胀
          new_path = simplify(path); // 再次简化路径
        }
      }
    }

  protected:
    /// 将路径均匀采样为多个段
    vec_Vecf<Dim> downsample(const vec_Vecf<Dim> &ps, decimal_t d) {
      // 根据长度进行细分
      if (ps.size() < 2)
        return ps;
      vec_Vecf<Dim> path;
      for (unsigned int i = 1; i < ps.size(); i++) {
        decimal_t dist = (ps[i] - ps[i - 1]).norm(); // 计算两点之间的距离
        int cnt = std::ceil(dist / d); // 计算需要细分的段数
        for (int j = 0; j < cnt; j++)
          path.push_back(ps[i - 1] + j * (ps[i] - ps[i - 1]) / cnt); // 将路径细分
      }
      path.push_back(ps.back()); // 添加最后一个点
      return path;
    }

    /// 获取最近的距离
    decimal_t cal_closest_dist(const Vecf<Dim>& pt, const Polyhedron<Dim>& vs){
      decimal_t dist = std::numeric_limits<decimal_t>::infinity(); // 初始化距离为无穷大
      for(const auto& it: vs.hyperplanes()){ // 遍历超平面
        decimal_t d = std::abs(it.n_.dot(pt - it.p_)); // 计算点到超平面的距离
        if(d < dist)
          dist = d; // 更新最小距离
      }
      return dist;
    }

    /// 移除冗余的航点
    vec_Vecf<Dim> simplify(const vec_Vecf<Dim>& path) {
			if(path.size() <= 2) // 如果路径点数小于等于2，直接返回
				return path;

			Vecf<Dim> ref_pt = path.front(); // 获取第一个点作为参考点
			vec_Vecf<Dim> new_path;
			new_path.push_back(ref_pt); // 将参考点添加到新路径中

			for(size_t i = 2; i < path.size(); i ++){ // 遍历路径中的点
				if(this->polyhedrons_[i-1].inside(ref_pt) && // 如果参考点在多面体内且距离大于0.1
					 cal_closest_dist(ref_pt, this->polyhedrons_[i-1]) > 0.1) {
				}
				else{
					ref_pt = path[i-1]; // 更新参考点
					new_path.push_back(ref_pt); // 将参考点添加到新路径中
				}
			}
			new_path.push_back(path.back()); // 添加最后一个点
			return new_path;
		}
};

typedef IterativeDecomp<2> IterativeDecomp2D; // 定义二维的 IterativeDecomp

typedef IterativeDecomp<3> IterativeDecomp3D; // 定义三维的 IterativeDecomp
#endif