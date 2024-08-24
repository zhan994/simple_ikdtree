/**
 * \file ikdtree.h
 * \author Zhihao Zhan (zhanzhihao_dt@163.com)
 * \brief incremental ikdtree
 * \version 0.1
 * \date 2024-08-22
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef IKDTREE_H
#define IKDTREE_H

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <pcl/point_types.h>
#include <queue>

#define EPSS 1e-6
#define MINIMAL_UNBALANCED_TREE_SIZE 10
#define MULTI_THREAD_REBUILD_POINT_NUM 1500
#define DOWNSAMPLE_SWITCH true
#define FORCE_REBUILD_PERCENT 0.2
#define Q_LEN 1000000

namespace ikdtree {
struct BoxPointType {
  float vertex_min[3];
  float vertex_max[3];
};

enum Operation {
  ADD_POINT,
  DELETE_POINT,
  DELETE_BOX,
  ADD_BOX,
  DOWNSAMPLE_DELETE,
  PUSH_DOWN
};

enum Delete { NOT_RECORD, DELETE_POINTS_RECORD, MULTI_THREAD_RECORD };

template <typename PointType> class iKdTree {
public:
  using PointVector =
      std::vector<PointType, Eigen::aligned_allocator<PointType>>;
  using Ptr = std::shared_ptr<iKdTree<PointType>>;

  /**
   * \brief 节点
   *
   */
  struct iKdTreeNode {
    PointType point;
    int division_axis;
    int tree_size = 1;
    int invalid_point_num = 0;
    int down_del_num = 0;
    bool point_deleted = false;
    bool tree_deleted = false;
    bool point_downsample_deleted = false;
    bool tree_downsample_deleted = false;
    bool need_push_down_to_left = false;
    bool need_push_down_to_right = false;
    bool working_flag = false;
    pthread_mutex_t push_down_mutex_lock;
    float node_range_x[2], node_range_y[2], node_range_z[2];
    float radius_sq;
    iKdTreeNode *left_son_ptr = nullptr;
    iKdTreeNode *right_son_ptr = nullptr;
    iKdTreeNode *father_ptr = nullptr;
    // For paper data record
    float alpha_del;
    float alpha_bal;
  };

  /**
   * \brief 操作记录
   *
   */
  struct OperationLoggerType {
    PointType point;
    BoxPointType boxpoint;
    bool tree_deleted, tree_downsample_deleted;
    Operation op;
  };

  /**
   * \brief 点的比较
   *
   */
  struct PointTypeCMP {
    /**
     * \brief 构造函数
     *
     * \param p
     * \param d
     */
    PointTypeCMP(PointType p = PointType(), float d = INFINITY) {
      point = p;
      dist = d;
    }

    /**
     * \brief 对比，dist接近时看点的x差距
     *
     * \param a
     * \return true
     * \return false
     */
    bool operator<(const PointTypeCMP &a) const {
      if (fabs(dist - a.dist) < 1e-10)
        return point.x < a.point.x;
      else
        return dist < a.dist;
    }

    PointType point;
    float dist = 0.0;
  };
};
} // namespace ikdtree

#endif // IKDTREE_H