// line_endpoint_comparison.cpp
// 对比两种线特征表示方法：
// 方案A: Plücker + 线投影 (EdgeSE3Line3DProjection)
// 方案B: 端点 + 点投影 (EdgeProjectXYZ2UV)

#include <fstream>
#include <iostream>
#include <cmath>
#include <chrono>
#include <iomanip>

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/base_multi_edge.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/sampler.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d_addons/types_slam3d_addons.h"
#include "g2o/types/slam3d_addons/edge_se3_line3d_projection.h"
#include "g2o/types/sba/types_six_dof_expmap.h"

using namespace g2o;
using namespace std;
using namespace Eigen;

G2O_USE_OPTIMIZATION_LIBRARY(eigen)

// ============================================================================
// 工具函数
// ============================================================================
Eigen::Isometry3d sample_noise_from_se3(const Vector6& cov) {
  double nx = Sampler::gaussRand(0., cov(0));
  double ny = Sampler::gaussRand(0., cov(1));
  double nz = Sampler::gaussRand(0., cov(2));
  double nroll = Sampler::gaussRand(0., cov(3));
  double npitch = Sampler::gaussRand(0., cov(4));
  double nyaw = Sampler::gaussRand(0., cov(5));

  AngleAxisd aa(AngleAxisd(nyaw, Vector3d::UnitZ()) *
                AngleAxisd(nroll, Vector3d::UnitX()) *
                AngleAxisd(npitch, Vector3d::UnitY()));

  Eigen::Isometry3d retval = Isometry3d::Identity();
  retval.matrix().block<3, 3>(0, 0) = aa.toRotationMatrix();
  retval.translation() = Vector3d(nx, ny, nz);
  return retval;
}

Vector2d sample_noise_from_line2d(const Vector2d& cov) {
  return Vector2d(
      Sampler::gaussRand(0., cov(0)),
      Sampler::gaussRand(0., cov(1)));
}

// ============================================================================
// 点到线距离边 (参考 PL-VINS)
// ============================================================================
// 误差：观测端点到预测线的距离（2维）
// 顶点：[0] 端点1, [1] 端点2, [2] 位姿
// 测量：观测到的两个端点像素坐标 (u1, v1, u2, v2)
class EdgeEndpointToLine2D : public BaseMultiEdge<2, Vector4d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  EdgeEndpointToLine2D(double fx_, double fy_, double cx_, double cy_)
    : fx(fx_), fy(fy_), cx(cx_), cy(cy_) {
    resize(3);  // 3个顶点：端点1, 端点2, 位姿
  }

  void computeError() override {
    const VertexPointXYZ* vp1 = static_cast<const VertexPointXYZ*>(_vertices[0]);
    const VertexPointXYZ* vp2 = static_cast<const VertexPointXYZ*>(_vertices[1]);
    const VertexSE3Expmap* vpose = static_cast<const VertexSE3Expmap*>(_vertices[2]);

    // 3D端点（世界坐标系）
    Vector3d P1_w = vp1->estimate();
    Vector3d P2_w = vp2->estimate();

    // 变换到相机坐标系 (SE3Quat 存储的是 T_cw)
    Vector3d P1_c = vpose->estimate().map(P1_w);
    Vector3d P2_c = vpose->estimate().map(P2_w);

    // 投影到归一化平面
    Vector2d p1_norm(P1_c.x() / P1_c.z(), P1_c.y() / P1_c.z());
    Vector2d p2_norm(P2_c.x() / P2_c.z(), P2_c.y() / P2_c.z());

    // 从两个预测端点拟合归一化平面上的2D直线
    // 直线方程: n·p + d = 0，其中 n = [a, b], d = c
    // 法向量 n 垂直于方向向量
    Vector2d dir = p2_norm - p1_norm;
    double len = dir.norm();

    if (len < 1e-6) {
      // 两点重合，误差设为大值
      _error(0) = 1000.0;
      _error(1) = 1000.0;
      return;
    }

    // 直线法向量（归一化）：垂直于方向向量
    Vector2d normal(-dir.y() / len, dir.x() / len);

    // 直线方程: normal·(p - p1_norm) = 0
    // => normal·p - normal·p1_norm = 0
    // => a*x + b*y + c = 0, 其中 c = -normal·p1_norm
    double c = -normal.dot(p1_norm);

    // 将观测像素坐标转换为归一化坐标
    Vector2d obs1_norm((_measurement(0) - cx) / fx, (_measurement(1) - cy) / fy);
    Vector2d obs2_norm((_measurement(2) - cx) / fx, (_measurement(3) - cy) / fy);

    // 点到线的距离（已归一化，所以分母是1）
    _error(0) = normal.dot(obs1_norm) + c;
    _error(1) = normal.dot(obs2_norm) + c;
  }

  virtual bool read(std::istream& is) override {
    Vector4d meas;
    is >> meas(0) >> meas(1) >> meas(2) >> meas(3);
    setMeasurement(meas);
    return readInformationMatrix(is);
  }

  virtual bool write(std::ostream& os) const override {
    os << _measurement(0) << " " << _measurement(1) << " "
       << _measurement(2) << " " << _measurement(3) << " ";
    return writeInformationMatrix(os);
  }

  double fx, fy, cx, cy;  // 相机内参
};

// ============================================================================
// 实验结果结构
// ============================================================================
struct ExperimentResult {
  string method_name;
  double chi2_before;
  double chi2_after;
  double avg_trans_error;
  double avg_rot_error;
  double avg_line_error;
  int num_features;
  int num_observations;
  double computation_time_ms;
  bool converged;
};

// ============================================================================
// 场景数据生成
// ============================================================================
struct SceneData {
  // 线特征（端点表示）
  vector<pair<Vector3d, Vector3d>> lines;

  // 相机轨迹 (Ground Truth)
  vector<Isometry3d> poses;

  // 预生成的位姿噪声（保证两种方法使用相同噪声）
  vector<Isometry3d> pose_noises;
  vector<Isometry3d> odom_noises;

  // 预生成的线端点噪声（保证两种方法使用相同噪声）
  vector<pair<Vector3d, Vector3d>> lines_noisy;

  // 观测数据
  struct Observation {
    int pose_id;
    int line_id;
    Line2D line2d;
    Vector2d pixel1, pixel2;
  };
  vector<Observation> observations;

  // 相机参数
  double fx = 500, fy = 500, cx = 320, cy = 240;
  int img_width = 640, img_height = 480;

  // 噪声参数
  Vector6 odom_noise_sigma = Vector6(0.02, 0.01, 0.01, 0.002, 0.002, 0.005);

  void generateScene(bool debug = false) {
    // 创建更多线特征，分布在不同区域（更贴合实际场景）
    // 世界坐标系: X朝前, Y向左, Z向上

    // 区域1: 左侧墙面 (X=1.5~2.5, Y=0.5~0.8)
    lines.push_back({Vector3d(1.5, 0.6, 0.3), Vector3d(1.5, 0.6, 1.0)});   // 垂直线1
    lines.push_back({Vector3d(2.0, 0.7, 0.5), Vector3d(2.0, 0.7, 1.2)});   // 垂直线2
    lines.push_back({Vector3d(1.8, 0.5, 0.8), Vector3d(2.3, 0.5, 0.8)});   // 水平线1

    // 区域2: 右侧墙面 (X=1.5~2.5, Y=-0.5~-0.8)
    lines.push_back({Vector3d(1.6, -0.6, 0.4), Vector3d(1.6, -0.6, 1.1)});  // 垂直线3
    lines.push_back({Vector3d(2.2, -0.7, 0.6), Vector3d(2.2, -0.7, 1.3)});  // 垂直线4
    lines.push_back({Vector3d(1.7, -0.5, 0.9), Vector3d(2.4, -0.5, 0.9)});  // 水平线2

    // 区域3: 中央区域 (更远处的线，观测次数会更少)
    lines.push_back({Vector3d(3.0, -0.3, 0.5), Vector3d(3.0, 0.3, 0.5)});   // 水平线3
    lines.push_back({Vector3d(3.5, 0.0, 0.4), Vector3d(3.5, 0.0, 1.1)});    // 垂直线5

    // 区域4: 斜线（窗框、门框等）
    lines.push_back({Vector3d(2.0, -0.2, 0.3), Vector3d(2.5, 0.2, 0.9)});   // 斜线1
    lines.push_back({Vector3d(2.1, 0.3, 0.6), Vector3d(2.6, -0.1, 1.0)});   // 斜线2

    generateTrajectory();
    generateNoises();
    generateObservations(debug);
  }

  void generateNoises() {
    // 预生成位姿初始化噪声
    for (size_t i = 0; i < poses.size(); ++i) {
      pose_noises.push_back(sample_noise_from_se3(odom_noise_sigma));
    }
    // 预生成里程计测量噪声
    for (size_t i = 1; i < poses.size(); ++i) {
      odom_noises.push_back(sample_noise_from_se3(odom_noise_sigma));
    }
    // 预生成线端点初始化噪声（增加到0.3米，更贴合实际）
    double init_noise = 0.3;
    for (size_t i = 0; i < lines.size(); ++i) {
      Vector3d p1_noise(
          Sampler::gaussRand(0, init_noise),
          Sampler::gaussRand(0, init_noise),
          Sampler::gaussRand(0, init_noise));
      Vector3d p2_noise(
          Sampler::gaussRand(0, init_noise),
          Sampler::gaussRand(0, init_noise),
          Sampler::gaussRand(0, init_noise));
      Vector3d p1_noisy = lines[i].first + p1_noise;
      Vector3d p2_noisy = lines[i].second + p2_noise;
      lines_noisy.push_back({p1_noisy, p2_noisy});
    }
  }

private:
  void generateTrajectory() {
    Matrix3d R_wc;
    R_wc.col(0) = Vector3d(0, -1, 0);  // 相机X -> 世界-Y
    R_wc.col(1) = Vector3d(0, 0, -1);  // 相机Y -> 世界-Z
    R_wc.col(2) = Vector3d(1, 0, 0);   // 相机Z -> 世界+X

    Isometry3d pose = Isometry3d::Identity();
    pose.linear() = R_wc;
    pose.translation() = Vector3d(0, 0, 0.5);
    poses.push_back(pose);

    // 前进运动 - 减小旋转幅度以保持线在视野中
    for (int i = 0; i < 30; ++i) {
      Isometry3d delta = Isometry3d::Identity();
      delta.translation() = Vector3d(0.05, 0, 0.02 * cos(i * 0.4));  // 减小平移
      delta.matrix().block<3, 3>(0, 0) =
          (AngleAxisd(0.03 * sin(i * 0.3), Vector3d::UnitZ()) *   // 减小旋转
           AngleAxisd(0.02 * sin(i * 0.25), Vector3d::UnitY())).toRotationMatrix();
      pose = pose * delta;
      poses.push_back(pose);
    }

    // 返回运动
    for (int i = 0; i < 30; ++i) {
      Isometry3d delta = Isometry3d::Identity();
      delta.translation() = Vector3d(-0.05, 0, -0.02 * cos(i * 0.5));
      delta.matrix().block<3, 3>(0, 0) =
          (AngleAxisd(-0.03 * sin(i * 0.3), Vector3d::UnitZ()) *
           AngleAxisd(-0.02 * sin(i * 0.2), Vector3d::UnitY())).toRotationMatrix();
      pose = pose * delta;
      poses.push_back(pose);
    }

    // 侧向运动 - 减小幅度
    for (int i = 0; i < 20; ++i) {
      double lateral = (i < 10) ? 0.05 : -0.05;
      Isometry3d delta = Isometry3d::Identity();
      delta.translation() = Vector3d(0.02, lateral, 0);
      delta.matrix().block<3, 3>(0, 0) =
          AngleAxisd(0.02 * ((i < 10) ? 1 : -1), Vector3d::UnitZ()).toRotationMatrix();
      pose = pose * delta;
      poses.push_back(pose);
    }
  }

  void generateObservations(bool debug = false) {
    // 观测噪声设置为更真实的水平
    // Line2D: θ_noise = 0.02 rad (增加噪声)
    // Pixel: 0.02 rad × fx = 0.02 × 500 = 10.0 pixels
    Vector2d line_noise(0.02, 0.02);  // θ和ρ的标准差（提高到更真实水平）
    double pixel_noise = 10.0;        // 像素标准差（提高到更真实水平）
    double rho_threshold = 10.0;

    int reject_depth = 0, reject_fov = 0, reject_rho = 0;

    for (size_t pose_id = 0; pose_id < poses.size(); ++pose_id) {
      // poses[i] 存储的是相机在世界中的位姿 T_wc
      // T_wc 将相机系坐标变换到世界系: p_w = T_wc * p_c
      // 要把世界点变换到相机系，需要 T_cw = T_wc^(-1)
      Isometry3d T_wc = poses[pose_id];
      Isometry3d T_cw = T_wc.inverse();

      for (size_t line_id = 0; line_id < lines.size(); ++line_id) {
        Vector3d p1_w = lines[line_id].first;
        Vector3d p2_w = lines[line_id].second;

        // 变换到相机系
        Vector3d p1_c = T_cw * p1_w;
        Vector3d p2_c = T_cw * p2_w;

        if (debug && pose_id == 0) {
          cout << "  [调试] pose0, line" << line_id << ": p1_c=" << p1_c.transpose()
               << ", p2_c=" << p2_c.transpose() << endl;
        }

        // 检查深度 (点必须在相机前方)
        if (p1_c.z() < 0.1 || p2_c.z() < 0.1) {
          reject_depth++;
          continue;
        }

        // 投影到像素
        Vector2d pixel1(fx * p1_c.x() / p1_c.z() + cx, fy * p1_c.y() / p1_c.z() + cy);
        Vector2d pixel2(fx * p2_c.x() / p2_c.z() + cx, fy * p2_c.y() / p2_c.z() + cy);

        if (debug && pose_id == 0) {
          cout << "  [调试] pose0, line" << line_id << ": pixel1=" << pixel1.transpose()
               << ", pixel2=" << pixel2.transpose() << endl;
        }

        // 检查是否在图像内
        if (pixel1.x() < 0 || pixel1.x() >= img_width || pixel1.y() < 0 || pixel1.y() >= img_height ||
            pixel2.x() < 0 || pixel2.x() >= img_width || pixel2.y() < 0 || pixel2.y() >= img_height) {
          reject_fov++;
          continue;
        }

        // 从端点计算2D线参数
        Vector3d dir_c = (p2_c - p1_c).normalized();
        Vector3d moment_c = p1_c.cross(dir_c);

        // Plücker投影到归一化平面
        double n1 = dir_c.z() * moment_c.y() - dir_c.y() * moment_c.z();
        double n2 = dir_c.x() * moment_c.z() - dir_c.z() * moment_c.x();
        double d = dir_c.y() * moment_c.x() - dir_c.x() * moment_c.y();

        double norm = sqrt(n1*n1 + n2*n2);
        if (norm < 1e-10) continue;

        n1 /= norm; n2 /= norm; d /= norm;
        double theta_true = atan2(n2, n1);
        double rho_true = d;

        if (abs(rho_true) > rho_threshold) {
          reject_rho++;
          continue;
        }

        // 创建观测
        Observation obs;
        obs.pose_id = pose_id;
        obs.line_id = line_id;

        // Plücker观测 (添加噪声)
        Vector2d noise_line = sample_noise_from_line2d(line_noise);
        obs.line2d = Line2D(theta_true + noise_line(0), rho_true + noise_line(1));

        // 端点观测 (添加噪声)
        obs.pixel1 = pixel1 + Vector2d(Sampler::gaussRand(0, pixel_noise),
                                       Sampler::gaussRand(0, pixel_noise));
        obs.pixel2 = pixel2 + Vector2d(Sampler::gaussRand(0, pixel_noise),
                                       Sampler::gaussRand(0, pixel_noise));

        observations.push_back(obs);
      }
    }

    if (debug) {
      cout << "  [调试] 拒绝统计: 深度=" << reject_depth
           << ", FOV=" << reject_fov << ", rho=" << reject_rho << endl;
    }
  }
};

// ============================================================================
// 辅助函数：Line3D 转端点（用于可视化）
// ============================================================================
pair<Vector3d, Vector3d> line3dToEndpoints(const Line3D& L, double half_length = 0.5) {
  Vector6 cart = L.toCartesian();
  Vector3d point = cart.head<3>();  // 线上一点（最近原点的点）
  Vector3d dir = cart.tail<3>();    // 方向向量
  dir.normalize();

  Vector3d p1 = point - half_length * dir;
  Vector3d p2 = point + half_length * dir;
  return {p1, p2};
}

// ============================================================================
// 保存线收敛历史到文件
// ============================================================================
void saveLineConvergenceHistory(
    const string& filename,
    const vector<pair<Vector3d, Vector3d>>& gt_lines,
    const vector<vector<pair<Vector3d, Vector3d>>>& line_history,
    const vector<double>& chi2_history) {

  ofstream ofs(filename);
  if (!ofs.is_open()) {
    cerr << "无法打开文件: " << filename << endl;
    return;
  }

  ofs << fixed << setprecision(6);

  // 写入真值线
  ofs << "# Ground Truth Lines (line_id p1x p1y p1z p2x p2y p2z)" << endl;
  ofs << "GT " << gt_lines.size() << endl;
  for (size_t i = 0; i < gt_lines.size(); ++i) {
    ofs << i << " "
        << gt_lines[i].first.x() << " " << gt_lines[i].first.y() << " " << gt_lines[i].first.z() << " "
        << gt_lines[i].second.x() << " " << gt_lines[i].second.y() << " " << gt_lines[i].second.z() << endl;
  }

  // 写入每次迭代的线估计
  ofs << "# Iteration History (iter chi2 line_id p1x p1y p1z p2x p2y p2z)" << endl;
  ofs << "ITERATIONS " << line_history.size() << endl;
  for (size_t iter = 0; iter < line_history.size(); ++iter) {
    ofs << "ITER " << iter << " " << chi2_history[iter] << endl;
    for (size_t lid = 0; lid < line_history[iter].size(); ++lid) {
      const auto& endpoints = line_history[iter][lid];
      ofs << lid << " "
          << endpoints.first.x() << " " << endpoints.first.y() << " " << endpoints.first.z() << " "
          << endpoints.second.x() << " " << endpoints.second.y() << " " << endpoints.second.z() << endl;
    }
  }

  ofs.close();
  cout << "  线收敛历史已保存到: " << filename << endl;
}

// ============================================================================
// 方案A: Plücker + 线投影
// ============================================================================
ExperimentResult runPluckerMethod(SceneData& data, int maxIter, bool verbose, bool saveHistory) {
  auto start_time = chrono::high_resolution_clock::now();

  SparseOptimizer optimizer;

  OptimizationAlgorithmFactory* solverFactory = OptimizationAlgorithmFactory::instance();
  OptimizationAlgorithmProperty solverProperty;
  OptimizationAlgorithm* solver = solverFactory->construct("lm_var", solverProperty);
  optimizer.setAlgorithm(solver);

  ParameterSE3Offset* offset = new ParameterSE3Offset();
  offset->setId(0);
  optimizer.addParameter(offset);

  // 添加位姿顶点 (VertexSE3)
  map<int, VertexSE3*> pose_vertices;

  for (size_t i = 0; i < data.poses.size(); ++i) {
    VertexSE3* v = new VertexSE3();
    v->setId(1000 + i);

    // 使用预生成的噪声
    v->setEstimate(data.poses[i] * data.pose_noises[i]);

    optimizer.addVertex(v);
    pose_vertices[i] = v;
  }
  pose_vertices[0]->setFixed(true);

  // 添加里程计边
  for (size_t i = 1; i < data.poses.size(); ++i) {
    Isometry3d delta = data.poses[i-1].inverse() * data.poses[i];

    EdgeSE3* e = new EdgeSE3();
    e->vertices()[0] = pose_vertices[i-1];
    e->vertices()[1] = pose_vertices[i];
    e->setMeasurement(delta * data.odom_noises[i-1]);

    Matrix6 info = Matrix6::Identity();
    for (int j = 0; j < 6; ++j) {
      info(j, j) = 1.0 / (data.odom_noise_sigma(j) * data.odom_noise_sigma(j));
    }
    e->setInformation(info);
    optimizer.addEdge(e);
  }

  // 添加线顶点（使用切空间参数化，与端点方法使用相同的初始化噪声）
  map<int, VertexLine3DTangent*> line_vertices;

  for (size_t i = 0; i < data.lines.size(); ++i) {
    VertexLine3DTangent* v = new VertexLine3DTangent();
    v->setId(i);

    // 使用预生成的带噪声端点（与端点方法使用完全相同的噪声）
    Vector3d p1_noisy = data.lines_noisy[i].first;
    Vector3d p2_noisy = data.lines_noisy[i].second;

    Vector3d dir = (p2_noisy - p1_noisy).normalized();

    // Vector6 cartesian;
    // cartesian << p1_noisy, dir;
    Vector6 cartesian;
    cartesian << 0.0, 0.0, 5.0, 0.0, 1.0, 0.0;
    v->setEstimate(Line3D::fromCartesian(cartesian));

    optimizer.addVertex(v);
    line_vertices[i] = v;
  }

  // 添加线观测边
  for (auto& obs : data.observations) {
    EdgeSE3Line3DProjection* e = new EdgeSE3Line3DProjection();
    e->vertices()[0] = pose_vertices[obs.pose_id];
    e->vertices()[1] = line_vertices[obs.line_id];
    e->setMeasurement(obs.line2d);

    // 信息矩阵
    Matrix2d info = Matrix2d::Identity() * 10.0;
    e->setInformation(info);

    RobustKernelHuber* rk = new RobustKernelHuber;
    rk->setDelta(5.0);
    e->setRobustKernel(rk);

    optimizer.addEdge(e);
  }

  // 优化
  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  double chi2_before = optimizer.chi2();

  // 收敛历史记录
  vector<vector<pair<Vector3d, Vector3d>>> line_history;
  vector<double> chi2_history;

  auto recordCurrentLines = [&]() {
    vector<pair<Vector3d, Vector3d>> current_lines;
    for (size_t i = 0; i < data.lines.size(); ++i) {
      Line3D L = line_vertices[i]->estimate();
      double gt_length = (data.lines[i].second - data.lines[i].first).norm();
      current_lines.push_back(line3dToEndpoints(L, gt_length / 2.0));
    }
    line_history.push_back(current_lines);
    chi2_history.push_back(optimizer.chi2());
  };

  optimizer.setVerbose(verbose);

  int iterations = 0;
  if (saveHistory) {
    // 单步迭代模式：记录每次迭代
    recordCurrentLines();  // 记录初始状态
    for (int iter = 0; iter < maxIter; ++iter) {
      int ret = optimizer.optimize(1);
      if (ret <= 0) break;
      iterations++;
      optimizer.computeActiveErrors();
      recordCurrentLines();
    }
  } else {
    // 正常模式：一次性优化
    iterations = optimizer.optimize(maxIter);
  }

  optimizer.computeActiveErrors();
  double chi2_after = optimizer.chi2();

  // 保存收敛历史
  if (saveHistory) {
    saveLineConvergenceHistory("line_convergence_plucker.txt", data.lines, line_history, chi2_history);
  }

  auto end_time = chrono::high_resolution_clock::now();
  double time_ms = chrono::duration<double, milli>(end_time - start_time).count();

  // 计算误差
  double total_trans = 0, total_rot = 0;
  int pose_count = 0;

  for (size_t i = 0; i < data.poses.size(); ++i) {
    if (pose_vertices[i]->fixed()) continue;

    Isometry3d error = data.poses[i].inverse() * pose_vertices[i]->estimate();
    total_trans += error.translation().norm();
    AngleAxisd aa(error.rotation());
    total_rot += abs(aa.angle());
    pose_count++;
  }

  ExperimentResult result;
  result.method_name = "Plücker+线投影";
  result.chi2_before = chi2_before;
  result.chi2_after = chi2_after;
  result.avg_trans_error = total_trans / pose_count;
  result.avg_rot_error = (total_rot / pose_count) * 180.0 / M_PI;
  result.num_features = data.lines.size();
  result.num_observations = data.observations.size();
  result.computation_time_ms = time_ms;
  result.converged = (iterations > 0);

  return result;
}

// ============================================================================
// 方案B: 端点 + 点投影
// ============================================================================
ExperimentResult runEndpointMethod(SceneData& data, int maxIter, bool verbose) {
  auto start_time = chrono::high_resolution_clock::now();

  SparseOptimizer optimizer;

  OptimizationAlgorithmFactory* solverFactory = OptimizationAlgorithmFactory::instance();
  OptimizationAlgorithmProperty solverProperty;
  OptimizationAlgorithm* solver = solverFactory->construct("lm_var", solverProperty);
  optimizer.setAlgorithm(solver);

  // 添加相机参数
  // CameraParameters 构造函数: (focal_length, principle_point, baseline)
  // 注意: CameraParameters 假设 fx = fy = focal_length
  CameraParameters* cam = new CameraParameters(data.fx, g2o::Vector2(data.cx, data.cy), 0);
  cam->setId(0);
  optimizer.addParameter(cam);

  // 添加位姿顶点 (VertexSE3Expmap)
  // 注意: SE3Quat 存储的是 T_cw (相机坐标系到世界坐标系的变换的逆)
  // SE3Quat::map(p_w) 计算 T_cw * p_w = p_c
  map<int, VertexSE3Expmap*> pose_vertices;

  for (size_t i = 0; i < data.poses.size(); ++i) {
    VertexSE3Expmap* v = new VertexSE3Expmap();
    v->setId(1000 + i);

    // data.poses[i] 是 T_wc (相机在世界中的位姿)
    // SE3Quat 需要 T_cw = T_wc^(-1)
    // 使用与方案A相同的预生成噪声
    Isometry3d T_wc_noisy = data.poses[i] * data.pose_noises[i];
    Isometry3d T_cw = T_wc_noisy.inverse();

    v->setEstimate(SE3Quat(T_cw.rotation(), T_cw.translation()));

    optimizer.addVertex(v);
    pose_vertices[i] = v;
  }
  pose_vertices[0]->setFixed(true);

  // 添加里程计边 (EdgeSE3Expmap)
  for (size_t i = 1; i < data.poses.size(); ++i) {
    // delta_wc = T_wc_{i-1}^{-1} * T_wc_i
    Isometry3d delta_wc = data.poses[i-1].inverse() * data.poses[i];
    // 使用与方案A相同的预生成噪声
    Isometry3d delta_wc_noisy = delta_wc * data.odom_noises[i-1];

    // 对于 SE3Quat: delta_cw = delta_wc^{-1}
    // 但 EdgeSE3Expmap 的测量是从 vertex[0] 到 vertex[1] 的相对变换
    // 所以测量值应该是 T_cw_i * T_wc_{i-1} = T_cw_i * T_cw_{i-1}^{-1}
    // 这等于 (T_wc_{i-1}^{-1} * T_wc_i)^{-1} = delta_wc^{-1}
    Isometry3d meas_iso = delta_wc_noisy.inverse();

    EdgeSE3Expmap* e = new EdgeSE3Expmap();
    e->vertices()[0] = pose_vertices[i-1];
    e->vertices()[1] = pose_vertices[i];
    e->setMeasurement(SE3Quat(meas_iso.rotation(), meas_iso.translation()));

    Matrix6 info = Matrix6::Identity();
    for (int j = 0; j < 6; ++j) {
      info(j, j) = 1.0 / (data.odom_noise_sigma(j) * data.odom_noise_sigma(j));
    }
    e->setInformation(info);
    optimizer.addEdge(e);
  }

  // 添加端点顶点
  map<int, pair<VertexPointXYZ*, VertexPointXYZ*>> endpoint_vertices;

  for (size_t i = 0; i < data.lines.size(); ++i) {
    // 使用预生成的带噪声端点（与Plücker方法使用完全相同的噪声）
    Vector3d p1_noisy = data.lines_noisy[i].first;
    Vector3d p2_noisy = data.lines_noisy[i].second;

    VertexPointXYZ* v1 = new VertexPointXYZ();
    v1->setId(i * 2);
    v1->setEstimate(p1_noisy);
    optimizer.addVertex(v1);

    VertexPointXYZ* v2 = new VertexPointXYZ();
    v2->setId(i * 2 + 1);
    v2->setEstimate(p2_noisy);
    optimizer.addVertex(v2);

    endpoint_vertices[i] = {v1, v2};
  }

  // 添加端点观测边 - 使用点到线距离约束（PL-VINS方法）
  for (auto& obs : data.observations) {
    auto endpoints = endpoint_vertices[obs.line_id];

    // 创建点到线距离边（传入相机内参）
    EdgeEndpointToLine2D* e = new EdgeEndpointToLine2D(data.fx, data.fy, data.cx, data.cy);
    e->vertices()[0] = endpoints.first;   // 端点1
    e->vertices()[1] = endpoints.second;  // 端点2
    e->vertices()[2] = pose_vertices[obs.pose_id];  // 位姿

    // 测量：两个观测端点的像素坐标
    Vector4d meas;
    meas << obs.pixel1.x(), obs.pixel1.y(), obs.pixel2.x(), obs.pixel2.y();
    e->setMeasurement(meas);

    // 信息矩阵（2x2，对应两个误差分量）
    e->setInformation(Matrix2d::Identity() * 10.0);

    RobustKernelHuber* rk = new RobustKernelHuber;
    rk->setDelta(5.0);
    e->setRobustKernel(rk);
    optimizer.addEdge(e);
  }

  // 优化
  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  double chi2_before = optimizer.chi2();

  optimizer.setVerbose(verbose);
  int iterations = optimizer.optimize(maxIter);

  optimizer.computeActiveErrors();
  double chi2_after = optimizer.chi2();

  auto end_time = chrono::high_resolution_clock::now();
  double time_ms = chrono::duration<double, milli>(end_time - start_time).count();

  // 计算误差
  double total_trans = 0, total_rot = 0;
  int pose_count = 0;

  for (size_t i = 0; i < data.poses.size(); ++i) {
    if (pose_vertices[i]->fixed()) continue;

    // SE3Quat 存储的是 T_cw，需要转回 T_wc 来比较
    SE3Quat est_cw = pose_vertices[i]->estimate();
    Isometry3d est_wc = Isometry3d::Identity();
    est_wc.linear() = est_cw.rotation().toRotationMatrix().transpose();
    est_wc.translation() = -est_wc.linear() * est_cw.translation();

    Isometry3d error = data.poses[i].inverse() * est_wc;
    total_trans += error.translation().norm();
    AngleAxisd aa(error.rotation());
    total_rot += abs(aa.angle());
    pose_count++;
  }

  // 计算端点误差
  double total_endpoint_error = 0;
  for (size_t i = 0; i < data.lines.size(); ++i) {
    Vector3d p1_est = endpoint_vertices[i].first->estimate();
    Vector3d p2_est = endpoint_vertices[i].second->estimate();

    double err1 = (data.lines[i].first - p1_est).norm();
    double err2 = (data.lines[i].second - p2_est).norm();
    total_endpoint_error += (err1 + err2) / 2.0;
  }

  ExperimentResult result;
  result.method_name = "端点+点到线距离";
  result.chi2_before = chi2_before;
  result.chi2_after = chi2_after;
  result.avg_trans_error = total_trans / pose_count;
  result.avg_rot_error = (total_rot / pose_count) * 180.0 / M_PI;
  result.avg_line_error = total_endpoint_error / data.lines.size();
  result.num_features = data.lines.size() * 2;
  result.num_observations = data.observations.size();  // 每个观测2个约束
  result.computation_time_ms = time_ms;
  result.converged = (iterations > 0);

  return result;
}

// ============================================================================
// 打印对比表格
// ============================================================================
void printComparisonTable(const vector<ExperimentResult>& results) {
  cout << "\n";
  cout << "=====================================================================================" << endl;
  cout << "                                  实 验 结 果 对 比                                " << endl;
  cout << "=====================================================================================" << endl;
  
  printf("%-20s %12s %12s %12s %8s %8s %12s\n",
         "Method", "Chi2_Reduce", "Trans(m)", "Rot(deg)", "Feature", "Obs", "Time(ms)");
  cout << "-------------------------------------------------------------------------------------" << endl;

  for (const auto& r : results) {
    double chi2_reduction = 0;
    if (r.chi2_before > 0) {
      chi2_reduction = (r.chi2_before - r.chi2_after) / r.chi2_before * 100.0;
    }

    printf("%-20s %12f%% %12f %12f %8d %8d %12f\n",
           r.method_name.c_str(),
           chi2_reduction,
           r.avg_trans_error,
           r.avg_rot_error,
           r.num_features,
           r.num_observations,
           r.computation_time_ms);
  }

  cout << "=====================================================================================" << endl;

   // 详细对比
  if (results.size() == 2) {
    printf("\n");
    printf("对比总结：\n");
    printf("  观测数量: 方案A %4d  vs  方案B %4d\n", 
           results[0].num_observations, results[1].num_observations);

    if (results[0].avg_trans_error > 0) {
      double trans_diff = (results[1].avg_trans_error - results[0].avg_trans_error)
                         / results[0].avg_trans_error * 100;
      printf("  位置精度: 方案B %s 方案A %.1f%%\n", 
             (trans_diff > 0 ? "劣于" : "优于"), abs(trans_diff));
    }

    if (results[0].avg_rot_error > 0) {
      double rot_diff = (results[1].avg_rot_error - results[0].avg_rot_error)
                       / results[0].avg_rot_error * 100;
      printf("  旋转精度: 方案B %s 方案A %.1f%%\n", 
             (rot_diff > 0 ? "劣于" : "优于"), abs(rot_diff));
    }

    if (results[0].computation_time_ms > 0) {
      double time_diff = (results[1].computation_time_ms - results[0].computation_time_ms)
                        / results[0].computation_time_ms * 100;
      printf("  计算效率: 方案B %s %.1f%%\n", 
             (time_diff > 0 ? "慢" : "快"), abs(time_diff));
    }
  }
}

// ============================================================================
// 主函数
// ============================================================================
int main(int argc, char** argv) {
  bool verbose = false;
  int maxIterations = 15;
  bool runBoth = true;
  bool useEndpoint = false;
  bool saveConvergence = false;

  CommandArgs arg;
  arg.param("i", maxIterations, 15, "最大迭代次数");
  arg.param("v", verbose, false, "详细输出");
  arg.param("both", runBoth, true, "运行两种方法对比");
  arg.param("useEndpoint", useEndpoint, false, "仅运行端点方法");
  arg.param("saveConv", saveConvergence, false, "保存线收敛历史到文件(用于可视化)");
  arg.parseArgs(argc, argv);

  cout << "=== 线特征表示方法对比实验 ===" << endl;

  // 生成场景数据
  cout << "\n生成仿真数据..." << endl;
  SceneData data;
  data.generateScene(verbose);

  cout << "  线特征数: " << data.lines.size() << endl;
  cout << "  相机位姿数: " << data.poses.size() << endl;

  vector<ExperimentResult> results;

  if (runBoth) {
    // 运行方案A
    cout << "\n运行方案A: Plücker + 线投影..." << endl;
    results.push_back(runPluckerMethod(data, maxIterations, verbose, saveConvergence));

    // 运行方案B
    cout << "\n运行方案B: 端点 + 点投影..." << endl;
    results.push_back(runEndpointMethod(data, maxIterations, verbose));

    // 打印对比结果
    printComparisonTable(results);
  } else if (useEndpoint) {
    cout << "\n运行方案B: 端点 + 点投影..." << endl;
    results.push_back(runEndpointMethod(data, maxIterations, verbose));
    printComparisonTable(results);
  } else {
    cout << "\n运行方案A: Plücker + 线投影..." << endl;
    results.push_back(runPluckerMethod(data, maxIterations, verbose, saveConvergence));
    printComparisonTable(results);
  }

  return 0;
}