# g2opy

This is a python binding of [g2o](https://github.com/RainerKuemmerle/g2o).

The code here is based on https://github.com/uoip/g2opy.git by qihang@outlook.com

Currently, this project doesn't support writing user-defined types in python,
but the predefined types are enough to implement the most common algorithms,
say **PnP, ICP, Bundle Adjustment and Pose Graph Optimization** in 2d or 3d
scenarios. g2o's visualization part is not wrapped, if you want to visualize
point clouds or graph, you can give
[pangolin](https://github.com/uoip/pangolin) a try, it's a python binding of
C++ library [Pangolin](http://github.com/stevenlovegrove/Pangolin).

For convenience, some frequently used Eigen types (Quaternion, Rotation2d,
Isometry3d, Isometry2d, AngleAxis) are packed into this library.  

## Requirements
* ([pybind11](https://github.com/pybind/pybind11)


## Installation
```
git clone https://github.com/uoip/g2opy.git
cd g2opy
mkdir build
cd build
cmake ..
make -j8
cd ..
python setup.py install
```
Tested under Ubuntu 16.04, Python 3.6+.


## Get Started
The code snippets below show the core parts of BA and Pose Graph Optimization in a SLAM system.
#### Bundle Adjustment
```python
import numpy
import g2o

class BundleAdjustment(g2o.SparseOptimizer):
    def __init__(self, ):
        super().__init__()
        solver = g2o.BlockSolverSE3(g2o.LinearSolverCSparseSE3())
        solver = g2o.OptimizationAlgorithmLevenberg(solver)
        super().set_algorithm(solver)

    def optimize(self, max_iterations=10):
        super().initialize_optimization()
        super().optimize(max_iterations)

    def add_pose(self, pose_id, pose, cam, fixed=False):
        sbacam = g2o.SBACam(pose.orientation(), pose.position())
        sbacam.set_cam(cam.fx, cam.fy, cam.cx, cam.cy, cam.baseline)

        v_se3 = g2o.VertexCam()
        v_se3.set_id(pose_id * 2)   # internal id
        v_se3.set_estimate(sbacam)
        v_se3.set_fixed(fixed)
        super().add_vertex(v_se3) 

    def add_point(self, point_id, point, fixed=False, marginalized=True):
        v_p = g2o.VertexSBAPointXYZ()
        v_p.set_id(point_id * 2 + 1)
        v_p.set_estimate(point)
        v_p.set_marginalized(marginalized)
        v_p.set_fixed(fixed)
        super().add_vertex(v_p)

    def add_edge(self, point_id, pose_id, 
            measurement,
            information=np.identity(2),
            robust_kernel=g2o.RobustKernelHuber(np.sqrt(5.991))):   # 95% CI

        edge = g2o.EdgeProjectP2MC()
        edge.set_vertex(0, self.vertex(point_id * 2 + 1))
        edge.set_vertex(1, self.vertex(pose_id * 2))
        edge.set_measurement(measurement)   # projection
        edge.set_information(information)

        if robust_kernel is not None:
            edge.set_robust_kernel(robust_kernel)
        super().add_edge(edge)

    def get_pose(self, pose_id):
        return self.vertex(pose_id * 2).estimate()

    def get_point(self, point_id):
        return self.vertex(point_id * 2 + 1).estimate()
```

#### Pose Graph Optimization
```python
import numpy
import g2o

class PoseGraphOptimization(g2o.SparseOptimizer):
    def __init__(self):
        super().__init__()
        solver = g2o.BlockSolverSE3(g2o.LinearSolverCholmodSE3())
        solver = g2o.OptimizationAlgorithmLevenberg(solver)
        super().set_algorithm(solver)

    def optimize(self, max_iterations=20):
        super().initialize_optimization()
        super().optimize(max_iterations)

    def add_vertex(self, id, pose, fixed=False):
        v_se3 = g2o.VertexSE3()
        v_se3.set_id(id)
        v_se3.set_estimate(pose)
        v_se3.set_fixed(fixed)
        super().add_vertex(v_se3)

    def add_edge(self, vertices, measurement, 
            information=np.identity(6),
            robust_kernel=None):

        edge = g2o.EdgeSE3()
        for i, v in enumerate(vertices):
            if isinstance(v, int):
                v = self.vertex(v)
            edge.set_vertex(i, v)

        edge.set_measurement(measurement)  # relative pose
        edge.set_information(information)
        if robust_kernel is not None:
            edge.set_robust_kernel(robust_kernel)
        super().add_edge(edge)

    def get_pose(self, id):
        return self.vertex(id).estimate()
```

For more details, checkout [python examples](python/examples).  Thanks to
[pybind11](https://github.com/pybind/pybind11), g2opy works seamlessly between
numpy and underlying Eigen.  

## License
* The binding code and python example code is licensed under BSD License.  
