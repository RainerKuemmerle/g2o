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

## License
* The binding code and python example code is licensed under BSD License.
