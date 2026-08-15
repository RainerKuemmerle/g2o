# g2opy - Python Bindings for g2o

This is a Python binding of [g2o](https://github.com/RainerKuemmerle/g2o), a C++ library for graph-based optimization.

## Features

g2opy provides high-level Python bindings for most common g2o use cases:

- **Optimization Algorithms**: Gauss-Newton, Levenberg-Marquardt, Dogleg
- **Linear Solvers**: Dense, Eigen, PCG, CSparse, CHOLMOD
- **SLAM Types**:
  - **2D SLAM**: SE2 poses, XY points, pose-to-pose/point constraints
  - **3D SLAM**: SE3 poses, XYZ points, pose-to-pose/point constraints
  - **SLAM Priors**: Pose priors, point priors, offset constraints
- **Bundle Adjustment (SBA)**: Multiple camera models and projection edges
- **SIM3 Optimization**: Similarity transforms with scale
- **ICP**: Generalized Iterative Closest Point (GiCP)
- **Sensor Calibration**: SCLAM2D and multi-sensor calibration

## Usage

### Basic Optimization Example

```python
import g2opy

# Create optimizer
optimizer = g2opy.SparseOptimizer()
solver = g2opy.BlockSolverSE2(g2opy.LinearSolverCholmodSE2())
algorithm = g2opy.OptimizationAlgorithmLevenberg(solver)
optimizer.set_algorithm(algorithm)

# Add vertices and edges
v1 = g2opy.VertexSE2()
v1.set_id(0)
v1.set_estimate(g2opy.SE2(0, 0, 0))
optimizer.add_vertex(v1)

# ... add more vertices and edges ...

# Optimize
optimizer.initialize_optimization()
optimizer.optimize(10)
```

### Creating Types by Factory

You can dynamically create vertices and edges using the Factory pattern:

```python
factory = g2opy.Factory()

# List all available types
all_types = factory.known_types()
print("Available types:", all_types)

# Create edge by type name
edge = factory.construct("EDGE_SE2")  # Creates EdgeSE2
```

You can also obtain information about a type:

```python
factory = g2opy.Factory()
info = factory.type_info("EDGE_SE2")
print(f"Dimension: {info.dimension}")
print(f"Error dimension: {info.error_dimension}")
```

### User-Defined Types

For dynamic optimization variables, you can use `VectorXVertex` and `VariableVectorXEdge`:

```python
import g2opy
import numpy as np

# Create a dynamic vertex
vertex = g2opy.VectorXVertex()
vertex.set_id(0)
vertex.set_dimension(3)  # 3-dimensional
vertex.set_estimate(np.array([1.0, 2.0, 3.0]))

# Create a dynamic edge
edge = g2opy.VariableVectorXEdge()
edge.resize(1)  # number of vertices
edge.set_dimension(1)  # dimension of the error function
edge.set_vertex(0, vertex)
edge.set_measurement(np.array([1.5, 2.5, 3.5]))
edge.set_information(np.eye(3))

# Subclass for custom behavior
class CustomEdge(g2opy.VariableVectorXEdge):
    def __init__(self) -> None:
        super().__init__()
        self.set_dimension(1)  # dimension of the error function
        self.information()
        self.resize(1)  # number of vertices
        self.set_measurement([0, 0])  # initial measurement

    def compute_error(self):
        # Custom error computation
        v = self.vertex(0).estimate()
        measurement = self.measurement()
        self.error = v - measurement

    def linearize_oplus(self):
        # Custom linearization
        self.set_jacobian(0, -np.eye(len(self.vertex(0).estimate())))
```

## Core Classes

### Optimizer

- `SparseOptimizer`: Main optimization framework
- `OptimizationAlgorithm`: Base class for optimization algorithms
  - `OptimizationAlgorithmGaussNewton`
  - `OptimizationAlgorithmLevenberg`
  - `OptimizationAlgorithmDogleg`

### Solvers

- `BlockSolver`: Base template for block solvers
  - `BlockSolverSE2`, `BlockSolverSE3`, `BlockSolverSim3`
- `LinearSolver`: Base class for linear solvers
  - `LinearSolverDense`, `LinearSolverEigen`
  - `LinearSolverPCG`, `LinearSolverCSparse`, `LinearSolverCholmod`

### Graph Elements

- `HyperGraph`: Graph structure
- `OptimizableGraph`: Base graph with vertices and edges
- `VertexContainer`: Base vertex class
- `EdgeContainer`: Base edge class

### Eigen Types

- `Quaternion`: Double-precision quaternion
- `Isometry2d`, `Isometry3d`: Rigid transformations
- `Rotation2d`: 2D rotation
- `AngleAxis`: Angle-axis representation
- `SE2`, `SE3Quat`: Special Euclidean groups

### Factory

- `Factory`: Dynamic type creation and registration
  - `construct(tag)`: Create vertex/edge by registered name
  - `known_types()`: List all registered types
  - `type_info(tag)`: Get dimension and structure information

### Parameters

- `Parameter`: Global optimization parameters
- `ParameterContainer`: Parameter storage
- Support for calibration parameters (camera intrinsics, SE2/SE3 offsets)

## Type Groups

Types are organized by category:

- **SLAM 2D**: `VertexSE2`, `VertexPointXY`, `EdgeSE2`, `EdgeSE2PointXY`, etc.
- **SLAM 3D**: `VertexSE3`, `VertexPointXYZ`, `EdgeSE3`, `EdgeSE3PointXYZ`, etc.
- **Bundle Adjustment**: `EdgeProjectXYZ2UV`, `EdgeStereoSE3ProjectXYZ`, etc.
- **SIM3**: `VertexSim3Expmap`, `EdgeSim3`, `EdgeSim3ProjectXYZ`
- **ICP**: `VertexStereoCamera`, `EdgeGiCP`, `EdgeStereoCamera`
- **SCLAM2D**: `VertexOdomDifferentialParams`, `EdgeSE2SensorCalib`

## Requirements & Building

Requirements:

- Python 3.11+
- Eigen3
- pybind11 (included via CMake FetchContent)
- g2o (from this repository)

## Building

The Python bindings are built as part of the main g2o CMake build:

```bash
cd g2o/build
cmake .. -DBUILD_PYTHON_BINDINGS=ON
make -j4
```

The compiled module will be in `build/lib/g2opy.*.so`.

## Performance Notes

### Threading

g2o operations that can take significant time release the Python GIL:
- `optimizer.optimize()` - Main optimization loop
- `optimizer.initialize_optimization()` - Graph initialization
- `optimizer.compute_marginals()` - Covariance computation
- `optimizer.compute_active_errors()` - Error evaluation

This allows other Python threads to run while optimization is in progress.

### Memory Management

The binding uses modern C++ memory management:
- Smart pointers for ownership clarity
- Automatic cleanup of resources
- Minimal copying of Eigen matrices

### Type Registration

All g2o types are automatically registered when the module loads.
The `Factory` class provides access to this registry for dynamic creation.

## References

- [Examples](examples)
- [g2o Repository](https://github.com/RainerKuemmerle/g2o)
- [g2o Paper](http://ais.informatik.uni-freiburg.de/publications/papers/kuemmerle11icra.pdf)
- [pybind11 Documentation](https://pybind11.readthedocs.io/)

## Contributing

Improvements and bug reports are welcome! Please report issues to the
[g2o project](https://github.com/RainerKuemmerle/g2o).

## License

The binding code and Python examples are licensed under the BSD License.
The g2o library itself is licensed under BSD and LGPL (see main repository).

## Acknowledgments

The binding code is originally based on [g2opy](https://github.com/uoip/g2opy.git) by qihang@outlook.com.
