"""Pytest fixtures for g2o Python bindings tests."""

import os
import sys
import tempfile
from pathlib import Path

# Ensure build lib is in path BEFORE any imports
build_lib_dir = os.environ.get(
    "G2O_LIB_DIR", str(Path(__file__).resolve().parent.parent.parent / "build" / "lib")
)
sys.path.insert(0, build_lib_dir)

# This must come after path setup
import g2opy as g2o
import numpy as np
import pytest


def pytest_runtest_setup(item):
    """Inject g2o into test module globals before each test runs."""
    if item.module:
        item.module.g2o = g2o


@pytest.fixture
def g2o_module():
    """Provide access to g2o module."""
    return g2o


@pytest.fixture
def temp_g2o_file():
    """Create a temporary file for saving .g2o graphs."""
    with tempfile.NamedTemporaryFile(suffix=".g2o", delete=False) as f:
        yield f.name
    Path(f.name).unlink(missing_ok=True)


@pytest.fixture
def basic_se2_optimizer():
    """Create a basic SE2 optimizer ready for optimization."""
    optimizer = g2o.SparseOptimizer()
    solver = g2o.BlockSolverSE2(g2o.LinearSolverEigenSE2())
    optimizer.set_algorithm(g2o.OptimizationAlgorithmLevenberg(solver))
    return optimizer


@pytest.fixture
def basic_se3_optimizer():
    """Create a basic SE3 optimizer ready for optimization."""
    optimizer = g2o.SparseOptimizer()
    solver = g2o.BlockSolverSE3(g2o.LinearSolverEigenSE3())
    optimizer.set_algorithm(g2o.OptimizationAlgorithmLevenberg(solver))
    return optimizer


@pytest.fixture
def simple_se2_graph(basic_se2_optimizer):
    """Create a simple 2D SLAM graph with 3 poses and 2 edges."""
    optimizer = basic_se2_optimizer

    # Add 3 vertices
    poses = [
        g2o.SE2(0, 0, 0),
        g2o.SE2(1, 0, 0),
        g2o.SE2(1, 1, np.pi / 2),
    ]

    for i, pose in enumerate(poses):
        v = g2o.VertexSE2()
        v.set_id(i)
        v.set_estimate(pose)
        if i == 0:
            v.set_fixed(True)  # Fix first vertex
        optimizer.add_vertex(v)

    # Add 2 edges connecting consecutive poses
    measurements = [
        g2o.SE2(1, 0, 0),
        g2o.SE2(0, 1, np.pi / 2),
    ]

    for i, measurement in enumerate(measurements):
        edge = g2o.EdgeSE2()
        edge.set_vertex(0, optimizer.vertex(i))
        edge.set_vertex(1, optimizer.vertex(i + 1))
        edge.set_measurement(measurement)
        edge.set_information(np.eye(3))
        optimizer.add_edge(edge)

    return optimizer


@pytest.fixture
def simple_se3_graph(basic_se3_optimizer):
    """Create a simple 3D SLAM graph with 2 poses and 1 edge."""
    optimizer = basic_se3_optimizer

    # Add 2 SE3 vertices
    poses = [
        g2o.Isometry3d(np.eye(3), [0, 0, 0]),
        g2o.Isometry3d(np.eye(3), [1, 0, 0]),
    ]

    for i, pose in enumerate(poses):
        v = g2o.VertexSE3()
        v.set_id(i)
        v.set_estimate(pose)
        if i == 0:
            v.set_fixed(True)
        optimizer.add_vertex(v)

    # Add edge
    edge = g2o.EdgeSE3()
    edge.set_vertex(0, optimizer.vertex(0))
    edge.set_vertex(1, optimizer.vertex(1))
    edge.set_measurement(g2o.Isometry3d(np.eye(3), [1, 0, 0]))
    edge.set_information(np.eye(6))
    optimizer.add_edge(edge)

    return optimizer


@pytest.fixture
def vector_vertex():
    """Create a VectorXVertex for testing."""
    vertex = g2o.VectorXVertex()
    vertex.set_dimension(3)
    vertex.set_estimate(np.array([1.0, 2.0, 3.0]))
    return vertex


@pytest.fixture
def vector_edge(vector_vertex):
    """Create a VariableVectorXEdge for testing."""
    edge = g2o.VariableVectorXEdge()
    edge.set_dimension(3)
    edge.resize(1)
    edge.set_vertex(0, vector_vertex)
    edge.set_measurement(np.array([1.5, 2.5, 3.5]))
    edge.set_information(np.eye(3))
    return edge
