"""Tests for optimization routines."""

# g2o will be available after conftest.py sets sys.path
import g2opy as g2o
import numpy as np
import pytest


class TestBasicOptimization:
    """Test basic optimization workflows."""

    def test_optimize_se2_graph(self, simple_se2_graph):
        """Test optimizing SE2 graph."""
        simple_se2_graph.initialize_optimization()
        simple_se2_graph.compute_active_errors()
        initial_chi2 = simple_se2_graph.chi2()

        num_iterations = 5
        result = simple_se2_graph.optimize(num_iterations)

        # Should return True for successful optimization
        assert result

        simple_se2_graph.compute_active_errors()
        final_chi2 = simple_se2_graph.chi2()

        # Chi2 should improve or stay same (allowing for numerical precision)
        assert final_chi2 <= initial_chi2 + 1e-6

    def test_optimize_se3_graph(self, simple_se3_graph):
        """Test optimizing SE3 graph."""
        simple_se3_graph.initialize_optimization()
        simple_se3_graph.compute_active_errors()
        initial_chi2 = simple_se3_graph.chi2()

        result = simple_se3_graph.optimize(5)
        assert result

        simple_se3_graph.compute_active_errors()
        final_chi2 = simple_se3_graph.chi2()
        assert final_chi2 <= initial_chi2 + 1e-6

    def test_zero_iterations(self, simple_se2_graph):
        """Test optimizing with zero iterations."""
        simple_se2_graph.initialize_optimization()
        simple_se2_graph.compute_active_errors()
        initial_chi2 = simple_se2_graph.chi2()

        simple_se2_graph.optimize(0)

        simple_se2_graph.compute_active_errors()
        final_chi2 = simple_se2_graph.chi2()

        # Chi2 should not change with zero iterations
        assert abs(final_chi2 - initial_chi2) < 1e-10


class TestAlgorithmComparison:
    """Compare different optimization algorithms."""

    def test_levenberg_marquardt_optimization(self, simple_se2_graph):
        """Test Levenberg-Marquardt optimization."""
        simple_se2_graph.initialize_optimization()
        result = simple_se2_graph.optimize(5)
        assert result

    def test_gauss_newton_optimization(self):
        """Test Gauss-Newton optimization."""
        optimizer = g2o.SparseOptimizer()
        solver = g2o.BlockSolverSE2(g2o.LinearSolverEigenSE2())
        optimizer.set_algorithm(g2o.OptimizationAlgorithmGaussNewton(solver))

        # Build simple graph
        v = g2o.VertexSE2()
        v.set_id(0)
        v.set_estimate(g2o.SE2(0, 0, 0))
        v.set_fixed(True)
        optimizer.add_vertex(v)

        v2 = g2o.VertexSE2()
        v2.set_id(1)
        v2.set_estimate(g2o.SE2(1.5, 0.5, 0.1))
        optimizer.add_vertex(v2)

        edge = g2o.EdgeSE2()
        edge.set_vertex(0, v)
        edge.set_vertex(1, v2)
        edge.set_measurement(g2o.SE2(1, 0, 0))
        edge.set_information(np.eye(3))
        optimizer.add_edge(edge)

        optimizer.initialize_optimization()
        result = optimizer.optimize(3)
        assert result

    def test_dogleg_optimization(self):
        """Test Dogleg optimization."""
        optimizer = g2o.SparseOptimizer()
        solver = g2o.BlockSolverSE2(g2o.LinearSolverEigenSE2())
        optimizer.set_algorithm(g2o.OptimizationAlgorithmDogleg(solver))

        v = g2o.VertexSE2()
        v.set_id(0)
        v.set_estimate(g2o.SE2(0, 0, 0))
        v.set_fixed(True)
        optimizer.add_vertex(v)

        v2 = g2o.VertexSE2()
        v2.set_id(1)
        v2.set_estimate(g2o.SE2(1.5, 0.5, 0.1))
        optimizer.add_vertex(v2)

        edge = g2o.EdgeSE2()
        edge.set_vertex(0, v)
        edge.set_vertex(1, v2)
        edge.set_measurement(g2o.SE2(1, 0, 0))
        edge.set_information(np.eye(3))
        optimizer.add_edge(edge)

        optimizer.initialize_optimization()
        result = optimizer.optimize(3)
        assert result


class TestLargerGraph:
    """Test optimization on larger graphs."""

    def test_optimize_chain_graph(self):
        """Test optimizing a chain of poses."""
        optimizer = g2o.SparseOptimizer()
        solver = g2o.BlockSolverSE2(g2o.LinearSolverEigenSE2())
        optimizer.set_algorithm(g2o.OptimizationAlgorithmLevenberg(solver))

        # Create chain of 10 poses
        num_poses = 10
        for i in range(num_poses):
            v = g2o.VertexSE2()
            v.set_id(i)
            v.set_estimate(g2o.SE2(i * 0.5, 0, 0))
            if i == 0:
                v.set_fixed(True)
            optimizer.add_vertex(v)

        # Connect consecutive poses
        for i in range(num_poses - 1):
            edge = g2o.EdgeSE2()
            edge.set_vertex(0, optimizer.vertex(i))
            edge.set_vertex(1, optimizer.vertex(i + 1))
            edge.set_measurement(g2o.SE2(0.5, 0, 0))
            edge.set_information(np.eye(3))
            optimizer.add_edge(edge)

        assert len(optimizer.vertices()) == num_poses
        assert len(optimizer.edges()) == num_poses - 1

        optimizer.initialize_optimization()
        result = optimizer.optimize(10)
        assert result


class TestOptimizationWithNoise:
    """Test optimization with noisy measurements."""

    def test_optimize_noisy_measurements(self):
        """Test optimization with noisy edge measurements."""
        np.random.seed(42)

        optimizer = g2o.SparseOptimizer()
        solver = g2o.BlockSolverSE2(g2o.LinearSolverEigenSE2())
        optimizer.set_algorithm(g2o.OptimizationAlgorithmLevenberg(solver))

        # Fixed first pose
        v0 = g2o.VertexSE2()
        v0.set_id(0)
        v0.set_estimate(g2o.SE2(0, 0, 0))
        v0.set_fixed(True)
        optimizer.add_vertex(v0)

        # Second pose with noise
        v1 = g2o.VertexSE2()
        v1.set_id(1)
        v1.set_estimate(g2o.SE2(1.2, 0.1, 0.05))  # Noisy initial estimate
        optimizer.add_vertex(v1)

        # Edge with noisy measurement
        edge = g2o.EdgeSE2()
        edge.set_vertex(0, v0)
        edge.set_vertex(1, v1)
        # True measurement should be (1, 0, 0) but we add noise
        edge.set_measurement(g2o.SE2(1.05, 0.02, 0.01))
        edge.set_information(np.eye(3) * 10)  # High confidence
        optimizer.add_edge(edge)

        optimizer.initialize_optimization()
        optimizer.compute_active_errors()
        initial_chi2 = optimizer.chi2()

        optimizer.optimize(10)
        optimizer.compute_active_errors()
        final_chi2 = optimizer.chi2()

        # Should improve
        assert final_chi2 < initial_chi2
