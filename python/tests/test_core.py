"""Tests for g2o core functionality (optimizer, solvers, algorithms)."""

# g2o will be available after conftest.py sets sys.path
import g2opy as g2o
import numpy as np
import pytest


class TestSparseOptimizer:
    """Test SparseOptimizer basic functionality."""

    def test_create_optimizer(self):
        """Test creating a SparseOptimizer."""
        optimizer = g2o.SparseOptimizer()
        assert optimizer is not None
        assert len(optimizer.vertices()) == 0
        assert len(optimizer.edges()) == 0

    def test_add_vertex(self, basic_se2_optimizer):
        """Test adding vertices to optimizer."""
        v = g2o.VertexSE2()
        v.set_id(0)
        v.set_estimate(g2o.SE2(0, 0, 0))

        basic_se2_optimizer.add_vertex(v)
        assert len(basic_se2_optimizer.vertices()) == 1
        assert basic_se2_optimizer.vertex(0) is not None

    def test_add_edge(self, simple_se2_graph):
        """Test adding edges to optimizer."""
        assert len(simple_se2_graph.edges()) == 2

    def test_vertex_retrieval(self, simple_se2_graph):
        """Test retrieving vertices."""
        for i in range(3):
            v = simple_se2_graph.vertex(i)
            assert v is not None
            assert v.id() == i

    def test_chi2_compute(self, simple_se2_graph):
        """Test computing chi2 error."""
        initial_chi2 = simple_se2_graph.chi2()
        assert initial_chi2 >= 0

    def test_initialize_optimization(self, simple_se2_graph):
        """Test optimization initialization."""
        simple_se2_graph.initialize_optimization()
        # Should not raise an exception

    def test_set_verbose(self, basic_se2_optimizer):
        """Test setting verbose output."""
        basic_se2_optimizer.set_verbose(True)
        basic_se2_optimizer.set_verbose(False)
        # Should not raise an exception


class TestBlockSolvers:
    """Test BlockSolver configurations."""

    def test_block_solver_se2(self):
        """Test BlockSolverSE2."""
        solver = g2o.BlockSolverSE2(g2o.LinearSolverEigenSE2())
        assert solver is not None

    def test_block_solver_se3(self):
        """Test BlockSolverSE3."""
        solver = g2o.BlockSolverSE3(g2o.LinearSolverEigenSE3())
        assert solver is not None

    def test_block_solver_variable(self):
        """Test variable dimension BlockSolverX."""
        solver = g2o.BlockSolverX(g2o.LinearSolverEigenX())
        assert solver is not None


class TestLinearSolvers:
    """Test various linear solver implementations."""

    def test_linear_solver_eigen_se2(self):
        """Test LinearSolverEigenSE2."""
        solver = g2o.LinearSolverEigenSE2()
        assert solver is not None

    def test_linear_solver_eigen_se3(self):
        """Test LinearSolverEigenSE3."""
        solver = g2o.LinearSolverEigenSE3()
        assert solver is not None

    def test_linear_solver_eigen_x(self):
        """Test LinearSolverEigenX (variable dimension)."""
        solver = g2o.LinearSolverEigenX()
        assert solver is not None


class TestOptimizationAlgorithms:
    """Test optimization algorithm implementations."""

    def test_levenberg_marquardt(self, basic_se2_optimizer):
        """Test Levenberg-Marquardt algorithm."""
        assert basic_se2_optimizer.algorithm() is not None

    def test_gauss_newton_creation(self):
        """Test creating Gauss-Newton algorithm."""
        solver = g2o.BlockSolverSE2(g2o.LinearSolverEigenSE2())
        algo = g2o.OptimizationAlgorithmGaussNewton(solver)
        assert algo is not None

    def test_dogleg_creation(self):
        """Test creating Dogleg algorithm."""
        solver = g2o.BlockSolverSE2(g2o.LinearSolverEigenSE2())
        algo = g2o.OptimizationAlgorithmDogleg(solver)
        assert algo is not None


class TestErrorComputation:
    """Test error and chi2 computation."""

    def test_compute_active_errors(self, simple_se2_graph):
        """Test computing active errors."""
        simple_se2_graph.compute_active_errors()
        chi2 = simple_se2_graph.chi2()
        assert chi2 >= 0

    def test_chi2_before_optimization(self, simple_se2_graph):
        """Test chi2 computation before optimization."""
        simple_se2_graph.initialize_optimization()
        simple_se2_graph.compute_active_errors()
        initial_chi2 = simple_se2_graph.chi2()

        simple_se2_graph.optimize(1)
        simple_se2_graph.compute_active_errors()
        final_chi2 = simple_se2_graph.chi2()

        # Chi2 should improve or stay the same
        assert final_chi2 <= initial_chi2 + 1e-6
