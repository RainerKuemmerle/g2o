"""Tests for custom user-defined types (VectorXVertex, VariableVectorXEdge)."""

import g2opy as g2o
import numpy as np
import pytest


class TestVectorXVertex:
    """Test VectorXVertex (dynamic dimension vertex)."""

    def test_create_vector_vertex(self):
        """Test creating VectorXVertex."""
        v = g2o.VectorXVertex()
        assert v is not None

    def test_vector_vertex_dimension(self):
        """Test setting dimension."""
        v = g2o.VectorXVertex()
        v.set_dimension(5)
        assert v.dimension() == 5

    def test_vector_vertex_estimate(self):
        """Test setting and getting estimate."""
        v = g2o.VectorXVertex()
        v.set_dimension(3)

        estimate = np.array([1.0, 2.0, 3.0])
        v.set_estimate(estimate)

        retrieved = v.estimate()
        assert np.allclose(retrieved, estimate)

    def test_vector_vertex_id(self):
        """Test ID management."""
        v = g2o.VectorXVertex()
        v.set_id(999)
        assert v.id() == 999

    def test_vector_vertex_fixed(self):
        """Test fixing vertices."""
        v = g2o.VectorXVertex()
        v.set_fixed(True)
        assert v.fixed()


class CustomVectorVertex(g2o.VectorXVertex):
    """Custom vertex subclass for testing."""

    def __init__(self):
        super().__init__()
        self.set_dimension(2)
        self.set_estimate(np.zeros(2))

    def oplus_impl(self, update):
        """Custom oplus implementation."""
        self.set_estimate(self.estimate() + update)


class TestCustomVertex:
    """Test subclassing VectorXVertex."""

    def test_custom_vertex_creation(self):
        """Test creating custom vertex subclass."""
        v = CustomVectorVertex()
        assert v.dimension() == 2

    def test_custom_vertex_estimate(self):
        """Test custom vertex estimate."""
        v = CustomVectorVertex()
        estimate = np.array([5.0, 7.0])
        v.set_estimate(estimate)

        retrieved = v.estimate()
        assert np.allclose(retrieved, estimate)

    def test_custom_vertex_in_optimizer(self):
        """Test adding custom vertex to optimizer."""
        optimizer = g2o.SparseOptimizer()

        v = CustomVectorVertex()
        v.set_id(0)
        optimizer.add_vertex(v)

        assert len(optimizer.vertices()) == 1


class TestVariableVectorXEdge:
    """Test VariableVectorXEdge (dynamic dimension edge)."""

    def test_create_variable_edge(self, vector_vertex):
        """Test creating VariableVectorXEdge."""
        edge = g2o.VariableVectorXEdge()
        assert edge is not None

    def test_variable_edge_dimension(self, vector_vertex):
        """Test setting edge dimension."""
        edge = g2o.VariableVectorXEdge()
        edge.set_dimension(2)
        assert edge.dimension() == 2

    def test_variable_edge_resize(self, vector_vertex):
        """Test resizing edges (number of vertices)."""
        edge = g2o.VariableVectorXEdge()
        edge.resize(1)
        # Should succeed without exception

    def test_variable_edge_measurement(self, vector_vertex):
        """Test setting measurement."""
        edge = g2o.VariableVectorXEdge()
        edge.set_dimension(3)
        edge.resize(1)

        measurement = np.array([1.0, 2.0, 3.0])
        edge.set_measurement(measurement)

        retrieved = edge.measurement()
        assert np.allclose(retrieved, measurement)

    def test_variable_edge_information(self, vector_vertex):
        """Test setting information matrix."""
        edge = g2o.VariableVectorXEdge()
        edge.set_dimension(3)
        edge.resize(1)

        info = np.eye(3)
        edge.set_information(info)

        retrieved = edge.information()
        assert np.allclose(retrieved, info)

    def test_variable_edge_vertex_connection(self):
        """Test connecting edge to vertices."""
        edge = g2o.VariableVectorXEdge()
        edge.set_dimension(2)
        edge.resize(1)

        v = g2o.VectorXVertex()
        v.set_dimension(2)
        v.set_id(0)

        edge.set_vertex(0, v)
        assert edge.vertex(0) is not None


class CustomVectorEdge(g2o.VariableVectorXEdge):
    """Custom edge subclass for testing."""

    def __init__(self):
        super().__init__()
        self.set_dimension(1)
        self.resize(1)
        self.set_measurement(np.array([0.0]))

    def compute_error(self):
        """Compute error as difference from measurement."""
        vertex = self.vertex(0)
        estimate = vertex.estimate()
        measurement = self.measurement()
        # Simple 1D error
        self.error = [np.linalg.norm(estimate - measurement)]

    def linearize_oplus(self):
        """Numerical Jacobian (default behavior)."""
        pass


class TestCustomEdge:
    """Test subclassing VariableVectorXEdge."""

    def test_custom_edge_creation(self):
        """Test creating custom edge subclass."""
        edge = CustomVectorEdge()
        assert edge.dimension() == 1

    def test_custom_edge_compute_error(self):
        """Test custom error computation."""
        edge = CustomVectorEdge()

        v = g2o.VectorXVertex()
        v.set_dimension(1)
        v.set_estimate(np.array([5.0]))

        edge.set_vertex(0, v)
        edge.set_measurement(np.array([3.0]))

        edge.compute_error()
        # Error should be approximately 2.0
        assert len(edge.error) == 1


class TestCustomTypeOptimization:
    """Test optimization with custom types."""

    def test_simple_curve_fitting(self):
        """Test simple linear curve fitting with custom types."""
        # Create optimizer
        optimizer = g2o.SparseOptimizer()
        solver = g2o.BlockSolverX(g2o.LinearSolverEigenX())
        optimizer.set_algorithm(g2o.OptimizationAlgorithmLevenberg(solver))

        # Create a vertex with 2 parameters (slope, intercept)
        v = CustomVectorVertex()
        v.set_dimension(2)
        v.set_id(0)
        v.set_estimate(np.array([0.0, 0.0]))
        optimizer.add_vertex(v)

        # Add some edges with measurements
        num_measurements = 5
        for i in range(num_measurements):
            edge = CustomVectorEdge()
            edge.set_dimension(1)
            edge.resize(1)
            edge.set_vertex(0, v)
            edge.set_measurement(np.array([float(i)]))
            edge.set_information(np.eye(1))
            optimizer.add_edge(edge)

        assert len(optimizer.vertices()) == 1
        assert len(optimizer.edges()) == num_measurements
