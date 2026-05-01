"""Tests for g2o vertex and edge types."""

# g2o will be available after conftest.py sets sys.path
import g2opy as g2o
import numpy as np


class TestSE2Types:
    """Test SE2 (2D SLAM) types."""

    def test_vertex_se2_creation(self):
        """Test creating VertexSE2."""
        v = g2o.VertexSE2()
        assert v is not None

    def test_vertex_se2_estimate(self):
        """Test SE2 vertex estimate."""
        v = g2o.VertexSE2()
        estimate = g2o.SE2(1.0, 2.0, np.pi / 4)
        v.set_estimate(estimate)

        retrieved = v.estimate()
        assert abs(retrieved.translation()[0] - 1.0) < 1e-6
        assert abs(retrieved.translation()[1] - 2.0) < 1e-6

    def test_vertex_se2_id(self):
        """Test SE2 vertex ID management."""
        v = g2o.VertexSE2()
        v.set_id(42)
        assert v.id() == 42

    def test_vertex_se2_fixed(self):
        """Test fixing SE2 vertices."""
        v = g2o.VertexSE2()
        v.set_fixed(True)
        assert v.fixed()
        v.set_fixed(False)
        assert not v.fixed()

    def test_edge_se2_creation(self):
        """Test creating EdgeSE2."""
        edge = g2o.EdgeSE2()
        assert edge is not None

    def test_edge_se2_measurement(self):
        """Test SE2 edge measurement."""
        edge = g2o.EdgeSE2()
        measurement = g2o.SE2(1.0, 0.0, 0.0)
        edge.set_measurement(measurement)

        retrieved = edge.measurement()
        assert abs(retrieved.translation()[0] - 1.0) < 1e-6


class TestSE3Types:
    """Test SE3 (3D SLAM) types."""

    def test_vertex_se3_expmap_creation(self):
        """Test creating VertexSE3Expmap."""
        v = g2o.VertexSE3Expmap()
        assert v is not None

    def test_vertex_se3_estimate(self):
        """Test SE3 vertex estimate."""
        v = g2o.VertexSE3Expmap()
        rotation = np.eye(3)
        translation = np.array([1.0, 2.0, 3.0])
        estimate = g2o.SE3Quat(rotation, translation)
        v.set_estimate(estimate)

        retrieved = v.estimate()
        assert np.allclose(retrieved.translation(), translation)

    def test_vertex_se3_id(self):
        """Test SE3 vertex ID management."""
        v = g2o.VertexSE3Expmap()
        v.set_id(10)
        assert v.id() == 10

    def test_edge_se3_creation(self):
        """Test creating EdgeSE3."""
        edge = g2o.EdgeSE3()
        assert edge is not None

    def test_point_xyz_creation(self):
        """Test creating VertexPointXYZ."""
        v = g2o.VertexPointXYZ()
        assert v is not None

    def test_point_xyz_estimate(self):
        """Test 3D point vertex estimate."""
        v = g2o.VertexPointXYZ()
        point = np.array([1.0, 2.0, 3.0])
        v.set_estimate(point)

        retrieved = v.estimate()
        assert np.allclose(retrieved, point)


class TestXYTypes:
    """Test 2D point types."""

    def test_vertex_xy_creation(self):
        """Test creating VertexPointXY."""
        v = g2o.VertexPointXY()
        assert v is not None

    def test_vertex_xy_estimate(self):
        """Test 2D point vertex estimate."""
        v = g2o.VertexPointXY()
        point = np.array([1.5, 2.5])
        v.set_estimate(point)

        retrieved = v.estimate()
        assert np.allclose(retrieved, point)


class TestMarginalization:
    """Test marginalization flags."""

    def test_vertex_marginalization_se2(self):
        """Test marginalizing SE2 vertices."""
        v = g2o.VertexSE2()
        assert not v.marginalized()

        v.set_marginalized(True)
        assert v.marginalized()

    def test_vertex_marginalization_point(self):
        """Test marginalizing point vertices."""
        v = g2o.VertexPointXYZ()
        v.set_marginalized(True)
        assert v.marginalized()
