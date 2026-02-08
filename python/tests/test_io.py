"""Tests for file I/O operations."""

# g2o will be available after conftest.py sets sys.path
import g2opy as g2o
import numpy as np
import pytest


class TestIOFormats:
    """Test IO format detection and handling."""

    def test_io_format_enum(self):
        """Test IoFormat enum."""
        assert hasattr(g2o, "IoFormat")
        # Test that formats are available
        formats = ["G2O", "JSON", "BINARY"]
        # At least G2O format should be available
        assert hasattr(g2o.IoFormat, "G2O")

    def test_io_wrapper_format_guessing(self):
        """Test IoWrapper format guessing."""
        # .g2o files should be guessed as G2O format
        guessed = g2o.IoWrapper.format_for_file_extension("g2o")
        assert guessed == g2o.IoFormat.G2O
        guessed = g2o.IoWrapper.format_for_file_extension("json")
        assert guessed == g2o.IoFormat.JSON
        guessed = g2o.IoWrapper.format_for_file_extension("bin")
        assert guessed == g2o.IoFormat.BINARY


class TestSaveLoad:
    """Test saving and loading graphs."""

    def test_save_se2_graph(self, simple_se2_graph, temp_g2o_file):
        """Test saving SE2 graph."""
        success = simple_se2_graph.save(temp_g2o_file)
        assert success

    def test_load_empty_graph(self, basic_se2_optimizer, temp_g2o_file):
        """Test loading empty graph."""
        # First save an empty graph
        basic_se2_optimizer.save(temp_g2o_file)

        # Create new optimizer and load
        new_optimizer = g2o.SparseOptimizer()
        new_optimizer.set_algorithm(
            g2o.OptimizationAlgorithmLevenberg(
                g2o.BlockSolverSE2(g2o.LinearSolverEigenSE2())
            )
        )
        success = new_optimizer.load(temp_g2o_file)
        assert success

    def test_save_load_roundtrip(self, simple_se2_graph, temp_g2o_file):
        """Test save and load roundtrip."""
        # Save graph
        save_success = simple_se2_graph.save(temp_g2o_file)
        assert save_success

        # Load into new optimizer
        new_optimizer = g2o.SparseOptimizer()
        new_optimizer.set_algorithm(
            g2o.OptimizationAlgorithmLevenberg(
                g2o.BlockSolverSE2(g2o.LinearSolverEigenSE2())
            )
        )
        load_success = new_optimizer.load(temp_g2o_file)
        assert load_success

        # Compare vertex and edge counts
        assert len(new_optimizer.vertices()) == len(simple_se2_graph.vertices())
        assert len(new_optimizer.edges()) == len(simple_se2_graph.edges())

    def test_save_se3_graph(self, simple_se3_graph, temp_g2o_file):
        """Test saving SE3 graph."""
        success = simple_se3_graph.save(temp_g2o_file)
        assert success


class TestIOWrapper:
    """Test IoWrapper functionality."""

    def test_io_to_string(self):
        """Test IoWrapper.to_string conversion."""
        format_str = g2o.IoWrapper.to_string(g2o.IoFormat.G2O)
        assert isinstance(format_str, str)
        assert len(format_str) > 0
