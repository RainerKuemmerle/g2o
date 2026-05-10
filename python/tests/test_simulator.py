"""Tests for g2o simulator Python bindings."""

# g2o will be available after conftest.py sets sys.path
import g2opy as g2o


def test_simulator2d_basic_workflow():
    """Test Simulator2D Python binding and basic simulation workflow."""
    simulator = g2o.Simulator2D()

    assert hasattr(simulator, "config")
    assert isinstance(simulator.config, g2o.SimulatorConfig)

    simulator.config.world_size = 20.0
    simulator.config.sim_steps = 10
    simulator.config.has_odom = True
    simulator.config.has_pose_sensor = True
    simulator.config.has_point_sensor = True
    simulator.config.has_point_bearing_sensor = True
    simulator.config.has_gps = True

    simulator.seed(42)
    simulator.setup()
    simulator.simulate()

    graph = simulator.graph()
    world_graph = simulator.world().graph()

    assert graph is world_graph
    assert len(graph.vertices()) > 0
    assert len(graph.edges()) > 0

    initial_vertex_count = len(graph.vertices())
    initial_edge_count = len(graph.edges())

    # Graph can be integrated with a SparseOptimizer. Note that add_graph() moves
    # the graph contents into the optimizer and clears the source graph.
    optimizer = g2o.SparseOptimizer()
    solver = g2o.BlockSolverSE2(g2o.LinearSolverEigenSE2())
    optimizer.set_algorithm(g2o.OptimizationAlgorithmLevenberg(solver))
    optimizer.add_graph(graph)

    assert len(optimizer.vertices()) == initial_vertex_count
    assert len(optimizer.edges()) == initial_edge_count
    assert len(graph.vertices()) == 0
    assert len(graph.edges()) == 0


def test_simulator3d_basic_workflow():
    """Test Simulator3D Python binding and basic simulation workflow."""
    simulator = g2o.Simulator3D()

    assert hasattr(simulator, "config")
    assert isinstance(simulator.config, g2o.SimulatorConfig)

    simulator.config.world_size = 10.0
    simulator.config.sim_steps = 5
    simulator.config.has_odom = True
    simulator.config.has_point_sensor = True
    simulator.config.has_point_depth_sensor = True
    simulator.config.has_point_disparity_sensor = True
    simulator.config.has_pose_sensor = True
    simulator.config.has_gps = True

    simulator.seed(123)
    simulator.setup()
    simulator.simulate()

    graph = simulator.graph()
    world_graph = simulator.world().graph()

    assert graph is world_graph
    assert len(graph.vertices()) > 0
    assert len(graph.edges()) > 0


def test_simulator2d_config_constructor():
    """Test Simulator2D construction from a config object."""
    config = g2o.Simulator2DConfig()
    config.has_odom = True
    config.has_point_sensor = True
    config.sim_steps = 4

    simulator = g2o.Simulator2D(config)
    assert simulator.config.has_odom
    assert simulator.config.has_point_sensor
    assert simulator.config.sim_steps == 4

    simulator.setup()
    simulator.simulate()

    assert len(simulator.graph().vertices()) > 0
    assert len(simulator.graph().edges()) > 0
