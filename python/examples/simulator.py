import g2opy as g2o

simulator = g2o.Simulator2D()
simulator.config.has_odom = True
simulator.config.has_pose_sensor = True
simulator.config.world_size = 50.0
simulator.config.sim_steps = 500

simulator.setup()
simulator.simulate()

print("Simulation result")
print(f"Number of vertices: {len(simulator.graph().vertices())}")
print(f"Number of edges {len(simulator.graph().edges())}")


def create_optimizer():
    optimizer = g2o.SparseOptimizer()
    solver = g2o.BlockSolverX(g2o.LinearSolverEigenX())
    solver = g2o.OptimizationAlgorithmLevenberg(solver)
    optimizer.set_algorithm(solver)
    return optimizer


optimizer = create_optimizer()
optimizer.add_graph(simulator.graph())
print("Optimizer")
print(f"Number of vertices: {len(optimizer.vertices())}")
print(f"Number of edges {len(optimizer.edges())}")

optimizer.vertices()[0].set_fixed(True)
optimizer.initialize_optimization()
optimizer.set_verbose(True)
optimizer.optimize(20)
