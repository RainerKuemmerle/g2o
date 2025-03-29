import g2opy as g2o
import numpy as np


class MyEdgeSE2(g2o.EdgeSE2):
    def __init__(self):
        super().__init__()

    def compute_error(self):
        super().compute_error()
        print(f"Compute Error {self.error}")

    def linearize_oplus(self):
        super().linearize_oplus()
        print(f"Linearize:\nxi\n{self.jacobian_oplus_xi}\nxj\n{self.jacobian_oplus_xj}")


optimizer = g2o.SparseOptimizer()
linearSolver = g2o.LinearSolverEigenX()
solver_ptr = g2o.BlockSolverX(linearSolver)
solver = g2o.OptimizationAlgorithmGaussNewton(solver_ptr)
optimizer.set_algorithm(solver)

v1 = g2o.VertexSE2()
v1.set_id(0)
v1.set_estimate(g2o.SE2())
v1.set_fixed(True)
optimizer.add_vertex(v1)

v2 = g2o.VertexSE2()
v2.set_id(1)
v2.set_estimate(g2o.SE2(1, 0, 0))
optimizer.add_vertex(v2)

edge = MyEdgeSE2()
edge.set_vertex(0, optimizer.vertex(0))
edge.set_vertex(1, optimizer.vertex(1))
edge.set_measurement(g2o.SE2(2, 1, 0.5))
edge.set_information(np.eye(3))
optimizer.add_edge(edge)

optimizer.initialize_optimization()
optimizer.set_verbose(True)
optimizer.optimize(2)
print(optimizer.vertex(1).estimate().to_vector())
