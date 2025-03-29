from typing import List

import g2opy as g2o
import numpy as np
import plotly.graph_objects as go


class VertexCircle(g2o.VectorXVertex):
    """A circle parameterized by position x,y with radius r"""

    def __init__(self) -> None:
        g2o.VectorXVertex.__init__(self)
        self.set_dimension(3)
        self.set_estimate([0] * 3)

    def oplus_impl(self, update) -> None:
        self.set_estimate(self.estimate() + update)


class EdgePointOnCircle(g2o.VariableVectorXEdge):
    def __init__(self) -> None:
        g2o.VariableVectorXEdge.__init__(self)
        self.set_dimension(1)  # dimension of the error function
        self.information()
        self.resize(1)  # number of vertices
        self.set_measurement([0, 0])  # initial measurement

    def compute_error(self):
        circle = self.vertex(0).estimate()
        radius = circle[2]
        self.error = [np.linalg.norm(self.measurement() - circle[0:2]) - radius]

    # if not implemented, will be numerically estimated
    def linearize_oplus(self):
        circle = self.vertex(0).estimate()
        expected_radius = np.linalg.norm(self.measurement() - circle[0:2])
        m = np.array(
            [
                [
                    -(self.measurement()[0] - circle[0]) / expected_radius,
                    -(self.measurement()[1] - circle[1]) / expected_radius,
                    -1.0,
                ]
            ],
            dtype=np.float64,
        )
        self.set_jacobian(0, m)


def main():
    num_points: int = 100
    max_iterations: int = 10
    verbose: bool = True
    # TODO: Parse from command line

    # Setup the optimizer
    optimizer = g2o.SparseOptimizer()
    solver = g2o.BlockSolverSE3(g2o.LinearSolverEigenSE3())
    solver = g2o.OptimizationAlgorithmLevenberg(solver)
    optimizer.set_algorithm(solver)

    # generate random data
    center: np.ndarray = np.array([4.0, 2.0])
    radius: float = 2.0

    def gen_point() -> np.array:
        r = np.random.normal(radius, 0.05)
        angle = np.random.random() * 2.0 * np.pi
        return center + np.array([r * np.cos(angle), r * np.sin(angle)])

    points: List[np.array] = [gen_point() for _ in range(num_points)]

    # build the optimization problem given the points
    # 1. add the circle vertex
    circle: VertexCircle = VertexCircle()
    circle.set_id(0)
    circle.set_estimate([3.0, 3.0, 3.0])  # some initial value for the circle
    optimizer.add_vertex(circle)

    # 2. add the points we measured
    for point in points:
        edge: EdgePointOnCircle = EdgePointOnCircle()
        edge.set_information(np.identity(1))
        edge.set_vertex(0, circle)
        edge.set_measurement(point)
        # print(f"error: {edge.compute_error()}")
        # print(f"edge slots: {dir(edge)}")
        optimizer.add_edge(edge)

    print(f"Number of vertices: {len(optimizer.vertices())}")
    print(f"Number of edges: {len(optimizer.edges())}")

    # perform the optimization
    optimizer.initialize_optimization()
    optimizer.set_verbose(verbose)
    optimizer.optimize(max_iterations)

    if verbose:
        print()

    # print out the result
    print("Iterative least squares solution")
    print(f"Center of the circle: {circle.estimate()[0:2]}")
    print(f"Radius of the circle: {circle.estimate()[2]}")
    optimizer.compute_active_errors()
    print(f"Final chi2: {optimizer.chi2()}")

    # plot the solution
    fig = go.Figure()
    fig.add_trace(
        go.Scatter(
            x=[p[0] for p in points],
            y=[p[1] for p in points],
            mode="markers",
        )
    )
    fig.add_shape(
        type="circle",
        xref="x",
        yref="y",
        x0=circle.estimate()[0] - circle.estimate()[2],
        y0=circle.estimate()[1] - circle.estimate()[2],
        x1=circle.estimate()[0] + circle.estimate()[2],
        y1=circle.estimate()[1] + circle.estimate()[2],
        line_color="LightSeaGreen",
    )
    fig.update_yaxes(scaleanchor="x", scaleratio=1)
    fig.show()


if __name__ == "__main__":
    main()
