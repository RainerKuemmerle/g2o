# https://github.com/RainerKuemmerle/g2o/blob/master/g2o/examples/icp/gicp_demo.cpp

import numpy as np
import g2o

import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--noise', type=float, help='noise in position', default=0.02)
parser.add_argument('--seed', type=int, help='random seed', default=0)
args = parser.parse_args()



def main():
    optimizer = g2o.SparseOptimizer()
    solver = g2o.BlockSolverX(g2o.LinearSolverDenseX())
    algorithm = g2o.OptimizationAlgorithmLevenberg(solver)
    optimizer.set_algorithm(algorithm)

    true_points = np.hstack([
        np.random.random((1000, 1)) * 3 - 1.5,
        np.random.random((1000, 1)) - 0.5,
        np.random.random((1000, 1)) + 10])


    for i in range(2):
        t = np.array([0, 0, i])
        cam = g2o.Isometry3d(np.identity(3), t)

        vc = g2o.VertexSE3()
        vc.set_id(i)
        vc.set_estimate(cam)
        if i == 0:
            vc.set_fixed(True)
        optimizer.add_vertex(vc)

    trans0 = optimizer.vertex(0).estimate().inverse()
    trans1 = optimizer.vertex(1).estimate().inverse()

    # set up point matches
    for i in range(len(true_points)):
        # calculate the relative 3d position of the point
        pt0 = trans0 * true_points[i]
        pt1 = trans1 * true_points[i]

        # add noise
        pt0 += np.random.randn(3) * args.noise
        pt1 += np.random.randn(3) * args.noise

        # form edge, with normals in varioius positions
        nm0 = np.array([0, i, 1])
        nm0 = nm0 / np.linalg.norm(nm0)
        nm1 = np.array([0, i, 1])
        nm1 = nm1 / np.linalg.norm(nm1)

        meas = g2o.EdgeGICP()
        meas.pos0 = pt0
        meas.pos1 = pt1
        meas.normal0 = nm0
        meas.normal1 = nm1

        edge = g2o.Edge_V_V_GICP()
        edge.set_vertex(0, optimizer.vertex(0))
        edge.set_vertex(1, optimizer.vertex(1))
        edge.set_measurement(meas)
        edge.set_information(meas.prec0(0.01))

        optimizer.add_edge(edge)

    # move second cam off of its true position
    vc = optimizer.vertex(1)
    cam = g2o.Isometry3d(vc.estimate().R, np.array([0, 0, 0.2]))
    vc.set_estimate(cam)


    optimizer.initialize_optimization()
    optimizer.compute_active_errors()
    print('Initial chi2 =', optimizer.chi2())

    optimizer.save('gicp.g2o')

    optimizer.set_verbose(True)
    optimizer.optimize(5)

    print('\nSecond vertex should be near [0, 0, 1]')
    print('before optimization:', cam.t)
    print('after  optimization:', optimizer.vertex(1).estimate().t)



if __name__ == '__main__':
    if args.seed > 0:
        np.random.seed(args.seed)

    main()