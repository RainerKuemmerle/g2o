# https://github.com/RainerKuemmerle/g2o/blob/master/g2o/examples/icp/gicp_sba_demo.cpp

import numpy as np 
import g2o

import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--num_points', type=int, help='num of points to use in projection SBA', default=0)
parser.add_argument('--pos_noise', type=float, help='noise in 3d position', default=0.2)
parser.add_argument('--pixel_noise', type=float, help='pixel noise', default=0.5)
parser.add_argument('--seed', type=int, help='random seed', default=0)
args = parser.parse_args()



def main():
    optimizer = g2o.SparseOptimizer()
    solver = g2o.BlockSolverX(g2o.LinearSolverCSparseX())
    solver = g2o.OptimizationAlgorithmLevenberg(solver)
    optimizer.set_algorithm(solver)

    true_points = np.hstack([
        np.random.random((1000, 1)) * 3 - 1.5,
        np.random.random((1000, 1)) - 0.5,
        np.random.random((1000, 1)) + 10])

    
    focal_length = (500, 500)
    principal_point = (320, 240)
    baseline = 0.075
    g2o.VertexSCam.set_cam(*focal_length, *principal_point, baseline)

    for i in range(2):
        t = np.array([0, 0, i])
        cam = g2o.Isometry3d(np.identity(3), t)

        vc = g2o.VertexSCam()
        vc.set_id(i)
        vc.set_estimate(cam)
        if i == 0:
            vc.set_fixed(True)
        vc.set_all()
        optimizer.add_vertex(vc)


    trans0 = optimizer.vertex(0).estimate().inverse()
    trans1 = optimizer.vertex(1).estimate().inverse()

    for i in range(len(true_points)):
        pt0 = trans0 * true_points[i]
        pt1 = trans1 * true_points[i]

        # add noise
        pt0 += np.random.randn(3) * args.pos_noise
        pt1 += np.random.randn(3) * args.pos_noise

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


    # set up SBA projections
    if args.num_points > 0:
        true_points = np.hstack([
            np.random.random((args.num_points, 1)) * 3 - 1.5,
            np.random.random((args.num_points, 1)) - 0.5,
            np.random.random((args.num_points, 1)) + 10])

        cam_num = 2
        for i, point in enumerate(true_points):
            vp = g2o.VertexSBAPointXYZ()
            vp.set_id(cam_num + i)
            vp.set_marginalized(True)
            vp.set_estimate(point + np.random.randn(3))
            optimizer.add_vertex(vp)

            for j in range(cam_num):
                z = optimizer.vertex(j).map_point(point)
                if 0 <= z[0] < 640 and 0 <= z[1] < 480:
                    z += np.random.randn(3) * args.pixel_noise * [1, 1, 1/16.]

                    edge = g2o.Edge_XYZ_VSC()
                    edge.set_vertex(0, vp)
                    edge.set_vertex(1, optimizer.vertex(j))
                    edge.set_measurement(z)
                    edge.set_information(np.identity(3))
                    edge.set_robust_kernel(g2o.RobustKernelHuber())

                    optimizer.add_edge(edge)


    # move second cam off of its true position
    vc = optimizer.vertex(1)
    cam = g2o.Isometry3d(vc.estimate().R, np.array([-0.1, -0.1, 0.2]))
    vc.set_estimate(cam)


    optimizer.initialize_optimization()
    optimizer.compute_active_errors()
    print('Initial chi2 =', optimizer.chi2())

    optimizer.set_verbose(True)
    optimizer.optimize(20)

    print('\nSecond vertex should be near [0, 0, 1]')
    print('before optimization:', cam.t)
    print('after  optimization:', optimizer.vertex(1).estimate().t)
    print('error:', optimizer.vertex(1).estimate().t - [0, 0, 1])


    '''
    Mean squared error (average over 100 loops):
    num_points     0:   [ 0.29733384  0.40814327  0.03907623]
                  10:   [ 0.14172112  0.31366953  0.03670497]
                 100:   [ 0.05319327  0.37413272  0.01637925]
                1000:   [ 0.01652139  0.19501433  0.00502872]
               10000:   [ 0.00901297  0.04055765  0.00130791]   ->  1/3 : 1 : 1/16
    
    '''



if __name__ == '__main__':
    if args.seed > 0:
        np.random.seed(args.seed)

    main()