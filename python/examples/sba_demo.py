# https://github.com/RainerKuemmerle/g2o/blob/master/g2o/examples/sba/sba_demo.cpp

import numpy as np
import g2o 

from collections import defaultdict
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--noise', dest='pixel_noise', type=float, default=1.,
    help='noise in image pixel space (default: 1.0)')
parser.add_argument('--outlier', dest='outlier_ratio', type=float, default=0.,
    help='probability of spuroius observation  (default: 0.0)')
parser.add_argument('--robust', dest='robust_kernel', action='store_true', help='use robust kernel')
parser.add_argument('--dense', action='store_true', help='use dense solver')
parser.add_argument('--seed', type=int, help='random seed', default=0)
args = parser.parse_args()



def main():    
    optimizer = g2o.SparseOptimizer()
    solver = g2o.BlockSolverSE3(g2o.LinearSolverCSparseSE3())
    solver = g2o.OptimizationAlgorithmLevenberg(solver)
    optimizer.set_algorithm(solver)


    true_points = np.hstack([
        np.random.random((500, 1)) * 3 - 1.5,
        np.random.random((500, 1)) - 0.5,
        np.random.random((500, 1)) + 3])


    focal_length = (500, 500)
    principal_point = (320, 240)
    baseline = 0.075
    g2o.VertexSCam.set_cam(*focal_length, *principal_point, baseline)

    true_poses = []
    num_pose = 5
    for i in range(num_pose):
        # pose here transform points from world coordinates to camera coordinates
        pose = g2o.Isometry3d(np.identity(3), [i*0.04-1, 0, 0])
        true_poses.append(pose)

        v_se3 = g2o.VertexSCam()
        v_se3.set_id(i)
        v_se3.set_estimate(pose)
        if i < 2:
            v_se3.set_fixed(True)
        v_se3.set_all()
        optimizer.add_vertex(v_se3)


    point_id = num_pose
    inliers = dict()
    sse = defaultdict(float)

    for i, point in enumerate(true_points):
        visible = []
        for j in range(num_pose):
            z = optimizer.vertex(j).map_point(point)
            if 0 <= z[0] < 640 and 0 <= z[1] < 480:
                visible.append((j, z))

        if len(visible) < 2:
            continue

        vp = g2o.VertexSBAPointXYZ()
        vp.set_id(point_id)
        vp.set_marginalized(True)
        vp.set_estimate(point + np.random.randn(3))
        optimizer.add_vertex(vp)

        inlier = True
        for j, z in visible:
            if np.random.random() < args.outlier_ratio:
                inlier = False
                z = np.array([
                    np.random.uniform(64, 640),
                    np.random.uniform(0, 480),
                    np.random.uniform(0, 64)])  # disparity
                z[2] = z[0] - z[2]
            z += np.random.randn(3) * args.pixel_noise * [1, 1, 1/16.]

            edge = g2o.Edge_XYZ_VSC()
            edge.set_vertex(0, vp)
            edge.set_vertex(1, optimizer.vertex(j))
            edge.set_measurement(z)
            edge.set_information(np.identity(3))
            if args.robust_kernel:
                edge.set_robust_kernel(g2o.RobustKernelHuber())

            edge.set_parameter_id(0, 0)
            optimizer.add_edge(edge)

        if inlier:
            inliers[point_id] = i
            error = vp.estimate() - true_points[i]
            sse[0] += np.sum(error**2)
        point_id += 1

    print('Performing full BA:')
    optimizer.initialize_optimization()
    optimizer.set_verbose(True)
    optimizer.optimize(10)


    for i in inliers:
        vp = optimizer.vertex(i)
        error = vp.estimate() - true_points[inliers[i]]
        sse[1] += np.sum(error**2)

    print('\nRMSE (inliers only):')
    print('before optimization:', np.sqrt(sse[0] / len(inliers)))
    print('after  optimization:', np.sqrt(sse[1] / len(inliers)))
                    


if __name__ == '__main__':
    if args.seed > 0:
        np.random.seed(args.seed)

    main()