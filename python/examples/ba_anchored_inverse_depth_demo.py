# https://github.com/RainerKuemmerle/g2o/blob/master/g2o/examples/ba_anchored_inverse_depth/ba_anchored_inverse_depth_demo.cpp

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
parser.add_argument('--no-schur', dest='schur_trick', action='store_false', help='not use Schur-complement trick')
parser.add_argument('--seed', type=int, default=0, help='random seed')
args = parser.parse_args()



def invert_depth(x):
    assert len(x) == 3 and x[2] != 0
    return np.array([x[0], x[1], 1]) / x[2]


def main():
    optimizer = g2o.SparseOptimizer()
    if args.schur_trick:
        solver = g2o.BlockSolverSE3(g2o.LinearSolverEigenSE3())
    else:
        solver = g2o.BlockSolverX(g2o.LinearSolverEigenX())   # slower
    solver = g2o.OptimizationAlgorithmLevenberg(solver)
    optimizer.set_algorithm(solver)

    true_points = np.hstack([
        np.random.random((500, 1)) * 3 - 1.5,
        np.random.random((500, 1)) - 0.5,
        np.random.random((500, 1)) + 3])

    
    focal_length = 1000.
    principal_point = (320, 240)
    cam = g2o.CameraParameters(focal_length, principal_point, 0)
    cam.set_id(0)

    optimizer.add_parameter(cam)

    true_poses = []
    num_pose = 15
    for i in range(num_pose):
        # pose here means transform points from world coordinates to camera coordinates
        pose = g2o.SE3Quat(np.identity(3), [i*0.04-1, 0, 0])
        true_poses.append(pose)

        v_se3 = g2o.VertexSE3Expmap()
        v_se3.set_id(i)
        v_se3.set_estimate(pose)
        if i < 2:
            v_se3.set_fixed(True)
        optimizer.add_vertex(v_se3)


    point_id = num_pose
    inliers = dict()
    sse = defaultdict(float)

    for i, point in enumerate(true_points):
        visible = []
        for j, pose in enumerate(true_poses):
            z = cam.cam_map(pose * point)
            if 0 <= z[0] < 640 and 0 <= z[1] < 480:
                visible.append((j, z))
        if len(visible) < 2:
            continue

        v_p = g2o.VertexSBAPointXYZ()
        v_p.set_id(point_id)
        v_p.set_marginalized(args.schur_trick)

        anchor = visible[0][0]
        point2 = true_poses[anchor] * (point + np.random.randn(3))
        if point2[2] == 0:
            continue
        v_p.set_estimate(invert_depth(point2))
        optimizer.add_vertex(v_p)

        inlier = True
        for j, z in visible:
            if np.random.random() < args.outlier_ratio:
                inlier = False
                z = np.random.random(2) * [640, 480]
            z += np.random.randn(2) * args.pixel_noise

            edge = g2o.EdgeProjectPSI2UV()
            edge.resize(3)
            edge.set_vertex(0, v_p)
            edge.set_vertex(1, optimizer.vertex(j))
            edge.set_vertex(2, optimizer.vertex(anchor))
            edge.set_measurement(z)
            edge.set_information(np.identity(2))
            if args.robust_kernel:
                edge.set_robust_kernel(g2o.RobustKernelHuber())

            edge.set_parameter_id(0, 0)
            optimizer.add_edge(edge)

        if inlier:
            inliers[point_id] = (i, anchor)
            error = (true_poses[anchor].inverse() * invert_depth(v_p.estimate()) - 
                true_points[i])
            sse[0] += np.sum(error**2)
        point_id += 1

    print('Performing full BA:')
    optimizer.initialize_optimization()
    optimizer.set_verbose(True)
    optimizer.optimize(10)


    for i in inliers:
        v_p = optimizer.vertex(i)
        v_anchor = optimizer.vertex(inliers[i][1])
        error = (v_anchor.estimate().inverse() * invert_depth(v_p.estimate()) - 
            true_points[inliers[i][0]])
        sse[1] += np.sum(error**2)


    print('\nRMSE (inliers only):')
    print('before optimization:', np.sqrt(sse[0] / len(inliers)))
    print('after  optimization:', np.sqrt(sse[1] / len(inliers)))



if __name__ == '__main__':
    if args.seed > 0:
        np.random.seed(args.seed)

    main()