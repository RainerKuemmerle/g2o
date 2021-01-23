# https://github.com/RainerKuemmerle/g2o/blob/master/g2o/examples/ba/ba_demo.cpp

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
    solver = g2o.BlockSolverSE3(g2o.LinearSolverEigenSE3())
    solver = g2o.OptimizationAlgorithmLevenberg(solver)
    optimizer.set_algorithm(solver)

    focal_length = (500, 500)
    principal_point = (320, 240)
    baseline = 0.075


    true_points = np.hstack([
        np.random.random((500, 1)) * 3 - 1.5,
        np.random.random((500, 1)) - 0.5,
        np.random.random((500, 1)) + 3])


    num_pose = 15
    for i in range(num_pose):
        # pose here means transform points from camera coordinates to world coordinates
        pose = g2o.SE3Quat(np.identity(3), [i*0.04-1, 0, 0])
        sbacam = g2o.SBACam(pose)
        sbacam.set_cam(*focal_length, *principal_point, baseline)

        v_cam = g2o.VertexCam()
        v_cam.set_id(i)
        v_cam.set_estimate(sbacam)
        if i < 2:
            v_cam.set_fixed(True)
        optimizer.add_vertex(v_cam)


    point_id = num_pose
    inliers = dict()
    sse = defaultdict(float)

    for i, point in enumerate(true_points):
        visible = []
        for j in range(num_pose):
            proj = optimizer.vertex(j).estimate().w2i.dot(
                np.hstack([point, [1]]))   # project to left camera
            z = proj[:2] / proj[2]
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
                z = np.random.random(2) * [640, 480]
            z += np.random.randn(2) * args.pixel_noise

            edge = g2o.EdgeProjectP2MC()   # if stereo, use EdgeProjectP2SC
            edge.set_vertex(0, vp)
            edge.set_vertex(1, optimizer.vertex(j))
            edge.set_measurement(z)
            edge.set_information(np.identity(2))
            if args.robust_kernel:
                edge.set_robust_kernel(g2o.RobustKernelHuber())

            #edge.set_parameter_id(0, 0)
            optimizer.add_edge(edge)

        if inlier:
            inliers[point_id] = i
            error = vp.estimate() - true_points[i]
            sse[0] += np.sum(error**2)
        point_id += 1

    print('num vertices:', len(optimizer.vertices()))
    print('num edges:', len(optimizer.edges()))

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