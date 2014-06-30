g2o_incremental implements a variant of the iSAM algorithm.
In contrast to iSAM the update of the Cholesky factor is done
using rank updates as provided via the CHOLMOD library.

“iSAM: Incremental Smoothing and Mapping”
by M. Kaess, A. Ranganathan, and F. Dellaert.
IEEE Trans. on Robotics, vol. 24, no. 6, Dec. 2008, pp. 1365-1378

Furthermore, it implements a simple protocol to control the operation of the
algorithm. See protocol.txt for more details or
http://slameval.willowgarage.com/workshop/submissions/ for an evaluation system
as well as data sets.

In the standard configuration g2o_incremental solves after inserting 10 nodes.
This behavior can be modified via a command line option. Furthermore, by
specifying -g on the command line a gnuplot instance is created to visualize
the graph.

Please note that both the visualization via Gnuplot and the verbose output
affect the timing results.

g2o_incremental is licensed under GPLv3.
