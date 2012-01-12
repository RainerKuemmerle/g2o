This directory implements an interface to g2o which reads the input data
via stdin and outputs upon request the current estimate to stdout.

In particular, the interface follows the grammar proposed for the RSS'11
workshop "Automated SLAM Evaluation", see
http://slameval.willowgarage.com/workshop/submissions/

Brief description:
- slam_parser (LGPL v3)
  The parser for the SLAM protocol
- g2o_interactive (LGPL v3)
  Run batch optimization every N nodes
- g2o_incremental (GPL v3)
  Incrementally solve the optimzation, batch every 100 nodes
