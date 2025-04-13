[x] update py_base_vertex/edge to map now common methods
[x] common functions to wrap edge/vertex possible?
[ ] build only one library instead of multiple?
[ ] remove dynamic loading of wildcard libs from g2o/g2o_viewer
[ ] update ID to int64
[x] drop cereal (replace with json directly, drop xml)
[x] install json on Windows CI
[x] wrap io_format.h to python
[x] wrap simulator into library
[x] add config types for simulation
[ ] add tests for simulator
[x] add python wrapper for simulator
[ ] Add typing for parameters to template?
[ ] Add fixed size types for pure python problems
[ ] Re-work python wrapping for parameters, templated base param
[ ] Test load/save for VertexCam (camera and baseline seem missing from traits)
[ ] use cmakedefine01 for compile flags in config
[ ] add test for setData / getData on the graph
[ ] Use FakeDependency in Traits
[ ] wrap abstract graph to python and update save wrapper
[x] CPack config
[x] Create parameter type for ParameterCamera to support load/save
[x] Refactor Data container. Can it be a container instead of linked list?
[x] EdgeSE2TwoPointsXY, EdgeSE2PointXYCalib can be fixed size edges -> IO test
[x] IO test fixture including parameter
[x] Type Based Tests for Jacobian
[x] support saving VectorX and MatrixX in G2O format
[x] update python optimizable graph wrapper (save/load) with format
[x] add version number for file format
[x] fix core API with exports for MSVC
[x] test save/load of dynamic vertex
[x] test save/load of dynamic edge
[x] binary format for save/load
[x] XML format for save/load
[x] ParameterContainer remove read/write
[x] remove read/write methods for vertex / edge
