[ ] Create parameter type for ParameterCamera to support load/save
[ ] Create parameter type for ParameterStereoCamera to support load/save
[ ] Re-work python wrapping for parameters, templated base param
[ ] Test load/save for VertexCam (camera and baseline seem missing from traits)
[ ] Refactor Data container. Can it be a container instead of linked list?
[ ] Use FakeDependency in Traits
[ ] EdgeSE2TwoPointsXY, EdgeSE2PointXYCalib can be fixed size edges -> IO test
[ ] IO test fixture including parameter
[ ] Type Based Tests for Jacobian possible?
[x] support saving VectorX and MatrixX in G2O format
[x] update python optimizable graph wrapper (save/load) with format
[x] add version number for file format
[x] fix core API with exports for MSVC
[x] test save/load of dynamic vertex
[x] test save/load of dynamic edge
[x] binary format for save/load
[x] XML format for save/load
[ ] wrap abstract graph to python and update save wrapper
[x] ParameterContainer remove read/write
[x] remove read/write methods for vertex / edge

[ ] use cmakedefine01 for compile flags in config
[ ] add test for setData / getData on the graph
[ ] update py_base_vertex/edge to map now common methods