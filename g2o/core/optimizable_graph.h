// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, H. Strasdat, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef G2O_AIS_OPTIMIZABLE_GRAPH_HH_
#define G2O_AIS_OPTIMIZABLE_GRAPH_HH_

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iosfwd>
#include <memory>
#include <set>
#include <string>
#include <typeinfo>
#include <unordered_map>
#include <vector>

#include "g2o/core/eigen_types.h"
#include "g2o/core/io/io_format.h"
#include "g2o_core_api.h"
#include "hyper_graph.h"
#include "jacobian_workspace.h"
#include "openmp_mutex.h"
#include "parameter.h"
#include "parameter_container.h"

namespace g2o {

class HyperGraphAction;
struct OptimizationAlgorithmProperty;
class CacheContainer;
class RobustKernel;
class AbstractGraph;

/**
   @addtogroup g2o
 */
/**
   This is an abstract class that represents one optimization
   problem.  It specializes the general graph to contain special
   vertices and edges.  The vertices represent parameters that can
   be optimized, while the edges represent constraints.  This class
   also provides basic functionalities to handle the backup/restore
   of portions of the vertices.
 */
class G2O_CORE_API OptimizableGraph : public HyperGraph {
 public:
  enum ActionType {
    kAtPreiteration,
    kAtPostiteration,
    kAtNumElements,  // keep as last element
  };

  using HyperGraphActionSet = std::set<std::shared_ptr<HyperGraphAction>>;

  // forward declarations
  class G2O_CORE_API Vertex;
  class G2O_CORE_API Edge;

  /**
   * \brief order vertices based on their ID
   */
  struct G2O_CORE_API VertexIDCompare {
    template <typename VertexPtrType1, typename VertexPtrType2>
    bool operator()(const VertexPtrType1& v1, const VertexPtrType2& v2) const {
      return v1->id() < v2->id();
    }
  };

  /**
   * \brief order edges based on the internal ID, which is assigned to the edge
   * in addEdge()
   */
  struct G2O_CORE_API EdgeIDCompare {
    bool operator()(const Edge* e1, const Edge* e2) const {
      return e1->internalId() < e2->internalId();
    }
    bool operator()(const HyperGraph::Edge* e1,
                    const HyperGraph::Edge* e2) const {
      return operator()(static_cast<const Edge*>(e1),
                        static_cast<const Edge*>(e2));
    }
    bool operator()(const std::shared_ptr<OptimizableGraph::Edge>& e1,
                    const HyperGraph::Edge* e2) const {
      return operator()(static_cast<const Edge*>(e1.get()),
                        static_cast<const Edge*>(e2));
    }
    bool operator()(const std::shared_ptr<OptimizableGraph::Edge>& e1,
                    const std::shared_ptr<OptimizableGraph::Edge>& e2) const {
      return operator()(static_cast<const Edge*>(e1.get()),
                        static_cast<const Edge*>(e2.get()));
    }
    bool operator()(const std::shared_ptr<HyperGraph::Edge>& e1,
                    const std::shared_ptr<HyperGraph::Edge>& e2) const {
      return operator()(static_cast<const Edge*>(e1.get()),
                        static_cast<const Edge*>(e2.get()));
    }
  };

  //! vector container for vertices
  using VertexContainer =
      std::vector<std::shared_ptr<OptimizableGraph::Vertex>>;
  using VertexContainerRaw = std::vector<OptimizableGraph::Vertex*>;
  //! vector container for edges
  using EdgeContainer = std::vector<std::shared_ptr<OptimizableGraph::Edge>>;

  /**
   * \brief A general case Vertex for optimization
   */
  class G2O_CORE_API Vertex : public HyperGraph::Vertex,
                              public HyperGraph::DataContainer {
   private:
    friend class OptimizableGraph;

   public:
    Vertex();
    ~Vertex() override;

    //! get the mapped memory of the hessian matrix
    [[nodiscard]] virtual double* hessianData() const = 0;

    //! return a map of the mapped Hessian
    [[nodiscard]] MatrixX::MapType hessianMap() const;

    /** maps the internal matrix to some external memory location */
    virtual void mapHessianMemory(double* d) = 0;

    /**
     * copies the b vector in the array b_
     * @return the number of elements copied
     */
    virtual int copyB(double* b_) const = 0;

    //! return a pointer to the b vector associated with this vertex
    [[nodiscard]] virtual double* bData() const = 0;

    //! return a map onto the b vector
    [[nodiscard]] VectorX::MapType bMap() const;

    /**
     * set the b vector part of this vertex to zero
     */
    virtual void clearQuadraticForm() = 0;

    /**
     * updates the current vertex with the direct solution x += H_ii\b_ii
     * @return True, iff solution was possible
     */
    virtual bool solveDirect(double lambda = 0) = 0;

    /**
     * sets the initial estimate from an array of double
     * @return true on success
     */
    virtual bool setEstimateData(const double* estimate) = 0;

    /**
     * sets the initial estimate from an array of double
     * @return true on success
     */
    bool setEstimateData(const std::vector<double>& estimate) {
      int dim = estimateDimension();
      if ((dim == -1) || (estimate.size() != static_cast<std::size_t>(dim)))
        return false;
      return setEstimateData(estimate.data());
    }

    /**
     * sets the initial estimate from an Eigen Vector type
     * @return true on success
     */
    template <typename Derived>
    bool setEstimateData(const Eigen::MatrixBase<Derived>& estimate) {
      int dim = estimateDimension();
      if ((dim == -1) || (estimate.size() != dim)) return false;
      return setEstimateData(estimate.derived().data());
    }

    /**
     * writes the estimates to an array of double
     * @returns true on success
     */
    virtual bool getEstimateData(double* estimate) const = 0;

    /**
     * writes the estimate to an array of double
     * @returns true on success
     */
    virtual bool getEstimateData(std::vector<double>& estimate) const {
      int dim = estimateDimension();
      if (dim < 0) return false;
      estimate.resize(dim);
      return getEstimateData(estimate.data());
    }

    /**
     * writes the estimate to an array of double
     * @returns true on success
     */
    template <typename Derived>
    bool getEstimateData(Eigen::MatrixBase<Derived>& estimate) const {
      int dim = estimateDimension();
      // If dim is -ve, getEstimateData is not implemented and fails
      if (dim < 0) return false;

      // If the vector isn't the right size to store the estimate, try to resize
      // it. This only works if the vector is dynamic. If it is static, fail.
      if (estimate.size() != dim) {
        if ((estimate.RowsAtCompileTime == Eigen::Dynamic) ||
            (estimate.ColsAtCompileTime == Eigen::Dynamic))
          estimate.derived().resize(dim);
        else
          return false;
      }
      return getEstimateData(estimate.derived().data());
    }

    /**
     * returns the dimension of the extended representation used by
     * get/setEstimate(double*) -1 if it is not supported
     */
    [[nodiscard]] virtual int estimateDimension() const = 0;
    /**
     * Returns the dimension of the estimate at compile time.
     */
    [[nodiscard]] virtual int estimateDimensionAtCompileTime() const = 0;

    /**
     * sets the initial estimate from an array of double.
     * Implement setMinimalEstimateDataImpl()
     * @return true on success
     */
    virtual bool setMinimalEstimateData(const double* estimate) = 0;

    /**
     * sets the initial estimate from an array of double.
     * Implement setMinimalEstimateDataImpl()
     * @return true on success
     */
    bool setMinimalEstimateData(const std::vector<double>& estimate) {
      int dim = minimalEstimateDimension();
      if ((dim == -1) || (estimate.size() != static_cast<std::size_t>(dim)))
        return false;
      return setMinimalEstimateData(estimate.data());
    }

    /**
     * sets the initial estimate from an eigen type of double.
     * Implement setMinimalEstimateDataImpl()
     * @return true on success
     */
    template <typename Derived>
    bool setMinimalEstimateData(const Eigen::MatrixBase<Derived>& estimate) {
      int dim = minimalEstimateDimension();
      if ((dim == -1) || (estimate.size() != dim)) return false;
      return setMinimalEstimateData(estimate.derived().data());
    }

    /**
     * writes the estimate to an array of double
     * @returns true on success
     */
    virtual bool getMinimalEstimateData(double* estimate) const = 0;

    /**
     * writes the estimate to an array of double
     * @returns true on success
     */
    virtual bool getMinimalEstimateData(std::vector<double>& estimate) const {
      int dim = minimalEstimateDimension();
      if (dim < 0) return false;
      estimate.resize(dim);
      return getMinimalEstimateData(estimate.data());
    }

    /**
     * writes the estimate to an eigen type double
     * @returns true on success
     */
    template <typename Derived>
    bool getMinimalEstimateData(Eigen::MatrixBase<Derived>& estimate) const {
      int dim = minimalEstimateDimension();
      // If dim is -ve, getMinimalEstimateData is not implemented and fails
      if (dim < 0) return false;

      // If the vector isn't the right size to store the estimate, try to resize
      // it. This only works if the vector is dynamic. If it is static, fail.
      if (estimate.size() != dim) {
        if ((estimate.RowsAtCompileTime == Eigen::Dynamic) ||
            (estimate.ColsAtCompileTime == Eigen::Dynamic))
          estimate.derived().resize(dim);
        else
          return false;
      }
      return getMinimalEstimateData(estimate.derived().data());
    }

    /**
     * returns the dimension of the extended representation used by
     * get/setEstimate(double*) -1 if it is not supported
     */
    [[nodiscard]] virtual int minimalEstimateDimension() const = 0;

    //! backup the position of the vertex to a stack
    virtual void push() = 0;

    //! restore the position of the vertex by retrieving the position from the
    //! stack
    virtual void pop() = 0;

    //! pop the last element from the stack, without restoring the current
    //! estimate
    virtual void discardTop() = 0;

    //! return the stack size
    [[nodiscard]] virtual int stackSize() const = 0;

    /**
     * Update the position of the node from the parameters in v.
     * Depends on the implementation of oplusImpl in derived classes to actually
     * carry out the update. Will also call updateCache() to update the caches
     * of depending on the vertex.
     */
    void oplus(const VectorX::MapType& v) {
      oplusImpl(v);
      updateCache();
    }

    //! temporary index of this node in the parameter vector obtained from
    //! linearization
    [[nodiscard]] int hessianIndex() const { return hessianIndex_; }
    //! set the temporary index of the vertex in the parameter blocks
    void setHessianIndex(int ti) { hessianIndex_ = ti; }

    //! true => this node is fixed during the optimization
    [[nodiscard]] bool fixed() const { return fixed_; }
    //! true => this node should be considered fixed during the optimization
    void setFixed(bool fixed) { fixed_ = fixed; }

    //! true => this node is marginalized out during the optimization
    [[nodiscard]] bool marginalized() const { return marginalized_; }
    //! true => this node should be marginalized out during the optimization
    void setMarginalized(bool marginalized) { marginalized_ = marginalized; }

    //! dimension of the estimated state belonging to this node
    [[nodiscard]] int dimension() const { return dimension_; }

    virtual bool setDimension(int /*dimension*/) { return false; }

    //! sets the id of the node in the graph be sure that the graph keeps
    //! consistent after changing the id
    void setId(int id) override { id_ = id; }

    //! set the row of this vertex in the Hessian
    void setColInHessian(int c) { colInHessian_ = c; }
    //! get the row of this vertex in the Hessian
    [[nodiscard]] int colInHessian() const { return colInHessian_; }

    [[nodiscard]] const OptimizableGraph* graph() const { return graph_; }
    OptimizableGraph* graph() { return graph_; }

    /**
     * lock for the block of the hessian and the b vector associated with this
     * vertex, to avoid race-conditions if multi-threaded.
     */
    void lockQuadraticForm() { quadraticFormMutex_.lock(); }
    /**
     * unlock the block of the hessian and the b vector associated with this
     * vertex
     */
    void unlockQuadraticForm() { quadraticFormMutex_.unlock(); }

    virtual void updateCache();

    CacheContainer& cacheContainer();

   protected:
    OptimizableGraph* graph_{nullptr};
    int hessianIndex_{-1};
    bool fixed_{false};
    bool marginalized_{false};
    int dimension_;
    int colInHessian_{-1};
    OpenMPMutex quadraticFormMutex_;

    std::unique_ptr<CacheContainer> cacheContainer_{nullptr};

    /**
     * update the position of the node from the parameters in v.
     * Implement in your class!
     */
    virtual void oplusImpl(const VectorX::MapType& v) = 0;

    /**
     * sets the initial estimate from an array of double
     * @return true on success
     */
    virtual bool setMinimalEstimateDataImpl(const double*) { return false; }
  };

  class G2O_CORE_API Edge : public HyperGraph::Edge,
                            public HyperGraph::DataContainer {
   private:
    friend class OptimizableGraph;

   public:
    Edge();
    ~Edge() override;

    // indicates if all vertices are fixed
    [[nodiscard]] virtual bool allVerticesFixed() const = 0;

    // computes the error of the edge and stores it in an internal structure
    virtual void computeError() = 0;

    //! sets the measurement from an array of double
    //! @returns true on success
    virtual bool setMeasurementData(const double* m) = 0;

    //! writes the measurement to an array of double
    //! @returns true on success
    virtual bool getMeasurementData(double* m) const = 0;

    //! returns the dimension of the measurement in the extended representation
    //! which is used by get/setMeasurement;
    [[nodiscard]] virtual int measurementDimension() const = 0;

    /**
     * @brief Returns the measurement's dimension at compile time.
     */
    [[nodiscard]] virtual int measurementDimensionAtCompileTime() const = 0;

    /**
     * returns the minimal dimension of the measurement which corresponds to the
     * dimension of the information matrix.
     */
    [[nodiscard]] virtual int minimalMeasurementDimension() const = 0;

    /**
     * sets the estimate to have a zero error, based on the current value of the
     * state variables returns false if not supported.
     */
    virtual bool setMeasurementFromState();

    //! if NOT NULL, error of this edge will be robustifed with the kernel
    [[nodiscard]] std::shared_ptr<RobustKernel> robustKernel() const {
      return robustKernel_;
    }
    /**
     * specify the robust kernel to be used in this edge
     */
    void setRobustKernel(std::shared_ptr<RobustKernel> ptr);

    //! returns the error vector cached after calling the computeError;
    [[nodiscard]] virtual const double* errorData() const = 0;
    virtual double* errorData() = 0;

    //! returns the memory of the information matrix, usable for example with a
    //! Eigen::Map<MatrixX>
    [[nodiscard]] virtual const double* informationData() const = 0;
    virtual double* informationData() = 0;

    //! computes the chi2 based on the cached error value, only valid after
    //! computeError has been called.
    [[nodiscard]] virtual double chi2() const = 0;

    /**
     * Linearizes the constraint in the edge.
     * Makes side effect on the vertices of the graph by changing
     * the parameter vector b and the hessian blocks ii and jj.
     * The off diagonal block is accessed via _hessian.
     */
    virtual void constructQuadraticForm() = 0;

    /**
     * maps the internal matrix to some external memory location,
     * you need to provide the memory before calling constructQuadraticForm
     * @param d the memory location to which we map
     * @param i index of the vertex i
     * @param j index of the vertex j (j > i, upper triangular fashion)
     * @param rowMajor if true, will write in rowMajor order to the block. Since
     * EIGEN is columnMajor by default, this results in writing the transposed
     */
    virtual void mapHessianMemory(double* d, int i, int j, bool rowMajor) = 0;

    /**
     * Linearizes the constraint in the edge in the manifold space, and store
     * the result in the given workspace
     */
    virtual void linearizeOplus(JacobianWorkspace& jacobianWorkspace) = 0;

    /** set the estimate of the to vertex, based on the estimate of the from
     * vertices in the edge.
     */
    virtual void initialEstimate(const OptimizableGraph::VertexSet& from,
                                 OptimizableGraph::Vertex* to) = 0;

    /**
     * override in your class if it's possible to initialize the vertices in
     * certain combinations. The return value may correspond to the cost for
     * initializing the vertex but should be positive if the initialization is
     * possible and negative if not possible.
     */
    virtual double initialEstimatePossible(
        const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to) {
      (void)from;
      (void)to;
      return -1.;
    }

    //! returns the level of the edge
    [[nodiscard]] int level() const { return level_; }
    //! sets the level of the edge
    void setLevel(int l) { level_ = l; }

    //! returns the dimensions of the error function
    [[nodiscard]] int dimension() const { return dimension_; }
    [[nodiscard]] virtual int dimensionAtCompileTime() const = 0;

    virtual Vertex* createVertex(int) { return nullptr; }

    //! the internal ID of the edge
    [[nodiscard]] int64_t internalId() const { return internalId_; }

    OptimizableGraph* graph();
    [[nodiscard]] const OptimizableGraph* graph() const;

    bool setParameterId(int argNum, int paramId);
    [[nodiscard]] std::shared_ptr<Parameter> parameter(int argNo) const {
      return parameters_.at(argNo);
    }
    [[nodiscard]] size_t numParameters() const { return parameters_.size(); }

    [[nodiscard]] const std::vector<int>& parameterIds() const {
      return parameterIds_;
    }

    //! return the number of vertices at compile time
    [[nodiscard]] virtual int numVerticesAtCompileTime() const { return -1; }

   protected:
    int dimension_ = -1;
    int level_ = 0;
    std::shared_ptr<RobustKernel> robustKernel_;
    int64_t internalId_;

    void resizeParameters(size_t newSize) {
      parameters_.resize(newSize, nullptr);
      parameterIds_.resize(newSize, -1);
      parameterTypes_.resize(newSize, typeid(void*).name());
    }

    template <typename ParameterType>
    bool installParameter(size_t argNo, int paramId = -1) {
      if (argNo >= parameters_.size()) return false;
      parameterIds_[argNo] = paramId;
      parameterTypes_[argNo] = typeid(ParameterType).name();
      return true;
    }

    template <typename CacheType>
    std::shared_ptr<CacheType> resolveCache(
        const std::shared_ptr<OptimizableGraph::Vertex>& v,
        const std::string& type, const ParameterVector& parameters);

    bool resolveParameters();
    virtual bool resolveCaches();

    std::vector<std::string> parameterTypes_;
    ParameterVector parameters_;
    std::vector<int> parameterIds_;
  };

  //! returns the vertex number <i>id</i> appropriately casted
  std::shared_ptr<Vertex> vertex(int id);
  std::shared_ptr<const OptimizableGraph::Vertex> vertex(int id) const;

  //! empty constructor
  OptimizableGraph();
  ~OptimizableGraph() override;

  /**
   * adds a new vertex.
   * @return false if a vertex with the same id as v is already in the graph,
   * true otherwise.
   */
  virtual bool addVertex(const std::shared_ptr<HyperGraph::Vertex>& v,
                         const std::shared_ptr<HyperGraph::Data>& userData);
  bool addVertex(const std::shared_ptr<HyperGraph::Vertex>& v) override {
    auto noData = std::shared_ptr<HyperGraph::Data>();
    return addVertex(v, noData);
  }

  //! removes a vertex from the graph. Returns true on success (vertex was
  //! present)
  bool removeVertex(const std::shared_ptr<HyperGraph::Vertex>& v,
                    bool detach = false) override;

  /**
   * adds a new edge.
   * The edge should point to the vertices that it is connecting
   * (setFrom/setTo).
   * @return false if the insertion does not work (incompatible types of the
   * vertices/missing vertex). true otherwise.
   */
  // virtual bool addEdge(const std::shared_ptr<HyperGraph::Edge>& e);
  bool addEdge(const std::shared_ptr<HyperGraph::Edge>& e) override;

  /**
   * overridden from HyperGraph, to maintain the bookkeeping of the
   * caches/parameters and jacobian workspaces consistent upon a change in the
   * vertex.
   * @return false if something goes wrong.
   */
  bool setEdgeVertex(const std::shared_ptr<HyperGraph::Edge>& e, int pos,
                     const std::shared_ptr<HyperGraph::Vertex>& v) override;

  //! returns the chi2 of the current configuration
  double chi2() const;

  //! return the maximum dimension of all vertices in the graph
  int maxDimension() const;

  //! Recompute the size of the Jacobian workspace from all the
  //! edges in the graph.
  void recomputeJacobianWorkspaceSize() {
    jacobianWorkspace_.updateSize(*this, true);
  }

  /**
   * iterates over all vertices and returns a set of all the vertex dimensions
   * in the graph
   */
  std::set<int> dimensions() const;

  /**
   * carry out n iterations
   * @return the number of performed iterations
   */
  virtual int optimize(int iterations, bool online = false);

  //! called at the beginning of an iteration (argument is the number of the
  //! iteration)
  virtual void preIteration(int);
  //! called at the end of an iteration (argument is the number of the
  //! iteration)
  virtual void postIteration(int);

  //! add an action to be executed before each iteration
  bool addPreIterationAction(std::shared_ptr<HyperGraphAction> action);
  //! add an action to be executed after each iteration
  bool addPostIterationAction(std::shared_ptr<HyperGraphAction> action);

  //! remove an action that should no longer be executed before each iteration
  bool removePreIterationAction(
      const std::shared_ptr<HyperGraphAction>& action);
  //! remove an action that should no longer be executed after each iteration
  bool removePostIterationAction(
      const std::shared_ptr<HyperGraphAction>& action);

  //! push the estimate of all variables onto a stack
  virtual void push();
  //! pop (restore) the estimate of all variables from the stack
  virtual void pop();
  //! discard the last backup of the estimate for all variables by removing it
  //! from the stack
  virtual void discardTop();

  //! load the graph from a stream. Uses the Factory singleton for creating the
  //! vertices and edges.
  virtual bool load(std::istream& is, io::Format format = io::Format::kG2O);
  bool load(const char* filename, io::Format format = io::Format::kG2O);
  /**
   * @brief Save the graph into a stream
   *
   * @param os Output stream. Note: Has to be opened with std::ios::binary for
   * writing in binary format.
   * @param format Format for saving the data, see also io::Format
   * @param level Level of the graph to save
   * @return true if successful
   * @return false otherwise.
   */
  virtual bool save(std::ostream& os, io::Format format = io::Format::kG2O,
                    int level = 0) const;
  //! function provided for convenience, see save() above
  bool save(const char* filename, io::Format format = io::Format::kG2O,
            int level = 0) const;

  //! save a subgraph to a stream. Again uses the Factory system.
  bool saveSubset(std::ostream& os, HyperGraph::VertexSet& vset,
                  io::Format format = io::Format::kG2O, int level = 0);

  //! save a subgraph to a stream. Again uses the Factory system.
  bool saveSubset(std::ostream& os, HyperGraph::EdgeSet& eset,
                  io::Format format = io::Format::kG2O);

  //! push the estimate of a subset of the variables onto a stack
  virtual void push(HyperGraph::VertexSet& vset);
  //! pop (restore) the estimate a subset of the variables from the stack
  virtual void pop(HyperGraph::VertexSet& vset);
  //! ignore the latest stored element on the stack, remove it from the stack
  //! but do not restore the estimate
  virtual void discardTop(HyperGraph::VertexSet& vset);

  //! fixes/releases a set of vertices
  virtual void setFixed(HyperGraph::VertexSet& vset, bool fixed);

  /**
   * set the renamed types lookup from a string, format is for example:
   * VERTEX_CAM=VERTEX_SE3:EXPMAP,EDGE_PROJECT_P2MC=EDGE_PROJECT_XYZ:EXPMAP
   * This will change the occurrence of VERTEX_CAM in the file to
   * VERTEX_SE3:EXPMAP
   */
  void setRenamedTypesFromString(const std::string& types);

  /**
   * test whether a solver is suitable for optimizing this graph.
   * @param solverProperty the solver property to evaluate.
   * @param vertDims should equal to the set returned by dimensions() to avoid
   * re-evaluating.
   */
  bool isSolverSuitable(const OptimizationAlgorithmProperty& solverProperty,
                        const std::set<int>& vertDims = std::set<int>()) const;

  //! remove all edges and vertices
  void clear() override;
  /**
   * remove the parameters of the graph
   */
  virtual void clearParameters();

  bool addParameter(const std::shared_ptr<Parameter>& p) {
    return parameters_.addParameter(p);
  }

  std::shared_ptr<Parameter> parameter(int id) {
    return parameters_.getParameter(id);
  }

  /**
   * verify that all the information of the edges are semi positive definite,
   * i.e., all Eigenvalues are >= 0.
   * @param verbose output edges with not PSD information matrix by logging
   * @return true if all edges have PSD information matrix
   */
  bool verifyInformationMatrices(bool verbose = false) const;

  //! the workspace for storing the Jacobians of the graph
  JacobianWorkspace& jacobianWorkspace() { return jacobianWorkspace_; }
  const JacobianWorkspace& jacobianWorkspace() const {
    return jacobianWorkspace_;
  }

  /**
   * Eigen starting from version 3.1 requires that we call an initialize
   * function, if we perform calls to Eigen from several threads.
   * Currently, this function calls Eigen::initParallel if g2o is compiled
   * with OpenMP support and Eigen's version is at least 3.1
   */
  static bool initMultiThreading();

  ParameterContainer& parameters() { return parameters_; }
  const ParameterContainer& parameters() const { return parameters_; }

  //! apply a unary function to all vertices
  void forEachVertex(const std::function<void(OptimizableGraph::Vertex*)>& fn);
  //! apply a unary function to the vertices in vset
  static void forEachVertex(
      HyperGraph::VertexSet& vset,
      const std::function<void(OptimizableGraph::Vertex*)>& fn);

 protected:
  std::unordered_map<std::string, std::string> renamedTypesLookup_;
  int64_t nextEdgeId_;
  std::vector<HyperGraphActionSet> graphActions_;

  ParameterContainer parameters_;
  JacobianWorkspace jacobianWorkspace_;

  void performActions(int iter, HyperGraphActionSet& actions);

  // helper functions to save an individual vertex
  static bool saveVertex(AbstractGraph& abstract_graph, Vertex* v);

  // helper functions to save an individual edge
  static bool saveEdge(AbstractGraph& abstract_graph, Edge* e);
};

/**
  @}
 */

}  // namespace g2o

#endif
