There is some relevant change in the devel release of g2o.

Here is the list

*CHANGE IN THE ACCESSORS*
The following non const accessors have been removed

Vertex class:
EstType& Vertex::estimate(); // use Vertex::setEstimate(const EstType& est);

Edge class:
MeasType& Edge::measurement(); // use Edge::setMeasurement(const MeasType& m);
MeasType& Edge::inverseMeasurement(); // the inverse measurement should be computed directly by setMeasurement. not possible to set it by hand to avoid inconsistencies

SE3Quat class:
Vector3d& SE3Quat::translation(); // use SE3Quat::setTranslation(...) instead;
Quarerniond& SE3Quat::rotation(); // use SE3Quat::setRotation(...) instead; This also ensures that the quaternion is normalized and w>0.
double& opertor[];                // we disabled the write access to se3 elements to avoid inconsistencied in the representation.
				  // people like to introduce inconsistencies in the representation.

SE2 class:
Vector2d& SE2::translation();     // use SE2::setTranslation(...) instead. 
Rotation2Dd& SE2::rotation();     // use SE2::setRotation(...) instead. This ensures that the orientation is normalized
double& opertor[];                // removed, same as in the list before.


*PARAMETERS*
It is possible to add parameters blocks in a graph.
Parameters are be quantities that are fixed during the optimization,
like the offset of a sensor or the intrinsics of a camera.

Parameters are identified by a unique int id.
See the Parameters class in optimizable_graph.


If you want to create a parameter block for a graph you have to

1) extend the class Parameters

class MyParams: public Parameters {
      ...
      ...
      virtual void read(istream& os) {
      	      //implement here your read function;
      }

      virtual void write(ostream& os) {
      	      //implement here your write function;
      }

}

2) register the parameters in the metatype system

factory->registerType("MY_PARAMS", new HyperGraphElementCreator<MyParams>);
The parameters are always saved at the beginning of a graph file


3) to insert parameters in a graph you have to
 
// create the parameters
MyParams* p=new MyParams();

// set an id
p->setId(0);

// add them to the graph
if (opt->addParameters(p)){
   cerr << "success" << endl;
} else {
   cerr << "fail" << endl;
}

4) to access the parameters by id you can use the 
OptimizableGraph::parameters(int id) function;

MyParams* p = dynamic_cast<MyParams*>opt->(parameters(id));


*CACHE*

   A cache is some structure that contains intermediate 
   calculations that depend on the estimate stored in a vertex.
   Cache stores some intermediate result that would be computer
   over and over again, during the computeError() of edges that involve the same vertex.
   g2o now supports the caches, however it is good practice to use them
   only when an initial system is running well, to get faster computation.

   For instance a cache of an SE3 can contain a rotation matrix, 
   its opposite, a useful quantity to compute the Jacobian and so on.

   Each vertex can have zero or more cache blocks, 
   whose pointers are stored in a vector within the vertex 
   (in _cacheVector).
   Thus each cache block is indexed within a vertex by a unique 
   id   (VertexCache::_id), that corresponds to its position in the
   _cacheVector.

   Cache blocks are dynamic, in the sense that they are created 
   only by those edges that require them, when they are inserted 
   in a graph (addEdge.) 
   To construct a cache, an edge should:
   -  tell which cache_id is associated to each of the 
      connected vertices.  These ids are stored in a vector 
      stored inside the edge (_cacheIds).
   - implement a function that "creates" a cache for a certain vertex
     VertexCache* createCache(int vertexNum, OptimizableGraph* g).


   So in short, if you want to use a cache you should:
   1) Extend the VertexCache class to do the right things
      In this class you have to implement the update() method,
      that will be called whenever the estimate of the 
      corresponding vertex changes.

   2) In an edge where you want to use a cache, you should
      define a createCache(int vertexNum) function that creates a 
      new instance of cache for the vertex at position 
      vertexNum withing Edge::_vertices.

   3) Before inserting an edge that uses a cache in a graph,
      you should tell the system which cache id is associated to 
      each vertex of the edge. This is done by filling the _cacheIds
      vector with the ids. An id of -1 means no cache.
   
   4) Each method that changes the estimate of a vertex 
      (e.g. oplus) should update the cache accordingly.


As an example:

class MyCache: public VertexCache{
	MyCache(Vertex*v, int id): VertexCache(v, id);	
	virtual void update(){
	// update here the cache 
	//contents based on the vertex estimate
	}
};

class EdgeThatUsesACache: public EdgeThatDoesNotUseACache{
	EdgeThatUsesACache(): EdgeThatDoesNotUseACache{
		// resize the cahce vector, all empty caches
		_cacheIds.resize(vertices().size(), -1);
	}
	
	
	virtual VertexCache* createCache(int vertexNum) {
		OptimizableGraph::Vertex*v 
		= (OptimizableGraph::Vertex*)vertices()[vertexNum];
		
		if (vertexNum==0){	
			return new MyCache(v, _cacheIds[0]);
		}
		if (vertexNum==1){
			return new AnotherCache(v, _cacheIds[1]);
		}
		....
	}

	void computeError() {
		// get the cache of the first vertex
		OptimizableGraph::Vertex*v = 	
		  (OptimizableGraph::Vertex*v) 
		     vertices[0];
		MyCache*  c1= 
		  static_cast<MyCache*>  
		  (v->getCache(_cacheIds[0]));

		// do things with the cache....

	}
};


When inserting an edge that uses the cache in a graph, you should do the following actions *in sequence*:
1) set the vertices in the edge

EdgeThatUsesACache e = new EdgeThatUsesACache();
e->vertices[0]=v0;
e->vertices[1]=v1;
...
e->vertices[n]=vn;


2) assign a cache ID to each vertex in the edge that has a cache that is used for that edge. *The cacheId should be unique for each type of cache, and as small as possible*.

e->setCacheId(0,first_cache_id);
e->setCacheId(1,second_cache_id);
e->setCacheId(2,third_cache_id);

3) Add the edge in the graph. This will take care of all necessary bookeeping.

g.addEdge(e);

*meaning of the cache_id*
The cache id is the position of a cache in the _vertexCache vector.
If you have multiple edges that leave from a vertex and use the same cache id,
only one cache block will be allocated for that id, and it will be shared among
all edges. 


CACHE VERY IMPORTANT
Any vertex type that uses a cache should call the updateCache() function in each method that can change the estimate.
These include, for instance (oplus(...), setEstimate(...)

For instance

class MyVertexThatUsesACache {
      //
      void oplus(double* u){
      	   // do the oplus things here
	   
	   updateCache(); // update the cache vector when the estimate is changes
      }

      ...
};
