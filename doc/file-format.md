# Content to be saved
- Parameters
- Vertex / Dynamic
  - Data
- Edge / Dynamic Edge
  - Data

The above list gives an overview about the data to be saved.
Each kind of element has a set of data which is agnostic to the concrete instance and we have data specific to the instance.

The agnostic data contains, for example, a single ID or a list of IDs. For a vertex and a parameter we will have an ID followed by the specific payload.

# Types

We have the following trivially copyable types which we need to handle.

- String: A string without spaces
- Int: An integer number
- Float:  A floating point

## Paramater
- String, the tag of the parameter
- Int, the unique ID of the parameter
- [Float], the value of the parameter

## Data
- String, the tag of the data
- [String], the data in serialized form

## Vertex / Dynamic Vertex
- String, the tag of the vertex
- Int, the unique ID of the vertex
- [Float], the estimate of the vertex
- [Data], the data associated to the vertex

## Edge / Dynamic Edge
- String, the tag of the vertex
- [Int], the unique IDs of the edge's vertices
- [Int], the unique IDs of the edge's parameters
- [Float], the measurement of the edge
- [Float], the information matrix of the edge
- [Data], the data associated to the edge

A Data element belongs to a vertex or an edge in a parent/child relation.

## Fixed vertices
- [Int], the vertex IDs which are fixed

# A graph

A graph comprises above information. And we will save it in a specific order.

- "Fixed"
  - [Int]
- "Parameters"
  - [Parameter]
- "Vertices"
  - [Vertex]
- "Edges"
  - [Edge]

# File formats to support

We want to save the original g2o file format but also support potentially new file formats like JSON or a binary format.

## Challenges

To support the original g2o file format, we need to handle the special case of a dynamic edge. However, we can assume that not many files with dynamically sized edges in the g2o format exist.
To support the original g2o format, we need anyhow to implement a custom reader for the data. Hence, we can handle in there the special format to separate IDs, estimate and information matrix.

As we assume only a very limited number of files have been written with dynamically sized edges, we will break compatibility for reading those. This allows as a smooth path forward.

Currently, g2o can only implicitly differentiate if a edge is dynamic. To this end, an edge with zero vertices upon construction is assumed to be a dynamic edge. The same holds for a vertex.
