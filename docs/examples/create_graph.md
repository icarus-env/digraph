@page create_graph Creating a Graph

To create a graph, use the default constructor of [`DiGraph`](classdigraph_1_1_di_graph.html) and add vertices and edges to it.
```cpp
#include "digraph/digraph.h"

using namespace digraph;

int main() {
    // Create a directed graph with nodes storing integer data
	DiGraph<int> graph;

	// Add vertices to the graph
	graph.addVertex("A");
	graph.addVertex("B");
	// Add an edge between vertices A and B
	graph.addEdge("A", "B");

	return 0;
}
```