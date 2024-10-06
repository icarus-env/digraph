/**
 * (c) 2024 Houssem Guissouma.
 * Licensed under the MIT License.
 * 
 * @file digraph/digraph.h
 * @brief Definition and implementation of the template class DiGraph.
 */
#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>

#include "digraph/graph_analyzer.h"
#include "digraph/graph_manager.h"
#include "digraph/vertex.h"

namespace digraph {

/**
 * @brief Empty enum as the default EdgeType
 */
enum class DefaultEdgeType {};

/**
 * @brief Simple directed graph (no loops, no multiple edges) with unique strings as node IDs.
 * 
 * @tparam VertexDataType Type of data stored in the vertices (saved as unique pointers).
 * @tparam EdgeType (optional) Enumeration representing the type of the edges, if needed.
 * @tparam EdgeDataType (optional) Type of data stored in the edges (saved as unique pointers).
 */
template <typename VertexDataType, 
          typename EdgeType = DefaultEdgeType, 
          typename EdgeDataType = int>
class DiGraph {
    static_assert(std::is_enum<EdgeType>::value, "EdgeType parameter must be an enum type.");

public:
    // Template aliases for readability.
    using VertexTemp = Vertex<VertexDataType, EdgeType, EdgeDataType>;
    using TraversalTemp = Traversal<VertexDataType, EdgeType, EdgeDataType>;
    using GraphAnalyzerTemp = GraphAnalyzer<VertexDataType, EdgeType, EdgeDataType>;

    /**
     * @brief Default constructor.
     */
    DiGraph() 
        : graph_manager_(vertices_), traversal_(vertices_, graph_manager_),
          analyzer_(vertices_, graph_manager_, traversal_) {}

    /**
     * @brief Destructor.
     */
    virtual ~DiGraph() = default;

    /**
     * @brief Gets the analyzer of the graph as interface to the analysis functions.
     */
    const GraphAnalyzerTemp& analyzer() const {
        return analyzer_;
    }

    /**
	 * @brief Gets the traversal of the graph as interface to the traversal algorithms.
	 */
    const TraversalTemp& traversal() const {
        return traversal_;
    }

    /**
     * @brief Adds a vertex (node) to the graph.
     * 
     * @param vertex_id ID of the vertex.
     * @param vertex_data (optional) Data to be stored in the vertex.
     * @throws std::runtime_error If the vertex already exists in the graph.
     */
    void addVertex(const std::string& vertex_id, 
                   std::unique_ptr<VertexDataType> vertex_data = nullptr) {
        graph_manager_.addVertex(vertex_id, std::move(vertex_data));
    }

    /**
     * @brief Adds a list of vertices without data to the graph.
     * 
     * @param vertex_ids List of vertex IDs to be added.
     */
    void addVertices(const std::vector<std::string>& vertex_ids) {
        for (const auto& vertex_id : vertex_ids) {
            addVertex(vertex_id);
        }
    }

    /**
     * @brief Adds an edge between two vertices.
     * 
     * @param source_id ID of the source vertex.
     * @param target_id ID of the target vertex.
     * @param edge_type (optional) Type of the edge connecting the vertices.
     * @param edge_data (optional) Data associated with the edge.
     */
    void addEdge(const std::string& source_id,
                 const std::string& target_id,
                 EdgeType edge_type = EdgeType(),
                 std::unique_ptr<EdgeDataType> edge_data = nullptr) {
        graph_manager_.addEdge(source_id, target_id, edge_type, std::move(edge_data));
    }

    /**
     * @brief Gets the number of vertices in the graph.
     *
     * @returns Number of vertices in the graph.
     */
    int getNumVertices() const {
        return vertices_.size();
    }

    /**
     * @brief Gets the number of edges in the graph.
     *
     * @returns Number of edges in the graph.
     */
    int getNumEdges() const {
        int num_edges = 0;
        for (const auto& vertex_pair : vertices_) {
            const auto& vertex = vertex_pair.second; // Access the Vertex object
            num_edges += vertex.getSuccessors().size();
        }
        return num_edges;
    }

    /**
	 * @brief Gets the data stored in a vertex.
     * 
     * @param vertex_id ID of the vertex.
     * @returns Pointer to the data stored in the vertex.
     * @throws std::out_of_range If the vertex does not exist.
	 */
    VertexDataType* getVertexData(const std::string& vertex_id) const {
		return vertices_.at(vertex_id).getData();
	}

    /**
     * @brief Clears the graph by removing all vertices and edges.
     */
    void clear() {
		graph_manager_.clear();
	}

    /**
     * @brief Checks whether a vertex with a given ID exists in the graph.
     *
     * @param vertex_id ID of the vertex
     * @returns True if the vertex exists, false otherwise
     */
    bool vertexExists(const std::string& vertex_id) const {
        return graph_manager_.vertexExists(vertex_id);
    }

    /**
     * @brief Checks whether the graph is empty (has no vertices inside).
     */
    bool isEmpty() const {
        return vertices_.empty();
    }

    /**
     * @brief Gets the map of the vertices in the graph.
     * 
     * @returns Const reference to the map of the graph's vertices.
     */
    const std::unordered_map<std::string, VertexTemp>& getVertices() const {
        return vertices_;
    }

    /**
     * @brief Gets the successors of a vertex as a set of IDs.
     *
     * @param vertex_id ID of the vertex to get the successors for.
     * @returns Unordered set of successor IDs.
     */
    std::unordered_set<std::string> getSuccessors(const std::string& vertex_id) const {
        return graph_manager_.getSuccessors(vertex_id);
    }
    
    /**
     * @brief Gets the successors of a vertex with a specific edge data value.
     * 
     * @param vertex_id ID of the vertex to get the successors for.
     * @param edge_type (optional) Edge type to filter the successors with.
	 * @returns Map of successor IDs and their corresponding edge data values.
	 */
    std::unordered_set<std::string> getSuccessorsByEdgeType(const std::string& vertex_id,
                                                            const EdgeType& edge_type) const {
        return graph_manager_.getSuccessorsByEdgeType(vertex_id, edge_type);
	}

    /**
     * @brief Gets the predecessors of a given vertex if they exist.
     * 
     * If the vertex does not exist, an empty vector is returned.
     * 
     * @param vertex_id ID of the vertex to get the predecessors for.
     */
    std::vector<std::string> getPredecessors(const std::string& vertex_id) const {
        return graph_manager_.getPredecessors(vertex_id);
    }

    /**
	 * @brief Checks whether a vertex is a successor of another vertex.
	 * 
	 * @param source_id ID of the source vertex.
	 * @param target_id ID of the target vertex.
	 * @returns True if the target vertex is a successor of the source vertex, false otherwise.
	 */
    bool isVertexSuccessor(const std::string& source_id, const std::string& target_id) const {
		return graph_manager_.isVertexSuccessor(source_id, target_id);
	}

private:
    /// Map of vertex IDs to their vertex objects (known as adjacency matrix).
    std::unordered_map<std::string, VertexTemp> vertices_;

    /// Manager for adding, removing and querying vertices and edges.
    GraphManager<VertexDataType, EdgeType, EdgeDataType> graph_manager_;
    /// Module providing traversal algorithms of the graph.
    TraversalTemp traversal_;
    /// Module for analyzing advanced properties of the graph, such as depth and cycles.
    GraphAnalyzerTemp analyzer_;
};

} // namespace digraph
