/**
 * (c) 2024 Houssem Guissouma.
 * Licensed under the MIT License.
 *
 * @file digraph/graph_manager.h
 * @brief Definition and implementation of the template class DiGraph.
 */
#pragma once

#include "digraph/vertex.h"

namespace digraph {

/**
 * @brief Graph manager, providing methods for adding and querying vertices and edges.
 */
template <typename VertexDataType, typename EdgeType, typename EdgeDataType>
class GraphManager {
public:
    // Template alias for readability.
    using VertexTemp = Vertex<VertexDataType, EdgeType, EdgeDataType>;

    explicit GraphManager(std::unordered_map<std::string, VertexTemp>& vertices)
        : vertices_(vertices) {}

    /**
     * @copydoc DiGraph::addVertex
     */
    void addVertex(const std::string& vertex_id,
                   std::unique_ptr<VertexDataType> vertex_data = nullptr) {
        // Check whether the vertex already exists
        if (vertexExists(vertex_id)) {
            throw std::runtime_error("Vertex with ID '" + vertex_id + "' already exists!");
        }
        vertices_[vertex_id] = VertexTemp(std::move(vertex_data));
    }

    /**
     * @copydoc DiGraph::addEdge
     */
    void addEdge(const std::string& source_id,
                 const std::string& target_id,
                 EdgeType edge_type,
                 std::unique_ptr<EdgeDataType> edge_data) {
        // If one of the vertices doesn't exist, add them without data
        if (!vertexExists(source_id)) {
			addVertex(source_id);
		}
        if (!vertexExists(target_id)) {
            addVertex(target_id);
        }

        vertices_.at(source_id).addSuccessor(target_id, edge_type, std::move(edge_data));
    }

    /**
     * @copydoc DiGraph::clear
     */
    void clear() {
        vertices_.clear();
    }

    /**
     * @copydoc DiGraph::vertexExists
     */
    bool vertexExists(const std::string& vertex_id) const {
        return vertices_.find(vertex_id) != vertices_.end();
    }

    /**
     * @copydoc DiGraph::getSuccessors
     */
    std::unordered_set<std::string> getSuccessors(const std::string& vertex_id) const {
        std::unordered_set<std::string> successor_ids;

        for (const auto& successor : vertices_.at(vertex_id).getSuccessors()) {
            successor_ids.insert(successor.first);
        }

        return successor_ids;
    }

    /**
     * @copydoc DiGraph::getSuccessorsByEdgeType
     */
    std::unordered_set<std::string> getSuccessorsByEdgeType(
            const std::string& vertex_id,
            const EdgeType& edge_type) const {
        std::unordered_set<std::string> successor_ids;

        for (const auto& successor : vertices_.at(vertex_id).getSuccessors()) {
            if (successor.second.type == edge_type) {
                successor_ids.insert(successor.first);
            }
        }

        return successor_ids;
    }

    /**
     * @copydoc DiGraph::getPredecessors
     */
    std::vector<std::string> getPredecessors(const std::string& vertex_id) const {
        // Check if the vertex is part of the graph
        if (!vertexExists(vertex_id)) {
            return {};
        }

        std::vector<std::string> predecessors;
        for (const auto& vertex : vertices_) {
            for (const auto& successor : vertex.second.getSuccessors()) {
                if (successor.first == vertex_id) {
                    predecessors.push_back(vertex.first);
                }
            }
        }
        return predecessors;
    }

    /**
     * @copydoc DiGraph::isVertexSuccessor
     */
    bool isVertexSuccessor(const std::string& source_id, const std::string& target_id) const {
        for (const auto& successor : vertices_.at(source_id).getSuccessors()) {
            if (successor.first == target_id) {
                return true;
            }
        }
        return false;
    }

    /// Reference to the vertices matrix of the graph.
    std::unordered_map<std::string, VertexTemp>& vertices_;
};

} // namespace digraph
