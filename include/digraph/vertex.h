/**
 * (c) 2024 Houssem Guissouma.
 * Licensed under the MIT License.
 *
 * @file digraph/vertex.h
 * @brief Definition and implementation of the template class Vertex.
 */
#pragma once

#include <string>
#include <unordered_map>
#include <memory>

namespace digraph {

/**
 * @brief Vertex of a directed graph with stored data and edges to other vertices (successors).
 */
template <typename VertexDataType, typename EdgeType, typename EdgeDataType>
class Vertex {
public:
    /**
     * @brief Structure holding the edge information between two vertices.
	 */
    struct Edge {
        EdgeType type;  ///< Type of the edge.
        std::unique_ptr<EdgeDataType> data;  ///< Data stored in the edge.
    };

    // Constructor to initialize the vertex with an ID and optional data
    Vertex(std::unique_ptr<VertexDataType> data = nullptr)
        : data_(std::move(data)) {}

    /**
     * @brief Adds a successor to the vertex.
     * 
     * @note If a successor with the same ID already exisit, it will be overwritten.
     * 
     * @param successor_id ID of the successor vertex.
     * @param edge_type Type of the edge connecting the vertices.
     * @param edge_data Data associated with the edge.
     */
    void addSuccessor(const std::string& successor_id, 
                      EdgeType edge_type,
                      std::unique_ptr<EdgeDataType> edge_data) {
        successors_[successor_id] = Edge{edge_type, std::move(edge_data)};
    }

    /**
    * @brief Gets the data stored in the vertex.
    * 
    * @returns Pointer to the data stored in the vertex.
	*/
    VertexDataType* getData() const {
        return data_.get();
    }

    /**
	 * @brief Sets the data associated with the vertex (mutator with move semantics).
	 */
    void setData(std::unique_ptr<VertexDataType> data) {
        data_ = std::move(data);
    }

    /**
	 * @brief Gets the successors of the vertex.
	 * 
	 * @returns Map of successors of the vertex and associated edge data.
	 */
    const std::unordered_map<std::string, Edge>& getSuccessors() const {
        return successors_;
    }

    /**
     * @brief Gets the IDs of the successors of the vertex.
     * 
     * @returns Set of IDs of the successors of the vertex.
     */
    std::unordered_set<std::string> getSuccessorIds() const {
        std::unordered_set<std::string> successor_ids;
        for (const auto& successor : successors_) {
            successor_ids.insert(successor.first); // Insert only the ID
        }
        return successor_ids;
    }

private:
    /// Data associated stored in the vertex.
    std::unique_ptr<VertexDataType> data_;
    /// Successors of this vertex with their associated edges.
    std::unordered_map<std::string, Edge> successors_;
};

} // namespace digraph
