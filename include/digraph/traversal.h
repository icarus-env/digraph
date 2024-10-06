/**
 * (c) 2024 Houssem Guissouma.
 * Licensed under the MIT License.
 *
 * @file digraph/traversal.h
 * @brief Definition and implementation of the template class DiGraph.
 */
#pragma once

#include <map>
#include <queue>
#include <stack>

namespace digraph {

/**
 * @brief Class providing basic traversal algorithms for a directed graph.
 */
template <typename VertexDataType, typename EdgeType, typename EdgeDataType>
class Traversal {
public:
    // Template aliases for readability.
    using VertexTemp = Vertex<VertexDataType, EdgeType, EdgeDataType>;
    using GraphManagerTemp = GraphManager<VertexDataType, EdgeType, EdgeDataType>;

    /**
	 * @brief Constructs a Traversal object for given vertices matrix and vertex manager.
	 *
	 * @param vertices Reference to the vertices matrix of the graph.
	 * @param vertex_manager Reference to the vertex manager of the graph.
	 */
    Traversal(const std::unordered_map<std::string, VertexTemp>& vertices, 
              const GraphManagerTemp& vertex_manager)
            : vertices_(vertices), vertex_manager_(vertex_manager) {}

    /**
     * @brief Performs a breadth-first search (BFS) starting from a given vertex.
     *
     * @param start_id ID of the starting vertex.
     * @returns Map with levels as keys and vectors of vertex IDs as values.
     * @throws std::runtime_error if the start vertex does not exist.
     */
    std::map<int, std::vector<std::string>> breadthFirstSearch(
        const std::string& start_id) const {
        if (!vertex_manager_.vertexExists(start_id)) {
            throw std::runtime_error("Starting vertex with ID '" 
                                     + start_id + "' does not exist!");
        }

        // Map to store the vertices at each depth level
        std::map<int, std::vector<std::string>> level_map;
        // Queue to manage the BFS process, storing vertex ID and its depth
        std::queue<std::pair<std::string, int>> to_visit;
        // Set to keep track of visited vertices
        std::unordered_set<std::string> visited;

        // Initialize the BFS with the starting vertex at depth 0
        to_visit.emplace(start_id, 0);
        visited.insert(start_id);

        while (!to_visit.empty()) {
            // Record the current vertex and its depth
            auto [current, depth] = to_visit.front();
            to_visit.pop();
            level_map[depth].push_back(current);

            // Iterate through the successors of the current vertex
            for (const auto& successor : vertices_.at(current).getSuccessors()) {
                // If the successor hasn't been visited, add it to the queue and 
                // mark it as visited
                if (visited.find(successor.first) == visited.end()) {
                    to_visit.emplace(successor.first, depth + 1);
                    visited.insert(successor.first);
                }
            }
        }

        return level_map;
    }

    /**
     * @brief Performs a depth-first search (DFS) starting from a given vertex.
     *
     * @param start_id ID of the starting vertex.
     * @returns Vector of visited vertex IDs in DFS order.
     * @throws std::runtime_error if the start vertex does not exist.
     */
    std::vector<std::string> depthFirstSearch(const std::string& start_id) const {
        if (!vertex_manager_.vertexExists(start_id)) {
            throw std::runtime_error("Starting vertex with ID '" 
                                     + start_id + "' does not exist!");
        }

        // Vector to store the order of visited vertices
        std::vector<std::string> visited_order;
        // Stack and queue to manage the DFS process
        std::stack<std::string> to_visit;
        std::unordered_set<std::string> visited;

        // Initialize the DFS with the starting vertex
        to_visit.push(start_id);
        visited.insert(start_id);

        while (!to_visit.empty()) {
            // Get the current vertex
            std::string current = to_visit.top();
            to_visit.pop();
            visited_order.push_back(current);

            // Iterate through the successors of the current vertex
            for (const auto& successor : vertices_.at(current).successors) {
                // If the successor hasn't been visited, add it to the stack 
                // and mark it as visited
                if (visited.find(successor.first) == visited.end()) {
                    to_visit.push(successor.first);
                    visited.insert(successor.first);
                }
            }
        }

        return visited_order;
    }

private:
    /// Reference to the vertices matrix of the graph.
    const std::unordered_map<std::string, VertexTemp>& vertices_;
    /// Reference to the vertex manager of the graph.
    const GraphManagerTemp& vertex_manager_;
};

} // namespace digraph
