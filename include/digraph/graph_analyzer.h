/**
 * (c) 2024 Houssem Guissouma.
 * Licensed under the MIT License.
 *
 * @file digraph/graph_analyzer.h
 * @brief Definition and implementation of the template class DiGraph.
 */
#pragma once

#include <stdexcept>
#include <string>
#include <unordered_set>
#include <unordered_map>

#include "digraph/vertex.h"
#include "digraph/graph_manager.h"
#include "digraph/traversal.h"

namespace digraph {

/**
 * @brief Analyzer of the properties of a directed graph.
 */
template <typename VertexDataType, typename EdgeType, typename EdgeDataType>
class GraphAnalyzer {
public:
    // Template aliases for readability.
    using VertexTemp = Vertex<VertexDataType, EdgeType, EdgeDataType>;
    using GraphManagerTemp = GraphManager<VertexDataType, EdgeType, EdgeDataType>;
    using TraversalTemp = Traversal<VertexDataType, EdgeType, EdgeDataType>;

    /**
	 * @brief Constructs a graph analyzer and initializes its references.
	 *
	 * @param vertices Reference to the vertices matrix of the graph.
	 * @param graph_manager Reference to the vertex manager of the graph.
	 * @param traversal Reference to the traversal module of the graph.
	 */
    GraphAnalyzer(const std::unordered_map<std::string, VertexTemp>& vertices,
                  const GraphManagerTemp& graph_manager, 
                  const TraversalTemp& traversal)
        : vertices_(vertices), graph_manager_(graph_manager), traversal_(traversal) {}

    /**
     * @brief Gets the depth of the graph starting from a given vertex.
     *
     * @param start_id ID of the starting vertex.
     * @returns Depth of the graph.
     * @throws std::runtime_error if the start vertex does not exist.
     */
    int getDepth(const std::string& start_id) const {
        // Check if the starting vertex exists
        if (!graph_manager_.vertexExists(start_id)) {
            throw std::runtime_error("Starting vertex with ID '" + start_id + "' does not exist!");
        }
        // Use a set to keep track of visited vertices
        std::unordered_set<std::string> visited;
        return calculateDepth(start_id, visited) - 1; // Subtract 1 to get depth from the start vertex
    }

    /**
     * @brief Checks whether the graph is a tree.
     *
     * A graph is a tree if and only if:\n
     *      1) It is acyclic.\n
     *      2) It is connected.\n
     *      3) There is exactly one root node (node with no incoming edges) and
               all other nodes have exactly one incoming edge.
     *
     * @returns True if the graph is a tree, false otherwise.
     */
    bool isTree() const {
        if (vertices_.empty()) {
            return true; // An empty graph is considered a tree.
        }

        // Check if the graph has exactly one root
        auto [has_root, root_node] = hasOneRoot();
        if (!has_root) {
            return false;
        }

        // Check if the graph is acyclic
        if (isCyclic()) {
            return false;
        }

        // Check if the graph is connected starting from the root node
        return isConnectedFromRoot(root_node);
    }

    /**
     * @brief Checks if the graph has exactly one root node.
     *
     * A root node is a node with no incoming edges.
     *
     * @returns A pair where the first element is true if exactly one root is found,
     *          and the second element is the root node's ID if it exists.
     */
    std::pair<bool, std::string> hasOneRoot() const {
        std::unordered_map<std::string, int> in_degree;
        // Initialize in-degrees to 0 for all vertices
        for (const auto& vertex : vertices_) {
            in_degree[vertex.first] = 0;
        }

        // Calculate in-degrees using getPredecessors()
        for (const auto& vertex : vertices_) {
            auto predecessors = graph_manager_.getPredecessors(vertex.first);
            for (const std::string& pred : predecessors) {
                in_degree[vertex.first]++;
            }
        }

        std::string root_node;
        int root_count = 0;
        for (const auto& pair : in_degree) {
            if (pair.second == 0) {  // No incoming edges, potential root node
                root_node = pair.first;  // Set the root node
                root_count++;
            }

            // If more than one root, return false
            if (root_count > 1) {
                return { false, "" };
            }
        }

        // Return true and the root node if exactly one root is found
        return { root_count == 1, root_node };
    }

    /**
     * @brief Checks whether the graph contains a cycle.
     *
     * @returns True if the graph contains a cycle, false otherwise.
     */
    bool isCyclic() const {
        std::unordered_set<std::string> visited;
        std::unordered_set<std::string> rec_stack; // To keep track of vertices in the recursion stack

        // Check all vertices in the graph
        for (const auto& vertex : vertices_) {
            if (visited.find(vertex.first) == visited.end()) {
                if (hasCycle(vertex.first, visited, rec_stack)) {
                    return true; // Cycle detected
                }
            }
        }
        return false; // No cycle detected
    }

private:
    /**
     * @brief Recursively calculates the depth of the graph.
     *
     * @param vertex_id ID of the current vertex.
     * @param visited Set of visited vertices to avoid infinite loops.
     * @returns Maximum depth from the current vertex.
     */
    int calculateDepth(const std::string& vertex_id,
        std::unordered_set<std::string>& visited) const {
        if (visited.find(vertex_id) != visited.end()) {
            return 0; // Avoid cycles by returning 0 if the vertex is already visited
        }

        visited.insert(vertex_id);

        int max_depth = 0;
        for (const auto& successor : vertices_.at(vertex_id).getSuccessors()) {
            max_depth = std::max(max_depth, calculateDepth(successor.first, visited));
        }
        return max_depth + 1;
    }

    /**
     * @brief Checks if the graph is connected from the root node.
     *
     * Uses BFS to check if all nodes are reachable from the root node.
     *
     * @param root_node The ID of the root node.
     * @returns True if all nodes are reachable from the root node, false otherwise.
     */
    bool isConnectedFromRoot(const std::string& root_node) const {
        auto level_map = traversal_.breadthFirstSearch(root_node);

        // Count the total number of nodes visited in the BFS (reachable nodes)
        int visited_count = 0;
        for (const auto& level : level_map) {
            visited_count += level.second.size();
        }

        return visited_count == vertices_.size();
    }

    /**
     * @brief Detects a cycle in the graph using DFS.
     *
     * @param vertex_id ID of the current vertex.
     * @param visited Set of visited vertices.
     * @param recurse_stack Set of vertices currently in the recursion stack.
     * @returns True if a cycle is detected, false otherwise.
     */
    bool hasCycle(const std::string& vertex_id,
        std::unordered_set<std::string>& visited,
        std::unordered_set<std::string>& recurse_stack) const {
        // Mark the current node as visited and add it to the recursion stack
        visited.insert(vertex_id);
        recurse_stack.insert(vertex_id);

        // Recur for all adjacent vertices
        const auto& successors = vertices_.at(vertex_id).getSuccessors();
        for (const auto& successor : successors) {
            const std::string& adj_vertex = successor.first;

            // If the adjacent vertex is in the recursion stack, then there is a cycle
            if (recurse_stack.find(adj_vertex) != recurse_stack.end()) {
                return true;
            }

            // If the adjacent vertex has not been visited, recurse on it
            if (visited.find(adj_vertex) == visited.end()) {
                if (hasCycle(adj_vertex, visited, recurse_stack)) {
                    return true;
                }
            }
        }

        // Remove the vertex from the recursion stack
        recurse_stack.erase(vertex_id);
        return false;
    }

    /// Reference to the vertices matrix of the graph.
    const std::unordered_map<std::string, VertexTemp>& vertices_;
    /// Reference to the vertex manager of the graph.
    const GraphManagerTemp& graph_manager_;
    /// Reference to the traversal module of the graph.
    const TraversalTemp& traversal_;
};

} // namespace digraph
