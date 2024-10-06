/**
 * (c) 2024 Houssem Guissouma.
 * Licensed under the MIT License.
 *
 * @file tests/graph_tests.cpp
 * @brief Implementation of the fixture GraphTests and definition of its test cases.
 */
#include "graph_tests.h"

using namespace digraph;

namespace tests {

// ============================
// GraphTests class
// ============================

GraphTests::GraphTests() {}

void GraphTests::buildSimpleGraph() {
    // Set the simple directed graph
    simple_graph_.addVertex("B");
    simple_graph_.addVertex("C");
    simple_graph_.addVertex("D");
    simple_graph_.addEdge("A", "B");
    simple_graph_.addEdge("A", "C");
    simple_graph_.addEdge("C", "A");
    simple_graph_.addEdge("B", "D");
    // Add two unconnected nodes
    simple_graph_.addVertex("Feature X");
    simple_graph_.addVertex("This is Feature Y");
}

void GraphTests::buildSimpleCalcTree() {
    simple_calc_tree_.addVertices(
        { "simple_calculation",
          "operands", "threshold comparison",
          "two inputs", "three inputs", "compare max", "compare min" }
    );
    simple_calc_tree_.addEdge("simple_calculation", "operands");
    simple_calc_tree_.addEdge("operands", "two inputs");
    simple_calc_tree_.addEdge("operands", "three inputs");
    simple_calc_tree_.addEdge("simple_calculation", "threshold comparison");
    simple_calc_tree_.addEdge("threshold comparison", "compare max");
    simple_calc_tree_.addEdge("threshold comparison", "compare min");
}

void GraphTests::buildMlAlgoTree() {
    ml_algo_tree_.addVertices(
        { "machine learning",
          "supervised", "unsupervised", "reinforcement",
          "regression", "classification", "clustering", "dimensionality reduction",
          "linear regression", "logistic regression", "decision tree", "random forest",
          "k-means", "hierarchical clustering", "PCA", "t-SNE" }
    );
    ml_algo_tree_.addEdge("machine learning", "supervised");
    ml_algo_tree_.addEdge("machine learning", "unsupervised");
    ml_algo_tree_.addEdge("machine learning", "reinforcement");
    ml_algo_tree_.addEdge("supervised", "regression");
    ml_algo_tree_.addEdge("supervised", "classification");
    ml_algo_tree_.addEdge("unsupervised", "clustering");
    ml_algo_tree_.addEdge("unsupervised", "dimensionality reduction");
    ml_algo_tree_.addEdge("regression", "linear regression");
    ml_algo_tree_.addEdge("regression", "logistic regression");
    ml_algo_tree_.addEdge("classification", "decision tree");
    ml_algo_tree_.addEdge("classification", "random forest");
    ml_algo_tree_.addEdge("clustering", "k-means");
    ml_algo_tree_.addEdge("clustering", "hierarchical clustering");
    ml_algo_tree_.addEdge("dimensionality reduction", "PCA");
    ml_algo_tree_.addEdge("dimensionality reduction", "t-SNE");
}

// ============================
// GraphTests tests
// ============================

/**
 * @test Tests the creation of a directed graph and getting its basic stats.
 */
TEST_F(GraphTests, CreateAndGetStats) {
    // Test for the simple graph
    buildSimpleGraph();
    ASSERT_EQ(simple_graph_.getNumVertices(), 6);
    ASSERT_EQ(simple_graph_.getNumEdges(), 4);
    ASSERT_EQ(simple_graph_.analyzer().getDepth("A"), 2);
    ASSERT_EQ(simple_graph_.analyzer().getDepth("C"), 3);

    // Test for the simple calculation tree
    buildSimpleCalcTree();
    ASSERT_EQ(simple_calc_tree_.getNumVertices(), 7);
    ASSERT_EQ(simple_calc_tree_.getNumEdges(), 6);
    ASSERT_EQ(simple_calc_tree_.analyzer().getDepth("simple_calculation"), 2);
}

/**
 * @test Tests if checking if a graph is a tree works correctly.
 */
TEST_F(GraphTests, CheckIfTree) {
    // Test for the simple graph
    buildSimpleGraph();
    // Dependency functions used for checking if the graph is a tree
    ASSERT_TRUE(simple_graph_.analyzer().isCyclic());
    ASSERT_FALSE(simple_graph_.analyzer().hasOneRoot().first);
    // Main verification function
    ASSERT_FALSE(simple_graph_.analyzer().isTree());

    // Test for the simple calculation tree
    buildSimpleCalcTree();
    // Dependency functions used for checking if the graph is a tree
    ASSERT_FALSE(simple_calc_tree_.analyzer().isCyclic());
    ASSERT_TRUE(simple_calc_tree_.analyzer().hasOneRoot().first);
    ASSERT_EQ(simple_calc_tree_.analyzer().hasOneRoot().second, "simple_calculation");
    // Main verification function
    ASSERT_TRUE(simple_calc_tree_.analyzer().isTree());
}

} // namespace tests
