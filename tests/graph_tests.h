/**
 * (c) 2024 Houssem Guissouma.
 * Licensed under the MIT License.
 *
 * @file tests/graph_tests.h
 * @brief Definition of the fixture class GraphTests.
 */
#pragma once

#include <gtest/gtest.h>

// Module under test
#include "digraph/digraph.h"

namespace tests {

/**
 * @brief Fixture for testing the DiGraph class and its methods.
 */
class GraphTests : public ::testing::Test {
protected:
    /**
     * @brief Sets up the data of the test suite.
     */
    GraphTests();

    /**
	 * @brief Builds the simple directed graph by adding its vertices and edges.
	 */
    void buildSimpleGraph();

    /**
     * @brief Builds the simple calculation tree by adding its vertices and edges.
     */
    void buildSimpleCalcTree();

    /**
	 * @brief Builds the machine learning algorithm tree by adding its vertices and edges.
	 */
    void buildMlAlgoTree();

    /// Simple directed graph for testing.
    digraph::DiGraph<int> simple_graph_;
    /// Directed tree corresponding to the feature model "simple calculation".
    digraph::DiGraph<int> simple_calc_tree_;
    /// Directed tree of machine learning algorithms.
    digraph::DiGraph<int> ml_algo_tree_;
};

} // namespace tests
