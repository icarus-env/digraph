    # CMakeLists.txt of the test executable "icarus-utils-tests"
    add_executable(digraph-tests
                   "src/main.cpp"
                   "src/graph_tests.cpp")
    target_link_libraries(digraph-tests PRIVATE digraph GTest::gtest)
    gtest_discover_tests(digraph-tests)