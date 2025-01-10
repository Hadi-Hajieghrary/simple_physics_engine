cmake_minimum_required(VERSION 3.10)
project(simple_physics_engine VERSION 1.0
				DESCRIPTION "Simple Physics Engine"
				LANGUAGES CXX)


find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(include)

add_library(lbasic_geometry src/basic_geometry.cpp)
target_link_libraries(lbasic_geometry Eigen3::Eigen)

add_library(lgeometry src/geometry.cpp)
target_link_libraries(lgeometry lbasic_geometry Eigen3::Eigen)

add_executable(main main.cpp)




# Enable testing
enable_testing()
# Find GoogleTest (you can change the location if needed)
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
link_directories(${GTEST_LIBRARY_DIRS})
set(TEST_SOURCES 
    test/test_geometry.cpp  # Add your test files here
)
# Create the test executable
add_executable(GeometryTests ${TEST_SOURCES})
# Link the test executable with GoogleTest and pthread (needed by GoogleTest)
target_link_libraries(GeometryTests gtest_main ${GTEST_LIBRARIES} pthread lgeometry lbasic_geometry)
# Enable the tests
add_test(NAME GeometryTests COMMAND GeometryTests)