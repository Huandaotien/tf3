cmake_minimum_required(VERSION 3.0.2)

project(tf3)

find_package(console_bridge REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(include ${console_bridge_INCLUDE_DIRS})

# export user definitions

#CPP Libraries
add_library(tf3 src/cache.cpp src/buffer_core.cpp src/static_cache.cpp)
target_link_libraries(tf3 ${Boost_LIBRARIES} ${console_bridge_LIBRARIES})

add_executable(simple_tf3_example examples/simple_tf3_example.cpp)
target_include_directories(simple_tf3_example PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(simple_tf3_example PRIVATE tf3 pthread ${console_bridge_LIBRARIES})

install(TARGETS tf3
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# Tests
option(BUILD_TESTING "Build tests" OFF)
if(BUILD_TESTING)
  enable_testing()
  add_executable(test_cache_unittest test/cache_unittest.cpp)
  target_link_libraries(test_cache_unittest tf3 ${console_bridge_LIBRARIES})

  add_executable(test_static_cache_unittest test/static_cache_test.cpp)
  target_link_libraries(test_static_cache_unittest tf3 ${console_bridge_LIBRARIES})

  add_executable(test_simple test/simple_tf3_core.cpp)
  target_link_libraries(test_simple tf3 ${console_bridge_LIBRARIES})

  add_executable(speed_test test/speed_test.cpp)
  target_link_libraries(speed_test tf3 ${console_bridge_LIBRARIES})

  add_executable(test_transform_datatypes test/test_transform_datatypes.cpp)
  target_link_libraries(test_transform_datatypes tf3 ${console_bridge_LIBRARIES})

  add_test(NAME test_cache_unittest COMMAND test_cache_unittest)
  add_test(NAME test_static_cache_unittest COMMAND test_static_cache_unittest)
  add_test(NAME test_simple COMMAND test_simple)
  add_test(NAME speed_test COMMAND speed_test)
  add_test(NAME test_transform_datatypes COMMAND test_transform_datatypes)
endif()
