find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

rock_library(cartographer
    SOURCES localMap.cpp globalMap.cpp costMap.cpp
    HEADERS localMap.hpp globalMap.hpp costMap.hpp
    DEPS_PKGCONFIG opencv)

rock_executable(cartographer_bin Main.cpp
    DEPS cartographer)

target_link_libraries(cartographer
  ${PCL_LIBRARIES}
  opencv_core)
