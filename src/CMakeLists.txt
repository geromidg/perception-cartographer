find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

rock_library(cartographer
    SOURCES localMap.cpp
    HEADERS localMap.hpp
    DEPS_PKGCONFIG opencv)

target_link_libraries(cartographer
  ${PCL_LIBRARIES}
  opencv_core)
