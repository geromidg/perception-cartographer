rock_library(cartographer
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(cartographer_bin Main.cpp
    DEPS cartographer)

