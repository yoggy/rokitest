cmake_minimum_required (VERSION 2.8)
    
project (roki_test)

set (LIBS roki zeo zm cure m ncurses)
set (CMAKE_CXX_FLAGS "-Wall -Wl,'--no-as-needed'")

add_executable (rokitest00_simple rokitest00_simple.cpp)
target_link_libraries (rokitest00_simple ${LIBS})

add_executable (rokitest01_freefall rokitest01_freefall.cpp)
target_link_libraries (rokitest01_freefall ${LIBS})

add_executable (rokitest02_collision rokitest02_collision.cpp)
target_link_libraries (rokitest02_collision ${LIBS})

add_executable (rokitest03_joint rokitest03_joint.cpp)
target_link_libraries (rokitest03_joint ${LIBS})

add_executable (rokitest04_motor rokitest04_motor.cpp)
target_link_libraries (rokitest04_motor ${LIBS})

add_executable (rokitest05_underconstructionjoint rokitest05_underconstructionjoint.cpp)
target_link_libraries (rokitest05_underconstructionjoint ${LIBS})

add_executable (rokitest06_prism rokitest06_prism.cpp)
target_link_libraries (rokitest06_prism ${LIBS})

add_executable (rokitest07_failure rokitest07_failure.cpp)
target_link_libraries (rokitest07_failure ${LIBS})

add_executable (rokitest08_addremove rokitest08_addremove.cpp)
target_link_libraries (rokitest08_addremove ${LIBS})
