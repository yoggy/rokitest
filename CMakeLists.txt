cmake_minimum_required (VERSION 2.8)
    
project (roki_test)

set (LIBS roki zeo zm cure m ncurses)
set (CMAKE_CXX_FLAGS "-Wall -Wl,'--no-as-needed'")

add_executable (rokitest01_freefall rokitest01_freefall.cpp)
target_link_libraries(rokitest01_freefall ${LIBS})

add_executable (rokitest02_collision rokitest02_collision.cpp)
target_link_libraries(rokitest02_collision ${LIBS})

add_executable (rokitest03_joint rokitest03_joint.cpp)
target_link_libraries(rokitest03_joint ${LIBS})
