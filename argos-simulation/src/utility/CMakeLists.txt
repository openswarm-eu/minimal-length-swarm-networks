add_library(utility SHARED
    team_color.h
    pid.h
    pid.cpp
    sct.h
    sct.cpp
    robot_message.h
    robot_message.cpp)
target_link_libraries(utility
    yaml-cpp
    argos3plugin_simulator_genericrobot)