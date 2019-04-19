add_library( MR_pose MR_POSE.cpp)
target_link_libraries(MR_pose  ${THIRD_PARTY_LIBS})

add_executable(demo demo.cpp)
target_link_libraries(demo MR_pose)