add_executable(generate_octree main.cpp )
target_link_libraries(generate_octree ${PCL_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(generate_octree stdc++fs)
