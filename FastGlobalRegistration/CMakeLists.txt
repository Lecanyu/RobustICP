find_package(PCL 1.8 REQUIRED)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_library(FastGlobalRegistrationLib ${FastGlobalRegistration_LINK_MODE} app.cpp app.h PointCloud3d.h)


add_executable(FastGlobalRegistration main.cpp)
add_executable(Evaluation evaluation.cpp)
add_executable(test_pointcloud3d test_pointcloud3d.cpp)



TARGET_LINK_LIBRARIES(FastGlobalRegistration FastGlobalRegistrationLib)
TARGET_LINK_LIBRARIES(Evaluation FastGlobalRegistrationLib)
TARGET_LINK_LIBRARIES(test_pointcloud3d FastGlobalRegistrationLib ${PCL_LIBRARIES})
