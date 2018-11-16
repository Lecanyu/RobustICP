# Create a shared variable so that it can be used in FastGlobalRegistration directary
set(FALKOLIB_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/include" CACHE INTERNAL "falkolib include location" )
set(FALKOLIB_EXT_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/EXTERNAL" CACHE INTERNAL "falkolib external include location")

include_directories(${FALKOLIB_INCLUDE_DIR} ${FALKOLIB_EXT_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})  

# library can be re-used in different directory
add_library(falkolib 
  src/Common/HoughSpectrum.cpp
  src/Feature/FALKOExtractor.cpp
  src/Feature/OCExtractor.cpp
  src/Feature/CGH.cpp
  src/Feature/BSC.cpp
)


add_executable(testCGHDescriptor test/testCGHDescriptor.cpp test/testData.cpp)
add_executable(testBSCDescriptor test/testBSCDescriptor.cpp test/testData.cpp)
add_executable(testKeypointFalko test/testKeypointFalko.cpp test/testData.cpp)
add_executable(testFalkoAHT test/testFalkoAHT.cpp test/testData.cpp)
add_executable(testFalkoCC test/testFalkoCC.cpp test/testData.cpp)
add_executable(testKeypointOC test/testKeypointOC.cpp test/testData.cpp)

TARGET_LINK_LIBRARIES(testCGHDescriptor falkolib ${Boost_LIBRARIES})
TARGET_LINK_LIBRARIES(testBSCDescriptor falkolib ${Boost_LIBRARIES})
TARGET_LINK_LIBRARIES(testKeypointFalko falkolib ${Boost_LIBRARIES})
TARGET_LINK_LIBRARIES(testFalkoAHT falkolib ${Boost_LIBRARIES})
TARGET_LINK_LIBRARIES(testFalkoCC falkolib ${Boost_LIBRARIES})
TARGET_LINK_LIBRARIES(testKeypointOC falkolib ${Boost_LIBRARIES}) # boost_iostreams boost_system boost_filesystem