cmake_minimum_required(VERSION 3.0.0)

project(FastGlobalRegistration)

# Set OS-specific things here
if(WIN32)
	add_definitions(-DWINDOWS)
	message(STATUS "Compiling on Windows")
	if(MSVC)
		message(STATUS "Compiling with MSVC")
		add_definitions(-DNOMINMAX)
		add_definitions(-D_USE_MATH_DEFINES)
		add_definitions(-D_CRT_SECURE_NO_DEPRECATE -D_CRT_NONSTDC_NO_DEPRECATE -D_SCL_SECURE_NO_WARNINGS)		# suppress C4996 warning
		add_definitions(/wd4267 /wd4244 /wd4305 /wd4522)
	endif(MSVC)
elseif(CYGWIN)
	message(STATUS "Compiling on Cygwin")
	# enable c++11
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
	add_definitions(-DCYGWIN)
elseif(APPLE)
	add_definitions(-DUNIX)
	# enable c++11
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
	# compile speed optimization for clang
	add_definitions(-Os)
	# disable OpenMP since it is not currently supported on OSX
	message(STATUS "Compiling on OSX")
elseif(UNIX)
	add_definitions(-DUNIX)
	add_compile_options(-Wno-deprecated-declarations)
	add_compile_options(-Wno-unused-result)
	# enable c++11
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif(WIN32)



find_package(Boost)  
if (${Boost_FOUND})
  message(STATUS "Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
  message(STATUS "Boost_LIBRARY_DIRS: ${Boost_LIBRARY_DIRS}")
  message(STATUS "Boost_LIBRARIES: ${Boost_LIBRARY_DIRS}")
  include_directories(${Boost_INCLUDE_DIRS})
  link_directories(${Boost_LIBRARY_DIRS})
endif(${Boost_FOUND})

SET(EIGEN3_INCLUDE_DIR D:/3rdParty/Eigen3.3.4)
add_subdirectory(falkolib)

# set include directories
include_directories(
	${FastGlobalRegistration_SOURCE_DIR}
	${FastGlobalRegistration_SOURCE_DIR}/External
	${FastGlobalRegistration_SOURCE_DIR}/External/Eigen
	${FastGlobalRegistration_SOURCE_DIR}/External/flann
	${FastGlobalRegistration_SOURCE_DIR}/FastGlobalRegistration
	${FastGlobalRegistration_SOURCE_DIR}/
	)


add_subdirectory(FastGlobalRegistration)


