CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
set (CMAKE_CXX_STANDARD 11)
set (CMAKE_BUILD_TYPE DEBUG)
PROJECT(planets)
#message("CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -o -std=gnu++11")

# file(GLOB_RECURSE SRC_FILES
# 	${CMAKE_SOURCE_DIR}/src/*.cpp)
set(SRC_FILES
	${CMAKE_SOURCE_DIR}/src/main.cpp
	# ${CMAKE_SOURCE_DIR}/src/tests.cpp
	${CMAKE_SOURCE_DIR}/src/planet.cpp
	# ${CMAKE_SOURCE_DIR}/src/solarSystem.cpp
	${CMAKE_SOURCE_DIR}/src/astroObjectBase.cpp)
	
message(${SRC_FILES})

# file(GLOB_RECURSE HEADER_FILES 
# 	${CMAKE_SOURCE_DIR}/include/*.h
# 	${CMAKE_SOURCE_DIR}/include/*.hpp)
set(HEADER_FILES
	${CMAKE_SOURCE_DIR}/include/main.h
	# ${CMAKE_SOURCE_DIR}/include/tests.h
	${CMAKE_SOURCE_DIR}/include/planet.h
	# ${CMAKE_SOURCE_DIR}/include/solarSystem.h
	${CMAKE_SOURCE_DIR}/include/astroObjectBase.h)

message(${HEADER_FILES})
# link_directories(${CMAKE_SOURCE_DIR}/lib)

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})
link_directories(${OpenGL_LIBRARY_DIRS})

find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIR})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(${PROJECT_NAME} ${HEADER_FILES} ${SRC_FILES} )
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(${PROJECT_NAME} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES})