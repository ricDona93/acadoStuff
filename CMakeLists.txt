cmake_minimum_required(VERSION 3.2)

# Project name and programming languages used
PROJECT( testACADO CXX )

# CMake module(s) path
SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )

#
# Prerequisites
#
FIND_PACKAGE( ACADO REQUIRED )

#
# Include directories
#
INCLUDE_DIRECTORIES( . ${ACADO_INCLUDE_DIRS} )

#
# Build an executable
#
#TARGET_LINK_LIBRARIES( <EXEC_NAME> ${ACADO_STATIC_LIBRARIES} )

set(CMAKE_CXX_STANDARD 11)

add_executable(test tests/main.cpp)
TARGET_LINK_LIBRARIES(test ${ACADO_SHARED_LIBRARIES} )
SET_TARGET_PROPERTIES(test
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin )

add_executable(codeGen tests/code_gen.cpp)
TARGET_LINK_LIBRARIES(codeGen ${ACADO_SHARED_LIBRARIES} )
SET_TARGET_PROPERTIES(codeGen
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/code_gen )

add_executable(singleTrack tests/single_track.cpp)
TARGET_LINK_LIBRARIES(singleTrack ${ACADO_SHARED_LIBRARIES} )
SET_TARGET_PROPERTIES(singleTrack
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin )

add_executable(singleTrackActuator tests/single_track_actuator.cpp)
TARGET_LINK_LIBRARIES(singleTrackActuator ${ACADO_SHARED_LIBRARIES} )
SET_TARGET_PROPERTIES(singleTrackActuator
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin )

add_executable(singleTrackXY tests/single_track_XY.cpp)
TARGET_LINK_LIBRARIES(singleTrackXY ${ACADO_SHARED_LIBRARIES} )
SET_TARGET_PROPERTIES(singleTrackXY
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin )

add_executable(kinematic_minT tests/kinematic_minT.cpp)
TARGET_LINK_LIBRARIES(kinematic_minT ${ACADO_SHARED_LIBRARIES} )
SET_TARGET_PROPERTIES(kinematic_minT
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin )

add_executable(singleTrackMinT tests/single_track_minT.cpp)
TARGET_LINK_LIBRARIES(singleTrackMinT ${ACADO_SHARED_LIBRARIES} )
SET_TARGET_PROPERTIES(singleTrackMinT
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin )

add_executable(singleTrackPacXY tests/single_track_pacejka_XY.cpp)
TARGET_LINK_LIBRARIES(singleTrackPacXY ${ACADO_SHARED_LIBRARIES} )
SET_TARGET_PROPERTIES(singleTrackPacXY
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin )

add_executable(singleTrackVec tests/single_track_vector_ref.cpp)
TARGET_LINK_LIBRARIES(singleTrackVec ${ACADO_SHARED_LIBRARIES} )
SET_TARGET_PROPERTIES(singleTrackVec
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin )