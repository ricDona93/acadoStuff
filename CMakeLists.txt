cmake_minimum_required(VERSION 3.2)

# Project name and programming languages used
PROJECT( testACADO CXX )

# CMake module(s) path
SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )

SET(PYTHON_INCLUDE_DIRS ${PYTHON_INCLUDE_DIRS}
    /usr/local/lib/python3.7/site-packages/numpy/core/include
    /usr/local/Cellar/python/3.7.5/Frameworks/Python.framework/Versions/3.7/lib
    /usr/local/Cellar/python/3.7.5/Frameworks/Python.framework/Versions/3.7/include/python3.7m
    )
#
# Prerequisites
#
FIND_PACKAGE( ACADO REQUIRED )

FIND_LIBRARY(PYTHON_LIB
    NAMES libpython2.7.dylib
    PATHS /Users/Shared/Epic Games/UE_4.24/Engine/Binaries/ThirdParty/Python/Mac
)

MESSAGE( STATUS "Python shared library: ${PYTHON_LIB}\n" )

#
# Include directories
#
INCLUDE_DIRECTORIES( . ${ACADO_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES( . ${PYTHON_INCLUDE_DIRS} )

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

add_executable(singleTrackMinS tests/single_track_minS.cpp)
TARGET_LINK_LIBRARIES(singleTrackMinS ${ACADO_SHARED_LIBRARIES} )
SET_TARGET_PROPERTIES(singleTrackMinS
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin )

add_executable(singleTrackJerkMinT tests/single_track_jerk_minT.cpp)
TARGET_LINK_LIBRARIES(singleTrackJerkMinT ${ACADO_SHARED_LIBRARIES} )
SET_TARGET_PROPERTIES(singleTrackJerkMinT
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin )

add_executable(singleTrackJerkRelaxMinT tests/single_track_jerk_relax_minT.cpp)
TARGET_LINK_LIBRARIES(singleTrackJerkRelaxMinT ${ACADO_SHARED_LIBRARIES} )
SET_TARGET_PROPERTIES(singleTrackJerkRelaxMinT
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin )

add_executable(singleTrackActRelaxMinT tests/single_track_act_relax_minT.cpp)
TARGET_LINK_LIBRARIES(singleTrackActRelaxMinT ${ACADO_SHARED_LIBRARIES} )
SET_TARGET_PROPERTIES(singleTrackActRelaxMinT
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin )

add_executable(singleTrackPacMinT tests/single_track_pac_minT.cpp)
TARGET_LINK_LIBRARIES(singleTrackPacMinT ${ACADO_SHARED_LIBRARIES} )
SET_TARGET_PROPERTIES(singleTrackPacMinT
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin )

add_executable(singleTrackPacRelaxMinT tests/single_track_pac_relax_minT.cpp)
TARGET_LINK_LIBRARIES(singleTrackPacRelaxMinT ${ACADO_SHARED_LIBRARIES} )
SET_TARGET_PROPERTIES(singleTrackPacRelaxMinT
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin )

add_executable(testLib libs/test_lib.cpp libs/lib_models.cpp)
TARGET_LINK_LIBRARIES(testLib  ${ACADO_SHARED_LIBRARIES} ${PYTHON_LIB})
SET_TARGET_PROPERTIES(testLib
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin )

add_executable(testLib_vehs libs/test_lib_vehs.cpp libs/lib_models.cpp)
TARGET_LINK_LIBRARIES(testLib_vehs  ${ACADO_SHARED_LIBRARIES} ${PYTHON_LIB})
SET_TARGET_PROPERTIES(testLib_vehs
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin )

add_library(testLib_shared SHARED
    libs/lib_models.cpp)
set_target_properties(testLib_shared PROPERTIES PUBLIC_HEADER libs/lib_models.h)
TARGET_LINK_LIBRARIES(testLib_shared  ${ACADO_SHARED_LIBRARIES} ${PYTHON_LIB})
SET_TARGET_PROPERTIES(testLib_shared
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/libs )

add_executable(singleTrackPacXY tests/single_track_pacejka_XY.cpp)
TARGET_LINK_LIBRARIES(singleTrackPacXY ${ACADO_SHARED_LIBRARIES} )
SET_TARGET_PROPERTIES(singleTrackPacXY
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin )

add_executable(singleTrackVec tests/single_track_vector_ref.cpp)
TARGET_LINK_LIBRARIES(singleTrackVec ${ACADO_SHARED_LIBRARIES} )
SET_TARGET_PROPERTIES(singleTrackVec
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin )