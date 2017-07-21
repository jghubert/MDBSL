cmake_minimum_required (VERSION 2.8.3)
project (MDB_SocialLearning CXX)
set(CMAKE_CXX_STANDARD 11)

if (NOT DEFINED fitnesses)
    message(FATAL_ERROR "No fitnesses provided. Cannot proceed further. Use -Dfitnesses:string=\"...\" to provide at least one. ")
endif ()

# set ()

OPTION(USE_ECF "Compile with Evolutionary Computation Framework." OFF)
OPTION(USE_FANN "Compile with FANN." ON)
OPTION(USE_GAUSSIAN_PROCESS "Compile with support Gaussian process. Requires the libgp library." ON)
OPTION(USE_FASTSIM "Compile with FastSim." ON)
OPTION(USE_FASTSIM_VIEWER "Compile with the support for FastSim viewer." OFF)
OPTION(USE_REV "Compile with the Robot Experiment Viewer." OFF)
#OPTION(USE_ANN "Compile with ANN, library for computing nearest neighbour point in a set." ON)
OPTION(USE_MLPACK "Compile with MLPack. It provides the functions for computing knn in a set." ON)


# set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -ggdb -Wall -std=c++11)
add_definitions(-ggdb -Wall -DSEPARATOR=" ")
# -std=c++11

set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost 1.45.0 COMPONENTS filesystem program_options thread chrono date_time REQUIRED) 
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )



if (USE_ECF)
    find_library(LIBECF ecf)
    if (LIBECF)
        set(EXTERNAL_LIBRARIES ${EXTERNAL_LIBRARIES} ${LIBECF})
        add_definitions(-DUSE_ECF)
    else (LIBECF)
        message (FATAL_ERROR "Impossible to locate the ECF library. Make sure it can be found in the path.")
    endif(LIBECF)
endif (USE_ECF)

if (USE_FANN)
    find_library(FANN doublefann)
    if (FANN)
        set(EXTERNAL_LIBRARIES ${EXTERNAL_LIBRARIES} ${FANN})
        add_definitions(-DUSE_FANN)
    else (FANN)
        message (FATAL_ERROR "Impossible to locate the FANN library. Make sure it can be found in the path.")
    endif (FANN)
endif (USE_FANN)

if (USE_GAUSSIAN_PROCESS)
    find_library(GP gp)
    if (GP)
        set(EXTERNAL_LIBRARIES ${EXTERNAL_LIBRARIES} ${GP})
        add_definitions(-DUSE_GAUSSIAN_PROCESS)
    else (GP)
        message (FATAL_ERROR "Impossible to locate the libgp library. Make sure it can be found in the path.")
    endif (GP)
endif (USE_GAUSSIAN_PROCESS)

if (USE_FASTSIM)
    set(EXTERNAL_LIBRARIES ${EXTERNAL_LIBRARIES} ${PROJECT_SOURCE_DIR}/MDB_SocialLearning/fastsim/lib/libfastsim.a)
    add_definitions(-DUSE_FASTSIM)
    if (USE_FASTSIM_VIEWER)
        find_library(LIBSDL SDL)
        if (LIBSDL)
            message (STATUS "Compiling with FastSim Viewer.")
            set(EXTERNAL_LIBRARIES ${EXTERNAL_LIBRARIES} SDL)
            add_definitions(-DUSE_FASTSIM_VIEWER)
        else (LIBSDL)
            message (FATAL_ERROR "Impossible to find the SDL library necessary for FastSim viewer. Make sure it can be found in the path.")
        endif(LIBSDL)
    endif(USE_FASTSIM_VIEWER)
endif(USE_FASTSIM)

if (USE_REV)
    message (STATUS "Compiling with Robot Experiment Viewer.")
    set (CMAKE_AUTOMOC ON)
    find_package(Qt5Widgets REQUIRED)
    include_directories(${Qt5Widgets_INCLUDE_DIRS})
    add_definitions(${Qt5Widgets_DEFINITIONS})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
    set(EXTERNAL_LIBRARIES ${EXTERNAL_LIBRARIES} Qt5::Widgets)
    add_definitions(-DUSE_REV)
endif (USE_REV)

#if (USE_ANN)
#    set(EXTERNAL_LIBRARIES ${EXTERNAL_LIBRARIES} ${PROJECT_SOURCE_DIR}/MDB_SocialLearning/ann/lib/libANN.a)
#    add_definitions(-DUSE_ANN)
#endif(USE_ANN)

if (USE_MLPACK)
    find_library(LIBMLPACK mlpack)
    if (LIBMLPACK)
        set(EXTERNAL_LIBRARIES ${EXTERNAL_LIBRARIES} ${LIBMLPACK})
    else (LIBMLPACK)
        message (FATAL_ERROR "Impossible to locate the MLPACK library. Make sure it can be found in the path.")
    endif (LIBMLPACK)
endif (USE_MLPACK)

message("PROJECT_SOURCE_DIR = ${PROJECT_SOURCE_DIR}" )

include_directories ("${PROJECT_SOURCE_DIR}/MDB_SocialLearning")
add_subdirectory (MDB_SocialLearning)



