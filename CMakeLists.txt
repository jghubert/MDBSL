cmake_minimum_required (VERSION 2.6)
project (MDB_SocialLearning CXX)

if (NOT DEFINED fitnesses)
    message(FATAL_ERROR "No fitnesses provided. Cannot proceed further. Use -Dfitnesses:string=\"...\" to provide at least one. ")
endif ()

# set ()

OPTION(USE_ECF "Compile with Evolutionary Computation Framework." OFF)
OPTION(USE_FANN "Compile with FANN." ON)
OPTION(USE_FASTSIM "Compile with FastSim." ON)
OPTION(USE_ANN "Compile with ANN, library for computing nearest neighbour point in a set." ON)


# set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -ggdb -Wall -std=c++11)
add_definitions(-ggdb -Wall -std=c++11 -DSEPARATOR=" ")

set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost 1.45.0 COMPONENTS filesystem program_options thread chrono date_time REQUIRED) 
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )



#find_library(BOOST_OPTIONS boost_program_options)
#if (BOOST_OPTIONS)
#    set(EXTERNAL_LIBRARIES ${EXTERNAL_LIBRARIES} ${BOOST_OPTIONS})
#elseif (BOOST_OPTIONS)
#    message (FATAL_ERROR "Boost program options was not found in the path. Please make sure it is installed and available.")
#endif(BOOST_OPTIONS)
#
#find_library(BOOST_FILESYSTEM boost_filesystem)
#if (BOOST_FILESYSTEM)
#    set(EXTERNAL_LIBRARIES ${EXTERNAL_LIBRARIES} ${BOOST_FILESYSTEM})
#elseif (BOOST_FILESYSTEM)
#    message (FATAL_ERROR "Boost filesystem was not found in the path. Please make sure it is installed and available.")
#endif(BOOST_FILESYSTEM)
#
#find_library(BOOST_THREAD boost_thread)
#if (BOOST_THREAD)
#    set(EXTERNAL_LIBRARIES ${EXTERNAL_LIBRARIES} ${BOOST_THREAD} boost_chrono boost_data_time)
#elseif (BOOST_THREAD)
#    message (FATAL_ERROR "Boost thread was not found in the path. Please make sure it is installed and available.")
#endif(BOOST_THREAD)


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

if (USE_FASTSIM)
    set(EXTERNAL_LIBRARIES ${EXTERNAL_LIBRARIES} ${PROJECT_SOURCE_DIR}/MDB_SocialLearning/fastsim/lib/libfastsim.a)
    add_definitions(-DUSE_FASTSIM)
endif(USE_FASTSIM)

if (USE_ANN)
    set(EXTERNAL_LIBRARIES ${EXTERNAL_LIBRARIES} ${PROJECT_SOURCE_DIR}/MDB_SocialLearning/ann/lib/libANN.a)
    add_definitions(-DUSE_ANN)
endif(USE_ANN)

message("PROJECT_SOURCE_DIR = ${PROJECT_SOURCE_DIR}" )

include_directories ("${PROJECT_SOURCE_DIR}/MDB_SocialLearning")
add_subdirectory (MDB_SocialLearning)



