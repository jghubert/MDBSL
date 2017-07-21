
set (MDB_SocialLearning_LIB_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/Policy.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/GeneticAlgorithm.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ValueFunction.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ValueFunctionMemory.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/TraceMemory.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Settings.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Manager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ResourceLibrary.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Memory.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/BabblingStandard.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/SimpleGeneticAlgorithm.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/RandomGenerators.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/BabblingPolicy.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/RobotID.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/SocialManager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/SocialManagerClient.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/FamiliarityTester.cpp
)

if (USE_ECF)
    set (MDB_SocialLearning_LIB_SOURCES ${MDB_SocialLearning_LIB_SOURCES} ${CMAKE_CURRENT_SOURCE_DIR}/ECF.cpp)
endif (USE_ECF)

if (USE_FANN)
    set (MDB_SocialLearning_LIB_SOURCES ${MDB_SocialLearning_LIB_SOURCES} ${CMAKE_CURRENT_SOURCE_DIR}/FeedforwardNN.cpp)
endif (USE_FANN)

if (USE_GAUSSIAN_PROCESS)
    set (MDB_SocialLearning_LIB_SOURCES ${MDB_SocialLearning_LIB_SOURCES} ${CMAKE_CURRENT_SOURCE_DIR}/GaussianProcess.cpp)
endif(USE_GAUSSIAN_PROCESS)

if (USE_FASTSIM)
    set (MDB_SocialLearning_LIB_SOURCES ${MDB_SocialLearning_LIB_SOURCES} ${CMAKE_CURRENT_SOURCE_DIR}/FastSimSimulator.cpp)
endif (USE_FASTSIM)

if (USE_REV)
    set (MDB_SocialLearning_LIB_SOURCES ${MDB_SocialLearning_LIB_SOURCES} ${CMAKE_CURRENT_SOURCE_DIR}/../RobotExperimentViewer/RobotExperimentViewer/rev.cpp ${CMAKE_CURRENT_SOURCE_DIR}/../RobotExperimentViewer/RobotExperimentViewer/revinit.cpp ${CMAKE_CURRENT_SOURCE_DIR}/../RobotExperimentViewer/RobotExperimentViewer/revinputviewer.cpp)
endif (USE_REV)


#Generate FitnessLibrary.hpp
# add_definitions (-ggdb -Wall -std=c++11 -L/usr/local/bin -lstdc++)
add_executable (generateFitnessClass generateFitnessClass.cpp)

separate_arguments (classList UNIX_COMMAND ${fitnesses})
# foreach (class ${classList})
#     set (ARG_FIT "${ARG_FIT} ${class}")
# endforeach()
# message("ARG_FIT = ${ARG_FIT}" )

add_custom_command (
    OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/FitnessLibrary.hpp
    COMMAND generateFitnessClass "${CMAKE_CURRENT_BINARY_DIR}/FitnessLibrary.hpp" ${fitnesses}
    DEPENDS generateFitnessClass
)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

message("Compiling experiments: ${fitnesses}")
# message("classList = ${classList}")

foreach (class ${classList})
    string(CONCAT filename ${class} ".cpp")
    set (MDB_SocialLearning_LIB_SOURCES ${MDB_SocialLearning_LIB_SOURCES} ${PROJECT_SOURCE_DIR}/Experiments/${filename})
endforeach (class ${classList})

add_library(MD_SOCIALLEARNING ${MDB_SocialLearning_LIB_SOURCES} ${CMAKE_CURRENT_BINARY_DIR}/FitnessLibrary.hpp)

add_executable(dream_single main.cpp)
target_link_libraries(dream_single MD_SOCIALLEARNING ${EXTERNAL_LIBRARIES} ${Boost_LIBRARIES} )

add_executable(dream_socially main_social.cpp)
target_link_libraries(dream_socially MD_SOCIALLEARNING ${EXTERNAL_LIBRARIES} ${Boost_LIBRARIES} )
