
enable_testing()

add_executable (UnitTests TestMain.cpp)
target_link_libraries (UnitTests MD_SOCIALLEARNING ${EXTERNAL_LIBRARIES} ${GTEST_LIBRARIES} ${Boost_LIBRARIES} pthread)

#target_compile_options(UnitTests PUBLIC -g)

add_test (AllTestsInUnitTests UnitTests)

