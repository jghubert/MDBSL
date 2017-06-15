#!/bin/sh

g++ -Wall -std=c++11 -o reformatRewardPropagation reformatRewardPropagation.cpp ../MDB_SocialLearning/libMD_SOCIALLEARNING.a -D SEPARATOR='" "' -lboost_program_options


