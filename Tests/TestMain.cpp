/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   TestMain.cpp
 * Author: Julien Hubert
 *
 * Created on August 15, 2017, 4:57 PM
 */

#include <cstdlib>
#include <gtest/gtest.h>
#include <iostream>
#include "CaffeTest.hpp"
#include "../MDB_SocialLearning/Settings.h"

using namespace std;
using namespace MDB_Social;

bool SettingsLoaded = false;

/*
 * 
 */
int main(int argc, char** argv) 
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}

