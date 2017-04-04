/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   main_social.cpp
 * Author: julienhubert
 *
 * Created on February 15, 2017, 6:11 PM
 */

#include <cstdlib>
#include "SocialManager.h"
#include "Settings.h"

using namespace std;
using namespace MDB_Social;

/*
 * 
 */
int main(int argc, char** argv) 
{

    Settings* settings = SettingsLibrary::getInstance();

    char* filename = NULL;
    if (argc > 1 && argv[1][0] != '-')
        filename = argv[1];

    SocialManager sm(filename, argc, argv);
//    sm.registerParameters();
//    sm.loadParameters();

    if (!settings->printHelpIfRequested()) {
        sm.initializeRobots();
        sm.runExperiment();
    }
    return 0;
}

