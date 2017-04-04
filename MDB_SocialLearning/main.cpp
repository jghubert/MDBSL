/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   main.cpp
 * Author: Julien Hubert
 *
 * Created on June 13, 2016, 4:35 PM
 */

#include <iostream>
#include <cstdlib>
#include "Manager.h"
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
    
    Manager manager(filename, argc, argv);
    
    settings->saveSettingsToFileRequested();
    if (!settings->printHelpIfRequested())
        manager.runExperiment();
 
    
    
    return 0;
}

