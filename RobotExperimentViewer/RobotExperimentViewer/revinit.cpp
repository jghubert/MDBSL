#include "revinit.h"
#include "rev.h"
#include <QApplication>
#include <iostream>
#include <sys/time.h>
#include <assert.h>
#include <unistd.h>

REVInit::REVInit()
{
    rev = NULL;    
    app = NULL;
    
    char appname[] = "teem.priv";
    argv = new char*[1];
    argv[0] = new char[10];
    memcpy(argv[0], appname, 10);
    argc = 1;
    
    app = new QApplication(argc, argv);
    rev = new REV();
//    rev->setQApplication(app);
    
    rev->show();
    app->processEvents();
    
    stopTimer = false;
}

REVInit::~REVInit()
{
    if (rev)
        delete rev;
    if (app)
        delete app;

    delete[] argv[0];
    delete[] argv;
}

void REVInit::startEventQueue()
{
    stopTimer = false;
    
    data.app = app;
    data.stopTimer = &(stopTimer);
    
    
    int rc = pthread_create(&timerThread, NULL, timerProcess, &data);
    if (rc) {
        std::cout << "RevInit: Failed to start the thread." << std::endl;
    }
}


void REVInit::stopEventQueue()
{
    stopTimer = true;
}

void* REVInit::timerProcess(void* args)
{
    unsigned freq = 100;
    struct revdata* data = (struct revdata*)args;
    
    struct timeval start, end;
    
    while (!*(data->stopTimer)) {
        gettimeofday(&start, NULL);
        data->app->processEvents();
        gettimeofday(&end, NULL);
        unsigned long elapsed = end.tv_usec - start.tv_usec;
        unsigned long tosleep = freq*1000 - elapsed;
        usleep(tosleep);
    }
    pthread_exit(NULL);
}

void REVInit::updateEventQueue()
{
    assert(app);
    app->processEvents();
}


REV* REVInit::getViewer()
{
    return rev;
}


