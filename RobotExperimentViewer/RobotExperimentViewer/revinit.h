#ifndef REVINIT_H
#define REVINIT_H

#include <pthread.h>

class REV;
class QApplication;

struct revdata {
    REV* rev;
    QApplication* app;
    int argc;
    char** argv;
    bool* stopTimer;
};

class REVInit
{
public:
    REVInit();
    ~REVInit();
    
    REV* getViewer();
    void startEventQueue();
    void stopEventQueue();
    
    void updateEventQueue();
    
private:
    REV* rev;
    QApplication* app;
    struct revdata data;
    char** argv;
    int argc;
    
    pthread_t timerThread;
    bool stopTimer;
    
    static void* timerProcess(void* args);
};

#endif // REVINIT_H
