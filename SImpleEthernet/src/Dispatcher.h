#ifndef __SIMPLEETHERNET_DISPATCHER_H_
#define __SIMPLEETHERNET_DISPATCHER_H_

#include <omnetpp.h>

using namespace omnetpp;

class Dispatcher : public cSimpleModule
{

  protected:
    int dimApp;
    virtual void initialize();
    virtual void handleMessage(cMessage *msg);
    virtual void sendToNic(cMessage *msg);
    virtual void sendToApp(cMessage *msg);
};

#endif
