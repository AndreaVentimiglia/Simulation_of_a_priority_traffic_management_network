#ifndef __SIMPLEETHERNET_PERIODICBURSTAPP_H_
#define __SIMPLEETHERNET_PERIODICBURSTAPP_H_

#include <omnetpp.h>

using namespace omnetpp;

class PeriodicBurstApp : public cSimpleModule {
  protected:
    virtual void initialize();
    virtual void handleMessage(cMessage *msg);

    virtual void startTransmission();
    virtual void e2eDelayMax(simtime_t e);
    virtual void e2eDelayAvg(simtime_t e);


    simtime_t period;
    simtime_t startTime;
    int payloadSize;
    const char *source;
    const char *destination;
    int burstSize;
    int priority;
    int flagVideo;
    simtime_t deadlineR;
    int priorità;
};

#endif
