#ifndef __SIMPLEETHERNET_ETHNIC_H_
#define __SIMPLEETHERNET_ETHNIC_H_

#include <omnetpp.h>
#include "EthFrames_m.h"

using namespace omnetpp;

class EthNIC : public cSimpleModule {
  protected:
    typedef enum {
        TX_STATE_IDLE,
        TX_STATE_TRANSMITTING,
        TX_STATE_WAIT_IFG
    } ethstate_t;

    virtual void initialize();
    virtual void handleMessage(cMessage *msg);

    virtual void transmit();
    virtual void reorderQueue();


    ethstate_t txstate;
    bool promMode;
    bool inSwitch;
    double datarate;
    cPacketQueue txqueue;
    int limCoda;
    //int idSw;
    bool flagV;
    int frame_trasmesse_nodo;
    int frame_scartate;
    int Frame_ricevute;
    int frame_trasmesse_switch;
};

#endif
