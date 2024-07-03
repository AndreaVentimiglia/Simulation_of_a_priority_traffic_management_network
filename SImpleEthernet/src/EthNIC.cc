#include "EthNIC.h"
#include "EthFrames_m.h"
#include "AppPackets_m.h"

Define_Module(EthNIC);

EthFrame *tempQueue[100];
//int frame_scartate = 0;
//int frame_trasmesse_nodo = 0;
//int frame_trasmesse_switch = 0;
simtime_t lateness;
int deadlineMiss = 0;
//int Frame_ricevute = 0;
int frame_perse_video = 0;


void EthNIC::initialize() {
    datarate = par("datarate");
    promMode = par("promMode");
    inSwitch = par("inSwitch");
    //idSw = par("idSw");
    txstate = TX_STATE_IDLE;
    limCoda = par("limCoda");
    flagV = par("flagV");

    frame_trasmesse_nodo = par("frame_trasmesse_nodo");
    frame_scartate = par("frame_scartate");
    Frame_ricevute = par("Frame_ricevute");
    frame_trasmesse_switch = par("frame_trasmesse_switch");
}

void EthNIC::handleMessage(cMessage *msg) {
    if(msg->isSelfMessage()) {
        if(strcmp(msg->getName(), "TxEndTimer") == 0) {
            assert(txstate == TX_STATE_TRANSMITTING);
            delete msg;
            txstate = TX_STATE_WAIT_IFG;
            double t = 96.0 / datarate;
            cMessage *ifgtim = new cMessage("IFGEndTimer");
            scheduleAt(simTime() + t, ifgtim);
            return;
        } else if(strcmp(msg->getName(), "IFGEndTimer") == 0) {
            assert(txstate == TX_STATE_WAIT_IFG);
            delete msg;
            txstate = TX_STATE_IDLE;
            transmit();
            return;
        } else if(strcmp(msg->getName(), "RxEndTimer") == 0) {
            cPacket *pkt = dynamic_cast<cPacket *>(msg->removeControlInfo());
            delete msg;
            send(pkt, "upperLayerOut");
            return;
        }
    }

    if(msg->getArrivalGate() == gate("upperLayerIn")) {

        if(inSwitch) {
            cPacket *pkt = check_and_cast<cPacket *>(msg);
            EthFrame *frame = check_and_cast<EthFrame *>(msg);

            simsignal_t sig;

            if(txqueue.length() <= limCoda) {
                txqueue.insert(pkt);
                reorderQueue();
                lateness = simTime() - frame->getDeadlineA();
                if(lateness > 0) {
                    deadlineMiss++;
                }
                EV<<"Transmit 2"<<endl;
                frame_trasmesse_switch++;
                EV<<"frame trasmesse_switch "<<frame_trasmesse_switch<<endl;
                transmit();
                return;
            }
            else {
                EV<<"coda piena "<<txqueue.length()<<endl;
                if(flagV) {
                    frame_perse_video++;
                    EV<<"Frame VIDEO scartate "<<frame_perse_video<<endl;
                }
                frame_scartate++;

                EV<<"Frame scartate"<<frame_scartate<<endl;

                delete pkt;
                return;
            }
        }

        AppControlInfo *ci = check_and_cast<AppControlInfo *>(msg->removeControlInfo());
        EthFrame *frame = new EthFrame();
        frame->setSrc(ci->getSrc());
        frame->setDst(ci->getDst());
        frame->setDeadlineA(ci->getPriority() + simTime()-1); //assegnamo la priorità alla frame

        delete ci;
        cPacket *app_pkt = check_and_cast<cPacket *>(msg);
        int padding = 0;


        if(app_pkt->getByteLength() < 46) {
            padding = 46 - app_pkt->getByteLength();
        } else if(app_pkt->getByteLength() > 1500) {
            delete app_pkt;
            delete frame;
            EV << "AppPacket oversize: dropping..." << endl;
            frame_scartate++;
            if(flagV) {
               frame_perse_video++;
               EV<<"Frame VIDEO scartate "<<frame_perse_video<<endl;
           }
            return;
        }
        frame->addByteLength(padding);

        frame->encapsulate(app_pkt);


        AppPacket *pkt = check_and_cast<AppPacket *>(msg);

        if(txqueue.length() <= limCoda) {
            txqueue.insert(frame);
            reorderQueue();
            EV<<"Transmit 1"<<endl;
            frame_trasmesse_nodo++;
            EV<<"frame trasmesse_nodo "<<frame_trasmesse_nodo<<endl;
            transmit();

            lateness = simTime() - frame->getDeadlineA();
            if(lateness > 0) {
                deadlineMiss++;
            }
            return;

        }
        else {
            EV<<"coda piena "<<txqueue.length()<<endl;
            frame_scartate++;
            if(flagV) {
               frame_perse_video++;
               EV<<"Frame VIDEO scartate "<<frame_perse_video<<endl;
           }
            EV<<"Frame scartate "<<frame_scartate<<endl;
            delete frame;
            return;
        }
    }

    EthFrame *frame = check_and_cast<EthFrame *>(msg);
    double t = frame->getBitLength() / datarate;
    cMessage *rxtim = new cMessage("RxEndTimer");

    if(inSwitch) {
        rxtim->setControlInfo(frame);
        scheduleAt(simTime()+t, rxtim);
        Frame_ricevute++;
        return;
    }

    cPacket *app_pkt = frame->decapsulate();
    AppControlInfo *ci = new AppControlInfo();
    ci->setSrc(frame->getSrc());
    ci->setDst(frame->getDst());
    app_pkt->setControlInfo(ci);
    delete frame;
    rxtim->setControlInfo(app_pkt);
    Frame_ricevute++;
    scheduleAt(simTime()+t, rxtim);
}



void EthNIC::reorderQueue() {
    int txLength = 0;
    int dimCoda = txqueue.length();

    if(dimCoda > 1) {
        for(int i=0; i<dimCoda; i++) {
            cPacket *frame = txqueue.pop();
            tempQueue[i] = check_and_cast<EthFrame *>(frame);
            txLength++;
        }

        EthFrame *t = new EthFrame;

        for(int i=0; i<(txLength-1); i++) {
            int min = i;
            for(int j=i+1; j<txLength; j++) {

                if(tempQueue[j]->getDeadlineA() < tempQueue[min]->getDeadlineA()) {
                    min = j;
                }
            }
            t = tempQueue[i];
            tempQueue[i] = tempQueue[min];
            tempQueue[min] = t;
}

        txqueue.insert(tempQueue[txLength-1]);
        for(int i=0; i<txLength-1; i++) {
            txqueue.insertAfter(txqueue.back(), tempQueue[i]);
        }
    }
}

void EthNIC::transmit() {
    if(txstate == TX_STATE_IDLE) {
        if(!txqueue.empty()) {//flagVideo
            cPacket *frame = txqueue.pop();
            send(frame, "channelOut");
            txstate = TX_STATE_TRANSMITTING;
            double t = frame->getBitLength() / datarate;
            cMessage *txtimer = new cMessage("TxEndTimer");
            scheduleAt(t + simTime(), txtimer);
        }

        simsignal_t sig;
        sig = registerSignal("FLR");
        emit(sig, frame_scartate/(frame_trasmesse_nodo + frame_trasmesse_switch));

        sig = registerSignal("Frame_trasmesse");
        emit(sig, (frame_trasmesse_nodo + frame_trasmesse_switch));

        sig = registerSignal("Frame_scartate");
        emit(sig, frame_scartate);

        sig = registerSignal("DMR");
        emit(sig, deadlineMiss/(frame_trasmesse_nodo + frame_trasmesse_switch));
    }
}
