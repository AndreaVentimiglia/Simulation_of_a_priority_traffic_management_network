
#include "PeriodicBurstApp.h"
#include "AppPackets_m.h"
#include "EthNIC.h"

Define_Module(PeriodicBurstApp);
int conta = 0;
int contaBurst = 0; //solo per i video
int burst = 10; //METTERE A 10
simtime_t e2eMax, e2eAvg;
simtime_t e2eSum, e2eCount;


void PeriodicBurstApp::initialize() {

    period = par("period");
    startTime = par("startTime");
    payloadSize = par("payloadSize");
    burstSize = par("burstSize");
    source = par("source").stringValue();
    destination = par("destination").stringValue();
    priorità = par("priorità");

    flagVideo = par("flagVideo");
    deadlineR = par("deadlineR");

    if(startTime > 0) {
        cMessage *txtimer = new cMessage("TxTimer");
        scheduleAt(startTime, txtimer);
    }

}

void PeriodicBurstApp::handleMessage(cMessage *msg) {
    if(msg->isSelfMessage() and !flagVideo) {
        EV<<"non sono video"<<endl;
        if(strcmp(msg->getName(), "TxTimer") == 0) {
            startTransmission();
            scheduleAt(simTime() + period, msg);
            return;
        }
    }
    else if(msg->isSelfMessage() and flagVideo) {
        EV<<"sono video"<<endl;
        if(strcmp(msg->getName(), "TxTimer") == 0) {
            burstSize = burst;
            startTransmission();
            scheduleAt(simTime() + 10*period, msg);
            return;
        }
    }

    AppControlInfo *ci = check_and_cast<AppControlInfo *>(msg->removeControlInfo());



    if(strcmp(ci->getDst(), source) != 0) { //se le stringhe sono uguali non lo deve scartare
        EV<<"hello" <<ci->getDst()<<" "<<source<<endl;
        delete msg;
        delete ci;
        return;
    }


    //EV << "E' arrivato un messaggio: " << msg->getName() << endl;
    AppPacket *pkt = check_and_cast<AppPacket *>(msg);
    EV<<"Rx: mi è arrivato il msg da "<<ci->getSrc()<<endl;
    EV<<"Id messaggio: "<<pkt->getIdx()<<endl;
    EV<<"Id burst: "<<pkt->getIdBurst()<<endl;
    EV<<"conta = "<<conta<<endl;

    simsignal_t sig;

    if(pkt->getLastBurstPacket()) {
        sig = registerSignal("e2eDelay");
        emit(sig, simTime() - pkt->getGenTime());

        simtime_t e2eDelayTime = simTime() - pkt->getGenTime();

        e2eDelayMax(e2eDelayTime);
        e2eDelayAvg(e2eDelayTime);
    }

    delete msg;
    delete ci;
}

void PeriodicBurstApp::e2eDelayAvg(simtime_t e2eA) {
    e2eCount += 1;
    e2eSum = e2eSum + e2eA;
    e2eAvg = e2eSum/e2eCount;



    simsignal_t sig;
    sig = registerSignal("e2eDelayAvg");
    emit(sig, e2eAvg);
}

void PeriodicBurstApp::e2eDelayMax(simtime_t e2eM) {
    if (e2eM > e2eMax) {
        e2eMax = e2eM;
    }

    simsignal_t sig;
    sig = registerSignal("e2eDelayMax");
    emit(sig, e2eMax);
}



void PeriodicBurstApp::startTransmission() {
    AppPacket *pkt;
    AppControlInfo *ci;

    EV <<"Inizio burst "<<burstSize<<endl;

    if(conta > 119 and flagVideo) {
        conta = 0;
        contaBurst++;
    }
    EV<<"CONTA "<<conta<<endl;

    if(flagVideo) {
        for(int i=0; i < burstSize; i++) {
            pkt = new AppPacket();
            pkt->setByteLength(payloadSize);
            pkt->setGenTime(simTime());
            if(i == (burstSize - 1)) {
                pkt->setLastBurstPacket(true);
                EV<<"Fine burst"<<endl;
            }

            ci = new AppControlInfo();
            ci->setSrc(source);
            ci->setDst(destination);
            pkt->setIdx(i);
            pkt->setIdBurst(contaBurst);
            pkt->setVideo(true);

            ci->setPriority(deadlineR + pkt->getArrivalTime());

            pkt->setControlInfo(ci);
            conta++;
            EV<<"conta: "<<conta<<endl;
            send(pkt, "lowerLayerOut");
        }
    }
    else {
        for(int i=0; i < burstSize; i++) {
            pkt = new AppPacket();
            pkt->setByteLength(payloadSize);
            pkt->setGenTime(simTime());
            if(i == (burstSize - 1)) {
                pkt->setLastBurstPacket(true);
                EV<<"Fine burst"<<endl;
            }

            ci = new AppControlInfo();
            ci->setSrc(source);
            ci->setDst(destination);
            pkt->setIdx(i);
            pkt->setVideo(false);

            ci->setPriority(deadlineR);

            pkt->setControlInfo(ci);
            pkt->setIdBurst(0);
            send(pkt, "lowerLayerOut");
        }
    }
}
