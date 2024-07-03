#include "Dispatcher.h"
#include "AppPackets_m.h"


Define_Module(Dispatcher);

void Dispatcher::initialize() {
    EV<<"dim App1: "<<dimApp<<endl;
    dimApp = gateSize("AppIn");

}

void Dispatcher::handleMessage(cMessage *msg) {

    if(msg->getArrivalGate() == gate("AppIn", getIndex())) { //quando il msg arriva dall'application
        sendToNic(msg);
    }

    if(msg->getArrivalGate() == gate("NicIn")) { //quando il msg arriva dal nic }
        EV<<"ci entri?"<<endl;
        sendToApp(msg);
    }
}

void Dispatcher::sendToNic(cMessage* msg) {
    send(msg, "NicOut");
}

void Dispatcher::sendToApp(cMessage *msg) {
   cMessage *dupli;
   int i=0;
   while(i<dimApp) {
      dupli = check_and_cast<cMessage *>(msg);
      send(dupli, "AppOut", i);

      i++;
   }

}
