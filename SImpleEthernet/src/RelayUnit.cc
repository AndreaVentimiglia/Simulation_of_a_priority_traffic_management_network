#include "RelayUnit.h"
#include "EthFrames_m.h"

Define_Module(RelayUnit);

void RelayUnit::initialize()
{
    port_count = gateSize("lowerLayerIn");
}

void RelayUnit::handleMessage(cMessage *msg)
{
    int idx;
    int port_idx;

    idx = msg->getArrivalGate()->getIndex();
    EV << "Gate di arrivo della frame: [" << idx << "] "  << endl;

    EthFrame *frame = check_and_cast<EthFrame *>(msg);

    port_idx = getPortFromAddr(frame->getDst());

    EV << "Sorgente della frame: [" << frame->getSrc() << "] "  << endl;



    if(checkTable(frame->getSrc()) == -1){
       TableEntry *entry = new TableEntry;
       strncpy(entry->mac_addr,frame->getSrc(), 30);
       entry->eth_port_idx = idx;
       EV << "Nuova entry in table -> " << entry->mac_addr << ": " << entry->eth_port_idx << endl;
       fw_table.push_back(entry);
       }

    if(port_idx != -1) {
        send(frame->dup(), "lowerLayerOut", port_idx);
    } else {
        EV<<"ci entri?"<<endl;
        for(int i=0; i<port_count; i++) {
            if(i != idx) {
            send(frame->dup(), "lowerLayerOut", i);
            }
        }

    }
    delete frame;
}

int RelayUnit::getPortFromAddr(const char *mac) {
    for(int i = 0; i<int(fw_table.size()); i++) {
        if(strncmp(fw_table[i]->mac_addr, mac, strlen(fw_table[i]->mac_addr)) == 0) {
            EV << "Corrispondenza trovata Dst:"<< fw_table[i]->mac_addr <<" in Gate:"<< fw_table[i]->eth_port_idx <<" forwarding.."<< endl;
            return fw_table[i]->eth_port_idx;
        }

    }
    return -1;
}

int RelayUnit::checkTable(const char *sourceMac) {
    for(int i = 0; i<int(fw_table.size()); i++) {
        if(strncmp(fw_table[i]->mac_addr, sourceMac, strlen(fw_table[i]->mac_addr)) == 0) {
            return 0;
        }
    }
    return -1;
}

