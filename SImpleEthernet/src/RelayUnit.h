#ifndef __ETHERNETSIMPLE_RELAYUNIT_H_
#define __ETHERNETSIMPLE_RELAYUNIT_H_

#include <omnetpp.h>
#include <vector>

using namespace omnetpp;

class RelayUnit : public cSimpleModule
{
  protected:
    typedef struct {
        char mac_addr[30];
        int eth_port_idx;
    }TableEntry;

    virtual void initialize();
    virtual void handleMessage(cMessage *msg);

    virtual int getPortFromAddr(const char *mac);
    virtual int checkTable(const char *mac);

    std::vector<TableEntry *> fw_table;

    int port_count;
};

#endif

