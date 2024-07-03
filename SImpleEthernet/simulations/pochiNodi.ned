
package simpleethernet.simulations;

import simpleethernet.EthNode;

import simpleethernet.EthSwitch;



network pochiNodi
{
    @display("bgb=973,469");
    submodules:
        Sw: EthSwitch {
            @display("p=385.915,148.9925");
        }
        Tx: EthNode {
            @display("p=207.6125,216.16125");
        }
        Rx: EthNode {
            @display("p=614.28876,179.52374");
        }
    connections:
        Tx.channelOut --> Sw.channelIn++;
        Sw.channelOut++ --> Tx.channelIn;
        Sw.channelOut++ --> Rx.channelIn;
        Rx.channelOut --> Sw.channelIn++;
}
