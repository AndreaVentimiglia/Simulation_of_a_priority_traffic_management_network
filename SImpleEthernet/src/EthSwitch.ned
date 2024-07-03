package simpleethernet;

module EthSwitch
{
    parameters:
        @display("i=device/hub;bgb=276,262");
    gates:
        input channelIn[];
        output channelOut[];
    submodules:
        relayUnit: RelayUnit {
            @display("p=138,47");
        }
        eth[sizeof(channelIn)]: EthNIC {
            @display("p=138,147");
            inSwitch = true;
        }
    connections:
        for i=0..sizeof(channelIn)-1 {
            eth[i].upperLayerIn <-- relayUnit.lowerLayerOut++;
            eth[i].upperLayerOut --> relayUnit.lowerLayerIn++;
            
            channelIn[i] --> eth[i].channelIn;
            channelOut[i] <-- eth[i].channelOut;
        }
}
