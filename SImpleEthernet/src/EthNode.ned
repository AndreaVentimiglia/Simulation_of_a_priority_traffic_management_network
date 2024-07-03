package simpleethernet;

module EthNode
{
    parameters:
        @display("i=device/device;bgb=300,292");
        int dimApp = default(1);

    gates:
        input channelIn;
        output channelOut;
    submodules:
        app[dimApp]: PeriodicBurstApp {
            @display("p=140,54");
        }
        nic: EthNIC {
            @display("p=140,238");
        }
        dispatcher: Dispatcher {
            @display("p=140,153");
        }
    connections:
        nic.channelOut --> channelOut;
        nic.channelIn <-- channelIn;
        dispatcher.NicOut --> nic.upperLayerIn;
        nic.upperLayerOut --> dispatcher.NicIn;

        for i=0..(dimApp-1) {
            dispatcher.AppOut++ --> app[i].lowerLayerIn;
            app[i].lowerLayerOut --> dispatcher.AppIn++;
	     }
		//dispatcher.AppOut++ --> app.lowerLayerIn;
        //app.lowerLayerOut --> dispatcher.AppIn++;
}
