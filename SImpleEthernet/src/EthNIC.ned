package simpleethernet;

simple EthNIC {
    parameters:
    	@display("i=block/mac");
    	bool promMode = default(false);
    	double datarate @unit(bps) = default(100Mbps);
    	bool inSwitch = default(false);
    	int limCoda = default(10); //limite
    	bool flagV = default(false);
    	
    	int frame_trasmesse_nodo = default(0);
    	int frame_scartate = default(0);
    	int Frame_ricevute = default(0);
    	int frame_trasmesse_switch = default(0);
    	@signal[FLR];
    	@statistic[FLR](record=vector, histogram);
    	
    	@signal[Frame_trasmesse];
    	@statistic[Frame_trasmesse](record=vector, histogram);
    	
    	@signal[Frame_scartate];
    	@statistic[Frame_scartate](record=vector, histogram);
    	
    gates:
        input upperLayerIn;
        output upperLayerOut;
        input channelIn;
        output channelOut;
}
