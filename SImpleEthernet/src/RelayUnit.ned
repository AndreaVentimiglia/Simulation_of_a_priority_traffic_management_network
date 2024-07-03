package simpleethernet;

simple RelayUnit {
    parameters:
    	@display("i=block/switch");
    gates:
        input lowerLayerIn[];
        output lowerLayerOut[];
}
