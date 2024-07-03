package simpleethernet;

simple PeriodicBurstApp {
  parameters:
    @display("i=block/telnet");
    double period @unit(s) = default(0s);
    int payloadSize @unit(B) = default(0B);
    string destination = default("");
    string source = default("");
    int burstSize = default(2);
    double startTime @unit(s) = default(0s); //se startTime=0 solo ricevitore
    
    int flagVideo = default(0);
    double deadlineR @unit(s) = default(0s); //utilizzato per la priorità
    int priorità;
    
    @signal[e2eDelay];    
    @statistic[e2eDelay](record=vector, histogram);
    
    @signal[e2eDelayMax];    
    @statistic[e2eDelayMax](record=vector, histogram);
    
    @signal[e2eDelayAvg];    
    @statistic[e2eDelayAvg](record=vector, histogram);
    
    
  gates:
    input lowerLayerIn;
    output lowerLayerOut;
}
