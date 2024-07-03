package simpleethernet;

simple Dispatcher
{
    @display("i=abstract/dispatcher");
    
    gates:
        input AppIn[]; //riceve dall'app
        output AppOut[]; //per mandare all'app

        input NicIn; //riceve dal nic
        output NicOut; //per mandare al nic
}
