package simpleethernet.simulations;

import simpleethernet.EthNIC;
import simpleethernet.EthNode;
import simpleethernet.EthSwitch;
import simpleethernet.PeriodicBurstApp;


network AppTestNet
{
    @display("bgb=1278.56,584.38");
    submodules:
        SW1: EthSwitch {
            @display("p=270.84,283.04");
        }
        SW2: EthSwitch {
            @display("p=1019.92004,275.72");
        }
        LD2: EthNode {
            @display("p=39.04,65.880005");
        }
        US2: EthNode {
            @display("p=45.14,164.70001");
        }
        CM1: EthNode {
            @display("p=45.14,268.4");
        }
        LD1: EthNode {
            @display("p=45.14,366");
        }
        US1: EthNode {
            @display("p=45.14,447.74002");
        }
        S2: EthNode {
            @display("p=269.62,115.9");
        }
        CU: EthNode {
            @display("p=219.6,187.88");
        }
        S1: EthNode {
            @display("p=268.4,447.74002");
        }
        HU: EthNode {
            @display("p=398.94,335.5");
        }
        RC: EthNode {
            @display("p=1229.76,269.62");
        }
        ME: EthNode {
            @display("p=1220,164.70001");
        }
        US3: EthNode {
            @display("p=1229.76,43.920002");
        }
        US4: EthNode {
            @display("p=1229.76,517.28");
        }
        TLM: EthNode {
            @display("p=1220,381.86002");
        }
        S4: EthNode {
            @display("p=1039.4401,134.2");
        }
        S3: EthNode {
            @display("p=1039.4401,381.86002");
        }
        RS1: EthNode {
            @display("p=847.9,218.38");
        }
        RS2: EthNode {
            @display("p=847.9,381.86002");
        }
    connections:
        SW1.channelOut++ --> HU.channelIn;
        HU.channelOut --> SW1.channelIn++;
        SW1.channelOut++ --> S1.channelIn;
        S1.channelOut --> SW1.channelIn++;
        SW1.channelOut++ --> CU.channelIn;
        CU.channelOut --> SW1.channelIn++;
        SW1.channelOut++ --> S2.channelIn;
        S2.channelOut --> SW1.channelIn++;
        SW1.channelOut++ --> US1.channelIn;
        US1.channelOut --> SW1.channelIn++;
        SW1.channelOut++ --> LD1.channelIn;
        LD1.channelOut --> SW1.channelIn++;
        SW1.channelOut++ --> CM1.channelIn;
        CM1.channelOut --> SW1.channelIn++;
        SW1.channelOut++ --> US2.channelIn;
        US2.channelOut --> SW1.channelIn++;
        SW1.channelOut++ --> LD2.channelIn;
        LD2.channelOut --> SW1.channelIn++;
        SW1.channelOut++ --> SW2.channelIn++;
        SW2.channelOut++ --> SW1.channelIn++;
        SW2.channelOut++ --> RS2.channelIn;
        RS2.channelOut --> SW2.channelIn++;
        SW2.channelOut++ --> S3.channelIn;
        S3.channelOut --> SW2.channelIn++;
        SW2.channelOut++ --> RS1.channelIn;
        RS1.channelOut --> SW2.channelIn++;
        SW2.channelOut++ --> S4.channelIn;
        S4.channelOut --> SW2.channelIn++;
        SW2.channelOut++ --> US3.channelIn;
        US3.channelOut --> SW2.channelIn++;
        SW2.channelOut++ --> ME.channelIn;
        ME.channelOut --> SW2.channelIn++;
        SW2.channelOut++ --> RC.channelIn;
        RC.channelOut --> SW2.channelIn++;
        SW2.channelOut++ --> TLM.channelIn;
        TLM.channelOut --> SW2.channelIn++;
        SW2.channelOut++ --> US4.channelIn;
        US4.channelOut --> SW2.channelIn++;
}
