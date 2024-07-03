package simpleethernet.simulations;

import simpleethernet.EthNode;
import simpleethernet.EthSwitch;


network Esteso
{
    @display("bgb=1247.4501,634.4");
    submodules:
        SW1: EthSwitch {
            @display("p=532,416");
        }
        SW2: EthSwitch {
            @display("p=532,129");
        }
        SW3: EthSwitch {
            @display("p=187,129");
        }
        SW5: EthSwitch {
            @display("p=975,407");
        }
        SW6: EthSwitch {
            @display("p=975,132");
        }
        CM2: EthNode {
            @display("p=128,32");
        }
        LD2: EthNode {
            @display("p=37,81");
        }
        RD2: EthNode {
            @display("p=37,154");
        }
        US2: EthNode {
            @display("p=112,220");
        }
        CU: EthNode {
            @display("p=238,203");
        }
        S2: EthNode {
            @display("p=263,67");
        }
        US1: EthNode {
            @display("p=260,456");
        }
        CM1: EthNode {
            @display("p=82,345");
        }
        LD1: EthNode {
            @display("p=59,407");
        }
        S1: EthNode {
            @display("p=285,320");
        }
        SW4: EthSwitch {
            @display("p=187,416");
        }
        RD1: EthNode {
            @display("p=134,473");
        }
        DC: EthNode {
            @display("p=151,295");
        }
        ZC1: EthNode {
            @display("p=292,386");
        }
        MC2: EthNode {
            @display("p=436,70");
        }
        TLM: EthNode {
            @display("p=532,48");
        }
        S3: EthNode {
            @display("p=627,70");
        }
        RS1: EthNode {
            @display("p=605,215");
        }
        HU: EthNode {
            @display("p=467,203");
        }
        MC1: EthNode {
            @display("p=447,375");
        }
        RS2: EthNode {
            @display("p=565,345");
        }
        S6: EthNode {
            @display("p=644,400");
        }
        ME: EthNode {
            @display("p=436,473");
        }
        CC: EthNode {
            @display("p=627,481");
        }
        ZC2: EthNode {
            @display("p=203,34");
        }
        S4: EthNode {
            @display("p=936,29");
        }
        ZC3: EthNode {
            @display("p=1023,31");
        }
        CM3: EthNode {
            @display("p=1101,96");
        }
        LD3: EthNode {
            @display("p=1078,188");
        }
        RD3: EthNode {
            @display("p=1031,245");
        }
        RC: EthNode {
            @display("p=862,97");
        }
        US3: EthNode {
            @display("p=856,191");
        }
        S5: EthNode {
            @display("p=910,334");
        }
        CM4: EthNode {
            @display("p=1060,351;b=5,14");
        }
        LD4: EthNode {
            @display("p=1106,416");
        }
        ZC4: EthNode {
            @display("p=843,382");
        }
        US4: EthNode {
            @display("p=845,459");
        }
        RD4: EthNode {
            @display("p=1024,458");
        }
    connections:
        SW3.channelOut++ --> CM2.channelIn;
        CM2.channelOut --> SW3.channelIn++;
        SW3.channelOut++ --> LD2.channelIn;
        LD2.channelOut --> SW3.channelIn++;
        SW3.channelOut++ --> RD2.channelIn;
        RD2.channelOut --> SW3.channelIn++;
        SW3.channelOut++ --> US2.channelIn;
        US2.channelOut --> SW3.channelIn++;
        SW3.channelOut++ --> S2.channelIn;
        S2.channelOut --> SW3.channelIn++;
        SW3.channelOut++ --> CU.channelIn;
        CU.channelOut --> SW3.channelIn++;
        US1.channelOut --> SW4.channelIn++;
        S1.channelOut --> SW4.channelIn++;
        RD1.channelOut --> SW4.channelIn++;
        LD1.channelOut --> SW4.channelIn++;
        SW4.channelOut++ --> CM1.channelIn;
        SW4.channelOut++ --> LD1.channelIn;
        SW4.channelOut++ --> RD1.channelIn;
        SW4.channelOut++ --> S1.channelIn;
        SW4.channelOut++ --> US1.channelIn;
        CM1.channelOut --> SW4.channelIn++;
        DC.channelOut --> SW4.channelIn++;
        SW4.channelOut++ --> DC.channelIn;
        SW4.channelOut++ --> SW3.channelIn++;
        SW3.channelOut++ --> SW4.channelIn++;
        SW2.channelOut++ --> TLM.channelIn;
        TLM.channelOut --> SW2.channelIn++;
        SW2.channelOut++ --> S3.channelIn;
        S3.channelOut --> SW2.channelIn++;
        SW2.channelOut++ --> RS1.channelIn;
        RS1.channelOut --> SW2.channelIn++;
        SW2.channelOut++ --> HU.channelIn;
        HU.channelOut --> SW2.channelIn++;
        SW2.channelOut++ --> MC2.channelIn;
        MC2.channelOut --> SW2.channelIn++;
        SW3.channelOut++ --> SW2.channelIn++;
        SW2.channelOut++ --> SW3.channelIn++;
        SW1.channelOut++ --> RS2.channelIn;
        RS2.channelOut --> SW1.channelIn++;
        SW1.channelOut++ --> S6.channelIn;
        S6.channelOut --> SW1.channelIn++;
        SW1.channelOut++ --> CC.channelIn;
        CC.channelOut --> SW1.channelIn++;
        SW1.channelOut++ --> ME.channelIn;
        ME.channelOut --> SW1.channelIn++;
        SW1.channelOut++ --> MC1.channelIn;
        MC1.channelOut --> SW1.channelIn++;
        SW2.channelOut++ --> SW1.channelIn++;
        SW1.channelOut++ --> SW2.channelIn++;
        SW4.channelOut++ --> SW1.channelIn++;
        SW1.channelOut++ --> SW4.channelIn++;
        SW3.channelOut++ --> ZC2.channelIn;
        ZC2.channelOut --> SW3.channelIn++;
        SW2.channelOut++ --> SW6.channelIn++;
        SW6.channelOut++ --> SW2.channelIn++;
        SW6.channelOut++ --> S4.channelIn;
        S4.channelOut --> SW6.channelIn++;
        SW6.channelOut++ --> RC.channelIn;
        RC.channelOut --> SW6.channelIn++;
        SW6.channelOut++ --> US3.channelIn;
        US3.channelOut --> SW6.channelIn++;
        SW6.channelOut++ --> RD3.channelIn;
        RD3.channelOut --> SW6.channelIn++;
        SW6.channelOut++ --> LD3.channelIn;
        LD3.channelOut --> SW6.channelIn++;
        SW6.channelOut++ --> CM3.channelIn;
        CM3.channelOut --> SW6.channelIn++;
        SW6.channelOut++ --> ZC3.channelIn;
        ZC3.channelOut --> SW6.channelIn++;
        SW5.channelOut++ --> CM4.channelIn;
        CM4.channelOut --> SW5.channelIn++;
        SW5.channelOut++ --> LD4.channelIn;
        LD4.channelOut --> SW5.channelIn++;
        SW5.channelOut++ --> RD4.channelIn;
        RD4.channelOut --> SW5.channelIn++;
        SW5.channelOut++ --> S5.channelIn;
        S5.channelOut --> SW5.channelIn++;
        SW5.channelOut++ --> ZC4.channelIn;
        ZC4.channelOut --> SW5.channelIn++;
        SW5.channelOut++ --> US4.channelIn;
        US4.channelOut --> SW5.channelIn++;
        SW6.channelOut++ --> SW5.channelIn++;
        SW5.channelOut++ --> SW6.channelIn++;
        SW1.channelOut++ --> SW5.channelIn++;
        SW5.channelOut++ --> SW1.channelIn++;
        ZC1.channelOut --> SW4.channelIn++;
        SW4.channelOut++ --> ZC1.channelIn;
}
