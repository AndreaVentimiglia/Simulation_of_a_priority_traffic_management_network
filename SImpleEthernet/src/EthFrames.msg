packet EthFrame {
    name = "EthFrame";
    string dst;
    string src;
    int ethertype;
    byteLength = 26;
    simtime_t deadlineA; //priorità
    
    //int flagSwitch;
    
    //simtime_t priority; //priorità
};