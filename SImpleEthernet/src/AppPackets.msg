
packet AppPacket {
    name = "AppPacket";
    simtime_t genTime;
    bool lastBurstPacket = false;    
    int idx;
    int idBurst;
    bool video;
    
    //int priority;
};

message AppControlInfo {
    string dst;
    string src;    
    simtime_t priority;
};
