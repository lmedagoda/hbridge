#ifndef AVALON_TYPES_H
#define AVALON_TYPES_H
enum ChangeReason {
    CR_INITIAL,
    CR_MB_TIMEOUT,
    CR_HB_ERROR,
    CR_LEGAL,
    CR_EMERGENCY
};

typedef struct {
    ARC_SYSTEM_STATE current_state;
    ARC_SYSTEM_STATE wanted_state;
    int32_t current_depth;
    uint8_t water_ingress_front:1;
    uint8_t water_ingress_back:1;
    enum ChangeReason change_reason:3; 
} __attribute__ ((packed)) __attribute__((__may_alias__)) avalon_status_t;

struct arc_avalon_control_packet{
    uint8_t dive;
    uint8_t strave;
    uint8_t left;
    uint8_t right;	
    uint8_t pitch;	
    uint8_t yaw;	
} __attribute__ ((packed)) __attribute__((__may_alias__));


#endif
