#ifndef ARC_NOTOKENDRIVER_H
#define ARC_NOTOKENDRIVER_H
int arcnotoken_init(arc_send_func_t sendFunc, arc_recv_func_t recvFunc, arc_seek_func_t seekFunc);
int arcnotoken_readPacket(arc_packet_t* packet);
int arcnotoken_add_serial_handler(arc_send_func_t sendFunc, arc_recv_func_t recvFunc, arc_seek_func_t seekFunc);
int arcnotoken_sendPacket(arc_packet_t* packet);
void arcnotoken_setOwnSystemID(ARC_SYSTEM_ID sys_id);
ARC_SYSTEM_ID arcnotoken_getOwnSystemID();

#endif
