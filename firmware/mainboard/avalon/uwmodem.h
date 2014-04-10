#ifndef UWMODEM_H
#define UWMODEM_H
typedef signed int (*uwmodem_recv_func_t)(unsigned char *data, const unsigned int dataSize);
typedef signed int (*uwmodem_seek_func_t)(unsigned char *data, const unsigned int dataSize);
typedef signed int (*uwmodem_send_func_t)(unsigned char *data, const unsigned int dataSize);
typedef signed int (*uwmodem_surface_func_t)();

int uwmodem_sendData(unsigned char *data, const unsigned int size);
void uwmodem_init(uwmodem_send_func_t sendFunc, uwmodem_recv_func_t recvFunc, uwmodem_seek_func_t seekFunc , uwmodem_surface_func_t surfaceFunc);
void uwmodem_process();
#endif
