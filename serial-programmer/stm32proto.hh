
#ifndef __STM32PROTO_HH__
#define __STM32PROTO_HH__

#include <sys/types.h>

class Stm32Proto {
public:
	enum rx_avail_t {
		HAVE_RX=0,
		NO_RX=1,
		DEBUG_NO_RX=2
	};
private:
	int fd;
	unsigned int rate;
	int rate_name;
	rx_avail_t rx_avail;
	bool verbose;
	void writec(char c);
	void writecmd(char c);
	int readc(int timeout);
	int check_ack(int timeout);
public:
	Stm32Proto(const char *device, unsigned int rate, int rate_name,
		   rx_avail_t rx_avail,bool verbose = false);
	~Stm32Proto();
	void get_command();
	void get_version();
	void get_id();
	int read_mem(int address, void *dst, size_t count);
	int write_mem(int address, void *src, size_t count);
	void erase_all();
};

#endif /*__STM32PROTO_HH__*/
