
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <errno.h>
#include <poll.h>
#include <string.h>
#include "stm32proto.hh"
#include "ioerror.hh"

Stm32Proto::Stm32Proto(const char *device, unsigned int rate, int rate_name,
		       rx_avail_t rx_avail,bool verbose)
  : rate(rate)
  , rate_name(rate_name)
  , rx_avail(rx_avail)
  , verbose(verbose)
{
	fd = open(device, O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);

	if (fd == -1)
	{
		char buf[256];
		sprintf(buf, "Unable to open device %s: %s\n",
			device,
			strerror(errno));
		throw IOError(strdup(buf));
	}

	int res;
	res = fcntl(fd,F_GETFL);
	if (res == -1) {
		char buf[256];
		sprintf(buf, "Unable to fcntl: %s\n",
			strerror(errno));
		throw IOError(strdup(buf));
	}
	res = fcntl(fd,F_SETFL, res & ~O_NONBLOCK);
	if (res == -1) {
		char buf[256];
		sprintf(buf, "Unable to fcntl: %s\n",
			strerror(errno));
		throw IOError(strdup(buf));
	}

	struct termios options;
	memset(&options,0,sizeof(options));
	/* Get the current options for the port */
	tcgetattr(fd, &options);
	cfsetspeed(&options, rate_name);
	cfmakeraw(&options);

	/* Enable the receiver and set local mode */
	options.c_cflag |= (CLOCAL | CREAD);
	options.c_cflag |= PARENB; /* Mask the character size to 8 bits, even parity */
	options.c_cflag &= ~PARODD;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |=  CS8;                              /* Select 8 data bits */
	options.c_cflag &= ~CRTSCTS;               /* Disable hardware flow control */

	/* Enable data to be processed as raw input */
	options.c_lflag &= ~(ICANON | ECHO | ISIG);

	/* Set the new options for the port */
	tcsetattr(fd, TCSAFLUSH, &options);
	tcflush(fd, TCIOFLUSH);


	if (rx_avail != HAVE_RX)
		usleep(1000000);
	else {
		char b;
		struct pollfd pfd;
		pfd.fd = fd;
		pfd.events = POLLIN;
		while(poll(&pfd,1,0) >= 0) {
			if (pfd.revents != POLLIN)
				break;
			read(fd, &b, 1);
		}//tcflush seems to be not reliable
	}

	writec(0x7f);
	check_ack(15);
}

Stm32Proto::~Stm32Proto() {
	close(fd);
}

void Stm32Proto::writec(char c)
{
	int res = write(fd,&c,1);
	if(res != 1) {
		char buf[256];
		sprintf(buf,"%s: bad result: %d, errno: %s\n",
			__FUNCTION__, res, strerror(errno));
		throw IOError(strdup(buf));
	}
}

void Stm32Proto::writecmd(char c)
{
	char buf[2] = {c, ~c};
	int res = write(fd,buf,2);
	if(res != 2) {
		char buf[256];
		sprintf(buf,"%s: write failed: result == %d, errno: %s\n",
			__FUNCTION__, res, strerror(errno));
		throw IOError(strdup(buf));
	}
}

int Stm32Proto::readc(int timeout)
{
	if (rx_avail == NO_RX) {
		throw IOError("readc called without rx line\n");
	}
	char c;
	int res;
	int loop = 0;
	do {
		res = read(fd,&c,1);
		if(res != 1) {
			char buf[256];
			sprintf(buf,"%s: read failed: result == %d, errno: %s(%d)\n",
				__FUNCTION__, res, strerror(errno),errno);
			throw IOError(strdup(buf));
		}
		loop++;
		if(loop > timeout && timeout != 0) {
			char buf[256];
			sprintf(buf,"timeout waiting for data\n");
			throw IOError(strdup(buf));
		}
		if (verbose)
			fprintf(stderr,".");
		usleep(1000);
	} while(res != 1);
	if (rx_avail == DEBUG_NO_RX)
		usleep(1000*(timeout-loop));
	return c;
}

int Stm32Proto::check_ack(int timeout)
{
	if (rx_avail == NO_RX) {
		if (timeout == 0) {
			char buf[256];
			sprintf(buf,"checking for ack with neither rx nor timeout\n");
			throw IOError(strdup(buf));
		}
		usleep(timeout * 1000);
		return 0;
	}
	char c = readc(timeout);
	if (c == 'y') {
		if (verbose)
			fprintf(stderr,"ack\n");
		return 0;
	}
	if (c == 0x1f) {
		if (verbose)
			fprintf(stderr,"nack\n");
	} else {
		char buf[256];
		sprintf(buf,"bad ack: %d\n",(unsigned)c);
		throw IOError(strdup(buf));
	}
	return 1;
}

void Stm32Proto::get_command()
{
	char c;

	if (verbose)
		fprintf(stderr,"command: get\n");
	writecmd(0x00);

	if (check_ack(0))
		return;

	c = readc(0);
	int bytes = (unsigned)c + 1;

	bytes--;
	c = readc(0);

	if (verbose)
		fprintf(stderr,"bootloader version %d.%d\n",((unsigned)c >> 4), c & 0xf);

	while(bytes--) {
		c = readc(0);
		if (verbose)
			fprintf(stderr,"command available: 0x%02x\n",((int)c)&0xff);
	}

	if (check_ack(0))
		return;
	if (verbose)
		fprintf(stderr,"done\n");
}

void Stm32Proto::get_version()
{
	char c;

	if (verbose)
		fprintf(stderr,"command: get version\n");
	writecmd(0x01);

	if (check_ack(0))
		return;

	c = readc(0);
	if (verbose)
		fprintf(stderr,"bootloader version %d.%d\n",((unsigned)c >> 4), c & 0xf);

	c = readc(0);
	if (verbose)
		fprintf(stderr,"option1: 0x%02x\n",((int)c)&0xff);
	c = readc(0);
	if (verbose)
		fprintf(stderr,"option2: 0x%02x\n",((int)c)&0xff);

	if (check_ack(0))
		return;
	if (verbose)
		fprintf(stderr,"done\n");
}

void Stm32Proto::get_id()
{
	char c;

	if (verbose)
		fprintf(stderr,"command: get id\n");
	writecmd(0x02);

	if (check_ack(0))
		return;

	c = readc(0);
	int bytes = (unsigned)c + 1;
	if (verbose)
		fprintf(stderr,"%d bytes follow\n",bytes);

	while(bytes--) {
		read(fd,&c,1);
		if (verbose)
			fprintf(stderr,"id data: 0x%02x\n",((int)c)&0xff);
	}

	if (check_ack(0))
		return;
	if (verbose)
		fprintf(stderr,"done\n");
}

int Stm32Proto::read_mem(int address, void *dst, size_t count)
{
	char c;

	if (verbose)
		fprintf(stderr,"command: read mem\n");
	writecmd(0x11);

	if (check_ack(20))
		return 1;

	writec(address >> 24);
	writec(address >> 16);
	writec(address >> 8);
	writec(address >> 0);
	writec((address >> 24)^(address >> 16)^(address >> 8)^(address >> 0));

	if (check_ack(20))
		return 1;

	/* if the address is otherwise invalid, e.g. not readable,
	   protected, etc, the bootloader simply aborts here.. */

	c = count - 1;
	writecmd(c);

	if (check_ack(20))
		return 1;

	char *p = (char*)dst;


	while(count) {
		int res = read(fd,p,count);
		if (res < 0) {
			if (errno == EAGAIN)
				continue;
			char buf[256];
			sprintf(buf,"error in read: %s\n",
				strerror(errno));
			throw IOError(strdup(buf));
		}
		count -= res;
		p += res;
	}

	if (verbose)
		fprintf(stderr,"done\n");
	return 0;
}

int Stm32Proto::write_mem(int address, void *src, size_t count)
{
	char c;

	if (verbose)
		fprintf(stderr,"command: write mem\n");
	writecmd(0x31);

	if (check_ack(20))
		return 1;

	writec(address >> 24);
	writec(address >> 16);
	writec(address >> 8);
	writec(address >> 0);
	writec((address >> 24)^(address >> 16)^(address >> 8)^(address >> 0));

	if (check_ack(20))
		return 1;

	char checksum = 0;

	c = count - 1;
	writec(c);
	checksum ^= c;

	char *p = (char*)src;

	int trans = count;
	while(trans) {
		int res = write(fd,p,trans);
		if (res < 0) {
			char buf[256];
			sprintf(buf,"error in read: %s\n",
				strerror(errno));
			throw IOError(strdup(buf));
		}
		trans -= res;
		while(res--) {
			checksum ^= *p++;
		}
	}

	writec(checksum);

	if (check_ack(10+(count*12*1000)/rate))
		return 1;
	if (verbose)
		fprintf(stderr,"done\n");
	return 0;
}

void Stm32Proto::erase_all()
{
	if (verbose)
		fprintf(stderr,"command: erase all\n");
	writecmd(0x43);

	if (check_ack(20))
		return;

	writecmd(0xff);

	if (check_ack(40))
		return;
	if (verbose)
		fprintf(stderr,"done\n");
}
