
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#define _GNU_SOURCE
#include <getopt.h>
#include <poll.h>

int fd;
#define HAVE_RX 0
#define NO_RX 1
#define DEBUG_NO_RX 2
int rx_avail = HAVE_RX;
const char *device = "/dev/ttyUSB0";
const char *image = "main.bin";
const char *exe;
int verbose = 0;
int baud = 38400;

void usage() {
	fprintf(stderr,"Usage: %s [-d DEVICE] [-n] IMAGE\n\n",exe);
	fprintf(stderr,
"  -d, --device=DEVICE  Serial device to use for communication, default\n"
"                          /dev/ttyUSB0\n");
	fprintf(stderr,
"  -b, --baud=RATE      Serial communication speed, default 38400\n");
	fprintf(stderr,
"  -n, --no-rx          Don't depend on acknowledgments received from serial\n"
"                           device, use timing based transmission instead\n");
	fprintf(stderr,
"      --debug-no-rx    Use timing like --no-rx, but still check received\n"
"                          acknowledgments\n");
	fprintf(stderr,
"  -v, --verbose        Output info about the commands currently run\n");
	exit(1);
}

void parseArgs(int argc,char **argv) {
	char option;
	int option_index;
	
	exe = *argv;
	
	static struct option long_options[] = {
		{ "debug-no-rx", no_argument,       NULL, 0},
		{ "device",      required_argument, NULL, 'd'},
		{ "baud",        required_argument, NULL, 'b'},
		{ "no-rx",       no_argument,       NULL, 'n'},
		{ "help",        no_argument,       NULL, 'h'},
		{ "verbose",     no_argument,       NULL, 'v'},
		{ NULL,          0,                 NULL, 0}
	};
	
	
	while ( (option = getopt_long( argc, argv, "d:b:nhv" ,
				       long_options, &option_index)) != EOF ) {
		switch ( option ) {
		case 0:
			switch ( option_index ) {
			case 0:
				rx_avail = DEBUG_NO_RX;
				break;
			default:
				fprintf(stderr,"%s: getopt_long returned option_index %d\n",*argv,option_index);
				usage();
			}
			break;
		case 'd' :
			device = strdup(optarg);
			break;
		case 'b' :
			baud = atoi(optarg);
			break;
		case 'n' :
			rx_avail = NO_RX;
			break;
		case 'v' :
			verbose = 1;
			break;
		case 'h' :
		case '?' :
			usage();
			break;
		default:
			fprintf(stderr,"%s: getopt_long returned code %d\n", *argv, option);
			usage();
			break;
		}
	}
	if (optind >= argc) {
		fprintf(stderr,"%s: No image to program\n", *argv);
		usage();
	}
	image = strdup(argv[optind]);
}

void writec(char c)
{
	int res = write(fd,&c,1);
	if(res != 1) {
		fprintf(stderr,"%s: bad result: %d, errno: %s\n",
			__FUNCTION__, res, strerror(errno));
		exit(1);
	}
}

void writecmd(char c)
{
	char buf[2] = {c, ~c};
	int res = write(fd,buf,2);
	if(res != 2) {
		fprintf(stderr,"%s: write failed: result == %d, errno: %s\n",
			__FUNCTION__, res, strerror(errno));
		exit(1);
	}
}

int readc(int timeout)
{
	if (rx_avail == NO_RX) {
		fprintf(stderr,"readc called without rx line\n");
		exit(1);
	}
	char c;
	int res;
	int loop = 0;
	do {
		res = read(fd,&c,1);
		if(res != 1) {
			fprintf(stderr,"%s: read failed: result == %d, errno: %s(%d)\n",
				__FUNCTION__, res, strerror(errno),errno);
			exit(1);
		}
		loop++;
		if(loop > timeout && timeout != 0) {
			fprintf(stderr,"timeout waiting for data\n");
			exit(1);
		}
		if (verbose)
			fprintf(stderr,".");
		usleep(1000);
	} while(res != 1);
	if (rx_avail == DEBUG_NO_RX)
		usleep(1000*(timeout-loop));
	return c;
}

int check_ack(int timeout)
{
	if (rx_avail == NO_RX) {
		if (timeout == 0) {
			fprintf(stderr,"checking for ack with neither rx nor timeout\n");
			exit(1);
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
		fprintf(stderr,"bad ack: %d\n",(unsigned)c);
		exit(1);
	}
	return 1;
}

/* these are not used in any way */
#if 0
void get_command()
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

void get_version()
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

void get_id()
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
#endif

int read_mem(int address, void *dst, size_t count)
{
	char c;

	if (verbose)
		fprintf(stderr,"command: read mem\n");
	writecmd(0x11);

	if (check_ack(0))
		return 1;

	writec(address >> 24);
	writec(address >> 16);
	writec(address >> 8);
	writec(address >> 0);
	writec((address >> 24)^(address >> 16)^(address >> 8)^(address >> 0));

	if (check_ack(0))
		return 1;

	/* if the address is otherwise invalid, e.g. not readable,
	   protected, etc, the bootloader simply aborts here.. */

	c = count - 1;
	writecmd(c);

	if (check_ack(0))
		return 1;

	char *p = dst;


	while(count) {
		int res = read(fd,p,count);
		if (res < 0) {
			if (errno == EAGAIN)
				continue;
			fprintf(stderr,"error in read: %s\n",
				strerror(errno));
			exit(1);
		}
		count -= res;
		p += res;
	}

	if (verbose)
		fprintf(stderr,"done\n");
	return 0;
}

int write_mem(int address, void *src, size_t count)
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

	char *p = src;

	int trans = count;
	while(trans) {
		int res = write(fd,p,trans);
		if (res < 0) {
			fprintf(stderr,"error in read: %s\n",
				strerror(errno));
			exit(1);
		}
		trans -= res;
		while(res--) {
			checksum ^= *p++;
		}
	}

	writec(checksum);

	if (check_ack(10+(count*12*1000)/baud))
		return 1;
	if (verbose)
		fprintf(stderr,"done\n");
	return 0;
}

void erase_all()
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

static struct 
{
	int rate; int name;
} baudMap[] = {
#define B(v) { v, B ## v }
	B(19200),
	B(38400),
	B(57600),
	B(115200),
	B(230400),
	B(460800),
	B(500000),
	B(576000),
	B(921600),
	B(1000000),
	B(1152000),
	B(1500000),
	B(2000000),
	B(2500000),
	B(3000000),
	B(3500000),
	B(4000000),
#undef B
};

int main(int argc, char **argv)
{
	parseArgs(argc,argv);

	fd = open(device, O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
	
	if (fd == -1)
	{
		fprintf(stderr, "Unable to open device %s: %s\n",
			device,
			strerror(errno));
		return 1;
	} 
	
	int res;
	res = fcntl(fd,F_GETFL);
	if (res == -1) {
		fprintf(stderr, "Unable to fcntl: %s\n",
			strerror(errno));
		return 1;
	}
	res = fcntl(fd,F_SETFL, res & ~O_NONBLOCK);
	if (res == -1) {
		fprintf(stderr, "Unable to fcntl: %s\n",
			strerror(errno));
		return 1;
	}

	{
		struct termios options;
		/* Get the current options for the port */
		tcgetattr(fd, &options);
		int i;
		for(i = 0; i < sizeof(baudMap)/sizeof(baudMap[0]); i++) {
			if (baudMap[i].rate == baud) {
				cfsetspeed(&options, baudMap[i].name);
				break;
			}
		}
		if (i == sizeof(baudMap)/sizeof(baudMap[0])) {
			fprintf(stderr, "Unrecognized baudrate %d\n",
				baud);
			return 1;
		}
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
	}

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

	erase_all();

	int ifd = open(image,O_RDONLY);

	if (ifd < 0) {
		fprintf(stderr, "Unable to open image file %s: %s\n",
			image,
			strerror(errno));
		return 1;
	}

	char buf[256];
	char vbuf[256];
	
	int count;
	int addr = 0x08000000;
	do {
		count = read(ifd,buf,256);
		if (verbose)
			fprintf(stderr,"writing %d bytes at address %08x\n", count, addr);
		if (count < 0) {
			fprintf(stderr,"read error: %s\n",strerror(errno));
			return 1;
		}
		if(write_mem(addr,buf,count)) {
			fprintf(stderr,"write error\n");
			return 1;
		}
		if (rx_avail == HAVE_RX) {
			if(read_mem(addr,vbuf,count)) {
				fprintf(stderr,"readback error\n");
				return 1;
			}
			if (memcmp(buf,vbuf,count) != 0) {
				fprintf(stderr,"verify failed\n");
				return 1;
			}
		}
		addr += count;
	} while(count == 256);

	return 0;
}
