
#include <stdio.h>
#include <termios.h>
#include <sys/types.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <getopt.h>
#include "stm32proto.hh"
#include "imagereader.hh"
#include "ioerror.hh"


static Stm32Proto::rx_avail_t rx_avail = Stm32Proto::HAVE_RX;
static const char *device = "/dev/ttyUSB0";
static const char *image = "main.bin";
static const char *exe;
static int verbose = 0;
static unsigned int rate = 38400;

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
				rx_avail = Stm32Proto::DEBUG_NO_RX;
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
			rate = atoi(optarg);
			break;
		case 'n' :
			rx_avail = Stm32Proto::NO_RX;
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

static struct
{
	unsigned int rate; int name;
} rateMap[] = {
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

	ImageReader *img = NULL;
#ifdef HAVE_LIBELF
	if (!img)
		try {
			img = new ELFImageReader(image);
		} catch(IOError e) {
			fprintf(stderr,"ELF reader reported: %s\n",
				e.what());
			img = NULL;
		}
#endif /*HAVE_LIBELF*/
	if (!img)
		try {
			img = new BinaryImageReader(image);
		} catch(IOError) {
			img = NULL;
		}
	if (!img) {
		fprintf(stderr,"Could not open image %s\n",
			image);
		return 1;
	}

	unsigned int i;
	int rate_name = B38400;
	for(i = 0; i < sizeof(rateMap)/sizeof(rateMap[0]); i++) {
		if (rateMap[i].rate == rate) {
			rate_name = rateMap[i].name;
			break;
		}
	}
	if (i == sizeof(rateMap)/sizeof(rateMap[0])) {
		fprintf(stderr, "Unrecognized baudrate %d\n",
			rate);
		return 1;
	}

	try {
		Stm32Proto proto(device,rate,rate_name,rx_avail,verbose != 0);

		proto.erase_all();

		char vbuf[256];

		ImageReader::Chunk chunk;
		while(img->getNextChunk(chunk)) {
			if (verbose)
				fprintf(stderr,
					"writing %lu bytes at address %08x\n",
					(unsigned long)chunk.size,
					chunk.address);
			if(proto.write_mem(chunk.address,chunk.buf,
					   chunk.size)) {
				fprintf(stderr,"write error\n");
				return 1;
			}
			if (rx_avail == Stm32Proto::HAVE_RX ||
			    rx_avail == Stm32Proto::DEBUG_NO_RX) {
				if(proto.read_mem(chunk.address,vbuf,
						  chunk.size)) {
					fprintf(stderr,"readback error\n");
					return 1;
				}
				if (memcmp(chunk.buf,vbuf,chunk.size) != 0) {
					fprintf(stderr,"verify failed\n");
					return 1;
				}
			}
		}

	} catch (IOError e) {
		fputs(e.what(),stderr);
		return 1;
	}

	return 0;
}
