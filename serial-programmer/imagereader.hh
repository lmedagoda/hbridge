
#ifndef __IMAGEREADER_HH__
#define __IMAGEREADER_HH__

#include <sys/types.h>
#include <stdint.h>
#include <set>

class ImageReader {
public:
	struct Chunk {
		uint32_t address;
		size_t size;
		char buf[256];
	};
public:
	virtual bool getNextChunk(Chunk &chunk) = 0;
};

class BinaryImageReader : public ImageReader {
private:
	uint32_t address;
	int fd;
public:
	BinaryImageReader(const char *file);
	~BinaryImageReader();
	virtual bool getNextChunk(Chunk &chunk);
};

#ifdef HAVE_LIBELF

typedef struct Elf Elf;

class ELFImageReader : public ImageReader {
private:
	struct SectionHead {
		uint32_t lma;
		uint32_t size;
		char buf[0];
	};

	uint32_t address;
	int fd;
	Elf *elf;

	std::set<SectionHead*> sections;
	std::set<SectionHead*>::iterator cur_section;

	void addSection(SectionHead*h);
public:
	ELFImageReader(const char *file);
	~ELFImageReader();
	virtual bool getNextChunk(Chunk &chunk);
};

#endif /*HAVE_LIBELF*/

#endif /*__IMAGEREADER_HH__*/
