
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
//#include <libelf.h>
#include <stdlib.h>
#include "imagereader.hh"
#include "ioerror.hh"

#define ROM_BASE 0x08000000

BinaryImageReader::BinaryImageReader(const char *file)
	: address(ROM_BASE)
{
	fd = open(file,O_RDONLY);

	if (fd < 0) {
		char buf[256];
		sprintf(buf, "Unable to open image file %s: %s\n",
			file,
			strerror(errno));
		throw IOError(strdup(buf));
	}

	fprintf(stderr,"Binary reader initialized\n");
}

BinaryImageReader::~BinaryImageReader() {
	close(fd);
}

bool BinaryImageReader::getNextChunk(Chunk &chunk) {
	int count = read(fd,chunk.buf,256);
	if (count < 0) {
		char buf[256];
		sprintf(buf,"read error: %s\n",strerror(errno));
		throw IOError(strdup(buf));
	}
	chunk.size = count;
	chunk.address = address;
	address += count;
	return count > 0;
}

#ifdef HAVE_LIBELF

void ELFImageReader::addSection(SectionHead*h) {
	//find a section after this one
	for(std::set<SectionHead*>::iterator it = sections.begin();
	    it != sections.end(); it++) {
		if (h->lma+h->size == (*it)->lma) {
			//pull it from the set
			SectionHead *h2 = *it;
			sections.erase(h2);

			// glue it to the end of h
			h = (SectionHead*)realloc
				(h,sizeof(SectionHead)+h->size+h2->size);
			memcpy(h->buf + h->size,
			       h2->buf,h2->size);
			h->size += h2->size;
			free(h2);
			break;
		}
	}
	//find a section before this one
	for(std::set<SectionHead*>::iterator it = sections.begin();
	    it != sections.end(); it++) {
		if ((*it)->lma+(*it)->size == h->lma) {
			//pull it from the set
			SectionHead *h2 = *it;
			sections.erase(h2);

			// glue h to the end of h2
			h2 = (SectionHead*)realloc
				(h2,sizeof(SectionHead)+h2->size+h->size);
			memcpy(h2->buf+h2->size,
			       h->buf,h->size);
			h2->size += h->size;
			free(h);
			h = h2;
			break;
		}
	}

	sections.insert(h);
}

ELFImageReader::ELFImageReader(const char *file) {

	if (elf_version(EV_CURRENT) == EV_NONE) {
		char buf[256];
		sprintf(buf, "ELF library too old\n");
		throw IOError(strdup(buf));
	}

	fd = open(file,O_RDONLY);

	if (fd < 0) {
		char buf[256];
		sprintf(buf, "Unable to open image file %s: %s\n",
			file,
			strerror(errno));
		throw IOError(strdup(buf));
	}

	elf = elf_begin(fd, ELF_C_READ, NULL);
	if (!elf) {
		char buf[256];
		sprintf(buf, "elf_begin failed for %s: %s\n",
			file,
			elf_errmsg(elf_errno()));
		throw IOError(strdup(buf));
	}

	char *ident = elf_getident(elf, NULL);
	if (!ident) {
		char buf[256];
		sprintf(buf, "elf_getident failed for %s: %s\n",
			file,
			elf_errmsg(elf_errno()));
		elf_end(elf);
		throw IOError(strdup(buf));
	}

	size_t phdrnum = 0;
	if (elf_getphdrnum(elf, &phdrnum)) {
		char buf[256];
		sprintf(buf, "elf_getphdrnum failed for %s: %s\n",
			file,
			elf_errmsg(elf_errno()));
		elf_end(elf);
		throw IOError(strdup(buf));
	}

	//this very likely _is_ an ELF binary. No failing after this point.

	Elf32_Phdr *phdr32 = elf32_getphdr(elf);
	Elf64_Phdr *phdr64 = elf64_getphdr(elf);

	Elf_Scn * scn = NULL;
	while( (scn = elf_nextscn(elf,scn))) {
		Elf32_Shdr *shdr32 = elf32_getshdr(scn);
		Elf64_Shdr *shdr64 = elf64_getshdr(scn);
		if (shdr32 && phdr32) {
			Elf32_Phdr *phdr32_match = NULL;
			for(unsigned int i = 0; i < phdrnum; i++) {
				if (phdr32[i].p_type == PT_LOAD &&
				    phdr32[i].p_offset == shdr32->sh_offset) {
					phdr32_match = phdr32+i;
					break;
				}
			}

			if ((shdr32->sh_flags & SHF_ALLOC) &&
			    phdr32_match && phdr32_match->p_filesz) {

				//pull them out, remember lma, if needed add rom base
				size_t lma = phdr32_match->p_paddr;

				//fix up lma
				if (lma < ROM_BASE)
					lma += ROM_BASE;

				Elf_Data *data = NULL;
				while( (data = elf_getdata(scn, data)) ) {
					if (data->d_type == ELF_T_BYTE) {
						SectionHead *h = (SectionHead*)malloc(data->d_size+sizeof(SectionHead));
						char *buffer = (char*)(h+1);
						memcpy(buffer,data->d_buf,data->d_size);

						h->lma = lma + data->d_off;
						h->size = data->d_size;
						addSection(h);
					}
				}
			}
		}
		if (shdr64 && phdr64) {
			Elf64_Phdr *phdr64_match = NULL;
			for(unsigned int i = 0; i < phdrnum; i++) {
				if (phdr64[i].p_type == PT_LOAD &&
				    phdr64[i].p_offset == shdr64->sh_offset) {
					phdr64_match = phdr64+i;
					break;
				}
			}

			if ((shdr64->sh_flags & SHF_ALLOC) &&
			    phdr64_match && phdr64_match->p_filesz) {

				//pull them out, remember lma, if needed add rom base
				size_t lma = phdr64_match->p_paddr;

				//fix up lma
				if (lma < ROM_BASE)
					lma += ROM_BASE;

				Elf_Data *data = NULL;
				while( (data = elf_getdata(scn, data)) ) {
					if (data->d_type == ELF_T_BYTE) {
						SectionHead *h = (SectionHead*)malloc(data->d_size+sizeof(SectionHead));
						char *buffer = (char*)(h+1);
						memcpy(buffer,data->d_buf,data->d_size);

						h->lma = lma + data->d_off;
						h->size = data->d_size;
						addSection(h);
					}
				}
			}
		}
	}
	cur_section = sections.begin();

	if (cur_section != sections.end()) {
		address = (*cur_section)->lma;
	}

	fprintf(stderr,"ELF reader initialized with %lu sections\n",
		(unsigned long)sections.size());
}

ELFImageReader::~ELFImageReader() {
	elf_end(elf);
}

bool ELFImageReader::getNextChunk(Chunk &chunk) {
	if (cur_section == sections.end())
		return false;
	if (address == (*cur_section)->lma + (*cur_section)->size) {
		//next section
		cur_section++;
		if (cur_section == sections.end())
			return false;
		address = (*cur_section)->lma;
	}

	size_t tocopy = 256;
	if (tocopy+address > (*cur_section)->lma+(*cur_section)->size)
		tocopy = (*cur_section)->lma+(*cur_section)->size-address;
	memcpy(chunk.buf,(*cur_section)->buf+address-(*cur_section)->lma,
	       tocopy);
	chunk.size = tocopy;
	chunk.address = address;
	address += tocopy;

	return true;
}

#endif /*HAVE_LIBELF*/
