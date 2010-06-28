
#ifndef __IOERROR_H__
#define __IOERROR_H__

#include <exception>

class IOError : public std::exception {
private:
	const char *_what;
public:
	IOError(const char* _what) : _what(_what) {}
	virtual const char* what() const throw() {
		return _what;
	}
};

#endif /*__IOERROR_H__*/
