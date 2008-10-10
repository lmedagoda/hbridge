#include "usart.h"
#include <stdarg.h>

#undef printf

int print(const unsigned char *format) {
  const unsigned char *fmt = format;
  unsigned int len = 0;
  
  while(*fmt) {
    fmt++;
    len++;
  }

  USART1_SendData(format, len);
  
  //USB_Send_Data(format, len);

  return len;
}

void fillInUnsignedLongInt(unsigned long int d, unsigned char *msg, int *pos) {
  int i, j;
  if(d == 0) {
    msg[*pos] = '0';
    (*pos)++;
    return;
  }
  
  i = 0;
  j = d;
  while(j) {
    j /= 10;
    i++;
  }
  
  j = i;
  while(i) {
    msg[*pos + i - 1] = (char) (d % 10) + '0';
    d /= 10;
    i--;
  }
  *pos += j;
}


int printf(const char *format, ...) {
  unsigned char msg[128];
  const char *fmt = format;
  int pos = 0;
  va_list ap;
  long int ds;
  unsigned long int d;
  unsigned short int h;
  short int hs;  
  char c, *s;

  va_start(ap, format);
  while (*fmt) {
  
    if(*fmt != '%') {
      msg[pos] = *fmt;
      pos++;
      fmt++;
      continue;
    } else {
      fmt++;
      assert_param(*fmt);
    }

    switch (*fmt) {
    case 's':              /* string */
      s = va_arg(ap, char *);
      while(*s) {
	msg[pos] = *s;
	pos++;
	s++;
      }
      break;

      
    case 'd':              /* int */
    case 'l':               /* unsigned long int */
      fmt++;
      if(*fmt == 'u') {
	d = va_arg(ap, unsigned long int);
      } else {
	ds = va_arg(ap, long int);

	if(ds < 0) {
	  msg[pos] = '-';
	  pos++;
	  //set first bit to zero, so there is no 
	  //difference singend / unsigend now
	  //and we can use generic function for unsigend int
	  ds *=-1;
	}
	d = ds;
      }

      fillInUnsignedLongInt(d, msg, &pos);

      break;

    case 'h':              /* short int */
      fmt++;
      if(*fmt == 'u') {
	h = (unsigned short int) va_arg(ap, int);
	d = h;
      } else {
	hs = (short int) va_arg(ap, int);
	if(hs < 0) {
	  msg[pos] = '-';
	  pos++;
	  //set first bit to zero, so there is no 
	  //difference singend / unsigend now
	  //and we can use generic function for unsigend int
	  hs *=-1;
	}
	d = hs;
      }
      fillInUnsignedLongInt(d, msg, &pos);
      break;
    case 'c':              /* char */
      /* need a cast here since va_arg only
	 takes fully promoted types */
      c = (char) va_arg(ap, int);
      msg[pos] = c;
      pos++;
      break;
    }
    fmt++;
  }
  
  va_end(ap);

  assert_param(pos < 128);

  u8 ret = 1;
  while(ret) {  
    ret = USART1_SendData(msg, pos);
  }
  
  //USB_Send_Data(msg, pos);  

  return pos;
}
