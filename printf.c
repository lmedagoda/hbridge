#include "usart.h"
#include <stdarg.h>

#undef printf

int print(const char *format) {
  const char *fmt = format;
  int len = 0;
  
  while(*fmt) {
    fmt++;
    len++;
  }

  USART1_SendData(format, len);
  
  //USB_Send_Data(format, len);

  return len;
}

int printf(const char *format, ...) {
  unsigned char msg[128];
  const char *fmt = format;
  int pos = 0;
  va_list ap;
  int d;
  int i, j;
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
    case 'l':               /* unsigned long int */
      fmt++;
      if(*fmt == 'u') {
	d = va_arg(ap, unsigned long int);
      } else {
	d = va_arg(ap, long int);
      }
      
      
      if(d == 0) {
	msg[pos] = '0';
	pos++;
	break;
      }

      if(d < 0) {
	msg[pos] = '-';
	pos++;
	d *= -1;
      }

      i = 0;
      j = d;
      while(j) {
	j /= 10;
	i++;
      }

      j = i;
      while(i) {
	msg[pos + i - 1] = (char) (d % 10) + '0';
	d /= 10;
	i--;
      }
      pos += j;
      break;

    case 'd':              /* int */
      d = va_arg(ap, int);

      if(d == 0) {
	msg[pos] = '0';
	pos++;
	break;
      }

      if(d < 0) {
	msg[pos] = '-';
	pos++;
	d *= -1;
      }

      i = 0;
      j = d;
      while(j) {
	j /= 10;
	i++;
      }

      j = i;
      while(i) {
	msg[pos + i - 1] = (char) (d % 10) + '0';
	d /= 10;
	i--;
      }
      pos += j;
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
