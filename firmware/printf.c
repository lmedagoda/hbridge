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
  unsigned long int i, j;
  
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
	fmt++;
	d = (unsigned long int) va_arg(ap, long int);
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
	fmt++;
	h = (unsigned short int) va_arg(ap, int);
	d = h;
      } else {
	hs = (short int) va_arg(ap, int);
	d = hs;
	if(hs < 0) {
	  msg[pos] = '-';
	  pos++;
	  //set first bit to zero, so there is no 
	  //difference singend / unsigend now
	  //and we can use generic function for unsigend int
	  ds = hs;
	  ds *=-1;
	  d = ds;
	}
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

void testprintf() {
  u16 unsignedShort;
  s16 signedShort;
  u32 unsignedLong;
  s32 signedLong;

  //u16
  unsignedShort = 0;
  printf("u16 0 is %hu \n", unsignedShort);

  unsignedShort = 12345;
  printf("u16 12345 is %hu \n", unsignedShort);
  
  unsignedShort = 65535;
  printf("u16 65535 is %hu\n", unsignedShort);
  
  //s16
  signedShort = 0;
  printf("s16 0 is %h \n", signedShort);

  signedShort = 32767;
  printf("s16 32767 is %h \n", signedShort);

  signedShort = -32768;
  printf("s16 -32768 is %h \n", signedShort);

  //u32
  unsignedLong = 0;
  printf("u32 0 is %lu \n", unsignedLong);

  unsignedLong = 1234567;
  printf("u32 1234567 is %lu \n", unsignedLong);

  unsignedLong = 4294967295;
  printf("u32 4294967295 is %lu \n", unsignedLong);

  //s32
  signedLong = 0;
  printf("s32 0 is %l \n", signedLong);

  signedLong = 2147483647;
  printf("s32 2147483647 is %l \n", signedLong);

  signedLong = -2147483648;
  printf("s32 -2147483648 is %l \n", signedLong);


}