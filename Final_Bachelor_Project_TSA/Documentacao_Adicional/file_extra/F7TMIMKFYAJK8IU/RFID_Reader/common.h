

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>


// This just makes my life easier
#define BIT16(b) ((unsigned long)0x00000001 << (b))
#define	BIT8(b) (0x01 << (b))
char * baseconv(unsigned int num, int base);

// from AVR035: Efficient C Coding for AVR 
#define BSET(ADDRESS,BIT) (ADDRESS |= (1<<BIT)) 
#define BCLR(ADDRESS,BIT) (ADDRESS &= ~(1<<BIT)) 
#define BTOG(ADDRESS,BIT) (ADDRESS ^= (1<<BIT)) 
#define BCHK(ADDRESS,BIT) (ADDRESS &  (1<<BIT)) 

#define BMSET(x,y) (x |= (y)) 
#define BMCLR(x,y) (x &= (~y)) 
#define BMTOG(x,y) (x ^= (y)) 
#define BMCHK(x,y) (x & (y)) 

#define BMPRINT(x) printf("[%8s]\n",baseconv(x,2));
