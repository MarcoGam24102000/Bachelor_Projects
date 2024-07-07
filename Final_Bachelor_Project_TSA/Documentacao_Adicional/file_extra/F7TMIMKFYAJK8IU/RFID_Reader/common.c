#include "common.h"
#include <stdio.h>
#include <limits.h>

char *
baseconv(unsigned int num, int base)
{
	static char retbuf[sizeof(int)*CHAR_BIT+1];
	char * p;

	if (base < 2 || base > 16) return NULL;
	p = &retbuf[sizeof(retbuf)-1];
	*p = '\0';

	do {
		*--p = "01234567890abcdef"[num % base];
		num /= base;
	} while(num != 0);

	return p;
}
