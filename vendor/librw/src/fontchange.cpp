#include <cstdint>
#include <cstdio>

const uint8_t fontbits[256*256] = {
#include "vgafont.inc"
};

int main(int, const char**) {

for (int i = 0; i < 256 * 256; i+= 8) {
	uint8_t v = 
				(fontbits[i+0] << 0) | 
				(fontbits[i+1] << 1) |
				(fontbits[i+2] << 2) |
				(fontbits[i+3] << 3) |
				(fontbits[i+4] << 4) |
				(fontbits[i+5] << 5) |
				(fontbits[i+6] << 6) |
				(fontbits[i+7] << 7);
	printf("%d,%c", v, (i & 255 ) == 248 ? '\n' : ' ' );
}
return 0;
}
