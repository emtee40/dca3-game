#pragma once

#ifndef RW_DC
#pragma warning(disable: 4244)	// int to float
#pragma warning(disable: 4800)	// int to bool
#pragma warning(disable: 4838)  // narrowing conversion
#pragma warning(disable: 4996)  // POSIX names
#elif !defined(DC_TEXCONV) && !defined(DC_SIM)
#include <kos.h>
#define DC_SH4
#else
#ifndef __always_inline 
#define __always_inline inline
#endif
#define memcpy4 memcpy
#define dcache_pref_block(a)	__builtin_prefetch(a)
#endif
