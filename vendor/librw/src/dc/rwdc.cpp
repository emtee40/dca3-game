#ifdef RW_DC

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#if !defined(DC_TEXCONV) && !defined(MACOS64)
#include <malloc.h>
#endif

#if defined(DC_TEXCONV)
#include "tri_stripper.h"
#include <sha256.h>
extern const char* currentFile;
#define texconvf(...) // printf(__VA_ARGS__)
#endif

#include "../../../src/vmu/vmu.h"
#include "../rwbase.h"
#include "../rwerror.h"
#include "../rwplg.h"
#include "../rwpipeline.h"
#include "../rwobjects.h"
#include "../rwengine.h"
#include "../rwanim.h"
#include "../rwplugins.h"
#include "../rwrender.h"
#include "rwdc.h"
#include "vq.h"
#include "tex-util.h"

#include <vector>
#include <set>
#include <list>
#include <map>
#include <functional>
#include <fstream>

#define logf(...) // printf(__VA_ARGS__)
bool re3RemoveLeastUsedModel();

// #include "rwdcimpl.h"

#include <dc/pvr.h>
#include "alloc.h"

#undef PVR_TXRFMT_STRIDE
#define PVR_TXRFMT_STRIDE       (1 << 25)

static_assert(PVR_TXRFMT_STRIDE == (1 << 25), "PVR_TXRFMT_STRIDE is bugged in your KOS version");

// TODO: probably needs a better place to be
bool doEnvironmentMaps = true;

#define fclamp0_1(n) ((n) > 1.0f ? 1.0f : n < 0.0f ? 0.0f : n)
#define fclamp1(n) ((n) > 1.0f ? 1.0f : n)

struct alignas(32) pvr_vertex16_t {
	uint32_t flags;			/**< \brief TA command (vertex flags) */
	float	 x;				/**< \brief X coordinate */
	float	 y;				/**< \brief Y coordinate */
	float	 z;				/**< \brief Z coordinate */
	union {
		struct {
			uint16_t v;		/**< \brief Texture V coordinate */
			uint16_t u;		/**< \brief Texture U coordinate */
		};
		uint32_t uv;
	};
	float w;				// not real, just padding
	union {
		struct {
			uint8_t b;		
			uint8_t g;	
			uint8_t r;	
			uint8_t a;		
		};
		uint32_t argb;
	};
	union {
		struct {
			uint8_t o_b;		
			uint8_t o_g;	
			uint8_t o_r;	
			uint8_t o_a;		
		};
		uint32_t oargb;
	};
};

struct alignas(32) pvr_vertex64_t {
	uint32_t flags;			/**< \brief TA command (vertex flags) */
	float	 x;				/**< \brief X coordinate */
	float	 y;				/**< \brief Y coordinate */
	float	 z;				/**< \brief Z coordinate */
	union {
		struct {
			uint16_t v;		/**< \brief Texture V coordinate */
			uint16_t u;		/**< \brief Texture U coordinate */
		};
		uint32_t uv;
		float uf32;
	};
	union {
		uint32_t _dmy1;
		float vf32;
	};
	
	union {
		uint32_t _dmy2;
		float tex_z;
	};
	uint32_t _dmy3;
	float a;
	float r;
	float g;
	float b;
	float o_a;
	float o_r;
	float o_g;
	float o_b;	
};

struct alignas(32) pvr_vertex64_t1 {
	uint32_t flags;			/**< \brief TA command (vertex flags) */
	float	 x;				/**< \brief X coordinate */
	float	 y;				/**< \brief Y coordinate */
	float	 z;				/**< \brief Z coordinate */
	union {
		struct {
			uint16_t v;		/**< \brief Texture V coordinate */
			uint16_t u;		/**< \brief Texture U coordinate */
		};
		uint32_t uv;
	};
	float    dmy1;
	float    w;             // not real, just padding
	uint32_t dmy3;
};

struct alignas(32) pvr_vertex64_t2 {
	float a;
	float r;
	float g;
	float b;
	float o_a;
	float o_r;
	float o_g;
	float o_b;
};

struct alignas(32) pvr_vertex32_ut {
	uint32_t flags;			/**< \brief TA command (vertex flags) */
	float	 x;				/**< \brief X coordinate */
	float	 y;				/**< \brief Y coordinate */
	float	 z;				/**< \brief Z coordinate */
	float a;
	float r;
	float g;
	float b; 
};

static_assert(sizeof(pvr_vertex16_t) == 32, "pvr_vertex16_t size mismatch");
static_assert(alignof(pvr_vertex16_t) == 32, "pvr_vertex16_t alignof mismatch");


#define MATH_Fast_Invert(x) ({ (((x) < 0.0f)? -1.0f : 1.0f) * frsqrt((x) * (x)); }) 

#define logf(...) // printf(__VA_ARGS__)

static pvr_dr_state_t drState;

#include <kos/dbglog.h>

#if !defined(DC_TEXCONV) && !defined(DC_SIM)
#include <kos.h>

#define VIDEO_MODE_WIDTH  vid_mode->width
#define VIDEO_MODE_HEIGHT vid_mode->height

#define mat_trans_nodiv_nomod(x, y, z, x2, y2, z2, w2) do { \
        register float __x __asm__("fr12") = (x); \
        register float __y __asm__("fr13") = (y); \
        register float __z __asm__("fr14") = (z); \
        register float __w __asm__("fr15") = 1.0f; \
        __asm__ __volatile__( "ftrv  xmtrx, fv12\n" \
                              : "=f" (__x), "=f" (__y), "=f" (__z), "=f" (__w) \
                              : "0" (__x), "1" (__y), "2" (__z), "3" (__w) ); \
        x2 = __x; y2 = __y; z2 = __z; w2 = __w; \
    } while(false)

#define mat_trans_nodiv_nomod_zerow(x, y, z, x2, y2, z2, w2) do { \
        register float __x __asm__("fr12") = (x); \
        register float __y __asm__("fr13") = (y); \
        register float __z __asm__("fr14") = (z); \
        register float __w __asm__("fr15") = 0.0f; \
        __asm__ __volatile__( "ftrv  xmtrx, fv12\n" \
                              : "=f" (__x), "=f" (__y), "=f" (__z), "=f" (__w) \
                              : "0" (__x), "1" (__y), "2" (__z), "3" (__w) ); \
        x2 = __x; y2 = __y; z2 = __z; w2 = __w; \
    } while(false)

#define mat_trans_w_nodiv_nomod(x, y, z, w) do { \
        register float __x __asm__("fr12") = (x); \
        register float __y __asm__("fr13") = (y); \
        register float __z __asm__("fr14") = (z); \
        register float __w __asm__("fr15") = 1.0f; \
        __asm__ __volatile__( "ftrv  xmtrx, fv12\n" \
                              : "=f" (__x), "=f" (__y), "=f" (__z), "=f" (__w) \
                              : "0" (__x), "1" (__y), "2" (__z), "3" (__w) ); \
        w = __w; \
    } while(false)

	// no declspec naked, so can't do rts / fschg. instead compiler pads with nop?

	inline void rw_mat_load_3x3(const rw::Matrix* mtx) {
		__asm__ __volatile__ (
			R"(
				fschg
				frchg

				fmov        @%[mtx]+, dr0

				fldi0 		fr12
				fldi0 		fr13

				fmov        @%[mtx]+, dr2
				fmov        @%[mtx]+, dr4
				fmov        @%[mtx]+, dr6
				fmov        @%[mtx]+, dr8
				fmov        @%[mtx]+, dr10

				fldi0	    fr3
				fldi0	    fr7
				fldi0	    fr11
				fmov        dr12, dr14

				fschg
				frchg
			)"
			: [mtx] "+r" (mtx)
		);
	}

	// sets pos.w to 1
	inline void rw_mat_load_4x4(const rw::Matrix* mtx) {
		__asm__ __volatile__ (
			R"(
				fschg
				frchg
				fmov        @%[mtx]+, dr0

				fmov        @%[mtx]+, dr2
				fmov        @%[mtx]+, dr4
				fmov        @%[mtx]+, dr6
				fmov        @%[mtx]+, dr8
				fmov        @%[mtx]+, dr10
				fmov        @%[mtx]+, dr12
				fmov        @%[mtx]+, dr14
				fldi1 	 	fr15

				fschg
				frchg
			)"
			: [mtx] "+r" (mtx)
		);
	}

#else
extern matrix_t XMTRX;

void rw_mat_load_3x3(rw::Matrix* mtx) {
	memcpy(XMTRX, mtx, sizeof(matrix_t));
	XMTRX[0][3] = 0.0f;
	XMTRX[1][3] = 0.0f;
	XMTRX[2][3] = 0.0f;

	XMTRX[3][0] = 0.0f;
	XMTRX[3][1] = 0.0f;
	XMTRX[3][2] = 0.0f;
	XMTRX[3][3] = 0.0f;
}

void rw_mat_load_4x4(rw::Matrix* mtx) {
	memcpy(XMTRX, mtx, sizeof(matrix_t));
	XMTRX[3][3] = 1.0f;
}

#include <dc/matrix.h>
#define VIDEO_MODE_WIDTH		640
#define VIDEO_MODE_HEIGHT 		480
#define frsqrt(a) 				(1.0f/sqrt(a))
#define dcache_pref_block(a)	__builtin_prefetch(a)

#ifndef __always_inline
#define __always_inline 		__attribute__((always_inline)) inline
#endif

#ifdef DC_TEXCONV
#define mat_transform(a, b, c, d)
#define mat_apply(a)
#define mat_load(a)
#define mat_store(a)
#define mat_identity(a)
#define pvr_fog_table_color(a,r,g,b)
#define pvr_fog_table_linear(s,e)
#endif

#define mat_trans_single3_nomod(x_, y_, z_, x2, y2, z2) do { \
		vector_t tmp = { x_, y_, z_, 1.0f }; \
		mat_transform(&tmp, &tmp, 1, 0); \
		z2 = 1.0f / tmp.w; \
		x2 = tmp.x * z2; \
		y2 = tmp.y * z2; \
	} while(false)

#define mat_trans_nodiv_nomod(x_, y_, z_, x2, y2, z2, w2) do { \
		vector_t tmp1233123 = { x_, y_, z_, 1.0f }; \
		mat_transform(&tmp1233123, &tmp1233123, 1, 0); \
		x2 = tmp1233123.x; y2 = tmp1233123.y; z2 = tmp1233123.z; w2 = tmp1233123.w; \
	} while(false)

#define mat_trans_nodiv_nomod_zerow(x_, y_, z_, x2, y2, z2, w2) do { \
		vector_t tmp1233123 = { x_, y_, z_, 0.0f }; \
		mat_transform(&tmp1233123, &tmp1233123, 1, 0); \
		x2 = tmp1233123.x; y2 = tmp1233123.y; z2 = tmp1233123.z; w2 = tmp1233123.w; \
	} while(false)

#define mat_trans_w_nodiv_nomod(x_, y_, z_, w_) do { \
		vector_t tmp1233123 = { x_, y_, z_, 1.0f }; \
		mat_transform(&tmp1233123, &tmp1233123, 1, 0); \
		w_ = tmp1233123.w; \
	} while(false)

#define memcpy4 memcpy

// END STUBS
#endif

static pvr_ptr_t fake_tex;

alignas(4) static const uint16_t fake_tex_data[] = {
    /* Alternating 0xffff / 0x7fff but pre-twiddled */
    0xffff, 0xffff, 0x7fff, 0x7fff, 0xffff, 0xffff, 0x7fff, 0x7fff,
    0xffff, 0xffff, 0x7fff, 0x7fff, 0xffff, 0xffff, 0x7fff, 0x7fff,
    0xffff, 0xffff, 0x7fff, 0x7fff, 0xffff, 0xffff, 0x7fff, 0x7fff,
    0xffff, 0xffff, 0x7fff, 0x7fff, 0xffff, 0xffff, 0x7fff, 0x7fff,
    0xffff, 0xffff, 0x7fff, 0x7fff, 0xffff, 0xffff, 0x7fff, 0x7fff,
    0xffff, 0xffff, 0x7fff, 0x7fff, 0xffff, 0xffff, 0x7fff, 0x7fff,
    0xffff, 0xffff, 0x7fff, 0x7fff, 0xffff, 0xffff, 0x7fff, 0x7fff,
    0xffff, 0xffff, 0x7fff, 0x7fff, 0xffff, 0xffff, 0x7fff, 0x7fff,
};

void enter_oix_() {
	#if defined(DC_SH4)
	auto mask = irq_disable();
	dcache_purge_all();
	volatile uint32_t * CCN_CCR = (uint32_t *)0xFF00001C;
	*CCN_CCR |= (1 << 7); // enable OIX
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");

	for (unsigned i = 0x92000000; i < (0x92000000 + 8192); i += 32) {
		__asm__ __volatile__ ("movca.l r0,@%0" : : "r" (i): "memory");
	}

	irq_restore(mask);
	#endif
}

void leave_oix_() {
	#if defined(DC_SH4)
	auto mask = irq_disable();
	dcache_inval_range(0x92000000, 8192);
	dcache_purge_all();
	volatile uint32_t * CCN_CCR = (uint32_t *)0xFF00001C;
	*CCN_CCR &= ~( 1 << 7); // disable OIX
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	irq_restore(mask);
	#endif
}

#if defined(DC_SH4)
#define FLUSH_TA_DATA(src) do { __asm__ __volatile__("ocbwb @%0" : : "r" (src) : "memory"); } while(0)
#else
#define FLUSH_TA_DATA(src) do { pvr_dr_commit(src); } while(0)
#endif

#if defined(DC_SH4)
void (*enter_oix)() = (void(*)())(((uintptr_t)&enter_oix_) - 0x8c000000 + 0xAc000000);
void (*leave_oix)() = (void(*)())(((uintptr_t)&leave_oix_) - 0x8c000000 + 0xAc000000);
#else
void (*enter_oix)() = enter_oix_;
void (*leave_oix)() = leave_oix_;
#endif
namespace {

matrix_t DCE_MAT_SCREENVIEW;
matrix_t DCE_MAT_IDENTITY;
matrix_t DCE_MAT_LOOKAT;
matrix_t DCE_MAT_PROJECTION;

matrix_t DCE_MAT_FRUSTUM = {
    { 0.0f, 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f, -1.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f }
};

matrix_t DCE_MESHLET_MAT_DECODE = {
	{ 1.0f/256, 0.0f, 0.0f, 0.0f},
	{ 0.0f, 1.0f/256, 0.0f, 0.0f},
	{ 0.0f, 0.0f, 1.0f/256, 0.0f},
	{ 0.0f, 0.0f, 0.0f, 1.0f},
};

matrix_t DCE_MESHLET_MAT_VERTEX_COLOR = {
	{ 1.0f, 0.0f, 0.0f, 0.0f},
	{ 0.0f, 1.0f, 0.0f, 0.0f},
	{ 0.0f, 0.0f, 1.0f, 0.0f},
	{ 0.0f, 0.0f, 0.0f, 0.0f},
};

void dce_set_mat_decode(float scale, float x, float y, float z) {
    DCE_MESHLET_MAT_DECODE[0][0] = scale;
    DCE_MESHLET_MAT_DECODE[1][1] = scale;
    DCE_MESHLET_MAT_DECODE[2][2] = scale;

    DCE_MESHLET_MAT_DECODE[3][0] = x;
    DCE_MESHLET_MAT_DECODE[3][1] = y;
    DCE_MESHLET_MAT_DECODE[3][2] = z;
}

void dce_set_mat_vertex_color(const rw::RGBAf* residual, const rw::RGBAf* material) {
	DCE_MESHLET_MAT_VERTEX_COLOR[0][0] = material->blue;
	DCE_MESHLET_MAT_VERTEX_COLOR[1][1] = material->green;
	DCE_MESHLET_MAT_VERTEX_COLOR[2][2] = material->red;

	DCE_MESHLET_MAT_VERTEX_COLOR[3][0] = 128 * material->blue + residual->blue;
	DCE_MESHLET_MAT_VERTEX_COLOR[3][1] = 128 * material->green + residual->green;
	DCE_MESHLET_MAT_VERTEX_COLOR[3][2] = 128 * material->red + residual->red;
	DCE_MESHLET_MAT_VERTEX_COLOR[3][3] = residual->alpha;
}

void DCE_MatrixViewport(float x, float y, float width, float height) {
    DCE_MAT_SCREENVIEW[0][0] = -width * 0.5f;
    DCE_MAT_SCREENVIEW[1][1] = height * 0.5f;
    DCE_MAT_SCREENVIEW[2][2] = 1;
    DCE_MAT_SCREENVIEW[3][0] = -DCE_MAT_SCREENVIEW[0][0] + x;
    DCE_MAT_SCREENVIEW[3][1] = VIDEO_MODE_HEIGHT - (DCE_MAT_SCREENVIEW[1][1] + y); 
}

void DCE_InitMatrices() {
	// Setup the screenview matrix.  Only need to do once since this matrix does not need to change for single player viewpoint.
	mat_identity();
	
	mat_store(&DCE_MAT_SCREENVIEW);
	
	DCE_MatrixViewport(0, 0, VIDEO_MODE_WIDTH, VIDEO_MODE_HEIGHT);
}

}


#define PLUGIN_ID 2

namespace rw {
namespace dc {
struct DcRaster
{
	uint32_t pvr_flags;
	pvr_ptr_t texaddr;
	uint32_t texoffs;
	#if defined(DC_TEXCONV)
	uint32_t texsize;
	#endif
	uint32_t pvr_id;
	int refs;
	bool native;
	int8_t u, v; // sizes
};

std::map<uint32_t, DcRaster*> cachedRasters;

struct DcRasterRef
{
	DcRaster* raster;
};

int32 nativeGeometryOffset;


struct DCModelDataHeader : rw::InstanceDataHeader
{
	uint32_t size;
	uint8_t data[0];
};

/* Lighting Structs and Defines */
#define MAX_LIGHTS 8

struct alignas(8) UniformObject
{
	float dir[MAX_LIGHTS/4][4][4];
	RGBAf col[MAX_LIGHTS];
	RGBAf        ambLight;
	int lightCount;
};

// N.B. PODs are zero initialized by placement new
// So we provide default ctors. We lose the POD status but win
// in perf for std::vector.

struct mesh_context_t {
	mesh_context_t() { }

	RGBA color;
	float32 ambient;
	float32 diffuse;
	size_t matfxContextOffset;

	uint32_t hdr_cmd;
	uint32_t hdr_mode1;
	uint32_t hdr_mode2;
	uint32_t hdr_mode3;
};

struct matfx_context_t {
	matfx_context_t() { }

	matrix_t mtx;
	float32 coefficient;

	uint32_t hdr_cmd;
	uint32_t hdr_mode1;
	uint32_t hdr_mode2;
	uint32_t hdr_mode3;
};

struct skin_context_t {
	skin_context_t() { }

	Matrix mtx;
};

static_assert(sizeof(skin_context_t) == sizeof(Matrix));

struct atomic_context_t {
	atomic_context_t() { }

	size_t meshContextOffset;
	size_t skinContextOffset;
	Atomic* atomic;
	Geometry* geo;
	Camera* cam;

	bool global_needsNoClip;
	bool skinMatrix0Identity;

	matrix_t worldView, mtx;
	UniformObject uniform;
};
/* END Ligting Structs and Defines */

/* TA Submission Functions */

// Macro for submitting pvr_vertex_t ( 32bit UV ) with perspective divide
__always_inline void DCE_RenderSubmitVertex(const pvr_vertex_t *v, uint32_t flags) {
    auto *sq  = reinterpret_cast<uint32_t *>(pvr_dr_target(drState));
    auto *src = reinterpret_cast<const uint32_t *>(v);
    float sz  = MATH_Fast_Invert(v->z);
    float sx  = v->x * sz;
    float sy  = v->y * sz;
    
    sq[0] = flags;
    sq[1] = *reinterpret_cast<uint32_t *>(&sx);
    sq[2] = *reinterpret_cast<uint32_t *>(&sy);
    sq[3] = *reinterpret_cast<uint32_t *>(&sz);
    sq[4] = src[4];
    sq[5] = src[5]; 
    sq[6] = src[6];
    
    pvr_dr_commit(sq);
}

static const float colorScaleCharToFloat = 1.0f / 255.0f / 255.0f;

inline float vec3f_dot_c(V3d * v, V3d * v2)
{
    return (v->x * v2->x) + (v->y * v2->y) + (v->z * v2->z);
}

__always_inline void DCE_RenderSubmitVertexIM3D(float x, float y, float w, 
                                                const float *uv, uint32_t argb, uint32_t flags)
{
    auto *sq   = reinterpret_cast<uint32_t *>(pvr_dr_target(drState));
    auto *uv32 = reinterpret_cast<const uint32_t *>(uv);
    float sz   = MATH_Fast_Invert(w);
    float sx   = x * sz;
    float sy   = y * sz;

    sq[0] = flags;
    sq[1] = *reinterpret_cast<uint32_t *>(&sx);
    sq[2] = *reinterpret_cast<uint32_t *>(&sy);
    sq[3] = *reinterpret_cast<uint32_t *>(&sz);
    sq[4] = uv32[0];
    sq[5] = uv32[1]; 
    sq[6] = argb;
    
    pvr_dr_commit(sq);
}

/* END TA Submission Functions*/




#if defined(DC_TEXCONV)
void malloc_stats() { }
#endif

#if 0
#define UNIMPL_LOG() printf("TODO: Implement %s @ %s:%d\n", __func__, __FILE__, __LINE__);
#define UNIMPL_LOGV(fmt, ...) printf("TODO: Implement %s @ %s:%d " fmt "\n", __func__, __FILE__, __LINE__, __VA_ARGS__);
#else
#define UNIMPL_LOG()
#define UNIMPL_LOGV(...)
#endif

void beginUpdate(Camera* cam)  {
	float view[16], proj[16];

	// View Matrix
	Matrix inv;
	Matrix::invert(&inv, cam->getFrame()->getLTM());
	// Since we're looking into positive Z,
	// flip X to ge a left handed view space.
	view[0]  = inv.right.x;
	view[1]  =  -inv.right.y;
	view[2]  =  inv.right.z;
	view[3]  =  0.0f;
	view[4]  = inv.up.x;
	view[5]  =  -inv.up.y;
	view[6]  =  inv.up.z;
	view[7]  =  0.0f;
	view[8]  =  inv.at.x;
	view[9]  =   -inv.at.y;
	view[10] =  inv.at.z;
	view[11] =  0.0f;
	view[12] = inv.pos.x;
	view[13] =  -inv.pos.y;
	view[14] =  inv.pos.z;
	view[15] =  1.0f;
	memcpy4(&cam->devView, view, sizeof(RawMatrix));
//	d3ddevice->SetTransform(D3DTS_VIEW, (D3DMATRIX*)view);

	// Projection Matrix
	float32 invwx = 1.0f/cam->viewWindow.x;
	float32 invwy = 1.0f/cam->viewWindow.y;
	float32 invz = 1.0f/(cam->farPlane-cam->nearPlane);

	proj[0] = invwx;
	proj[1] = 0.0f;
	proj[2] = 0.0f;
	proj[3] = 0.0f;

	proj[4] = 0.0f;
	proj[5] = invwy;
	proj[6] = 0.0f;
	proj[7] = 0.0f;

	proj[8] = cam->viewOffset.x*invwx;
	proj[9] = cam->viewOffset.y*invwy;
	proj[12] = -proj[8];
	proj[13] = -proj[9];
	if(cam->projection == Camera::PERSPECTIVE){
		proj[10] = cam->farPlane*invz;
		proj[11] = 1.0f;

		proj[15] = 0.0f;
	}else{
		proj[10] = invz;
		proj[11] = 0.0f;

		proj[15] = 1.0f;
	}
	proj[14] = -cam->nearPlane*proj[10];
	memcpy4(&cam->devProj, proj, sizeof(RawMatrix));
	
	mat_load((matrix_t*)&DCE_MAT_SCREENVIEW);
	mat_apply((matrix_t*)&cam->devProj);
	mat_store((matrix_t*)&cam->devProjScreen);

}


std::vector<atomic_context_t> atomicContexts;
std::vector<mesh_context_t> meshContexts;
std::vector<skin_context_t> skinContexts;
std::vector<matfx_context_t> matfxContexts;
std::vector<std::function<void()>> opCallbacks;
std::vector<std::function<void()>> blendCallbacks;
std::vector<std::function<void()>> ptCallbacks;

void dcMotionBlur_v1(uint8_t a, uint8_t r, uint8_t g, uint8_t b) {
	
	uint32_t col = (a << 24) | (r << 16) | (g << 8) | b;

	blendCallbacks.emplace_back([=]() {
		pvr_poly_cxt_t cxt;
			
		auto addr32b = PVR_GET(PVR_FB_ADDR);
		bool is_bank1 = !(addr32b & 4 * 1024 * 1024);
		auto addr64b = ((addr32b & ( 4 * 1024 * 1024 - 1)) * 2);
	#if !defined(DC_SIM)
		auto addr1 = (pvr_ptr_t)(((uintptr_t)PVR_RAM_BASE) + addr64b);
		auto addr2 = (pvr_ptr_t)(((uintptr_t)PVR_RAM_BASE) + addr64b + 640 * 2);
	#else
		auto addr1 = (pvr_ptr_t)&emu_vram[addr64b];
		auto addr2 = (pvr_ptr_t)&emu_vram[addr64b + 640 * 2];
	#endif



		PVR_SET(PVR_TEXTURE_MODULO, 640/32);

		auto doquad = [=](float x, float y, float w, float h, float tx, float ty, float tw, float th) {
			auto vtx = reinterpret_cast<pvr_vertex_t *>(pvr_dr_target(drState));
			vtx->flags = PVR_CMD_VERTEX;
			vtx->x = x;
			vtx->y = y;
			vtx->z = 1000000.0f;
			vtx->u = tx/1024.f;
			vtx->v = ty/512.f;
			vtx->argb = col;
			pvr_dr_commit(vtx);

			vtx = reinterpret_cast<pvr_vertex_t *>(pvr_dr_target(drState));
			vtx->flags = PVR_CMD_VERTEX;
			vtx->x = x+w;
			vtx->y = y;
			vtx->z = 1000000.0f;
			vtx->u = (tx+tw)/1024.f;
			vtx->v = ty/512.f;
			vtx->argb = col;
			pvr_dr_commit(vtx);

			vtx = reinterpret_cast<pvr_vertex_t *>(pvr_dr_target(drState));
			vtx->flags = PVR_CMD_VERTEX;
			vtx->x = x;
			vtx->y = y+h;
			vtx->z = 1000000.0f;
			vtx->u = tx/1024.f;
			vtx->v = (ty+th)/512.0f;
			vtx->argb = col;
			pvr_dr_commit(vtx);

			vtx = reinterpret_cast<pvr_vertex_t *>(pvr_dr_target(drState));
			vtx->flags = PVR_CMD_VERTEX_EOL;
			vtx->x = x+w;
			vtx->y = y+h;
			vtx->z = 1000000.0f;
			vtx->u = (tx+tw)/1024.f;
			vtx->v = (ty+th)/512.0f;
			vtx->argb = col;
			pvr_dr_commit(vtx);
		};
		{
			pvr_poly_cxt_txr(&cxt, 
				PVR_LIST_TR_POLY, 
				PVR_TXRFMT_RGB565 | PVR_TXRFMT_NONTWIDDLED | PVR_TXRFMT_STRIDE, 
				1024, 
				1024,
				addr1, 
				PVR_FILTER_NEAREST);
			
			cxt.depth.comparison = PVR_DEPTHCMP_ALWAYS;
			cxt.depth.write = PVR_DEPTHWRITE_DISABLE;

			cxt.blend.src = PVR_BLEND_SRCALPHA;
			cxt.blend.dst = PVR_BLEND_INVSRCALPHA;

			auto* hdr = reinterpret_cast<pvr_poly_hdr_t *>(pvr_dr_target(drState));
			pvr_poly_compile(hdr, &cxt);
			pvr_dr_commit(hdr);
		}
		for (int x = 0; x < 320; x+=2) {
			doquad(x, 0, 2, 480, x*2 + (is_bank1 ? 2 : 0), 0, 2, 480);
		}
		{
			pvr_poly_cxt_txr(&cxt, 
				PVR_LIST_TR_POLY, 
				PVR_TXRFMT_RGB565 | PVR_TXRFMT_NONTWIDDLED | PVR_TXRFMT_STRIDE, 
				1024, 
				1024,
				addr2, 
				PVR_FILTER_NEAREST);

			cxt.depth.comparison = PVR_DEPTHCMP_ALWAYS;
			cxt.depth.write = PVR_DEPTHWRITE_DISABLE;
			
			cxt.blend.src = PVR_BLEND_SRCALPHA;
			cxt.blend.dst = PVR_BLEND_INVSRCALPHA;

			auto* hdr = reinterpret_cast<pvr_poly_hdr_t *>(pvr_dr_target(drState));
			pvr_poly_compile(hdr, &cxt);
			pvr_dr_commit(hdr);
		}
		for (int x = 0; x < 320; x+=2) {
			doquad(320+x, 0, 2, 480, x*2 + (is_bank1 ? 2 : 0), 0, 2, 480);
		}
	});
}

void dcMotionBlur_v3(uint8_t a, uint8_t r, uint8_t g, uint8_t b) {
	uint32_t alpha = a;
	uint32_t col = (alpha * r / 255 << 16) | (alpha * g / 255 << 8) | alpha * b / 255;
	uint32_t mask_col = ((255 - a) << 16) | ((255 - a) << 8) | (255 - a);

	blendCallbacks.emplace_back([=]() {
		pvr_poly_cxt_t cxt;

		uint32_t addr32b = PVR_GET(PVR_FB_ADDR);
		pvr_ptr_t addr64b = (pvr_ptr_t)((addr32b & ( 4 * 1024 * 1024 - 1)) * 2);

		#if defined(DC_SIM)
		addr64b = (pvr_ptr_t)&emu_vram[(uintptr_t)addr64b];
		#endif

		auto doquad = [=](float x, float y, float z, float w, float h,
				  float umin, float umax, float vmin, float vmax, uint32_t col) {
			auto vtx = reinterpret_cast<pvr_vertex_t *>(pvr_dr_target(drState));
			vtx->flags = PVR_CMD_VERTEX;
			vtx->x = x;
			vtx->y = y;
			vtx->z = z;
			vtx->u = umin;
			vtx->v = vmin;
			vtx->argb = col;
			pvr_dr_commit(vtx);

			vtx = reinterpret_cast<pvr_vertex_t *>(pvr_dr_target(drState));
			vtx->flags = PVR_CMD_VERTEX;
			vtx->x = x+w;
			vtx->y = y;
			vtx->z = z;
			vtx->u = umax;
			vtx->v = vmin;
			vtx->argb = col;
			pvr_dr_commit(vtx);

			vtx = reinterpret_cast<pvr_vertex_t *>(pvr_dr_target(drState));
			vtx->flags = PVR_CMD_VERTEX;
			vtx->x = x;
			vtx->y = y+h;
			vtx->z = z;
			vtx->u = umin;
			vtx->v = vmax;
			vtx->argb = col;
			pvr_dr_commit(vtx);

			vtx = reinterpret_cast<pvr_vertex_t *>(pvr_dr_target(drState));
			vtx->flags = PVR_CMD_VERTEX_EOL;
			vtx->x = x+w;
			vtx->y = y+h;
			vtx->z = z;
			vtx->u = umax;
			vtx->v = vmax;
			vtx->argb = col;
			pvr_dr_commit(vtx);
		};

		pvr_poly_cxt_txr(&cxt, PVR_LIST_TR_POLY,
				 PVR_TXRFMT_ARGB1555, 8, 8, fake_tex,
				 PVR_FILTER_NONE);
		cxt.txr.env = PVR_TXRENV_REPLACE;
		cxt.txr.alpha = PVR_TXRALPHA_ENABLE;
		cxt.blend.src = PVR_BLEND_ONE;
		cxt.blend.dst = PVR_BLEND_ZERO;
		cxt.depth.write = PVR_DEPTHWRITE_DISABLE;
		cxt.depth.comparison = PVR_DEPTHCMP_ALWAYS;
		cxt.blend.dst_enable = PVR_BLEND_ENABLE;

		auto* hdr = reinterpret_cast<pvr_poly_hdr_t *>(pvr_dr_target(drState));
		pvr_poly_compile(hdr, &cxt);
		pvr_dr_commit(hdr);

		doquad(0.0f, 0.0f, 1e6f, 640.0f, 480.0f,
		       0.0f, 640.0f / 8.0f, 0.0f, 480.0f / 8.0f, mask_col);

		pvr_poly_cxt_txr(&cxt, PVR_LIST_TR_POLY,
				 PVR_TXRFMT_RGB565 | PVR_TXRFMT_NONTWIDDLED | PVR_TXRFMT_STRIDE,
				 1024, 1024, addr64b, PVR_FILTER_NONE);

		cxt.blend.src = PVR_BLEND_DESTALPHA;
		cxt.blend.dst = PVR_BLEND_ZERO;
		cxt.txr.alpha = PVR_TXRALPHA_DISABLE;
		cxt.txr.env = PVR_TXRENV_MODULATE;
		cxt.depth.write = PVR_DEPTHWRITE_DISABLE;
		cxt.depth.comparison = PVR_DEPTHCMP_ALWAYS;
		cxt.blend.dst_enable = PVR_BLEND_ENABLE;

		hdr = reinterpret_cast<pvr_poly_hdr_t *>(pvr_dr_target(drState));
		pvr_poly_compile(hdr, &cxt);
		pvr_dr_commit(hdr);

		doquad(0.0f, 0.0f, 2e6f, 320.0f, 480.0f,
		       0.0f, 640.0f / 1024.0f,
		       0.0f, 960.0f / 1024.0f, col);
		doquad(320.0f, 0.0f, 2e6f, 320.0f, 480.0f,
		       0.0f, 640.0f / 1024.0f,
		       1.0f / 1024.0f, 961.0f / 1024.0f, col);

		cxt.blend.src = PVR_BLEND_INVDESTALPHA;
		cxt.blend.dst = PVR_BLEND_ONE;

		hdr = reinterpret_cast<pvr_poly_hdr_t *>(pvr_dr_target(drState));
		pvr_poly_compile(hdr, &cxt);
		pvr_dr_commit(hdr);

		doquad(0.0f, 0.0f, 3e6f, 320.0f, 480.0f,
		       -1.0f / 1024.0f, 639.0f / 1024.0f,
		       0.0f, 960.0f / 1024.0f, col);
		doquad(320.0f, 0.0f, 3e6f, 320.0f, 480.0f,
		       -1.0f / 1024.0f, 639.0f / 1024.0f,
		       1.0f / 1024.0f, 961.0f / 1024.0f, col);

		pvr_poly_cxt_col(&cxt, PVR_LIST_TR_POLY);
		cxt.depth.write = PVR_DEPTHWRITE_DISABLE;
		cxt.depth.comparison = PVR_DEPTHCMP_ALWAYS;
		cxt.blend.dst_enable = PVR_BLEND_ENABLE;

		cxt.blend.src = PVR_BLEND_INVDESTCOLOR;
		cxt.blend.dst = PVR_BLEND_ZERO;

		hdr = reinterpret_cast<pvr_poly_hdr_t *>(pvr_dr_target(drState));
		pvr_poly_compile(hdr, &cxt);
		pvr_dr_commit(hdr);

		doquad(0.0f, 0.0f, 4e6f, 640.0f, 480.0f,
		       0.0f, 640.0f / 8.0f, 0.0f, 480.0f / 8.0f, 0xffffffff);

		cxt.blend.src = PVR_BLEND_ONE;
		cxt.blend.dst = PVR_BLEND_ONE;

		hdr = reinterpret_cast<pvr_poly_hdr_t *>(pvr_dr_target(drState));
		pvr_poly_compile(hdr, &cxt);
		pvr_dr_commit(hdr);

		doquad(0.0f, 0.0f, 4e6f, 640.0f, 480.0f,
		       0.0f, 640.0f / 8.0f, 0.0f, 480.0f / 8.0f, 0x000f0f0f);

		cxt.blend.src = PVR_BLEND_INVDESTCOLOR;
		cxt.blend.dst = PVR_BLEND_ZERO;

		hdr = reinterpret_cast<pvr_poly_hdr_t *>(pvr_dr_target(drState));
		pvr_poly_compile(hdr, &cxt);
		pvr_dr_commit(hdr);

		doquad(0.0f, 0.0f, 4e6f, 640.0f, 480.0f,
		       0.0f, 640.0f / 8.0f, 0.0f, 480.0f / 8.0f, 0xffffffff);

		cxt.blend.dst_enable = PVR_BLEND_DISABLE;
		cxt.blend.src_enable = PVR_BLEND_ENABLE;
		cxt.blend.src = PVR_BLEND_ONE;
		cxt.blend.dst = PVR_BLEND_ONE;

		hdr = reinterpret_cast<pvr_poly_hdr_t *>(pvr_dr_target(drState));
		pvr_poly_compile(hdr, &cxt);
		pvr_dr_commit(hdr);

		doquad(0.0f, 0.0f, 4e6f, 640.0f, 480.0f,
		       0.0f, 640.0f / 8.0f, 0.0f, 480.0f / 8.0f, 0xffffffff);
	});
}

void allocDefrag(int iterations);

void endUpdate(Camera*) {

	#if !defined(DC_SIM) && defined(SKIP_FRAMES)
	if (pvr_check_ready() >= 0)
	#endif
	{
		pvr_set_zclip(0.0f);
		pvr_wait_ready();
		pvr_scene_begin();
		pvr_dr_init(&drState);
		pvr_list_begin(PVR_LIST_OP_POLY);
		enter_oix();
		if (opCallbacks.size()) {
			for (auto&& cb: opCallbacks) {
				cb();
			}
		}
		pvr_list_finish();
		if (ptCallbacks.size()) {
			PVR_SET(0x11C, 128); // PT Alpha test value
			pvr_dr_init(&drState);
			pvr_list_begin(PVR_LIST_PT_POLY);
			for (auto&& cb: ptCallbacks) {
				cb();
			}
			pvr_list_finish();
		}
		if (blendCallbacks.size()) {
			pvr_dr_init(&drState);
			pvr_list_begin(PVR_LIST_TR_POLY);
			for (auto&& cb: blendCallbacks) {
				cb();
			}
			pvr_list_finish();
		}

#if !defined(DC_TEXCONV)
		VmuProfiler::getInstance()->updateVertexBufferUsage();
#endif
		// XXX not by definition the best place to do this XXX
		// We need to know that pvr isn't using textures before moving them
		// XXX This is also very slow right now XXX
		// allocDefrag(1);
		pvr_scene_finish();
		leave_oix();
	}
	opCallbacks.clear();
	ptCallbacks.clear();
	blendCallbacks.clear();
	atomicContexts.clear();
	meshContexts.clear();
	skinContexts.clear();
	matfxContexts.clear();
}

void clearCamera(Camera*,RGBA*,uint32) {
    UNIMPL_LOG();
}

void showRaster(Raster*,uint32) {
    UNIMPL_LOG();
}

static uint32 blendMap[] = {
	PVR_BLEND_ZERO,	// actually invalid
	PVR_BLEND_ZERO,
	PVR_BLEND_ONE,
	PVR_BLEND_INVDESTCOLOR, // should be SRCCOLOR
	PVR_BLEND_DESTCOLOR, // should be INVSRCCOLOR
	PVR_BLEND_SRCALPHA,
	PVR_BLEND_INVSRCALPHA,
	PVR_BLEND_DESTALPHA,
	PVR_BLEND_INVDESTALPHA,
	PVR_BLEND_DESTCOLOR,
	PVR_BLEND_INVDESTCOLOR,
	PVR_BLEND_SRCALPHA //SATurating should be
};

static int srcBlend, dstBlend, zFunction, zWrite;
static Texture::Addressing addressingU, addressingV;
static int blendEnabled;
static Raster* current_raster;
static bool doAlphaTest;

static uint8_t fogFuncPvr = PVR_FOG_DISABLE;
static uint32_t fogColor = 0;
static uint32 cullModePvr;

static inline unsigned pvrCullMode(uint32_t cullMode) {
	switch(cullMode) {
		case CULLNONE: return PVR_CULLING_SMALL;
		case CULLBACK: return PVR_CULLING_CW;
		default:       return PVR_CULLING_CCW;
	}
}

static void
setRenderState(int32 state, void *pvalue)
{
	uint32 value = (uint32)(uintptr)pvalue;
	switch(state){
	case TEXTURERASTER:
        current_raster = (Raster*)pvalue;
		if (current_raster && Raster::formatHasAlpha(current_raster->format)) {
        	blendEnabled |= 2;
		} else {
			blendEnabled &= ~2;
		}
		// UNIMPL_LOGV("Texture: %p", pvalue ? ((Raster*)pvalue)->pixels : pvalue);
		break;
    case CULLMODE:
        cullModePvr = pvrCullMode(value);
        break;
    case SRCBLEND:
        srcBlend = blendMap[value];
		break;
	case DESTBLEND:
        dstBlend = blendMap[value];
		break;
    case VERTEXALPHA:
		if (value) {
        	blendEnabled |= 1;
		} else {
			blendEnabled &= ~1;
		}
        // if (PVR_LIST_OP_POLY == currentList) {
        //     pvr_list_finish();
        //     currentList = PVR_LIST_TR_POLY;
        //     pvr_list_begin(PVR_LIST_TR_POLY);
        // }
        break;
	case ZTESTENABLE:
		if (value) {
			zFunction = PVR_DEPTHCMP_GEQUAL;
		} else {
			zFunction = PVR_DEPTHCMP_ALWAYS;
		}
		break;
	case ZWRITEENABLE:
		zWrite = value ? PVR_DEPTHWRITE_ENABLE : PVR_DEPTHWRITE_DISABLE;
		break;
	case TEXTUREADDRESS:
		addressingU = addressingV = (Texture::Addressing)value;
		break;
	case TEXTUREADDRESSU:
		addressingU = (Texture::Addressing)value;
		break;
	case TEXTUREADDRESSV:
		addressingV = (Texture::Addressing)value;
		break;
	case ALPHATESTREF:
		if (value >= 128) {
			doAlphaTest = true;
		} else {
			doAlphaTest = false;
		}
		break;
	// case TEXTUREFILTER:
	// 	setFilterMode(0, value);
	// 	break;
	// case VERTEXALPHA:
	// 	setVertexAlpha(value);
	// 	break;
	// case SRCBLEND:
	// 	if(rwStateCache.srcblend != value){
	// 		rwStateCache.srcblend = value;
	// 		setGlRenderState(RWGL_SRCBLEND, blendMap[rwStateCache.srcblend]);
	// 	}
	// 	break;
	// case DESTBLEND:
	// 	if(rwStateCache.destblend != value){
	// 		rwStateCache.destblend = value;
	// 		setGlRenderState(RWGL_DESTBLEND, blendMap[rwStateCache.destblend]);
	// 	}
	// 	break;
	// case ZTESTENABLE:
	// 	setDepthTest(value);
	// 	break;
	// case ZWRITEENABLE:
	// 	setDepthWrite(value);
	// 	break;
	case FOGENABLE:
	 	//Set Flag for Fog
	 	fogFuncPvr = value ? PVR_FOG_TABLE : PVR_FOG_DISABLE;
	 	break;
	 case FOGCOLOR:
        // Set fog color when state changes
        if(fogColor != value) {
            fogColor = value;
            RGBA c;
            c.red = value;
            c.green = value>>8;
            c.blue = value>>16;
            c.alpha = value>>24;
            pvr_fog_table_color(c.alpha / 255.0f, c.red / 255.0f, c.green  / 255.0f, c.blue  / 255.0f);
            pvr_fog_table_linear(50.0f, 450.0f);
        }
	 	break;
	// case CULLMODE:
	// 	if(rwStateCache.cullmode != value){
	// 		rwStateCache.cullmode = value;
	// 		if(rwStateCache.cullmode == CULLNONE)
	// 			setGlRenderState(RWGL_CULL, false);
	// 		else{
	// 			setGlRenderState(RWGL_CULL, true);
	// 			setGlRenderState(RWGL_CULLFACE, rwStateCache.cullmode == CULLBACK ? GL_BACK : GL_FRONT);
	// 		}
	// 	}
	// 	break;

	// case STENCILENABLE:
	// 	if(rwStateCache.stencilenable != value){
	// 		rwStateCache.stencilenable = value;
	// 		setGlRenderState(RWGL_STENCIL, value);
	// 	}
	// 	break;
	// case STENCILFAIL:
	// 	if(rwStateCache.stencilfail != value){
	// 		rwStateCache.stencilfail = value;
	// 		setGlRenderState(RWGL_STENCILFAIL, stencilOpMap[value]);
	// 	}
	// 	break;
	// case STENCILZFAIL:
	// 	if(rwStateCache.stencilzfail != value){
	// 		rwStateCache.stencilzfail = value;
	// 		setGlRenderState(RWGL_STENCILZFAIL, stencilOpMap[value]);
	// 	}
	// 	break;
	// case STENCILPASS:
	// 	if(rwStateCache.stencilpass != value){
	// 		rwStateCache.stencilpass = value;
	// 		setGlRenderState(RWGL_STENCILPASS, stencilOpMap[value]);
	// 	}
	// 	break;
	// case STENCILFUNCTION:
	// 	if(rwStateCache.stencilfunc != value){
	// 		rwStateCache.stencilfunc = value;
	// 		setGlRenderState(RWGL_STENCILFUNC, stencilFuncMap[value]);
	// 	}
	// 	break;
	// case STENCILFUNCTIONREF:
	// 	if(rwStateCache.stencilref != value){
	// 		rwStateCache.stencilref = value;
	// 		setGlRenderState(RWGL_STENCILREF, value);
	// 	}
	// 	break;
	// case STENCILFUNCTIONMASK:
	// 	if(rwStateCache.stencilmask != value){
	// 		rwStateCache.stencilmask = value;
	// 		setGlRenderState(RWGL_STENCILMASK, value);
	// 	}
	// 	break;
	// case STENCILFUNCTIONWRITEMASK:
	// 	if(rwStateCache.stencilwritemask != value){
	// 		rwStateCache.stencilwritemask = value;
	// 		setGlRenderState(RWGL_STENCILWRITEMASK, value);
	// 	}
	// 	break;

	// case ALPHATESTFUNC:
	// 	setAlphaTestFunction(value);
	// 	break;
	// case ALPHATESTREF:
	// 	if(alphaRef != value/255.0f){
	// 		alphaRef = value/255.0f;
	// 		uniformStateDirty[RWGL_ALPHAREF] = true;
	// 		stateDirty = 1;
	// 	}
	// 	break;
	// case GSALPHATEST:
	// 	rwStateCache.gsalpha = value;
	// 	break;
	// case GSALPHATESTREF:
	// 	rwStateCache.gsalpharef = value;
        break;
        default:
            UNIMPL_LOGV("state: %d", state);
	}

}

void  *getRenderState(int32) { 
    UNIMPL_LOG();
    return 0;
}

bool32 rasterRenderFast(Raster *raster, int32 x, int32 y) { 
    UNIMPL_LOG();
    return 0; 
}

void im2DRenderLine(void*, int32, int32, int32) {
    UNIMPL_LOG();
}

void im2DRenderTriangle(void*, int32, int32, int32, int32) {
    UNIMPL_LOG();
}

int32 pvrFormatForRaster(Raster *r) {
	// switch(r->format & 0xF00) {
	// 	case Raster::C4444: rv = PVR_TXRFMT_ARGB4444; break;
	// 	case Raster::C565: rv = PVR_TXRFMT_RGB565; break;
	// 	case Raster::C1555: rv = PVR_TXRFMT_ARGB1555; break;
	// 	default:
	// 	logf("pvrFormatForRaster format: %x\n", r->format & 0xF00);
	// 	assert(false && "pvrFormatForRaster format??");
	// }
	
	auto natras = GETDCRASTEREXT(r);
	return natras->raster->pvr_flags;
}

void pvrTexAddress(pvr_poly_cxt_t *cxt, Texture::Addressing u, Texture::Addressing v) {
	switch(u) {
		case Texture::WRAP: cxt->txr.uv_flip |= PVR_UVFLIP_NONE; break;
		case Texture::MIRROR: cxt->txr.uv_flip |= PVR_UVFLIP_U; break;
		case Texture::CLAMP: cxt->txr.uv_clamp |= PVR_UVCLAMP_U; break;
		default: assert(0 && "unknown address mode");
	}

	switch(v) {
		case Texture::WRAP: cxt->txr.uv_flip |= PVR_UVFLIP_NONE; break;
		case Texture::MIRROR: cxt->txr.uv_flip |= PVR_UVFLIP_V; break;
		case Texture::CLAMP: cxt->txr.uv_clamp |= PVR_UVCLAMP_V; break;
		default: assert(0 && "unknown address mode");
	}
}

pvr_ptr_t pvrTexturePointer(Raster *r) {
	auto natras = GETDCRASTEREXT(r);

	return (uint8_t*)natras->raster->texaddr - natras->raster->texoffs;
}

void im2DRenderPrimitive(PrimitiveType primType, void *vertices, int32_t numVertices) {
	auto *verts = reinterpret_cast<Im2DVertex *>(vertices);

	auto renderCB = 
		[=,
			current_raster = dc::current_raster,
			blend_enabled  = dc::blendEnabled,
			src_blend      = dc::srcBlend,
			dst_blend      = dc::dstBlend,
			z_function     = dc::zFunction,
			z_write        = dc::zWrite,
			cull_mode_pvr  = dc::cullModePvr,
			addressingU    = dc::addressingU,
			addressingV    = dc::addressingV,
            fog_func_pvr   = dc::fogFuncPvr]
		(const Im2DVertex* vtx) __attribute__((always_inline)) 
	{

		auto pvrHeaderSubmit = [=]() __attribute__((always_inline)) {
			pvr_poly_cxt_t cxt;

			if (current_raster) [[likely]] {
				pvr_poly_cxt_txr(&cxt, 
								PVR_LIST_TR_POLY, 
								pvrFormatForRaster(current_raster), 
								current_raster->width, 
								current_raster->height,
								pvrTexturePointer(current_raster), 
								PVR_FILTER_BILINEAR);
				pvrTexAddress(&cxt, addressingU, addressingV);
			} else { 
				pvr_poly_cxt_col(&cxt, PVR_LIST_TR_POLY);
			}

			if (blend_enabled) [[likely]] {
				cxt.blend.src = src_blend;
				cxt.blend.dst = dst_blend;
			} else {
				// non blended sprites are also submitted in TR lists
				// so we need to reset the blend mode
				cxt.blend.src = PVR_BLEND_ONE;
				cxt.blend.dst = PVR_BLEND_ZERO;
			}

			cxt.gen.culling      = cull_mode_pvr;
			cxt.depth.comparison = z_function;
			cxt.depth.write      = z_write;

			cxt.gen.fog_type = fog_func_pvr;

			auto* hdr = reinterpret_cast<pvr_poly_hdr_t *>(pvr_dr_target(drState));
			pvr_poly_compile(hdr, &cxt);
			pvr_dr_commit(hdr);
		};

		auto pvrVertexSubmit = [](const Im2DVertex &gtaVert, unsigned flags) 
		__attribute__((always_inline)) 
		{
			auto *pvrVert  = pvr_dr_target(drState); 
			pvrVert->flags = flags;
			pvrVert->x 	   = gtaVert.x;
			pvrVert->y	   = gtaVert.y;
			pvrVert->z 	   = MATH_Fast_Invert(gtaVert.w); // this is perfect for almost every case...
			pvrVert->u 	   = gtaVert.u;
			pvrVert->v 	   = gtaVert.v;
			pvrVert->argb  = (gtaVert.a << 24) |
							 (gtaVert.r << 16) |
							 (gtaVert.g <<  8) |
							 (gtaVert.b <<  0);
			pvr_dr_commit(pvrVert);
		};

		switch(primType) {
			case PRIMTYPETRILIST:
				pvrHeaderSubmit();
				dcache_pref_block(vtx);
				for(int i = 0; i < numVertices; i += 3) [[likely]] {
					dcache_pref_block(&vtx[i + 1]);
					pvrVertexSubmit(vtx[i + 0], PVR_CMD_VERTEX);
					dcache_pref_block(&vtx[i + 2]);
					pvrVertexSubmit(vtx[i + 1], PVR_CMD_VERTEX);
					dcache_pref_block(&vtx[i + 3]);
					pvrVertexSubmit(vtx[i + 2], PVR_CMD_VERTEX_EOL);
				}
			break;
			case PRIMTYPETRIFAN: {
				pvrHeaderSubmit();
				const auto *vtxA = vtx + 0;
				const auto *vtxB = vtx + 1;
				dcache_pref_block(vtxA);
				for(int i = 2; i < numVertices; ++i) [[likely]] {
					const auto *vtxC = vtx + i;
					dcache_pref_block(vtxB);
					pvrVertexSubmit(*vtxA, PVR_CMD_VERTEX);
					dcache_pref_block(vtxC);
					pvrVertexSubmit(*vtxB, PVR_CMD_VERTEX);
					dcache_pref_block(&vtx[i]);
					pvrVertexSubmit(*vtxC, PVR_CMD_VERTEX_EOL);
					vtxB = vtxC;
				}
			}
			break;
		default:
			UNIMPL_LOGV("primType: %d, vertices: %p, numVertices: %d", primType, vertices, numVertices);
		}
	};

	std::vector<Im2DVertex> vertData(verts, verts + numVertices);
	blendCallbacks.emplace_back([=, data = std::move(vertData)]() {
		renderCB(&data[0]);
	});
}

void im2DRenderIndexedPrimitive(PrimitiveType primType, void *vertices, int32 numVertices, void *indices, int32 numIndices) {
	auto idx = (unsigned short*)indices;
	auto vtx = (Im2DVertex*)vertices;

    std::vector<Im2DVertex> vertData(numIndices);

	for (int32 i = 0; i < numIndices; i++) {
		vertData[i] = vtx[idx[i]];
	}

	im2DRenderPrimitive(primType, &vertData[0], vertData.size());
}

static std::vector<Im3DVertex> im3dVertices; 
void im3DTransform(void *vertices, int32 numVertices, Matrix *worldMat, uint32 flags) {
    // UNIMPL_LOGV("start %d", numVertices);
    if(worldMat == nil){
		static Matrix ident;
		ident.setIdentity();
		worldMat = &ident;
	}
	
	rw::RawMatrix mtx, proj, world, worldview;
	rw::Camera *cam = engine->currentCamera;

	rw::convMatrix(&world, worldMat);
	rw::RawMatrix::mult(&worldview, &world, &cam->devView);
	rw::RawMatrix::mult(&proj, &worldview, &cam->devProj);
	rw::RawMatrix::mult(&mtx, &proj, (RawMatrix*)&DCE_MAT_SCREENVIEW);
	// mat_load(&DCE_MAT_SCREENVIEW);     // ~11 cycles.
	mat_load(( matrix_t*)&mtx.right);  // Number of cycles: ~32.
    im3dVertices.resize(numVertices);

    auto vtx = (Im3DVertex*)vertices;

    for (int32 i = 0; i < numVertices; i++) {
        im3dVertices[i] = vtx[i];

		#define vd im3dVertices[i].position
		#define vs vtx[i].position

		float W;
		mat_trans_nodiv_nomod(vs.x, vs.y, vs.z, vd.x, vd.y, W, vd.z); // store the undivided W value in the Z component
		
		#undef vs
		#undef vd
    }
}

void im3DRenderPrimitive(PrimitiveType primType) {
    UNIMPL_LOG();
}

#define VTXSUBMITIM3D(vtx, flags)  DCE_RenderSubmitVertexIM3D(vtx.position.x, vtx.position.y, vtx.position.z, &vtx.u, vtx.b | (vtx.g << 8) | (vtx.r << 16) | (vtx.a << 24), flags)	

void im3DRenderIndexedPrimitive(PrimitiveType primType, 
								void         *indices, 
								int32_t       numIndices) 
{
	auto renderCB = 
		[=,
		 current_raster = dc::current_raster,
		 cull_mode_pvr  = dc::cullModePvr,
		 src_blend      = dc::srcBlend,
		 dst_blend      = dc::dstBlend,
		 blend_enabled  = dc::blendEnabled,
		 z_function     = dc::zFunction,
		 z_write        = dc::zWrite,
		 addressingU    = dc::addressingU,
		 addressingV    = dc::addressingV,
		 fog_func_pvr   = dc::fogFuncPvr]
		 (const void* indices, const Im3DVertex *im3dVertices) __attribute__((always_inline)) 
		
	{
		auto pvrHeaderSubmit = [=]() __attribute__((always_inline)) {
			pvr_poly_cxt_t cxt;

			if (current_raster) [[likely]] {
				pvr_poly_cxt_txr(&cxt, 
								blendEnabled? PVR_LIST_TR_POLY : PVR_LIST_OP_POLY, 
								pvrFormatForRaster(current_raster), 
								current_raster->width, 
								current_raster->height,
								pvrTexturePointer(current_raster), 
								PVR_FILTER_BILINEAR);
				pvrTexAddress(&cxt, addressingU, addressingV);
			} else pvr_poly_cxt_col(&cxt, blendEnabled? PVR_LIST_TR_POLY : PVR_LIST_OP_POLY);		

			if (blend_enabled) [[likely]] {
				cxt.blend.src = src_blend;
				cxt.blend.dst = dst_blend;
			}

			cxt.gen.culling      = cull_mode_pvr;
			cxt.depth.comparison = z_function;
			cxt.depth.write      = z_write;


			cxt.gen.fog_type = fog_func_pvr;

			auto* hdr = reinterpret_cast<pvr_poly_hdr_t *>(pvr_dr_target(drState));
			pvr_poly_compile(hdr, &cxt);
			pvr_dr_commit(hdr);
		};

		auto pvrVertexSubmit_interp = [](const Im3DVertex &gv1, const Im3DVertex &gv2, unsigned flags) 
			__attribute__((always_inline)) 
		{
			// REMEMBER: undivided W is stored in the Z for gv1 & gv2

			// assuming near plane is 0.0f
			// gv1 is visible (posi), and gv2 is behind the plane (negative)
			float t = (1.0f - gv1.position.z) * MATH_Fast_Invert(gv2.position.z - gv1.position.z);

			pvr_vertex_t pvrVert; 

			pvrVert.flags  = flags;
			pvrVert.z	   = gv1.position.z + t * (gv2.position.z - gv1.position.z);
			pvrVert.x	   = (gv1.position.x + t * (gv2.position.x - gv1.position.x));
			pvrVert.y	   = (gv1.position.y + t * (gv2.position.y - gv1.position.y));
/*
			float16 pvrVert_u = gv1.u + t * (gv2.u - gv1.u);
			float16 pvrVert_v = gv1.v + t * (gv2.v - gv1.v);

			pvrVert.u = pvrVert_u.raw;
			pvrVert.v = pvrVert_v.raw;
*/
			float pvrVert_u = gv1.u + t * (gv2.u - gv1.u);
			float pvrVert_v = gv1.v + t * (gv2.v - gv1.v);

			pvrVert.u = pvrVert_u;
			pvrVert.v = pvrVert_v;
			
			// i realize this is The Dumb Way to do it
			// but i want as few differences between this and a known working vtx interp function for now
			uint32 gv1_argb = (gv1.a << 24) |
							  (gv1.r << 16) |
							  (gv1.g <<  8) |
							  (gv1.b <<  0);
			uint32 gv2_argb = (gv2.a << 24) |
							  (gv2.r << 16) |
							  (gv2.g <<  8) |
							  (gv2.b <<  0);
			for (int i = 0; i < 4; i++) {
				((uint8_t*)&pvrVert.argb)[i] = ((uint8_t*)&gv1_argb)[i] + t * (((uint8_t*)&gv2_argb)[i] - ((uint8_t*)&gv1_argb)[i]);
			}

			DCE_RenderSubmitVertex(&pvrVert, flags);
		};

		if(primType == PRIMTYPETRILIST) [[likely]] {
			const auto *idx = reinterpret_cast<const uint16 *>(indices);
			
			pvrHeaderSubmit();

			dcache_pref_block(idx);
			for (int32_t i = 0; i < numIndices; i += 3) [[likely]]{
				uint16_t idx0 = idx[i + 0];
				auto     vtx0 = im3dVertices[idx0];
				uint16_t idx1 = idx[i + 1];
				auto     vtx1 = im3dVertices[idx1];
				uint16_t idx2 = idx[i + 2];
				auto     vtx2 = im3dVertices[idx2];

				uint32_t vismask = 0;
				if(vtx0.position.z > 1.0f) vismask |= 0b100;
				vismask >>= 1;
				if(vtx1.position.z > 1.0f) vismask |= 0b100;
				vismask >>= 1;
				if(vtx2.position.z > 1.0f) vismask |= 0b100;

				if (vismask == 0) continue;

				if (vismask == 7) {
					VTXSUBMITIM3D(vtx0, PVR_CMD_VERTEX);
					VTXSUBMITIM3D(vtx1, PVR_CMD_VERTEX);
					VTXSUBMITIM3D(vtx2, PVR_CMD_VERTEX_EOL);
				}

				switch (vismask) {
					case 1: // 0 visible, 1 and 2 hidden
						VTXSUBMITIM3D(vtx0, PVR_CMD_VERTEX);
						pvrVertexSubmit_interp(vtx0, vtx1, PVR_CMD_VERTEX);
						pvrVertexSubmit_interp(vtx0, vtx2, PVR_CMD_VERTEX_EOL);
						break;
					case 2: // 0 hidden, 1 visible, 2 hidden
						pvrVertexSubmit_interp(vtx1, vtx0, PVR_CMD_VERTEX);
						VTXSUBMITIM3D(vtx1, PVR_CMD_VERTEX);
						pvrVertexSubmit_interp(vtx1, vtx2, PVR_CMD_VERTEX_EOL);
						break;
					case 3: // 0 and 1 visible, 2 hidden
						VTXSUBMITIM3D(vtx0, PVR_CMD_VERTEX);
						VTXSUBMITIM3D(vtx1, PVR_CMD_VERTEX);
						pvrVertexSubmit_interp(vtx1, vtx2, PVR_CMD_VERTEX_EOL);
						VTXSUBMITIM3D(vtx0, PVR_CMD_VERTEX);
						pvrVertexSubmit_interp(vtx1, vtx2, PVR_CMD_VERTEX);
						pvrVertexSubmit_interp(vtx0, vtx2, PVR_CMD_VERTEX_EOL);
						break;
					case 4: // 0 and 1 hidden, 2 visible
						VTXSUBMITIM3D(vtx2, PVR_CMD_VERTEX);
						pvrVertexSubmit_interp(vtx2, vtx0, PVR_CMD_VERTEX);
						pvrVertexSubmit_interp(vtx2, vtx1, PVR_CMD_VERTEX_EOL);
						break;
					case 5: // 0 visible, 1 hidden, 2 visible
						VTXSUBMITIM3D(vtx0, PVR_CMD_VERTEX);
						pvrVertexSubmit_interp(vtx0, vtx1, PVR_CMD_VERTEX);
						VTXSUBMITIM3D(vtx2, PVR_CMD_VERTEX_EOL);
						VTXSUBMITIM3D(vtx2, PVR_CMD_VERTEX);
						pvrVertexSubmit_interp(vtx0, vtx1, PVR_CMD_VERTEX);
						pvrVertexSubmit_interp(vtx2, vtx1, PVR_CMD_VERTEX_EOL);
						break;
					case 6: // 0 hidden, 1 and 2 visible
						VTXSUBMITIM3D(vtx1, PVR_CMD_VERTEX);
						VTXSUBMITIM3D(vtx2, PVR_CMD_VERTEX);
						pvrVertexSubmit_interp(vtx1, vtx0, PVR_CMD_VERTEX_EOL);
						VTXSUBMITIM3D(vtx2, PVR_CMD_VERTEX);
						pvrVertexSubmit_interp(vtx1, vtx0, PVR_CMD_VERTEX);
						pvrVertexSubmit_interp(vtx2, vtx0, PVR_CMD_VERTEX_EOL);
						break;
					default:
						break;
				}
			}
		} 
		else UNIMPL_LOGV("primType: %d", primType);
	};

	if (blendEnabled) {
		auto *idx = reinterpret_cast<uint16_t *>(indices);
		std::vector<uint16_t> indexBuffer(idx, idx + numIndices);
		blendCallbacks.emplace_back([=, 
								 data = std::move(indexBuffer), 
								 vtxData = im3dVertices](){
				renderCB(&data[0], &vtxData[0]);
		});
	} else {
		auto *idx = reinterpret_cast<uint16_t *>(indices);
		std::vector<uint16_t> indexBuffer(idx, idx + numIndices);
		opCallbacks.emplace_back([=, 
								 data = std::move(indexBuffer), 
								 vtxData = im3dVertices](){
				renderCB(&data[0], &vtxData[0]);
		});
	}

}

void im3DEnd(void) {
    // UNIMPL_LOG();
    im3dVertices.resize(0);
}

template<typename Vin, typename Vout>
void transform_kos(Vout& vd, const Vin& vs) {
	mat_trans_single3_nomod(vs.x, vs.y, vs.z, vd.x, vd.y, vd.z);
}

void addInterpolatedVertex(const pvr_vertex16_t& v1, const pvr_vertex16_t& v2, uint32_t flags = PVR_CMD_VERTEX) {
	float t = (- v1.z) / (v2.z - v1.z);
	// float t = (-v1.w - v1.z) / ((v2.z - v2.w) - (v1.z - v1.w));

	pvr_vertex_t v;
	v.flags = flags;

	// Interpolate the positions using the 1/w values
	v.x = v1.x + t * (v2.x - v1.x);
	v.y = v1.y + t * (v2.y - v1.y);
	v.z = v1.z + t * (v2.z - v1.z);

	float16 v1_u,v1_v,v2_u,v2_v;
	v1_u.raw = v1.u;
	v1_v.raw = v1.v;

	v2_u.raw = v2.u;
	v2_v.raw = v2.v;

	float16 v_u = v1_u + t *(v2_u - v1_u);
	float16 v_v = v1_v + t *(v2_v - v1_v);

	v.u = v_u.raw;
	v.v = v_v.raw;

	for (int i = 0; i < 4; i++) {
		((uint8_t*)&v.argb)[i] = ((uint8_t*)&v1.argb)[i] + t * (((uint8_t*)&v2.argb)[i] - ((uint8_t*)&v1.argb)[i]);
	}

	/*
	pvr_vertex_t* vd = pvr_dr_target(drState);
	memcpy(vd, &v, sizeof(pvr_vertex_t));
	pvr_dr_commit(vd);
	*/
}

struct MeshInfo {
	int16_t meshletCount;
	int16_t meshletOffset;
};
static_assert(sizeof(MeshInfo) == 4);

struct MeshletInfo {
	RwSphere boundingSphere;
	uint16_t flags;
	int8_t pad;
	int8_t vertexSize;
	uint16_t vertexCount;
	uint16_t indexCount;
	uint32_t vertexOffset;
	uint32_t indexOffset;
	uint32_t skinIndexOffset;
	uint32_t skinWeightOffset;
};
static_assert(sizeof(MeshletInfo) == 40); // or 32 if !skin


 inline __attribute__((always_inline))  void setLights(Atomic *atomic, WorldLights *lightData, UniformObject &uniformObject)
{
	int n = 0;

	uniformObject.ambLight = lightData->ambient;

	if (lightData->numDirectionals) {
		Matrix mat;
		Matrix matsrc = *atomic->getFrame()->getLTM();
		matsrc.pos = V3d {0,0,0};
		Matrix::invert(&mat, &matsrc);
		
		n = 0;
		for(int i = 0; i < lightData->numDirectionals && i < MAX_LIGHTS; i++){
			Light *l = lightData->directionals[i];
			uniformObject.col[n] = scale(l->color, 255);
			V3d at = l->getFrame()->getLTM()->at;

			V3d dir;

			V3d::transformVectors(&dir, &at, 1, &mat);

			uniformObject.dir[n>>2][0][n&3] = -dir.x / 127.0f;
			uniformObject.dir[n>>2][1][n&3] = -dir.y / 127.0f;
			uniformObject.dir[n>>2][2][n&3] = -dir.z / 127.0f;
			uniformObject.dir[n>>2][3][n&3] = 0;

			n++;
			if(n >= MAX_LIGHTS)
				goto out;
		}
	}

out:
	uniformObject.lightCount = n;
}


 inline __attribute__((always_inline)) void lightingCB(Atomic *atomic, UniformObject &uniformObject) 
{
	WorldLights lightData;
	Light *directionals[8];
	Light *locals[8];
	lightData.directionals = directionals;
	lightData.numDirectionals = 8;
	lightData.locals = locals;
	lightData.numLocals = 8;

	if(atomic->geometry->flags & rw::Geometry::LIGHT){
		((World*)engine->currentWorld)->enumerateLights(atomic, &lightData);
		assert(lightData.numLocals == 0);
		if((atomic->geometry->flags & rw::Geometry::NORMALS) == 0){
			// Get rid of lights that need normals when we don't have any
			lightData.numDirectionals = 0;
			lightData.numLocals = 0;
		}
		setLights(atomic, &lightData, uniformObject);
	}else{
		lightData.numDirectionals = 0;
		lightData.ambient.red = 0;
		lightData.ambient.green = 0;
		lightData.ambient.blue = 0;
		lightData.ambient.alpha = 0;

		setLights(atomic, &lightData, uniformObject);
	}
}

template<bool small_xyz, unsigned forClip>
__attribute__ ((noinline)) void tnlMeshletTransform(uint8_t* dst, const uint8_t* vertexData, uint32_t vertexCount, uint32_t vertexSize) {
	const uint8_t* next_vertex = vertexData;
	dcache_pref_block(vertexData);

	pvr_vertex64_t *sq  = (pvr_vertex64_t *)dst;

	do {
		auto vertex = next_vertex;
		next_vertex += vertexSize;

		float x, y, z, w;

		dcache_pref_block(vertex + 32);

		if (!small_xyz) {
			auto stripVert = reinterpret_cast<const V3d *>(vertex);

			mat_trans_nodiv_nomod(stripVert->x, stripVert->y, stripVert->z, x, y, z, w);

		} else {
			auto stripVert = reinterpret_cast<const int16_t *>(vertex);

			mat_trans_nodiv_nomod((float)stripVert[0], (float)stripVert[1], (float)stripVert[2], 
								x, y, z, w);
		}


		if (forClip) {
			sq->o_a = x;
			sq->o_r = y;
			sq->o_g = w;	
		}

		if (forClip == 1) { // not textured
			sq->o_b = z;
		} else if (forClip == 2) { // textured
			sq->tex_z = z;
		}

		w = frsqrt(w * w);

		sq->x = x * w;
		sq->y = y * w;
		sq->z = w;

		sq += 1;
	} while(--vertexCount != 0);
}

#if defined(DC_SH4)
template<>
__attribute__ ((noinline)) void tnlMeshletTransform<true, 0>(uint8_t* dst, const uint8_t* vertexData, uint32_t vertexCount, uint32_t vertexSize) {
	// small_xyz = true, forClip = false
	// %0 is dst (r4)
	// %1 is vertexData (r5)
	// %2 is vertexCount (r6)
	// %3 is vertexSize (r7)


	// xmtrx is already loaded into the FPU

#if 1 
	// easy, one thing at a time version
	// in the future, we want to have a few different vertices in flight to better mask
	// ftrv, fmul, fssra, fmul fmul stalls
	__asm__ __volatile__ (
		R"(
			pref @%[vtxData]

			add #16, %[dst]				! %[dst] is 12 bytes ahead, as we write back-to-front, also +4 to skip pcw

			mov %[vtxData], r0		! r0 is for pref
			add #32, r0				! mov has 0 cycle latency

			pref @r0				! prefetch next cache line, r0 is free after this
			mov %[vtxData], r1		! r1 is current_vertex

			mov.w @r1,r0			! read x
			add %[vtxSz], %[vtxData]		! advance next_vertex by vertex_size

			lds	r0,fpul				! load x to fpul
			mov.w @(2, r1),r0		! read y, not sure this will dual issue

			float fpul,fr0			! convert x to float
			lds	r0,fpul				! load y to fpul, not sure this will dual issue

			mov.w @(4, r1),r0		! read z vertex, r1 is free after this
			float	fpul,fr1		! convert y to float

			lds	r0,fpul				! load z to fpul
			! nop

			float	fpul,fr2		! convert z to float
			fldi1	fr3		    	! load 1.0f to fr3
			
			.align 2
			1:

				! we might get some stalls with ftrv here
				ftrv	xmtrx,fv0		! transform
				dt %[vtxCnt]

				mov %[vtxData], r0		! r0 is for pref
				add #32, r0				! mov has 0 cycle latency

				pref @r0				! prefetch next cache line, r0 is free after this
				mov %[vtxData], r1		! r1 is current_vertex

				mov.w @r1,r0			! read x
				add %[vtxSz], %[vtxData]		! advance next_vertex by vertex_size

				fmul	fr3, fr3		! w = w * w
				! nop

				lds	r0,fpul				! load x to fpul
				mov.w @(2, r1),r0		! read y, not sure this will dual issue

				float fpul,fr4			! convert x to float
				lds	r0,fpul				! load y to fpul, not sure this will dual issue

				fsrra	fr3				! w = 1.0f / sqrt(w)
				! nop

				mov.w @(4, r1),r0		! read z vertex, r1 is free after this
				float	fpul,fr5		! convert y to float

				lds	r0,fpul				! load z to fpul
				! nop

				fmul	fr3, fr1		! y = y * w
				fmov.s fr3, @-%[dst]	! write w

				fmul	fr3, fr0		! x = x * w
				! nop

				float	fpul,fr2		! convert z to float
				fldi1	fr3		    	! load 1.0f to fr3

				fmov.s fr1, @-%[dst]	! write y
				! nop

				fmov fr5, fr1
				! nop

				fmov.s fr0, @-%[dst]	! write x
				! nop

				fmov fr4, fr0
				! nop

				bf.s 1b
				add #76, %[dst]			! increment %0 by 64+12=76 bytes ahead, as we write back-to-front
		)"
		: [dst] "+r" (dst), [vtxData] "+r" (vertexData), [vtxCnt] "+r" (vertexCount), [vtxSz] "+r" (vertexSize) 
		:
		: "r0", "r1", "fr0", "fr1", "fr2", "fr3", "fr4", "fr5", "memory"
	);
#endif
}
#endif

float16 decode_compessed_uv[256] = {
	-128 / 127.f, -127 / 127.f, -126 / 127.f, -125 / 127.f, -124 / 127.f, -123 / 127.f, -122 / 127.f, -121 / 127.f, -120 / 127.f, -119 / 127.f, -118 / 127.f, -117 / 127.f, -116 / 127.f, -115 / 127.f, -114 / 127.f, -113 / 127.f,
	-112 / 127.f, -111 / 127.f, -110 / 127.f, -109 / 127.f, -108 / 127.f, -107 / 127.f, -106 / 127.f, -105 / 127.f, -104 / 127.f, -103 / 127.f, -102 / 127.f, -101 / 127.f, -100 / 127.f, -99 / 127.f, -98 / 127.f, -97 / 127.f,
	-96 / 127.f, -95 / 127.f, -94 / 127.f, -93 / 127.f, -92 / 127.f, -91 / 127.f, -90 / 127.f, -89 / 127.f, -88 / 127.f, -87 / 127.f, -86 / 127.f, -85 / 127.f, -84 / 127.f, -83 / 127.f, -82 / 127.f, -81 / 127.f,
	-80 / 127.f, -79 / 127.f, -78 / 127.f, -77 / 127.f, -76 / 127.f, -75 / 127.f, -74 / 127.f, -73 / 127.f, -72 / 127.f, -71 / 127.f, -70 / 127.f, -69 / 127.f, -68 / 127.f, -67 / 127.f, -66 / 127.f, -65 / 127.f,
	-64 / 127.f, -63 / 127.f, -62 / 127.f, -61 / 127.f, -60 / 127.f, -59 / 127.f, -58 / 127.f, -57 / 127.f, -56 / 127.f, -55 / 127.f, -54 / 127.f, -53 / 127.f, -52 / 127.f, -51 / 127.f, -50 / 127.f, -49 / 127.f,
	-48 / 127.f, -47 / 127.f, -46 / 127.f, -45 / 127.f, -44 / 127.f, -43 / 127.f, -42 / 127.f, -41 / 127.f, -40 / 127.f, -39 / 127.f, -38 / 127.f, -37 / 127.f, -36 / 127.f, -35 / 127.f, -34 / 127.f, -33 / 127.f,
	-32 / 127.f, -31 / 127.f, -30 / 127.f, -29 / 127.f, -28 / 127.f, -27 / 127.f, -26 / 127.f, -25 / 127.f, -24 / 127.f, -23 / 127.f, -22 / 127.f, -21 / 127.f, -20 / 127.f, -19 / 127.f, -18 / 127.f, -17 / 127.f,
	-16 / 127.f, -15 / 127.f, -14 / 127.f, -13 / 127.f, -12 / 127.f, -11 / 127.f, -10 / 127.f, -9 / 127.f, -8 / 127.f, -7 / 127.f, -6 / 127.f, -5 / 127.f, -4 / 127.f, -3 / 127.f, -2 / 127.f, -1 / 127.f,

	0 / 127.f, 1 / 127.f, 2 / 127.f, 3 / 127.f, 4 / 127.f, 5 / 127.f, 6 / 127.f, 7 / 127.f, 8 / 127.f, 9 / 127.f, 10 / 127.f, 11 / 127.f, 12 / 127.f, 13 / 127.f, 14 / 127.f, 15 / 127.f,
	16 / 127.f, 17 / 127.f, 18 / 127.f, 19 / 127.f, 20 / 127.f, 21 / 127.f, 22 / 127.f, 23 / 127.f, 24 / 127.f, 25 / 127.f, 26 / 127.f, 27 / 127.f, 28 / 127.f, 29 / 127.f, 30 / 127.f, 31 / 127.f,
	32 / 127.f, 33 / 127.f, 34 / 127.f, 35 / 127.f, 36 / 127.f, 37 / 127.f, 38 / 127.f, 39 / 127.f, 40 / 127.f, 41 / 127.f, 42 / 127.f, 43 / 127.f, 44 / 127.f, 45 / 127.f, 46 / 127.f, 47 / 127.f,
	48 / 127.f, 49 / 127.f, 50 / 127.f, 51 / 127.f, 52 / 127.f, 53 / 127.f, 54 / 127.f, 55 / 127.f, 56 / 127.f, 57 / 127.f, 58 / 127.f, 59 / 127.f, 60 / 127.f, 61 / 127.f, 62 / 127.f, 63 / 127.f,
	64 / 127.f, 65 / 127.f, 66 / 127.f, 67 / 127.f, 68 / 127.f, 69 / 127.f, 70 / 127.f, 71 / 127.f, 72 / 127.f, 73 / 127.f, 74 / 127.f, 75 / 127.f, 76 / 127.f, 77 / 127.f, 78 / 127.f, 79 / 127.f,
	80 / 127.f, 81 / 127.f, 82 / 127.f, 83 / 127.f, 84 / 127.f, 85 / 127.f, 86 / 127.f, 87 / 127.f, 88 / 127.f, 89 / 127.f, 90 / 127.f, 91 / 127.f, 92 / 127.f, 93 / 127.f, 94 / 127.f, 95 / 127.f,
	96 / 127.f, 97 / 127.f, 98 / 127.f, 99 / 127.f, 100 / 127.f, 101 / 127.f, 102 / 127.f, 103 / 127.f, 104 / 127.f, 105 / 127.f, 106 / 127.f, 107 / 127.f, 108 / 127.f, 109 / 127.f, 110 / 127.f, 111 / 127.f,
	112 / 127.f, 113 / 127.f, 114 / 127.f, 115 / 127.f, 116 / 127.f, 117 / 127.f, 118 / 127.f, 119 / 127.f, 120 / 127.f, 121 / 127.f, 122 / 127.f, 123 / 127.f, 124 / 127.f, 125 / 127.f, 126 / 127.f, 127 / 127.f
};

// decompressing via table may kick out some source verts from cache, but we can't do much about it
// it is still faster
template<bool small_uv>
__attribute__ ((noinline)) void tnlMeshletCopyUVs(uint8_t* dst, const uint8_t* uvData, uint32_t vertexCount, uint32_t vertexSize) {
	const uint8_t* next_vertex = uvData;

	pvr_vertex64_t *sq  = (pvr_vertex64_t *)dst;
	auto decompress_table = decode_compessed_uv + 128; // bias so we can do signed loads

	do {
		auto vertex = next_vertex;
		next_vertex += vertexSize;

		if (!small_uv) {
			sq->uv = *(const uint32_t*)vertex;
		} else {
			sq->u = decompress_table[*(const int8_t*)vertex++].raw;
			sq->v = decompress_table[*(const int8_t*)vertex].raw;
		}

		sq += 1;
	} while(--vertexCount != 0);
}

__attribute__ ((noinline)) void tnlMeshletFillResidual(uint8_t* dstCol, uint32_t vertexCount, const RGBAf *residual) {
	do {
		float *colors = (float*)dstCol;

		*colors++ = residual->alpha;
		*colors++ = residual->red;
		*colors++ = residual->green;
		*colors++ = residual->blue;

		dstCol += 64;
	} while(--vertexCount != 0);
}

#if !defined(DC_SH4)
__attribute__ ((noinline)) void tnlMeshletVertexColor(uint8_t* dstCol, const int8_t* colData, uint32_t vertexCount, uint32_t vertexSize) {
	const int8_t* next_vertex = colData;
	// should be already in cache
	// dcache_pref_block(next_vertex);

	auto vertex = next_vertex;
	next_vertex += vertexSize;
	float cB = *vertex++;
	float cG = *vertex++;
	float cR = *vertex++;
	float cA;

	vertexCount--;

	dstCol += 4 * sizeof(float);
	do {
		vertex = next_vertex;
		// should be alraedy in cache
		// dcache_pref_block(vertex + 32);

		next_vertex += vertexSize;

		float* cols = (float*)dstCol;
		
		mat_trans_nodiv_nomod(cB, cG, cR, cB, cG, cR, cA);

		*--cols = cB;
		cB = *vertex++;
		*--cols = cG;
		cG = *vertex++;
		*--cols = cR;
		cR = *vertex++;
		*--cols = cA;
		
		dstCol += 64;
	} while(--vertexCount != 0);

	float* cols = (float*)dstCol;

	mat_trans_nodiv_nomod(cB, cG, cR, cB, cG, cR, cA);

	*--cols = cB;
	*--cols = cG;
	*--cols = cR;
	*--cols = cA;
}
#else
__attribute__ ((noinline)) void tnlMeshletVertexColor(uint8_t* dstCol, const int8_t* colData, uint32_t vertexCount, uint32_t vertexSize) {
	const int8_t* next_vertex = colData;

	dstCol += 4 * sizeof(float);

	__asm__ __volatile__ (
		R"(
			mov %[next_vertex], r0
			add %[vertexSize], %[next_vertex]
			dt %[vertexCount]

			! preload B
			mov.b @r0+, r2
			lds r2, fpul
			float fpul, fr0

			! preload G
			mov.b @r0+, r2
			lds r2, fpul
			float fpul, fr1

			! preload R
			mov.b @r0+, r2
			lds r2, fpul
			float fpul, fr2

			fldi1 fr3
			
			.align 2
			1:
				ftrv xmtrx, fv0
				mov %[next_vertex], r0

				mov.b @r0+, r2
				mov %[dstCol], r1
				
				mov.b @r0+, r3
				add #64, %[dstCol]

				dt %[vertexCount]
				mov.b @r0+, r4

				lds r2, fpul
				add %[vertexSize], %[next_vertex]

				float fpul, fr4
				lds r3, fpul

				float fpul, fr5
				fmov.s fr0, @-r1

				lds r4, fpul
				! nop

				fldi1 fr7
				! nop

				fmov.s fr1, @-r1
				float fpul, fr6

				fmov.s fr2, @-r1
				bt/s 2f
				fmov.s fr3, @-r1 ! store A

				ftrv xmtrx, fv4
				mov %[next_vertex], r0

				mov.b @r0+, r2
				mov %[dstCol], r1
				
				mov.b @r0+, r3
				add #64, %[dstCol]

				dt %[vertexCount]
				mov.b @r0+, r4

				lds r2, fpul
				add %[vertexSize], %[next_vertex]

				float fpul, fr0
				lds r3, fpul

				float fpul, fr1
				fmov.s fr4, @-r1

				lds r4, fpul
				! nop

				fldi1 fr3
				! nop

				fmov.s fr5, @-r1
				float fpul, fr2

				fmov.s fr6, @-r1
				bf/s 1b
				fmov.s fr7, @-r1 ! store A
			
			! do final vertex, fv0

			ftrv xmtrx, fv0

			fmov.s fr0, @-%[dstCol]
			fmov.s fr1, @-%[dstCol]
			fmov.s fr2, @-%[dstCol]

			bra 3f
			fmov.s fr3, @-%[dstCol] ! delay slot

			2:
			! do final vertex, fv4
			ftrv xmtrx, fv4

			fmov.s fr4, @-%[dstCol]
			fmov.s fr5, @-%[dstCol]
			fmov.s fr6, @-%[dstCol]
			fmov.s fr7, @-%[dstCol]

			3:

		)"
		: [dstCol] "+r" (dstCol), [next_vertex] "+r" (next_vertex), [vertexCount] "+r" (vertexCount), [vertexSize] "+r" (vertexSize) 
		:
		: "r0", "r1", "r2", "r3", "r4", "fr0", "fr1", "fr2", "fr3", "fr4", "fr5", "fr6", "fr7", "memory"
	);
}
#endif


template <unsigned cntDiffuse, bool floatNormals>
__attribute__ ((noinline)) void tnlMeshletDiffuseColor(uint8_t* dstCol, const uint8_t* normalData, uint32_t vertexCount, uint32_t vertexSize, const RGBAf *lights) {
	
	const uint8_t* next_vertex = normalData;
	dcache_pref_block(next_vertex);

	const float light1R = lights[0].red;
	const float light1G = lights[0].green;
	const float light1B = lights[0].blue;

	const float light2R = lights[1].red;
	const float light2G = lights[1].green;
	const float light2B = lights[1].blue;

	const float light3R = lights[2].red;
	const float light3G = lights[2].green;
	const float light3B = lights[2].blue;

	const float light4R = lights[3].red;
	const float light4G = lights[3].green;
	const float light4B = lights[3].blue;

	dstCol += 1 * sizeof(float); // skip alpha, write rgb

	do {
		auto vertex = next_vertex;
		dcache_pref_block(vertex + 32);
		next_vertex += vertexSize;

		const int8_t* inxyz = (const int8_t*)vertex;

		V3d normal;
		if (!floatNormals) {
			normal = { static_cast<float32>(inxyz[0]), static_cast<float32>(inxyz[1]), static_cast<float32>(inxyz[2])};
		} else {
			normal = *(V3d*)inxyz;
		}
		
		float light1, light2, light3, light4;
		mat_trans_nodiv_nomod_zerow(normal.x, normal.y, normal.z, light1, light2, light3, light4);

		float dR = 0, dG = 0, dB = 0;

		if (light1 > 0) {
			dR += light1 * light1R;
			dG += light1 * light1G;
			dB += light1 * light1B;
		}

		if (cntDiffuse > 1 && light2 > 0) {
			dR += light2 * light2R;
			dG += light2 * light2G;
			dB += light2 * light2B;
		}

		if (cntDiffuse > 2 && light3 > 0) {
			dR += light3 * light3R;
			dG += light3 * light3G;
			dB += light3 * light3B;
		}

		if (cntDiffuse > 3 && light4 > 0) {
			dR += light4 * light4R;
			dG += light4 * light4G;
			dB += light4 * light4B;
		}

		float *cols = (float*)dstCol;

		*cols++ += dR;
		*cols++ += dG;
		*cols++ += dB;

		dstCol += 64;
	} while(--vertexCount != 0);
}

template<bool textured>
__attribute__ ((noinline)) void submitMeshlet(uint8_t* OCR, const int8_t* indexData, uint32_t indexCount) {
	struct SQBUF {
		union {
			uint32_t flags;
			uint64_t data[4];
			uint8_t data8[32];
		};
	};

	static_assert(sizeof(SQBUF) == 32);
 
	do {
		auto idx = *indexData++;
		auto flags = idx & 0x80 ? PVR_CMD_VERTEX_EOL : PVR_CMD_VERTEX;
		auto lookup_idx = idx & 0x7F;

		auto src = (SQBUF*)(OCR +  lookup_idx * 64);
		src[0].flags = flags;
		FLUSH_TA_DATA(src);
		if (textured) {
			src[1].data8[31] = 0;
			FLUSH_TA_DATA(src+1);
		}
	} while(--indexCount);
}


#if defined(DC_SH4)
template<>
__attribute__ ((noinline)) void submitMeshlet<true>(uint8_t* OCR, const int8_t* indexData, uint32_t indexCount) {

	__asm__ __volatile__ (
		R"(
			mov %[idxData], r0
			mov %[idxCnt], r1

			add #31, r1
			shlr2 r1
			shlr2 r1
			shlr r1

			.align 2
			1:
				dt r1
				pref @r0
				bf/s 1b
				add #32, r0


			mov.b @%[idxData]+, r0
			.align 2
			1:
				mov %[cmdVtx], r1

				cmp/pz r0
				and #0x7F, r0

				bt 2f
				mov %[cmdVtxEol], r1
				2:

				shll8 r0
				! nop

				shlr2 r0
				mov.b @%[idxData]+, r2

				add %[ocr], r0
				! nop

				mov.l r1, @r0

				ocbwb @r0
				! nop

				dt %[idxCnt]
				! nop

				add #32, r0
				! nop

				! nop
				! nop

				! nop
				! nop

				! nop
				! nop
				mov.w r0, @(30, r0)

				ocbwb @r0

				bf.s 1b
				mov r2, r0
		)"
		: [idxData] "+r" (indexData), [idxCnt] "+r" (indexCount)
		: [ocr] "r" (OCR), [cmdVtx] "r" (PVR_CMD_VERTEX), [cmdVtxEol] "r" (PVR_CMD_VERTEX_EOL) 
		: "r0", "r1", "r2", "memory"
	);
}
#endif

// 8 kb in total
#if defined(DC_SH4)
uint8_t* OCR_SPACE = (uint8_t*)0x92000000;
#else
uint8_t OCR_SPACE[32 * 256] __attribute__((aligned(32)));
#endif
template<bool hasTexture>
void* interpolateAndSubmit(void* dst, const void* src1, const void* src2, uint32_t flags)  {
	auto v = (pvr_vertex64_t1 *)dst;
	auto v1 = (const pvr_vertex64_t *)src1;
	auto v2 = (const pvr_vertex64_t *)src2;

	v->flags = flags;

	// assuming near plane is 0.0f
	// v1 is visible (posi), and v2 is behind the plane (negative)
	// z is w here
	// float t = fclamp0_1((1.0f - v1->o_g) / (v2->o_g - v1->o_g));
	float SA = (hasTexture?v1->tex_z : v1->o_b) + v1->o_g;
	float SB = (hasTexture?v2->tex_z : v2->o_b) + v2->o_g;
	float t  = SA / (SA - SB);

	float x = v1->o_a + t * (v2->o_a - v1->o_a);
	float y = v1->o_r + t * (v2->o_r - v1->o_r);
	float w = v1->o_g + t * (v2->o_g - v1->o_g);

	w = frsqrt(w * w);

	v->x = x * w;
	v->y = y * w;
	v->z = w;

	if (hasTexture) {
		float16 v1_u,v1_v,v2_u,v2_v;
		v1_u.raw = v1->u;
		v1_v.raw = v1->v;

		v2_u.raw = v2->u;
		v2_v.raw = v2->v;

		float16 v_u = v1_u + t * (v2_u - v1_u);
		float16 v_v = v1_v + t * (v2_v - v1_v);

		v->u = v_u.raw;
		v->v = v_v.raw;

		pvr_dr_commit(v);
		v ++;
	}

	auto sq2 = (pvr_vertex64_t2 *)v;
	auto sq3 = (pvr_vertex32_ut *)v;

	float v1_a, v1_r, v1_g, v1_b, v2_a, v2_r, v2_g, v2_b;

	if (hasTexture) {
		auto v1t = (const pvr_vertex64_t *)v1;
		v1_a = v1t->a;
		v1_r = v1t->r;
		v1_g = v1t->g;
		v1_b = v1t->b;

		auto v2t = (const pvr_vertex64_t *)v2;
		v2_a = v2t->a;
		v2_r = v2t->r;
		v2_g = v2t->g;
		v2_b = v2t->b;
	} else {
		auto v1t = (const pvr_vertex32_ut *)v1;
		v1_a = v1t->a;
		v1_r = v1t->r;
		v1_g = v1t->g;
		v1_b = v1t->b;

		auto v2t = (const pvr_vertex32_ut *)v2;
		v2_a = v2t->a;
		v2_r = v2t->r;
		v2_g = v2t->g;
		v2_b = v2t->b;
	}

	(hasTexture ? sq2->a : sq3->a) = v1_a + t * (v2_a - v1_a);
	(hasTexture ? sq2->r : sq3->r) = v1_r + t * (v2_r - v1_r);
	(hasTexture ? sq2->g : sq3->g) = v1_g + t * (v2_g - v1_g);
	(hasTexture ? sq2->b : sq3->b) = v1_b + t * (v2_b - v1_b);

	pvr_dr_commit(hasTexture ? (void*)sq2 : (void*)sq3);

	v++;
	return v;
}

template<bool textured>
__attribute__ ((noinline)) void clipAndsubmitMeshlet(uint8_t* vertexData, const int8_t* indexData, uint32_t indexCount) {

	struct SQBUF {
		union {
			uint32_t flags;
			uint64_t data[4];
			uint8_t data8[32];
		};
	};

	static_assert(sizeof(SQBUF) == 32);

	SQBUF* sq = (SQBUF*)pvr_dr_target(drState);

	constexpr int8_t VERTEX = 0;
	constexpr int8_t VERTEX_EOL = 0x80;
	
	#define FILLVERT(n) \
		do { \
			auto idx = *indexData++; \
			auto local_idx = idx & 0x7f; \
			eol_now = idx & 0x80; \
			auto local_ptr = (vertexData + local_idx * 64); \
			vpp[n] = local_ptr; \
			auto v = (const pvr_vertex64_t*)local_ptr; \
			vismask >>= 1; \
			if((textured?v->tex_z:v->o_b) >= -v->o_g) vismask |= 0b100;	\
			indexCount--; \
			currentCount++; \
		} while(0)

	#define SUBMIT_VTX(vid, eolf) \
		do { \
			auto src = (SQBUF*) vpp[vid]; \
			src[0].flags = eolf ? PVR_CMD_VERTEX_EOL : PVR_CMD_VERTEX; \
			FLUSH_TA_DATA(src); \
			if (textured) { \
				src[1].data8[31] = 0; \
				FLUSH_TA_DATA(src + 1); \
			} \
		} while(0)

	#define SUBMIT_INTERPOLATE(vid1, vid2, eolf) \
		do { \
			sq = (SQBUF*)interpolateAndSubmit<textured>(sq, vpp[vid1], vpp[vid2], eolf ? PVR_CMD_VERTEX_EOL : PVR_CMD_VERTEX); \
		} while(0)

	uint32_t vismask = 0;

	uint8_t* vpp[3];

	int8_t eol = 0;
	int8_t eol_now = 0;

	do {
		uint32_t currentCount = -1;

		FILLVERT(0);
		FILLVERT(1);
		FILLVERT(2);

		if (vismask & 1) {
			SUBMIT_VTX(0, VERTEX);
			if (vismask & 2) {
				// both first verts visible
				SUBMIT_VTX(1, VERTEX);
			} else {
				// 0 visible, 1 hidden
				SUBMIT_INTERPOLATE(0, 1, VERTEX);
			}
		} else if (vismask & 2) {
			// 0 hidden, 1 visible
			SUBMIT_INTERPOLATE(1, 0, VERTEX);
			SUBMIT_VTX(1, VERTEX);
		}

		eol = 0;
		// each remaining vertex of the strip
		while(!eol) {
			// "ring buffery" indices
			uint8_t vertZeroIdx = (currentCount - 2) % 3;
			uint8_t vertOneIdx = (currentCount - 1) % 3;
			uint8_t vertTwoIdx = currentCount % 3;
			//dcache_pref_block(&vph[vertZeroIdx]); not sure where to put this honestly -jaxyn

			eol = eol_now;

			if (!vismask) {
				if (!eol) {
					// "ring buffery" filling
					FILLVERT(vertZeroIdx);
				}
				continue;
			}

			if (vismask == 7) {
				// all visible
				SUBMIT_VTX(vertTwoIdx, eol);
				if (!eol) {
					// "ring buffery" filling
					FILLVERT(vertZeroIdx);
				}
				continue;
			}

			switch (vismask) {
				case 1: // 0 visible, 1 and 2 hidden
					// pause strip
					SUBMIT_INTERPOLATE(vertZeroIdx, vertTwoIdx, VERTEX_EOL);
					break;
				case 3: // 0 and 1 visible, 2 hidden
					SUBMIT_INTERPOLATE(vertZeroIdx, vertTwoIdx, VERTEX);
					SUBMIT_VTX(vertOneIdx, VERTEX);
				case 2: // 0 hidden, 1 visible, 2 hidden
					SUBMIT_INTERPOLATE(vertOneIdx, vertTwoIdx, eol);
					break;
				case 4: // 0 and 1 hidden, 2 visible
					SUBMIT_INTERPOLATE(vertTwoIdx, vertZeroIdx, VERTEX);
					if (currentCount & 0x01) { // flip directionality
				case 5: // 0 visible, 1 hidden, 2 visible
						SUBMIT_VTX(vertTwoIdx, VERTEX);
					}
					SUBMIT_INTERPOLATE(vertTwoIdx, vertOneIdx, VERTEX);
					SUBMIT_VTX(vertTwoIdx, eol);
					break;
				case 6: // 0 hidden, 1 and 2 visible
					SUBMIT_INTERPOLATE(vertTwoIdx, vertZeroIdx, VERTEX);
					SUBMIT_VTX(vertOneIdx, VERTEX);
					SUBMIT_VTX(vertTwoIdx, eol);
					break;
				default:
					break;
			}

			if (!eol) {
				// "ring buffery" filling
				FILLVERT(vertZeroIdx);
			}
		};
	} while(indexCount != 0);
}


template<bool small_xyz, bool matrix0Identity>
void tnlMeshletSkinVertices(uint8_t *OCR, uint8_t *OCR_normal, const uint8_t* vertex, const uint8_t* normals, const uint8_t* skinWeights, const uint8_t* skinIndexes, int vertexCount, int vertexSize, Matrix* skinMatrices) {
	
	auto dest = OCR + 4;
	auto destNormal = OCR_normal;

	// do vertexes
	{
		auto skinningIndexData = (int16_t*)skinIndexes;
		auto skinningWeightData = (uint8_t*)skinWeights;

		if (!matrix0Identity) {
			rw_mat_load_4x4(&skinMatrices[0]);
			if (small_xyz) {
				mat_apply(&DCE_MESHLET_MAT_DECODE);
			}
		} else {
			if (small_xyz) {
				mat_load(&DCE_MESHLET_MAT_DECODE);
			}
		}

		for(;;) {
			int16_t flags = *skinningIndexData++;
			if (flags >= 0) {
				const uint8_t* srcVtxBytes = vertex + flags;
				int count = *skinningIndexData++;
				uint8_t* dstVertexBytes = dest + *skinningIndexData++;

				if (matrix0Identity && !small_xyz) {
					do {
						const V3d* srcVtx = (const V3d*)(srcVtxBytes);
						srcVtxBytes += vertexSize;
						V3d* dstVertex = (V3d*)(dstVertexBytes);
						dstVertexBytes += 64;
						*dstVertex = *srcVtx;
					} while(--count != 0);
				} else {
					do {
						const V3d* srcVtx = (const V3d*)(srcVtxBytes);
						V3d tmp;
						if (small_xyz) {
							tmp =  makeV3d(*(int16_t*)srcVtxBytes, *(int16_t*)(srcVtxBytes + 2), *(int16_t*)(srcVtxBytes + 4));
							srcVtx = &tmp;
						}
						srcVtxBytes += vertexSize;
						V3d* dstVertex = (V3d*)(dstVertexBytes);
						dstVertexBytes += 64;
						float x, y, z, w;
						mat_trans_nodiv_nomod(srcVtx->x, srcVtx->y, srcVtx->z, x, y, z, w);
						dstVertex->x = x;
						dstVertex->y = y;
						dstVertex->z = z;
					} while(--count != 0);
				}
			} else if (!(flags & 0x80)) {
				int count = flags & 0x7FFF;
				uint8_t* dstVertexBytes = dest + *skinningIndexData++;

				do {
					V3d* dstVertex = (V3d*)(dstVertexBytes);
					*dstVertex = { 0, 0 ,0 };
					dstVertexBytes += 64;
				} while(--count != 0);

			} else {
				break;
			}
		}

		Matrix* currentMatrix = skinMatrices;
		for(;;) {
			auto count = *skinningIndexData++;
			if (!count) { //some matrixes may be empty
				currentMatrix++;
				continue;
			}

			if (count < 0) {	// end of skinning
				break;
			}

			rw_mat_load_4x4(currentMatrix);
			if (small_xyz){
				mat_apply(&DCE_MESHLET_MAT_DECODE);
			}

			do {
				auto srcOffset = *skinningIndexData++;
				auto dstOffset = *skinningIndexData++;

				auto srcVtxBytes = vertex + srcOffset;
				auto srcVtx = (const V3d*)(srcVtxBytes);
				V3d tmpSrc;
				if (small_xyz) {
					tmpSrc = makeV3d(*(int16_t*)srcVtxBytes, *(int16_t*)(srcVtxBytes + 2), *(int16_t*)(srcVtxBytes + 4));
					srcVtx = &tmpSrc;
				}
				auto dstVtx = (V3d*)(dest + dstOffset);
				float x, y, z, w;
				mat_trans_nodiv_nomod(srcVtx->x, srcVtx->y, srcVtx->z, x, y, z, w);
				V3d tmp = { x, y, z };
				tmp = scale(tmp, *skinningWeightData++ / 255.0f);
				*dstVtx = add(*dstVtx, tmp);
			} while (--count != 0);
			currentMatrix++;
		}
	}

	// now do normals
	{
		auto skinningIndexData = (int16_t*)skinIndexes;
		auto skinningWeightData = (uint8_t*)skinWeights;

		if (!matrix0Identity) {
			rw_mat_load_3x3(&skinMatrices[0]);
		}

		for(;;) {
			int16_t flags = *skinningIndexData++;
			if (flags >= 0) {
				const int8_t* srcNormalBytes = (int8_t*)(normals + flags);
				int count = *skinningIndexData++;
				uint8_t* dstNormalBytes = destNormal + *skinningIndexData++;

				if (matrix0Identity) {
					do {
						V3d srcNormal = { static_cast<float32>(srcNormalBytes[0]), static_cast<float32>(srcNormalBytes[1]), static_cast<float32>(srcNormalBytes[2]) };
						
						srcNormalBytes += vertexSize;
						V3d* dstNormal = (V3d*)(dstNormalBytes);
						dstNormalBytes += 64;

						*dstNormal = srcNormal;
					} while(--count != 0);
				} else {
					do {
						V3d srcNormal = { static_cast<float32>(srcNormalBytes[0]), static_cast<float32>(srcNormalBytes[1]), static_cast<float32>(srcNormalBytes[2]) };
						srcNormalBytes += vertexSize;
						V3d* dstNormal = (V3d*)(dstNormalBytes);
						dstNormalBytes += 64;
						float x, y, z, w;
						mat_trans_nodiv_nomod_zerow(srcNormal.x, srcNormal.y, srcNormal.z, x, y, z, w);
						*dstNormal = { x, y, z };
					} while(--count != 0);
				}
			} else if (!(flags & 0x80)) {
				int count = flags & 0x7FFF;
				uint8_t* dstNormalBytes = destNormal + *skinningIndexData++;

				do {
					V3d* dstNormal = (V3d*)(dstNormalBytes);
					*dstNormal = { 0, 0 ,0 };
					dstNormalBytes += 64;
				} while(--count != 0);

			} else {
				break;
			}
		}

		Matrix* currentMatrix = skinMatrices;
		for(;;) {
			auto count = *skinningIndexData++;
			if (!count) { //some matrixes may be empty
				currentMatrix++;
				continue;
			}

			if (count < 0) {	// end of skinning
				break;
			}

			rw_mat_load_3x3(currentMatrix);

			do {
				auto srcOffset = *skinningIndexData++;
				auto dstOffset = *skinningIndexData++;

				const int8_t* srcNormalBytes = (int8_t*)(normals + srcOffset);

				V3d srcNormal = { static_cast<float32>(srcNormalBytes[0]), static_cast<float32>(srcNormalBytes[1]), static_cast<float32>(srcNormalBytes[2]) };
				auto dstNormal = (V3d*)(destNormal + dstOffset);

				V3d tmp;
				float w;
				mat_trans_nodiv_nomod_zerow(srcNormal.x, srcNormal.y, srcNormal.z, tmp.x, tmp.y, tmp.z, w);
				tmp = scale(tmp, *skinningWeightData++ / 255.0f);
				*dstNormal = add(*dstNormal, tmp);
			} while (--count != 0);
			currentMatrix++;
		}
	}
}

__attribute__((noinline))
void tnlMeshletEnvMap(uint8_t* OCR, uint8_t* normal, int vertexCount, int vertexSize, matrix_t* matfxMatrix, float matfxCoefficient) {

	mat_load(matfxMatrix);

	do {
		pvr_vertex64_t* v = (pvr_vertex64_t*)OCR;
		OCR += 64;

		int8_t* normal_int8 = (int8_t*)normal;
		normal += vertexSize;

		V3d normal = { static_cast<float32>(normal_int8[0]), static_cast<float32>(normal_int8[1]), static_cast<float32>(normal_int8[2]) };

		float x, y, z, w;
		mat_trans_nodiv_nomod(normal.x, normal.y, normal.z, x, y, z, w);

		v->u = float16(x).raw;
		v->v = float16(1-y).raw;

		v->r *= matfxCoefficient;
		v->g *= matfxCoefficient;
		v->b *= matfxCoefficient;

	} while(--vertexCount != 0);
}


inline  __attribute__((always_inline))  RwFrustumTestResult AtomicFrustumSphereCB(Atomic *atomic, rw::Camera *cam)
{
    return RwCameraFrustumTestSphere(cam, atomic->getWorldBoundingSphere());
}

static constexpr void (*tnlMeshletTransformSelector[6])(uint8_t* dst, const uint8_t* vertexData, uint32_t vertexCount, uint32_t vertexSize) {
	&tnlMeshletTransform<false, 0>,
	&tnlMeshletTransform<true , 0>,
	&tnlMeshletTransform<false, 1 >,
	&tnlMeshletTransform<true , 1 >,
	&tnlMeshletTransform<false, 2 >,
	&tnlMeshletTransform<true , 2 >,
};

static constexpr void (*tnlMeshletCopyUVsSelector[2])(uint8_t* dst, const uint8_t* uvData, uint32_t vertexCount, uint32_t vertexSize) = {
	&tnlMeshletCopyUVs<false>,
	&tnlMeshletCopyUVs<true>
};

static constexpr void (*tnlMeshletFillResidualSelector[1])(uint8_t* dstCol, uint32_t vertexCount, const RGBAf *residual) = {
	&tnlMeshletFillResidual,
};

static constexpr void (*tnlMeshletVertexColorSelector[1])(uint8_t* dstCol, const int8_t* colData, uint32_t vertexCount, uint32_t vertexSize) = {
	&tnlMeshletVertexColor,
};

//tnlMeshletDiffuseColor
static constexpr void (*tnlMeshletDiffuseColorSelector[8])(uint8_t* dstCol, const uint8_t* normalData, uint32_t vertexCount, uint32_t vertexSize, const RGBAf *lights) = {
	&tnlMeshletDiffuseColor<1, false>,
	&tnlMeshletDiffuseColor<2, false>,
	&tnlMeshletDiffuseColor<3, false>,
	&tnlMeshletDiffuseColor<4, false>,

	&tnlMeshletDiffuseColor<1, true>,
	&tnlMeshletDiffuseColor<2, true>,
	&tnlMeshletDiffuseColor<3, true>,
	&tnlMeshletDiffuseColor<4, true>,
};

static constexpr void (*submitMeshletSelector[2])(uint8_t* OCR, const int8_t* indexData, uint32_t indexCount) = {
	&submitMeshlet<false>,
	&submitMeshlet<true>,
};

static constexpr void (*clipAndsubmitMeshletSelector[2])(uint8_t* OCR, const int8_t* indexData, uint32_t indexCount) = {
	&clipAndsubmitMeshlet<false>,
	&clipAndsubmitMeshlet<true>,
};

static constexpr void(*tnlMeshletSkinVerticesSelector[4])(uint8_t *OCR, uint8_t *OCR_normal, const uint8_t* vertex, const uint8_t* normals, const uint8_t* skinWeights, const uint8_t* skinIndexes, int vertexCount, int vertexSize, Matrix* skinMatrices) = {
	&tnlMeshletSkinVertices<false, false>,
	&tnlMeshletSkinVertices<true , false>,
	&tnlMeshletSkinVertices<false, true >,
	&tnlMeshletSkinVertices<true , true >,
};

bool
uploadSkinMatrices(Atomic *a, Matrix* skinMatrices)
{
	int i;
	Skin *skin = Skin::get(a->geometry);
	Matrix *m = (Matrix*)skinMatrices;
	HAnimHierarchy *hier = Skin::getHierarchy(a);

	if(hier){
		Matrix *invMats = (Matrix*)skin->inverseMatrices;
		Matrix tmp;

		assert(skin->numBones == hier->numNodes);
		if(hier->flags & HAnimHierarchy::LOCALSPACEMATRICES){
			for(i = 0; i < hier->numNodes; i++){
				invMats[i].flags = 0;
				Matrix::mult(m, &invMats[i], &hier->matrices[i]);
				m++;
			}
		}else{
			Matrix invAtmMat;
			Matrix::invert(&invAtmMat, a->getFrame()->getLTM());
			for(i = 0; i < hier->numNodes; i++){
				invMats[i].flags = 0;
				Matrix::mult(&tmp, &hier->matrices[i], &invAtmMat);
				Matrix::mult(m, &invMats[i], &tmp);
				m++;
			}
		}
	}else{
		for(i = 0; i < skin->numBones; i++){
			m->setIdentity();
			m++;
		}

		return true;
	}

	// optimization if the first matrix is identity
	return skinMatrices[0].identityError() < 0.01f;
}

static RawMatrix normal2texcoord = {
	{ 0.5f / 127,  0.0f, 0.0f }, 0.0f,
	{ 0.0f, -0.5f / 127, 0.0f }, 0.0f,
	{ 0.0f,  0.0f, 1.0f }, 0.0f,
	{ 0.5f,  0.5f, 0.0f }, 1.0f
};

void
uploadEnvMatrix(Frame *frame, RawMatrix *world, matrix_t* envMatrix)
{
	Matrix invMat;
	if(frame == nil)
		frame = engine->currentCamera->getFrame();

	RawMatrix *envMtx = (RawMatrix*)envMatrix;
	{

		RawMatrix invMtx;
		Matrix::invert(&invMat, frame->getLTM());
		convMatrix(&invMtx, &invMat);
		invMtx.pos.set(0.0f, 0.0f, 0.0f);
		float uscale = fabs(normal2texcoord.right.x);
		normal2texcoord.right.x = MatFX::envMapFlipU ? -uscale : uscale;
		
		RawMatrix tmpMtx;

		RawMatrix::mult(&tmpMtx, &invMtx, &normal2texcoord);

		world->pos = { 0, 0, 0 };
		world->rightw = 0;
		world->upw = 0;
		world->atw = 0;
		RawMatrix::mult(envMtx, world, &tmpMtx);
	}
}

// These /really/ depend on the compiler to optimize the constants out in order to be fast
// Ugly, but works
// Will rewrite later on to be more optimized

/* Compile a polygon context into a polygon header */
inline void pvr_poly_compile_fast(pvr_poly_hdr_t *dst, pvr_poly_cxt_t *src) {
    int u, v;
    uint32  txr_base;

    /* Basically we just take each parameter, clip it, shift it
       into place, and OR it into the final result. */

    /* The base values for CMD */
    dst->cmd = PVR_CMD_POLYHDR;

    if(src->txr.enable == PVR_TEXTURE_ENABLE)
        dst->cmd |= 8;

    /* Or in the list type, shading type, color and UV formats */
    dst->cmd |= (src->list_type << PVR_TA_CMD_TYPE_SHIFT);// & PVR_TA_CMD_TYPE_MASK;
    dst->cmd |= (src->fmt.color << PVR_TA_CMD_CLRFMT_SHIFT);// & PVR_TA_CMD_CLRFMT_MASK;
    dst->cmd |= (src->gen.shading << PVR_TA_CMD_SHADE_SHIFT);// & PVR_TA_CMD_SHADE_MASK;
    dst->cmd |= (src->fmt.uv << PVR_TA_CMD_UVFMT_SHIFT);// & PVR_TA_CMD_UVFMT_MASK;
    dst->cmd |= (src->gen.clip_mode << PVR_TA_CMD_USERCLIP_SHIFT);// & PVR_TA_CMD_USERCLIP_MASK;
    dst->cmd |= (src->fmt.modifier << PVR_TA_CMD_MODIFIER_SHIFT);// & PVR_TA_CMD_MODIFIER_MASK;
    dst->cmd |= (src->gen.modifier_mode << PVR_TA_CMD_MODIFIERMODE_SHIFT);// & PVR_TA_CMD_MODIFIERMODE_MASK;
    dst->cmd |= (src->gen.specular << PVR_TA_CMD_SPECULAR_SHIFT);// & PVR_TA_CMD_SPECULAR_MASK;

    /* Polygon mode 1 */
    dst->mode1  = (src->depth.comparison << PVR_TA_PM1_DEPTHCMP_SHIFT);// & PVR_TA_PM1_DEPTHCMP_MASK;
    dst->mode1 |= (src->gen.culling << PVR_TA_PM1_CULLING_SHIFT);// & PVR_TA_PM1_CULLING_MASK;
    dst->mode1 |= (src->depth.write << PVR_TA_PM1_DEPTHWRITE_SHIFT);// & PVR_TA_PM1_DEPTHWRITE_MASK;
    dst->mode1 |= (src->txr.enable << PVR_TA_PM1_TXRENABLE_SHIFT);// & PVR_TA_PM1_TXRENABLE_MASK;

    /* Polygon mode 2 */
    dst->mode2  = (src->blend.src << PVR_TA_PM2_SRCBLEND_SHIFT);// & PVR_TA_PM2_SRCBLEND_MASK;
    dst->mode2 |= (src->blend.dst << PVR_TA_PM2_DSTBLEND_SHIFT);// & PVR_TA_PM2_DSTBLEND_MASK;
    dst->mode2 |= (src->blend.src_enable << PVR_TA_PM2_SRCENABLE_SHIFT);// & PVR_TA_PM2_SRCENABLE_MASK;
    dst->mode2 |= (src->blend.dst_enable << PVR_TA_PM2_DSTENABLE_SHIFT);// & PVR_TA_PM2_DSTENABLE_MASK;
    dst->mode2 |= (src->gen.fog_type << PVR_TA_PM2_FOG_SHIFT);// & PVR_TA_PM2_FOG_MASK;
    dst->mode2 |= (src->gen.color_clamp << PVR_TA_PM2_CLAMP_SHIFT);// & PVR_TA_PM2_CLAMP_MASK;
    dst->mode2 |= (src->gen.alpha << PVR_TA_PM2_ALPHA_SHIFT);// & PVR_TA_PM2_ALPHA_MASK;

    if(src->txr.enable == PVR_TEXTURE_DISABLE) {
        dst->mode3 = 0;
    }
    else {
        dst->mode2 |= (src->txr.alpha << PVR_TA_PM2_TXRALPHA_SHIFT);// & PVR_TA_PM2_TXRALPHA_MASK;
        dst->mode2 |= (src->txr.uv_flip << PVR_TA_PM2_UVFLIP_SHIFT);// & PVR_TA_PM2_UVFLIP_MASK;
        dst->mode2 |= (src->txr.uv_clamp << PVR_TA_PM2_UVCLAMP_SHIFT);// & PVR_TA_PM2_UVCLAMP_MASK;
        dst->mode2 |= (src->txr.filter << PVR_TA_PM2_FILTER_SHIFT);// & PVR_TA_PM2_FILTER_MASK;
        dst->mode2 |= (src->txr.mipmap_bias << PVR_TA_PM2_MIPBIAS_SHIFT);// & PVR_TA_PM2_MIPBIAS_MASK;
        dst->mode2 |= (src->txr.env << PVR_TA_PM2_TXRENV_SHIFT);// & PVR_TA_PM2_TXRENV_MASK;

		u = src->txr.width;
		v = src->txr.height;

        dst->mode2 |= (u << PVR_TA_PM2_USIZE_SHIFT);// & PVR_TA_PM2_USIZE_MASK;
        dst->mode2 |= (v << PVR_TA_PM2_VSIZE_SHIFT);// & PVR_TA_PM2_VSIZE_MASK;

        /* Polygon mode 3 */
        dst->mode3  = (src->txr.mipmap << PVR_TA_PM3_MIPMAP_SHIFT);// & PVR_TA_PM3_MIPMAP_MASK;
        dst->mode3 |= (src->txr.format << PVR_TA_PM3_TXRFMT_SHIFT);// & PVR_TA_PM3_TXRFMT_MASK;

        /* Convert the texture address */
		#if defined(DC_SIM)
		txr_base = (ptr_t)src->txr.base - (ptr_t)emu_vram;
		#else
        txr_base = (ptr_t)src->txr.base;
		#endif
        txr_base = (txr_base & 0x00fffff8) >> 3;
        dst->mode3 |= txr_base;
    }
}

/* Create a colored polygon context with parameters similar to
   the old "ta" function `ta_poly_hdr_col' */
void pvr_poly_cxt_col_fast(pvr_poly_hdr_t *hdr, pvr_list_t list,
				int fmt_color,
				// isMatFX ? PVR_BLEND_SRCALPHA : doBlend ? srcBlend : PVR_BLEND_ONE,
					int blend_src,
				// isMatFX ? PVR_BLEND_INVSRCALPHA : doBlend ? dstBlend : PVR_BLEND_ZERO,
					int blend_dst,
				// zFunction,
					int depth_comparison,
				// zWrite,
					int depth_write,
				// cullMode == CULLNONE ? PVR_CULLING_SMALL : cullMode == CULLBACK ? PVR_CULLING_CW : PVR_CULLING_CCW,
					int gen_culling,
				// fogEnabled ? PVR_FOG_TABLE : PVR_FOG_DISABLE
					int gen_fog_type
				) {
    int alpha;
	pvr_poly_cxt_t cxt;
	pvr_poly_cxt_t *dst = &cxt;

    /* Start off blank */
    memset(dst, 0, sizeof(pvr_poly_cxt_t));

    /* Fill in a few values */
    dst->list_type = list;
    alpha = list > PVR_LIST_OP_MOD;
    dst->fmt.color = fmt_color;
    dst->fmt.uv = PVR_UVFMT_32BIT;
    dst->gen.shading = PVR_SHADE_GOURAUD;
    dst->depth.comparison = depth_comparison;
    dst->depth.write = depth_write;
    dst->gen.culling = gen_culling;
    dst->txr.enable = PVR_TEXTURE_DISABLE;

    if(!alpha) {
        dst->gen.alpha = PVR_ALPHA_DISABLE;
    }
    else {
        dst->gen.alpha = PVR_ALPHA_ENABLE;
    }

	dst->blend.src = blend_src;
	dst->blend.dst = blend_dst;

    dst->blend.src_enable = PVR_BLEND_DISABLE;
    dst->blend.dst_enable = PVR_BLEND_DISABLE;
    dst->gen.fog_type = gen_fog_type;
    dst->gen.color_clamp = PVR_CLRCLAMP_DISABLE;

	pvr_poly_compile_fast(hdr, dst);
}

/* Create a textured polygon context with parameters similar to
   the old "ta" function `ta_poly_hdr_txr' */
void pvr_poly_cxt_txr_fast(pvr_poly_hdr_t *hdr, pvr_list_t list,
                      int textureformat, int tw, int th, pvr_ptr_t textureaddr,
                      int filtering,
					// pvrTexAddress(&cxt, meshes[n].material->texture->getAddressU(), meshes[n].material->texture->getAddressV()),
					  int flip_u,
					  int clamp_u,
					  int flip_v,
					  int clamp_v,
						// PVR_UVFMT_16BIT,
						int fmt_uv,
				// PVR_CLRFMT_4FLOATS,
					  int fmt_color,
				// isMatFX ? PVR_BLEND_SRCALPHA : doBlend ? srcBlend : PVR_BLEND_ONE,
					int blend_src,
				// isMatFX ? PVR_BLEND_INVSRCALPHA : doBlend ? dstBlend : PVR_BLEND_ZERO,
					int blend_dst,
				// zFunction,
					int depth_comparison,
				// zWrite,
					int depth_write,
				// cullMode == CULLNONE ? PVR_CULLING_SMALL : cullMode == CULLBACK ? PVR_CULLING_CW : PVR_CULLING_CCW,
					int gen_culling,
				// fogEnabled ? PVR_FOG_TABLE : PVR_FOG_DISABLE
					int gen_fog_type
					  ) {
    int alpha;

	pvr_poly_cxt_t cxt;
	pvr_poly_cxt_t *dst = &cxt;

    /* Start off blank */
    memset(dst, 0, sizeof(pvr_poly_cxt_t));

    /* Fill in a few values */
    dst->list_type = list;
    alpha = list > PVR_LIST_OP_MOD;
    dst->fmt.color = fmt_color;
    dst->fmt.uv = fmt_uv;
    dst->gen.shading = PVR_SHADE_GOURAUD;
    dst->depth.comparison = depth_comparison;
    dst->depth.write = depth_write;
    dst->gen.culling = gen_culling;
    dst->txr.enable = PVR_TEXTURE_ENABLE;

    if(!alpha) {
        dst->gen.alpha = PVR_ALPHA_DISABLE;
        dst->txr.alpha = PVR_TXRALPHA_ENABLE;
        dst->txr.env = PVR_TXRENV_MODULATE;
    }
    else {
        dst->gen.alpha = PVR_ALPHA_ENABLE;
        dst->txr.alpha = PVR_TXRALPHA_ENABLE;
        dst->txr.env = PVR_TXRENV_MODULATEALPHA;
    }

	dst->blend.src = blend_src;
	dst->blend.dst = blend_dst;

    dst->blend.src_enable = PVR_BLEND_DISABLE;
    dst->blend.dst_enable = PVR_BLEND_DISABLE;
    dst->gen.fog_type = gen_fog_type;
    dst->gen.color_clamp = PVR_CLRCLAMP_DISABLE;
    dst->txr.uv_flip = flip_u | flip_v;
    dst->txr.uv_clamp = clamp_u | clamp_v;
    dst->txr.filter = filtering;
    dst->txr.mipmap_bias = PVR_MIPBIAS_NORMAL;
    dst->txr.width = tw;
    dst->txr.height = th;
    dst->txr.base = textureaddr;
    dst->txr.format = textureformat;

	pvr_poly_compile_fast(hdr, dst);
}



size_t vertexBufferFree() {
    size_t end   = PVR_GET(PVR_TA_VERTBUF_END);
    size_t pos   = PVR_GET(PVR_TA_VERTBUF_POS);

    size_t free  = end - pos;

	return free;
}


void defaultRenderCB(ObjPipeline *pipe, Atomic *atomic) {
    rw::Camera *cam = engine->currentCamera;
    // Frustum Culling
    auto global_frustumTestResult = AtomicFrustumSphereCB(atomic, cam);

	if (global_frustumTestResult == rwSPHEREOUTSIDE) {
		return;
	}

	bool global_needsNoClip = global_frustumTestResult == rwSPHEREINSIDE;
	
	// Material *m;

	auto geo =	atomic->geometry;
	Skin* skin = Skin::get(geo);
	uint32 flags = atomic->geometry->flags;

	bool isPrelit = !!(geo->flags & Geometry::PRELIT);
	bool isNormaled = !!(geo->flags & Geometry::NORMALS);
	bool isTextured = geo->numTexCoordSets > 0;

	int32 numMeshes = geo->meshHeader->numMeshes;

	size_t skinContextOffset = skinContexts.size();
	bool skinMatrix0Identity = false;
	if (skin) {
		skinContexts.resize(skinContextOffset + skin->numBones);
		skinMatrix0Identity = uploadSkinMatrices(atomic, &(skinContexts.data() + skinContextOffset)->mtx);
	}

	atomicContexts.emplace_back();
	auto ac = &atomicContexts.back();

	ac->meshContextOffset = meshContexts.size();
	ac->skinContextOffset = skinContextOffset;
	ac->atomic = atomic;
	ac->geo = geo;
	ac->cam = cam;

	ac->global_needsNoClip = global_needsNoClip;
	ac->skinMatrix0Identity = skinMatrix0Identity;

	lightingCB(atomic, ac->uniform);


	rw::RawMatrix world;
	rw::convMatrix(&world, atomic->getFrame()->getLTM());
	

	mat_load((matrix_t*)&cam->devView);
	mat_apply((matrix_t*)&world);
	mat_store((matrix_t*)&atomicContexts.back().worldView);

	mat_load((matrix_t*)&cam->devProjScreen);
	mat_apply((matrix_t*)&atomicContexts.back().worldView);
	mat_store((matrix_t*)&atomicContexts.back().mtx);

	int16_t contextId = atomicContexts.size() - 1;

	assert(numMeshes <= 32767);
	assert(atomicContexts.size() <= 32767);
	auto meshes = geo->meshHeader->getMeshes();

	for (int16_t n = 0; n < numMeshes; n++) {
		bool doBlend = meshes[n].material->color.alpha != 255; // TODO: check all vertexes for alpha?
		bool textured = geo->numTexCoordSets && meshes[n].material->texture;
		if (textured) {
			doBlend |= Raster::formatHasAlpha(meshes[n].material->texture->raster->format);
		}

		MatFX *matfx = MatFX::get(meshes[n].material);

		bool isMatFX = false;
		float matfxCoefficient = 0.0f;
		size_t matfxContextOffset = matfxContexts.size();
		if (doEnvironmentMaps && matfx && matfx->type == MatFX::ENVMAP && matfx->fx[0].env.tex != nil && matfx->fx[0].env.coefficient != 0.0f) {
			isMatFX = true;
			matfxCoefficient = matfx->fx[0].env.coefficient;
			matfxContexts.resize(matfxContexts.size() + 1);
			// N.B. world here gets converted to a 3x3 matrix
			// 		this is fine, as we only use it for env mapping from now on
			uploadEnvMatrix(matfx->fx[0].env.frame, &world, &matfxContexts.back().mtx);
			matfxContexts.back().coefficient = matfxCoefficient;
			
			pvr_poly_cxt_t cxt;

			auto matfxTexture = matfx->fx[0].env.tex;

			pvr_poly_cxt_txr(&cxt, PVR_LIST_TR_POLY, pvrFormatForRaster(matfxTexture->raster), matfxTexture->raster->width, matfxTexture->raster->height, pvrTexturePointer(matfxTexture->raster), PVR_FILTER_BILINEAR);
			pvrTexAddress(&cxt, matfxTexture->getAddressU(), matfxTexture->getAddressV());
			cxt.fmt.uv = PVR_UVFMT_16BIT;

			cxt.txr.alpha = PVR_TXRALPHA_DISABLE;
			
			cxt.fmt.color = PVR_CLRFMT_4FLOATS;

			cxt.blend.src = PVR_BLEND_SRCALPHA;
			cxt.blend.dst = PVR_BLEND_ONE;

			cxt.depth.comparison = zFunction;
			cxt.depth.write 	 = zWrite;
			cxt.gen.culling = cullModePvr;

			pvr_poly_hdr_t hdr;
			pvr_poly_compile(&hdr, &cxt);
			matfxContexts.back().hdr_cmd = hdr.cmd;
			matfxContexts.back().hdr_mode1 = hdr.mode1;
			matfxContexts.back().hdr_mode2 = hdr.mode2;
			matfxContexts.back().hdr_mode3 = hdr.mode3;
		}

		pvr_poly_cxt_t cxt;
		int pvrList;
		if (doBlend || isMatFX) {
			if (doAlphaTest) {
				pvrList = PVR_LIST_PT_POLY;
			} else {
				pvrList = PVR_LIST_TR_POLY;
			}
		} else {
			pvrList = PVR_LIST_OP_POLY;
		}		

		pvr_poly_hdr_t hdr;

		if (textured) {
			pvr_poly_cxt_txr_fast(
				&hdr,
				pvrList,

				pvrFormatForRaster(meshes[n].material->texture->raster),
				GETDCRASTEREXT(meshes[n].material->texture->raster)->raster->u,
				GETDCRASTEREXT(meshes[n].material->texture->raster)->raster->v,
				pvrTexturePointer(meshes[n].material->texture->raster),
				PVR_FILTER_BILINEAR,
				// flip_u, clamp_u, flip_v, clamp_v,
				meshes[n].material->texture->getAddressU() == Texture::MIRROR ? PVR_UVFLIP_U : PVR_UVFLIP_NONE,
				meshes[n].material->texture->getAddressU() == Texture::CLAMP ? PVR_UVCLAMP_U : PVR_UVCLAMP_NONE,
				meshes[n].material->texture->getAddressV() == Texture::MIRROR ? PVR_UVFLIP_V : PVR_UVFLIP_NONE,
				meshes[n].material->texture->getAddressV() == Texture::CLAMP ? PVR_UVCLAMP_V : PVR_UVCLAMP_NONE,
				PVR_UVFMT_16BIT,

				PVR_CLRFMT_4FLOATS,
				isMatFX ? PVR_BLEND_SRCALPHA : doBlend ? srcBlend : PVR_BLEND_ONE,
				isMatFX ? PVR_BLEND_INVSRCALPHA : doBlend ? dstBlend : PVR_BLEND_ZERO,
				zFunction,
				zWrite,
				cullModePvr,
				fogFuncPvr
			);
		} else {
			pvr_poly_cxt_col_fast(
				&hdr,
				pvrList,

				PVR_CLRFMT_4FLOATS,
				isMatFX ? PVR_BLEND_SRCALPHA : doBlend ? srcBlend : PVR_BLEND_ONE,
				isMatFX ? PVR_BLEND_INVSRCALPHA : doBlend ? dstBlend : PVR_BLEND_ZERO,
				zFunction,
				zWrite,
				cullModePvr,
				fogFuncPvr
			);
		}
		
		meshContexts.emplace_back();
		auto mc = &meshContexts.back();

		mc->color = meshes[n].material->color;
		mc->ambient = meshes[n].material->surfaceProps.ambient;
		mc->diffuse = meshes[n].material->surfaceProps.diffuse;
		mc->matfxContextOffset = isMatFX ? matfxContextOffset : SIZE_MAX;

		mc->hdr_cmd = hdr.cmd;
		mc->hdr_mode1 = hdr.mode1;
		mc->hdr_mode2 = hdr.mode2;
		mc->hdr_mode3 = hdr.mode3;

		// clipping performed per meshlet
		auto renderCB = [contextId, n] {
			if (vertexBufferFree() < (128 * 1024)) {
				return;
			}
			const atomic_context_t* acp = &atomicContexts[contextId];
			auto geo = acp->geo;
			auto mesh = geo->meshHeader->getMeshes() + n;
			const auto& global_needsNoClip = acp->global_needsNoClip;
			const auto& uniformObject = acp->uniform;
			const auto& mtx = acp->mtx;
			const auto& worldView = acp->worldView;
			const auto& atomic = acp->atomic;
			const auto& cam = acp->cam;
			const auto meshContext = &meshContexts[acp->meshContextOffset + n];
			Skin* skin = Skin::get(geo);

			bool textured = geo->numTexCoordSets && mesh->material->texture;
			
			auto* hdr = reinterpret_cast<pvr_poly_hdr_t *>(pvr_dr_target(drState));
			hdr->cmd = meshContext->hdr_cmd;
			hdr->mode1 = meshContext->hdr_mode1;
			hdr->mode2 = meshContext->hdr_mode2;
			hdr->mode3 = meshContext->hdr_mode3;
			pvr_dr_commit(hdr);

			auto dcModel = (DCModelDataHeader*)geo->instData;

			RGBAf residual, material;
			// Ambient Alpha ALWAYS = 1.0
			residual.alpha = (1/255.0f) * meshContext->color.alpha;
			residual.red = uniformObject.ambLight.red * meshContext->ambient * meshContext->color.red * (1/ 255.0f);
			residual.green = uniformObject.ambLight.green * meshContext->ambient * meshContext->color.green * (1/ 255.0f);
			residual.blue = uniformObject.ambLight.blue * meshContext->ambient * meshContext->color.blue * (1/ 255.0f);
			material.alpha = (1/255.0f) * meshContext->color.alpha;
			material.red = colorScaleCharToFloat * meshContext->color.red;
			material.green = colorScaleCharToFloat * meshContext->color.green;
			material.blue = colorScaleCharToFloat * meshContext->color.blue;
			
			if (dcModel) {
				const MeshInfo* meshInfo = (const MeshInfo*)&dcModel->data[0];
				auto meshletInfoBytes = &dcModel->data[meshInfo[n].meshletOffset];

				unsigned cntDiffuse = 0;
				RGBAf lightDiffuseColors[MAX_LIGHTS];
				
				cntDiffuse = uniformObject.lightCount;

				for (unsigned i = 0; i < cntDiffuse; i++) {
					lightDiffuseColors[i].red = material.red * uniformObject.col[i].red * meshContext->diffuse;
					lightDiffuseColors[i].green = material.green * uniformObject.col[i].green * meshContext->diffuse;
					lightDiffuseColors[i].blue = material.blue * uniformObject.col[i].blue * meshContext->diffuse;
				}

				for (unsigned meshletNum = 0; meshletNum < meshInfo[n].meshletCount; meshletNum++) {
					auto meshlet = (const MeshletInfo*)meshletInfoBytes;
					meshletInfoBytes += sizeof(MeshletInfo) - (skin ? 0 : 8);

					unsigned clippingRequired = 0;

					if (!global_needsNoClip) {
						RwSphere sphere = meshlet->boundingSphere;
						RwV3dTransformPoints(&sphere.center, &sphere.center, 1, atomic->getFrame()->getLTM());
						auto local_frustumTestResult = RwCameraFrustumTestSphere(cam, &sphere);
						if ( local_frustumTestResult == rwSPHEREOUTSIDE) {
							// printf("Outside frustum cull\n");
							continue;
						}

						if (local_frustumTestResult == rwSPHEREBOUNDARY) {
							// printf("meshlet %d, vertexOffset %d, indexOffset %d, vertexCount %d, indexCount %d\n", meshletNum, meshlet->vertexOffset, meshlet->indexOffset, meshlet->vertexCount, meshlet->indexCount);
							mat_load(&worldView);  // Number of cycles: ~11.
							
							float x, y, z, w;
							
							mat_trans_nodiv_nomod(meshlet->boundingSphere.center.x, meshlet->boundingSphere.center.y, meshlet->boundingSphere.center.z, x, y, z, w);

							if (z < meshlet->boundingSphere.radius) {
								clippingRequired = 1 + textured;
							}
						}
					}

					if (meshContext->matfxContextOffset != SIZE_MAX) {
						auto* hdr = reinterpret_cast<pvr_poly_hdr_t *>(pvr_dr_target(drState));
						hdr->cmd = meshContext->hdr_cmd;
						hdr->mode1 = meshContext->hdr_mode1;
						hdr->mode2 = meshContext->hdr_mode2;
						hdr->mode3 = meshContext->hdr_mode3;
						pvr_dr_commit(hdr);
					}

					//isTextured, isNormaled, isColored, small_xyz, pad_xyz, small_uv
					unsigned selector = meshlet->flags;

					// template<bool hasTexture, bool small_xyz, bool forClip>
					unsigned smallSelector = ((selector & 8) ? 1 : 0) | clippingRequired * 2;

					dce_set_mat_decode(
						meshlet->boundingSphere.radius / 32767.0f,
						meshlet->boundingSphere.center.x,
						meshlet->boundingSphere.center.y,
						meshlet->boundingSphere.center.z
					);

					if (skin) {
						unsigned normalOffset = (selector & 8) ? (3 * 2) : (3 * 4);
						if (selector & 16) {
							normalOffset += 1 * 2;
						}

						normalOffset += (selector & 32) ? 2 : 4;

						auto normalSrc = &dcModel->data[meshlet->vertexOffset] + normalOffset;

						uint8_t * normalDst;
						if (textured) {
							normalDst = OCR_SPACE + offsetof(pvr_vertex64_t, _dmy1);
						} else {
							normalDst = OCR_SPACE + offsetof(pvr_vertex64_t, a);
						}
						
						bool small_xyz = selector & 8;
						unsigned skinSelector = small_xyz + acp->skinMatrix0Identity*2;
						tnlMeshletSkinVerticesSelector[skinSelector](OCR_SPACE, normalDst, &dcModel->data[meshlet->vertexOffset],  normalSrc, &dcModel->data[meshlet->skinWeightOffset], &dcModel->data[meshlet->skinIndexOffset], meshlet->vertexCount, meshlet->vertexSize, &(skinContexts.data() + acp->skinContextOffset)->mtx);
						
						mat_load(&mtx);
						tnlMeshletTransformSelector[clippingRequired * 2](OCR_SPACE, OCR_SPACE + 4, meshlet->vertexCount, 64);
					} else {
						
						if (selector & 8) {
							mat_load(&mtx);
							mat_apply(&DCE_MESHLET_MAT_DECODE);
						} else {
							mat_load(&mtx);
						}
						tnlMeshletTransformSelector[smallSelector](OCR_SPACE, &dcModel->data[meshlet->vertexOffset], meshlet->vertexCount, meshlet->vertexSize);
					}

					if (textured) {
						unsigned uvOffset = (selector & 8) ? (3 * 2) : (3 * 4);
						if (selector & 16) {
							uvOffset += 1 * 2;
						}

						unsigned small_uv = (selector & 32) ? 1 : 0;
						tnlMeshletCopyUVsSelector[small_uv](OCR_SPACE, &dcModel->data[meshlet->vertexOffset] + uvOffset, meshlet->vertexCount, meshlet->vertexSize);
					}

					if (selector & 4) {
						unsigned colOffset = (selector & 8) ? (3 * 2) : (3 * 4);
						if (selector & 16) {
							colOffset += 1 * 2;
						}

						colOffset += (selector & 32) ? 2 : 4;

						colOffset += (selector & 2) ? 4 : 0;

						unsigned dstColOffset = textured ? offsetof(pvr_vertex64_t, a) : offsetof(pvr_vertex32_ut, a);
						dce_set_mat_vertex_color(&residual, &material);
						mat_load(&DCE_MESHLET_MAT_VERTEX_COLOR);
						tnlMeshletVertexColorSelector[0](OCR_SPACE + dstColOffset, (int8_t*)&dcModel->data[meshlet->vertexOffset] + colOffset, meshlet->vertexCount, meshlet->vertexSize);
					} else {
						unsigned dstColOffset = textured ? offsetof(pvr_vertex64_t, a) : offsetof(pvr_vertex32_ut, a);
						tnlMeshletFillResidualSelector[0](OCR_SPACE + dstColOffset, meshlet->vertexCount, &residual);
					}

					if (cntDiffuse) {
						unsigned normalOffset = (selector & 8) ? (3 * 2) : (3 * 4);
						if (selector & 16) {
							normalOffset += 1 * 2;
						}

						normalOffset += (selector & 32) ? 2 : 4;

						unsigned dstColOffset = textured ? offsetof(pvr_vertex64_t, a) : offsetof(pvr_vertex32_ut, a);

						unsigned pass1 = cntDiffuse > 4? 4 : cntDiffuse;
						unsigned pass2 = cntDiffuse > 4 ? cntDiffuse - 4 : 0;

						
						unsigned normalSelector = (pass1 - 1) + (skin != 0) * 4;
						mat_load((matrix_t*)&uniformObject.dir[0][0][0]);
						auto normalPointer = &dcModel->data[meshlet->vertexOffset] + normalOffset;
						auto vtxSize = meshlet->vertexSize;
						if (skin) {
							vtxSize = 64;
							if (textured) {
								normalPointer = OCR_SPACE + offsetof(pvr_vertex64_t, _dmy1);
							} else {
								normalPointer = OCR_SPACE + offsetof(pvr_vertex64_t, a);
							}
						}
						tnlMeshletDiffuseColorSelector[normalSelector](OCR_SPACE + dstColOffset, normalPointer, meshlet->vertexCount, vtxSize, &lightDiffuseColors[0]);
					
						if (pass2) {
							unsigned normalSelector = (pass2 - 1) + (skin != 0) * 4;
							mat_load((matrix_t*)&uniformObject.dir[1][0][0]);
							tnlMeshletDiffuseColorSelector[normalSelector](OCR_SPACE + dstColOffset, normalPointer, meshlet->vertexCount, vtxSize, &lightDiffuseColors[4]);
						}
					}

					auto indexData = (int8_t*)&dcModel->data[meshlet->indexOffset];

					if (!clippingRequired) {
						submitMeshletSelector[textured](OCR_SPACE, indexData, meshlet->indexCount);
					} else {
						clipAndsubmitMeshletSelector[textured](OCR_SPACE, indexData, meshlet->indexCount);
					}

					if (meshContext->matfxContextOffset != SIZE_MAX) {
						assert(!skin);
						auto matfxContext = &matfxContexts[meshContext->matfxContextOffset];

						auto* hdr = reinterpret_cast<pvr_poly_hdr_t *>(pvr_dr_target(drState));
						hdr->cmd = matfxContext->hdr_cmd;
						hdr->mode1 = matfxContext->hdr_mode1;
						hdr->mode2 = matfxContext->hdr_mode2;
						hdr->mode3 = matfxContext->hdr_mode3;
						pvr_dr_commit(hdr);
						
						unsigned normalOffset = (selector & 8) ? (3 * 2) : (3 * 4);
						if (selector & 16) {
							normalOffset += 1 * 2;
						}

						normalOffset += (selector & 32) ? 2 : 4;

						tnlMeshletEnvMap(OCR_SPACE, &dcModel->data[meshlet->vertexOffset] + normalOffset, meshlet->vertexCount, meshlet->vertexSize, &matfxContext->mtx, matfxContext->coefficient);

						if (!clippingRequired) {
							submitMeshletSelector[true](OCR_SPACE, indexData, meshlet->indexCount);
						} else {
							clipAndsubmitMeshletSelector[true](OCR_SPACE, indexData, meshlet->indexCount);
						}
					}
				}
			} else if (geo->meshHeader->flags & rw::MeshHeader::TRISTRIP) {
				auto numIndices = mesh->numIndices;
				auto vertices = geo->morphTargets[0].vertices;
				auto texcoords = geo->texCoords[0];
				auto colors = geo->colors;

				assert(numIndices >= 3);
				assert(geo->numVertices <= 128);
				bool isPrelit = !!(geo->flags & Geometry::PRELIT);
				assert(isPrelit);
				assert(textured);
				bool isNormaled = !!(geo->flags & Geometry::NORMALS);
				assert(!isNormaled);

				std::vector<int8_t> indices(numIndices);
				for (int i = 0; i < numIndices; i++) {
					auto idx = mesh->indices[i];
					assert(idx < 128);
					indices[i] = idx;
				}
				indices.back() |= 0x80;

				pvr_vertex64_t *vd = (pvr_vertex64_t *)OCR_SPACE;
				mat_load(&mtx);  // Number of cycles: ~11

				for (int idx = 0; idx < geo->numVertices; idx++) {
					auto& vert = vertices[idx];
					auto& c = colors[idx];
					auto& t = texcoords[idx];

					float x, y, z, w;
					mat_trans_nodiv_nomod(vert.x, vert.y, vert.z,
											x, y, z, w);
					
					vd->o_a = x;
					vd->o_r = y;
					vd->tex_z = z;
					vd->o_g = w;

					w = frsqrt(w * w);

					vd->x = x * w;
					vd->y = y * w;
					vd->z = w;

					vd->a = c.alpha * (1/255.0f);
					vd->r = c.red * (1/255.0f);
					vd->g = c.green * (1/255.0f);
					vd->b = c.blue * (1/255.0f);
					
					float16 u = texcoords[idx].u;
					float16 v = texcoords[idx].v;
					vd->u = u.raw;
					vd->v = v.raw;
					vd++;
				}

				clipAndsubmitMeshletSelector[textured](OCR_SPACE, indices.data(), indices.size());
			} else { // no trilist assets anymore
				assert(false && "Unsupported geometry type");
			}
		};

		if (doBlend || isMatFX) {
			if (doAlphaTest) {
				ptCallbacks.emplace_back(std::move(renderCB));
			} else {
				blendCallbacks.emplace_back(std::move(renderCB));
			}
		} else {
			opCallbacks.emplace_back(std::move(renderCB));
		}
	}
}

void allocDefrag(int iterations) {
	dbglog(DBG_CRITICAL, "allocDefrag(%d): Starting\n", iterations);
	alloc_run_defrag([](void* src, void* dst, void* ctx, void* user_data) {
		auto *dcr = (DcRaster*)ctx;
		assert(dcr->texaddr == src);
		dcr->texaddr = dst;
	}, iterations, nullptr);
	dbglog(DBG_CRITICAL, "allocDefrag(%d): Finished\n", iterations);
}

pvr_ptr_t allocTexture(DcRaster* ctx, size_t size) {
	auto rv = alloc_malloc(ctx, size);
	while(rv == 0) {

		if(!re3RemoveLeastUsedModel()) {
			allocDefrag(10);
			if ((rv = alloc_malloc(ctx, size))) {
				break;
			}
			dbglog(DBG_CRITICAL, "failed to free or defrag vram, sz: %lu, cont: %lu, free: %lu\n", size, alloc_count_continuous(), alloc_count_free());
			return 0;
		}
		rv = alloc_malloc(ctx, size);
	}
	logf("allocTexture: allocated: %d bytes @ 0x%08X\n", size, rv);
	// pvr_mem_stats();
	// #if !defined(MACOS64)
	// malloc_stats();
	// #endif
	return rv;
}


Raster*
rasterCreate(Raster* raster)
{
	auto natras = GETDCRASTEREXT(raster);

    if (raster->type != Raster::TEXTURE) {
        printf("rasterCreate: unsupported type %d\n", raster->type);
    }

	if(raster->width == 0 || raster->height == 0){
		raster->flags |= Raster::DONTALLOCATE;
		raster->stride = 0;
        return raster;
	}

	if (raster->width < 8) {
		printf("rasterCreate: Increasing width to 8 from %d\n", raster->width);
		raster->width = 8;
	}

	if (raster->height < 8) {
		printf("rasterCreate: Increasing height to 8 from %d\n", raster->height);
		raster->height = 8;
	}
	auto rasterFmt = raster->format & 0x0F00;
	// assert(raster->depth == 16);
	if (raster->depth != 16) {
		raster->depth = 16;
		// TODO: stop this from happening
		printf("rasterCreate: Usupported raster depth: this raster will be corrupted\n");
	}

	natras->raster = (DcRaster*)malloc(sizeof(DcRaster));
	memset(natras->raster, 0, sizeof(DcRaster));
	natras->raster->refs = 1;
	natras->raster->u = __builtin_ctz(raster->width) - 3;
	natras->raster->v = __builtin_ctz(raster->height) - 3;

	if (rasterFmt == Raster::C565) {
		natras->raster->pvr_flags |= PVR_TXRFMT_RGB565;
	} else if (rasterFmt == Raster::C1555) {
		natras->raster->pvr_flags |= PVR_TXRFMT_ARGB1555;
	} else if (rasterFmt == Raster::C4444) {
		natras->raster->pvr_flags |= PVR_TXRFMT_ARGB4444;
	} else {
		// TODO: stop this from happening
		printf("rasterCreate: Usupported raster depth: this raster will be corrupted\n");
		// assert(false && "unsupported rasterFmt");
	}

	raster->stride = raster->width * 2;
	return raster;
}

uint8*
rasterLock(Raster* raster, int32 level, int32 lockMode)
{
	auto natras = GETDCRASTEREXT(raster);
	assert(!natras->raster->native);
	
	assert(level == 0);
	// assert(0 && "lockRaster not implemented");
	// logf("Implement this: %s %d\n", __func__, __LINE__);
	// return nil;
	if (!natras->raster->texaddr) {
		natras->raster->texaddr = allocTexture(natras->raster, raster->width * raster->height * 2);
		natras->raster->pvr_flags |= PVR_TXRFMT_NONTWIDDLED;
	}
	if (raster->pixels == nullptr) {
		raster->pixels = (rw::uint8*)natras->raster->texaddr;
	}
	return raster->pixels;
}

void
rasterUnlock(Raster* raster, int32)
{
	// assert(0 && "unlockRaster not implemented");
	// logf("Implement this: %s %d\n", __func__, __LINE__);
	raster->pixels = nullptr;
}

uint8*
rasterLockPalette(Raster*, int32)
{
	assert(0 && "rasterLockPalette not implemented");
	logf("Implement this: %s %d\n", __func__, __LINE__);
	return nil;
}

void
rasterUnlockPalette(Raster*)
{
	assert(0 && "rasterUnlockPalette not implemented");
	logf("Implement this: %s %d\n", __func__, __LINE__);
}

int32
rasterNumLevels(Raster*)
{
    // TODO: Add mipmapping
	// assert(0 && "rasterNumLevels not implemented");
	// logf("Implement this: %s %d\n", __func__, __LINE__);
	return 1;
}

#if !defined(DC_TEXCONV)
int32 maxRasterWidth = 32;
int32 maxRasterHeight = 32;
int32 downsampleMode = NONE;
int32 conversionTool = PVRTEX;
#else
int32 maxRasterWidth = 64;
int32 maxRasterHeight = 64;
int32 downsampleMode = NONE;
int32 pvrEncoder = PVRTEX;
#endif

bool32
imageFindRasterFormat(Image *img, int32 type,
	int32 *pWidth, int32 *pHeight, int32 *pDepth, int32 *pFormat)
{
	int32 width, height, depth, format;

	assert((type&0xF) == Raster::TEXTURE);

	for(width = 1; width < img->width; width <<= 1)
		;
	for(height = 1; height < img->height; height <<= 1)
		;

    if(downsampleMode >= HALF) {
        if(height / 2 >= 16) {
            height /= 2;
        }
        if(width / 2 >= 16) {
            width /= 2;
        }
    }

    if(downsampleMode >= QUARTER) {
        if(height / 2 >= 16) {
            height /= 2;
        }
        if(width / 2 >= 16) {
            width /= 2;
        }
    }

    // If width or height exceed the configured max res, cap to max while preserving aspect ratio for
    if (height > maxRasterHeight || width > maxRasterWidth) {
        float aspectRatio = (float)width / height;
        // Determine which dimension to scale based on the maximum allowed size
        if (width > maxRasterWidth) {
            // Cap width and adjust height based on aspect ratio
            width = maxRasterWidth;
            height = (int)(width / aspectRatio);
        }
        if (height > maxRasterHeight) {
            // Cap height and adjust width based on aspect ratio
            height = maxRasterHeight;
            width = (int)(height * aspectRatio);
        }
    }

	depth = img->depth;

	if(depth <= 8)
		depth = 32;

	switch(depth){
		case 32:
			if(img->hasAlpha())
				format = Raster::C4444;
			else{
				format = Raster::C565;
			}
		break;
	case 24:
		format = Raster::C565;
		break;
	case 16:
		format = Raster::C1555;
		break;
	default:
		RWERROR((ERR_INVRASTER));
		return 0;
	}

	format |= type;

	*pWidth = width;
	*pHeight = height;
	*pDepth = 16;
	*pFormat = format;

	return 1;
}

#if defined(DC_TEXCONV)

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#if defined(_WIN32) || defined(_WIN64)
#include <windows.h> // For QueryPerformanceCounter and GetCurrentThreadId
#elif defined(__APPLE__)
#include <mach/mach_time.h> // For mach_absolute_time
#include <pthread.h>
#else
#include <pthread.h>
#include <sys/time.h>
#endif

void generate_uuid(char *uuid, size_t size) {
    const char *chars = "0123456789abcdef";
    int i;

    if (size < 37) { // UUID requires at least 36 characters + null terminator
        fprintf(stderr, "Buffer too small for UUID\n");
        exit(1);
    }

    unsigned int seed;

#if defined(_WIN32) || defined(_WIN64)
    // Use a high-resolution performance counter and thread ID for randomness
    LARGE_INTEGER counter;
    QueryPerformanceCounter(&counter);
    seed = (unsigned int)(counter.QuadPart ^ GetCurrentThreadId());

#elif defined(__APPLE__)
    // Use mach_absolute_time and pthread_self for randomness on macOS
    uint64_t time = mach_absolute_time();
    seed = (unsigned int)(time ^ (uintptr_t)pthread_self());

#else
    // Use high-resolution time and pthread_self for Linux/Unix
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    seed = (unsigned int)(pthread_self() ^ ts.tv_nsec ^ ts.tv_sec);
#endif

    srand(seed); // Seed the random number generator

    for (i = 0; i < 36; i++) {
        if (i == 8 || i == 13 || i == 18 || i == 23) {
            uuid[i] = '-'; // Add dashes at the appropriate positions
        } else {
            uuid[i] = chars[rand() % 16];
        }
    }
    uuid[36] = '\0'; // Null-terminate the string
}

bool32
rasterFromImage(Raster* raster, Image* image)
{
	auto natras = GETDCRASTEREXT(raster);

    Image *truecolimg = nil;
	if(image->depth <= 8){
		truecolimg = Image::create(image->width, image->height, image->depth);
		truecolimg->pixels = image->pixels;
		truecolimg->stride = image->stride;
		truecolimg->palette = image->palette;
		truecolimg->unpalettize();
		image = truecolimg;
	}

	auto rasterFmt = raster->format & 0x0F00;

	std::vector<Color> imageData;
	if (image->depth == 32) {
		assert(rasterFmt == Raster::C4444 || rasterFmt == Raster::C1555);
		imageData = createImageFromData_ARGB8888(image->pixels, image->width, image->height, image->stride);
    } else if (image->depth == 24) {
		assert(rasterFmt == Raster::C565);
		imageData = createImageFromData_RGB888(image->pixels, image->width, image->height, image->stride);
	} else if (image->depth == 16) {
		assert(rasterFmt == Raster::C1555);
		imageData = createImageFromData_ARGB1555(image->pixels, image->width, image->height, image->stride);
    } else {
		assert(false && "Unhandled texture format");
    }

	if (raster->width != image->width || raster->height != image->height) {
	    printf("Downsample: %ix%i -> %ix%i\n", image->width, image->height, raster->width, raster->height);
		imageData = downscaleImage(imageData, image->width, image->height, raster->width, raster->height);
	}

    char uuid[37];
    generate_uuid(uuid,  sizeof(uuid));
    char filename_tga[80];
    char filename_pvr[80];

#ifdef _WIN32
    snprintf(filename_tga, sizeof(filename_tga), "repack-data\\%s.tga", uuid);
    snprintf(filename_pvr, sizeof(filename_tga), "repack-data\\%s.pvr", uuid);
#else
    snprintf(filename_tga, sizeof(filename_tga), "./repack-data/%s.tga", uuid);
    snprintf(filename_pvr, sizeof(filename_tga), "./repack-data/%s.pvr", uuid);
#endif

    // Write the image to a .tga file
    int tgaDepth = rasterFmt == Raster::C565 ? 24 : 32; // Determine TGA depth
    if (!writeTGA(filename_tga, imageData, raster->width, raster->height, tgaDepth)) {
        fprintf(stderr, "Failed to write TGA file: %s\n", filename_tga);
        return false;
    }

    char encodeCommand[512];

    // Now call PVR Compression
    switch(pvrEncoder) {
#if defined(_WIN32) || defined(_WIN64)
        case PVRTEX:
            snprintf(encodeCommand, sizeof(encodeCommand),
                 "pvrtex\\pvrtex.exe -i %s -o %s -c small -d", filename_tga, filename_pvr);
        break;
        case PVRTOOL:
            snprintf(encodeCommand, sizeof(encodeCommand),
                     "pvrtool.exe %s -OF pvr -TW -CF SMART -VQ -VQDITHER 1 -o %s",
                     filename_tga, filename_pvr);
            break;
#else
        case PVRTEX:
            snprintf(encodeCommand, sizeof(encodeCommand),
                 "./pvrtex/pvrtex -i %s -o %s -c small -d", filename_tga, filename_pvr);
        break;
        case PVRTOOL:
            snprintf(encodeCommand, sizeof(encodeCommand),
                     "./pvrtool %s -OF pvr -TW -CF SMART -VQ -VQDITHER 1 -o %s",
                     filename_tga, filename_pvr);
            break;
#endif
    }

    int retCode = system(encodeCommand);

    // Read PVR File
    assert(raster->width && raster->height && !natras->raster->texaddr);
    loadPVR(filename_pvr, raster, natras->raster, PVR_TXRFMT_TWIDDLED | PVR_TXRFMT_VQ_ENABLE);


    remove(filename_tga);
    remove(filename_pvr);

	hash_sha256 hash;
	hash.sha256_init();
	hash.sha256_update((const uint8_t*)natras->raster->texaddr, natras->raster->texsize);
	sha256_type hash_result = hash.sha256_final();

	natras->raster->pvr_id = hash_result[0] | (hash_result[1] << 8) | (hash_result[2] << 16) | (hash_result[3] << 24);


    if(truecolimg)
		truecolimg->destroy();

	return 1;
}
#else
bool32
rasterFromImage(Raster* raster, Image* image)
{
	auto natras = GETDCRASTEREXT(raster);

    Image *truecolimg = nil;
	if(image->depth <= 8){
		truecolimg = Image::create(image->width, image->height, image->depth);
		truecolimg->pixels = image->pixels;
		truecolimg->stride = image->stride;
		truecolimg->palette = image->palette;
		truecolimg->unpalettize();
		image = truecolimg;
	}

	assert(raster->width && raster->height );
	assert(raster->depth == 16);

    // PVR allocate to pixels
    if (!natras->raster->texaddr) {
        natras->raster->texaddr = allocTexture(natras->raster, raster->width * raster->height * 2);
		natras->raster->pvr_flags |= PVR_TXRFMT_NONTWIDDLED;
    }

	auto rasterFmt = raster->format & 0x0F00;

    logf("rasterFromImage: updated %d x %d texture @ %p from %p format %d\n", raster->width, raster->height, natras->raster->texaddr, image->pixels, image->depth);

    if (image->depth == 32) {
        auto dst = (uint16*)natras->raster->texaddr;
        if (rasterFmt == Raster::C4444) {
			auto src = (uint8*)image->pixels;
			int ratioW = image->width / raster->width;
			int ratioH = image->height / raster->height;

			for (int y = 0; y < raster->height; y++) {
				int imgY=y*image->height/ raster->height;
				for (int x = 0; x<raster->width; x++) {
					int imgX=x*image->width/ raster->width;
					auto imgBase = (imgY * image->width + imgX)*4;
					
					auto colR = src[imgBase + 0] >> 4;
					auto colG = src[imgBase + 1] >> 4;
					auto colB = src[imgBase + 2] >> 4;
					auto colA = src[imgBase + 3] >> 4;
					
					dst[y * raster->width + x] = (colB << 0) | (colG << 4) | (colR << 8) | (colA << 12);
				}
			}
		} else if (rasterFmt == Raster::C1555) {
			auto src = (uint8*)natras->raster->texaddr;
			int ratioW = image->width / raster->width;
			int ratioH = image->height / raster->height;

			for (int y = 0; y < raster->height; y++) {
				int imgY=y*image->height/ raster->height;
				for (int x = 0; x<raster->width; x++) {
					int imgX=x*image->width/ raster->width;
					auto imgBase = (imgY * image->width + imgX)*4;
					
					auto colR = src[imgBase + 0] >> 3;
					auto colG = src[imgBase + 1] >> 3;
					auto colB = src[imgBase + 2] >> 3;
					auto colA = src[imgBase + 3] >= 0x80 ? 1 : 0;
					
					dst[y * raster->width + x] = (colB << 0) | (colG << 5) | (colR << 10) | (colA << 15);
				}
			}
		}
		
    } else if (image->depth == 24) {
		assert(rasterFmt == Raster::C565);
        auto dst = (uint16*)natras->raster->texaddr;
        auto src = (uint8*)image->pixels;
        int ratioW = image->width / raster->width;
        int ratioH = image->height / raster->height;

        for (int y = 0; y < raster->height; y++) {
            int imgY=y*image->height/ raster->height;
            for (int x = 0; x<raster->width; x++) {
                int imgX=x*image->width/ raster->width;
                auto imgBase = (imgY * image->width + imgX)*3;
                auto colR = src[imgBase + 0] >> 3;
                auto colG = src[imgBase + 1] >> 2;
                auto colB = src[imgBase + 2] >> 3;

                dst[y * raster->width + x] = (colB << 0) | (colG << 5) | (colR << 11);
            }
        }
	} else if (image->depth == 16) {
		assert(rasterFmt == Raster::C1555);
		auto dst = (uint16*)natras->raster->texaddr;
        auto src = (uint16*)image->pixels;
        int ratioW = image->width / raster->width;
        int ratioH = image->height / raster->height;

        for (int y = 0; y < raster->height; y++) {
            int imgY=y*image->height/ raster->height;
            for (int x = 0; x<raster->width; x++) {
                int imgX=x*image->width/ raster->width;
                auto imgBase = (imgY * image->width + imgX);
                auto colR = (src[imgBase] >> 0) & 0x1F;
                auto colG = (src[imgBase] >> 5) & 0x1F;
                auto colB = (src[imgBase] >> 10) & 0x1F;
				auto colA = (src[imgBase] >> 15) & 0x1;

                dst[y * raster->width + x] = (colB << 0) | (colG << 5) | (colR << 10) | (colA << 15);
            }
        }
    } else {
		assert(false && "Unhandled texture format");
        memcpy(raster->pixels, image->pixels, raster->width * raster->height * 2);
    }

    if(truecolimg)
		truecolimg->destroy();

	return 1;
}
#endif

Image*
rasterToImage(Raster*)
{
	logf("Implement this: %s %d\n", __func__, __LINE__);
	assert(0 && "rasterToImage not implemented");
	return nil;
}

int
deviceSystem(DeviceReq req, void *arg0, int32 n)
{
	switch(req){
	case DEVICEGETNUMSUBSYSTEMS:
		return 1;
	case DEVICEGETCURRENTSUBSYSTEM:
		return 0;
	case DEVICEGETSUBSSYSTEMINFO:
		strncpy(((SubSystemInfo*)arg0)->name,"TV",80);
		return 1;
	
	case DEVICEOPEN:
		return 1;
	case DEVICECLOSE:
		return 1;

	case DEVICEINIT:
		return 1;
	case DEVICETERM:
		return 1;

	case DEVICEFINALIZE:
		return 1;

	// TODO: implement subsystems

	case DEVICEGETVIDEOMODEINFO:{
		auto rwmode = (VideoMode*)arg0;
		rwmode->width = 640;
		rwmode->height = 480;
		rwmode->depth = 16;
		rwmode->flags = VIDEOMODEEXCLUSIVE;
		return 1;
	}
		

	case DEVICEGETMAXMULTISAMPLINGLEVELS:
		{
			return 1;
		}
	case DEVICEGETMULTISAMPLINGLEVELS:
		return 1;
	case DEVICESETMULTISAMPLINGLEVELS:
		return 1;
	case DEVICESETSUBSYSTEM:
		return 1;
	case DEVICEGETNUMVIDEOMODES:
		return 1;
	case DEVICEGETCURRENTVIDEOMODE:
		return 0;
	case DEVICESETVIDEOMODE:
		return 1;
	default:
		assert(0 && "not implemented");
		return 0;
	}
	return 1;
}

Device renderdevice = {
	999.0f, 0.0f,
	beginUpdate,
	endUpdate,
	clearCamera,
	showRaster,
	rasterRenderFast,
	setRenderState,
	getRenderState,
	im2DRenderLine,
	im2DRenderTriangle,
	im2DRenderPrimitive,
	im2DRenderIndexedPrimitive,
	im3DTransform,
	im3DRenderPrimitive,
	im3DRenderIndexedPrimitive,
	im3DEnd,
	deviceSystem
};

static pvr_init_params_t pvr_params = {
	.opb_sizes = {
				PVR_BINSIZE_16, PVR_BINSIZE_0, PVR_BINSIZE_8, PVR_BINSIZE_0,
				PVR_BINSIZE_8
	},
	.vertex_buf_size = (1024 + 1024) * 1024,
	.autosort_disabled = true,
	.opb_overflow_count = 7 // 268800 bytes
};

void defaultInstance(ObjPipeline *pipe, Atomic *atomic) {
	#if defined(DC_TEXCONV)
		processGeom(atomic->geometry);
	#else
		assert(0 && "defaultInstanceCB not implemented");
	#endif
}
void defaultUninstance(ObjPipeline *pipe, Atomic *atomic) {
	assert(0 && "defaultUninstanceCB not implemented");
}

ObjPipeline* makeDefaultPipeline(void)
{
	ObjPipeline *pipe = rw::ObjPipeline::create();
    pipe->init(PLATFORM_DC);
	pipe->impl.instance = defaultInstance;
	pipe->impl.uninstance = defaultUninstance;
	pipe->impl.render = defaultRenderCB;
	return pipe;
}

static void*
driverOpen(void *o, int32, int32)
{
    pvr_init(&pvr_params);

	fake_tex = pvr_mem_malloc(sizeof(fake_tex_data));

	#if !defined(DC_SIM)
	pvr_txr_load(fake_tex_data, fake_tex, sizeof(fake_tex_data));
	#else
	memcpy(fake_tex, fake_tex_data, sizeof(fake_tex_data));
	#endif

	PVR_SET(PVR_TEXTURE_MODULO, 640/32);
	PVR_SET(PVR_OBJECT_CLIP, 0x3f800000);
	DCE_InitMatrices();

	engine->driver[PLATFORM_DC]->defaultPipeline = makeDefaultPipeline();

	engine->driver[PLATFORM_DC]->rasterNativeOffset = nativeRasterOffset;
	engine->driver[PLATFORM_DC]->rasterCreate = rasterCreate;
	engine->driver[PLATFORM_DC]->rasterLock = rasterLock;
	engine->driver[PLATFORM_DC]->rasterUnlock = rasterUnlock;
	engine->driver[PLATFORM_DC]->rasterLockPalette = rasterLockPalette;
	engine->driver[PLATFORM_DC]->rasterUnlockPalette = rasterUnlockPalette;
	engine->driver[PLATFORM_DC]->rasterNumLevels = rasterNumLevels;
	engine->driver[PLATFORM_DC]->imageFindRasterFormat = imageFindRasterFormat;
	engine->driver[PLATFORM_DC]->rasterFromImage = rasterFromImage;
	engine->driver[PLATFORM_DC]->rasterToImage = rasterToImage;

	return o;
}

static void*
driverClose(void *o, int32, int32)
{
	pvr_mem_free(fake_tex);

	return o;
}


void
registerPlatformPlugins(void)
{
	Driver::registerPlugin(PLATFORM_DC, 0, PLATFORM_DC,
	                       driverOpen, driverClose);

    // TODO: this ?
	registerNativeRaster();
}

#define DC_TEXTURE_VERSION 4

Texture*
readNativeTexture(Stream *stream)
{
	if(!findChunk(stream, ID_STRUCT, nil, nil)){
		RWERROR((ERR_CHUNK, "STRUCT"));
		return nil;
	}
	auto platform = stream->readU32();
	assert(platform == PLATFORM_DC);
	auto version = stream->readU32();
	assert(version == DC_TEXTURE_VERSION);
    
	Texture *tex = Texture::create(nil);
	if(tex == nil)
		return nil;

	tex->filterAddressing = stream->readU32();
	stream->read8(tex->name, 32);
	stream->read8(tex->mask, 32);

	// Raster
	uint32 format = stream->readU32();
	int32 width = stream->readU16();
	int32 height = stream->readU16();
	int32 depth = stream->readU8();
	int32 numLevels = stream->readU8();
	int32 type = stream->readU8();
	uint32 pvr_flags = stream->readU32();
	uint32 pvr_size = stream->readU32();
	uint32 pvr_offs = stream->readU32();
	uint32 pvr_id = stream->readU32();

	auto raster = Raster::create(width, height, depth, format | type);
	assert(numLevels == 1);

	auto natras = GETDCRASTEREXT(raster);
	
	auto cached = cachedRasters.find(pvr_id);

	if (pvr_id != 0 && cached != cachedRasters.end()) {
		cached->second->refs++;
		natras->raster = cached->second;
		stream->seek(pvr_size);
		printf("Raster reused for texture %s\n", tex->name);
	} else {
		natras->raster = (DcRaster*)malloc(sizeof(DcRaster));
		memset(natras->raster, 0, sizeof(DcRaster));

		natras->raster->refs = 1;
		natras->raster->pvr_flags = pvr_flags;
		natras->raster->texaddr = allocTexture(natras->raster, pvr_size);
		natras->raster->texoffs = pvr_offs;
		natras->raster->pvr_id = pvr_id;
		natras->raster->native = true;

		natras->raster->u = __builtin_ctz(width) - 3;
		natras->raster->v = __builtin_ctz(height) - 3;

		cachedRasters.insert(std::make_pair(pvr_id, natras->raster));

		if (natras->raster->texaddr) {
			assert(!(pvr_size&3));

	#ifdef DC_SH4
			uint8 *src = stream->mmap(pvr_size);
			assert((pvr_size & 31) == 0);
			if (src) {
				if (((uintptr_t)src & 31)) {
					dcache_pref_block(src);

					if ((uintptr_t)src & 3) {
						uint8 *data = (uint8*)natras->raster->texaddr;
						for (int i = 0; i < pvr_size; i += 4) {
							(uint32_t&)data[i] = src[i + 3] << 24 | src[i + 2] << 16 | src[i + 1] << 8 | src[i];
						}
					}
					else {
						memcpy(natras->raster->texaddr, src, pvr_size);
					}
				}
				else {
					/*
					Doesn't need flush data cache with GD DMA streams
					when PR #763 will be merged to KOS.
					*/
					dcache_flush_range((uintptr_t)src, pvr_size);
					/* 
					TODO: Replace to PVR-if DMA on rootbus instead of TA DMA on TA bus.
					We don't need to act faster than the G1 bus here,
					but this will free up the TA bus for rendering.
					*/
					while (pvr_txr_load_dma(src, natras->raster->texaddr, pvr_size, 0, NULL, NULL) < 0) {
						thd_pass();
					}
				}
			}
			else
	#endif
			{
				// TODO: Direct DMA for a file
				uint8 *data = (uint8*)natras->raster->texaddr;
				for (int i = 0; i < pvr_size; i += 4) {
					uint32_t textel;
					stream->read32(&textel, 4);
					(uint32_t&)data[i] = textel;
				}
			}
		} else {
			stream->seek(pvr_size);
			printf("Failed to allocate raster pixels for texture %s\n", tex->name);
		}
	}

	tex->raster = raster;

	return tex;
}

#if defined(DC_TEXCONV)
void
writeNativeTexture(Texture *tex, Stream *stream)
{
	auto fheader = stream->tell();
	// size will be written later
	writeChunkHeader(stream, ID_STRUCT, 4);
	auto fbegin = stream->tell();

	stream->writeU32(PLATFORM_DC);
	stream->writeU32(DC_TEXTURE_VERSION);

	// Texture
	stream->writeU32(tex->filterAddressing);
	stream->write8(tex->name, 32);
	stream->write8(tex->mask, 32);

	// Raster
	Raster *raster = tex->raster;
	auto natras = GETDCRASTEREXT(raster);

	int32 numLevels = raster->getNumLevels();
	stream->writeI32(raster->format);
	stream->writeU16(raster->width);
	stream->writeU16(raster->height);
	stream->writeU8(raster->depth);
	stream->writeU8(numLevels);
	stream->writeU8(raster->type);
	stream->writeU32(natras->raster->pvr_flags);
	stream->writeU32(natras->raster->texsize);
	stream->writeU32(natras->raster->texoffs);
	stream->writeU32(natras->raster->pvr_id);
	
	assert(numLevels == 1);

	uint32 size = raster->width * raster->height * 2;
	stream->write8((uint8 *)natras->raster->texaddr, natras->raster->texsize);

	// rewrite header with correct size
	auto fend = stream->tell();
	stream->seek(fheader, 0);
	writeChunkHeader(stream, ID_STRUCT, fend - fbegin);
	stream->seek(fend, 0);
}
#endif

#define DC_MODEL_VERSION 5

void*
destroyNativeData(void *object, int32, int32)
{
	auto geo = (Geometry*)object;
	rwFree(geo->instData);
	geo->instData = nil;

	return object;
}

Stream*
readNativeData(Stream *stream, int32 length, void *object, int32, int32)
{
	auto geo = (Geometry*)object;
	
	uint32 chunkLen;
	if(!findChunk(stream, ID_STRUCT, &chunkLen, nil)){
		RWERROR((ERR_CHUNK, "STRUCT"));
		return nil;
	}

	DCModelDataHeader *header = (DCModelDataHeader *)rwNew(sizeof(DCModelDataHeader) + chunkLen - 8, MEMDUR_EVENT | ID_GEOMETRY);
	geo->instData = header;
	stream->read32(&header->platform, 4);
	uint32_t version;
	stream->read32(&version, 4);
	assert(version == DC_MODEL_VERSION);
	header->size = chunkLen - 8;
	stream->read8(header->data, header->size);

	return stream;
}


Stream*
writeNativeData(Stream *stream, int32 len, void *object, int32, int32)
{
	auto geo = (Geometry*)object;
	auto instData = (DCModelDataHeader*)geo->instData;
	assert(instData != nil);
	auto fheader = stream->tell();
	// size will be written later
	rw::writeChunkHeader(stream, ID_STRUCT, 0);
	auto fbegin = stream->tell();

	stream->write32(&instData->platform, 4);
	uint32_t version = DC_MODEL_VERSION;
	stream->write32(&version, 4);

	stream->write8(instData->data, instData->size);

	// rewrite header with correct size
	auto fend = stream->tell();
	stream->seek(fheader, 0);
	rw::writeChunkHeader(stream, ID_STRUCT, fend - fbegin);
	stream->seek(fend, 0);

	return nullptr;
}

int32
getSizeNativeData(void *object, int32 offset, int32 size)
{
	auto geo = (Geometry*)object;
	auto instData = (DCModelDataHeader*)geo->instData;
	return instData ? instData->size : 0;
}
static void* createNativeRaster(void *object, int32 offset, int32) {
	auto natras = GETDCRASTEREXT(object);
	memset(natras, 0, sizeof(DcRasterRef));

    return object;
}

static void*
destroyNativeRaster(void *object, int32 offset, int32) {
    auto rs = (Raster*)object;
	auto natras = GETDCRASTEREXT(rs);
    
	if (natras->raster && --natras->raster->refs == 0) {
		logf("destroyNativeRaster: %p: destroying texture @ %p\n", natras->raster, natras->raster->texaddr);
		if (natras->raster->texaddr) {
			alloc_free(natras->raster->texaddr);
			logf("destroyNativeRaster: freed %d x %d texture @ 0x%08X\n", rs->width, rs->height, natras->raster->texaddr);
			// pvr_mem_stats();
			// #if !defined(MACOS64)
			// malloc_stats();
			// #endif
		}
		cachedRasters.erase(natras->raster->pvr_id);
		free(natras->raster);
	}
	
	return object;
}

static void*
copyNativeRaster(void *dst, void *src, int32 offset, int32)
{
	auto dstraster = GETDCRASTEREXT(dst);
	auto srcraster = GETDCRASTEREXT(src);
	*dstraster = *srcraster;
	return dst;
}

int32 nativeRasterOffset;

#if defined(DC_TEXCONV)
static int32
replaceID(int32 *ids, int32 num, int32 from, int32 to)
{
	int32 n = 0;
	for(int32 i = 0; i < num; i++)
		if(ids[i] == from){
			ids[i] = to;
			n++;
		}
	return n;
}

// find connected groups of triangles and assign vertices to groups
static int32*
findGroupIDs(Geometry *g)
{
	int32 i;
	int32 *ids = rwNewT(int32, g->numVertices, 0);
	for(i = 0; i < g->numVertices; i++)
		ids[i] = -1;

	// assign initial IDs
	int32 nextID = 0;
	for(i = 0; i < g->numTriangles; i++){
		int v1 = g->triangles[i].v[0];
		int v2 = g->triangles[i].v[1];
		int v3 = g->triangles[i].v[2];

		int32 id =
			ids[v1] != -1 ? ids[v1] :
			ids[v2] != -1 ? ids[v2] :
			ids[v3] != -1 ? ids[v3] : nextID++;
		if(ids[v1] != id && ids[v1] != -1)
			replaceID(ids, g->numVertices, ids[v1], id);
		ids[v1] = id;
		if(ids[v2] != id && ids[v2] != -1)
			replaceID(ids, g->numVertices, ids[v2], id);
		ids[v2] = id;
		if(ids[v3] != id && ids[v3] != -1)
			replaceID(ids, g->numVertices, ids[v3], id);
		ids[v3] = id;
	}

	// ID range can have gaps now, compress
	int32 numIDs = nextID;
	nextID = 0;
	for(i = 0; i < numIDs; i++)
		if(replaceID(ids, g->numVertices, i, nextID) > 0)
			nextID++;

	return ids;
}

// TODO: be more careful about materials and texture wrapping modes
void
centerTexCoords(Geometry *g)
{
	if(g->flags & Geometry::NATIVE)
		return;

	int32 *groupIDs = findGroupIDs(g);

	for(int32 k = 0; k < g->numTexCoordSets; k++)
	for(int32 id = 0;; id++){
		float minU = 100000.0f;
		float minV = 100000.0f;
		float maxU = -100000.0f;
		float maxV = -100000.0f;
		int n = 0;

		for(int32 i = 0; i < g->numVertices; i++)
			if(groupIDs[i] == id){
				float u = g->texCoords[k][i].u;
				float v = g->texCoords[k][i].v;
				minU = std::min(minU, u);
				minV = std::min(minV, v);
				maxU = std::max(maxU, u);
				maxV = std::max(maxV, v);
				n++;
			}
		if(n == 0)
			break;

		float midU = (int)((maxU+minU)/2.0f);
		float midV = (int)((maxV+minV)/2.0f);
		for(int32 i = 0; i < g->numVertices; i++)
			if(groupIDs[i] == id){
				g->texCoords[k][i].u -= midU;
				g->texCoords[k][i].v -= midV;
			}
	}

	rwFree(groupIDs);
}

bool isDegenerate(const V3d& v1, const V3d& v2, const V3d& v3) {
    V3d u = {v2.x - v1.x, v2.y - v1.y, v2.z - v1.z};
    V3d v = {v3.x - v1.x, v3.y - v1.y, v3.z - v1.z};
	V3d crs = cross(u, v);
	if (length(crs) < 0.0000001f) {
		return true;
	} else {
		return false;
	}
}

bool isDegenerateByIndex(uint16_t idx1, uint16_t idx2, uint16_t idx3) {
	return idx1 == idx2 || idx1 == idx3 || idx2 == idx3;
}
static int8_t packFloat8(float f) {
	auto rounded =  lroundf(f * 127);

	assert(rounded >= -127 && rounded <= 127);

	return static_cast<int8_t>(rounded);
}

static int16_t packFloat16(float f) {
	auto rounded =  lroundf(f * 32767);

	assert(rounded >= -32767 && rounded <= 32767);

	return static_cast<int16_t>(rounded);
}
struct write_vector: std::vector<uint8_t> {
	template<typename T>
	void write(const T& val) {
		const uint8_t* p = (const uint8_t*)&val;
		insert(end(), p, p + sizeof(T));
	}

	template<typename T>
	void rewrite(size_t offset, const T& val) {
		const uint8_t* p = (const uint8_t*)&val;
		std::copy(p, p + sizeof(T), begin() + offset);
	}

	void packVertex(RwSphere* volume, V3d* vertex, TexCoords* texcoord, V3d* normal, RGBA* color, bool big_vertex, bool pad_xyz, bool big_uv) {
		if (big_vertex) {
			write<float>(vertex->x);
			write<float>(vertex->y);
			write<float>(vertex->z);
		} else {
			write<int16_t>(packFloat16((vertex->x - volume->center.x) / volume->radius));
			write<int16_t>(packFloat16((vertex->y - volume->center.y) / volume->radius));
			write<int16_t>(packFloat16((vertex->z - volume->center.z) / volume->radius));
			
			if (pad_xyz) {
				write<int16_t>(0);
			}
		}

		if (texcoord) {
			if (big_uv) {
				float16 u = texcoord->u;
				float16 v = texcoord->v;
				write<uint32_t>((u.raw << 16) | v.raw);
			} else {
				write<int8_t>(lroundf(texcoord->u * 127));
				write<int8_t>(lroundf(texcoord->v * 127));
			}
		}

		if (normal) {
			auto normaly = normalize(*normal);
			if (length(*normal) < 0.0001) {
				normaly.x = 1;
				normaly.y = 0;
				normaly.z = 0;
				printf("*WARNING* invalid normal\n");
			}
			
			int8_t nxi = lroundf(normaly.x * 127);
			int8_t nyi = lroundf(normaly.y * 127);
			int8_t nzi = lroundf(normaly.z * 127);
			
			V3d normal2 = { static_cast<float32>(nxi), static_cast<float32>(nyi), static_cast<float32>(nzi) };

			assert(fabs(normaly.x-normal2.x/127) < 0.05f);
			assert(fabs(normaly.y-normal2.y/127) < 0.05f);
			assert(fabs(normaly.z-normal2.z/127) < 0.05f);

			write<int8_t>(nxi);
			write<int8_t>(nyi);
			write<int8_t>(nzi);
			write<int8_t>(0);
		}
		
		if (color) {
			write<uint32_t>((color->blue ^ 128) | ((color->green ^ 128) << 8) | ((color->red ^ 128) << 16) | ((color->alpha ^ 128) << 24));
		}
	}
};

bool isBigVertex(const V3d& vtx) {
	return vtx.x > 127 || vtx.y > 127 || vtx.z > 127 || vtx.x < -127 || vtx.y < -127 || vtx.z < -127;
}

bool isBigUV(const TexCoords& uv) {
	return uv.u > 1 || uv.v > 1 || uv.u < -1 || uv.v < -1;
}

void adjustFlagsForAlingment(bool textured, bool colored, bool& big_vertex, bool& big_uv, bool& pad_xyz) {
	pad_xyz = false;

	if (textured) {
		if (big_vertex && !big_uv) {
			big_uv = true;
		}

		if (!big_vertex && big_uv) {
			pad_xyz = true;
		}
	}
}

unsigned caluclateVertexAlignment(bool textured, bool normaled, bool colored, bool big_vertex, bool big_uv, bool pad_xyz) {
	if (big_vertex) {
		return 4;
	} else if (textured && big_uv) {
		return 4;
	} else {
		return 2;
	}
}
unsigned caluclateVertexSize(bool textured, bool normaled, bool colored, bool big_vertex, bool big_uv, bool pad_xyz) {
	uint32_t vertexBytes = 0; //xyz

	if (big_vertex) {
		vertexBytes += 4 * 3;
	} else {
		vertexBytes += 2 * 3;

		if (pad_xyz) {
			vertexBytes += 2;
		}
	}

	if (textured) {
		if (big_uv) {
			vertexBytes += 4;
		} else {
			vertexBytes += 2;
		}
	}

	if (normaled) {
		vertexBytes += 4;
	}

	if (colored) {
		vertexBytes += 4;
	}

	return vertexBytes;
}

RwSphere calculateBoundingSphere(V3d* vertexData, size_t count) {
	RwSphere sphere;
	sphere.center = {0, 0, 0};
	sphere.radius = 0;

	for (size_t i = 0; i < count; i++) {
		sphere.center = add(sphere.center, vertexData[i]);
	}

	sphere.center.x /= count;
	sphere.center.y /= count;
	sphere.center.z /= count;

	for (size_t i = 0; i < count; i++) {
		sphere.radius = std::max(sphere.radius, length(sub(vertexData[i], sphere.center)));
	}

	return sphere;
}
struct meshlet {
	std::set<uint16_t> vertices;
	std::map<uint16_t, uint8_t> vertexToLocalIndex;
	std::vector<triangle_stripper::primitive_group*> strips;
	size_t vertexDataOffset;
	size_t indexDataOffset;
	size_t skinIndexDataOffset;
	size_t skinWeightDataOffset;
	size_t rewriteOffsetVDO;
	size_t rewriteOffsetIDO;
	size_t rewriteOffsetSIDO;
	size_t rewriteOffsetSWDO;

	bool isOfBigVertex(V3d* vertexData, Sphere* volume) {
		for (auto v : vertices) {
			if (isBigVertex(sub(vertexData[v], volume->center))) {
				return true;
			}
		}
		return false;
	}

	bool isOfBigUV(TexCoords* uvData) {
		for (auto v : vertices) {
			if (isBigUV(uvData[v])) {
				return true;
			}
		}
		return false;
	}

	RwSphere calculateBoundingSphere(V3d* vertexData) {
		RwSphere sphere;
		sphere.center = {0, 0, 0};
		sphere.radius = 0;

		for (auto v : vertices) {
			sphere.center = add(sphere.center, vertexData[v]);
		}

		sphere.center.x /= vertices.size();
		sphere.center.y /= vertices.size();
		sphere.center.z /= vertices.size();

		for (auto v : vertices) {
			sphere.radius = std::max(sphere.radius, length(sub(vertexData[v], sphere.center)));
		}

		return sphere;
	}
};

void processGeom(Geometry *geo) {
	centerTexCoords(geo);
	using namespace triangle_stripper;

	assert(geo->instData == nil);
	Skin* skin = Skin::get(geo);
	
	int32 n = geo->meshHeader->numMeshes;
	auto meshes = geo->meshHeader->getMeshes();
	std::vector<primitive_vector> pvecs(n);
	std::vector<std::vector<meshlet>> meshMeshlets(n);

	size_t totalIndices = 0, strips = 0,  totalTrilist = 0;

	bool colored = geo->flags & Geometry::PRELIT;
	bool normaled = geo->flags & Geometry::NORMALS;
	bool texcoorded = geo->numTexCoordSets > 0;
	assert(geo->numMorphTargets == 1);
	assert(geo->numTexCoordSets == 0 || geo->numTexCoordSets == 1);

	geo->morphTargets[0].boundingSphere = calculateBoundingSphere(geo->morphTargets[0].vertices, geo->numVertices);

	auto vertices = geo->morphTargets[0].vertices;
	auto normals = geo->morphTargets[0].normals;
	auto texcoords = geo->texCoords[0];
	auto colors = geo->colors;

	V4d* skinWeights = nullptr;
	uint32_t* skinIndices = nullptr;

	if (skin) {
		skinWeights = (V4d*)skin->weights;
		skinIndices = (uint32_t*)skin->indices;
	}


	std::vector<size_t> canonicalIdx(geo->numVertices, SIZE_MAX);
	for (size_t i = 0; i < geo->numVertices; i++) {
		for (size_t j = i+1; j < geo->numVertices; j++) {
			bool duplicate = true;
			if (vertices[i].x != vertices[j].x || vertices[i].y != vertices[j].y || vertices[i].z != vertices[j].z) {
				duplicate = false;
			}

			if (texcoorded && (texcoords[i].u != texcoords[j].u || texcoords[i].v != texcoords[j].v)) {
				duplicate = false;
			}
			if (normaled && (normals[i].x != normals[j].x || normals[i].y != normals[j].y || normals[i].z != normals[j].z)) {
				duplicate = false;
			}
			if (colored && (colors[i].red != colors[j].red || colors[i].green != colors[j].green || colors[i].blue != colors[j].blue || colors[i].alpha != colors[j].alpha)) {
				duplicate = false;
			}
			if (duplicate) {
				// texconvf("Vertex %zu: %.2f %.2f %.2f deemed dup with %zu: %.2f %.2f %.2f\n", i, vertices[i].x, vertices[i].y, vertices[i].z, j, vertices[j].x, vertices[j].y, vertices[j].z);
				if(canonicalIdx[i] == SIZE_MAX) {
					if (canonicalIdx[j] != SIZE_MAX) {
						canonicalIdx[i] = canonicalIdx[j];
						// texconvf("Warning: Duplicate Chain vertex %zu and %zu\n", i, j);
					} else {
						canonicalIdx[i] = i;
						canonicalIdx[j] = i;
						// texconvf("Warning: Duplicate vertex %zu and %zu\n", i, j);
					}
				} else {
					canonicalIdx[j] = canonicalIdx[i];
					// texconvf("Warning: Duplicate Double vertex %zu and %zu\n", i, j);
				}
			}
		}
		if(canonicalIdx[i] == SIZE_MAX) {
			canonicalIdx[i] = i;
		}
	}

	size_t dups = 0;
	for (size_t i = 0; i < geo->numVertices; i++) {
		if (canonicalIdx[i] != i) {
			dups++;
		}
	}
	texconvf("Found %zu vertex duplicates, %.2f%%\n", dups, (float)dups/geo->numVertices*100);
	for (int meshNum = 0; meshNum < n; meshNum++) {
		auto mesh = &meshes[meshNum];
		
		totalTrilist += mesh->numIndices;

		std::vector<uint16_t> idx_unstrip;			
		uint16_t* oldIndices = mesh->indices;
		uint32_t oldNumIndices = mesh->numIndices;

		if (geo->meshHeader->flags & rw::MeshHeader::TRISTRIP) {			
			for (int i = 0; i < mesh->numIndices-2; i++) {
				if (isDegenerateByIndex(mesh->indices[i], mesh->indices[i+1], mesh->indices[i+2])) {
					continue; // Skip to the next index
				}
				
				if ((i & 1) == 0) {
					idx_unstrip.push_back(mesh->indices[i]);
					idx_unstrip.push_back(mesh->indices[i+1]);
					idx_unstrip.push_back(mesh->indices[i+2]);
				} else {
					idx_unstrip.push_back(mesh->indices[i]);
					idx_unstrip.push_back(mesh->indices[i+2]);
					idx_unstrip.push_back(mesh->indices[i+1]);
				}
				
			}

			mesh->indices = idx_unstrip.data();
			mesh->numIndices = idx_unstrip.size();
		}

		for (size_t i = 0; i < mesh->numIndices; i++) {
			mesh->indices[i] = canonicalIdx[mesh->indices[i]];
		}

		{
			indices Indices(mesh->indices, mesh->indices + mesh->numIndices);

			tri_stripper TriStripper(Indices);

			TriStripper.SetMinStripSize(0);
			TriStripper.SetCacheSize(0);
			TriStripper.SetBackwardSearch(true);

			TriStripper.Strip(&pvecs[meshNum]);
		}

		mesh->indices = oldIndices;
		mesh->numIndices = oldNumIndices;

		for (auto &&strip: pvecs[meshNum]) {
			totalIndices += strip.Indices.size();
			if (strip.Type == TRIANGLES) {
				assert(strip.Indices.size()%3==0);
				strips += strip.Indices.size()/3;
			} else {
				strips ++;
			}
		}
	}
	#undef printf
	texconvf("%s: Down to %ld strips (%.2f), %ld indices from %ld (vtx: %d)\n", currentFile, strips, (float)totalIndices/strips, totalIndices, totalTrilist, geo->numVertices);

	// construct meshlets
	
	size_t meshIndexesCount = 0;
	size_t meshVerticesCount = 0;
	size_t meshletIndexesCount = 0;
	size_t meshletVerticesCount = 0;
	for (int pvn = 0; pvn < pvecs.size(); pvn++) {
		auto &&prims = pvecs[pvn];

		std::set<uint16_t> meshletVertices;
		std::vector<primitive_group*> meshletStrips;

		std::list<primitive_group*> strips;
		for (auto &&strip: prims) {
			strips.push_back(&strip);
		}
		#undef printf

		while(strips.size()) {
			for(;;) {
				// pluck strip with fewest new indices

				primitive_group* bestStrip = nullptr;

				size_t remainingVertices = 128 - meshletVertices.size();
				size_t bestSharedVertices = 0;

				for (auto &&strip_ptr: strips) {
					auto &&strip = *strip_ptr;
					std::set<uint16_t> newVertices;
					size_t sharedVertices = 0;
					for (auto &&idx: strip.Indices) {
						if (meshletVertices.find(idx) == meshletVertices.end()) {
							newVertices.insert(idx);
						} else {
							sharedVertices++;
						}
					}
					if (newVertices.size() == 0) {
						bestStrip = strip_ptr;
						break;
					}
					if (newVertices.size() <= remainingVertices && sharedVertices >= bestSharedVertices) {
						bestStrip = strip_ptr;
						bestSharedVertices = sharedVertices;
					}
				}

				if (bestStrip == nullptr) {
					break;
				}

				// add strip to meshlet
				meshletStrips.push_back(bestStrip);
				for (auto &&idx: bestStrip->Indices) {
					meshletVertices.insert(idx);
				}
				strips.remove(bestStrip);
			}

			assert(meshletStrips.size() != 0);

			// printf("Meshlet constructed, %ld strips, %zu vertices\n", meshletStrips.size(), meshletVertices.size());
			for (auto &&strip: meshletStrips) {
				meshletIndexesCount += strip->Indices.size();
			}
			meshletVerticesCount += meshletVertices.size();

			meshMeshlets[pvn].push_back(meshlet{meshletVertices, {}, meshletStrips, 0, 0});

			uint8_t localIndex = 0;
			for (auto &&idx: meshletVertices) {
				meshMeshlets[pvn].back().vertexToLocalIndex[idx] = localIndex++;
			}

			assert(localIndex <= 128);

			meshletStrips.clear();
			meshletVertices.clear();
		}

		std::set<uint16_t> meshVertices;
		for (auto &&strip: prims) {
			meshIndexesCount += strip.Indices.size();
			for (auto &&idx: strip.Indices) {
				meshVertices.insert(idx);
			}
		}
		meshVerticesCount += meshVertices.size();
	}
	texconvf("%s: %zu; %.2f; Meshlets complete %zu vertices %zu indexes from %zu vertices %zu indexes\n", currentFile, meshletVerticesCount - meshVerticesCount, (float)(meshletVerticesCount - meshVerticesCount)/meshVerticesCount, meshletVerticesCount, meshletIndexesCount, meshVerticesCount, meshIndexesCount);

	write_vector meshData;
	write_vector meshletData;
	write_vector vertexData;
	write_vector indexData;
	write_vector skinningIndexData;
	write_vector skinningWeightData;

	for (size_t i = 0; i < meshMeshlets.size(); i++) {
		auto &&mesh = meshMeshlets[i];
		
		assert(mesh.size() <= 32767);
		meshData.write<int16_t>(mesh.size());

		assert((meshletData.size() + meshMeshlets.size() * 4) <= 32767);
		meshData.write<int16_t>(meshletData.size() + meshMeshlets.size() * 4);

		for (auto && meshlet: mesh) {
			auto boundingSphere = meshlet.calculateBoundingSphere(vertices);

			uint32_t totalIndexes = 0;
			for(auto&& strip: meshlet.strips) {
				totalIndexes += strip->Indices.size();
			}

			// write out vertex data

			bool big_vertex = meshlet.isOfBigVertex(vertices, &boundingSphere);
			bool big_uv = texcoorded && meshlet.isOfBigUV(texcoords);
			bool pad_xyz;
			adjustFlagsForAlingment(texcoorded, colored, big_vertex, big_uv, pad_xyz);
			uint8_t vertexSize = caluclateVertexSize(texcoorded, normaled, colored, big_vertex, big_uv, pad_xyz);
			uint8_t vertexAlignment = caluclateVertexAlignment(texcoorded, normaled, colored, big_vertex, big_uv, pad_xyz);

			assert(vertexSize % vertexAlignment == 0);
			assert(vertexData.size() % vertexAlignment == 0);

			meshlet.vertexDataOffset = vertexData.size();

			for (auto &&idx: meshlet.vertices) {
				vertexData.packVertex(&boundingSphere, &vertices[idx], texcoorded ? &texcoords[idx] : nullptr, normaled ? &normals[idx] : nullptr, colored ? &colors[idx] : nullptr, big_vertex, pad_xyz, big_uv);
			}

			// write out index data
			meshlet.indexDataOffset = indexData.size();
			
			for(auto&& strip: meshlet.strips) {
				if (strip->Type == TRIANGLES) {
					for (size_t i = 0; i < strip->Indices.size(); i+=3) {
						indexData.write<uint8_t>(meshlet.vertexToLocalIndex[strip->Indices[i]]);
						indexData.write<uint8_t>(meshlet.vertexToLocalIndex[strip->Indices[i+1]]);
						indexData.write<uint8_t>(meshlet.vertexToLocalIndex[strip->Indices[i+2]] | 128);
					}
				} else {
					for (size_t i = 0; i < strip->Indices.size(); i++) {
						indexData.write<uint8_t>(meshlet.vertexToLocalIndex[strip->Indices[i]] | ((i + 1) == strip->Indices.size() ? 128 : 0));
					}
				}
			}

			// write out skinning data
			if (!skin) {
				meshlet.skinIndexDataOffset = SIZE_MAX;
				meshlet.skinWeightDataOffset = SIZE_MAX;
			} else {
				meshlet.skinIndexDataOffset = skinningIndexData.size();
				meshlet.skinWeightDataOffset = skinningWeightData.size();

				std::vector<std::pair<uint8_t, uint8_t>> skinMatrixVtx[64];
				std::vector<uint8_t> skinMatrix0Only;

				for (auto &&idx: meshlet.vertices) {
					auto weights = (float*)&skinWeights[idx];
					auto indices = (uint8_t*)&skinIndices[idx];

					for (int mtxIndex = 0; mtxIndex < 4; mtxIndex++) {
						uint8_t quantWeight = weights[mtxIndex] * 255;
						if (quantWeight == 255 && indices[mtxIndex] == 0) {
							skinMatrix0Only.push_back(meshlet.vertexToLocalIndex[idx]);
							continue;
						}

						if (quantWeight > 0) {
							skinMatrixVtx[indices[mtxIndex]].push_back({meshlet.vertexToLocalIndex[idx], quantWeight});
						}
					}
				}

				size_t expectedIdx = 0;
				size_t spanStartIdx = 0;
				size_t spanCount = 0;

				size_t skinningIndexDataStart = skinningIndexData.size();

				for (auto &&idx: skinMatrix0Only) {
					if (idx == expectedIdx) {
						expectedIdx++;
						spanCount++;
						continue;
					}
					// not matching
					if (spanCount) {
						skinningIndexData.write<uint16_t>(0x0000 | spanStartIdx * vertexSize);	// src offset + transform flag
						skinningIndexData.write<uint16_t>(spanCount);					// count
						skinningIndexData.write<uint16_t>(spanStartIdx * 64);			// dst offset
						assert(spanStartIdx + spanCount <= meshlet.vertices.size());
						spanCount = 0;
					}

					spanStartIdx = expectedIdx;

					while (expectedIdx != meshlet.vertices.size() && idx != expectedIdx) {
						expectedIdx++;
						spanCount++;
					}

					assert(spanCount);
					
					skinningIndexData.write<uint16_t>(0x8000 | spanCount);			// count + clear flag
					skinningIndexData.write<uint16_t>(spanStartIdx * 64);			// dst offset
					assert(spanStartIdx + spanCount <= meshlet.vertices.size());

					if (expectedIdx == meshlet.vertices.size()) {
						spanStartIdx += spanCount;
						spanCount = 0;
						break;
					}
					spanStartIdx = expectedIdx;
					expectedIdx++;
					spanCount = 1;
				}

				// last span, if any
				if (spanCount) {
					skinningIndexData.write<uint16_t>(0x0000 | spanStartIdx * vertexSize);	// src offset + transform flag
					skinningIndexData.write<uint16_t>(spanCount);					// count
					skinningIndexData.write<uint16_t>(spanStartIdx * 64);			// dst offset
					spanCount = 0;
				}

				spanStartIdx = expectedIdx;

				while (expectedIdx != meshlet.vertices.size()) {
					expectedIdx++;
					spanCount++;
				}
				if (spanCount) {
					skinningIndexData.write<uint16_t>(0x8000 | spanCount);			// count + clear flag
					skinningIndexData.write<uint16_t>(spanStartIdx * 64);			// dst offset
				}
				
				assert(spanStartIdx + spanCount == meshlet.vertices.size());

				skinningIndexData.write<uint16_t>(0x8080);	// end of list

				// validate skinningIndexData
				size_t readVtx;
				size_t currentMtx0Idx = 0;
				for(;;) {
					int16_t flags = skinningIndexData[skinningIndexDataStart] | (skinningIndexData[skinningIndexDataStart + 1] << 8);
					skinningIndexDataStart += 2;
					
					if (flags >= 0) {
						int startVtx = flags / vertexSize;
						int count = skinningIndexData[skinningIndexDataStart] | (skinningIndexData[skinningIndexDataStart + 1] << 8);
						skinningIndexDataStart += 2;
						int dstVertex = skinningIndexData[skinningIndexDataStart] | (skinningIndexData[skinningIndexDataStart + 1] << 8);
						skinningIndexDataStart += 2;
						texconvf("%s: Transform: start %d, count %d, dst %d\n", currentFile, startVtx, count, dstVertex/64);
						for (int k = 0; k < count; k++) {
							assert(skinMatrix0Only[currentMtx0Idx++] == (startVtx + k));
						}
					} else if (!(flags & 0x80)) {
						int count = flags & 0x7FFF;
						int dstVertex = skinningIndexData[skinningIndexDataStart] | (skinningIndexData[skinningIndexDataStart + 1] << 8);
						skinningIndexDataStart += 2;
						texconvf("%s: Clear: count %d, dst %d\n", currentFile, count, dstVertex/64);
					} else {
						texconvf("%s: End of list\n", currentFile);
						break;
					}
				}

				assert(currentMtx0Idx == skinMatrix0Only.size());

				for (int i = 0; i < skin->numBones; i++) {
					// some matrixes may be empty
					skinningIndexData.write<uint16_t>(skinMatrixVtx[i].size());	// only 8 bits used here
					for (auto &&idx: skinMatrixVtx[i]) {
						skinningIndexData.write<uint16_t>(idx.first * vertexSize);	// src offset
						skinningIndexData.write<uint16_t>(idx.first * 64);		// dst offset
						skinningWeightData.write<uint8_t>(idx.second);
					}
				}
				skinningIndexData.write<uint16_t>(0x8000);	// end of list

				texconvf("%s: Original skin size: %ld, packed: %ld\n", currentFile, meshlet.vertices.size() * 8, skinningIndexData.size() - meshlet.skinIndexDataOffset + skinningWeightData.size() - meshlet.skinWeightDataOffset);
				texconvf("%s: Skin matrix 0 only: %ld\n", currentFile, skinMatrix0Only.size());
				for (int i = 0; i < skin->numBones; i++) {
					if (skinMatrixVtx[i].size()) {
						texconvf("%s: Skin matrix %d: %ld\n", currentFile, i, skinMatrixVtx[i].size());
					}
				}
			}

			// write out meshlet data
			meshletData.write(boundingSphere);
			//isTextured, isNormaled, isColored, small_xyz, pad_xyz, small_uv
			uint16_t flags = texcoorded | (normaled << 1) | (colored << 2) | (!big_vertex << 3) | (pad_xyz << 4) | (!big_uv << 5);
			meshletData.write<uint16_t>(flags);
			meshletData.write<uint8_t>(0);
			//bool textured, bool normaled, bool colored, bool big_vertex, bool big_uv, bool pad_xyz
			meshletData.write<uint8_t>(vertexSize);
			assert(meshlet.vertices.size() <= 65535);
			meshletData.write<uint16_t>(meshlet.vertices.size());
			assert(totalIndexes <= 65535);
			meshletData.write<uint16_t>(totalIndexes);
			meshlet.rewriteOffsetVDO = meshletData.size();
			meshletData.write<uint32_t>(meshlet.vertexDataOffset); // will be patched
			meshlet.rewriteOffsetIDO = meshletData.size();
			meshletData.write<uint32_t>(meshlet.indexDataOffset); // will be patched

			if (skin) {
				meshlet.rewriteOffsetSIDO = meshletData.size();
				meshletData.write<uint32_t>(meshlet.skinIndexDataOffset); // will be patched

				meshlet.rewriteOffsetSWDO = meshletData.size();
				meshletData.write<uint32_t>(meshlet.skinWeightDataOffset); // will be patched
			}
		}
	}

	assert(skinningIndexData.size() % 2 == 0);

	bool isIdx8 = geo->numVertices < 256;

	auto dataSize = meshData.size() + meshletData.size() + vertexData.size() + skinningIndexData.size() + skinningWeightData.size() + indexData.size();

	auto vertexBase = meshData.size() + meshletData.size();
	auto skinIndexBase = vertexBase + vertexData.size();
	auto skinWeightBase = skinIndexBase + skinningIndexData.size();
	assert(skinWeightBase % 2 == 0);
	auto indexBase = skinWeightBase + skinningWeightData.size();

	for (auto&& mesh: meshMeshlets) {
		for (auto&& meshlet: mesh) {
			assert(vertexBase % 4 == 0);
			meshletData.rewrite<uint32_t>(meshlet.rewriteOffsetVDO, meshlet.vertexDataOffset + vertexBase);
			meshletData.rewrite<uint32_t>(meshlet.rewriteOffsetIDO, meshlet.indexDataOffset + indexBase);

			if (skin) {
				assert((meshlet.skinIndexDataOffset + skinIndexBase) %2 == 0);
				meshletData.rewrite<uint32_t>(meshlet.rewriteOffsetSIDO, meshlet.skinIndexDataOffset + skinIndexBase);
				meshletData.rewrite<uint32_t>(meshlet.rewriteOffsetSWDO, meshlet.skinWeightDataOffset + skinWeightBase);
			}
		}
	}

	DCModelDataHeader *header = (DCModelDataHeader *)rwNew(sizeof(DCModelDataHeader) + dataSize , MEMDUR_EVENT | ID_GEOMETRY);
	geo->instData = header;
	header->platform = PLATFORM_DC;
	header->size = dataSize;

	uint8_t* dataPtr = header->data;
	memcpy(dataPtr, meshData.data(), meshData.size());
	dataPtr += meshData.size();
	memcpy(dataPtr, meshletData.data(), meshletData.size());
	dataPtr += meshletData.size();
	memcpy(dataPtr, vertexData.data(), vertexData.size());
	dataPtr += vertexData.size();
	if (skin) {
		memcpy(dataPtr, skinningIndexData.data(), skinningIndexData.size());
		dataPtr += skinningIndexData.size();
		memcpy(dataPtr, skinningWeightData.data(), skinningWeightData.size());
		dataPtr += skinningWeightData.size();
	}
	memcpy(dataPtr, indexData.data(), indexData.size());
	dataPtr += indexData.size();

	assert(dataPtr - header->data == dataSize);
}
#endif

void registerNativeRaster(void)
{
	nativeRasterOffset = Raster::registerPlugin(sizeof(DcRasterRef),
	                                            ID_RASTERDC,
	                                            createNativeRaster,
	                                            destroyNativeRaster,
	                                            copyNativeRaster);
}

void
defaultSkinRenderCB(ObjPipeline *pipe, Atomic *atomic)
{
	defaultRenderCB(pipe, atomic);
}

ObjPipeline*
makeSkinPipeline(void)
{
	ObjPipeline *pipe = rw::ObjPipeline::create();
    pipe->init(PLATFORM_DC);
	pipe->impl.instance = defaultInstance;
	pipe->impl.uninstance = defaultUninstance;
	pipe->impl.render = defaultSkinRenderCB;
	pipe->pluginID = ID_SKIN;
	pipe->pluginData = 1;
	return pipe;
}
static void*
skinOpen(void *o, int32, int32)
{
	skinGlobals.pipelines[PLATFORM_DC] = makeSkinPipeline();
	return o;
}

static void*
skinClose(void *o, int32, int32)
{
// 	((ObjPipeline*)skinGlobals.pipelines[PLATFORM_DC])->groupPipeline->destroy();
// 	((ObjPipeline*)skinGlobals.pipelines[PLATFORM_DC])->groupPipeline = nil;
	skinGlobals.pipelines[PLATFORM_DC]->destroy();
	skinGlobals.pipelines[PLATFORM_DC] = nil;
	return o;
}

void
initSkin(void)
{
	Driver::registerPlugin(PLATFORM_DC, 0, ID_SKIN,
	                       skinOpen, skinClose);
}

Stream*
readNativeSkin(Stream *stream, int32, void *object, int32 offset) {
	Geometry *geometry = (Geometry*)object;
	uint32 platform;
	if(!findChunk(stream, ID_STRUCT, nil, nil)){
		RWERROR((ERR_CHUNK, "STRUCT"));
		return nil;
	}
	platform = stream->readU32();
	assert(platform == PLATFORM_DC);
	
	uint32 header;
	stream->read8(&header, 4);

	Skin *skin = rwNewT(Skin, 1, MEMDUR_EVENT | ID_SKIN);
	*PLUGINOFFSET(Skin*, geometry, offset) = skin;

	skin->init(header, header, 0);
	
	skin->numWeights = 0;

	if(skin->numBones)
		stream->read32(skin->inverseMatrices, skin->numBones*64);

	// is this required?
	skin->numWeights = 4;
	for(int32 i = 0; i < skin->numUsedBones; i++)
		skin->usedBones[i] = i;

	return stream;
}

Stream*
writeNativeSkin(Stream *stream, int32 len, void *object, int32 offset)
{
	writeChunkHeader(stream, ID_STRUCT, len-12);
	stream->writeU32(PLATFORM_DC);
	Skin *skin = *PLUGINOFFSET(Skin*, object, offset);

	stream->write8(&skin->numBones, 4);

	stream->write32(skin->inverseMatrices, skin->numBones*64);
	return stream;
}

int32 getSizeNativeSkin(void *object, int32 offset) {
	Skin *skin = *PLUGINOFFSET(Skin*, object, offset);
	if(skin == nil)
		return -1;
	int32 size = 12 + 4 + 4 + skin->numBones*64;
	return size;
}

void
defaultMatFXRenderCB(ObjPipeline *pipe, Atomic *atomic)
{
	//TODO: Implement skinning?
	defaultRenderCB(pipe, atomic);
}

ObjPipeline*
makeMatFXPipeline(void)
{
	ObjPipeline *pipe = rw::ObjPipeline::create();
    pipe->init(PLATFORM_DC);
	pipe->impl.instance = defaultInstance;
	pipe->impl.uninstance = defaultUninstance;
	pipe->impl.render = defaultMatFXRenderCB;
	pipe->pluginID = ID_MATFX;
	pipe->pluginData = 1;
	return pipe;
}

static void*
matfxOpen(void *o, int32, int32)
{
	matFXGlobals.pipelines[PLATFORM_DC] = makeMatFXPipeline();

	return o;
}

static void*
matfxClose(void *o, int32, int32)
{
	((ObjPipeline*)matFXGlobals.pipelines[PLATFORM_DC])->destroy();
	matFXGlobals.pipelines[PLATFORM_DC] = nil;

	return o;
}

void
initMatFX(void)
{
	Driver::registerPlugin(PLATFORM_DC, 0, ID_MATFX,
	                       matfxOpen, matfxClose);
}

}
}

#endif
