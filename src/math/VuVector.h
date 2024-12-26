#pragma once

#include "maths.h"

class TYPEALIGN(8) CVuVector : public CVector
{
public:
	float w;
	CVuVector(void) {}
	CVuVector(float x, float y, float z) : CVector(x, y, z) {}
	CVuVector(float x, float y, float z, float w) : CVector(x, y, z), w(w) {}
	CVuVector(const CVector &v) : CVector(v.x, v.y, v.z) {}
	CVuVector(const RwV3d &v) : CVector(v) {}
/*
	void Normalise(void) {
		float sq = MagnitudeSqr();
		// TODO: VU0 code
		if(sq > 0.0f){
			float invsqrt = RecipSqrt(sq);
			x *= invsqrt;
			y *= invsqrt;
			z *= invsqrt;
		}else
			x = 1.0f;
	}
*/

	// TODO: operator-
};

__always_inline void TransformPoint(CVuVector &out, const CMatrix &mat, const CVuVector &in)
{
#ifdef GTA_PS2
	__asm__ __volatile__("\n\
		lqc2    vf01,0x0(%2)\n\
		lqc2    vf02,0x0(%1)\n\
		lqc2    vf03,0x10(%1)\n\
		lqc2    vf04,0x20(%1)\n\
		lqc2    vf05,0x30(%1)\n\
		vmulax.xyz	ACC,   vf02,vf01\n\
		vmadday.xyz	ACC,   vf03,vf01\n\
		vmaddaz.xyz	ACC,   vf04,vf01\n\
		vmaddw.xyz	vf06,vf05,vf00\n\
		sqc2    vf06,0x0(%0)\n\
		": : "r" (&out) , "r" (&mat) ,"r" (&in): "memory");
#elif defined(DC_SH4)
	mat_load(reinterpret_cast<matrix_t *>(const_cast<CMatrix *>(&mat)));
	mat_trans_nodiv_nomod(in.x, in.y, in.z, out.x, out.y, out.z, out.y);
#else
	out = mat * in;
#endif
}

__always_inline void TransformPoint(CVuVector &out, const CMatrix &mat, const RwV3d &in)
{
#ifdef GTA_PS2
	__asm__ __volatile__("\n\
		ldr	$8,0x0(%2)\n\
		ldl	$8,0x7(%2)\n\
		lw	$9,0x8(%2)\n\
		pcpyld	$10,$9,$8\n\
		qmtc2	$10,vf01\n\
		lqc2    vf02,0x0(%1)\n\
		lqc2    vf03,0x10(%1)\n\
		lqc2    vf04,0x20(%1)\n\
		lqc2    vf05,0x30(%1)\n\
		vmulax.xyz	ACC,   vf02,vf01\n\
		vmadday.xyz	ACC,   vf03,vf01\n\
		vmaddaz.xyz	ACC,   vf04,vf01\n\
		vmaddw.xyz	vf06,vf05,vf00\n\
		sqc2    vf06,0x0(%0)\n\
		": : "r" (&out) , "r" (&mat) ,"r" (&in): "memory");
#elif defined(DC_SH4)
	mat_load(reinterpret_cast<matrix_t *>(const_cast<CMatrix *>(&mat)));
	mat_trans_nodiv_nomod(in.x, in.y, in.z, out.x, out.y, out.z, out.y);
#else
	out = mat * in;
#endif
}

__always_inline void TransformPoints(CVuVector *out, int n, const CMatrix &mat, const RwV3d *in, int stride)
{
	assert(false);
#ifdef GTA_PS2
	__asm__ __volatile__("\n\
		paddub	$3,%4,$0\n\
		lqc2    vf02,0x0(%2)\n\
		lqc2    vf03,0x10(%2)\n\
		lqc2    vf04,0x20(%2)\n\
		lqc2    vf05,0x30(%2)\n\
		ldr	$8,0x0(%3)\n\
		ldl	$8,0x7(%3)\n\
		lw	$9,0x8(%3)\n\
		pcpyld	$10,$9,$8\n\
		qmtc2	$10,vf01\n\
	1:	vmulax.xyz	ACC,   vf02,vf01\n\
		vmadday.xyz	ACC,   vf03,vf01\n\
		vmaddaz.xyz	ACC,   vf04,vf01\n\
		vmaddw.xyz	vf06,vf05,vf00\n\
		add	%3,%3,$3\n\
		ldr	$8,0x0(%3)\n\
		ldl	$8,0x7(%3)\n\
		lw	$9,0x8(%3)\n\
		pcpyld	$10,$9,$8\n\
		qmtc2	$10,vf01\n\
		addi	%1,%1,-1\n\
		addiu	%0,%0,0x10\n\
		sqc2    vf06,-0x10(%0)\n\
		bnez	%1,1b\n\
		": : "r" (out) , "r" (n), "r" (&mat), "r" (in), "r" (stride): "memory");
#elif defined(DC_SH4)
    mat_load(reinterpret_cast<matrix_t *>(const_cast<CMatrix *>(&mat)));
	mat_transform(reinterpret_cast<vector_t *>(const_cast<RwV3d *>(in)),
	              reinterpret_cast<vector_t *>(out), 
				  n, stride - sizeof(vector_t));
#else
	while(n--){
		*out = mat * *in;
		in = (RwV3d*)((uint8*)in + stride);
		out++;
	}
#endif
}

__always_inline void TransformPoints(CVuVector *out, int n, const CMatrix &mat, const CVuVector *in)
{
#ifdef GTA_PS2
	__asm__ __volatile__("\n\
		lqc2    vf02,0x0(%2)\n\
		lqc2    vf03,0x10(%2)\n\
		lqc2    vf04,0x20(%2)\n\
		lqc2    vf05,0x30(%2)\n\
		lqc2    vf01,0x0(%3)\n\
		nop\n\
	1:	vmulax.xyz	ACC,   vf02,vf01\n\
		vmadday.xyz	ACC,   vf03,vf01\n\
		vmaddaz.xyz	ACC,   vf04,vf01\n\
		vmaddw.xyz	vf06,vf05,vf00\n\
		lqc2	vf01,0x10(%3)\n\
		addiu	%3,%3,0x10\n\
		addi	%1,%1,-1\n\
		addiu	%0,%0,0x10\n\
		sqc2    vf06,-0x10(%0)\n\
		bnez	%1,1b\n\
		": : "r" (out) , "r" (n), "r" (&mat) ,"r" (in): "memory");
#else
    TransformPoints(out, n, mat, in, sizeof(CVuVector));
#endif
}
