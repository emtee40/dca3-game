#include "common.h"

void
CVector::Normalise(void)
{
#ifdef DC_SH4_BROKEN
	// TODO: This needs to handle zero vectors here
	vec3f_normalize(x, y, z);
#else
	float sq = MagnitudeSqr();
	if (sq > 0.0f) {
		float invsqrt = RecipSqrt(sq);
		x *= invsqrt;
		y *= invsqrt;
		z *= invsqrt;
	} else
		x = 1.0f;
#endif
}

CVector 
CrossProduct(const CVector &v1, const CVector &v2)
{
	return CVector(v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x);
}

CVector
Multiply3x3(const CMatrix &mat, const CVector &vec)
{
#ifdef DC_SH4
    register float __x __asm__("fr12") = vec.x;
    register float __y __asm__("fr13") = vec.y;
    register float __z __asm__("fr14") = vec.z;
    register float __w __asm__("fr15") = 0.0f;
	
	mat_load(reinterpret_cast<matrix_t *>(const_cast<CMatrix *>(&mat)));

	asm volatile( "ftrv  xmtrx, fv12\n"
                : "=f" (__x), "=f" (__y), "=f" (__z), "=f" (__w)
                : "0" (__x), "1" (__y), "2" (__z), "3" (__w) );
    return { __x, __y, __z };
#else
	// TODO: VU0 code
	return CVector(mat.rx * vec.x + mat.fx * vec.y + mat.ux * vec.z,
	               mat.ry * vec.x + mat.fy * vec.y + mat.uy * vec.z,
	               mat.rz * vec.x + mat.fz * vec.y + mat.uz * vec.z);
#endif
}

CVector
Multiply3x3(const CVector &vec, const CMatrix &mat)
{
	return CVector(mat.rx * vec.x + mat.ry * vec.y + mat.rz * vec.z,
	               mat.fx * vec.x + mat.fy * vec.y + mat.fz * vec.z,
	               mat.ux * vec.x + mat.uy * vec.y + mat.uz * vec.z);
}

CVector
operator*(const CMatrix &mat, const CVector &vec)
{
#ifdef DC_SH4
	CVector out;
	mat_load(reinterpret_cast<matrix_t *>(const_cast<CMatrix *>(&mat)));
	mat_trans_single3_nodiv_nomod(vec.x, vec.y, vec.z, out.x, out.y, out.z);
	return out;
#else
	// TODO: VU0 code
	return CVector(mat.rx * vec.x + mat.fx * vec.y + mat.ux * vec.z + mat.px,
	               mat.ry * vec.x + mat.fy * vec.y + mat.uy * vec.z + mat.py,
	               mat.rz * vec.x + mat.fz * vec.y + mat.uz * vec.z + mat.pz);
#endif
}
