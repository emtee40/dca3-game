#pragma once

#include "Quaternion.h"

#ifdef MoveMemory
#undef MoveMemory	// windows shit
#endif

// TODO: put them somewhere else?
static int16 checked_f2i16(float f) {
	assert(f >= -32768 && f <= 32767);
	return f;
}

static uint16 checked_f2u16(float f) {
	assert(f >= 0 && f <= 65535);
	return f;
}

#define KF_MINDELTA (1/256.f)

struct KeyFrame {
	int16 rot[4];		// 4096
	uint16 dltTime;	// 256

	CQuaternion rotation_() {
		return { rot[0] * (1/4096.f), rot[1] * (1/4096.f), rot[2] * (1/4096.f), rot[3] * (1/4096.f) };
	}

	void rotation_(const CQuaternion& q) {
		rot[0] = checked_f2i16(q.x * 4096.0f);
		rot[1] = checked_f2i16(q.y * 4096.0f);
		rot[2] = checked_f2i16(q.z * 4096.0f);
		rot[3] = checked_f2i16(q.w * 4096.0f);
	}

	float deltaTime_() {
		return dltTime * (1/256.0f);
	}

	void deltaTime_(float t) {
		dltTime = checked_f2u16(t * 256); // always round down
	}
};

struct KeyFrameTransUncompressed : KeyFrame {
	// Some animations use bigger range, eg during the intro
	CVector trans;
	CVector translation_() {
		return trans;
	}

	void translation_(const CVector &v) {
		trans = v;
	}
};

struct KeyFrameTransCompressed : KeyFrame {
	int16 trans[3];		// 128

	CVector translation_() {
		return { trans[0] * (1/128.f), trans[1] * (1/128.f), trans[2] * (1/128.f)};
	}

	void translation_(const CVector &v) {
		trans[0] = checked_f2i16(v.x * 128.f);
		trans[1] = checked_f2i16(v.y * 128.f);
		trans[2] = checked_f2i16(v.z * 128.f);
	}
};


// The sequence of key frames of one animated node
class CAnimBlendSequence
{
public:
	enum {
		KF_ROT = 1,
		KF_TRANS = 2,
		KF_COMPRESSED = 4, // only applicable for KF_TRANS
	};
	int32 type;
	char name[24];
	int32 numFrames;
#ifdef PED_SKIN
	int16 boneTag;
#endif
	void *keyFrames;

	CAnimBlendSequence(void);
	virtual ~CAnimBlendSequence(void);
	void SetName(char *name);
	void SetNumFrames(int numFrames, bool translation, bool compress);
	void RemoveQuaternionFlips(void);
	

	void SetTranslation(int n, const CVector &v) {
		if (type & KF_COMPRESSED) {
			((KeyFrameTransCompressed*)keyFrames)[n].translation_(v);
		} else if (type & KF_TRANS) {
			((KeyFrameTransUncompressed*)keyFrames)[n].translation_(v);
		} else {
			assert(false && "SetTranslation called on sequence without translation");
		}
	}

	CVector GetTranslation(int n) {
		if (type & KF_COMPRESSED) {
			return ((KeyFrameTransCompressed*)keyFrames)[n].translation_();
		} else if (type & KF_TRANS) {
			return ((KeyFrameTransUncompressed*)keyFrames)[n].translation_();
		} else {
			assert(false && "GetTranslation called on sequence without translation");
		}
	}

	void SetRotation(int n, const CQuaternion &q) {
		if (type & KF_COMPRESSED) {
			((KeyFrameTransCompressed*)keyFrames)[n].rotation_(q);
		} else if (type & KF_TRANS) {
			((KeyFrameTransUncompressed*)keyFrames)[n].rotation_(q);
		} else {
			((KeyFrame*)keyFrames)[n].rotation_(q);
		}
	}

	CQuaternion GetRotation(int n) {
		if (type & KF_COMPRESSED) {
			return ((KeyFrameTransCompressed*)keyFrames)[n].rotation_();
		} else if (type & KF_TRANS) {
			return ((KeyFrameTransUncompressed*)keyFrames)[n].rotation_();
		} else {
			return ((KeyFrame*)keyFrames)[n].rotation_();
		}
	}

	void SetDeltaTime(int n, float t) {
		if (type & KF_COMPRESSED) {
			((KeyFrameTransCompressed*)keyFrames)[n].deltaTime_(t);
		} else if (type & KF_TRANS) {
			((KeyFrameTransUncompressed*)keyFrames)[n].deltaTime_(t);
		} else {
			((KeyFrame*)keyFrames)[n].deltaTime_(t);
		}
	}

	float GetDeltaTime(int n) {
		if (type & KF_COMPRESSED) {
			return ((KeyFrameTransCompressed*)keyFrames)[n].deltaTime_();
		} else if (type & KF_TRANS) {
			return ((KeyFrameTransUncompressed*)keyFrames)[n].deltaTime_();
		} else {
			return ((KeyFrame*)keyFrames)[n].deltaTime_();
		}
	}

	bool HasTranslation(void) { return !!(type & KF_TRANS); }
	bool MoveMemory(void);

#ifdef PED_SKIN
	void SetBoneTag(int tag) { boneTag = tag; }
#endif
};
#ifndef PED_SKIN
VALIDATE_SIZE(CAnimBlendSequence, 0x2C);
#endif
