#pragma once

#include "templates.h"

#ifdef MoveMemory
#undef MoveMemory	// windows shit
#endif

class CAnimBlendSequence;

// A collection of sequences
class CAnimBlendHierarchy
{
public:
	char name[24];
	CAnimBlendSequence *sequences;
	int16 numSequences;
	float totalLength;

	CAnimBlendHierarchy(void);
	void Shutdown(void);
	void SetName(char *name);
	void CalcTotalTime(void);
	void RemoveQuaternionFlips(void);
	void RemoveAnimSequences(void);
	void MoveMemory(bool onlyone = false);
};

VALIDATE_SIZE(CAnimBlendHierarchy, 0x28);