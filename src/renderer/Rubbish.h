#pragma once

class CVehicle;

enum {

#ifdef RW_DC // Frogbull (not Dirty) Hack that allow less Debris on screen for the Dreamcast Port
	NUM_RUBBISH_SHEETS = 16 // NUM_RUBBISH_SHEETS must be a multiple of 4 (4 or 8 is the minimum value I think, max value is 64)
#elif SQUEEZE_PERFORMANCE
	NUM_RUBBISH_SHEETS = 32
#else
	NUM_RUBBISH_SHEETS = 64 // NB: not all values are allowed, check the code
#endif

};

class COneSheet
{
public:
	CVector m_basePos;
	CVector m_animatedPos;
	float m_targetZ;
	int8 m_state;
	int8 m_animationType;
	uint32 m_moveStart;
	uint32 m_moveDuration;
	float m_animHeight;
	float m_xDist;
	float m_yDist;
	float m_angle;
	bool m_isVisible;
	bool m_targetIsVisible;
	COneSheet *m_next;
	COneSheet *m_prev;

	void AddToList(COneSheet *list);
	void RemoveFromList(void);
};

class CRubbish
{
	static bool bRubbishInvisible;
	static int RubbishVisibility;
	static COneSheet aSheets[NUM_RUBBISH_SHEETS];
	static COneSheet StartEmptyList;
	static COneSheet EndEmptyList;
	static COneSheet StartStaticsList;
	static COneSheet EndStaticsList;
	static COneSheet StartMoversList;
	static COneSheet EndMoversList;
public:
	static void Render(void);
	static void StirUp(CVehicle *veh);	// CAutomobile on PS2
	static void Update(void);
	static void SetVisibility(bool visible);
	static void Init(void);
	static void Shutdown(void);
};
