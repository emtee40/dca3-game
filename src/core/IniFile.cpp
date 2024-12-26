#include "common.h"

#include "IniFile.h"

#include "CarCtrl.h"
#include "FileMgr.h"
#include "main.h"
#include "Population.h"

float CIniFile::PedNumberMultiplier = 0.6f;	// dreamcast default
float CIniFile::CarNumberMultiplier = 0.6f;	// dreamcast default

void CIniFile::LoadIniFile()
{
	CFileMgr::SetDir("");
	// gta3.ini is ignored for now
	#if 0
	int f = CFileMgr::OpenFile("gta3.ini", "r");
	if (f){
		CFileMgr::ReadLine(f, gString, 200);
		sscanf(gString, "%f", &PedNumberMultiplier);
		PedNumberMultiplier = Min(3.0f, Max(0.5f, PedNumberMultiplier));
		CFileMgr::ReadLine(f, gString, 200);
		sscanf(gString, "%f", &CarNumberMultiplier);
		CarNumberMultiplier = Min(3.0f, Max(0.5f, CarNumberMultiplier));
		CFileMgr::CloseFile(f);
	}
	#endif
	CPopulation::MaxNumberOfPedsInUse = DEFAULT_MAX_NUMBER_OF_PEDS * PedNumberMultiplier;
	CCarCtrl::MaxNumberOfCarsInUse = DEFAULT_MAX_NUMBER_OF_CARS * CarNumberMultiplier;
}