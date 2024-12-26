#define WITHWINDOWS
#include "common.h"
#include "crossplatform.h"

#include "FileMgr.h"
#include "Font.h"
#ifdef MORE_LANGUAGES
#include "Game.h"
#endif
#include "GenericGameStorage.h"
#include "Messages.h"
#include "PCSave.h"
#include "Text.h"

#include "minilzo.h"
#include "main.h"

#include "../vmu/vmu.h"

const char* _psGetUserFilesFolder();

C_PcSave PcSaveHelper;

void
C_PcSave::SetSaveDirectory(const char *path)
{
	#if defined(RW_DC)
	sprintf(DefaultPCSaveFileName, "%s/%s", path, "GTA3SF");
	#else
	sprintf(DefaultPCSaveFileName, "%s\\%s", path, "GTA3sf");
	#endif
}

bool
C_PcSave::DeleteSlot(int32 slot)
{
#ifdef FIX_BUGS
	char FileName[MAX_PATH];
#else
	char FileName[200];
#endif

	PcSaveHelper.nErrorCode = SAVESTATUS_SUCCESSFUL;
	sprintf(FileName, "%s%i%s", slot==7?"GTA3SF":DefaultPCSaveFileName, slot + 1, slot==7?".b": "");
	DeleteFile(FileName);
	SlotSaveDate[slot][0] = '\0';
	return true;
}

bool
C_PcSave::SaveSlot(int32 slot)
{
	RAIIVmuBeep(VMU_DEFAULT_PATH, 1.0f);
	MakeValidSaveName(slot);
	PcSaveHelper.nErrorCode = SAVESTATUS_SUCCESSFUL;
	_psGetUserFilesFolder();
	int file = CFileMgr::OpenFile(ValidSaveName, "wb");
	if (file != 0) {
#ifdef MISSION_REPLAY
		if (!IsQuickSave)
#endif
			DoGameSpecificStuffBeforeSave();
		if (GenericSave(file)) {
			if (!!CFileMgr::CloseFile(file))
				nErrorCode = SAVESTATUS_ERR_SAVE_CLOSE;
			return true;
		}

		return false;
	}
	PcSaveHelper.nErrorCode = SAVESTATUS_ERR_SAVE_CREATE;
	return false;
}

uint32_t C_PcSave::PcClassLoadRoutine(int32 file, uint8 *data) {
	uint32 size;
	CFileMgr::Read(file, (char*)&size, sizeof(size));
	

	assert(data == work_buff);

	if (!(size & 0x80000000)) {
		assert(align4bytes(size) == size);
		CFileMgr::Read(file, (char*)data, align4bytes(size));
		if (CFileMgr::GetErrorReadWrite(file)) {
			return 0;
		}
		return size;
	} else {
		size &= ~0x80000000;
		uint8* compressed = (uint8*)malloc(size);
		CFileMgr::Read(file, (const char*)compressed, size);
		if (CFileMgr::GetErrorReadWrite(file)) {
			free(compressed);
			return 0;
		}

		lzo_uint decompressed_size = 0;
		auto crv = lzo1x_decompress(compressed, size, data, &decompressed_size, NULL);
		free(compressed);
		if (crv != LZO_E_OK) {
			return 0;
		}

		if (align4bytes(decompressed_size) != decompressed_size) {
			return 0;
		}

		return decompressed_size;
	}
}
bool
C_PcSave::PcClassSaveRoutine(int32 file, uint8 *data, uint32 size)
{
	void* wrkmem = malloc(LZO1X_1_MEM_COMPRESS);
	uint8* compressed = (uint8*)malloc(size*2);
	lzo_uint compressed_size;
	int crv = lzo1x_1_compress(data, size, compressed, &compressed_size, wrkmem);
	free(wrkmem);

	if (crv == LZO_E_OK) {
		uint32_t compressed_size32 = compressed_size | 0x80000000;
		CFileMgr::Write(file, (const char*)&compressed_size32, sizeof(compressed_size32));
		if (CFileMgr::GetErrorReadWrite(file)) {
			free(compressed);
			nErrorCode = SAVESTATUS_ERR_SAVE_WRITE;
			strncpy(SaveFileNameJustSaved, ValidSaveName, sizeof(ValidSaveName) - 1);
			return false;
		}

		CFileMgr::Write(file, (const char*)compressed, compressed_size);
		free(compressed);
	} else if (crv == LZO_E_NOT_COMPRESSIBLE) {
		free(compressed);
		uint32_t compressed_size32 = size;
		CFileMgr::Write(file, (const char*)&compressed_size32, sizeof(compressed_size32));
		if (CFileMgr::GetErrorReadWrite(file)) {
			nErrorCode = SAVESTATUS_ERR_SAVE_WRITE;
			strncpy(SaveFileNameJustSaved, ValidSaveName, sizeof(ValidSaveName) - 1);
			return false;
		}
		CFileMgr::Write(file, (const char*)data, align4bytes(size));
	} else {
		free(compressed);
		return false;
	}

	CheckSum += (uint8) size;
	CheckSum += (uint8) (size >> 8);
	CheckSum += (uint8) (size >> 16);
	CheckSum += (uint8) (size >> 24);
	for (int i = 0; i < align4bytes(size); i++) {
		CheckSum += *data++;
	}
	if (CFileMgr::GetErrorReadWrite(file)) {
		nErrorCode = SAVESTATUS_ERR_SAVE_WRITE;
		strncpy(SaveFileNameJustSaved, ValidSaveName, sizeof(ValidSaveName) - 1);
		return false;
	}

	return true;
}

void
C_PcSave::PopulateSlotInfo()
{
	RAIIVmuBeep(VMU_DEFAULT_PATH, 1.0f);
	for (int i = 0; i < SLOT_COUNT; i++) {
		Slots[i + 1] = SLOT_EMPTY;
		SlotFileName[i][0] = '\0';
		SlotSaveDate[i][0] = '\0';
	}
	for (int i = 0; i < SLOT_COUNT; i++) {
#ifdef FIX_BUGS
		char savename[MAX_PATH];
#else
		char savename[52];
#endif
		struct header_t {
			wchar FileName[24];
			SYSTEMTIME SaveDateTime;
		} header;
		sprintf(savename, "%s%i%s", i==7?"GTA3SF":DefaultPCSaveFileName, i + 1, i==7?".b": "");
		int file = CFileMgr::OpenFile(savename, "rb");
		printf("file: %s: %d\n", savename, file);
		if (file != 0) {
			if (C_PcSave::PcClassLoadRoutine(file, (uint8*)work_buff)) {
				header = *(header_t*)work_buff;
				Slots[i + 1] = SLOT_OK;
				memcpy(SlotFileName[i], &header.FileName, sizeof(header.FileName));
				SlotFileName[i][24] = '\0';
			}
			CFileMgr::CloseFile(file);
		}
		if (Slots[i + 1] == SLOT_OK) {
			if (CheckDataNotCorrupt(i, savename)) {
#ifdef FIX_INCOMPATIBLE_SAVES
				if (!FixSave(i, GetSaveType(savename))) {
					CMessages::InsertNumberInString(TheText.Get("FEC_SLC"), i + 1, -1, -1, -1, -1, -1, SlotFileName[i]);
					Slots[i + 1] = SLOT_CORRUPTED;
					continue;
				}
#endif
				SYSTEMTIME st;
				memcpy(&st, &header.SaveDateTime, sizeof(SYSTEMTIME));
				const char *month;
				switch (st.wMonth)
				{
				case 1: month = "JAN"; break;
				case 2: month = "FEB"; break;
				case 3: month = "MAR"; break;
				case 4: month = "APR"; break;
				case 5: month = "MAY"; break;
				case 6: month = "JUN"; break;
				case 7: month = "JUL"; break;
				case 8: month = "AUG"; break;
				case 9: month = "SEP"; break;
				case 10: month = "OCT"; break;
				case 11: month = "NOV"; break;
				case 12: month = "DEC"; break;
				default: assert(0);
				}
				char date[70];
#ifdef MORE_LANGUAGES
				if (CGame::japaneseGame)
					sprintf(date, "%02d %02d %04d %02d:%02d:%02d", st.wDay, st.wMonth, st.wYear, st.wHour, st.wMinute, st.wSecond);
				else
#endif // MORE_LANGUAGES
					sprintf(date, "%02d %s %04d %02d:%02d:%02d", st.wDay, UnicodeToAsciiForSaveLoad(TheText.Get(month)), st.wYear, st.wHour, st.wMinute, st.wSecond);
				AsciiToUnicode(date, SlotSaveDate[i]);

			} else {
				CMessages::InsertNumberInString(TheText.Get("FEC_SLC"), i + 1, -1, -1, -1, -1, -1, SlotFileName[i]);
				Slots[i + 1] = SLOT_CORRUPTED;
			}
		}
	}
}
