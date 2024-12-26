#include <dc/sound/sound.h>
#include <dc/sound/sfxmgr.h>
#include <dc/sound/stream.h>
#include <string>
#include <iostream>
#include <fstream>
#include <thread>
#include <mutex>

#include "common.h"
#include "crossplatform.h"

#if !defined(AUDIO_OAL) &&  !defined(AUDIO_MSS)
#define verbosef(...) // dbglog(DBG_CRITICAL, __VA_ARGS__)
#define debugf(...) // dbglog(DBG_CRITICAL, __VA_ARGS__)

#include "sampman.h"
#include "AudioManager.h"
#include "MusicManager.h"
#include "Frontend.h"
#include "Timer.h"

#include <dc/spu.h>
#include <dc/g2bus.h>
#include <dc/sound/aica_comm.h>

#include <map>
#include <vector>

#include "CdStream.h"

#define STREAM_STAGING_BUFFER_SIZE 16384
#define STREAM_STAGING_READ_SIZE_STEREO 16384
#define STREAM_STAGING_READ_SIZE_MONO (STREAM_STAGING_READ_SIZE_STEREO / 2)
#define STREAM_CHANNEL_BUFFER_SIZE (STREAM_STAGING_READ_SIZE_MONO * 2)	// lower and upper halves
#define STREAM_CHANNEL_SAMPLE_COUNT (STREAM_CHANNEL_BUFFER_SIZE * 2)		// 4 bit adpcm

#define SPU_RAM_UNCACHED_BASE_U8 ((uint8_t *)SPU_RAM_UNCACHED_BASE)
// ************************************************************************************************
// Begin AICA Driver stuff

#define AICA_MEM_CHANNELS   0x020000    /* 64 * 16*4 = 4K */

/* Quick access to the AICA channels */
#define AICA_CHANNEL(x)     (AICA_MEM_CHANNELS + (x) * sizeof(aica_channel_t))

int aica_play_chn(int chn, int size, uint32_t aica_buffer, int fmt, int vol, int pan, int loop, int freq) {
	// assert(size <= 65534);
	// We gotta fix this at some point
	if (size >= 65535) {
		debugf("aica_play_chn: size too large for %p, %d, truncating to 65534\n", (void*)aica_buffer, size);
		size = 65534;
	}

    AICA_CMDSTR_CHANNEL(tmp, cmd, chan);
    cmd->cmd = AICA_CMD_CHAN;
    cmd->timestamp = 0;
    cmd->size = AICA_CMDSTR_CHANNEL_SIZE;
    cmd->cmd_id = chn;
    chan->cmd = AICA_CH_CMD_START;
    chan->base = aica_buffer;
    chan->type = fmt;
    chan->length = size;
    chan->loop = loop;
    chan->loopstart = 0;
    chan->loopend = size;
    chan->freq = freq;
    chan->vol = vol;
    chan->pan = pan;
	snd_sh4_to_aica(tmp, cmd->size);
    return chn;
}

void aica_stop_chn(int chn) {
	AICA_CMDSTR_CHANNEL(tmp, cmd, chan);

    cmd->cmd = AICA_CMD_CHAN;
    cmd->timestamp = 0;
    cmd->size = AICA_CMDSTR_CHANNEL_SIZE;
    cmd->cmd_id = chn;
    chan->cmd = AICA_CH_CMD_STOP;
    snd_sh4_to_aica(tmp, cmd->size);
}

void aica_volpan_chn(int chn, int vol, int pan) {
	AICA_CMDSTR_CHANNEL(tmp, cmd, chan);

    cmd->cmd = AICA_CMD_CHAN;
    cmd->timestamp = 0;
    cmd->size = AICA_CMDSTR_CHANNEL_SIZE;
    cmd->cmd_id = chn;
    chan->cmd = AICA_CH_CMD_UPDATE | AICA_CH_UPDATE_SET_PAN | AICA_CH_UPDATE_SET_VOL;
    chan->vol = vol;
	chan->pan = pan;
    snd_sh4_to_aica(tmp, cmd->size);
}


void aica_snd_sfx_volume(int chn, int vol) {
    AICA_CMDSTR_CHANNEL(tmp, cmd, chan);

    cmd->cmd = AICA_CMD_CHAN;
    cmd->timestamp = 0;
    cmd->size = AICA_CMDSTR_CHANNEL_SIZE;
    cmd->cmd_id = chn;
    chan->cmd = AICA_CH_CMD_UPDATE | AICA_CH_UPDATE_SET_VOL;
    chan->vol = vol;
    snd_sh4_to_aica(tmp, cmd->size);
}

void aica_snd_sfx_pan(int chn, int pan) {
    AICA_CMDSTR_CHANNEL(tmp, cmd, chan);

    cmd->cmd = AICA_CMD_CHAN;
    cmd->timestamp = 0;
    cmd->size = AICA_CMDSTR_CHANNEL_SIZE;
    cmd->cmd_id = chn;
    chan->cmd = AICA_CH_CMD_UPDATE | AICA_CH_UPDATE_SET_PAN;
    chan->pan = pan;
    snd_sh4_to_aica(tmp, cmd->size);
}

void aica_snd_sfx_freq(int chn, int freq) {
    AICA_CMDSTR_CHANNEL(tmp, cmd, chan);

    cmd->cmd = AICA_CMD_CHAN;
    cmd->timestamp = 0;
    cmd->size = AICA_CMDSTR_CHANNEL_SIZE;
    cmd->cmd_id = chn;
    chan->cmd = AICA_CH_CMD_UPDATE | AICA_CH_UPDATE_SET_FREQ;
    chan->freq = freq;
    snd_sh4_to_aica(tmp, cmd->size);
}


void aica_snd_sfx_freq_vol(int chn, int freq, int vol) {
    AICA_CMDSTR_CHANNEL(tmp, cmd, chan);

    cmd->cmd = AICA_CMD_CHAN;
    cmd->timestamp = 0;
    cmd->size = AICA_CMDSTR_CHANNEL_SIZE;
    cmd->cmd_id = chn;
    chan->cmd = AICA_CH_CMD_UPDATE | AICA_CH_UPDATE_SET_FREQ | AICA_CH_UPDATE_SET_VOL;
    chan->freq = freq;
	chan->vol = vol;
    snd_sh4_to_aica(tmp, cmd->size);
}

// End of aica Driver stuff
// ************************************************************************************************

cSampleManager SampleManager;
bool8 _bSampmanInitialised = FALSE;
bool _dcAudioInitialized = false;

uint32 BankStartOffset[MAX_SFX_BANKS];
char SampleBankDescFilename[] = "sfx/sfx_all.dsc";
char SampleBankDataFilename[] = "sfx/sfx_all.raw";

struct sfx_bank {
	uintptr_t base;
	std::vector<uintptr_t> effects;
	std::vector<uintptr_t> effects_loop;
};

std::map<int, sfx_bank> sfx_banks;

int nPedSlotSfx[MAX_PEDSFX];
uintptr_t nPedSlotSfxAddr[MAX_PEDSFX];
uint8_t nCurrentPedSlot;

struct WavHeader {
    // RIFF Header
    char riff[4];        // RIFF Header Magic header
    uint32_t chunkSize;  // RIFF Chunk Size
    char wave[4];        // WAVE Header
    // "fmt" sub-chunk
    char fmt[4];         // FMT header
    uint32_t subchunk1Size; // Size of the fmt chunk
    uint16_t audioFormat;   // Audio format 1=PCM, other values indicate compression
    uint16_t numOfChan;     // Number of channels 1=Mono, 2=Stereo
    uint32_t samplesPerSec; // Sampling Frequency in Hz
    uint32_t bytesPerSec;   // bytes per second
    uint16_t blockAlign;    // 2=16-bit mono, 4=16-bit stereo
    uint16_t bitsPerSample; // Number of bits per sample
    // "data" sub-chunk
    char data[4];        // "data" string
    uint32_t dataSize;   // Size of the data section
};

static inline uint8_t linearlize_volume(uint8_t vol) {
	// uint8_t rv = powf(10.0f, (vol - MAX_VOLUME) / 42.0f) * 255;
	uint8_t rv = vol * 255 / MAX_VOLUME;
	// verbosef("linearlize_volume(%d) = %d\n", vol, rv);
	return rv;
}
cSampleManager::cSampleManager(void)
{
	;
}

cSampleManager::~cSampleManager(void)
{
	
}

bool8
cSampleManager::IsMP3RadioChannelAvailable(void)
{
	return FALSE;
}


void cSampleManager::ReleaseDigitalHandle(void)
{
}

void cSampleManager::ReacquireDigitalHandle(void)
{
}

struct alignas(32) stream_info {
	uint8_t buffer[STREAM_STAGING_BUFFER_SIZE + 128];
	std::mutex mtx;
	file_t fd;
	uint32_t aica_buffers[2]; // left, right
	int mapped_ch[2];	// left, right
	int rate;
	int total_samples;
	int played_samples;
	int file_offset;
	int vol;
	uint8_t nPan;
	uint8_t pan[2];

	bool stereo;
	bool playing;
	bool next_is_upper_half;
	bool first_refill;
	bool paused;
}; 

stream_info streams[MAX_STREAMS];


std::mutex channel_mtx;
static struct {
	uintptr_t ptr;
	uintptr_t ptr_loop; // or 0 if no loop
	int nSfx;
	int freq;
	float distMin;
	float distMax;
	float fX;
	float fY;
	float fZ;
	uint8_t attenuationVol;
	uint8_t emittingVol;
	uint8_t vol;
	uint8_t pan;
	char ch;
	char mapped_ch;
	bool loop;
	bool in_hnd_loop;
	int8_t nBank;
} channels[MAXCHANNELS+MAX2DCHANNELS];


// static void * fill_audio_cb (snd_stream_hnd_t hnd, int smp_req, int *smp_recv) {
// 	auto si = (stream_info*)snd_stream_get_userdata(hnd);

// 	assert(si->hnd == hnd);

// 	verbosef("fill_audio_cb: %p %d\n", si, smp_req);

// 	size_t read_bytes = 0;

// 	{
// 		std::lock_guard<std::mutex> lk(si->mtx);
// 		if (si->playing) {
// 			read_bytes = fs_read(si->fd, si->buffer, smp_req);
// 			if (read_bytes <= 0) {
// 				verbosef("fill_audio_cb: %p stream end\n", si);
// 				*smp_recv = 0;
// 				return NULL;
// 			}
// 		}
// 	}

// 	*smp_recv = read_bytes;

// 	if(si->seek_buf > 0) {
// 		size_t seek_bytes = si->seek_buf;
// 		si->seek_buf = 0;

// 		while(seek_bytes & 3) {
// 			++seek_bytes;
// 		}
// 		if(seek_bytes > 128) {
// 			seek_bytes = 128;
// 		}
// 		memset(si->buffer + sizeof(si->buffer) - seek_bytes, 0, seek_bytes);
// 		return si->buffer + seek_bytes;
// 	}

// 	return si->buffer;
// }

std::thread snd_thread;
bool8
cSampleManager::Initialise(void)
{
	auto init = snd_init();
	assert(init >= 0);
	// snd_stream_init_ex(2, STREAM_BUFFER_SIZE);

	for (int i = 0; i< MAX_STREAMS; i++) {
		streams[i].mapped_ch[0] = snd_sfx_chn_alloc();
		streams[i].mapped_ch[1] = snd_sfx_chn_alloc();
		streams[i].aica_buffers[0] = snd_mem_malloc(STREAM_CHANNEL_BUFFER_SIZE);
		streams[i].aica_buffers[1] = snd_mem_malloc(STREAM_CHANNEL_BUFFER_SIZE);
		debugf("Stream %d mapped to: %d, %d\n", i, streams[i].mapped_ch[0], streams[i].mapped_ch[1]);
		debugf("Stream %d buffers: %p, %p\n", i, (void*)streams[i].aica_buffers[0], (void*)streams[i].aica_buffers[1]);
		assert(streams[i].mapped_ch[0] != -1);
		assert(streams[i].mapped_ch[1] != -1);
		streams[i].fd = -1;

		streams[i].vol = 255;
		streams[i].nPan = 63;
		streams[i].pan[0] = 128;
		streams[i].pan[1] = 128;
	}

	for (int i = 0; i < (MAXCHANNELS+MAX2DCHANNELS); i++) {
		channels[i].mapped_ch = snd_sfx_chn_alloc();
		debugf("Channel %d mapped to %d\n", i, channels[i].mapped_ch);
		assert(channels[i].mapped_ch != -1);
	}

	if (!InitialiseSampleBanks())
		return FALSE;
	
	snd_thread = std::thread([]() {
		for(;;) {
			{
				std::lock_guard<std::mutex> lk(channel_mtx);
				for (int i = 0; i < MAXCHANNELS+MAX2DCHANNELS; i++) {
					if (channels[i].ch != -1) {
						assert(channels[i].nSfx != -1);

						uint16_t channel_pos = (g2_read_32(SPU_RAM_UNCACHED_BASE + AICA_CHANNEL(channels[i].ch) + offsetof(aica_channel_t, pos)) & 0xffff);
						// verbosef("Channel %d pos: %d\n", i, channel_pos);
						if (!channels[i].loop) {
							auto channel_looped = g2_read_32(SPU_RAM_UNCACHED_BASE + AICA_CHANNEL(channels[i].ch) + offsetof(aica_channel_t, looped));
							// the looped flag is set even for one shots and is a reliable way to know if the channel has finished playing
							if (channel_looped) {
								debugf("Auto stopping channel: %d -> %d\n", i, channels[i].ch);
								channels[i].ch = -1;
							}
						} else if (channels[i].ptr_loop && !channels[i].in_hnd_loop) {
							if (channel_pos >= SampleManager.m_aSamples[channels[i].nSfx].nLoopStartSample) {
								channels[i].in_hnd_loop = true;
								debugf("Starting loop section: for sfx_%d_loop.wav\n", channels[i].nSfx);
								snd_sfx_stop(channels[i].ch);
								channels[i].ch = aica_play_chn(channels[i].mapped_ch, SampleManager.m_aSamples[channels[i].nSfx].nLoopByteSize * 2, channels[i].ptr_loop, 2 /* ADPCM */, channels[i].vol, channels[i].pan, 1, channels[i].freq);
							}
						}
					}
				}
			}

			for (int i = 0; i< MAX_STREAMS; i++) {
				size_t do_read = 0;
				{
					std::lock_guard<std::mutex> lk(streams[i].mtx);
					if (streams[i].playing) {
						// get channel pos
						uint32_t channel_pos = g2_read_32(SPU_RAM_UNCACHED_BASE + AICA_CHANNEL(streams[i].mapped_ch[0]) + offsetof(aica_channel_t, pos)) & 0xffff;
						uint32_t logical_pos = channel_pos;
						if (logical_pos > STREAM_CHANNEL_SAMPLE_COUNT/2) {
							logical_pos -= STREAM_CHANNEL_SAMPLE_COUNT/2;
						}
						verbosef("Stream %d pos: %d, log: %d, rem: %d\n", i, channel_pos, logical_pos, streams[i].played_samples);
			
						bool can_refill = (streams[i].played_samples + STREAM_CHANNEL_SAMPLE_COUNT/2) < streams[i].total_samples;
						bool can_fetch = (streams[i].played_samples + STREAM_CHANNEL_SAMPLE_COUNT/2 + STREAM_CHANNEL_SAMPLE_COUNT/2 + STREAM_CHANNEL_SAMPLE_COUNT/2) < streams[i].total_samples;
						// copy over data if needed from staging
						if (channel_pos >= STREAM_CHANNEL_SAMPLE_COUNT/2 && !streams[i].next_is_upper_half) {
							streams[i].next_is_upper_half = true;
							if (can_refill) { // could we need a refill?
								verbosef("Filling channel %d with lower half\n", i);
								// fill lower half
								spu_memload(streams[i].aica_buffers[0], streams[i].buffer, STREAM_CHANNEL_BUFFER_SIZE/2);
								if (streams[i].stereo) {
									spu_memload(streams[i].aica_buffers[1], streams[i].buffer + STREAM_STAGING_READ_SIZE_MONO, STREAM_CHANNEL_BUFFER_SIZE/2);
								}
								// queue next read to staging if any
								if (can_fetch) {
									do_read = streams[i].stereo ? STREAM_STAGING_READ_SIZE_STEREO : STREAM_STAGING_READ_SIZE_MONO;
								}
							}
							assert(streams[i].first_refill == false);
							streams[i].played_samples += STREAM_CHANNEL_SAMPLE_COUNT/2;
						} else if (channel_pos < STREAM_CHANNEL_SAMPLE_COUNT/2 && streams[i].next_is_upper_half) {
							streams[i].next_is_upper_half = false;
							if (can_refill) { // could we need a refill?
								verbosef("Filling channel %d with upper half\n", i);
								// fill upper half
								spu_memload(streams[i].aica_buffers[0] + STREAM_CHANNEL_BUFFER_SIZE/2, streams[i].buffer, STREAM_CHANNEL_BUFFER_SIZE/2);
								if (streams[i].stereo) {
									spu_memload(streams[i].aica_buffers[1] + STREAM_CHANNEL_BUFFER_SIZE/2, streams[i].buffer + STREAM_STAGING_READ_SIZE_MONO, STREAM_CHANNEL_BUFFER_SIZE/2);
								}
								// queue next read to staging, if any
								if (can_fetch) {
									do_read = streams[i].stereo ? STREAM_STAGING_READ_SIZE_STEREO : STREAM_STAGING_READ_SIZE_MONO;
								}
							}
							if (streams[i].first_refill) {
								streams[i].first_refill = false;
							} else {
								streams[i].played_samples += STREAM_CHANNEL_SAMPLE_COUNT/2;
							}
						}
						// if end of file, stop
						if ((streams[i].played_samples + logical_pos) > streams[i].total_samples) {
							// stop channel
							debugf("Auto stopping stream: %d -> {%d, %d}, %d total\n", i, streams[i].mapped_ch[0], streams[i].mapped_ch[1], streams[i].total_samples);
							aica_stop_chn(streams[i].mapped_ch[0]);
							aica_stop_chn(streams[i].mapped_ch[1]);
							streams[i].playing = false;
						}
					}
					
					if (do_read) {
						debugf("Queueing stream read: %d, file: %d, buffer: %p, size: %d, tell: %d\n", i, streams[i].fd, streams[i].buffer, do_read, fs_tell(streams[i].fd));
						CdStreamQueueAudioRead(streams[i].fd, streams[i].buffer, do_read, streams[i].file_offset);
						streams[i].file_offset += do_read;
					}
				}
			}
			thd_sleep(50);
		}
	});
	
	for ( int32 i = 0; i < MAX_PEDSFX; i++ )
	{
		nPedSlotSfx[i]     = -1;
		nPedSlotSfxAddr[i] = snd_mem_malloc(PED_BLOCKSIZE_ADPCM);
		debugf("PedSlot %d buffer: %p\n", i, (void*)nPedSlotSfxAddr[i]);
	}
	
	nCurrentPedSlot = 0;

	_dcAudioInitialized = true;
	return TRUE;
}

void
cSampleManager::Terminate(void)
{

}

bool8 cSampleManager::CheckForAnAudioFileOnCD(void)
{
	return TRUE;
}

char cSampleManager::GetCDAudioDriveLetter(void)
{
	return '\0';
}

void
cSampleManager::UpdateEffectsVolume(void)
{
	// TODO
}


void
cSampleManager::SetEffectsMasterVolume(uint8 nVolume)
{
	m_nEffectsVolume = nVolume;
	UpdateEffectsVolume();
}

void
cSampleManager::SetMusicMasterVolume(uint8 nVolume)
{
	m_nMusicVolume = nVolume;
}

void
cSampleManager::SetEffectsFadeVolume(uint8 nVolume)
{
	m_nEffectsFadeVolume = nVolume;
	UpdateEffectsVolume();
}

void
cSampleManager::SetMusicFadeVolume(uint8 nVolume)
{
	m_nMusicFadeVolume = nVolume;
}

void
cSampleManager::SetMonoMode(uint8 nMode)
{
}

bool8
cSampleManager::LoadSampleBank(uint8 nBank)
{
	ASSERT( nBank < MAX_SFX_BANKS );
	ASSERT( nBank != SFX_BANK_PED_COMMENTS );
	verbosef("LoadSampleBank(%d)\n", nBank);

	auto it = sfx_banks.find(nBank);

	if (it == sfx_banks.end()) {
		debugf("Loading bank %d\n", nBank);
		sfx_bank bank;
		int firstSfx = BankStartOffset[nBank];
		int nextBankSfx = BankStartOffset[nBank+1];
		assert(firstSfx<nextBankSfx);
		size_t fileStart = m_aSamples[firstSfx].nFileOffset;
		size_t fileEnd = m_aSamples[nextBankSfx].nFileOffset;
		size_t fileSize = fileEnd - fileStart;

		debugf("File start: %d, end: %d\n", fileStart, fileEnd);
		debugf("Allocating %d bytes for bank %d\n", fileSize, nBank);
		while (!(bank.base = snd_mem_malloc(fileSize))) {
			UnloadUnusedSampleBank();
		}
		debugf("Allocated %d bytes at %p\n", fileSize, (void*)bank.base);

		// TODO: Split per-bank sfx file
		int fd = fs_open(SampleBankDataFilename, O_RDONLY);
		assert(fd >= 0);
		// this is very wasteful and temporary
		void* stagingBuffer = memalign(32, 32 * 2048);
		assert(stagingBuffer != 0);

		// Ideally, we'd suspend the CdStream thingy here or read via that instead
		uintptr_t loadOffset = bank.base;
		fs_seek(fd, fileStart, SEEK_SET);

		while (fileSize > 0) {
			size_t readSize = fileSize > 32 * 2048 ? 32 * 2048 : fileSize;
			int rs = fs_read(fd, stagingBuffer, readSize);
			debugf("Read %d bytes, expected %d\n", rs, readSize);
			assert(rs == readSize);
			spu_memload(loadOffset, stagingBuffer, readSize);
			loadOffset += readSize;
			fileSize -= readSize;
			debugf("Loaded %d bytes, %d remaining\n", readSize, fileSize);
		}
		fs_close(fd);
		free(stagingBuffer);
		

		for (int nSfx = BankStartOffset[nBank]; nSfx < BankStartOffset[nBank+1]; nSfx++) {
			bank.effects.push_back(m_aSamples[nSfx].nFileOffset - fileStart + bank.base);
			if (m_aSamples[nSfx].nLoopStartSample) {
				bank.effects_loop.push_back(m_aSamples[nSfx].nLoopFileOffset - fileStart + bank.base);
			} else {
				bank.effects_loop.push_back(0);
			}
			debugf("Loaded sfx %d at %p, loop at %p\n", nSfx, (void*)bank.effects.back(), (void*)bank.effects_loop.back());
		}
		
		debugf("Loaded bank %d\n", nBank);
		sfx_banks[nBank] = bank;
	}
	return TRUE;
}

void
cSampleManager::UnloadUnusedSampleBank()
{
	auto bank = (--sfx_banks.end())->first;
	assert(bank != SFX_BANK_0); // can't unload bank 0 for OOM
	UnloadSampleBank(bank);
}

void
cSampleManager::UnloadSampleBank(uint8 nBank)
{
	ASSERT( nBank < MAX_SFX_BANKS );

	verbosef("UnloadSampleBank(%d)\n", nBank);

	auto it = sfx_banks.find(nBank);
	if (it != sfx_banks.end()) {
		debugf("Unloading bank %d\n", nBank);
		{
			std::lock_guard<std::mutex> lk(channel_mtx);
			for (int i = 0; i < MAXCHANNELS+MAX2DCHANNELS; i++) {
				if (channels[i].nBank == nBank) {
					dbglog(DBG_CRITICAL, "Warning: Channel %d is in unloaded nBank %d\n", i, nBank);
					if (channels[i].ch != -1) {
						dbglog(DBG_CRITICAL, "Warning: Channel %d was activelly playing from bank %d\n", i, nBank);
						snd_sfx_stop(channels[i].ch);
					}
					channels[i].ch = -1;
					channels[i].nSfx = -1;
					channels[i].nBank = -1;
					channels[i].ptr = 0;
					channels[i].ptr_loop = 0;
				}
			}
		}
		snd_mem_free(it->second.base);
		sfx_banks.erase(it);
	}
}

int8
cSampleManager::IsSampleBankLoaded(uint8 nBank)
{
	ASSERT( nBank < MAX_SFX_BANKS );
	
	return LOADING_STATUS_LOADED;
}

uint8
cSampleManager::IsPedCommentLoaded(uint32 nComment)
{
	int8 slot;

	for ( int32 i = 0; i < _TODOCONST(3); i++ )
	{
		slot = nCurrentPedSlot - i - 1;
#ifdef FIX_BUGS
		if (slot < 0)
			slot += ARRAY_SIZE(nPedSlotSfx);
#endif
		if ( nComment == nPedSlotSfx[slot] )
			return LOADING_STATUS_LOADED;
	}
	
	return LOADING_STATUS_NOT_LOADED;
}


int32
cSampleManager::_GetPedCommentSlot(uint32 nComment)
{
	int8 slot;

	for ( int32 i = 0; i < _TODOCONST(3); i++ )
	{
		slot = nCurrentPedSlot - i - 1;
#ifdef FIX_BUGS
		if (slot < 0)
			slot += ARRAY_SIZE(nPedSlotSfx);
#endif
		if ( nComment == nPedSlotSfx[slot] )
			return slot;
	}
	
	return -1;
}

bool8
cSampleManager::LoadPedComment(uint32 nComment)
{
	verbosef("LoadPedComment %ld\n", nComment);

	ASSERT( nComment < TOTAL_AUDIO_SAMPLES );
	ASSERT (nComment >= SAMPLEBANK_PED_START && nComment < SAMPLEBANK_PED_MAX);

	if ( CTimer::GetIsCodePaused() )
		return FALSE;

	// no talking peds during cutsenes or the game end
	if ( MusicManager.IsInitialised() )
	{
		switch ( MusicManager.GetMusicMode() )
		{
			case MUSICMODE_CUTSCENE:
			{
				return FALSE;

				break;
			}
			
			case MUSICMODE_FRONTEND:
			{
				if ( MusicManager.GetNextTrack() == STREAMED_SOUND_GAME_COMPLETED )
					return FALSE;

				break;
			}
		}
	}

	assert(m_aSamples[nComment].nByteSize < PED_BLOCKSIZE_ADPCM);

	file_t fd = fs_open(SampleBankDataFilename, O_RDONLY);

	assert(fd >= 0);
	debugf("Loading ped comment %d, offset: %d, size: %d\n", nComment, m_aSamples[nComment].nFileOffset, m_aSamples[nComment].nByteSize);
	fs_seek(fd, m_aSamples[nComment].nFileOffset, SEEK_SET);


	// TODO: When we can dma directly to AICA, we can use this instead
	// fs_read(fd, SPU_BASE_U8 + nPedSlotSfxAddr[nCurrentPedSlot], sizeof(nPedSlotSfxAddr));

	void* stagingBuffer = memalign(32, m_aSamples[nComment].nByteSize);
	assert(stagingBuffer != 0);
	debugf("Allocated %d bytes at %p\n", m_aSamples[nComment].nByteSize, stagingBuffer);
	int rs = fs_read(fd, stagingBuffer, m_aSamples[nComment].nByteSize);
	debugf("Read %d bytes, expected %d\n", rs, m_aSamples[nComment].nByteSize);
	assert(rs == m_aSamples[nComment].nByteSize);

	fs_close(fd);

	spu_memload(nPedSlotSfxAddr[nCurrentPedSlot], stagingBuffer, m_aSamples[nComment].nByteSize);
	free(stagingBuffer);
	nPedSlotSfx[nCurrentPedSlot] = nComment;

	if ( ++nCurrentPedSlot >= MAX_PEDSFX )
		nCurrentPedSlot = 0;

	return TRUE;
}

int32
cSampleManager::GetBankContainingSound(uint32 nSfx)
{
	assert(nSfx < TOTAL_AUDIO_SAMPLES);
	
	for (int i = MAX_SFX_BANKS-1; i >= 0; i--)
	{
		if ( nSfx >= BankStartOffset[i] )
			return i;
	}
	
	return INVALID_SFX_BANK;
}

uint32
cSampleManager::GetSampleBaseFrequency(uint32 nSample)
{
	ASSERT( nSample < TOTAL_AUDIO_SAMPLES );
	return m_aSamples[nSample].nFrequency;
}

// now in samples
uint32
cSampleManager::GetSampleLoopStartOffset(uint32 nSample)
{
	ASSERT( nSample < TOTAL_AUDIO_SAMPLES );
	return m_aSamples[nSample].nLoopStartSample;
}

// always loop to end
int32
cSampleManager::GetSampleLoopEndOffset(uint32 nSample)
{
	ASSERT( nSample < TOTAL_AUDIO_SAMPLES );
	return -1;
}

uint32
cSampleManager::GetSampleLength(uint32 nSample)
{
	ASSERT( nSample < TOTAL_AUDIO_SAMPLES );
	return m_aSamples[nSample].nByteSize * 2; // adcpcm is 4 bit
}

bool8 cSampleManager::UpdateReverb(void)
{
	return FALSE;
}

void
cSampleManager::SetChannelReverbFlag(uint32 nChannel, bool8 nReverbFlag)
{
	ASSERT( nChannel < MAXCHANNELS+MAX2DCHANNELS );
}

bool8
cSampleManager::InitialiseChannel(uint32 nChannel, uint32 nSfx, uint8 nBank)
{
	ASSERT( nChannel < MAXCHANNELS+MAX2DCHANNELS );
	// printf("InitialiseChannel(nChannel: %d, nSfx: %d, nBank: %d)\n", nChannel, nSfx, nBank);
	StopChannel(nChannel);
	nBank = GetBankContainingSound(nSfx);
	if (nBank != SFX_BANK_PED_COMMENTS) {
		// Load the bank here, without holding channel_mtx as it may call UnloadSampleBank that locks it
		LoadSampleBank(nBank);
	}
	
	std::lock_guard<std::mutex> lk(channel_mtx);
	
	verbosef("InitialiseChannel %ld %ld %d\n", nChannel, nSfx, nBank);

	if (nBank == SFX_BANK_PED_COMMENTS) {
		int32 i;
		for ( i = 0; i < _TODOCONST(3); i++ )
		{
			int32 slot = nCurrentPedSlot - i - 1;
#ifdef FIX_BUGS
			if (slot < 0)
				slot += ARRAY_SIZE(nPedSlotSfx);
#endif
			if ( nSfx == nPedSlotSfx[slot] )
			{
				channels[nChannel].ptr = nPedSlotSfxAddr[slot];
				break;
			}
		}

		if (i == _TODOCONST(3))
			return FALSE;
		debugf("Channel %d is using ped comment %d, buffer %p, samples: %d\n", nChannel, nSfx, channels[nChannel].ptr, m_aSamples[nSfx].nByteSize*2);
	} else {
		channels[nChannel].ptr = sfx_banks[nBank].effects[nSfx - BankStartOffset[nBank]];
	}

	channels[nChannel].nSfx = nSfx;
	channels[nChannel].nBank = nBank;

	if (m_aSamples[nSfx].nLoopStartSample != 0) {
		channels[nChannel].ptr_loop = sfx_banks[nBank].effects_loop[nSfx - BankStartOffset[nBank]];
	} else {
		channels[nChannel].ptr_loop = 0;
	}

	channels[nChannel].freq = m_aSamples[nSfx].nFrequency;
	channels[nChannel].attenuationVol = 255;
	// channels[nChannel].vol = 255;
	// channels[nChannel].pan = 128;
	channels[nChannel].loop = 0;
	
	return TRUE;
}

void updateVol(uint32 nChannel) {

	auto newVol = channels[nChannel].emittingVol * channels[nChannel].attenuationVol / 255;
	// newVol = 255;
	// printf("updateVol(nChannel: %d) vol: %d, newVol: %d\n", nChannel, channels[nChannel].vol, newVol);
	if (channels[nChannel].vol != newVol) {
		channels[nChannel].vol = newVol;
		// printf("updateVol(nChannel: %d) vol: %d\n", nChannel, channels[nChannel].vol);
		if (channels[nChannel].ch != -1) {
			aica_snd_sfx_volume(channels[nChannel].ch, channels[nChannel].vol);
		}
	}
}

void
cSampleManager::SetChannelVolume(uint32 nChannel, uint32 nVolume)
{
	// ASSERT( nChannel >= MAXCHANNELS );
	ASSERT( nChannel < MAXCHANNELS+MAX2DCHANNELS );
	if (nVolume > MAX_VOLUME)
		nVolume = MAX_VOLUME;
	std::lock_guard<std::mutex> lk(channel_mtx);
	channels[nChannel].emittingVol = linearlize_volume(nVolume);// nVolume * 255 / MAX_VOLUME;
	channels[nChannel].attenuationVol = 255;

	updateVol(nChannel);
	verbosef("SetChannelVolume(nChannel: %d) vol: %d\n", nChannel, nVolume);
}

void
cSampleManager::SetChannelPan(uint32 nChannel, uint32 gta_pan)
{
	// ASSERT( nChannel >= MAXCHANNELS );
	ASSERT( nChannel < MAXCHANNELS+MAX2DCHANNELS );
	std::lock_guard<std::mutex> lk(channel_mtx);
	// DC uses logarithmic pan
	auto linear_gain = 1-abs(gta_pan-64.0f)/64.0f;

	auto db = 20.0f * std::log10(linear_gain);
	if (db < -45) {
		db = -45;
	}

	db = fabs(db);
	auto aica_paned_att = db / 0.355;

	int aica_pan;

	if (gta_pan < 64) {
		aica_pan = 128 - aica_paned_att;
	} else {
		aica_pan = 128 + aica_paned_att;
	}

	// nPan = nPan * 2; // 64 is center, for dc it is 128
	if (channels[nChannel].pan != aica_pan) {
		channels[nChannel].pan = aica_pan;
		if (channels[nChannel].ch != -1) {
			aica_snd_sfx_pan(channels[nChannel].ch, channels[nChannel].pan);
		}
	}
	// printf("SetChannelPan(nChannel: %d) pan: %d\n", nChannel, nPan);
}

void
cSampleManager::SetChannelFrequency(uint32 nChannel, uint32 nFreq)
{
	ASSERT( nChannel < MAXCHANNELS+MAX2DCHANNELS );
	// printf("SetChannelFrequency(nChannel: %d) freq: %d\n", nChannel, nFreq);
	std::lock_guard<std::mutex> lk(channel_mtx);
	if (channels[nChannel].freq != nFreq) {
		channels[nChannel].freq = nFreq;
		if (channels[nChannel].ch != -1) {
			aica_snd_sfx_freq(channels[nChannel].ch, channels[nChannel].freq);
		}
	}
}

void
cSampleManager::SetChannelLoopPoints(uint32 nChannel, uint32 nLoopStart, int32 nLoopEnd)
{
	ASSERT( nChannel < MAXCHANNELS+MAX2DCHANNELS );
	std::lock_guard<std::mutex> lk(channel_mtx);
	assert(channels[nChannel].nSfx != -1);
	assert(m_aSamples[channels[nChannel].nSfx].nLoopStartSample == nLoopStart && -1 == nLoopEnd);
}

void
cSampleManager::SetChannelLoopCount(uint32 nChannel, uint32 nLoopCount)
{
	ASSERT( nChannel < MAXCHANNELS+MAX2DCHANNELS );
	std::lock_guard<std::mutex> lk(channel_mtx);
	assert(nLoopCount == 0 || nLoopCount == 1);
	channels[nChannel].loop = nLoopCount == 0;
	// printf("SetChannelLoopCount(nChannel: %d) loop: %d\n", nChannel, nLoopCount);
}

bool8
cSampleManager::GetChannelUsedFlag(uint32 nChannel)
{
	ASSERT( nChannel < MAXCHANNELS+MAX2DCHANNELS );
	std::lock_guard<std::mutex> lk(channel_mtx);
	return channels[nChannel].ch != -1;
}

void
cSampleManager::StartChannel(uint32 nChannel)
{
	StopChannel(nChannel);
	std::lock_guard<std::mutex> lk(channel_mtx);
	// printf("StartChannel(nChannel: %d) vol: %d, pan: %d, freq: %d, loop: %d, sfx: %d\n", nChannel, channels[nChannel].vol, channels[nChannel].pan, channels[nChannel].freq, channels[nChannel].loop, channels[nChannel].nSfx);
	channels[nChannel].in_hnd_loop = false;
	assert(channels[nChannel].nSfx != -1);
	channels[nChannel].ch = aica_play_chn(
		channels[nChannel].mapped_ch,
		m_aSamples[channels[nChannel].nSfx].nByteSize * 2,
		channels[nChannel].ptr,
		2 /* ADPCM */,
		channels[nChannel].vol,
		channels[nChannel].pan,
		channels[nChannel].loop,
		channels[nChannel].freq
	);
	ASSERT( nChannel < MAXCHANNELS+MAX2DCHANNELS );
}

void
cSampleManager::StopChannel(uint32 nChannel)
{
	ASSERT( nChannel < MAXCHANNELS+MAX2DCHANNELS );
	std::lock_guard<std::mutex> lk(channel_mtx);
	// printf("StopChannel(nChannel: %d)\n", nChannel);
	if (channels[nChannel].ch != -1) {
		snd_sfx_stop(channels[nChannel].ch);
		channels[nChannel].ch = -1;
	}
}

void
cSampleManager::PreloadStreamedFile(uint8 nFile, uint8 nStream, uint32_t seek_bytes_aligned)
{
	ASSERT( nStream < MAX_STREAMS );
	file_t f = fs_open(DCStreamedNameTable[nFile], O_RDONLY);
	debugf("PreloadStreamedFile(%p, %d, %d) is %s\n", f, nFile, nStream, DCStreamedNameTable[nFile]);
	assert(f >= 0 );
	WavHeader hdr;
	assert(fs_read(f, &hdr, sizeof(hdr)) == sizeof(hdr));

	{
		std::lock_guard<std::mutex> lk(streams[nStream].mtx);

		// Do we gotta stop it here?
		assert(!streams[nStream].playing);
		streams[nStream].rate = hdr.samplesPerSec;
		streams[nStream].stereo = hdr.numOfChan == 2;
		if (streams[nStream].fd >= 0) {
			CdStreamDiscardAudioRead(streams[nStream].fd);
			fs_close(streams[nStream].fd);
		}
		streams[nStream].fd = f;
		streams[nStream].playing = false;
		streams[nStream].total_samples = hdr.dataSize * 2 / hdr.numOfChan;
		streams[nStream].played_samples = 0;
		streams[nStream].next_is_upper_half = true;
		streams[nStream].first_refill = true;

		debugf("PreloadStreamedFile: %s: stream: %d, freq: %d, chans: %d, byte size: %d, played samples: %d\n", DCStreamedNameTable[nFile], nStream, hdr.samplesPerSec, hdr.numOfChan, hdr.dataSize, streams[nStream].played_samples);
	

		// How to avoid the lock?
		if (seek_bytes_aligned) {
			streams[nStream].played_samples = seek_bytes_aligned * (streams[nStream].stereo ? 1 : 2);
			debugf("Seeking aligned to: %d, played_samples: %d\n", seek_bytes_aligned, streams[nStream].played_samples);
			fs_seek(streams[nStream].fd, 2048 + seek_bytes_aligned, SEEK_SET);
		} else {
			fs_seek(f, 2048, SEEK_SET);
		}

		#if 0
		// Read directly in the future
		fs_read(f, SPU_RAM_UNCACHED_BASE_U8 + streams[nStream].aica_buffers[0], STREAM_STAGING_READ_SIZE_MONO);
		if (streams[nStream].stereo) {
			fs_read(f, SPU_RAM_UNCACHED_BASE_U8 + streams[nStream].aica_buffers[1], STREAM_STAGING_READ_SIZE_MONO);
		}
		#else
		// Stage to memory
		fs_read(f, streams[nStream].buffer, streams[nStream].stereo ? STREAM_STAGING_READ_SIZE_STEREO : STREAM_STAGING_READ_SIZE_MONO);
		spu_memload(streams[nStream].aica_buffers[0], streams[nStream].buffer, STREAM_CHANNEL_BUFFER_SIZE/2);
		if (streams[nStream].stereo) {
			spu_memload(streams[nStream].aica_buffers[1], streams[nStream].buffer + STREAM_STAGING_READ_SIZE_MONO, STREAM_CHANNEL_BUFFER_SIZE/2);
		}
		#endif

		if (streams[nStream].total_samples > STREAM_CHANNEL_SAMPLE_COUNT/2) {
			// If more than one buffer, prefetch the next one
			fs_read(f, streams[nStream].buffer, streams[nStream].stereo ? STREAM_STAGING_READ_SIZE_STEREO : STREAM_STAGING_READ_SIZE_MONO);
		}

		streams[nStream].file_offset = fs_tell(f);
	}

	verbosef("PreloadStreamedFile: %p %d - %s, %d, %d, \n", f, nFile, DCStreamedNameTable[nFile], streams[nStream].rate, streams[nStream].stereo);
}

// we can't really pause a stream, so we just make it go very slow with zero volume
void
cSampleManager::PauseStream(bool8 nPauseFlag, uint8 nStream)
{
	ASSERT( nStream < MAX_STREAMS );
	{
		std::lock_guard<std::mutex> lk(streams[nStream].mtx);
		
		if (nPauseFlag != streams[nStream].paused) {
			streams[nStream].paused = nPauseFlag;
			if(nPauseFlag) {
				// rate of 0 is 172 samples/second. not ideal but gets the job done.
				aica_snd_sfx_freq_vol(streams[nStream].mapped_ch[0], 0, 0);
				aica_snd_sfx_freq_vol(streams[nStream].mapped_ch[1], 0, 0);
			} else {
				aica_snd_sfx_freq_vol(streams[nStream].mapped_ch[0], streams[nStream].rate, streams[nStream].vol);
				aica_snd_sfx_freq_vol(streams[nStream].mapped_ch[1], streams[nStream].rate, streams[nStream].vol);
			}
		}
	}
}

void
cSampleManager::StartPreloadedStreamedFile(uint8 nStream)
{
	ASSERT( nStream < MAX_STREAMS );
	debugf("StartPreloadedStreamedFile(%d)\n", nStream);
	std::lock_guard<std::mutex> lk(streams[nStream].mtx);
	if (streams[nStream].playing) {
		return;
	}
	debugf("StartPreloadedStreamedFile(%d) - actually starting stream\n", nStream);
	// int aica_play_chn(int chn, int size, uint32_t aica_buffer, int fmt, int vol, int pan, int loop, int freq)
	aica_play_chn(
		streams[nStream].mapped_ch[0],
		STREAM_CHANNEL_SAMPLE_COUNT,
		streams[nStream].aica_buffers[0],
		3 /* adpcm long stream */,
		streams[nStream].vol,
		streams[nStream].pan[0],
		1,
		streams[nStream].rate
	);

	aica_play_chn(
		streams[nStream].mapped_ch[1],
		STREAM_CHANNEL_SAMPLE_COUNT,
		streams[nStream].aica_buffers[streams[nStream].stereo ? 1 : 0],
		3 /* adpcm long stream */,
		streams[nStream].vol,
		streams[nStream].pan[1],
		1,
		streams[nStream].rate
	);

	streams[nStream].playing = true;
	// printf("StartPreloadedStreamedFile(%d)\n", nStream);
	// {
	// 	std::lock_guard<std::mutex> lk(streams[nStream].mtx);
	// 	streams[nStream].playing = true;
	// 	if (streams[nStream].active) {
	// 		snd_stream_stop(streams[nStream].hnd);
	// 	}
	// 	streams[nStream].active = true;
	// }
	// snd_stream_start_adpcm(streams[nStream].hnd, streams[nStream].rate, streams[nStream].stereo);
	// snd_stream_volume(streams[nStream].hnd, streams[nStream].vol);
	verbosef("StartPreloadedStreamedFile(%d)\n", nStream);
}

bool8
cSampleManager::StartStreamedFile(uint8 nFile, uint32 nPos, uint8 nStream)
{	
	ASSERT( nStream < MAX_STREAMS );
	debugf("StartStreamedFile(%d, %d, %d)\n", nFile, nPos, nStream);
	uint32_t seek_aligned = 0;
	if (nPos) {
		uint32 seek_bytes = nPos * streams[nStream].rate / (streams[nStream].stereo ? 1000: 2000);
		seek_aligned = seek_bytes & ~(streams[nStream].stereo ? (STREAM_STAGING_READ_SIZE_STEREO-1) : (STREAM_STAGING_READ_SIZE_MONO-1));
	}
	PreloadStreamedFile(nFile, nStream, seek_aligned);
	StartPreloadedStreamedFile(nStream);
	return TRUE;
}

void
cSampleManager::StopStreamedFile(uint8 nStream)
{
	ASSERT( nStream < MAX_STREAMS );
	verbosef("StopStreamedFile(%d)\n", nStream);
	std::lock_guard<std::mutex> lk(streams[nStream].mtx);
	streams[nStream].playing = false;
	
	aica_stop_chn(streams[nStream].mapped_ch[0]);
	aica_stop_chn(streams[nStream].mapped_ch[1]);
	
	if (streams[nStream].fd >= 0) {
		CdStreamDiscardAudioRead(streams[nStream].fd);
		fs_close(streams[nStream].fd);
	}
	streams[nStream].fd = -1;
}

int32
cSampleManager::GetStreamedFilePosition(uint8 nStream)
{
	ASSERT( nStream < MAX_STREAMS );
	int32 rv = 0;

	return streams[nStream].played_samples * 1000 / streams[nStream].rate;
	// if(streams[nStream].fd >= 0) {
	// 	rv = fs_tell(streams[nStream].fd);
	// }
	debugf("GetStreamedFilePosition: %d %d\n", nStream, rv);
	return rv;
}

void
cSampleManager::SetStreamedVolumeAndPan(uint8 nVolume, uint8 nPan, uint8 nEffectFlag, uint8 nStream)
{
	ASSERT( nStream < MAX_STREAMS );
	if (nVolume > MAX_VOLUME)
		nVolume = MAX_VOLUME;
	nVolume = linearlize_volume(nVolume); //nVolume * 255 / MAX_VOLUME;
	if (streams[nStream].vol != nVolume || streams[nStream].nPan != nPan) {
		streams[nStream].vol = nVolume;
		streams[nStream].nPan = nPan;

		auto lpan = (int8_t)nPan - 63;
		lpan = lpan < 0 ? 0 : lpan > 63 ? 63 : lpan;
		
		auto rpan = (int8_t)nPan + 63;
		rpan = rpan < 64 ? 64 : rpan > 127 ? 127 : rpan;

		streams[nStream].pan[0] = lpan * 2;
		streams[nStream].pan[1] = rpan * 2;
		aica_volpan_chn(streams[nStream].mapped_ch[0], streams[nStream].vol,  streams[nStream].pan[0]);
		aica_volpan_chn(streams[nStream].mapped_ch[1], streams[nStream].vol,  streams[nStream].pan[1]);
	}
	// verbosef("SetStreamedVolumeAndPan: %d %d %d %d\n", nStream, nVolume, nPan, nEffectFlag);
}

int32
cSampleManager::GetStreamedFileLength(uint8 nFile)
{
	ASSERT( nFile < TOTAL_STREAMED_SOUNDS );
	int32 rv = 1; // Look in MusicManager.cpp:268
	file_t fd = fs_open(DCStreamedNameTable[nFile], O_RDONLY);
	assert(fd >= 0);
	WavHeader hdr;
	assert(fs_read(fd, &hdr, sizeof(hdr)) == sizeof(hdr));

	rv = hdr.dataSize * 2000 / hdr.numOfChan / hdr.samplesPerSec;

	fs_close(fd);

	debugf("GetStreamedFileLength: %d %d\n", nFile, rv);
	return rv <= 0 ? 1 : rv;
}

bool8
cSampleManager::IsStreamPlaying(uint8 nStream)
{
	ASSERT( nStream < MAX_STREAMS );
	std::lock_guard<std::mutex> lk(streams[nStream].mtx);
	return streams[nStream].playing && !streams[nStream].paused;
}

bool8
cSampleManager::InitialiseSampleBanks(void)
{
	file_t fd = fs_open(SampleBankDescFilename, O_RDONLY);
	if (fd < 0)
		return FALSE;

	size_t rs = fs_read(fd, m_aSamples, sizeof(tSample)*TOTAL_AUDIO_SAMPLES);

	if (rs != sizeof(tSample)*TOTAL_AUDIO_SAMPLES) {
		fs_close(fd);
		return FALSE;
	}

	fs_close(fd);

	for (int i = 0; i < (MAXCHANNELS+MAX2DCHANNELS); i++) {
		channels[i].ptr = 0;
		channels[i].ptr_loop = 0;
		channels[i].ch = -1;
		channels[i].nSfx = -1;
		channels[i].nBank = -1;
	}

	LoadSampleBank(SFX_BANK_0);
	
	return TRUE;
}

#if defined(EXTERNAL_3D_SOUND)
void cSampleManager::SetSpeakerConfig(int32 nConfig) {

}
uint32 cSampleManager::GetMaximumSupportedChannels(void) {
	return MAXCHANNELS;
}

uint32 cSampleManager::GetNum3DProvidersAvailable(void) {
	return 32;
}
void cSampleManager::SetNum3DProvidersAvailable(uint32 num) {
	
}

char *cSampleManager::Get3DProviderName(uint8 id) {
	return "dummy";
}
void cSampleManager::Set3DProviderName(uint8 id, char *name) {

}

int8 cSampleManager::GetCurrent3DProviderIndex(void) {
	return 0;
}
int8 cSampleManager::SetCurrent3DProvider(uint8 which) {
	return 0;
}

void  cSampleManager::SetChannelEmittingVolume(uint32 nChannel, uint32 nVolume) {
	ASSERT( nChannel < MAXCHANNELS+MAX2DCHANNELS );
	// printf("SetChannelEmittingVolume(nChannel: %d) vol: %d\n", nChannel, nVolume);
	if (nVolume > MAX_VOLUME)
		nVolume = MAX_VOLUME;

	std::lock_guard<std::mutex> lk(channel_mtx);
	channels[nChannel].emittingVol = linearlize_volume(nVolume); // nVolume * 255 / MAX_VOLUME;
	updateVol(nChannel);
}

float calculatePan(float x, float z) {
    // Azimuth angle (in radians), range from -PI to +PI
    float theta = Atan2(x, z);

    // Use the sine function for smooth panning
    float pan = Sin(theta);  // Use sine to map -PI to +PI to -1.0 to +1.0

    return pan;
}

float calculateAttenuation(float x, float y, float z, float mindist, float maxdist) {
    // Distance calculation (Euclidean distance)
    float distance = Sqrt(x * x + y * y + z * z);

    // If distance is less than or equal to mindist, return full volume (attenuation = 1)
    if (distance <= mindist) {
        return 1.0f;
    }

    // If distance is greater than or equal to maxdist, return no sound (attenuation = 0)
    if (distance >= maxdist) {
        return 0.0f;
    }

    // Calculate attenuation using inverse distance model and rolloff factor
    // Attenuation = (referenceDistance / distance) ^ rolloffFactor
    float attenuation = (mindist / distance);	// rolloffFactor = 1.0f

    return attenuation;
}



void  cSampleManager::SetChannel3DPosition    (uint32 nChannel, float fX, float fY, float fZ) {
	ASSERT( nChannel < MAXCHANNELS+MAX2DCHANNELS );
	// printf("SetChannel3DPosition(nChannel: %d) x: %f, y: %f, z: %f\n", nChannel, fX, fY, fZ);
	{
		std::lock_guard<std::mutex> lk(channel_mtx);
		channels[nChannel].fX = fX;
		channels[nChannel].fY = fY;
		channels[nChannel].fZ = fZ;
		channels[nChannel].attenuationVol = calculateAttenuation(channels[nChannel].fX, channels[nChannel].fY, channels[nChannel].fZ, channels[nChannel].distMin, channels[nChannel].distMax) * 255;
		updateVol(nChannel);
	}

	SetChannelPan(nChannel, calculatePan(-fX, fZ) * 63 + 64);
	
}
void  cSampleManager::SetChannel3DDistances   (uint32 nChannel, float fMax, float fMin) {
	ASSERT( nChannel < MAXCHANNELS+MAX2DCHANNELS );
	std::lock_guard<std::mutex> lk(channel_mtx);
	// printf("SetChannel3DDistances(nChannel: %d) min: %f, max: %f\n", nChannel, fMin, fMax);
	channels[nChannel].distMin = fMin;
	channels[nChannel].distMax = fMax;
	channels[nChannel].attenuationVol = calculateAttenuation(channels[nChannel].fX, channels[nChannel].fY, channels[nChannel].fZ, channels[nChannel].distMin, channels[nChannel].distMax) * 255;
	updateVol(nChannel);
}
#endif

#endif
