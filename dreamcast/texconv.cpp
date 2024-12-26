#include <cstdio>
#include <cassert>
#include <cstring>

#include <set>

#include <iostream>
using namespace std;

void re3_assert(const char *expr, const char *filename, unsigned int lineno, const char *func)
{
	printf("\nRE3 ASSERT FAILED\n\tFile: %s\n\tLine: %d\n\tFunction: %s\n\tExpression: %s\n",filename,lineno,func,expr);
	fflush(stdout);
	assert(false);
}

#include "dc_hle_types.h"

#define debug(f, ...)

#include "rwcore.h"
#include "rwplcore.h"
#include "rpworld.h"
#include "skeleton.h"
#include "TxdStore.h"
#include "rpskin.h"
#include "rphanim.h"
#include "RpAnimBlend.h"
#include "rpmatfx.h"
#include "NodeName.h"
#include "rpanisot.h"
#include "common.h"
#include "VisibilityPlugins.h"

#include "src/dc/rwdc.h"
#include "src/dc/tex-util.h"

#include <dc/pvr.h>

#define debug(...)

// stubs
uint8_t emu_vram[PVR_RAM_SIZE];

void emu_init() {}
void emu_term() {}
void emu_pump_events() {}
void pvr_queue_interrupt(int interrupt) {}
uint32 pvrRegRead(uint32 A) {return 0;}
void pvrRegWrite(uint32 A, uint32 D) {}
uint32_t pvr_map32(uint32_t offset32) {return 0;}
void Hackpresent() { }
void re3RemoveLeastUsedModel() { assert(false); }
void RwTexDictionaryGtaStreamRead1(rw::Stream*){ assert(false); }
void RwTexDictionaryGtaStreamRead2(rw::Stream*, rw::TexDictionary*) { assert(false);  }
void pvr_ta_data(void* data, int size) {
    assert(false);
}

namespace rw::dc {
	extern int32 maxRasterWidth;
	extern int32 maxRasterHeight;
	extern int32 downsampleMode;
	extern int32 pvrEncoder;
}

RwTexDictionary *LoadTxd(RwStream *stream);
void StoreTxd(RwTexDictionary *texDict, RwStream *stream);



#ifdef LIBRW
#define READNATIVE(stream, tex, size) rwNativeTextureHackRead(stream, tex, size)
#else
#define READNATIVE(stream, tex, size) RWSRCGLOBAL(stdFunc[rwSTANDARDNATIVETEXTUREREAD](stream, tex, size))
#endif

void RwTextureGtaStreamWrite(RwStream *stream, RwTexture* texture)
{
	auto fheader = stream->tell();
	// size will be written later
	rw::writeChunkHeader(stream, rwID_TEXTURENATIVE, 0);
	auto fbegin = stream->tell();

	texture->streamWriteNative(stream);

	// rewrite header with correct size
	auto fend = stream->tell();
	stream->seek(fheader, 0);
	rw::writeChunkHeader(stream, rwID_TEXTURENATIVE, fend - fbegin);
	stream->seek(fend, 0);
}
RwTexture*
RwTextureGtaStreamRead(RwStream *stream)
{
	RwUInt32 size, version;
	RwTexture *tex;

	if(!RwStreamFindChunk(stream, rwID_TEXTURENATIVE, &size, &version))
		return nil;

	if(!READNATIVE(stream, &tex, size))
		return nil;

	return tex;
}

void
RwTexDictionaryGtaStreamWrite(RwStream *stream, RwTexDictionary *texDict) {
	rw::writeChunkHeader(stream, rwID_STRUCT, 4);
	stream->writeI32(texDict->count());
	
	RwTexDictionaryForAllTextures(texDict, [](RwTexture *texture, void* vstream) {
		RwTextureGtaStreamWrite((RwStream*)vstream, texture);
		return texture;
	}, stream);
}

RwTexDictionary*
RwTexDictionaryGtaStreamRead(RwStream *stream)
{
	RwUInt32 size, version;
	RwInt32 numTextures;
	RwTexDictionary *texDict;
	RwTexture *tex;

	if(!RwStreamFindChunk(stream, rwID_STRUCT, &size, &version))
		return nil;
	if(RwStreamRead(stream, &numTextures, size) != size)
		return nil;

	texDict = RwTexDictionaryCreate();
	if(texDict == nil)
		return nil;

	while(numTextures--){
		tex = RwTextureGtaStreamRead(stream);
		if(tex == nil){
			return nil;
		}
		RwTexDictionaryAddTexture(texDict, tex);
	}

	return texDict;
}

RpClump *LoadModelFile(const char *filename)
{
	RwStream *stream;
	RpClump *clump = nullptr;

	debug("Loading model file %s\n", filename);
	stream = RwStreamOpen(rwSTREAMFILENAME, rwSTREAMREAD, filename);
	if(RwStreamFindChunk(stream, rwID_CLUMP, nil, nil)){
		clump = RpClumpStreamRead(stream);
	}
	RwStreamClose(stream, nil);
	assert(clump);
	return clump;
}

void StoreModelFile(const char *filename, RpClump *clump)
{
	RwStream *stream;

	debug("Storing model file %s\n", filename);
	stream = RwStreamOpen(rwSTREAMFILENAME, rwSTREAMWRITE, filename);

	// on write it includes rwCLUMP
	auto ok = RpClumpStreamWrite(clump, stream);
	assert(ok);
		
	RwStreamClose(stream, nil);
	assert(clump);
}


static RwBool 
PluginAttach(void)
{
	if( !RpWorldPluginAttach() )
	{
		printf("Couldn't attach world plugin\n");
		
		return FALSE;
	}
	
	if( !RpSkinPluginAttach() )
	{
		printf("Couldn't attach RpSkin plugin\n");
		
		return FALSE;
	}
	
	if( !RpHAnimPluginAttach() )
	{
		printf("Couldn't attach RpHAnim plugin\n");
		
		return FALSE;
	}
	
	if( !NodeNamePluginAttach() )
	{
		printf("Couldn't attach node name plugin\n");
		
		return FALSE;
	}
	
	if( !CVisibilityPlugins::PluginAttach() )
	{
		printf("Couldn't attach visibility plugins\n");
		
		return FALSE;
	}
	
	if( !RpAnimBlendPluginAttach() )
	{
	 	printf("Couldn't attach RpAnimBlend plugin\n");
		
	 	return FALSE;
	}
	
	if( !RpMatFXPluginAttach() )
	{
		printf("Couldn't attach RpMatFX plugin\n");
		
		return FALSE;
	}
#ifdef ANISOTROPIC_FILTERING
	RpAnisotPluginAttach();
#endif
#ifdef EXTENDED_PIPELINES
	CustomPipes::CustomPipeRegister();
#endif

	return TRUE;
}

const char* currentFile;

int main(int argc, const char** argv) {
	if (argc >= 5) {
	    int width = atoi(argv[3]);
	    int height = atoi(argv[4]);
	    if(width >= 16 && width <= 1024)
		    rw::dc::maxRasterWidth = width;
        if(height >= 16 && height <= 1024)
		    rw::dc::maxRasterHeight = height;
	}
    for (int i = 0; i < argc; i++) {
        if (argv[i] != nullptr) {
            // Downsample Parameter
            if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "-D") == 0) {
                if (i + 1 < argc) {
                    const char* param = argv[i + 1];
                    if (strcmp(param, "half") == 0 || strcmp(param, "HALF") == 0 || strcmp(param, "h") == 0) {
                        rw::dc::downsampleMode = HALF;
                    } else if (strcmp(param, "quarter") == 0 || strcmp(param, "QUARTER") == 0 || strcmp(param, "q") == 0) {
                        rw::dc::downsampleMode = QUARTER;
                    }
                }
            }
            // PVR Encoder parameter
            if (strcmp(argv[i], "-e") == 0 || strcmp(argv[i], "-E") == 0) {
                if (i + 1 < argc) {
                    const char* param = argv[i + 1];
                    if (strcmp(param, "pvrtool") == 0 || strcmp(param, "PVRTOOL") == 0) {
                        rw::dc::pvrEncoder = PVRTOOL;
                    } else if (strcmp(param, "pvrtex") == 0 || strcmp(param, "PVRTEX") == 0) {
                        rw::dc::pvrEncoder = PVRTEX;
                    }
                }
            }
        }
    }

    printf("REPACK TXD: %s, Width: %i, Height: %i, Downsample: %s, Encoder: %s\n",
        argv[1],
        rw::dc::maxRasterWidth,
        rw::dc::maxRasterHeight,
        rw::dc::downsampleMode == NONE ? "NONE" : rw::dc::downsampleMode == HALF ? "HALF" : "QUARTER",
        rw::dc::pvrEncoder == PVRTOOL ? "PVRTOOL" : "PVRTEX");

	RwEngineInit(nullptr, 0, rsRESOURCESDEFAULTARENASIZE);
	RwEngineOpenParams openParams = { 0 };

	assert(RwEngineOpen(&openParams));
	assert(PluginAttach() == TRUE);
	assert(RwEngineStart());
	currentFile = argv[1];

	if (strstr(argv[1], ".txd") || strstr(argv[1], ".TXD")) {
		auto stream = RwStreamOpen(rwSTREAMFILENAME, rwSTREAMREAD, argv[1]);
		assert(stream && "failed to open input");

		auto texDict = LoadTxd(stream);
		assert(texDict && "Failed to LoadTxd");

		RwStreamClose(stream, nil);

		auto streamOut = RwStreamOpen(rwSTREAMFILENAME, rwSTREAMWRITE, argv[2]);
		assert(streamOut && "failed to open output");

		StoreTxd(texDict, streamOut);

		RwStreamClose(streamOut, nil);
	} else if (strstr(argv[1], ".dff") || strstr(argv[1], ".DFF")) {
		rw::Texture::setLoadTextures(false);
		rw::Texture::setCreateDummies(true);
		auto clump = LoadModelFile(argv[1]);
		FORLIST(lnk, clump->atomics) {
			rw::Atomic::fromClump(lnk)->instance();
		}
		StoreModelFile(argv[2], clump);
	} else {
		printf("Invalid format: %s\n", argv[1]);
		return 1;
	}
	
	return 0;
}