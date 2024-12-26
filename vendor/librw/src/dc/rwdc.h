#pragma once
namespace rw {

#ifdef RW_DC
struct EngineOpenParams
{
};
#endif

namespace dc {

void registerPlatformPlugins(void);

extern Device renderdevice;

struct Im3DVertex
{
	V3d     position;
	uint8   r, g, b, a;
	float32 u, v;

	void setX(float32 x) { this->position.x = x; }
	void setY(float32 y) { this->position.y = y; }
	void setZ(float32 z) { this->position.z = z; }
	void setColor(uint8 r, uint8 g, uint8 b, uint8 a) {
		this->r = r; this->g = g; this->b = b; this->a = a; }
	void setU(float32 u) { this->u = u; }
	void setV(float32 v) { this->v = v; }

	float getX(void) { return this->position.x; }
	float getY(void) { return this->position.y; }
	float getZ(void) { return this->position.z; }
	RGBA getColor(void) { return makeRGBA(this->r, this->g, this->b, this->a); }
	float getU(void) { return this->u; }
	float getV(void) { return this->v; }
};

struct Im2DVertex
{
	float32 x, y, z, w;
	uint8   r, g, b, a;
	float32 u, v;

	void setScreenX(float32 x) { this->x = x; }
	void setScreenY(float32 y) { this->y = y; }
	void setScreenZ(float32 z) { this->z = z; }
	// This is a bit unefficient but we have to counteract GL's divide, so multiply
	void setCameraZ(float32 z) { this->w = z; }
	void setRecipCameraZ(float32 recipz) { this->w = 1.0f/recipz; }
	void setColor(uint8 r, uint8 g, uint8 b, uint8 a) {
		this->r = r; this->g = g; this->b = b; this->a = a; }
	void setU(float32 u, float recipz) { this->u = u; }
	void setV(float32 v, float recipz) { this->v = v; }

	float getScreenX(void) { return this->x; }
	float getScreenY(void) { return this->y; }
	float getScreenZ(void) { return this->z; }
	float getCameraZ(void) { return this->w; }
	float getRecipCameraZ(void) { return 1.0f/this->w; }
	RGBA getColor(void) { return makeRGBA(this->r, this->g, this->b, this->a); }
	float getU(void) { return this->u; }
	float getV(void) { return this->v; }
};


enum {
	VU_Lights	= 0x3d0
};

enum PS2Attribs {
	AT_V2_32	= 0x64000000,
	AT_V2_16	= 0x65000000,
	AT_V2_8		= 0x66000000,
	AT_V3_32	= 0x68000000,
	AT_V3_16	= 0x69000000,
	AT_V3_8		= 0x6A000000,
	AT_V4_32	= 0x6C000000,
	AT_V4_16	= 0x6D000000,
	AT_V4_8		= 0x6E000000,
	AT_UNSGN	= 0x00004000,

	AT_RW		= 0x6
};

// Not really types as in RW but offsets
enum PS2AttibTypes {
	AT_XYZ		= 0,
	AT_UV		= 1,
	AT_RGBA		= 2,
	AT_NORMAL	= 3
};

void *destroyNativeData(void *object, int32, int32);
Stream *readNativeData(Stream *stream, int32 len, void *object, int32, int32);
Stream *writeNativeData(Stream *stream, int32 len, void *object, int32, int32);
int32 getSizeNativeData(void *object, int32, int32);
void registerNativeDataPlugin(void);

struct PipeAttribute
{
	const char *name;
	uint32 attrib;
};

extern PipeAttribute attribXYZ;
extern PipeAttribute attribXYZW;
extern PipeAttribute attribUV;
extern PipeAttribute attribUV2;
extern PipeAttribute attribRGBA;
extern PipeAttribute attribNormal;
extern PipeAttribute attribWeights;

class MatPipeline : public rw::Pipeline
{
public:
	uint32 vifOffset;
	uint32 inputStride;
	// number of vertices for tri strips and lists
	uint32 triStripCount, triListCount;
	PipeAttribute *attribs[10];
	void (*instanceCB)(MatPipeline*, Geometry*, Mesh*, uint8**);
	void (*uninstanceCB)(MatPipeline*, Geometry*, uint32*, Mesh*, uint8**);
	void (*preUninstCB)(MatPipeline*, Geometry*);
	void (*postUninstCB)(MatPipeline*, Geometry*);
	// RW has more:
	//  instanceTestCB()
	//  resEntryAllocCB()
	//  bridgeCB()
	//  postMeshCB()
	//  vu1code
	//  primtype

	static uint32 getVertCount(uint32 top, uint32 inAttribs,
	                           uint32 outAttribs, uint32 outBufs) {
		return (top-outBufs)/(inAttribs*2+outAttribs*outBufs);
	}

	void init(void);
	static MatPipeline *create(void);
	void destroy(void);
	void dump(void);
	void setTriBufferSizes(uint32 inputStride, uint32 bufferSize);
	void instance(Geometry *g, void *inst, Mesh *m);
	uint8 *collectData(Geometry *g, void *inst, Mesh *m, uint8 *data[]);
};

struct Vertex {
	V3d       p;
	TexCoords t;
	TexCoords t1;
	RGBA      c;
	V3d       n;
	// skin
	float32 w[4];
	uint8   i[4];
};

void insertVertex(Geometry *geo, int32 i, uint32 mask, Vertex *v);

extern ObjPipeline *defaultObjPipe;
extern MatPipeline *defaultMatPipe;

void genericUninstanceCB(MatPipeline *pipe, Geometry *geo, uint32 flags[], Mesh *mesh, uint8 *data[]);
void genericPreCB(MatPipeline *pipe, Geometry *geo);	// skin and ADC
//void defaultUninstanceCB(MatPipeline *pipe, Geometry *geo, uint32 flags[], Mesh *mesh, uint8 *data[]);
void skinInstanceCB(MatPipeline *, Geometry *g, Mesh *m, uint8 **data);
//void skinUninstanceCB(MatPipeline*, Geometry *geo, uint32 flags[], Mesh *mesh, uint8 *data[]);

ObjPipeline *makeDefaultPipeline(void);
void dumpPipeline(rw::Pipeline *pipe);
void initSkin(void);
Stream *readNativeSkin(Stream *stream, int32, void *object, int32 offset);
Stream *writeNativeSkin(Stream *stream, int32 len, void *object, int32 offset);
int32 getSizeNativeSkin(void *object, int32 offset);
void initMatFX(void);

// ADC plugin

// Each element in adcBits corresponds to an index in Mesh->indices,
// this assumes the Mesh indices are ADC formatted.
// ADCData->numBits != Mesh->numIndices. ADCData->numBits is probably
// equal to Mesh->numIndices before the Mesh gets ADC formatted.
//
// Can't convert between ADC-formatted and non-ADC-formatted yet :(

struct ADCData
{
	bool32 adcFormatted;
	int8 *adcBits;
	int32 numBits;
};
extern int32 adcOffset;
void registerADCPlugin(void);

int8 *getADCbits(Geometry *geo);
int8 *getADCbitsForMesh(Geometry *geo, Mesh *mesh);
void convertADC(Geometry *g);
void unconvertADC(Geometry *geo);
void allocateADC(Geometry *geo);

// PDS plugin

Pipeline *getPDSPipe(uint32 data);
void registerPDSPipe(Pipeline *pipe);
void registerPDSPlugin(int32 n);
void registerPluginPDSPipes(void);

// Native Texture and Raster


extern int32 nativeRasterOffset;
void registerNativeRaster(void);
#define GETDCRASTEREXT(raster) PLUGINOFFSET(rw::dc::DcRasterRef, raster, rw::dc::nativeRasterOffset)

Texture *readNativeTexture(Stream *stream);
void writeNativeTexture(Texture *tex, Stream *stream);
uint32 getSizeNativeTexture(Texture *tex);
void processGeom(Geometry *geo);

}
}
