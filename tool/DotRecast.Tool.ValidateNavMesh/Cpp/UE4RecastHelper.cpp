// Copyright 2019 Lipeng Zha, Inc. All Rights Reserved.

#include "UE4RecastHelper.h"
#include "Detour/DetourStatus.h"
#include "Detour/DetourNavMeshQuery.h"
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cassert>


// Check system endianness
static bool IsLittleEndian()
{
	/*union {
		uint32_t i;
		uint8_t c[4];
	} test = { 0x01020304 };
	return test.c[0] == 0x04;*/
	//uint8 IsLittleEndian = uint8(PLATFORM_LITTLE_ENDIAN);
	return FPlatformProperties::IsLittleEndian();
}

#ifdef USE_DETOUR_BUILT_INTO_UE4
	#include "Resources/Version.h"
#endif

static const long RCN_NAVMESH_VERSION = 1;
static const int INVALID_NAVMESH_POLYREF = 0;
static const int MAX_POLYS = 256;
static const int NAV_ERROR_NEARESTPOLY = -2;
// DT_NAVMESH_MAGIC is commented out in UE5, define it ourselves
static const int DT_NAVMESH_MAGIC = 'D'<<24 | 'N'<<16 | 'A'<<8 | 'V';
#pragma warning (disable:4996)

bool UE4RecastHelper::dtIsValidNavigationPoint(dtNavMesh* InNavMeshData, const UE4RecastHelper::FVector3& InPoint, const UE4RecastHelper::FVector3& InExtent)
{
	bool bSuccess = false;

	using namespace UE4RecastHelper;

	if (!InNavMeshData) return bSuccess;

	FVector3 RcPoint = UE4RecastHelper::Unreal2RecastPoint(InPoint);
	const FVector3 ModifiedExtent = InExtent;
	FVector3 RcExtent = UE4RecastHelper::Unreal2RecastPoint(ModifiedExtent).GetAbs();
	FVector3 ClosestPoint;


	dtNavMeshQuery NavQuery;
#ifdef USE_DETOUR_BUILT_INTO_UE4
	dtQuerySpecialLinkFilter LinkFilter;
	NavQuery.init(InNavMeshData, 0, &LinkFilter);
	// UE_LOG(LogTemp, Warning, TEXT("CALL NavQuery.init(InNavMeshData, 0, &LinkFilter);"));
#else
	NavQuery.init(InNavMeshData, 0);
#endif

	dtPolyRef PolyRef;
	dtQueryFilter QueryFilter;

#ifdef USE_DETOUR_BUILT_INTO_UE4
	NavQuery.findNearestPoly2D(&RcPoint.X, &RcExtent.X, &QueryFilter, &PolyRef, (dtReal*)(&ClosestPoint));
	UE_LOG(LogTemp, Log, TEXT("dtIsValidNavigationPoint PolyRef is %ud."), PolyRef);
#else
	NavQuery.findNearestPoly(&RcPoint.X, &RcExtent.X, &QueryFilter, &PolyRef, (dtReal*)(&ClosestPoint));
#endif

	if (PolyRef > 0)
	{
		const FVector3& UnrealClosestPoint = UE4RecastHelper::Recast2UnrealPoint(ClosestPoint);
		const FVector3 ClosestPointDelta = UnrealClosestPoint - InPoint;
		if (-ModifiedExtent.X <= ClosestPointDelta.X && ClosestPointDelta.X <= ModifiedExtent.X
			&& -ModifiedExtent.Y <= ClosestPointDelta.Y && ClosestPointDelta.Y <= ModifiedExtent.Y
			&& -ModifiedExtent.Z <= ClosestPointDelta.Z && ClosestPointDelta.Z <= ModifiedExtent.Z)
		{
			bSuccess = true;
		}
	}

	return bSuccess;
}

int UE4RecastHelper::findStraightPath(dtNavMesh* InNavMeshData, dtNavMeshQuery* InNavmeshQuery, const FVector3& start, const FVector3& end, std::vector<FVector3>& paths)
{
	bool bSuccess = false;

	using namespace UE4RecastHelper;

	if (!InNavMeshData) return bSuccess;

	FVector3 RcStart = UE4RecastHelper::Unreal2RecastPoint(start);
	FVector3 RcEnd = UE4RecastHelper::Unreal2RecastPoint(end);
	FVector3 RcExtent{ 10.f,10.f,10.f };

	dtNavMeshQuery NavQuery;
#ifdef USE_DETOUR_BUILT_INTO_UE4
	dtQuerySpecialLinkFilter LinkFilter;
	NavQuery.init(InNavMeshData, 0, &LinkFilter);
	// UE_LOG(LogTemp, Warning, TEXT("CALL NavQuery.init(InNavMeshData, 0, &LinkFilter);"));
#else
	NavQuery.init(InNavMeshData, 0);
#endif


	FVector3 StartClosestPoint;
	FVector3 EndClosestPoint;

	dtPolyRef StartPolyRef;
	dtPolyRef EndPolyRef;
	dtQueryFilter QueryFilter;

#ifdef USE_DETOUR_BUILT_INTO_UE4
	NavQuery.findNearestPoly2D(&RcStart.X, &RcExtent.X, &QueryFilter, &StartPolyRef, (dtReal*)(&StartClosestPoint));
	// UE_LOG(LogTemp, Warning, TEXT("CALL findNearestPoly2D"));
#else
	NavQuery.findNearestPoly(&RcStart.X, &RcExtent.X, &QueryFilter, &StartPolyRef, (dtReal*)(&StartClosestPoint));
#endif

#ifdef USE_DETOUR_BUILT_INTO_UE4
	NavQuery.findNearestPoly2D(&RcEnd.X, &RcExtent.X, &QueryFilter, &EndPolyRef, (dtReal*)(&EndClosestPoint));
	// UE_LOG(LogTemp, Warning, TEXT("CALL findNearestPoly2D"));
#else
	NavQuery.findNearestPoly(&RcEnd.X, &RcExtent.X, &QueryFilter, &EndPolyRef, (dtReal*)(&EndClosestPoint));
#endif

#ifdef USE_DETOUR_BUILT_INTO_UE4
	UE_LOG(LogTemp, Log, TEXT("FindDetourPath StartPolyRef is %u."), StartPolyRef);
	UE_LOG(LogTemp, Log, TEXT("FindDetourPath EndPolyRef is %u."), EndPolyRef);

	UE_LOG(LogTemp, Log, TEXT("FindDetourPath StartClosestPoint is %s."), *FVector(StartClosestPoint.X,StartClosestPoint.Y,StartClosestPoint.Z).ToString());
	UE_LOG(LogTemp, Log, TEXT("FindDetourPath EndClosestPoint is %s."), *FVector(EndClosestPoint.X, EndClosestPoint.Y, EndClosestPoint.Z).ToString());
#endif

	dtQueryResult Result;

#if ENGINE_MAJOR_VERSION <=4 && ENGINE_MINOR_VERSION < 24
	dtStatus FindPathStatus = NavQuery.findPath(StartPolyRef, EndPolyRef, (dtReal*)(&StartClosestPoint), (dtReal*)(&EndClosestPoint), &QueryFilter, Result, NULL);
#else
	const float CostLimit = FLT_MAX;
	dtStatus FindPathStatus = NavQuery.findPath(StartPolyRef, EndPolyRef, (dtReal*)(&StartClosestPoint), (dtReal*)(&EndClosestPoint), CostLimit, &QueryFilter, Result, NULL);
#endif

#ifdef USE_DETOUR_BUILT_INTO_UE4
	UE_LOG(LogTemp, Log, TEXT("FindDetourPath FindPath return status is %u."), FindPathStatus);

	UE_LOG(LogTemp, Log, TEXT("FindDetourPath dtQueryResult size is %u."), Result.size());
#endif
//	InNavmeshQuery->findStraightPath(&StartNearestPt.X, &EndNearestPt.X, );
	return 0;
}

bool UE4RecastHelper::GetRandomPointInRadius(dtNavMeshQuery* InNavmeshQuery, dtQueryFilter* InQueryFilter, const FVector3& InOrigin, const FVector3& InRedius, FVector3& OutPoint)
{
	bool bStatus = false;
	dtNavMeshQuery* NavQuery = InNavmeshQuery;
	if (!NavQuery)
	{
		return false;
	}
	dtPolyRef OriginPolyRef;
	FVector3 ClosestPoint;
	FVector3 RcPoint = UE4RecastHelper::Unreal2RecastPoint(InOrigin);

#ifdef USE_DETOUR_BUILT_INTO_UE4
	InNavmeshQuery->findNearestPoly2D(&RcPoint.X, &InRedius.X, InQueryFilter, &OriginPolyRef, (dtReal*)(&ClosestPoint));
	// UE_LOG(LogTemp, Warning, TEXT("CALL findNearestPoly2D"));
#else
	NavQuery->findNearestPoly(&RcPoint.X, &InRedius.X, InQueryFilter, &OriginPolyRef, (dtReal*)(&ClosestPoint));
#endif

	dtPolyRef ResultPoly;
	FVector3 ResultPoint;
	auto NormalRand = []()->float
	{
		return std::rand() / (float)RAND_MAX;
	};

	dtStatus Status = NavQuery->findRandomPointAroundCircle(OriginPolyRef, &RcPoint.X, InRedius.X, InQueryFilter, NormalRand, &ResultPoly, &ResultPoint.X);

	if (dtStatusSucceed(Status))
	{
		OutPoint = UE4RecastHelper::Recast2UnrealPoint(ResultPoint);
		bStatus = true;
	}
	return bStatus;
}


// ------------------------------------------
// 基础写入工具（小端）
// ------------------------------------------
static void WriteInt(FILE* fp, int32_t v)
{
    fwrite(&v, sizeof(v), 1, fp);
}

static void WriteUInt(FILE* fp, uint32_t v)
{
    fwrite(&v, sizeof(v), 1, fp);
}

static void WriteInt64(FILE* fp, int64_t v)
{
    fwrite(&v, sizeof(v), 1, fp);
}

static void WriteUInt16(FILE* fp, uint16_t v)
{
    fwrite(&v, sizeof(v), 1, fp);
}

static void WriteUInt8(FILE* fp, uint8_t v)
{
    fwrite(&v, sizeof(v), 1, fp);
}

static void WriteFloat(FILE* fp, float v)
{
    fwrite(&v, sizeof(v), 1, fp);
}

// ------------------------------------------
// 可选：写入你之前的测试结构体（200 字节）
// 这里放个占位，建议直接用你现有的实现替换
// ------------------------------------------
static void WriteTestStructures(FILE* fp)
{
    // 如果你已经有一套验证用测试写入代码，可以直接搬过来。
    // 这里只给一个最小示例，保证总字节数是 200（和 ValidateTestStructures 里的 TEST_STRUCTS_SIZE 对齐）。
    // 为了简单起见，下面不展开，按你现有代码来会更安全。

    // TODO: 用你原来的测试写入逻辑替换这里。
    // 先简单写 200 个 0 占位（如果你不再需要测试区，也可以把 C# 的 TEST_STRUCTS_SIZE 改成 0）。
    std::vector<uint8_t> zeros(200, 0);
    fwrite(zeros.data(), 1, zeros.size(), fp);

    printf("[测试] 写入占位测试结构体 200 字节\n");
}

// ------------------------------------------
// 帮助函数：统计实际有数据的 tile 数量
// ------------------------------------------
static int CountTiles(const dtNavMesh* mesh)
{
    const int maxTiles = mesh->getMaxTiles();
    int count = 0;
    for (int i = 0; i < maxTiles; ++i)
    {
        const dtMeshTile* tile = mesh->getTile(i);
        if (!tile || !tile->header) continue;
        ++count;
    }
    return count;
}

// RcVec3fEx - C++ 结构体，与 C# RcVec3f 兼容用于序列化
// 对应 C#: public struct RcVec3f { public float X; public float Y; public float Z; }
struct RcVec3fEx
{
    float X;
    float Y;
    float Z;

    RcVec3fEx() : X(0.0f), Y(0.0f), Z(0.0f) {}
    RcVec3fEx(float x, float y, float z) : X(x), Y(y), Z(z) {}

    // 从 float[3] 数组构造（兼容 dtReal[3]）
    RcVec3fEx(const float* v) : X(v[0]), Y(v[1]), Z(v[2]) {}
};

// DtMeshHeaderEx - C++ 结构体，与 C# DtMeshHeader 兼容用于序列化
// 对应 C#: public class DtMeshHeader { ... }
// 注意：字段顺序和类型必须与 C# 版本完全一致，以确保序列化兼容性
struct DtMeshHeaderEx
{
	int32_t magic;              // Tile magic number. (Used to identify the data format.)
    int32_t version;            // Tile data format version number.
    int32_t x;                  // The x-position of the tile within the dtNavMesh tile grid. (x, y, layer)
    int32_t y;                  // The y-position of the tile within the dtNavMesh tile grid. (x, y, layer)
    int32_t layer;              // The layer of the tile within the dtNavMesh tile grid. (x, y, layer)
    int32_t userId;             // The user defined id of the tile.
    int32_t polyCount;          // The number of polygons in the tile.
    int32_t vertCount;          // The number of vertices in the tile.
    int32_t maxLinkCount;       // The number of allocated links.
    int32_t detailMeshCount;    // The number of sub-meshes in the detail mesh.
    int32_t detailVertCount;    // The number of unique vertices in the detail mesh. (In addition to the polygon vertices.)
    int32_t detailTriCount;     // The number of triangles in the detail mesh.
    int32_t bvNodeCount;        // The number of bounding volume nodes. (Zero if bounding volumes are disabled.)
    int32_t offMeshConCount;    // The number of off-mesh connections.
    int32_t offMeshBase;        // The index of the first polygon which is an off-mesh connection.
    float walkableHeight;       // The height of the agents using the tile.
    float walkableRadius;       // The radius of the agents using the tile.
    float walkableClimb;        // The maximum climb height of the agents using the tile.
    RcVec3fEx bmin;             // The minimum bounds of the tile's AABB. [(x, y, z)]
    RcVec3fEx bmax;             // The maximum bounds of the tile's AABB. [(x, y, z)]
    float bvQuantFactor;        // The bounding volume quantization factor.

    DtMeshHeaderEx()
        : magic(0)
        , version(0)
        , x(0)
        , y(0)
        , layer(0)
        , userId(0)
        , polyCount(0)
        , vertCount(0)
        , maxLinkCount(0)
        , detailMeshCount(0)
        , detailVertCount(0)
        , detailTriCount(0)
        , bvNodeCount(0)
        , offMeshConCount(0)
        , offMeshBase(0)
        , walkableHeight(50.0f)
        , walkableRadius(50.0f)
        , walkableClimb(120.0f)
        , bmin()
        , bmax()
        , bvQuantFactor(0.0f)
    {
    }
#if 0
    // 从 dtMeshHeader 和 dtNavMeshParams 构造（用于单个 tile）
    DtMeshHeaderEx(const dtMeshHeader* h, const dtNavMeshParams* params)
        : magic('D' << 24 | 'N' << 16 | 'A' << 8 | 'V')  // DT_NAVMESH_MAGIC
        , version(h->version)
        , x(h->x)
        , y(h->y)
        , layer((int32_t)h->layer)
        , userId(0)  // dtMeshHeader 中没有 userId，默认为 0
        , polyCount((int32_t)h->polyCount)
        , vertCount((int32_t)h->vertCount)
        , maxLinkCount((int32_t)h->maxLinkCount)
        , detailMeshCount((int32_t)h->detailMeshCount)
        , detailVertCount((int32_t)h->detailVertCount)
        , detailTriCount((int32_t)h->detailTriCount)
        , bvNodeCount((int32_t)h->bvNodeCount)
        , offMeshConCount((int32_t)h->offMeshConCount)
        , offMeshBase((int32_t)h->offMeshBase)
        , walkableHeight(params ? (float)params->walkableHeight : 0.0f)
        , walkableRadius(params ? (float)params->walkableRadius : 0.0f)
        , walkableClimb(params ? (float)params->walkableClimb : 0.0f)
        , bmin((const float*)h->bmin)  // 从 dtReal[3] 转换为 RcVec3fEx
        , bmax((const float*)h->bmax)  // 从 dtReal[3] 转换为 RcVec3fEx
        , bvQuantFactor(0.0f)
    {
        // 获取 bvQuantFactor
        if (params && h->resolution < DT_RESOLUTION_COUNT)
        {
            bvQuantFactor = (float)params->resolutionParams[h->resolution].bvQuantFactor;
        }
    }
#endif
    // 从 dtNavMesh 构造（用于文件集头部）
    // 注意：某些字段的含义在文件集头部中不同
    DtMeshHeaderEx(const dtNavMesh* mesh, int32_t numTiles)
        : magic(UE4RecastHelper::NAVMESHSET_MAGIC)
        , version(UE4RecastHelper::NAVMESHSET_VERSION)
        , x(0)  // 文件集头部中不使用
        , y(0)  // 文件集头部中不使用
        , layer(numTiles)  // 复用 layer 字段存储 numTiles
        , userId(0)
        , polyCount(0)  // 文件集头部中不使用
        , vertCount(0)  // 文件集头部中不使用
        , maxLinkCount(0)  // 文件集头部中不使用
        , detailMeshCount(0)  // 文件集头部中不使用
        , detailVertCount(0)  // 文件集头部中不使用
        , detailTriCount(0)  // 文件集头部中不使用
        , bvNodeCount(0)  // 文件集头部中不使用
        , offMeshConCount(0)  // 文件集头部中不使用
        , offMeshBase(0)  // 文件集头部中不使用
        , walkableHeight(50.0f)
        , walkableRadius(50.0f)
        , walkableClimb(120.0f)
        , bmin()
        , bmax()
        , bvQuantFactor(0.0f)
    {
        const dtNavMeshParams* params = mesh ? mesh->getParams() : nullptr;
        if (params)
        {
            // 将 dtNavMeshParams 的信息映射到 DtMeshHeaderEx 的字段
            // orig 映射到 bmin（复用字段）
            bmin.X = (float)params->orig[0];
            bmin.Y = (float)params->orig[1];
            bmin.Z = (float)params->orig[2];
            
            // tileWidth 和 tileHeight 映射到 bmax（复用字段）
            bmax.X = (float)params->tileWidth;
            bmax.Y = (float)params->tileHeight;
            bmax.Z = 0.0f;
            
            // maxTiles 和 maxPolys 映射到 walkableHeight 和 walkableRadius（复用字段）
            walkableHeight = (float)params->maxTiles;
            walkableRadius = (float)params->maxPolys;
            
            // walkableClimb 保持为 0 或可以存储其他信息
            walkableClimb = 0.0f;
        }
    }
};

// ------------------------------------------
// 写 NavMeshSetHeader（文件集头部）
// 对应 C# 中 NavMeshSetHeader + DtNavMeshParamWriter
// 使用 DtMeshHeaderEx 结构体进行序列化
// ------------------------------------------
static void WriteHeader(FILE* fp, const dtNavMesh* mesh, int numTiles)
{
	using namespace UE4RecastHelper;

    long pos = ftell(fp);
    printf("[WriteHeader] 开始写入文件集头部\n");
    printf("[WriteHeader] 文件位置: %ld\n", pos);

    // 构造 DtMeshHeaderEx 结构体（用于文件集头部）
    DtMeshHeaderEx headerEx(mesh, numTiles);

    // 直接写入整个结构体（与 C# 版本兼容）
    fwrite(&headerEx, sizeof(DtMeshHeaderEx), 1, fp);

    printf("[WriteHeader] 写入 magic: 0x%08x\n", headerEx.magic);
    printf("[WriteHeader] 写入 version: %d\n", headerEx.version);
    printf("[WriteHeader] 写入 numTiles (layer): %d\n", headerEx.layer);
    const dtNavMeshParams* params = mesh ? mesh->getParams() : nullptr;
    if (params)
    {
        printf("[WriteHeader] 写入 params: orig=(%.3f, %.3f, %.3f), tileSize=(%.3f, %.3f), maxTiles=%.0f, maxPolys=%.0f\n",
            headerEx.bmin.X, headerEx.bmin.Y, headerEx.bmin.Z,
            headerEx.bmax.X, headerEx.bmax.Y,
            headerEx.walkableHeight, headerEx.walkableRadius);
    }
    printf("[WriteHeader] 头部写入完成，总大小: %ld 字节，文件位置: %ld\n", ftell(fp) - pos, ftell(fp));
}

// ------------------------------------------
// 写 MeshHeader（单个 tile 的 DtMeshHeader）
// 对应 C# DtMeshDataReader.Read 里 header 那一段
// ------------------------------------------
static void WriteMeshHeader(FILE* fp, const dtMeshHeader* header)
{
    using namespace UE4RecastHelper;

    long pos = ftell(fp);
    printf("[WriteMeshHeader] 开始写入，文件位置: %ld\n", pos);

    // magic + version
    WriteInt(fp, DT_NAVMESH_MAGIC);
    WriteInt(fp, DT_NAVMESH_VERSION);

    // 和 C# 读取顺序完全一致：
    // 注意：UE5 的 dtMeshHeader 结构不同，不包含 userId, walkableHeight 等字段
    WriteInt(fp, header->x);
    WriteInt(fp, header->y);
    WriteInt(fp, header->layer);
    WriteInt(fp, 0); // userId - UE5 中不存在，写 0
    WriteInt(fp, header->polyCount);
    WriteInt(fp, header->vertCount);
    WriteInt(fp, header->maxLinkCount);
    WriteInt(fp, header->detailMeshCount);
    WriteInt(fp, header->detailVertCount);
    WriteInt(fp, header->detailTriCount);
    WriteInt(fp, header->bvNodeCount);
    WriteInt(fp, header->offMeshConCount);
    WriteInt(fp, header->offMeshBase);

    WriteFloat(fp, 80.0f); // walkableHeight - UE5 中不存在，写 0
    WriteFloat(fp, 50.0f); // walkableRadius - UE5 中不存在，写 0
    WriteFloat(fp, 120.0f); // walkableClimb - UE5 中不存在，写 0

    WriteFloat(fp, (float)header->bmin[0]);
    WriteFloat(fp, (float)header->bmin[1]);
    WriteFloat(fp, (float)header->bmin[2]);

    WriteFloat(fp, (float)header->bmax[0]);
    WriteFloat(fp, (float)header->bmax[1]);
    WriteFloat(fp, (float)header->bmax[2]);

    WriteFloat(fp, 0.0f); // bvQuantFactor - UE5 中不存在，写 0

    printf("[WriteMeshHeader] 完成，文件位置: %ld\n", ftell(fp));
}

// ------------------------------------------
// 写顶点数组：float[3 * vertCount]
// ------------------------------------------
static void WriteVerts(FILE* fp, const dtMeshHeader* header, const dtReal* verts)
{
    const int count = header->vertCount * 3;
    // 按 float 直接写，C# 端逐个 GetFloat
    // dtReal 在 UE5 中通常是 float，直接转换
    fwrite(verts, sizeof(dtReal), count, fp);
    printf("[WriteVerts] 写入 %d 个顶点 (%.d floats), 文件位置: %ld\n",
        header->vertCount, count, ftell(fp));
}

// ------------------------------------------
// 写 polys：C 版本的 dtPoly 序列化
// 对应 C# ReadPolys
// ------------------------------------------
static void WritePolys(FILE* fp, const dtNavMesh* mesh,
    const dtMeshHeader* header, const dtPoly* polys)
{
    using namespace UE4RecastHelper;

    const int polyCount = header->polyCount;
    const int maxVertsPerPoly = DT_VERTS_PER_POLYGON; // 使用常量而不是方法调用

    printf("[WritePolys] 写入 %d 个多边形, maxVertsPerPoly=%d, 文件位置: %ld\n",
        polyCount, maxVertsPerPoly, ftell(fp));

    for (int i = 0; i < polyCount; ++i)
    {
        const dtPoly& p = polys[i];

        // firstLink (C# 里如果 version < NO_POLY_FIRSTLINK，会读一个 int 丢掉)
        WriteInt(fp, (int)p.firstLink);

        // verts
        for (int j = 0; j < maxVertsPerPoly; ++j)
            WriteUInt16(fp, p.verts[j]);

        // neis
        for (int j = 0; j < maxVertsPerPoly; ++j)
            WriteUInt16(fp, p.neis[j]);

        // flags
        WriteUInt16(fp, p.flags);
        // vertCount
        WriteUInt8(fp, p.vertCount);
        // areaAndtype
        WriteUInt8(fp, p.areaAndtype);
    }
}

// ------------------------------------------
// 写 Link 占位区域：maxLinkCount * 16 字节，全 0
// C# 在 cCompatibility 下会直接跳过这些 bytes
// ------------------------------------------
static void WriteLinkPlaceholder(FILE* fp, const dtMeshHeader* header, bool is32Bit)
{
    const int linkSize = is32Bit ? 12 : 16;
    const int totalSize = header->maxLinkCount * linkSize;

    std::vector<uint8_t> zeros(totalSize, 0);
    fwrite(zeros.data(), 1, zeros.size(), fp);

    printf("[WriteLinkPlaceholder] 写入 %d 个链接占位符, 每个 %d 字节, 总计 %d 字节, 文件位置: %ld\n",
        header->maxLinkCount, linkSize, totalSize, ftell(fp));
}

// ------------------------------------------
// 写 detailMeshes：dtPolyDetail[]
// ------------------------------------------
static void WritePolyDetails(FILE* fp, const dtMeshHeader* header,
    const dtPolyDetail* dmeshes, bool cCompatibility)
{
    using namespace UE4RecastHelper;

    const int n = header->detailMeshCount;

    printf("[WritePolyDetails] 写入 %d 个细节网格, 文件位置: %ld\n", n, ftell(fp));

    for (int i = 0; i < n; ++i)
    {
        const dtPolyDetail& d = dmeshes[i];
        WriteUInt(fp, d.vertBase);
        WriteUInt(fp, d.triBase);
        WriteUInt8(fp, d.vertCount);
        WriteUInt8(fp, d.triCount);
        if (cCompatibility)
        {
            // C# 里读一个 padding 的 short
            WriteUInt16(fp, 0);
        }
    }
}

// ------------------------------------------
// 写 detailVerts：float[3 * detailVertCount]
// ------------------------------------------
static void WriteDetailVerts(FILE* fp, const dtMeshHeader* header,
    const dtReal* detailVerts)
{
    const int count = header->detailVertCount * 3;
    fwrite(detailVerts, sizeof(dtReal), count, fp);
    printf("[WriteDetailVerts] 写入 %d 个细节顶点 (%.d floats), 文件位置: %ld\n",
        header->detailVertCount, count, ftell(fp));
}

// ------------------------------------------
// 写 detailTris：uint8[4 * detailTriCount]
// ------------------------------------------
static void WriteDTris(FILE* fp, const dtMeshHeader* header,
    const unsigned char* detailTris)
{
    const int count = header->detailTriCount * 4;
    fwrite(detailTris, sizeof(unsigned char), count, fp);
    printf("[WriteDTris] 写入 %d 个细节三角形 (%d bytes), 文件位置: %ld\n",
        header->detailTriCount, count, ftell(fp));
}

// ------------------------------------------
// 写 BVTree：dtBVNode[]，C 兼容模式用 16bit bmin/bmax
// ------------------------------------------
static void WriteBVTree(FILE* fp, const dtMeshHeader* header,
    const dtBVNode* nodes, bool cCompatibility)
{
    const int n = header->bvNodeCount;
    printf("[WriteBVTree] 写入 %d 个BV节点, 文件位置: %ld\n", n, ftell(fp));

    if (!cCompatibility)
    {
        // 如果将来支持 recast4j 的 32bit BVTREE，可以在这里加另一分支。
        // 使用 check 而不是 assert，因为这是 UE 代码
        checkf(false, TEXT("32bit BV tree not implemented here"));
    }

    for (int i = 0; i < n; ++i)
    {
        const dtBVNode& node = nodes[i];
        WriteUInt16(fp, node.bmin[0]);
        WriteUInt16(fp, node.bmin[1]);
        WriteUInt16(fp, node.bmin[2]);
        WriteUInt16(fp, node.bmax[0]);
        WriteUInt16(fp, node.bmax[1]);
        WriteUInt16(fp, node.bmax[2]);
        WriteInt(fp, node.i);
    }
}

// ------------------------------------------
// 写 offMeshCons：dtOffMeshConnection[]
// ------------------------------------------
static void WriteOffMeshCons(FILE* fp, const dtMeshHeader* header,
    const dtOffMeshConnection* cons)
{
    using namespace UE4RecastHelper;

    const int n = header->offMeshConCount;
    printf("[WriteOffMeshCons] 写入 %d 个离网连接, 文件位置: %ld\n", n, ftell(fp));

    for (int i = 0; i < n; ++i)
    {
        const dtOffMeshConnection& c = cons[i];
        for (int k = 0; k < 6; ++k)
            WriteFloat(fp, c.pos[k]);
        WriteFloat(fp, c.rad);
        WriteUInt16(fp, c.poly);
        WriteUInt8(fp, c.flags);
        WriteUInt8(fp, c.side);
        WriteUInt(fp, c.userId);
    }
}

// ------------------------------------------
// 写单个 Tile 的完整数据块，返回 dataSize（字节数）
// 对应 C# DtMeshDataReader.Read 的逆过程
// ------------------------------------------
static int WriteTileData(FILE* fp, const dtNavMesh* mesh, const dtMeshTile* tile,
    bool cCompatibility, bool is32Bit)
{
    using namespace UE4RecastHelper;

    long start = ftell(fp);
    printf("[WriteTileData] 开始写入瓦片数据，文件位置: %ld\n", start);

    const dtMeshHeader* header = tile->header;

    // 1) header
    WriteMeshHeader(fp, header);

    // 2) verts
    WriteVerts(fp, header, tile->verts);

    // 3) polys
    WritePolys(fp, mesh, header, tile->polys);

    // 4) link 占位
    if (cCompatibility)
    {
        WriteLinkPlaceholder(fp, header, is32Bit);
    }

    // 5) detailMeshes
    WritePolyDetails(fp, header, tile->detailMeshes, cCompatibility);

    // 6) detailVerts
    WriteDetailVerts(fp, header, tile->detailVerts);

    // 7) detailTris
    WriteDTris(fp, header, tile->detailTris);

    // 8) BVTree
    WriteBVTree(fp, header, tile->bvTree, cCompatibility);

    // 9) OffMeshCons
    WriteOffMeshCons(fp, header, tile->offMeshCons);

    long end = ftell(fp);
    int dataSize = static_cast<int>(end - start);

    printf("[WriteTileData] 完成，总大小: %d 字节，文件位置: %ld\n", dataSize, end);
    return dataSize;
}

// ------------------------------------------
// 写所有 Tile（含 tileHeader）
// 对应 C# DtMeshSetReader.ReadTiles
// ------------------------------------------
static void WriteTiles(FILE* fp, const dtNavMesh* mesh, int numTiles)
{
    using namespace UE4RecastHelper;

    const bool is32Bit = false;                         // 你现在 C# 读的是 64bit 版本
    const bool cCompatibility = true;                          // version == 1

    const int maxTiles = mesh->getMaxTiles();
    printf("[WriteTiles] 开始写入瓦片，总槽数=%d，实际瓦片数=%d，文件位置: %ld\n",
        maxTiles, numTiles, ftell(fp));

    int writtenTiles = 0;

    for (int i = 0; i < maxTiles; ++i)
    {
        const dtMeshTile* tile = mesh->getTile(i);
        if (!tile || !tile->header)
            continue;

        const dtMeshHeader* header = tile->header;
        printf("[WriteTiles] 开始写入瓦片 %d, 坐标=(%d, %d, layer=%d), polyCount=%d, vertCount=%d, 文件位置: %ld\n",
            i, header->x, header->y, header->layer,
            header->polyCount, header->vertCount,
            ftell(fp));

        // 生成 tileRef: 为了简单，用原生 dtNavMesh::getTileRef(tile) 转成 64bit
        // （如果你想保持之前那种 0x2000... 的值，可以改成自己的 Encode 函数）
        dtTileRef tileRef32 = mesh->getTileRef(tile);
        uint64_t  tileRef = (uint64_t)tileRef32;

        long tileHeaderPos = ftell(fp);
        printf("[WriteTiles] 瓦片 %d: tileHeader 起始位置: %ld\n", i, tileHeaderPos);

        // 写 tileRef (int64)
        WriteInt64(fp, (int64_t)tileRef);
        printf("[WriteTiles] 瓦片 %d: 已写入 tileRef=0x%016llx, 文件位置: %ld\n",
            i, (unsigned long long)tileRef, ftell(fp));

        // 占位 dataSize
        long dataSizePos = ftell(fp);
        WriteInt(fp, 0);
        printf("[WriteTiles] 瓦片 %d: 已写入 dataSize 占位符, 文件位置: %ld\n", i, ftell(fp));

        // C 兼容 padding
        WriteInt(fp, 0);
        printf("[WriteTiles] 瓦片 %d: 已写入 padding, 文件位置: %ld\n", i, ftell(fp));

        long tileDataStart = ftell(fp);
        printf("[WriteTiles] 瓦片 %d: tileData 起始位置: %ld\n", i, tileDataStart);

        // 写实际 tile 数据
        int dataSize = WriteTileData(fp, mesh, tile, cCompatibility, is32Bit);

        long tileDataEnd = ftell(fp);

        // 回填 dataSize
        long cur = ftell(fp);
        fseek(fp, dataSizePos, SEEK_SET);
        WriteInt(fp, dataSize);
        fseek(fp, cur, SEEK_SET);

        printf("[WriteTiles] 瓦片 %d: WriteTileData 返回 dataSize=%d\n", i, dataSize);
        printf("[WriteTiles] 瓦片 %d: 完成，tileRef=0x%016llx, dataSize=%d, 总大小: %ld 字节\n",
            i, (unsigned long long)tileRef, dataSize, tileDataEnd - tileHeaderPos);

        ++writtenTiles;
    }

    printf("[WriteTiles] 所有瓦片写入完成，实际写入瓦片数: %d\n", writtenTiles);
}

// ------------------------------------------
// 公开接口：UE4RecastHelper::SerializedtNavMesh
// ------------------------------------------
void UE4RecastHelper::SerializedtNavMesh(const char* path, const dtNavMesh* mesh)
{
    if (!mesh || !path)
        return;

    FILE* fp = std::fopen(path, "wb");
    if (!fp)
    {
        printf("[SerializedtNavMesh] 打开文件失败: %s\n", path);
        return;
    }

    printf("[SerializedtNavMesh] 开始序列化 navmesh 到文件: %s\n", path);

    // 1) 可选：测试结构体区（200 字节）
    //WriteTestStructures(fp);

    // 2) header
    int numTiles = CountTiles(mesh);
    WriteHeader(fp, mesh, numTiles);

    // 3) tiles
    WriteTiles(fp, mesh, numTiles);

    printf("[SerializedtNavMesh] 序列化完成，最终文件大小: %ld 字节\n", ftell(fp));

    std::fclose(fp);
}


dtNavMesh* UE4RecastHelper::DeSerializedtNavMesh(const char* path)
{

	std::FILE* fp = std::fopen(path, "rb");
	if (!fp) return 0;

	using namespace UE4RecastHelper;
	// Read header.
	NavMeshSetHeader header;
	size_t sizenum = sizeof(NavMeshSetHeader);
	size_t readLen = std::fread(&header, sizenum, 1, fp);
	if (readLen != 1)
	{
		std::fclose(fp);
		return 0;
	}
	
	// Convert byte order if needed (file is stored in big-endian)
	if (IsLittleEndian())
	{
		header.magic = (header.magic);
		header.version = (header.version);
		header.numTiles = (header.numTiles);
		header.maxTiles = (header.maxTiles);
		header.maxPolys = (header.maxPolys);
		//header.maxVertsPerPoly = (header.maxVertsPerPoly);
		// Floats also need byte order conversion
		header.origX = (header.origX);
		header.origY = (header.origY);
		header.origZ = (header.origZ);
		header.tileWidth = (header.tileWidth);
		header.tileHeight = (header.tileHeight);
	}
	
	if (header.magic != NAVMESHSET_MAGIC)
	{
		std::fclose(fp);
		return 0;
	}
	if (header.version != NAVMESHSET_VERSION)
	{
		std::fclose(fp);
		return 0;
	}

	dtNavMesh* mesh = dtAllocNavMesh();
	if (!mesh)
	{
		std::fclose(fp);
		return 0;
	}
	
	// Reconstruct dtNavMeshParams from manually serialized fields
	dtNavMeshParams params;
	params.orig[0] = header.origX;
	params.orig[1] = header.origY;
	params.orig[2] = header.origZ;
	params.tileWidth = header.tileWidth;
	params.tileHeight = header.tileHeight;
	params.maxTiles = header.maxTiles;
	params.maxPolys = header.maxPolys;
	
	// Initialize navmesh with reconstructed params
	// Note: dtNavMesh::init() only accepts dtNavMeshParams* parameter
	// maxVertsPerPoly is stored in header for DotRecast compatibility but not used here
	dtStatus status = mesh->init(&params);
	if (dtStatusFailed(status))
	{
		std::fclose(fp);
		return 0;
	}

	// Read tiles.
	for (int i = 0; i < header.numTiles; ++i)
	{
		NavMeshTileHeader tileHeader;
		readLen = std::fread(&tileHeader, sizeof(tileHeader), 1, fp);
		if (readLen != 1)
		{
			std::fclose(fp);
			return 0;
		}
		
		// Convert tile header byte order if needed (file stored in big-endian)
		if (IsLittleEndian())
		{
			tileHeader.tileRef = static_cast<dtTileRef>(SwapEndian32(static_cast<int32_t>(tileHeader.tileRef)));
			tileHeader.dataSize = SwapEndian32(tileHeader.dataSize);
		}

		if (!tileHeader.tileRef || !tileHeader.dataSize)
			break;

		unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_TEMP);
		if (!data) break;
		std::memset(data, 0, tileHeader.dataSize);
		readLen = fread(data, tileHeader.dataSize, 1, fp);
		if (readLen != 1)
		{
			dtFree(data, DT_ALLOC_TEMP);
			fclose(fp);
			return 0;
		}

		mesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0);
	}

	std::fclose(fp);

	return mesh;
}

namespace UE4RecastHelper
{
	FVector3 Recast2UnrealPoint(const FVector3& Vector)
	{
		return FVector3(-Vector.X, -Vector.Z, Vector.Y);
	}

	FVector3 Unreal2RecastPoint(const FVector3& Vector)
	{
		return FVector3(-Vector.X, Vector.Z, -Vector.Y);
	}
	
	// Byte order conversion helpers
	int32_t SwapEndian32(int32_t value)
	{
		return ((value & 0x000000FF) << 24) |
			   ((value & 0x0000FF00) << 8) |
			   ((value & 0x00FF0000) >> 8) |
			   ((value & 0xFF000000) >> 24);
	}
	
	float SwapEndianFloat(float value)
	{
		union {
			float f;
			uint32_t i;
		} u;
		u.f = value;
		u.i = SwapEndian32(static_cast<int32_t>(u.i));
		return u.f;
	}
};