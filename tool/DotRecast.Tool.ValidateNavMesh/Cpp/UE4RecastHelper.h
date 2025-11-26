
namespace UE4RecastHelper
{
	struct NavMeshSetHeader
    {
        int32_t magic;
        int32_t version;
        int32_t numTiles;
        // dtNavMeshParams fields (manually serialized for DotRecast compatibility)
        // Note: DotRecast expects these fields in a specific order:
        // orig.X, orig.Y, orig.Z (3 floats)
        // tileWidth, tileHeight (2 floats)
        // maxTiles, maxPolys (2 ints)
        float origX;
        float origY;
        float origZ;
        float tileWidth;
        float tileHeight;
        int32_t maxTiles;
        int32_t maxPolys;
        // Note: maxVertsPerPoly is only written if version == NAVMESHSET_VERSION_RECAST4J
        // For standard version (NAVMESHSET_VERSION = 1), it's NOT written
    };

    struct NavMeshTileHeader
    {
        dtTileRef tileRef;  // Should be 64-bit (long), not int32_t
        int32_t dataSize;
        // Note: For standard version (NAVMESHSET_VERSION = 1), 
        // there's a 4-byte padding after dataSize (C struct alignment)
    };

    // C# 对应的结构体（用于 WriteTiles 函数，与 C# DotRecast.Detour.Io.NavMeshTileHeader 对应）
    // 对应 C#: public struct NavMeshTileHeader { public long tileRef; public int dataSize; }
    struct NavMeshTileHeaderCshp
    {
        int64_t tileRef;    // C# 中的 long 类型，64 位有符号整数
        int32_t dataSize;   // C# 中的 int 类型，32 位有符号整数
    };

    // Magic and version constants
    static const int NAVMESHSET_MAGIC = 'M' << 24 | 'S' << 16 | 'E' << 8 | 'T'; //'MSET';
    static const int NAVMESHSET_VERSION = 1;
    static const int NAVMESHSET_VERSION_RECAST4J = 0x8802;  // Recast4J version that includes maxVertsPerPoly
	
	// Byte order conversion helpers for cross-platform compatibility
	static int32_t SwapEndian32(int32_t value);
	static float SwapEndianFloat(float value);

	struct FVector3
	{
		dtReal X;
		dtReal Y;
		dtReal Z;
	public:
		inline FVector3() :X(0.f), Y(0.f), Z(0.f) {}
		inline FVector3(dtReal* InV) : X(InV[0]), Y(InV[1]), Z(InV[2]) {}
		inline FVector3(dtReal px, dtReal py, dtReal pz) : X(px), Y(py), Z(pz) {}
		FVector3(const FVector3&) = default;

		inline FVector3 operator-(const FVector3& V) const {
			return FVector3(X - V.X, Y - V.Y, Z - V.Z);
		}
		inline FVector3 operator+(const FVector3& V)const {
			return FVector3(X + V.X, Y + V.Y, Z + V.Z);
		}
		inline FVector3 operator-(const dtReal& V)const {
			return FVector3(X - V, Y - V, Z - V);
		}
		inline FVector3 operator+(const dtReal& V)const {
			return FVector3(X + V, Y + V, Z + V);
		}
		inline FVector3 GetAbs()const
		{
			return FVector3{ fabsf(X),fabsf(Y),fabsf(Z) };
		}
#ifdef USE_DETOUR_BUILT_INTO_UE4
		inline FVector3(FVector InUE4Vector) :X(InUE4Vector.X), Y(InUE4Vector.Y), Z(InUE4Vector.Z) {}

		inline FVector UE4Vector()const
		{
			return FVector{ X,Y,Z };
		}
#endif
	};

	FVector3 Recast2UnrealPoint(const FVector3& Vector);
	FVector3 Unreal2RecastPoint(const FVector3& Vector);

	void SerializedtNavMesh(const char* path, const dtNavMesh* mesh);
	dtNavMesh* DeSerializedtNavMesh(const char* path);

	 int findStraightPath(dtNavMesh* InNavMeshData, dtNavMeshQuery* InNavmeshQuery, const FVector3& start, const FVector3& end, std::vector<FVector3>& paths);
	 bool dtIsValidNavigationPoint(dtNavMesh* InNavMeshData, const FVector3& InPoint, const FVector3& InExtent = FVector3{ 10.f,10.f,10.f });
	 bool GetRandomPointInRadius(dtNavMeshQuery* InNavmeshQuery, dtQueryFilter* InQueryFilter,const FVector3& InOrigin,const FVector3& InRedius,FVector3& OutPoint);
};