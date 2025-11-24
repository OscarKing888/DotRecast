
// Helper function to write an int32 in little-endian byte order
void WriteInt32LE(std::FILE* fp, int32_t value)
{
    uint8_t bytes[4];
    bytes[0] = (uint8_t)(value & 0xFF);
    bytes[1] = (uint8_t)((value >> 8) & 0xFF);
    bytes[2] = (uint8_t)((value >> 16) & 0xFF);
    bytes[3] = (uint8_t)((value >> 24) & 0xFF);
    std::fwrite(bytes, sizeof(uint8_t), 4, fp);
}

// Helper function to write a float in little-endian byte order
void WriteFloatLE(std::FILE* fp, float value)
{
    union { float f; int32_t i; } converter;
    converter.f = value;
    WriteInt32LE(fp, converter.i);
}

// Helper function to write a short in little-endian byte order
void WriteInt16LE(std::FILE* fp, int16_t value)
{
    uint8_t bytes[2];
    bytes[0] = (uint8_t)(value & 0xFF);
    bytes[1] = (uint8_t)((value >> 8) & 0xFF);
    std::fwrite(bytes, sizeof(uint8_t), 2, fp);
}

// Helper function to write a single tile's mesh data in DotRecast format
// Returns the number of bytes written
size_t WriteTileData(std::FILE* fp, const dtMeshTile* tile, const dtNavMesh* mesh, bool cCompatibility)
{
    if (!tile || !tile->header || !mesh) return 0;

    long startPos = std::ftell(fp);

    const dtMeshHeader* h = tile->header;
    const dtNavMeshParams* params = mesh->getParams();

    // Write dtMeshHeader - 必须按照 C# DtMeshDataWriter.cs 的精确顺序

    // 1. Magic - 关键：必须正确写入
    // DT_NAVMESH_MAGIC = 'D' << 24 | 'N' << 16 | 'A' << 8 | 'V'
    // = 0x444E4156 (大端序表示) 或 0x56414E44 (小端序表示)
    // 按小端序写入应该是：56 41 4E 44
    int32_t magic = 'D' << 24 | 'N' << 16 | 'A' << 8 | 'V';
    WriteInt32LE(fp, magic);

    // 2. Version
    int32_t version = cCompatibility ? 7 : 0x8809;
    WriteInt32LE(fp, version);

    // 3-15. Header fields (all as int32_t)
    WriteInt32LE(fp, h->x);
    WriteInt32LE(fp, h->y);
    WriteInt32LE(fp, (int32_t)h->layer);
    WriteInt32LE(fp, 0); // userId

    WriteInt32LE(fp, (int32_t)h->polyCount);
    WriteInt32LE(fp, (int32_t)h->vertCount);
    WriteInt32LE(fp, (int32_t)h->maxLinkCount);
    WriteInt32LE(fp, (int32_t)h->detailMeshCount);
    WriteInt32LE(fp, (int32_t)h->detailVertCount);
    WriteInt32LE(fp, (int32_t)h->detailTriCount);
    WriteInt32LE(fp, (int32_t)h->bvNodeCount);
    WriteInt32LE(fp, (int32_t)h->offMeshConCount);
    WriteInt32LE(fp, (int32_t)h->offMeshBase);

    // 16-18. walkableHeight, walkableRadius, walkableClimb
    float walkableHeight = params ? params->walkableHeight : 0.0f;
    float walkableRadius = params ? params->walkableRadius : 0.0f;
    float walkableClimb = params ? params->walkableClimb : 0.0f;
    WriteFloatLE(fp, walkableHeight);
    WriteFloatLE(fp, walkableRadius);
    WriteFloatLE(fp, walkableClimb);

    // 19-24. bmin, bmax (6 floats)
    WriteFloatLE(fp, h->bmin[0]);
    WriteFloatLE(fp, h->bmin[1]);
    WriteFloatLE(fp, h->bmin[2]);
    WriteFloatLE(fp, h->bmax[0]);
    WriteFloatLE(fp, h->bmax[1]);
    WriteFloatLE(fp, h->bmax[2]);

    // 25. bvQuantFactor
    float bvQuantFactor = 0.0f;
    if (params && h->resolution < DT_RESOLUTION_COUNT)
    {
        bvQuantFactor = params->resolutionParams[h->resolution].bvQuantFactor;
    }
    WriteFloatLE(fp, bvQuantFactor);

    // 26. Write vertices
    for (int i = 0; i < h->vertCount * 3; ++i)
    {
        WriteFloatLE(fp, tile->verts[i]);
    }

    // 27. Write polygons
    for (int i = 0; i < h->polyCount; ++i)
    {
        const dtPoly* p = &tile->polys[i];
        if (cCompatibility)
        {
            WriteInt32LE(fp, 0xFFFF);
        }
        for (int j = 0; j < DT_VERTS_PER_POLYGON; ++j)
        {
            WriteInt16LE(fp, (int16_t)p->verts[j]);
        }
        for (int j = 0; j < DT_VERTS_PER_POLYGON; ++j)
        {
            WriteInt16LE(fp, (int16_t)p->neis[j]);
        }
        WriteInt16LE(fp, (int16_t)p->flags);
        uint8_t polyVertCount = (uint8_t)p->vertCount;
        uint8_t areaAndtype = (uint8_t)(p->getArea() | (p->getType() << 6));
        std::fwrite(&polyVertCount, sizeof(uint8_t), 1, fp);
        std::fwrite(&areaAndtype, sizeof(uint8_t), 1, fp);
    }

    // 28. Write link placeholder
    if (cCompatibility)
    {
        uint8_t zero = 0;
        for (int i = 0; i < h->maxLinkCount * 16; ++i)
        {
            std::fwrite(&zero, sizeof(uint8_t), 1, fp);
        }
    }

    // 29. Write detail meshes
    for (int i = 0; i < h->detailMeshCount; ++i)
    {
        const dtPolyDetail* dm = &tile->detailMeshes[i];
        WriteInt32LE(fp, (int32_t)dm->vertBase);
        WriteInt32LE(fp, (int32_t)dm->triBase);
        uint8_t detailMeshVertCount = (uint8_t)dm->vertCount;
        uint8_t triCount = (uint8_t)dm->triCount;
        std::fwrite(&detailMeshVertCount, sizeof(uint8_t), 1, fp);
        std::fwrite(&triCount, sizeof(uint8_t), 1, fp);
        if (cCompatibility)
        {
            WriteInt16LE(fp, 0);
        }
    }

    // 30. Write detail vertices
    for (int i = 0; i < h->detailVertCount * 3; ++i)
    {
        WriteFloatLE(fp, tile->detailVerts[i]);
    }

    // 31. Write detail triangles
    std::fwrite(tile->detailTris, sizeof(uint8_t), h->detailTriCount * 4, fp);

    // 32. Write BV tree
    for (int i = 0; i < h->bvNodeCount; ++i)
    {
        const dtBVNode* node = &tile->bvTree[i];
        if (cCompatibility)
        {
            WriteInt16LE(fp, (int16_t)node->bmin[0]);
            WriteInt16LE(fp, (int16_t)node->bmin[1]);
            WriteInt16LE(fp, (int16_t)node->bmin[2]);
            WriteInt16LE(fp, (int16_t)node->bmax[0]);
            WriteInt16LE(fp, (int16_t)node->bmax[1]);
            WriteInt16LE(fp, (int16_t)node->bmax[2]);
        }
        else
        {
            WriteInt32LE(fp, (int32_t)node->bmin[0]);
            WriteInt32LE(fp, (int32_t)node->bmin[1]);
            WriteInt32LE(fp, (int32_t)node->bmin[2]);
            WriteInt32LE(fp, (int32_t)node->bmax[0]);
            WriteInt32LE(fp, (int32_t)node->bmax[1]);
            WriteInt32LE(fp, (int32_t)node->bmax[2]);
        }
        WriteInt32LE(fp, node->i);
    }

    // 33. Write off-mesh connections
    for (int i = 0; i < h->offMeshConCount; ++i)
    {
        const dtOffMeshConnection* con = &tile->offMeshCons[i];
        for (int j = 0; j < 6; ++j)
        {
            WriteFloatLE(fp, con->pos[j]);
        }
        WriteFloatLE(fp, con->rad);
        WriteInt16LE(fp, (int16_t)con->poly);
        uint8_t flags = (uint8_t)con->flags;
        uint8_t side = (uint8_t)con->side;
        std::fwrite(&flags, sizeof(uint8_t), 1, fp);
        std::fwrite(&side, sizeof(uint8_t), 1, fp);
        WriteInt32LE(fp, (int32_t)con->userId);
    }

    long endPos = std::ftell(fp);
    return (size_t)(endPos - startPos);
}

void UE4RecastHelper::SerializedtNavMesh(const char* path, const dtNavMesh* mesh)
{
    using namespace UE4RecastHelper;

    if (!mesh) return;

    std::FILE* fp = std::fopen(path, "wb");
    if (!fp)
        return;

    // Write NavMeshSetHeader
    NavMeshSetHeader header;
    header.magic = NAVMESHSET_MAGIC;
    header.version = NAVMESHSET_VERSION;
    header.numTiles = 0;

    // Count valid tiles
    for (int i = 0; i < mesh->getMaxTiles(); ++i)
    {
        const dtMeshTile* tile = mesh->getTile(i);
        if (!tile || !tile->header || !tile->dataSize) continue;
        header.numTiles++;
    }

    // Get navmesh params
    const dtNavMeshParams* params = mesh->getParams();
    if (params)
    {
        header.origX = params->orig[0];
        header.origY = params->orig[1];
        header.origZ = params->orig[2];
        header.tileWidth = params->tileWidth;
        header.tileHeight = params->tileHeight;
        header.maxTiles = params->maxTiles;
        header.maxPolys = params->maxPolys;
    }
    else
    {
        header.origX = 0.0f;
        header.origY = 0.0f;
        header.origZ = 0.0f;
        header.tileWidth = 0.0f;
        header.tileHeight = 0.0f;
        header.maxTiles = 0;
        header.maxPolys = 0;
    }

    // Write header
    WriteInt32LE(fp, header.magic);
    WriteInt32LE(fp, header.version);
    WriteInt32LE(fp, header.numTiles);
    WriteFloatLE(fp, header.origX);
    WriteFloatLE(fp, header.origY);
    WriteFloatLE(fp, header.origZ);
    WriteFloatLE(fp, header.tileWidth);
    WriteFloatLE(fp, header.tileHeight);
    WriteInt32LE(fp, header.maxTiles);
    WriteInt32LE(fp, header.maxPolys);

    // Write tiles
    bool cCompatibility = true;
    bool is64Bit = (sizeof(dtTileRef) == 8);

    for (int i = 0; i < mesh->getMaxTiles(); ++i)
    {
        const dtMeshTile* tile = mesh->getTile(i);
        if (!tile || !tile->header || !tile->dataSize) continue;

        NavMeshTileHeader tileHeader;
        tileHeader.tileRef = mesh->getTileRef(tile);

        // 记录 tile header 的起始位置
        long tileHeaderStart = std::ftell(fp);

        // 写入 tileRef
        if (is64Bit)
        {
            uint64_t tileRef = (uint64_t)tileHeader.tileRef;
            uint8_t bytes[8];
            bytes[0] = (uint8_t)(tileRef & 0xFF);
            bytes[1] = (uint8_t)((tileRef >> 8) & 0xFF);
            bytes[2] = (uint8_t)((tileRef >> 16) & 0xFF);
            bytes[3] = (uint8_t)((tileRef >> 24) & 0xFF);
            bytes[4] = (uint8_t)((tileRef >> 32) & 0xFF);
            bytes[5] = (uint8_t)((tileRef >> 40) & 0xFF);
            bytes[6] = (uint8_t)((tileRef >> 48) & 0xFF);
            bytes[7] = (uint8_t)((tileRef >> 56) & 0xFF);
            std::fwrite(bytes, sizeof(uint8_t), 8, fp);
        }
        else
        {
            WriteInt32LE(fp, (int32_t)tileHeader.tileRef);
        }

        // 先写入 dataSize 占位符（稍后会更新）
        long dataSizePos = std::ftell(fp);
        WriteInt32LE(fp, 0); // 占位符

        // 写入 padding（如果需要）
        if (cCompatibility && is64Bit)
        {
            WriteInt32LE(fp, 0);
        }

        // 记录 tile data 的起始位置
        long tileDataStart = std::ftell(fp);

        // 写入 tile data
        size_t dataSize = WriteTileData(fp, tile, mesh, cCompatibility);

        // 回写 dataSize
        long currentPos = std::ftell(fp);
        std::fseek(fp, dataSizePos, SEEK_SET);
        WriteInt32LE(fp, (int32_t)dataSize);
        std::fseek(fp, currentPos, SEEK_SET);
    }

    std::fclose(fp);
}