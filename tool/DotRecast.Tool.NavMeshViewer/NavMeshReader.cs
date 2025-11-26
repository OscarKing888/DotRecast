using System;
using System.Collections.Generic;
using System.IO;
using DotRecast.Core;
using DotRecast.Detour;
using DotRecast.Detour.Io;

namespace DotRecast.Tool.NavMeshViewer
{
    public class NavMeshInfo
    {
        public NavMeshSetHeader SetHeader { get; set; }
        public List<NavMeshTileHeader> TileHeaders { get; set; } = new List<NavMeshTileHeader>();
        public List<DtMeshHeader> TileMeshHeaders { get; set; } = new List<DtMeshHeader>();
    }

    public static class NavMeshReader
    {
        public static NavMeshInfo ReadNavMeshInfo(string filePath)
        {
            NavMeshInfo info = new NavMeshInfo();

            using (var fs = new FileStream(filePath, FileMode.Open, FileAccess.Read, FileShare.Read))
            using (var br = new BinaryReader(fs))
            {
                RcByteBuffer bb = RcIO.ToByteBuffer(br);

                // 读取 SetHeader
                info.SetHeader = ReadSetHeader(bb);

                // 读取所有 TileHeaders
                bool is32Bit = false; // 默认假设是64位，如果需要可以检测
                bool cCompatibility = info.SetHeader.version == NavMeshSetHeader.NAVMESHSET_VERSION;

                for (int i = 0; i < info.SetHeader.numTiles; i++)
                {
                    NavMeshTileHeader tileHeader = new NavMeshTileHeader();
                    
                    // 读取 tileRef
                    if (is32Bit)
                    {
                        int ref32 = bb.GetInt();
                        tileHeader.tileRef = Convert32BitRef(ref32, info.SetHeader.option);
                    }
                    else
                    {
                        tileHeader.tileRef = bb.GetLong();
                    }

                    // 读取 dataSize
                    tileHeader.dataSize = bb.GetInt();

                    if (tileHeader.tileRef == 0 || tileHeader.dataSize == 0)
                    {
                        break;
                    }

                    // C兼容性填充
                    if (cCompatibility && !is32Bit)
                    {
                        bb.GetInt(); // C struct padding
                    }

                    info.TileHeaders.Add(tileHeader);

                    // 读取 Tile 的 MeshHeader（但不读取完整数据）
                    int positionBeforeTile = bb.Position();
                    DtMeshHeader meshHeader = ReadMeshHeader(bb, is32Bit);
                    info.TileMeshHeaders.Add(meshHeader);

                    // 跳过剩余的 tile 数据
                    int positionAfterHeader = bb.Position();
                    int dataSizeToSkip = tileHeader.dataSize - (positionAfterHeader - positionBeforeTile);
                    if (dataSizeToSkip > 0)
                    {
                        bb.Position(positionAfterHeader + dataSizeToSkip);
                    }
                }
            }

            return info;
        }

        private static NavMeshSetHeader ReadSetHeader(RcByteBuffer bb)
        {
            NavMeshSetHeader header = new NavMeshSetHeader();
            header.magic = bb.GetInt();
            
            if (header.magic != NavMeshSetHeader.NAVMESHSET_MAGIC)
            {
                header.magic = RcIO.SwapEndianness(header.magic);
                if (header.magic != NavMeshSetHeader.NAVMESHSET_MAGIC)
                {
                    throw new IOException("Invalid magic " + header.magic);
                }
                bb.Order(bb.Order() == RcByteOrder.BIG_ENDIAN ? RcByteOrder.LITTLE_ENDIAN : RcByteOrder.BIG_ENDIAN);
            }

            header.version = bb.GetInt();
            if (header.version != NavMeshSetHeader.NAVMESHSET_VERSION 
                && header.version != NavMeshSetHeader.NAVMESHSET_VERSION_RECAST4J_1
                && header.version != NavMeshSetHeader.NAVMESHSET_VERSION_RECAST4J)
            {
                throw new IOException("Invalid version " + header.version);
            }

            header.numTiles = bb.GetInt();
            
            // 读取 DtNavMeshParams
            DtNavMeshParamsReader paramReader = new DtNavMeshParamsReader();
            header.option = paramReader.Read(bb);
            
            header.maxVertsPerPoly = -1;
            if (header.version == NavMeshSetHeader.NAVMESHSET_VERSION_RECAST4J)
            {
                header.maxVertsPerPoly = bb.GetInt();
            }

            return header;
        }

        private static DtMeshHeader ReadMeshHeader(RcByteBuffer bb, bool is32Bit)
        {
            DtMeshHeader header = new DtMeshHeader();
            
            header.magic = bb.GetInt();
            if (header.magic != DtDetour.DT_NAVMESH_MAGIC)
            {
                header.magic = RcIO.SwapEndianness(header.magic);
                if (header.magic != DtDetour.DT_NAVMESH_MAGIC)
                {
                    throw new IOException("Invalid tile magic");
                }
                bb.Order(bb.Order() == RcByteOrder.BIG_ENDIAN ? RcByteOrder.LITTLE_ENDIAN : RcByteOrder.BIG_ENDIAN);
            }

            header.version = bb.GetInt();
            if (header.version != DtDetour.DT_NAVMESH_VERSION)
            {
                if (header.version < DtDetour.DT_NAVMESH_VERSION_RECAST4J_FIRST
                    || header.version > DtDetour.DT_NAVMESH_VERSION_RECAST4J_LAST)
                {
                    throw new IOException("Invalid tile version " + header.version);
                }
            }

            bool cCompatibility = header.version == DtDetour.DT_NAVMESH_VERSION;
            
            header.x = bb.GetInt();
            header.y = bb.GetInt();
            header.layer = bb.GetInt();
            header.userId = bb.GetInt();
            header.polyCount = bb.GetInt();
            header.vertCount = bb.GetInt();
            header.maxLinkCount = bb.GetInt();
            header.detailMeshCount = bb.GetInt();
            header.detailVertCount = bb.GetInt();
            header.detailTriCount = bb.GetInt();
            header.bvNodeCount = bb.GetInt();
            header.offMeshConCount = bb.GetInt();
            header.offMeshBase = bb.GetInt();
            header.walkableHeight = bb.GetFloat();
            header.walkableRadius = bb.GetFloat();
            header.walkableClimb = bb.GetFloat();
            header.bmin.X = bb.GetFloat();
            header.bmin.Y = bb.GetFloat();
            header.bmin.Z = bb.GetFloat();
            header.bmax.X = bb.GetFloat();
            header.bmax.Y = bb.GetFloat();
            header.bmax.Z = bb.GetFloat();
            header.bvQuantFactor = bb.GetFloat();

            return header;
        }

        private static long Convert32BitRef(int refs, DtNavMeshParams option)
        {
            int m_tileBits = DtUtils.Ilog2(DtUtils.NextPow2(option.maxTiles));
            int m_polyBits = DtUtils.Ilog2(DtUtils.NextPow2(option.maxPolys));
            int m_saltBits = Math.Min(31, 32 - m_tileBits - m_polyBits);
            int saltMask = (1 << m_saltBits) - 1;
            int tileMask = (1 << m_tileBits) - 1;
            int polyMask = (1 << m_polyBits) - 1;
            int salt = ((refs >> (m_polyBits + m_tileBits)) & saltMask);
            int it = ((refs >> m_polyBits) & tileMask);
            int ip = refs & polyMask;
            return DtDetour.EncodePolyId(salt, it, ip);
        }
    }
}

