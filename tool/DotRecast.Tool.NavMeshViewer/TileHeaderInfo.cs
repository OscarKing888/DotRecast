using DotRecast.Detour;
using DotRecast.Detour.Io;

namespace DotRecast.Tool.NavMeshViewer
{
    /// <summary>
    /// 用于在属性查看器中显示 TileHeader 和 MeshHeader 的组合信息
    /// </summary>
    public class TileHeaderInfo
    {
        public int Index { get; set; }
        public NavMeshTileHeader TileHeader { get; set; }
        public DtMeshHeader MeshHeader { get; set; }

        public TileHeaderInfo(int index, NavMeshTileHeader tileHeader, DtMeshHeader meshHeader)
        {
            Index = index;
            TileHeader = tileHeader;
            MeshHeader = meshHeader;
        }

        // 为了在属性查看器中更好地显示，添加一些计算属性
        [System.ComponentModel.DisplayName("Tile Reference (Hex)")]
        [System.ComponentModel.Description("瓦片引用标识符（十六进制）")]
        public string TileRefHex => $"0x{TileHeader.tileRef:X}";

        [System.ComponentModel.DisplayName("Data Size (Bytes)")]
        [System.ComponentModel.Description("瓦片数据大小（字节）")]
        public int DataSize => TileHeader.dataSize;

        [System.ComponentModel.DisplayName("Tile Position")]
        [System.ComponentModel.Description("瓦片在网格中的位置")]
        public string TilePosition => $"({MeshHeader.x}, {MeshHeader.y}, Layer: {MeshHeader.layer})";

        [System.ComponentModel.DisplayName("Polygon Count")]
        [System.ComponentModel.Description("多边形数量")]
        public int PolygonCount => MeshHeader.polyCount;

        [System.ComponentModel.DisplayName("Vertex Count")]
        [System.ComponentModel.Description("顶点数量")]
        public int VertexCount => MeshHeader.vertCount;

        [System.ComponentModel.DisplayName("Tile Reference (Int)")]
        [System.ComponentModel.Description("瓦片引用标识符（整数）")]
        public long TileRef => TileHeader.tileRef;

        [System.ComponentModel.DisplayName("Magic (Hex)")]
        [System.ComponentModel.Description("瓦片标识符（十六进制）")]
        public string MagicHex => $"0x{MeshHeader.magic:X8}";

        [System.ComponentModel.DisplayName("Version")]
        [System.ComponentModel.Description("瓦片版本号")]
        public int Version => MeshHeader.version;

        [System.ComponentModel.DisplayName("User ID")]
        [System.ComponentModel.Description("用户定义的ID")]
        public int UserId => MeshHeader.userId;

        [System.ComponentModel.DisplayName("Max Link Count")]
        [System.ComponentModel.Description("最大链接数")]
        public int MaxLinkCount => MeshHeader.maxLinkCount;

        [System.ComponentModel.DisplayName("Detail Mesh Count")]
        [System.ComponentModel.Description("细节网格数量")]
        public int DetailMeshCount => MeshHeader.detailMeshCount;

        [System.ComponentModel.DisplayName("Detail Vertex Count")]
        [System.ComponentModel.Description("细节顶点数量")]
        public int DetailVertCount => MeshHeader.detailVertCount;

        [System.ComponentModel.DisplayName("Detail Triangle Count")]
        [System.ComponentModel.Description("细节三角形数量")]
        public int DetailTriCount => MeshHeader.detailTriCount;

        [System.ComponentModel.DisplayName("BV Node Count")]
        [System.ComponentModel.Description("包围盒节点数量")]
        public int BvNodeCount => MeshHeader.bvNodeCount;

        [System.ComponentModel.DisplayName("Off-Mesh Connection Count")]
        [System.ComponentModel.Description("离网连接数量")]
        public int OffMeshConCount => MeshHeader.offMeshConCount;

        [System.ComponentModel.DisplayName("Bounds Min")]
        [System.ComponentModel.Description("最小边界")]
        public string BoundsMin => $"({MeshHeader.bmin.X:F3}, {MeshHeader.bmin.Y:F3}, {MeshHeader.bmin.Z:F3})";

        [System.ComponentModel.DisplayName("Bounds Max")]
        [System.ComponentModel.Description("最大边界")]
        public string BoundsMax => $"({MeshHeader.bmax.X:F3}, {MeshHeader.bmax.Y:F3}, {MeshHeader.bmax.Z:F3})";

        [System.ComponentModel.DisplayName("Walkable Height")]
        [System.ComponentModel.Description("可行走高度")]
        public float WalkableHeight => MeshHeader.walkableHeight;

        [System.ComponentModel.DisplayName("Walkable Radius")]
        [System.ComponentModel.Description("可行走半径")]
        public float WalkableRadius => MeshHeader.walkableRadius;

        [System.ComponentModel.DisplayName("Walkable Climb")]
        [System.ComponentModel.Description("可行走攀爬高度")]
        public float WalkableClimb => MeshHeader.walkableClimb;

        [System.ComponentModel.DisplayName("Mesh Header")]
        [System.ComponentModel.Description("完整的网格头信息")]
        [System.ComponentModel.TypeConverter(typeof(System.ComponentModel.ExpandableObjectConverter))]
        public DtMeshHeader MeshHeaderFull => MeshHeader;

        [System.ComponentModel.DisplayName("Tile Header")]
        [System.ComponentModel.Description("完整的瓦片头信息")]
        [System.ComponentModel.TypeConverter(typeof(System.ComponentModel.ExpandableObjectConverter))]
        public NavMeshTileHeader TileHeaderFull => TileHeader;
    }
}

