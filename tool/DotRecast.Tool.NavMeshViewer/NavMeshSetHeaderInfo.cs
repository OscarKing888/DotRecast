using DotRecast.Detour;
using DotRecast.Detour.Io;

namespace DotRecast.Tool.NavMeshViewer
{
    /// <summary>
    /// 用于在属性查看器中显示 NavMeshSetHeader 信息
    /// </summary>
    public class NavMeshSetHeaderInfo
    {
        private readonly NavMeshSetHeader _header;

        public NavMeshSetHeaderInfo(NavMeshSetHeader header)
        {
            _header = header;
        }

        [System.ComponentModel.DisplayName("Magic (Hex)")]
        [System.ComponentModel.Description("文件标识符（十六进制）")]
        public string MagicHex => $"0x{_header.magic:X8} ({GetMagicString(_header.magic)})";

        [System.ComponentModel.DisplayName("Magic (Int)")]
        [System.ComponentModel.Description("文件标识符（整数）")]
        public int Magic => _header.magic;

        [System.ComponentModel.DisplayName("Version")]
        [System.ComponentModel.Description("文件版本号")]
        public int Version => _header.version;

        [System.ComponentModel.DisplayName("Version Name")]
        [System.ComponentModel.Description("版本名称")]
        public string VersionName => GetVersionName(_header.version);

        [System.ComponentModel.DisplayName("Number of Tiles")]
        [System.ComponentModel.Description("瓦片数量")]
        public int NumTiles => _header.numTiles;

        [System.ComponentModel.DisplayName("Max Verts Per Poly")]
        [System.ComponentModel.Description("每个多边形的最大顶点数")]
        public int MaxVertsPerPoly => _header.maxVertsPerPoly;

        [System.ComponentModel.DisplayName("NavMesh Parameters")]
        [System.ComponentModel.Description("导航网格参数")]
        [System.ComponentModel.TypeConverter(typeof(System.ComponentModel.ExpandableObjectConverter))]
        public DtNavMeshParams Option => _header.option;

        private string GetMagicString(int magic)
        {
            char c1 = (char)((magic >> 24) & 0xFF);
            char c2 = (char)((magic >> 16) & 0xFF);
            char c3 = (char)((magic >> 8) & 0xFF);
            char c4 = (char)(magic & 0xFF);
            return $"'{c1}{c2}{c3}{c4}'";
        }

        private string GetVersionName(int version)
        {
            if (version == NavMeshSetHeader.NAVMESHSET_VERSION)
                return "标准版本 (C兼容)";
            else if (version == NavMeshSetHeader.NAVMESHSET_VERSION_RECAST4J_1)
                return "Recast4J 版本 1";
            else if (version == NavMeshSetHeader.NAVMESHSET_VERSION_RECAST4J)
                return "Recast4J 版本";
            else
                return "未知版本";
        }
    }
}

