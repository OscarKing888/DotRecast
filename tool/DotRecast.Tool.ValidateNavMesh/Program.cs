using System;
using System.IO;
using DotRecast.Detour;
using DotRecast.Detour.Io;
using DotRecast.Core.Numerics;
using DotRecast.Core;

namespace DotRecast.Tool.ValidateNavMesh;

public static class Program
{
    private static StreamWriter? _logWriter;
    
    private static void Log(string message)
    {
        Console.WriteLine(message);
        _logWriter?.WriteLine(message);
        _logWriter?.Flush();
    }
    
    private static void Log()
    {
        Console.WriteLine();
        _logWriter?.WriteLine();
        _logWriter?.Flush();
    }
    
    public static int Main(string[] args)
    {
        // 初始化日志文件
        try
        {
            _logWriter = new StreamWriter("Verify.log", false, System.Text.Encoding.UTF8);
            _logWriter.AutoFlush = true;
        }
        catch (Exception ex)
        {
            Log($"警告: 无法创建日志文件 Verify.log: {ex.Message}");
        }
        
        try
        {
            if (args == null || args.Length == 0)
            {
                Log("用法: DotRecast.Tool.ValidateNavMesh <navmesh文件路径>");
                Log("示例: DotRecast.Tool.ValidateNavMesh path/to/navmesh.bin");
                return 1;
            }

            string filePath = args[0];
            
            if (string.IsNullOrWhiteSpace(filePath))
            {
                Log("错误: 文件路径不能为空");
                return 1;
            }

            if (!File.Exists(filePath))
            {
                Log($"错误: 文件不存在 - {filePath}");
                return 1;
            }

            try
            {
                Log($"正在验证导航网格文件: {filePath}");
                Log("==========================================");
                
                bool isValid = ValidateNavMesh(filePath);
                
                Log("==========================================");
                if (isValid)
                {
                    Log("✓ 验证成功: 导航网格文件格式正确");
                    return 0;
                }
                else
                {
                    Log("✗ 验证失败: 导航网格文件存在问题");
                    return 1;
                }
            }
            catch (Exception ex)
            {
                Log($"错误: 验证过程中发生异常");
                Log($"异常类型: {ex.GetType().Name}");
                Log($"异常消息: {ex.Message}");
                if (ex.InnerException != null)
                {
                    Log($"内部异常: {ex.InnerException.Message}");
                }
                Log($"堆栈跟踪: {ex.StackTrace}");
                return 1;
            }
        }
        finally
        {
            _logWriter?.Dispose();
            _logWriter = null;
        }
    }

    private static bool ValidateNavMesh(string filePath)
    {
        try
        {
            using var fs = new FileStream(filePath, FileMode.Open, FileAccess.Read, FileShare.Read);
            
            // 读取文件基本信息
            FileInfo fileInfo = new FileInfo(filePath);
            Log($"文件大小: {fileInfo.Length} 字节 ({fileInfo.Length / 1024.0:F2} KB)");
            Log();
            
            // 先读取文件集头部信息（magic、version和numTiles）
            int expectedNumTiles = 0;
            using (var tempBr = new BinaryReader(fs, System.Text.Encoding.UTF8, true))
            {
                Log($"[日志] 开始读取文件集头部信息，文件位置: {fs.Position} 字节");
                RcByteBuffer bb = RcIO.ToByteBuffer(tempBr);
                Log($"[日志] 读取 Magic，文件位置: {fs.Position} 字节");
                int setMagic = bb.GetInt();
                Log($"[日志] 读取 Version，文件位置: {fs.Position} 字节");
                int setVersion = bb.GetInt();
                
                // 验证magic
                bool isValidMagic = (setMagic == NavMeshSetHeader.NAVMESHSET_MAGIC);
                if (!isValidMagic)
                {
                    setMagic = RcIO.SwapEndianness(setMagic);
                    isValidMagic = (setMagic == NavMeshSetHeader.NAVMESHSET_MAGIC);
                }
                
                // 读取numTiles
                Log($"[日志] 读取 numTiles，文件位置: {fs.Position} 字节");
                expectedNumTiles = bb.GetInt();
                Log($"[日志] 读取到的 numTiles: {expectedNumTiles}");
                
                // 输出文件集头部信息
                Log("文件集头部信息:");
                Log($"  Magic: 0x{setMagic:X8} ({GetMagicString(setMagic)})");
                Log($"    期望: 0x{NavMeshSetHeader.NAVMESHSET_MAGIC:X8} ({GetMagicString(NavMeshSetHeader.NAVMESHSET_MAGIC)})");
                Log($"    状态: {(isValidMagic ? "✓ 有效" : "✗ 无效")}");
                Log($"  Version: {setVersion} (0x{setVersion:X})");
                string versionName = GetVersionName(setVersion);
                Log($"    期望: {NavMeshSetHeader.NAVMESHSET_VERSION} (标准) 或 {NavMeshSetHeader.NAVMESHSET_VERSION_RECAST4J_1} (Recast4J v1) 或 {NavMeshSetHeader.NAVMESHSET_VERSION_RECAST4J} (Recast4J)");
                Log($"    类型: {versionName}");
                Log($"  声明瓦片数: {expectedNumTiles}");
                Log($"    说明: 文件头部声明的瓦片数量（实际加载的瓦片数可能不同）");
                Log();
            }
            
            // 重置流位置，重新读取完整导航网格
            fs.Position = 0;
            using var br = new BinaryReader(fs);
            
            // 尝试读取导航网格
            Log($"[日志] 开始读取完整导航网格数据，文件位置: {fs.Position} 字节");
            DtMeshSetReader reader = new DtMeshSetReader();
            DtNavMesh navMesh = reader.Read(br, DtDetour.DT_VERTS_PER_POLYGON);
            Log($"[日志] 导航网格读取完成，文件位置: {fs.Position} 字节");
            
            if (navMesh == null)
            {
                Log("错误: 无法读取导航网格，返回null");
                return false;
            }
            
            // 验证导航网格参数
            var @params = navMesh.GetParams();
            Log("导航网格参数:");
            Log($"  原点: ({@params.orig.X:F3}, {@params.orig.Y:F3}, {@params.orig.Z:F3})");
            Log($"  瓦片尺寸: {@params.tileWidth:F3} x {@params.tileHeight:F3}");
            Log($"  最大瓦片数: {@params.maxTiles}");
            Log($"  每瓦片最大多边形数: {@params.maxPolys}");
            Log($"  每多边形最大顶点数: {navMesh.GetMaxVertsPerPoly()}");
            Log();
            
            // 统计瓦片信息
            int maxTiles = navMesh.GetMaxTiles();
            int tileCount = 0;
            
            // 统计每个瓦片的信息
            int totalPolys = 0;
            int totalVerts = 0;
            int totalDetailMeshes = 0;
            int totalDetailVerts = 0;
            int totalDetailTris = 0;
            int totalOffMeshCons = 0;
            int totalBvNodes = 0;
            
            RcVec3f boundsMin = new RcVec3f(float.MaxValue, float.MaxValue, float.MaxValue);
            RcVec3f boundsMax = new RcVec3f(float.MinValue, float.MinValue, float.MinValue);
            
            // 遍历所有瓦片，统计实际有数据的瓦片
            Log($"[日志] 开始遍历瓦片，最大瓦片数: {maxTiles}");
            for (int i = 0; i < maxTiles; i++)
            {
                Log($"[日志] 处理瓦片索引 {i}/{maxTiles - 1}");
                DtMeshTile tile = navMesh.GetTile(i);
                if (tile == null || tile.data == null)
                {
                    Log($"[日志] 瓦片索引 {i} 为空或没有数据，跳过");
                    continue; // 跳过没有数据的瓦片
                }
                
                Log($"[日志] 瓦片索引 {i} 有数据，开始验证");
                DtMeshData data = tile.data;
                DtMeshHeader header = data.header;
                
                if (header == null)
                {
                    Log($"错误: 瓦片 {i} 头信息为null");
                    Log($"  位置: 瓦片索引 {i}");
                    Log($"  期望: 有效的 DtMeshHeader 对象");
                    Log($"  实际: null");
                    return false;
                }
                
                tileCount++; // 统计实际有数据的瓦片
                Log($"[日志] 瓦片索引 {i}: 坐标=({header.x}, {header.y}, layer={header.layer}), userId={header.userId}, polyCount={header.polyCount}, vertCount={header.vertCount}");
                
                // 输出瓦片的magic和version信息（仅第一个瓦片详细输出，其他瓦片简要输出）
                if (tileCount == 1)
                {
                    Log("瓦片头部信息 (示例 - 第一个瓦片):");
                    Log($"  Magic: 0x{header.magic:X8} ({GetTileMagicString(header.magic)})");
                    Log($"    期望: 0x{DtDetour.DT_NAVMESH_MAGIC:X8} ({GetTileMagicString(DtDetour.DT_NAVMESH_MAGIC)})");
                    Log($"    状态: {(header.magic == DtDetour.DT_NAVMESH_MAGIC ? "✓ 有效" : "✗ 无效")}");
                    Log($"  Version: {header.version} (0x{header.version:X})");
                    string tileVersionName = GetTileVersionName(header.version);
                    Log($"    期望: {DtDetour.DT_NAVMESH_VERSION} (标准) 或 {DtDetour.DT_NAVMESH_VERSION_RECAST4J_FIRST}-{DtDetour.DT_NAVMESH_VERSION_RECAST4J_LAST} (Recast4J范围)");
                    Log($"    类型: {tileVersionName}");
                    Log($"  瓦片坐标: ({header.x}, {header.y}, layer={header.layer})");
                    Log($"  用户ID: {header.userId}");
                    Log();
                }
                
                // 验证头信息中的基本数据
                Log($"[日志] 瓦片索引 {i}: 验证头信息基本数据 (polyCount={header.polyCount}, vertCount={header.vertCount})");
                if (header.polyCount < 0)
                {
                    Log($"错误: 瓦片 {i} 多边形数量无效");
                    Log($"  位置: 瓦片索引 {i}, header.polyCount");
                    Log($"  期望: >= 0");
                    Log($"  实际: {header.polyCount}");
                    return false;
                }
                
                if (header.vertCount < 0)
                {
                    Log($"错误: 瓦片 {i} 顶点数量无效");
                    Log($"  位置: 瓦片索引 {i}, header.vertCount");
                    Log($"  期望: >= 0");
                    Log($"  实际: {header.vertCount}");
                    return false;
                }
                
                totalPolys += header.polyCount;
                totalVerts += header.vertCount;
                totalDetailMeshes += header.detailMeshCount;
                totalDetailVerts += header.detailVertCount;
                totalDetailTris += header.detailTriCount;
                totalOffMeshCons += header.offMeshConCount;
                totalBvNodes += header.bvNodeCount;
                
                // 更新边界
                if (header.bmin.X < boundsMin.X) boundsMin.X = header.bmin.X;
                if (header.bmin.Y < boundsMin.Y) boundsMin.Y = header.bmin.Y;
                if (header.bmin.Z < boundsMin.Z) boundsMin.Z = header.bmin.Z;
                
                if (header.bmax.X > boundsMax.X) boundsMax.X = header.bmax.X;
                if (header.bmax.Y > boundsMax.Y) boundsMax.Y = header.bmax.Y;
                if (header.bmax.Z > boundsMax.Z) boundsMax.Z = header.bmax.Z;
                
                // 验证边界有效性
                if (header.bmin.X > header.bmax.X || header.bmin.Y > header.bmax.Y || header.bmin.Z > header.bmax.Z)
                {
                    Log($"错误: 瓦片 {i} 边界信息无效");
                    Log($"  位置: 瓦片索引 {i}, header.bmin/bmax");
                    Log($"  期望: bmin <= bmax (所有分量)");
                    Log($"  实际: bmin=({header.bmin.X:F3}, {header.bmin.Y:F3}, {header.bmin.Z:F3}), bmax=({header.bmax.X:F3}, {header.bmax.Y:F3}, {header.bmax.Z:F3})");
                    return false;
                }
                
                // 验证瓦片数据完整性
                Log($"[日志] 瓦片索引 {i}: 验证瓦片数据完整性");
                if (data.polys == null)
                {
                    Log($"错误: 瓦片 {i} 多边形数组为null");
                    Log($"  位置: 瓦片索引 {i}, data.polys");
                    Log($"  期望: 非null数组，长度 = {header.polyCount}");
                    Log($"  实际: null");
                    return false;
                }
                
                if (data.polys.Length != header.polyCount)
                {
                    Log($"错误: 瓦片 {i} 多边形数组长度不匹配");
                    Log($"  位置: 瓦片索引 {i}, data.polys.Length");
                    Log($"  期望: {header.polyCount}");
                    Log($"  实际: {data.polys.Length}");
                    Log($"  差异: {data.polys.Length - header.polyCount}");
                    return false;
                }
                
                if (data.verts == null)
                {
                    Log($"错误: 瓦片 {i} 顶点数组为null");
                    Log($"  位置: 瓦片索引 {i}, data.verts");
                    Log($"  期望: 非null数组，长度 = {header.vertCount * 3}");
                    Log($"  实际: null");
                    return false;
                }
                
                int expectedVertArrayLength = header.vertCount * 3;
                if (data.verts.Length != expectedVertArrayLength)
                {
                    Log($"错误: 瓦片 {i} 顶点数组长度不匹配");
                    Log($"  位置: 瓦片索引 {i}, data.verts.Length");
                    Log($"  期望: {expectedVertArrayLength} (header.vertCount={header.vertCount} * 3)");
                    Log($"  实际: {data.verts.Length}");
                    Log($"  差异: {data.verts.Length - expectedVertArrayLength}");
                    return false;
                }
                
                // 验证多边形数据
                int maxVertsPerPoly = navMesh.GetMaxVertsPerPoly();
                Log($"[日志] 瓦片索引 {i}: 开始验证多边形数据，总数={header.polyCount}, 最大顶点数/多边形={maxVertsPerPoly}");
                for (int j = 0; j < header.polyCount; j++)
                {
                    if (j % 100 == 0 || j == header.polyCount - 1)
                    {
                        Log($"[日志] 瓦片索引 {i}: 验证多边形 {j}/{header.polyCount - 1}");
                    }
                    DtPoly poly = data.polys[j];
                    if (poly == null)
                    {
                        Log($"错误: 瓦片 {i} 多边形 {j} 为null");
                        Log($"  位置: 瓦片索引 {i}, 多边形索引 {j}, data.polys[{j}]");
                        Log($"  期望: 非null DtPoly 对象");
                        Log($"  实际: null");
                        return false;
                    }
                    
                    if (poly.vertCount < 3)
                    {
                        Log($"错误: 瓦片 {i} 多边形 {j} 顶点数过少");
                        Log($"  位置: 瓦片索引 {i}, 多边形索引 {j}, poly.vertCount");
                        Log($"  期望: >= 3 (多边形至少需要3个顶点)");
                        Log($"  实际: {poly.vertCount}");
                        return false;
                    }
                    
                    if (poly.vertCount > maxVertsPerPoly)
                    {
                        Log($"错误: 瓦片 {i} 多边形 {j} 顶点数超过限制");
                        Log($"  位置: 瓦片索引 {i}, 多边形索引 {j}, poly.vertCount");
                        Log($"  期望: <= {maxVertsPerPoly} (导航网格配置的最大值)");
                        Log($"  实际: {poly.vertCount}");
                        return false;
                    }
                    
                    // 验证顶点索引
                    if (poly.vertCount > 0)
                    {
                        Log($"[日志] 瓦片索引 {i}, 多边形索引 {j}: 验证顶点索引，顶点数={poly.vertCount}");
                    }
                    for (int k = 0; k < poly.vertCount; k++)
                    {
                        int vertIdx = poly.verts[k];
                        if (vertIdx < 0)
                        {
                            Log($"错误: 瓦片 {i} 多边形 {j} 顶点索引 {k} 为负数");
                            Log($"  位置: 瓦片索引 {i}, 多边形索引 {j}, 顶点索引 {k}, poly.verts[{k}]");
                            Log($"  期望: >= 0");
                            Log($"  实际: {vertIdx}");
                            return false;
                        }
                        
                        if (vertIdx >= header.vertCount)
                        {
                            Log($"错误: 瓦片 {i} 多边形 {j} 顶点索引 {k} 超出范围");
                            Log($"  位置: 瓦片索引 {i}, 多边形索引 {j}, 顶点索引 {k}, poly.verts[{k}]");
                            Log($"  期望: < {header.vertCount} (瓦片顶点总数)");
                            Log($"  实际: {vertIdx}");
                            return false;
                        }
                    }
                }
                
                // 验证细节网格数据
                Log($"[日志] 瓦片索引 {i}: 验证细节网格数据，detailMeshCount={header.detailMeshCount}");
                if (header.detailMeshCount > 0)
                {
                    if (data.detailMeshes == null)
                    {
                        Log($"错误: 瓦片 {i} 细节网格数组为null但头信息中声明有细节网格");
                        Log($"  位置: 瓦片索引 {i}, data.detailMeshes");
                        Log($"  期望: 非null数组，长度 = {header.detailMeshCount}");
                        Log($"  实际: null (header.detailMeshCount = {header.detailMeshCount})");
                        return false;
                    }
                    
                    if (data.detailMeshes.Length != header.detailMeshCount)
                    {
                        Log($"错误: 瓦片 {i} 细节网格数组长度不匹配");
                        Log($"  位置: 瓦片索引 {i}, data.detailMeshes.Length");
                        Log($"  期望: {header.detailMeshCount}");
                        Log($"  实际: {data.detailMeshes.Length}");
                        Log($"  差异: {data.detailMeshes.Length - header.detailMeshCount}");
                        return false;
                    }
                    
                    // 验证每个细节网格的有效性
                    Log($"[日志] 瓦片索引 {i}: 验证 {header.detailMeshCount} 个细节网格元素");
                    for (int dmIdx = 0; dmIdx < header.detailMeshCount; dmIdx++)
                    {
                        var dm = data.detailMeshes[dmIdx];
                        if (dmIdx == 0 || dmIdx == header.detailMeshCount - 1 || dmIdx % 100 == 0)
                        {
                            Log($"[日志] 瓦片索引 {i}: 细节网格 {dmIdx}/{header.detailMeshCount - 1}: vertBase={dm.vertBase}, triBase={dm.triBase}, vertCount={dm.vertCount}, triCount={dm.triCount}");
                        }
                    }
                }
                
                // 验证细节顶点数据
                Log($"[日志] 瓦片索引 {i}: 验证细节顶点数据，detailVertCount={header.detailVertCount}");
                if (header.detailVertCount > 0)
                {
                    if (data.detailVerts == null)
                    {
                        Log($"错误: 瓦片 {i} 细节顶点数组为null但头信息中声明有细节顶点");
                        Log($"  位置: 瓦片索引 {i}, data.detailVerts");
                        Log($"  期望: 非null数组，长度 = {header.detailVertCount * 3}");
                        Log($"  实际: null (header.detailVertCount = {header.detailVertCount})");
                        return false;
                    }
                    
                    int expectedDetailVertArrayLength = header.detailVertCount * 3;
                    if (data.detailVerts.Length != expectedDetailVertArrayLength)
                    {
                        Log($"错误: 瓦片 {i} 细节顶点数组长度不匹配");
                        Log($"  位置: 瓦片索引 {i}, data.detailVerts.Length");
                        Log($"  期望: {expectedDetailVertArrayLength} (header.detailVertCount={header.detailVertCount} * 3)");
                        Log($"  实际: {data.detailVerts.Length}");
                        Log($"  差异: {data.detailVerts.Length - expectedDetailVertArrayLength}");
                        return false;
                    }
                }
                
                // 验证细节三角形数据
                Log($"[日志] 瓦片索引 {i}: 验证细节三角形数据，detailTriCount={header.detailTriCount}");
                if (header.detailTriCount > 0)
                {
                    if (data.detailTris == null)
                    {
                        Log($"错误: 瓦片 {i} 细节三角形数组为null但头信息中声明有细节三角形");
                        Log($"  位置: 瓦片索引 {i}, data.detailTris");
                        Log($"  期望: 非null数组，长度 = {header.detailTriCount * 4}");
                        Log($"  实际: null (header.detailTriCount = {header.detailTriCount})");
                        return false;
                    }
                    
                    int expectedDetailTriArrayLength = header.detailTriCount * 4;
                    if (data.detailTris.Length != expectedDetailTriArrayLength)
                    {
                        Log($"错误: 瓦片 {i} 细节三角形数组长度不匹配");
                        Log($"  位置: 瓦片索引 {i}, data.detailTris.Length");
                        Log($"  期望: {expectedDetailTriArrayLength} (header.detailTriCount={header.detailTriCount} * 4)");
                        Log($"  实际: {data.detailTris.Length}");
                        Log($"  差异: {data.detailTris.Length - expectedDetailTriArrayLength}");
                        return false;
                    }
                }
                
                // 验证离网连接数据
                Log($"[日志] 瓦片索引 {i}: 验证离网连接数据，offMeshConCount={header.offMeshConCount}");
                if (header.offMeshConCount > 0)
                {
                    if (data.offMeshCons == null)
                    {
                        Log($"错误: 瓦片 {i} 离网连接数组为null但头信息中声明有离网连接");
                        Log($"  位置: 瓦片索引 {i}, data.offMeshCons");
                        Log($"  期望: 非null数组，长度 = {header.offMeshConCount}");
                        Log($"  实际: null (header.offMeshConCount = {header.offMeshConCount})");
                        return false;
                    }
                    
                    if (data.offMeshCons.Length != header.offMeshConCount)
                    {
                        Log($"错误: 瓦片 {i} 离网连接数组长度不匹配");
                        Log($"  位置: 瓦片索引 {i}, data.offMeshCons.Length");
                        Log($"  期望: {header.offMeshConCount}");
                        Log($"  实际: {data.offMeshCons.Length}");
                        Log($"  差异: {data.offMeshCons.Length - header.offMeshConCount}");
                        return false;
                    }
                    
                    // 验证每个离网连接的有效性
                    Log($"[日志] 瓦片索引 {i}: 验证 {header.offMeshConCount} 个离网连接元素");
                    for (int omcIdx = 0; omcIdx < header.offMeshConCount; omcIdx++)
                    {
                        var omc = data.offMeshCons[omcIdx];
                        if (omcIdx == 0 || omcIdx == header.offMeshConCount - 1 || omcIdx % 10 == 0)
                        {
                            Log($"[日志] 瓦片索引 {i}: 离网连接 {omcIdx}/{header.offMeshConCount - 1}: poly={omc.poly}, userId={omc.userId}, rad={omc.rad:F3}");
                        }
                    }
                }
                
                // 验证包围盒树数据
                Log($"[日志] 瓦片索引 {i}: 验证包围盒树数据，bvNodeCount={header.bvNodeCount}");
                if (header.bvNodeCount > 0)
                {
                    if (data.bvTree == null)
                    {
                        Log($"错误: 瓦片 {i} 包围盒树数组为null但头信息中声明有包围盒节点");
                        Log($"  位置: 瓦片索引 {i}, data.bvTree");
                        Log($"  期望: 非null数组，长度 = {header.bvNodeCount}");
                        Log($"  实际: null (header.bvNodeCount = {header.bvNodeCount})");
                        return false;
                    }
                    
                    if (data.bvTree.Length != header.bvNodeCount)
                    {
                        Log($"错误: 瓦片 {i} 包围盒树数组长度不匹配");
                        Log($"  位置: 瓦片索引 {i}, data.bvTree.Length");
                        Log($"  期望: {header.bvNodeCount}");
                        Log($"  实际: {data.bvTree.Length}");
                        Log($"  差异: {data.bvTree.Length - header.bvNodeCount}");
                        return false;
                    }
                    
                    // 验证每个包围盒节点的有效性
                    Log($"[日志] 瓦片索引 {i}: 验证 {header.bvNodeCount} 个包围盒节点元素");
                    for (int bvIdx = 0; bvIdx < header.bvNodeCount; bvIdx++)
                    {
                        var bvNode = data.bvTree[bvIdx];
                        if (bvIdx == 0 || bvIdx == header.bvNodeCount - 1 || bvIdx % 100 == 0)
                        {
                            Log($"[日志] 瓦片索引 {i}: 包围盒节点 {bvIdx}/{header.bvNodeCount - 1}: i={bvNode.i}, bmin=({bvNode.bmin.X}, {bvNode.bmin.Y}, {bvNode.bmin.Z}), bmax=({bvNode.bmax.X}, {bvNode.bmax.Y}, {bvNode.bmax.Z})");
                        }
                    }
                }
                
                Log($"[日志] 瓦片索引 {i}: 验证完成");
            }
            
            Log($"[日志] 所有瓦片验证完成，共处理 {tileCount} 个有效瓦片");
            Log($"瓦片统计:");
            Log($"  最大瓦片数: {maxTiles}");
            Log($"  声明瓦片数: {expectedNumTiles} (文件头部)");
            Log($"  实际瓦片数: {tileCount} (通过检查 tile.data != null 统计)");
            
            if (tileCount == 0)
            {
                Log("错误: 导航网格中没有有效瓦片");
                Log($"  位置: 导航网格整体");
                Log($"  期望: >= 1 个有效瓦片 (有数据的瓦片)");
                Log($"  实际: 0 个有效瓦片");
                Log($"  诊断信息:");
                Log($"    - 最大瓦片数: {maxTiles}");
                Log($"    - 声明瓦片数: {expectedNumTiles}");
                Log($"    - 实际加载瓦片数: {tileCount}");
                Log($"    - 说明: 实际瓦片数通过遍历 m_tiles[] 数组，检查每个 tile.data != null 来统计");
                Log($"    - 可能原因:");
                Log($"      1. 文件中的瓦片数据未正确写入");
                Log($"      2. 读取过程中 AddTile() 失败但未抛出异常");
                Log($"      3. 瓦片数据被存储在哈希表 m_posLookup 中，但 m_tiles[] 中的 tile.data 为 null");
                
                // 尝试通过其他方式查找瓦片
                int foundTiles = 0;
                for (int i = 0; i < maxTiles; i++)
                {
                    DtMeshTile tile = navMesh.GetTile(i);
                    if (tile != null)
                    {
                        if (tile.data != null)
                        {
                            foundTiles++;
                        }
                        else if (tile.salt != 1) // salt != 1 表示瓦片可能被使用过
                        {
                            Log($"    发现: 瓦片 {i} 的 salt = {tile.salt} (可能被使用过，但 data 为 null)");
                        }
                    }
                }
                
                if (foundTiles == 0)
                {
                    Log($"    - 通过 m_tiles[] 数组未找到任何有数据的瓦片");
                }
                
                return false;
            }
            
            if (expectedNumTiles > 0 && tileCount != expectedNumTiles)
            {
                Log($"警告: 实际瓦片数与声明瓦片数不匹配");
                Log($"  声明瓦片数: {expectedNumTiles}");
                Log($"  实际瓦片数: {tileCount}");
                Log($"  差异: {tileCount - expectedNumTiles}");
            }
            
            Log();
            Log("数据统计:");
            Log($"  总多边形数: {totalPolys}");
            Log($"  总顶点数: {totalVerts}");
            Log($"  总细节网格数: {totalDetailMeshes}");
            Log($"  总细节顶点数: {totalDetailVerts}");
            Log($"  总细节三角形数: {totalDetailTris}");
            Log($"  总离网连接数: {totalOffMeshCons}");
            Log($"  总包围盒节点数: {totalBvNodes}");
            Log();
            
            Log("边界信息:");
            Log($"  最小边界: ({boundsMin.X:F3}, {boundsMin.Y:F3}, {boundsMin.Z:F3})");
            Log($"  最大边界: ({boundsMax.X:F3}, {boundsMax.Y:F3}, {boundsMax.Z:F3})");
            float width = boundsMax.X - boundsMin.X;
            float height = boundsMax.Y - boundsMin.Y;
            float depth = boundsMax.Z - boundsMin.Z;
            Log($"  尺寸: {width:F3} x {height:F3} x {depth:F3}");
            Log();
            
            // 尝试创建查询器验证功能
            Log($"[日志] 开始创建导航网格查询器");
            try
            {
                DtNavMeshQuery query = new DtNavMeshQuery(navMesh);
                Log($"[日志] 导航网格查询器创建完成");
                if (query == null)
                {
                    Log("警告: 无法创建导航网格查询器");
                }
                else
                {
                    Log("✓ 导航网格查询器创建成功");
                }
            }
            catch (Exception ex)
            {
                Log($"警告: 创建导航网格查询器时发生异常: {ex.Message}");
            }
            
            return true;
        }
        catch (IOException ex)
        {
            Log($"[日志] 发生IO错误");
            Log($"IO错误: {ex.Message}");
            Log($"堆栈跟踪: {ex.StackTrace}");
            return false;
        }
        catch (Exception ex)
        {
            Log($"[日志] 发生验证错误");
            Log($"验证错误: {ex.Message}");
            Log($"异常类型: {ex.GetType().Name}");
            if (ex.InnerException != null)
            {
                Log($"内部异常: {ex.InnerException.Message}");
            }
            Log($"堆栈跟踪: {ex.StackTrace}");
            return false;
        }
    }
    
    private static string GetMagicString(int magic)
    {
        char c1 = (char)((magic >> 24) & 0xFF);
        char c2 = (char)((magic >> 16) & 0xFF);
        char c3 = (char)((magic >> 8) & 0xFF);
        char c4 = (char)(magic & 0xFF);
        return $"'{c1}{c2}{c3}{c4}'";
    }
    
    private static string GetTileMagicString(int magic)
    {
        char c1 = (char)((magic >> 24) & 0xFF);
        char c2 = (char)((magic >> 16) & 0xFF);
        char c3 = (char)((magic >> 8) & 0xFF);
        char c4 = (char)(magic & 0xFF);
        return $"'{c1}{c2}{c3}{c4}'";
    }
    
    private static string GetVersionName(int version)
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
    
    private static string GetTileVersionName(int version)
    {
        if (version == DtDetour.DT_NAVMESH_VERSION)
            return "标准版本 (C兼容)";
        else if (version >= DtDetour.DT_NAVMESH_VERSION_RECAST4J_FIRST && version <= DtDetour.DT_NAVMESH_VERSION_RECAST4J_LAST)
        {
            if (version == DtDetour.DT_NAVMESH_VERSION_RECAST4J_NO_POLY_FIRSTLINK)
                return "Recast4J (无多边形首链接)";
            else if (version == DtDetour.DT_NAVMESH_VERSION_RECAST4J_32BIT_BVTREE)
                return "Recast4J (32位包围盒树)";
            else
                return $"Recast4J 版本 (0x{version:X})";
        }
        else
            return "未知版本";
    }
}

