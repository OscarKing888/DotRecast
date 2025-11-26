using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Windows.Forms;
using DotRecast.Detour;
using DotRecast.Detour.Io;
using DotRecast.Core;

namespace DotRecast.Tool.NavMeshViewer
{
    public partial class MainForm : Form
    {
        private SplitContainer splitContainer;
        private TreeView treeView;
        private PropertyGrid propertyGrid;
        private NavMeshInfo currentNavMeshInfo;

        public MainForm()
        {
            InitializeComponent();
        }

        private void InitializeComponent()
        {
            this.Text = "NavMesh 信息查看器";
            this.Size = new System.Drawing.Size(1200, 800);
            this.StartPosition = FormStartPosition.CenterScreen;
            this.AllowDrop = true;

            // 创建分割容器
            splitContainer = new SplitContainer
            {
                Dock = DockStyle.Fill,
                SplitterDistance = 400,
                SplitterWidth = 5
            };

            // 创建左侧树形视图
            treeView = new TreeView
            {
                Dock = DockStyle.Fill,
                ShowLines = true,
                ShowRootLines = true,
                ShowPlusMinus = true,
                FullRowSelect = true
            };
            treeView.AfterSelect += TreeView_AfterSelect;

            // 创建右侧属性网格
            propertyGrid = new PropertyGrid
            {
                Dock = DockStyle.Fill,
                ToolbarVisible = true,
                HelpVisible = true
            };

            splitContainer.Panel1.Controls.Add(treeView);
            splitContainer.Panel2.Controls.Add(propertyGrid);

            this.Controls.Add(splitContainer);

            // 设置拖放事件
            this.DragEnter += MainForm_DragEnter;
            this.DragDrop += MainForm_DragDrop;
            treeView.DragEnter += MainForm_DragEnter;
            treeView.DragDrop += MainForm_DragDrop;
        }

        private void MainForm_DragEnter(object sender, DragEventArgs e)
        {
            if (e.Data.GetDataPresent(DataFormats.FileDrop))
            {
                e.Effect = DragDropEffects.Copy;
            }
        }

        private void MainForm_DragDrop(object sender, DragEventArgs e)
        {
            if (e.Data.GetDataPresent(DataFormats.FileDrop))
            {
                string[] files = (string[])e.Data.GetData(DataFormats.FileDrop);
                if (files.Length > 0)
                {
                    LoadNavMeshFile(files[0]);
                }
            }
        }

        private void LoadNavMeshFile(string filePath)
        {
            try
            {
                if (!File.Exists(filePath))
                {
                    MessageBox.Show($"文件不存在: {filePath}", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
                    return;
                }

                this.Text = $"NavMesh 信息查看器 - {Path.GetFileName(filePath)}";

                // 读取 NavMesh 信息
                currentNavMeshInfo = NavMeshReader.ReadNavMeshInfo(filePath);

                // 更新树形视图
                UpdateTreeView();
            }
            catch (Exception ex)
            {
                MessageBox.Show($"加载文件失败: {ex.Message}\n\n堆栈跟踪:\n{ex.StackTrace}", 
                    "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void UpdateTreeView()
        {
            treeView.Nodes.Clear();

            if (currentNavMeshInfo == null)
                return;

            // 添加 NavMeshSetHeader 节点
            TreeNode headerNode = new TreeNode("NavMeshSetHeader")
            {
                Tag = new NavMeshSetHeaderInfo(currentNavMeshInfo.SetHeader)
            };
            treeView.Nodes.Add(headerNode);

            // 添加 NavMeshTileHeader 节点
            TreeNode tilesNode = new TreeNode($"NavMeshTileHeaders ({currentNavMeshInfo.TileHeaders.Count})");
            for (int i = 0; i < currentNavMeshInfo.TileHeaders.Count; i++)
            {
                var tileHeader = currentNavMeshInfo.TileHeaders[i];
                TreeNode tileNode = new TreeNode($"Tile {i} (Ref: 0x{tileHeader.tileRef:X}, Size: {tileHeader.dataSize})")
                {
                    Tag = new TileHeaderInfo(i, tileHeader, currentNavMeshInfo.TileMeshHeaders[i])
                };
                tilesNode.Nodes.Add(tileNode);
            }
            treeView.Nodes.Add(tilesNode);

            // 展开所有节点
            treeView.ExpandAll();
        }

        private void TreeView_AfterSelect(object sender, TreeViewEventArgs e)
        {
            if (e.Node?.Tag != null)
            {
                propertyGrid.SelectedObject = e.Node.Tag;
            }
            else
            {
                propertyGrid.SelectedObject = null;
            }
        }
    }
}

