# Recast 导航网格构建流程 —— `Sample_SoloMesh::handleBuild` 深度分析

## 📖 文档目录

本系列文档对 `Sample_SoloMesh::handleBuild()` 函数的完整导航网格构建流程进行了深入分析，
涵盖从输入三角形网格到最终 Detour 运行时导航数据的 **8 个核心步骤**。

源代码位置：`RecastDemo/Source/Sample_SoloMesh.cpp` — `handleBuild()` 函数

---

### 文档列表

| 编号 | 文档 | 内容概述 |
|------|------|----------|
| 00 | [总览：构建流程全景](./00_构建流程总览.md) | 8 步构建流程全景图、数据流向、核心数据结构关系 |
| 01 | [Step 1：初始化构建配置](./01_初始化构建配置.md) | `rcConfig` 参数详解、世界单位到体素单位的转换公式 |
| 02 | [Step 2：体素化光栅化](./02_体素化光栅化.md) | 三角形光栅化到高度场、Span 链表结构、`rasterizeTri` 裁剪算法 |
| 03 | [Step 3：过滤可行走表面](./03_过滤可行走表面.md) | 三种过滤函数的原理与实现：低矮障碍物、悬崖边缘、低净空区域 |
| 04 | [Step 4：区域划分](./04_区域划分.md) | 紧凑高度场转换、腐蚀、距离场、Watershed/Monotone/Layer 三种分区算法 |
| 05 | [Step 5：轮廓提取与简化](./05_轮廓提取与简化.md) | 区域边界追踪、Douglas-Peucker 简化、孔洞合并 |
| 06 | [Step 6：构建多边形网格](./06_构建多边形网格.md) | 轮廓三角化、三角形合并为凸多边形、`rcPolyMesh` 数据结构 |
| 07 | [Step 7：构建细节网格](./07_构建细节网格.md) | 高度采样、细节三角化、`rcPolyMeshDetail` 数据结构 |
| 08 | [Step 8：创建 Detour 导航数据](./08_创建Detour导航数据.md) | 区域到标志位映射、`dtNavMeshCreateParams` 填充、BVTree、序列化 |
| 09 | [附录：核心数据结构速查](./09_核心数据结构速查.md) | rcSpan、rcHeightfield、rcCompactHeightfield、rcPolyMesh 等结构体汇总 |

---

### 构建流程速览

```
输入三角形网格 (vertices + triangles)
        │
        ▼
   ┌─────────────────────┐
   │ Step 1: 初始化配置    │  rcConfig 参数设置
   └──────────┬──────────┘
              ▼
   ┌─────────────────────┐
   │ Step 2: 体素化光栅化  │  rcRasterizeTriangles → rcHeightfield
   └──────────┬──────────┘
              ▼
   ┌─────────────────────┐
   │ Step 3: 过滤可行走表面│  rcFilterXxx (3个过滤函数)
   └──────────┬──────────┘
              ▼
   ┌─────────────────────┐
   │ Step 4: 区域划分      │  rcBuildCompactHeightfield → rcErodeWalkableArea
   │                      │  → rcBuildDistanceField → rcBuildRegions
   └──────────┬──────────┘
              ▼
   ┌─────────────────────┐
   │ Step 5: 轮廓提取简化  │  rcBuildContours → rcContourSet
   └──────────┬──────────┘
              ▼
   ┌─────────────────────┐
   │ Step 6: 多边形网格    │  rcBuildPolyMesh → rcPolyMesh
   └──────────┬──────────┘
              ▼
   ┌─────────────────────┐
   │ Step 7: 细节网格      │  rcBuildPolyMeshDetail → rcPolyMeshDetail
   └──────────┬──────────┘
              ▼
   ┌─────────────────────┐
   │ Step 8: Detour数据    │  dtCreateNavMeshData → dtNavMesh
   └─────────────────────┘
              ▼
       运行时寻路可用
```

---

### 阅读建议

1. **初学者**：建议从 [00_构建流程总览](./00_构建流程总览.md) 开始，了解全局脉络后再深入各步骤。
2. **进阶读者**：可直接跳到感兴趣的步骤文档，每个文档都是独立的。
3. **查询数据结构**：参考 [09_核心数据结构速查](./09_核心数据结构速查.md)。
