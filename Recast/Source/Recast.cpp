//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

namespace
{
/// Allocates and constructs an object of the given type, returning a pointer.
/// @param[in]		allocLifetime	Allocation lifetime hint
template<typename T>
T* rcNew(const rcAllocHint allocLifetime)
{
	T* ptr = (T*)rcAlloc(sizeof(T), allocLifetime);
	::new(rcNewTag(), (void*)ptr) T();
	return ptr;
}

/// Destroys and frees an object allocated with rcNew.
/// @param[in]     ptr    The object pointer to delete.
template<typename T>
void rcDelete(T* ptr)
{
	if (ptr)
	{
		ptr->~T();
		rcFree((void*)ptr);
	}
}
} // anonymous namespace

float rcSqrt(float x)
{
	return sqrtf(x);
}

void rcContext::log(const rcLogCategory category, const char* format, ...)
{
	if (!m_logEnabled)
	{
		return;
	}
	static const int MSG_SIZE = 512;
	char msg[MSG_SIZE];
	va_list argList;
	va_start(argList, format);
	int len = vsnprintf(msg, MSG_SIZE, format, argList);
	if (len >= MSG_SIZE)
	{
		len = MSG_SIZE - 1;
		msg[MSG_SIZE - 1] = '\0';

		const char* errorMessage = "Log message was truncated";
		doLog(RC_LOG_ERROR, errorMessage, (int)strlen(errorMessage));
	}
	va_end(argList);
	doLog(category, msg, len);
}

void rcContext::doResetLog()
{
	// Defined out of line to fix the weak v-tables warning
}

/// @brief 分配并构造一个 rcHeightfield（高度场）对象，返回指向该对象的指针。
///
/// 该函数是导航网格构建流程中的第一步辅助操作，用于创建高度场数据结构。
/// 高度场（rcHeightfield）是 Recast 体素化阶段的核心数据结构，它在 XZ 平面上
/// 划分为二维网格，每个网格列中存储一系列沿 Y 轴的高度跨度（rcSpan），
/// 用于表示场景中被几何体占据的空间。
///
/// 内部实现调用了模板函数 rcNew<rcHeightfield>，该模板函数的工作流程为：
///   1. 调用 rcAlloc 分配 sizeof(rcHeightfield) 大小的原始内存块；
///   2. 使用 placement new（定位 new）在已分配的内存上调用 rcHeightfield 的默认构造函数，
///      完成对象的初始化（所有成员变量被值初始化为零/空）。
///
/// 传入的 RC_ALLOC_PERM 是内存分配生命周期提示，表示该内存为"持久分配"，
/// 即内存将在函数调用结束后持续存在，直到显式释放（通过 rcFreeHeightField）。
///
/// @return 指向新分配并已构造好的 rcHeightfield 对象的指针；
///         如果内存分配失败，rcAlloc 返回 NULL，后续 placement new 行为未定义
///         （调用方应检查返回值是否为空）。
///
/// @see rcFreeHeightField, rcCreateHeightfield, rcNew
rcHeightfield* rcAllocHeightfield()
{
	// 调用 rcNew 模板函数，以 RC_ALLOC_PERM（持久分配）方式
	// 分配内存并在原地构造一个 rcHeightfield 对象，然后返回其指针。
	// rcNew 内部流程：
	//   1. rcAlloc(sizeof(rcHeightfield), RC_ALLOC_PERM) — 分配原始内存
	//   2. ::new(rcNewTag(), (void*)ptr) rcHeightfield() — placement new 调用默认构造函数
	//      其中 rcNewTag 是一个空结构体标记，用于区分 Recast 自定义的 placement new
	//      与 STL 标准库中的 placement new，避免在同时引入两者时产生符号冲突。
	//   3. 返回指向已构造对象的指针
	return rcNew<rcHeightfield>(RC_ALLOC_PERM);
}

void rcFreeHeightField(rcHeightfield* heightfield)
{
	rcDelete(heightfield);
}

rcHeightfield::rcHeightfield()
: width()
, height()
, bmin()
, bmax()
, cs()
, ch()
, spans()
, pools()
, freelist()
{
}

rcHeightfield::~rcHeightfield()
{
	// Delete span array.
	rcFree(spans);
	// Delete span pools.
	while (pools)
	{
		rcSpanPool* next = pools->next;
		rcFree(pools);
		pools = next;
	}
}

rcCompactHeightfield* rcAllocCompactHeightfield()
{
	return rcNew<rcCompactHeightfield>(RC_ALLOC_PERM);
}

void rcFreeCompactHeightfield(rcCompactHeightfield* compactHeightfield)
{
	rcDelete(compactHeightfield);
}

rcCompactHeightfield::rcCompactHeightfield()
: width()
, height()
, spanCount()
, walkableHeight()
, walkableClimb()
, borderSize()
, maxDistance()
, maxRegions()
, bmin()
, bmax()
, cs()
, ch()
, cells()
, spans()
, dist()
, areas()
{
}

rcCompactHeightfield::~rcCompactHeightfield()
{
	rcFree(cells);
	rcFree(spans);
	rcFree(dist);
	rcFree(areas);
}

rcHeightfieldLayerSet* rcAllocHeightfieldLayerSet()
{
	return rcNew<rcHeightfieldLayerSet>(RC_ALLOC_PERM);
}

void rcFreeHeightfieldLayerSet(rcHeightfieldLayerSet* layerSet)
{
	rcDelete(layerSet);
}

rcHeightfieldLayerSet::rcHeightfieldLayerSet()
: layers()
, nlayers()
{
}

rcHeightfieldLayerSet::~rcHeightfieldLayerSet()
{
	for (int i = 0; i < nlayers; ++i)
	{
		rcFree(layers[i].heights);
		rcFree(layers[i].areas);
		rcFree(layers[i].cons);
	}
	rcFree(layers);
}


rcContourSet* rcAllocContourSet()
{
	return rcNew<rcContourSet>(RC_ALLOC_PERM);
}

void rcFreeContourSet(rcContourSet* contourSet)
{
	rcDelete(contourSet);
}

rcContourSet::rcContourSet()
: conts()
, nconts()
, bmin()
, bmax()
, cs()
, ch()
, width()
, height()
, borderSize()
, maxError()
{
}

rcContourSet::~rcContourSet()
{
	for (int i = 0; i < nconts; ++i)
	{
		rcFree(conts[i].verts);
		rcFree(conts[i].rverts);
	}
	rcFree(conts);
}

rcPolyMesh* rcAllocPolyMesh()
{
	return rcNew<rcPolyMesh>(RC_ALLOC_PERM);
}

void rcFreePolyMesh(rcPolyMesh* polyMesh)
{
	rcDelete(polyMesh);
}

rcPolyMesh::rcPolyMesh()
: verts()
, polys()
, regs()
, flags()
, areas()
, nverts()
, npolys()
, maxpolys()
, nvp()
, bmin()
, bmax()
, cs()
, ch()
, borderSize()
, maxEdgeError()
{
}

rcPolyMesh::~rcPolyMesh()
{
	rcFree(verts);
	rcFree(polys);
	rcFree(regs);
	rcFree(flags);
	rcFree(areas);
}

rcPolyMeshDetail* rcAllocPolyMeshDetail()
{
	return rcNew<rcPolyMeshDetail>(RC_ALLOC_PERM);
}

void rcFreePolyMeshDetail(rcPolyMeshDetail* detailMesh)
{
	if (detailMesh == NULL)
	{
		return;
	}
	rcFree(detailMesh->meshes);
	rcFree(detailMesh->verts);
	rcFree(detailMesh->tris);
	rcFree(detailMesh);
}

rcPolyMeshDetail::rcPolyMeshDetail()
: meshes()
, verts()
, tris()
, nmeshes()
, nverts()
, ntris()
{
}

void rcCalcBounds(const float* verts, int numVerts, float* minBounds, float* maxBounds)
{
	// Calculate bounding box.
	rcVcopy(minBounds, verts);
	rcVcopy(maxBounds, verts);
	for (int i = 1; i < numVerts; ++i)
	{
		const float* v = &verts[i * 3];
		rcVmin(minBounds, v);
		rcVmax(maxBounds, v);
	}
}

/// @brief 根据包围盒范围和体素格子大小，计算高度场的网格尺寸（XZ平面的体素数）。
///
/// 该函数是导航网格构建流程 Step 1（配置初始化）的关键步骤，
/// 用于将世界空间的包围盒范围转换为离散的体素网格维度。
/// 计算结果会赋值给 rcConfig::width 和 rcConfig::height。
///
/// 计算公式：
///   sizeX = (int)((maxBounds[0] - minBounds[0]) / cellSize + 0.5f)
///   sizeZ = (int)((maxBounds[2] - minBounds[2]) / cellSize + 0.5f)
///
/// 其中 +0.5f 是为了实现四舍五入取整，确保网格能完整覆盖包围盒范围，
/// 避免因截断导致边界处的几何体被遗漏。
///
/// 注意：这里使用 X 和 Z 两个轴（即 XZ 水平平面），因为 Recast 中 Y 轴朝上，
/// 高度场是在 XZ 平面上展开的二维网格，每个格子在 Y 方向上存储一系列高度跨度(Span)。
///
/// @param[in]  minBounds  构建区域 AABB 包围盒的最小角坐标 (x, y, z)，世界单位。
/// @param[in]  maxBounds  构建区域 AABB 包围盒的最大角坐标 (x, y, z)，世界单位。
/// @param[in]  cellSize   XZ 平面上每个体素格子的边长（世界单位），即 rcConfig::cs。
/// @param[out] sizeX      计算得到的 X 轴方向网格数量（体素数），对应 rcConfig::width。
/// @param[out] sizeZ      计算得到的 Z 轴方向网格数量（体素数），对应 rcConfig::height。
void rcCalcGridSize(const float* minBounds, const float* maxBounds, const float cellSize, int* sizeX, int* sizeZ)
{
	// 计算 X 轴方向的体素数：包围盒在 X 方向的跨度 / 体素边长，四舍五入取整
	*sizeX = (int)((maxBounds[0] - minBounds[0]) / cellSize + 0.5f);
	// 计算 Z 轴方向的体素数：包围盒在 Z 方向的跨度 / 体素边长，四舍五入取整
	*sizeZ = (int)((maxBounds[2] - minBounds[2]) / cellSize + 0.5f);
}

/// @brief 初始化一个已分配的 rcHeightfield（高度场）对象，设置网格参数并分配 spans 数组。
///
/// 该函数是导航网格构建流程 Step 2（体素化准备阶段）的关键步骤。
/// 在 rcAllocHeightfield() 分配了空的高度场对象之后，需要调用此函数完成实际的初始化工作，
/// 包括：设置网格尺寸、包围盒范围、体素分辨率参数，以及分配 spans 指针数组。
///
/// 初始化完成后，高度场即可用于 rcRasterizeTriangles() 将三角形光栅化为 Span。
///
/// 典型调用顺序：
///   1. rcCalcBounds()        — 计算场景几何体的包围盒 (bmin, bmax)
///   2. rcCalcGridSize()      — 根据包围盒和 cellSize 计算网格尺寸 (sizeX, sizeZ)
///   3. rcAllocHeightfield()  — 分配 rcHeightfield 对象
///   4. rcCreateHeightfield() — [本函数] 初始化高度场参数并分配 spans 数组
///   5. rcRasterizeTriangles()— 将三角形网格光栅化填入高度场
///
/// @param[in]     context       构建上下文（用于日志和计时），当前此函数中未使用。
/// @param[in,out] heightfield   待初始化的高度场对象引用（应由 rcAllocHeightfield 创建）。
/// @param[in]     sizeX         X 轴方向的网格列数（体素数），由 rcCalcGridSize 计算得出。
/// @param[in]     sizeZ         Z 轴方向的网格行数（体素数），由 rcCalcGridSize 计算得出。
/// @param[in]     minBounds     构建区域 AABB 包围盒最小角坐标 [(x, y, z)]，世界单位。
/// @param[in]     maxBounds     构建区域 AABB 包围盒最大角坐标 [(x, y, z)]，世界单位。
/// @param[in]     cellSize      XZ 平面上每个体素格子的边长（世界单位），即水平分辨率。
/// @param[in]     cellHeight    Y 轴方向每个体素的高度（世界单位），即垂直分辨率。
/// @return 初始化成功返回 true；如果 spans 数组内存分配失败返回 false。
///
/// @see rcAllocHeightfield, rcCalcGridSize, rcCalcBounds, rcHeightfield
bool rcCreateHeightfield(rcContext* context, rcHeightfield& heightfield, int sizeX, int sizeZ,
                         const float* minBounds, const float* maxBounds,
                         float cellSize, float cellHeight)
{
	// 消除编译器对 context 参数未使用的警告。
	// 此函数当前不使用 context 进行日志或计时，但保留参数以与其他 rc* 函数签名一致。
	// rcIgnoreUnused 是一个空模板函数：template<class T> void rcIgnoreUnused(const T&) {}
	rcIgnoreUnused(context);

	// ---- 步骤 1：设置网格尺寸参数 ----

	// 设置高度场在 X 轴方向的网格列数（体素数）。
	// 该值通常由 rcCalcGridSize() 计算：(int)((bmax[0] - bmin[0]) / cs + 0.5f)
	heightfield.width = sizeX;

	// 设置高度场在 Z 轴方向的网格行数（体素数）。
	// 注意：rcHeightfield 中 "height" 指的是 Z 轴方向的格子数，而非 Y 轴的高度。
	// 该值通常由 rcCalcGridSize() 计算：(int)((bmax[2] - bmin[2]) / cs + 0.5f)
	heightfield.height = sizeZ;

	// ---- 步骤 2：设置包围盒范围 ----

	// 将包围盒最小角坐标 (x, y, z) 复制到高度场的 bmin 数组中。
	// rcVcopy 是内联函数，执行逐分量复制：dest[0]=v[0], dest[1]=v[1], dest[2]=v[2]
	// bmin 定义了高度场覆盖区域的下界，是世界坐标到体素坐标转换的基准原点。
	rcVcopy(heightfield.bmin, minBounds);

	// 将包围盒最大角坐标 (x, y, z) 复制到高度场的 bmax 数组中。
	// bmax 定义了高度场覆盖区域的上界。
	rcVcopy(heightfield.bmax, maxBounds);

	// ---- 步骤 3：设置体素分辨率参数 ----

	// 设置 XZ 平面上每个体素格子的边长（世界单位），即水平分辨率。
	// cs 越小，网格越精细（width × height 越大），精度越高但内存和计算开销也越大。
	// 典型值范围：0.1 ~ 1.0（取决于场景规模）。
	heightfield.cs = cellSize;

	// 设置 Y 轴方向每个体素的高度（世界单位），即垂直分辨率。
	// ch 决定了 rcSpan 的 smin/smax 的量化精度。
	// 世界高度到体素高度的转换：voxelY = (worldY - bmin[1]) / ch
	// 典型值范围：0.1 ~ 0.5（通常 ch <= cs）。
	heightfield.ch = cellHeight;

	// ---- 步骤 4：分配 spans 指针数组 ----

	// 分配一个 rcSpan* 指针数组，大小为 width × height 个指针。
	// 这个数组是高度场的核心存储结构：每个元素 spans[x + z * width] 对应
	// XZ 平面上 (x, z) 位置的 Span 链表头指针。
	// 使用 rcAlloc 进行内存分配（默认实现为 malloc），分配提示为 RC_ALLOC_PERM（持久分配）。
	// 总分配大小 = sizeof(rcSpan*) × width × height 字节。
	// 例如：1000×1000 的网格在 64 位系统上需要 ~8MB 的指针数组。
	heightfield.spans = (rcSpan**)rcAlloc(sizeof(rcSpan*) * heightfield.width * heightfield.height, RC_ALLOC_PERM);

	// 检查内存分配是否成功。如果 rcAlloc 返回 NULL，表示内存不足，
	// 函数提前返回 false，调用方应根据返回值进行错误处理。
	if (!heightfield.spans)
	{
		return false;
	}

	// 将整个 spans 指针数组清零（所有指针设为 NULL）。
	// 清零后，每个网格列的 Span 链表初始为空（无实体跨度），
	// 后续 rcRasterizeTriangles() 会逐步向各列中插入 Span。
	// 使用 memset 而非逐元素赋值，效率更高。
	memset(heightfield.spans, 0, sizeof(rcSpan*) * heightfield.width * heightfield.height);

	// 初始化成功，高度场已准备就绪，可以开始光栅化三角形。
	return true;
}

/// @brief 计算三角形面的单位法向量。
///
/// 通过三角形的三个顶点，利用两条边向量的叉积计算面法线方向，
/// 然后归一化为单位向量。法线方向遵循右手定则（v0→v1 × v0→v2）。
///
/// 在 Recast 坐标系中 Y 轴朝上，因此法向量的 Y 分量 (faceNormal[1])
/// 表示面朝上的程度：
///   - faceNormal[1] = 1.0  → 面完全水平朝上（地面）
///   - faceNormal[1] = 0.0  → 面完全垂直（墙壁）
///   - faceNormal[1] < 0.0  → 面朝下（天花板/悬崖底面）
///
/// 该值后续用于与可行走斜坡阈值比较，判断三角形是否可行走。
///
/// @param[in]  v0          三角形第一个顶点坐标 [(x, y, z)]
/// @param[in]  v1          三角形第二个顶点坐标 [(x, y, z)]
/// @param[in]  v2          三角形第三个顶点坐标 [(x, y, z)]
/// @param[out] faceNormal  输出的单位法向量 [(x, y, z)]，长度为 1.0
static void calcTriNormal(const float* v0, const float* v1, const float* v2, float* faceNormal)
{
	// 声明两个 3D 向量，用于存储三角形的两条边。
	float e0[3], e1[3];

	// 计算边向量 e0 = v1 - v0（从顶点 v0 指向 v1 的方向向量）。
	// rcVsub 执行逐分量减法：dest[i] = v1[i] - v2[i]
	rcVsub(e0, v1, v0);

	// 计算边向量 e1 = v2 - v0（从顶点 v0 指向 v2 的方向向量）。
	rcVsub(e1, v2, v0);

	// 计算两条边向量的叉积：faceNormal = e0 × e1。
	// 叉积结果是垂直于三角形面的法向量，方向由右手定则决定。
	// rcVcross 计算公式：
	//   dest[0] = v1[1]*v2[2] - v1[2]*v2[1]
	//   dest[1] = v1[2]*v2[0] - v1[0]*v2[2]
	//   dest[2] = v1[0]*v2[1] - v1[1]*v2[0]
	// 此时 faceNormal 的长度等于三角形面积的两倍（平行四边形面积），尚未归一化。
	rcVcross(faceNormal, e0, e1);

	// 将法向量归一化为单位向量（长度 = 1.0）。
	// rcVnormalize 计算：d = 1/sqrt(x² + y² + z²)，然后各分量乘以 d。
	// 归一化后，faceNormal[1]（Y 分量）即为法向量与 Y 轴的夹角余弦值 cos(θ)，
	// 可直接与可行走斜坡的余弦阈值比较。
	rcVnormalize(faceNormal);
}

/// @brief 标记可行走的三角形，将其区域 ID 设置为 RC_WALKABLE_AREA。
///
/// 该函数是导航网格构建流程中**体素化前的预处理步骤**。
/// 它遍历所有输入的三角形，通过计算每个三角形面法线的 Y 分量（即法线与竖直方向的夹角余弦值），
/// 来判断该三角形的坡度是否在可行走范围内。
///
/// 判断原理：
///   - 三角形面法线 N 与 Y 轴（竖直向上）的夹角为 θ
///   - 法线的 Y 分量 N.y = cos(θ)（归一化后）
///   - 若 θ < walkableSlopeAngle，则 cos(θ) > cos(walkableSlopeAngle)，即面足够平坦
///   - 满足条件的三角形被标记为 RC_WALKABLE_AREA (63)
///
/// 注意：此函数只会将可行走的三角形标记为 RC_WALKABLE_AREA，
/// 不会修改不可行走三角形的现有 triAreaIDs 值。调用方通常应在调用前
/// 先将 triAreaIDs 数组全部初始化为 0（RC_NULL_AREA，不可行走）。
///
/// 与 rcClearUnwalkableTriangles 是互补函数：
///   - rcMarkWalkableTriangles:   可行走 → 设为 RC_WALKABLE_AREA（正向标记）
///   - rcClearUnwalkableTriangles: 不可行走 → 设为 RC_NULL_AREA（反向清除）
///
/// @param[in]     context             构建上下文（当前未使用，保留以与 API 签名一致）。
/// @param[in]     walkableSlopeAngle  最大可行走坡度角（单位：度）。
///                                    例如 45.0f 表示坡度 ≤ 45° 的面可行走。
/// @param[in]     verts               顶点数组，连续存储 [(x, y, z) * numVerts]。
/// @param[in]     numVerts            顶点数量（当前未使用，仅用于 API 完整性）。
/// @param[in]     tris                三角形索引数组，每 3 个整数为一组 [(v0, v1, v2) * numTris]，
///                                    每个值是 verts 数组中的顶点索引。
/// @param[in]     numTris             三角形数量。
/// @param[in,out] triAreaIDs          三角形区域 ID 数组 [大小: numTris]。
///                                    可行走的三角形会被设置为 RC_WALKABLE_AREA (63)。
///
/// @see calcTriNormal, rcClearUnwalkableTriangles, RC_WALKABLE_AREA
void rcMarkWalkableTriangles(rcContext* context, const float walkableSlopeAngle,
                             const float* verts, const int numVerts,
                             const int* tris, const int numTris,
                             unsigned char* triAreaIDs)
{
	// 消除编译器对 context 参数未使用的警告。
	// 当前函数不进行日志记录或计时，但保留参数以保持 API 一致性。
	rcIgnoreUnused(context);

	// 消除编译器对 numVerts 参数未使用的警告。
	// numVerts 未在函数内部使用（三角形索引直接从 tris 数组获取），
	// 但作为参数保留以提供 API 完整性和便于调试。
	rcIgnoreUnused(numVerts);

	// 将可行走坡度角（度）转换为对应的余弦值阈值。
	// 转换过程：
	//   1. walkableSlopeAngle / 180.0f * RC_PI — 将角度制转换为弧度制
	//      例如：45° → 45/180 * π = π/4 ≈ 0.7854 弧度
	//   2. cosf(...) — 计算该弧度的余弦值
	//      例如：cos(π/4) ≈ 0.7071
	// 后续用法：若三角形法线的 Y 分量 > walkableThr，则该三角形可行走。
	// 原理：法线 Y 分量 = cos(法线与Y轴夹角)，夹角越小（面越平）→ cos 值越大。
	const float walkableThr = cosf(walkableSlopeAngle / 180.0f * RC_PI);

	// 声明一个 3D 向量，用于存储每个三角形的面法线。
	// 在循环中反复复用此变量，避免重复分配。
	float norm[3];

	// 遍历所有三角形，逐个判断是否可行走。
	for (int i = 0; i < numTris; ++i)
	{
		// 获取第 i 个三角形的三个顶点索引。
		// tris 数组以连续的三元组存储：[tri0_v0, tri0_v1, tri0_v2, tri1_v0, ...]。
		// tri[0], tri[1], tri[2] 分别是三个顶点在 verts 数组中的索引。
		const int* tri = &tris[i * 3];

		// 计算三角形的面法线（单位向量）。
		// 每个顶点在 verts 中占 3 个 float (x, y, z)，所以乘以 3 得到偏移量。
		// 例如：&verts[tri[0] * 3] 指向第 tri[0] 号顶点的 (x, y, z) 起始位置。
		// calcTriNormal 通过叉积计算面法线并归一化为单位向量。
		calcTriNormal(&verts[tri[0] * 3], &verts[tri[1] * 3], &verts[tri[2] * 3], norm);

		// 判断三角形是否可行走：比较法线的 Y 分量与可行走余弦阈值。
		// norm[1] 是法线的 Y 分量，等于 cos(法线与Y轴的夹角)：
		//   - norm[1] ≈ 1.0 → 面接近水平（完全平地）
		//   - norm[1] ≈ 0.0 → 面接近垂直（墙壁）
		//   - norm[1] < 0.0 → 面朝下（天花板）
		// 若 norm[1] > walkableThr，说明法线与 Y 轴夹角 < walkableSlopeAngle，
		// 即三角形坡度在可行走范围内。
		if (norm[1] > walkableThr)
		{
			// 将该三角形的区域 ID 标记为 RC_WALKABLE_AREA (63)。
			// 此标记会在后续的光栅化步骤 (rcRasterizeTriangles) 中
			// 被写入到对应 rcSpan 的 area 字段中，
			// 最终影响导航网格中哪些区域允许寻路代理通行。
			triAreaIDs[i] = RC_WALKABLE_AREA;
		}
	}
}

void rcClearUnwalkableTriangles(rcContext* context, const float walkableSlopeAngle,
                                const float* verts, int numVerts,
                                const int* tris, int numTris,
                                unsigned char* triAreaIDs)
{
	rcIgnoreUnused(context);
	rcIgnoreUnused(numVerts);

	// The minimum Y value for a face normal of a triangle with a walkable slope.
	const float walkableLimitY = cosf(walkableSlopeAngle / 180.0f * RC_PI);

	float faceNormal[3];
	for (int i = 0; i < numTris; ++i)
	{
		const int* tri = &tris[i * 3];
		calcTriNormal(&verts[tri[0] * 3], &verts[tri[1] * 3], &verts[tri[2] * 3], faceNormal);
		// Check if the face is walkable.
		if (faceNormal[1] <= walkableLimitY)
		{
			triAreaIDs[i] = RC_NULL_AREA;
		}
	}
}

/// @brief 统计链表式高度场（rcHeightfield）中所有**可行走** Span 的总数。
///
/// 该函数遍历高度场的所有网格列，对每一列中的 Span 链表逐一检查，
/// 只统计 area 字段不为 RC_NULL_AREA (0) 的 Span 数量。
///
/// 统计结果用于 rcBuildCompactHeightfield() 中为紧凑高度场的
/// spans[] 和 areas[] 数组预分配精确大小的内存空间。
///
/// 为什么只统计可行走 Span？
///   - 不可行走的 Span（area == RC_NULL_AREA）表示墙壁、天花板等不可通行的实体，
///     在后续的紧凑高度场中不需要保留，因此不计入数组大小。
///   - 只有可行走 Span 的顶面才是 Agent 可以站立的表面，才需要转换为紧凑 Span。
///
/// 数据流在构建流程中的位置：
///   rcRasterizeTriangles → rcFilterXxx → [rcGetHeightFieldSpanCount]
///     → rcBuildCompactHeightfield（使用返回值分配数组）
///
/// @param[in]  context      构建上下文（当前未使用，保留以与 API 签名一致）。
/// @param[in]  heightfield  已完成光栅化和过滤的链表式高度场。
/// @return 高度场中所有可行走 Span 的总数。
///
/// @see rcBuildCompactHeightfield, rcHeightfield, RC_NULL_AREA
int rcGetHeightFieldSpanCount(rcContext* context, const rcHeightfield& heightfield)
{
	// 消除编译器对 context 参数未使用的警告。
	// 当前函数不进行日志记录或计时，但保留参数以保持 API 一致性。
	rcIgnoreUnused(context);

	// 计算高度场在 XZ 平面上的网格列总数。
	// numCols = width × height，即 X 方向列数 × Z 方向行数。
	// 注意：这里的 height 是 Z 方向的网格行数（rcHeightfield::height），不是 Y 方向的高度。
	const int numCols = heightfield.width * heightfield.height;

	// 可行走 Span 的累计计数器，初始为 0。
	int spanCount = 0;

	// 外层循环：遍历高度场中的每一个网格列（共 numCols 列）。
	// 每个 columnIndex 对应 XZ 平面上的一个格子位置 (x, z)，
	// 其中 x = columnIndex % width，z = columnIndex / width。
	for (int columnIndex = 0; columnIndex < numCols; ++columnIndex)
	{
		// 内层循环：遍历当前列中的 Span 链表。
		// heightfield.spans[columnIndex] 是该列的链表头指针，
		// 链表按 smin 升序排列（从下到上），通过 span->next 遍历。
		//
		// 示意图（单列的 Span 链表）：
		//
		//   Y轴↑
		//        ┌──────────┐
		//        │ span2    │ area=63  ← 可行走 ✅ spanCount++
		//   smax ├──────────┤
		//        │ span1    │ area=0   ← 不可行走 ❌ 跳过
		//   smax ├──────────┤
		//        │ span0    │ area=63  ← 可行走 ✅ spanCount++
		//   smax ├──────────┤
		//        └──────────┘
		//
		//   该列贡献 spanCount += 2
		for (rcSpan* span = heightfield.spans[columnIndex]; span != NULL; span = span->next)
		{
			// 检查当前 Span 是否可行走。
			// area == RC_NULL_AREA (0) 表示不可行走（被 rcFilterXxx 系列函数标记，
			// 或在 rcMarkWalkableTriangles 中未被标记为可行走）。
			// area != RC_NULL_AREA（通常为 RC_WALKABLE_AREA = 63）表示可行走。
			if (span->area != RC_NULL_AREA)
			{
				// 累加可行走 Span 计数。
				spanCount++;
			}
		}
	}

	// 返回可行走 Span 的总数。
	// 该值将被 rcBuildCompactHeightfield 用于分配以下数组：
	//   - compactHeightfield.spans[spanCount]  — 紧凑 Span 数组
	//   - compactHeightfield.areas[spanCount]  — 区域类型 ID 数组
	return spanCount;
}

/// @brief 将链表式高度场（rcHeightfield）转换为紧凑高度场（rcCompactHeightfield），
///        并建立四邻域连接关系。
///
/// 该函数是导航网格构建流程中的**关键转换步骤**，将光栅化+过滤后的链表式高度场
/// 转换为数组式的紧凑高度场。紧凑高度场用连续数组取代链表存储，大幅提升后续
/// 区域划分、距离计算等步骤的缓存命中率和遍历效率。
///
/// 核心转换思想——从"实体视角"到"空间视角"：
///
///   rcHeightfield（链表高度场）：
///     - 每个 rcSpan 表示一段被**实体几何**占据的垂直区间 [smin, smax]
///     - area 标记在 Span 的**顶面**（smax）上
///     - 链表结构，不利于随机访问和缓存
///
///   rcCompactHeightfield（紧凑高度场）：
///     - 每个 rcCompactSpan 表示一段**可通行的开放空间**
///     - y = 开放空间的底面（= 原 rcSpan.smax，即实体顶面）
///     - h = 开放空间的高度（= 下一个 rcSpan.smin - 当前 rcSpan.smax）
///     - 用连续数组存储，支持高效索引访问
///     - 包含四方向邻居连接信息（con 字段）
///
/// 函数分为两个主要阶段：
///   阶段一：转换数据结构（链表 → 数组）
///   阶段二：建立四邻域连接关系
///
/// 在构建流程中的位置：
///   rcRasterizeTriangles → rcFilterXxx(三步过滤)
///     → [rcBuildCompactHeightfield]
///     → rcErodeWalkableArea → rcBuildDistanceField → rcBuildRegions
///     → rcBuildContours → rcBuildPolyMesh → rcBuildPolyMeshDetail
///
/// @param[in]     context              构建上下文，用于日志和计时。不可为 NULL。
/// @param[in]     walkableHeight       Agent 的最小通行高度（体素单位）。
///                                     开放空间高度 ≥ 此值时才允许通行。
/// @param[in]     walkableClimb        Agent 的最大可攀爬高度（体素单位）。
///                                     相邻 Span 底面高度差 ≤ 此值时才建立连接。
/// @param[in]     heightfield          输入的链表式高度场（已光栅化并过滤完成）。
/// @param[in,out] compactHeightfield   输出的紧凑高度场（应由 rcAllocCompactHeightfield 创建）。
/// @return 成功返回 true；内存分配失败返回 false。
///
/// @see rcAllocCompactHeightfield, rcHeightfield, rcCompactHeightfield
bool rcBuildCompactHeightfield(rcContext* context, const int walkableHeight, const int walkableClimb,
                               const rcHeightfield& heightfield, rcCompactHeightfield& compactHeightfield)
{
	// 断言上下文非空，确保日志和计时功能可用。
	rcAssert(context);

	// RAII 作用域计时器：自动记录本函数的执行耗时到 RC_TIMER_BUILD_COMPACTHEIGHTFIELD 类别。
	rcScopedTimer timer(context, RC_TIMER_BUILD_COMPACTHEIGHTFIELD);

	// 获取原始高度场的网格尺寸。
	const int xSize = heightfield.width;   // X 轴方向的体素列数
	const int zSize = heightfield.height;  // Z 轴方向的体素行数

	// 统计原始高度场中可行走 Span 的总数（area != RC_NULL_AREA）。
	// 该值决定了紧凑高度场中 spans/areas 数组需要分配的大小。
	// 只有可行走的 Span 才会被转入紧凑高度场，不可行走的 Span 被丢弃。
	const int spanCount = rcGetHeightFieldSpanCount(context, heightfield);

	// ========================================================================
	// 阶段一：填充紧凑高度场的头部信息并分配内存
	// ========================================================================

	// 复制网格基本参数。
	compactHeightfield.width = xSize;          // X 轴网格列数
	compactHeightfield.height = zSize;         // Z 轴网格行数
	compactHeightfield.spanCount = spanCount;  // 可行走 Span 总数
	compactHeightfield.walkableHeight = walkableHeight; // 保存 Agent 通行高度，后续步骤会用到
	compactHeightfield.walkableClimb = walkableClimb;   // 保存 Agent 攀爬高度，后续步骤会用到
	compactHeightfield.maxRegions = 0;         // 区域计数初始化为 0，后续 rcBuildRegions 会填充

	// 复制包围盒范围。
	rcVcopy(compactHeightfield.bmin, heightfield.bmin);
	rcVcopy(compactHeightfield.bmax, heightfield.bmax);

	// 将包围盒上界 Y 值向上扩展 walkableHeight 个体素的高度。
	// 这是因为紧凑高度场存储的是"开放空间"，最顶部的开放空间可以向上延伸
	// walkableHeight 的距离（Agent 的身高），所以包围盒需要相应扩展以覆盖这部分空间。
	compactHeightfield.bmax[1] += walkableHeight * heightfield.ch;

	// 复制体素分辨率参数。
	compactHeightfield.cs = heightfield.cs;  // 水平体素尺寸
	compactHeightfield.ch = heightfield.ch;  // 垂直体素尺寸

	// ---- 分配 cells 数组 ----
	// cells 数组大小 = xSize * zSize，每个元素对应 XZ 平面上一个网格列。
	// rcCompactCell 包含：
	//   index (24位): 该列中第一个 Span 在 spans 数组中的起始索引
	//   count (8位) : 该列中 Span 的数量（最多 255 个）
	compactHeightfield.cells = (rcCompactCell*)rcAlloc(sizeof(rcCompactCell) * xSize * zSize, RC_ALLOC_PERM);
	if (!compactHeightfield.cells)
	{
		context->log(RC_LOG_ERROR, "rcBuildCompactHeightfield: Out of memory 'chf.cells' (%d)", xSize * zSize);
		return false;
	}
	memset(compactHeightfield.cells, 0, sizeof(rcCompactCell) * xSize * zSize);

	// ---- 分配 spans 数组 ----
	// spans 数组大小 = spanCount，所有列的可行走 Span 紧凑地存储在一起。
	// rcCompactSpan 包含：
	//   y   (16位): 开放空间底面高度（= 原始 rcSpan 的 smax）
	//   h   (8位) : 开放空间的垂直高度（= 上方 Span 的 smin - 本 Span 的 smax）
	//   con (24位): 四方向邻居连接信息，每个方向 6 位
	//   reg (16位): 所属区域 ID（后续 rcBuildRegions 填充）
	compactHeightfield.spans = (rcCompactSpan*)rcAlloc(sizeof(rcCompactSpan) * spanCount, RC_ALLOC_PERM);
	if (!compactHeightfield.spans)
	{
		context->log(RC_LOG_ERROR, "rcBuildCompactHeightfield: Out of memory 'chf.spans' (%d)", spanCount);
		return false;
	}
	memset(compactHeightfield.spans, 0, sizeof(rcCompactSpan) * spanCount);

	// ---- 分配 areas 数组 ----
	// areas 数组大小 = spanCount，与 spans 一一对应，存储每个 Span 的区域类型 ID。
	// 初始化为 RC_NULL_AREA (0)，后续从原始高度场中复制实际区域值。
	compactHeightfield.areas = (unsigned char*)rcAlloc(sizeof(unsigned char) * spanCount, RC_ALLOC_PERM);
	if (!compactHeightfield.areas)
	{
		context->log(RC_LOG_ERROR, "rcBuildCompactHeightfield: Out of memory 'chf.areas' (%d)", spanCount);
		return false;
	}
	memset(compactHeightfield.areas, RC_NULL_AREA, sizeof(unsigned char) * spanCount);

	// 最大高度值（16位无符号整数的上限 65535）。
	// 当列中最上方的 Span 没有上方邻居时，其开放空间的顶面被视为"无穷高"，
	// 用此值表示。
	const int MAX_HEIGHT = 0xffff;

	// ========================================================================
	// 阶段一（续）：遍历原始高度场，填充 cells 和 spans 数组
	// ========================================================================
	//
	// 核心转换逻辑示意：
	//
	//   原始 rcHeightfield 的一列                紧凑 rcCompactHeightfield 的对应表示
	//
	//   Y轴↑                                    Y轴↑
	//        ┌──────────────┐ ← MAX_HEIGHT            开放空间2 (h = 很大)
	//        │  开放空间 2   │                          ↑
	//        │  (可通行)     │                    ┌──────────────┐
	//   smin ├──────────────┤ ← span2 (area=63)  │ CompactSpan1 │ y=span2.smax
	//        │  ████实体████ │                    │              │ h=MAX_HEIGHT-span2.smax
	//   smax ├──────────────┤                    │ area=63      │
	//        │  开放空间 1   │                    └──────────────┘
	//        │  (可通行)     │
	//   smin ├──────────────┤ ← span1 (area=63)  ┌──────────────┐
	//        │  ████实体████ │                    │ CompactSpan0 │ y=span1.smax
	//   smax ├──────────────┤                    │              │ h=span2.smin-span1.smax
	//        │  开放空间 0   │                    │ area=63      │
	//        │  (不可通行)   │                    └──────────────┘
	//   smin ├──────────────┤ ← span0 (area=0)   (跳过，area=0)
	//        │  ████实体████ │
	//   smax ├──────────────┤
	//        └──────────────┘
	//

	// currentCellIndex 是全局的 Span 写入游标，在所有列中递增。
	// 同一列内的 Span 在数组中连续存放，通过 cell.index 和 cell.count 定位。
	int currentCellIndex = 0;
	const int numColumns = xSize * zSize;

	// 遍历高度场中的每一列。
	for (int columnIndex = 0; columnIndex < numColumns; ++columnIndex)
	{
		// 获取当前列的 Span 链表头指针。
		const rcSpan* span = heightfield.spans[columnIndex];
			
		// 如果该列没有任何 Span，跳过。
		// cells 数组已清零，index=0 和 count=0 表示空列。
		if (span == NULL)
		{
			continue;
		}
			
		// 获取当前列对应的 CompactCell 引用，设置起始索引。
		rcCompactCell& cell = compactHeightfield.cells[columnIndex];
		cell.index = currentCellIndex;  // 该列第一个 Span 在 spans 数组中的位置
		cell.count = 0;                 // 该列的 Span 数量，在下方循环中递增

		// 遍历当前列中所有原始 Span（从下到上，链表按 smin 升序排列）。
		for (; span != NULL; span = span->next)
		{
			// 只处理可行走的 Span（area != RC_NULL_AREA）。
			// 不可行走的 Span（如墙壁、天花板）被直接跳过，不会进入紧凑高度场。
			if (span->area != RC_NULL_AREA)
			{
				// ---- 计算开放空间的底面高度（y）----
				// 紧凑 Span 的 y = 原始 Span 的 smax（实体顶面 = 开放空间底面）。
				// 这就是"从实体视角到空间视角"的核心转换：
				//   原始 Span：[smin, smax] 描述实体占据的范围
				//   紧凑 Span：y 描述可站立表面的高度，即实体的顶面
				const int bot = (int)span->smax;

				// ---- 计算开放空间的顶面高度 ----
				// 如果上方还有 Span，则开放空间的顶面 = 上方 Span 的 smin（上方实体的底面）。
				// 如果这是最顶部的 Span，则开放空间向上延伸到"无穷高"（MAX_HEIGHT = 0xffff）。
				const int top = span->next ? (int)span->next->smin : MAX_HEIGHT;

				// ---- 填充紧凑 Span ----
				// y: 开放空间底面高度，钳制到 [0, 0xffff]（16位无符号整数范围）。
				compactHeightfield.spans[currentCellIndex].y = (unsigned short)rcClamp(bot, 0, 0xffff);

				// h: 开放空间的垂直高度（顶面 - 底面），钳制到 [0, 0xff]（8位，最大 255 体素）。
				// 注意：h 的 8 位限制意味着单个开放空间最大高度为 255 个体素。
				// 超过此范围的开放空间高度会被截断为 255，但这通常不影响导航，
				// 因为 Agent 高度远小于 255 个体素。
				compactHeightfield.spans[currentCellIndex].h = (unsigned char)rcClamp(top - bot, 0, 0xff);

				// 从原始 Span 复制区域类型 ID。
				compactHeightfield.areas[currentCellIndex] = span->area;

				// 移动全局游标，准备写入下一个 Span。
				currentCellIndex++;
				// 递增当前列的 Span 计数。
				cell.count++;
			}
		}
	}
	
	// ========================================================================
	// 阶段二：建立四邻域连接关系
	// ========================================================================
	//
	// 对每个紧凑 Span，检查其上下左右四个方向的相邻列，
	// 寻找可以通行的邻居 Span 并建立连接。
	//
	// 连接存储方式：
	//   rcCompactSpan.con 是一个 24 位字段，被分成 4 个 6 位区段，
	//   每个区段存储一个方向的邻居 Span 索引（在邻居列内的局部索引 0~62）。
	//   0x3f (63) = RC_NOT_CONNECTED 表示无连接。
	//
	//   con 字段布局（24位）：
	//   [dir3: 6位][dir2: 6位][dir1: 6位][dir0: 6位]
	//
	// 四方向定义（基于 rcGetDirOffsetX/Y）：
	//   dir=0: X-1（左）   dir=1: Z+1（前/北）
	//   dir=2: X+1（右）   dir=3: Z-1（后/南）
	//
	//   方向示意图：
	//        Z+1 (dir=1)
	//         ↑
	//   X-1 ←   → X+1
	//  (dir=0)  (dir=2)
	//         ↓
	//        Z-1 (dir=3)
	//
	// 连接条件（两个条件必须同时满足）：
	//   1. 两个 Span 的开放空间在 Y 轴方向有足够的重叠区域，
	//      重叠高度 ≥ walkableHeight（Agent 能站立通过）
	//   2. 两个 Span 底面（可站立表面）的高度差 ≤ walkableClimb（Agent 能爬上去）
	//

	// MAX_LAYERS = RC_NOT_CONNECTED - 1 = 62。
	// 这是单列内最大可索引的 Span 层数。因为 6 位中 0x3f (63) 被保留为 "未连接" 标记，
	// 所以有效的邻居索引范围是 0 ~ 62，即最多 63 层。
	const int MAX_LAYERS = RC_NOT_CONNECTED - 1;

	// 记录遇到的最大层索引，用于最后的溢出检测和错误报告。
	int maxLayerIndex = 0;

	// Z 轴步长 = xSize，用于二维坐标到一维索引的转换：index = x + z * zStride。
	const int zStride = xSize;

	// 遍历每个网格列 (x, z)。
	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			// 获取当前列的 CompactCell，定位该列的所有 Span。
			const rcCompactCell& cell = compactHeightfield.cells[x + z * zStride];

			// 遍历当前列中的每个紧凑 Span。
			// i 从 cell.index 开始，到 cell.index + cell.count 结束。
			for (int i = (int)cell.index, ni = (int)(cell.index + cell.count); i < ni; ++i)
			{
				rcCompactSpan& span = compactHeightfield.spans[i];

				// 检查四个方向 (0=左, 1=前, 2=右, 3=后) 的邻居。
				for (int dir = 0; dir < 4; ++dir)
				{
					// 先将该方向初始化为"未连接"。
					// RC_NOT_CONNECTED = 0x3f (63)，填入 con 的对应 6 位区段。
					rcSetCon(span, dir, RC_NOT_CONNECTED);

					// 计算该方向邻居列的坐标。
					// rcGetDirOffsetX: dir=0 → -1, dir=1 → 0, dir=2 → +1, dir=3 → 0
					// rcGetDirOffsetY: dir=0 → 0,  dir=1 → +1, dir=2 → 0,  dir=3 → -1
					const int neighborX = x + rcGetDirOffsetX(dir);
					const int neighborZ = z + rcGetDirOffsetY(dir);

					// 边界检查：邻居列是否在高度场范围内。
					// 超出边界的方向保持 RC_NOT_CONNECTED。
					if (neighborX < 0 || neighborZ < 0 || neighborX >= xSize || neighborZ >= zSize)
					{
						continue;
					}

					// 获取邻居列的 CompactCell，用于遍历该列中的所有 Span。
					const rcCompactCell& neighborCell = compactHeightfield.cells[neighborX + neighborZ * zStride];

					// 遍历邻居列中的每个 Span，寻找可连接的邻居。
					for (int k = (int)neighborCell.index, nk = (int)(neighborCell.index + neighborCell.count); k < nk; ++k)
					{
						const rcCompactSpan& neighborSpan = compactHeightfield.spans[k];

						// 计算两个 Span 的开放空间在 Y 轴方向的重叠区域。
						//
						//   当前 Span:  [span.y, span.y + span.h]  — 开放空间范围
						//   邻居 Span:  [neighborSpan.y, neighborSpan.y + neighborSpan.h]
						//
						//   重叠区域 = [max(底面), min(顶面)]
						//   重叠高度 = top - bot
						//
						//   示意图：
						//   Y轴↑
						//        ┌─────┐           ┌─────┐
						//        │     │ span.y+h  │     │ neighbor.y+h
						//        │     │     ┌─────┤     │
						//        │     │     │ top │     │  ← min(顶面)
						//        │     │     │  ↕  │     │  ← 重叠高度
						//        │     │     │ bot │     │  ← max(底面)
						//        │     ├─────┘     │     │
						//        │     │ span.y    │     │ neighbor.y
						//        └─────┘           └─────┘
						//        当前列              邻居列
						const int bot = rcMax(span.y, neighborSpan.y);
						const int top = rcMin(span.y + span.h, neighborSpan.y + neighborSpan.h);

						// 连接条件判断：
						//   条件1: (top - bot) >= walkableHeight
						//          — 两个 Span 的开放空间重叠区域高度 ≥ Agent 身高，Agent 能通过
						//   条件2: rcAbs(neighborSpan.y - span.y) <= walkableClimb
						//          — 两个 Span 的底面（可站立表面）高度差 ≤ Agent 最大攀爬高度
						//
						// 两个条件缺一不可：
						//   - 只满足条件1（高度够但台阶太高）→ Agent 无法爬上去
						//   - 只满足条件2（台阶矮但头顶太低）→ Agent 过不去（会撞头）
						if ((top - bot) >= walkableHeight && rcAbs((int)neighborSpan.y - (int)span.y) <= walkableClimb)
						{
							// 计算邻居 Span 在其所在列内的局部索引（layer index）。
							// 该索引用于存入 con 字段的 6 位区段中。
							const int layerIndex = k - (int)neighborCell.index;

							// 检查层索引是否在 6 位可表示的范围内 [0, 62]。
							// MAX_LAYERS = 62（因为 63 = RC_NOT_CONNECTED 被保留）。
							// 如果层索引超出范围，记录最大值用于最后的错误报告，然后跳过。
							if (layerIndex < 0 || layerIndex > MAX_LAYERS)
							{
								maxLayerIndex = rcMax(maxLayerIndex, layerIndex);
								continue;
							}

							// 建立连接：将邻居的 layerIndex 写入 span.con 的对应方向位段。
							// rcSetCon 的实现：
							//   shift = dir * 6
							//   con = (con & ~(0x3f << shift)) | ((layerIndex & 0x3f) << shift)
							rcSetCon(span, dir, layerIndex);

							// 每个方向只连接第一个满足条件的邻居 Span（从下往上找到的第一个），
							// 然后 break 跳出内层循环，继续下一个方向。
							// 这确保了连接的唯一性：每个方向最多一个邻居。
							break;
						}
					}
				}
			}
		}
	}

	// 如果有任何列中的 Span 层数超过了 6 位所能表示的范围（> 62 层），
	// 输出错误日志告警。这意味着某些邻居连接无法正确建立。
	// 在实际场景中，单列超过 62 层是极其罕见的（需要极高的垂直复杂度）。
	if (maxLayerIndex > MAX_LAYERS)
	{
		context->log(RC_LOG_ERROR, "rcBuildCompactHeightfield: Heightfield has too many layers %d (max: %d)",
		         maxLayerIndex, MAX_LAYERS);
	}

	return true;
}
