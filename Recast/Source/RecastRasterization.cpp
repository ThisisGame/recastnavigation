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

#include <math.h>
#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"

/// Check whether two bounding boxes overlap
///
/// @param[in]	aMin	Min axis extents of bounding box A
/// @param[in]	aMax	Max axis extents of bounding box A
/// @param[in]	bMin	Min axis extents of bounding box B
/// @param[in]	bMax	Max axis extents of bounding box B
/// @returns true if the two bounding boxes overlap.  False otherwise.
static bool overlapBounds(const float* aMin, const float* aMax, const float* bMin, const float* bMax)
{
	return
		aMin[0] <= bMax[0] && aMax[0] >= bMin[0] &&
		aMin[1] <= bMax[1] && aMax[1] >= bMin[1] &&
		aMin[2] <= bMax[2] && aMax[2] >= bMin[2];
}

/// Allocates a new span in the heightfield.
/// Use a memory pool and free list to minimize actual allocations.
/// 
/// @param[in]	heightfield		The heightfield
/// @returns A pointer to the allocated or re-used span memory. 
static rcSpan* allocSpan(rcHeightfield& heightfield)
{
	// If necessary, allocate new page and update the freelist.
	if (heightfield.freelist == NULL || heightfield.freelist->next == NULL)
	{
		// Create new page.
		// Allocate memory for the new pool.
		rcSpanPool* spanPool = (rcSpanPool*)rcAlloc(sizeof(rcSpanPool), RC_ALLOC_PERM);
		if (spanPool == NULL)
		{
			return NULL;
		}

		// Add the pool into the list of pools.
		spanPool->next = heightfield.pools;
		heightfield.pools = spanPool;
		
		// Add new spans to the free list.
		rcSpan* freeList = heightfield.freelist;
		rcSpan* head = &spanPool->items[0];
		rcSpan* it = &spanPool->items[RC_SPANS_PER_POOL];
		do
		{
			--it;
			it->next = freeList;
			freeList = it;
		}
		while (it != head);
		heightfield.freelist = it;
	}

	// Pop item from the front of the free list.
	rcSpan* newSpan = heightfield.freelist;
	heightfield.freelist = heightfield.freelist->next;
	return newSpan;
}

/// Releases the memory used by the span back to the heightfield, so it can be re-used for new spans.
/// @param[in]	heightfield		The heightfield.
/// @param[in]	span	A pointer to the span to free
static void freeSpan(rcHeightfield& heightfield, rcSpan* span)
{
	if (span == NULL)
	{
		return;
	}
	// Add the span to the front of the free list.
	span->next = heightfield.freelist;
	heightfield.freelist = span;
}

/// @brief 向高度场的指定列 (x, z) 中插入一个新的 Span。
///        如果新 Span 与已有 Span 在垂直方向上存在重叠，则自动合并。
///
/// 这是高度场构建过程中的**关键插入函数**，由 rasterizeTri() 在完成三角形光栅化后调用。
///
/// 核心算法：有序链表插入 + 重叠合并
///   高度场每一列 (x, z) 维护一个按 smin 从小到大排序的 Span 单链表。
///   插入新 Span 时，遍历链表找到合适的位置，同时处理三种情况：
///     1. 当前 Span 完全在新 Span 之上 → 找到插入点，跳出循环
///     2. 当前 Span 完全在新 Span 之下 → 继续向后遍历
///     3. 两者存在重叠 → 合并高度范围，并可能合并区域 ID
///
/// 合并后的 Span 链表示意图：
///
///   插入前：                     插入 new[25,45] 后：
///   Y轴                          Y轴
///    ^  [60,70] area=63           ^  [60,70] area=63
///    |  [40,50] area=0            |  [25,50] area=63  ← 合并了 [30,40] 和 [25,45]
///    |  [30,40] area=63           |  [10,20] area=0
///    |  [10,20] area=0            |
///
/// @param[in,out] heightfield         目标高度场。Span 将插入到其 spans[x + z * width] 链表中。
/// @param[in]     x                   新 Span 所在列的 X 轴网格索引，范围 [0, width-1]。
/// @param[in]     z                   新 Span 所在列的 Z 轴网格索引，范围 [0, height-1]。
/// @param[in]     min                 新 Span 的底部高度（体素单位），即 smin。
/// @param[in]     max                 新 Span 的顶部高度（体素单位），即 smax。
/// @param[in]     areaID              新 Span 的区域类型 ID。
///                                    RC_WALKABLE_AREA (63) = 可行走，RC_NULL_AREA (0) = 不可行走。
/// @param[in]     flagMergeThreshold  区域 ID 合并阈值（体素单位）。
///                                    当新旧 Span 重叠，且两者 smax 之差 ≤ 此值时，
///                                    取两者中较大的 area ID（即更高优先级的区域类型）。
///                                    通常设置为 walkableClimb 对应的体素数。
///                                    这个机制的目的是：当两个表面高度非常接近时（如楼梯台阶），
///                                    保留可行走属性而非被不可行走区域覆盖。
///
/// @return 成功返回 true；如果 allocSpan() 内存分配失败返回 false。
///
/// @see allocSpan, freeSpan, rasterizeTri
static bool addSpan(rcHeightfield& heightfield,
                    const int x, const int z,
                    const unsigned short min, const unsigned short max,
                    const unsigned char areaID, const int flagMergeThreshold)
{
	// ============================================================
	// 第一步：分配并初始化新的 Span
	// ============================================================

	// 从高度场的内存池中分配一个新的 rcSpan 实例。
	// allocSpan() 内部使用空闲链表（freelist）实现 O(1) 的快速分配：
	//   - 如果 freelist 中有可用 Span → 直接弹出链表头部
	//   - 如果 freelist 为空 → 分配新的 rcSpanPool（2048 个 Span），加入 freelist
	rcSpan* newSpan = allocSpan(heightfield);
	if (newSpan == NULL)
	{
		// 内存池分配失败（系统内存不足），返回 false 通知调用方。
		return false;
	}

	// 初始化新 Span 的字段：
	newSpan->smin = min;     // 底部高度（体素单位）
	newSpan->smax = max;     // 顶部高度（体素单位）
	newSpan->area = areaID;  // 区域类型 ID
	newSpan->next = NULL;    // 暂时不连接到链表，后续插入时设置
	
	// ============================================================
	// 第二步：定位插入位置，准备遍历目标列的 Span 链表
	// ============================================================

	// 将二维坐标 (x, z) 转换为一维数组索引。
	// 高度场的 spans 数组按行优先（row-major）存储：index = x + z * width。
	const int columnIndex = x + z * heightfield.width;

	// previousSpan: 遍历过程中用于记录"当前 Span 的前驱"，用于链表插入操作。
	//               初始为 NULL，表示尚未遍历过任何 Span。
	rcSpan* previousSpan = NULL;

	// currentSpan: 当前正在检查的 Span，从链表头开始遍历。
	//              链表按 smin 从小到大排列（从低到高）。
	rcSpan* currentSpan = heightfield.spans[columnIndex];
	
	// ============================================================
	// 第三步：遍历链表，处理插入和合并
	// ============================================================
	// 遍历当前列的 Span 链表（已按 smin 升序排列），
	// 对每个已有 Span 判断其与新 Span 的空间关系。
	while (currentSpan != NULL)
	{
		// ---- 情况 1：当前 Span 完全在新 Span 之上 ----
		// 当前 Span 的底部(smin) > 新 Span 的顶部(smax)，
		// 说明当前 Span 和后续所有 Span 都不可能与新 Span 重叠（链表是升序的）。
		// 新 Span 应该插入到 currentSpan 之前，跳出循环去执行插入。
		if (currentSpan->smin > newSpan->smax)
		{
			break;
		}
		
		// ---- 情况 2：当前 Span 完全在新 Span 之下 ----
		// 当前 Span 的顶部(smax) < 新 Span 的底部(smin)，
		// 说明两者在垂直方向上不重叠，需要继续向后遍历。
		// 记录当前 Span 为 previousSpan，方便后续的链表插入操作。
		if (currentSpan->smax < newSpan->smin)
		{
			previousSpan = currentSpan;
			currentSpan = currentSpan->next;
		}
		else
		{
			// ---- 情况 3：当前 Span 与新 Span 存在重叠 → 合并 ----
			// 两者在垂直方向上有交集，需要将 currentSpan 合并到 newSpan 中。

			// 合并高度范围：取两者的并集 [min(smin), max(smax)]。
			// 如果当前 Span 的底部更低，扩展新 Span 的底部。
			if (currentSpan->smin < newSpan->smin)
			{
				newSpan->smin = currentSpan->smin;
			}
			// 如果当前 Span 的顶部更高，扩展新 Span 的顶部。
			if (currentSpan->smax > newSpan->smax)
			{
				newSpan->smax = currentSpan->smax;
			}
			
			// 合并区域 ID（flagMergeThreshold 机制）：
			// 仅当新旧 Span 的 smax（顶部高度）差值 ≤ flagMergeThreshold 时才合并区域 ID。
			// 这个条件的含义是：只有两个表面高度足够接近（如楼梯的两个相邻台阶），
			// 才应该合并区域属性。否则说明两个 Span 虽然在体素范围上重叠，
			// 但实际表面高度差异较大，应保留新 Span 自身的区域 ID。
			if (rcAbs((int)newSpan->smax - (int)currentSpan->smax) <= flagMergeThreshold)
			{
				// 取两者中较大的 area ID。
				// 数值越大的 area ID 代表越高的优先级：
				//   RC_NULL_AREA (0) < 用户自定义区域 (1~62) < RC_WALKABLE_AREA (63)
				// 这样可以保证可行走区域不会被不可行走区域覆盖。
				newSpan->area = rcMax(newSpan->area, currentSpan->area);
			}
			
			// 从链表中移除已被合并的 currentSpan，并将其归还到空闲链表。
			// 注意：合并后需要继续遍历，因为 newSpan 的范围可能已扩大，
			// 后续还可能存在其他 Span 也与扩大后的 newSpan 重叠。
			rcSpan* next = currentSpan->next;  // 先保存下一个节点
			freeSpan(heightfield, currentSpan); // 释放当前 Span 回内存池

			// 修复链表指针：从链表中摘除 currentSpan。
			if (previousSpan)
			{
				// 有前驱节点：前驱的 next 跳过被删除的节点，指向 next。
				previousSpan->next = next;
			}
			else
			{
				// 无前驱节点（currentSpan 是链表头）：更新列头指针。
				heightfield.spans[columnIndex] = next;
			}

			// 继续检查下一个 Span（不更新 previousSpan，因为 currentSpan 已被删除）。
			currentSpan = next;
		}
	}
	
	// ============================================================
	// 第四步：将新 Span 插入到链表中的正确位置
	// ============================================================
	// 此时遍历已结束，newSpan 应插入在 previousSpan 之后、currentSpan 之前。
	if (previousSpan != NULL)
	{
		// 有前驱节点：将 newSpan 插入到 previousSpan 和 previousSpan->next 之间。
		//   插入前：previousSpan → [currentSpan or NULL]
		//   插入后：previousSpan → newSpan → [currentSpan or NULL]
		newSpan->next = previousSpan->next;
		previousSpan->next = newSpan;
	}
	else
	{
		// 无前驱节点：newSpan 应成为链表的新头节点。
		// 这发生在以下两种情况：
		//   a) 该列原本为空（没有任何 Span）
		//   b) newSpan 的高度范围低于所有已有 Span
		//   插入前：spans[columnIndex] → [currentSpan or NULL]
		//   插入后：spans[columnIndex] → newSpan → [currentSpan or NULL]
		newSpan->next = heightfield.spans[columnIndex];
		heightfield.spans[columnIndex] = newSpan;
	}

	return true;
}

bool rcAddSpan(rcContext* context, rcHeightfield& heightfield,
               const int x, const int z,
               const unsigned short spanMin, const unsigned short spanMax,
               const unsigned char areaID, const int flagMergeThreshold)
{
	rcAssert(context);

	if (!addSpan(heightfield, x, z, spanMin, spanMax, areaID, flagMergeThreshold))
	{
		context->log(RC_LOG_ERROR, "rcAddSpan: Out of memory.");
		return false;
	}

	return true;
}

enum rcAxis
{
	RC_AXIS_X = 0,
	RC_AXIS_Y = 1,
	RC_AXIS_Z = 2
};

/// Divides a convex polygon of max 12 vertices into two convex polygons
/// across a separating axis.
/// 
/// @param[in]	inVerts			The input polygon vertices
/// @param[in]	inVertsCount	The number of input polygon vertices
/// @param[out]	outVerts1		Resulting polygon 1's vertices
/// @param[out]	outVerts1Count	The number of resulting polygon 1 vertices
/// @param[out]	outVerts2		Resulting polygon 2's vertices
/// @param[out]	outVerts2Count	The number of resulting polygon 2 vertices
/// @param[in]	axisOffset		THe offset along the specified axis
/// @param[in]	axis			The separating axis
static void dividePoly(const float* inVerts, int inVertsCount,
                       float* outVerts1, int* outVerts1Count,
                       float* outVerts2, int* outVerts2Count,
                       float axisOffset, rcAxis axis)
{
	rcAssert(inVertsCount <= 12);
	
	// How far positive or negative away from the separating axis is each vertex.
	float inVertAxisDelta[12];
	for (int inVert = 0; inVert < inVertsCount; ++inVert)
	{
		inVertAxisDelta[inVert] = axisOffset - inVerts[inVert * 3 + axis];
	}

	int poly1Vert = 0;
	int poly2Vert = 0;
	for (int inVertA = 0, inVertB = inVertsCount - 1; inVertA < inVertsCount; inVertB = inVertA, ++inVertA)
	{
		// If the two vertices are on the same side of the separating axis
		bool sameSide = (inVertAxisDelta[inVertA] >= 0) == (inVertAxisDelta[inVertB] >= 0);

		if (!sameSide)
		{
			float s = inVertAxisDelta[inVertB] / (inVertAxisDelta[inVertB] - inVertAxisDelta[inVertA]);
			outVerts1[poly1Vert * 3 + 0] = inVerts[inVertB * 3 + 0] + (inVerts[inVertA * 3 + 0] - inVerts[inVertB * 3 + 0]) * s;
			outVerts1[poly1Vert * 3 + 1] = inVerts[inVertB * 3 + 1] + (inVerts[inVertA * 3 + 1] - inVerts[inVertB * 3 + 1]) * s;
			outVerts1[poly1Vert * 3 + 2] = inVerts[inVertB * 3 + 2] + (inVerts[inVertA * 3 + 2] - inVerts[inVertB * 3 + 2]) * s;
			rcVcopy(&outVerts2[poly2Vert * 3], &outVerts1[poly1Vert * 3]);
			poly1Vert++;
			poly2Vert++;
			
			// add the inVertA point to the right polygon. Do NOT add points that are on the dividing line
			// since these were already added above
			if (inVertAxisDelta[inVertA] > 0)
			{
				rcVcopy(&outVerts1[poly1Vert * 3], &inVerts[inVertA * 3]);
				poly1Vert++;
			}
			else if (inVertAxisDelta[inVertA] < 0)
			{
				rcVcopy(&outVerts2[poly2Vert * 3], &inVerts[inVertA * 3]);
				poly2Vert++;
			}
		}
		else
		{
			// add the inVertA point to the right polygon. Addition is done even for points on the dividing line
			if (inVertAxisDelta[inVertA] >= 0)
			{
				rcVcopy(&outVerts1[poly1Vert * 3], &inVerts[inVertA * 3]);
				poly1Vert++;
				if (inVertAxisDelta[inVertA] != 0)
				{
					continue;
				}
			}
			rcVcopy(&outVerts2[poly2Vert * 3], &inVerts[inVertA * 3]);
			poly2Vert++;
		}
	}

	*outVerts1Count = poly1Vert;
	*outVerts2Count = poly2Vert;
}

/// @brief 将单个三角形光栅化到高度场中。
///
/// 这是整个光栅化流程中**最热（调用最频繁）的函数**，对性能极为敏感，
/// 因此在维护时应特别注意保持最高性能。
///
/// 核心算法思路（Sutherland-Hodgman 风格的逐行逐列裁剪）：
///   1. 计算三角形的 AABB，与高度场 AABB 做快速排斥测试
///   2. 确定三角形在 Z 轴方向上覆盖的网格行范围 [z0, z1]
///   3. 外层循环：沿 Z 轴逐行用 dividePoly() 裁剪多边形
///   4. 确定当前行多边形在 X 轴方向上覆盖的网格列范围 [x0, x1]
///   5. 内层循环：沿 X 轴逐列用 dividePoly() 裁剪多边形
///   6. 对裁剪到单个网格单元内的多边形碎片，计算 Y 轴（高度）范围
///   7. 量化为体素高度索引，调用 addSpan() 插入/合并到高度场的对应列
///
/// @param[in]     v0                   三角形的第 0 个顶点坐标 (x, y, z)
/// @param[in]     v1                   三角形的第 1 个顶点坐标 (x, y, z)
/// @param[in]     v2                   三角形的第 2 个顶点坐标 (x, y, z)
/// @param[in]     areaID               要赋给光栅化产生的 Span 的区域类型 ID。
///                                     RC_WALKABLE_AREA (63) = 可行走，RC_NULL_AREA (0) = 不可行走。
/// @param[in,out] heightfield          目标高度场，光栅化产生的 Span 将插入此高度场。
/// @param[in]     heightfieldBBMin     高度场包围盒的最小角 (x, y, z)
/// @param[in]     heightfieldBBMax     高度场包围盒的最大角 (x, y, z)
/// @param[in]     cellSize             XZ 平面上每个体素的尺寸（即 heightfield.cs）
/// @param[in]     inverseCellSize      1.0f / cellSize，预计算的倒数，用于避免除法
/// @param[in]     inverseCellHeight    1.0f / cellHeight，预计算的倒数，用于避免除法
/// @param[in]     flagMergeThreshold   Span 合并阈值：当两个重叠 Span 的最大高度差
///                                     ≤ 此值时，合并区域 ID（取较大值，即较高优先级）。
///
/// @return 操作成功返回 true；如果在 addSpan 中内存分配失败返回 false。
///
/// @see dividePoly, addSpan, overlapBounds
static bool rasterizeTri(const float* v0, const float* v1, const float* v2,
                         const unsigned char areaID, rcHeightfield& heightfield,
                         const float* heightfieldBBMin, const float* heightfieldBBMax,
                         const float cellSize, const float inverseCellSize, const float inverseCellHeight,
                         const int flagMergeThreshold)
{
	// ============================================================
	// 第一步：计算三角形的轴对齐包围盒（AABB）
	// ============================================================

	// 用三角形顶点 v0 初始化最小包围盒角，然后用 v1、v2 逐分量取最小值。
	float triBBMin[3];
	rcVcopy(triBBMin, v0);   // triBBMin = v0
	rcVmin(triBBMin, v1);    // triBBMin = min(triBBMin, v1) —— 逐分量取最小
	rcVmin(triBBMin, v2);    // triBBMin = min(triBBMin, v2)

	// 用三角形顶点 v0 初始化最大包围盒角，然后用 v1、v2 逐分量取最大值。
	float triBBMax[3];
	rcVcopy(triBBMax, v0);   // triBBMax = v0
	rcVmax(triBBMax, v1);    // triBBMax = max(triBBMax, v1) —— 逐分量取最大
	rcVmax(triBBMax, v2);    // triBBMax = max(triBBMax, v2)

	// ============================================================
	// 第二步：快速排斥测试 —— 三角形 AABB 是否与高度场 AABB 重叠
	// ============================================================
	// 如果两个包围盒在任意一个轴上不重叠，则三角形完全在高度场外部，直接跳过。
	// 返回 true（不是错误，只是该三角形不需要光栅化）。
	if (!overlapBounds(triBBMin, triBBMax, heightfieldBBMin, heightfieldBBMax))
	{
		return true;
	}

	// ============================================================
	// 第三步：获取高度场网格尺寸，计算 Z 轴覆盖范围
	// ============================================================

	const int w = heightfield.width;   // 高度场在 X 轴方向的网格列数
	const int h = heightfield.height;  // 高度场在 Z 轴方向的网格行数

	// by = 高度场在 Y 轴方向的总高度范围（世界坐标单位），
	// 后续用于将 Span 的 Y 范围钳制在高度场有效范围内。
	const float by = heightfieldBBMax[1] - heightfieldBBMin[1];

	// 将三角形 AABB 在 Z 轴上的最小/最大坐标转换为网格行索引。
	// 公式：zIndex = (worldZ - bbMinZ) * inverseCellSize
	// 使用 (int) 截断取整，得到三角形底部和顶部所在的行号。
	int z0 = (int)((triBBMin[2] - heightfieldBBMin[2]) * inverseCellSize);
	int z1 = (int)((triBBMax[2] - heightfieldBBMin[2]) * inverseCellSize);

	// 将行范围钳制到有效区间：
	// z0 允许为 -1 而非 0，这是为了在 tile（瓦片）起始边缘能正确裁剪多边形。
	// 当 z = -1 时，裁剪会在 z=0 的边界处正确切割，但不会真正写入 z=-1 的 Span。
	// z1 钳制到 [0, h-1]，确保不超出高度场的有效行范围。
	z0 = rcClamp(z0, -1, h - 1);
	z1 = rcClamp(z1, 0, h - 1);

	// ============================================================
	// 第四步：准备裁剪缓冲区
	// ============================================================
	// 分配栈上的临时多边形缓冲区。一个凸多边形经过一次裁剪，顶点数最多增加 1 个，
	// 三角形（3 个顶点）经过多次裁剪，最多可能产生 7 个顶点。
	// 每个顶点占 3 个 float (x, y, z)，共需要 4 组缓冲区。
	// 总大小：7 顶点 × 3 分量 × 4 缓冲区 = 84 个 float。
	float buf[7 * 3 * 4];
	float* in = buf;                // in:    当前待裁剪的输入多边形（初始为原始三角形）
	float* inRow = buf + 7 * 3;     // inRow: Z 轴裁剪后落在当前行内的多边形碎片
	float* p1 = inRow + 7 * 3;      // p1:    裁剪产生的"左侧"多边形（临时缓冲区）
	float* p2 = p1 + 7 * 3;         // p2:    裁剪产生的"右侧"多边形（临时缓冲区）

	// 将原始三角形的三个顶点复制到输入缓冲区 in 中。
	rcVcopy(&in[0], v0);       // in[0] = v0
	rcVcopy(&in[1 * 3], v1);   // in[1] = v1
	rcVcopy(&in[2 * 3], v2);   // in[2] = v2

	int nvRow;                  // 当前行裁剪后的多边形顶点数
	int nvIn = 3;               // 当前输入多边形的顶点数（初始为三角形的 3 个顶点）

	// ============================================================
	// 第五步：外层循环 —— 沿 Z 轴逐行裁剪
	// ============================================================
	// 遍历三角形覆盖的每一行（z0 到 z1）。
	// 每次迭代用 dividePoly 在 z 方向的网格线处将多边形一分为二：
	//   - inRow: 落在当前行（z 行）内的部分 → 继续沿 X 轴裁剪
	//   - p1/in: 剩余部分（z+1 以上） → 留给下一行继续裁剪
	for (int z = z0; z <= z1; ++z)
	{
		// 计算当前行的 Z 轴起始世界坐标。
		const float cellZ = heightfieldBBMin[2] + (float)z * cellSize;

		// 沿 Z 轴方向，在 cellZ + cellSize（即当前行的上边界）处裁剪多边形。
		// dividePoly 将 in 分为两部分：
		//   - inRow (nvRow 个顶点): Z ≤ cellZ+cellSize 的部分（当前行内的多边形）
		//   - p1    (nvIn 个顶点):  Z > cellZ+cellSize 的部分（剩余的、下一行的多边形）
		dividePoly(in, nvIn, inRow, &nvRow, p1, &nvIn, cellZ + cellSize, RC_AXIS_Z);

		// 交换 in 和 p1 指针：下一次迭代的输入变为本次裁剪的"剩余部分"。
		// 这避免了内存拷贝，只是交换指针，非常高效。
		rcSwap(in, p1);
		
		// 如果裁剪后当前行内的多边形顶点数 < 3，说明不构成有效多边形，跳过。
		if (nvRow < 3)
		{
			continue;
		}

		// z = -1 是辅助裁剪用的虚拟行，不应真正写入 Span，跳过。
		if (z < 0)
		{
			continue;
		}
		
		// ============================================================
		// 第六步：计算当前行多边形在 X 轴方向的覆盖范围
		// ============================================================
		// 遍历当前行多边形的所有顶点，找到 X 坐标的最小值和最大值。
		float minX = inRow[0];     // 初始化为第一个顶点的 X 坐标
		float maxX = inRow[0];
		for (int vert = 1; vert < nvRow; ++vert)
		{
			if (minX > inRow[vert * 3])     // inRow[vert*3] 是第 vert 个顶点的 X 分量
			{
				minX = inRow[vert * 3];
			}
			if (maxX < inRow[vert * 3])
			{
				maxX = inRow[vert * 3];
			}
		}

		// 将 X 范围转换为网格列索引。
		int x0 = (int)((minX - heightfieldBBMin[0]) * inverseCellSize);
		int x1 = (int)((maxX - heightfieldBBMin[0]) * inverseCellSize);

		// 如果 X 范围完全在高度场外部，跳过当前行。
		if (x1 < 0 || x0 >= w)
		{
			continue;
		}

		// 将列范围钳制到有效区间：
		// x0 允许为 -1（与 z0 同理，用于在瓦片起始边缘正确裁剪）。
		// x1 钳制到 [0, w-1]，确保不超出高度场的有效列范围。
		x0 = rcClamp(x0, -1, w - 1);
		x1 = rcClamp(x1, 0, w - 1);

		int nv;              // X 轴裁剪后落在当前列内的多边形顶点数
		int nv2 = nvRow;     // 当前待裁剪的行内多边形顶点数（初始为整行多边形）

		// ============================================================
		// 第七步：内层循环 —— 沿 X 轴逐列裁剪
		// ============================================================
		// 遍历当前行多边形覆盖的每一列（x0 到 x1）。
		// 每次迭代用 dividePoly 在 X 方向的网格线处将多边形一分为二：
		//   - p1:      落在当前列（x 列）内的部分 → 计算 Y 范围并生成 Span
		//   - p2/inRow: 剩余部分（x+1 以右） → 留给下一列继续裁剪
		for (int x = x0; x <= x1; ++x)
		{
			// 计算当前列的 X 轴起始世界坐标。
			const float cx = heightfieldBBMin[0] + (float)x * cellSize;

			// 沿 X 轴方向，在 cx + cellSize（当前列的右边界）处裁剪多边形。
			// dividePoly 将 inRow 分为两部分：
			//   - p1  (nv 个顶点):  X ≤ cx+cellSize 的部分（当前列内的多边形碎片）
			//   - p2  (nv2 个顶点): X > cx+cellSize 的部分（剩余的、下一列的多边形）
			dividePoly(inRow, nv2, p1, &nv, p2, &nv2, cx + cellSize, RC_AXIS_X);

			// 交换 inRow 和 p2 指针：下一次迭代的输入变为本次裁剪的"剩余部分"。
			rcSwap(inRow, p2);
			
			// 如果裁剪后当前列内的多边形顶点数 < 3，说明不构成有效多边形，跳过。
			if (nv < 3)
			{
				continue;
			}

			// x = -1 是辅助裁剪用的虚拟列，不应真正写入 Span，跳过。
			if (x < 0)
			{
				continue;
			}
			
			// ============================================================
			// 第八步：计算当前网格单元内多边形碎片的 Y（高度）范围
			// ============================================================
			// p1 中存储了裁剪到当前 (x, z) 网格单元内的多边形碎片。
			// 遍历其所有顶点，找到 Y 坐标的最小值（spanMin）和最大值（spanMax）。
			// p1[vert * 3 + 1] 是第 vert 个顶点的 Y 分量。
			float spanMin = p1[1];     // 用第一个顶点的 Y 初始化
			float spanMax = p1[1];
			for (int vert = 1; vert < nv; ++vert)
			{
				spanMin = rcMin(spanMin, p1[vert * 3 + 1]);  // 取所有顶点 Y 的最小值
				spanMax = rcMax(spanMax, p1[vert * 3 + 1]);  // 取所有顶点 Y 的最大值
			}

			// 将 Y 范围从世界坐标转换为相对于高度场底部的局部坐标。
			// 即 spanMin/spanMax 变为"距离高度场底部的高度"。
			spanMin -= heightfieldBBMin[1];
			spanMax -= heightfieldBBMin[1];
			
			// ============================================================
			// 第九步：边界检测与钳制
			// ============================================================

			// 如果 Span 的最大高度在高度场底部以下，说明完全在高度场外部，跳过。
			if (spanMax < 0.0f)
			{
				continue;
			}
			// 如果 Span 的最小高度在高度场顶部以上，说明完全在高度场外部，跳过。
			if (spanMin > by)
			{
				continue;
			}
			
			// 将 Span 的 Y 范围钳制到高度场包围盒内。
			// 如果 Span 底部低于高度场底部，截断为 0。
			if (spanMin < 0.0f)
			{
				spanMin = 0;
			}
			// 如果 Span 顶部高于高度场顶部，截断为 by（高度场总高度）。
			if (spanMax > by)
			{
				spanMax = by;
			}

			// ============================================================
			// 第十步：量化为体素高度索引，并插入到高度场
			// ============================================================

			// 将连续的 Y 范围 [spanMin, spanMax] 量化为离散的体素高度索引：
			//   - spanMinCellIndex: 向下取整（floor），确保 Span 底部不会被截断
			//   - spanMaxCellIndex: 向上取整（ceil），确保 Span 顶部不会被截断
			//   - spanMaxCellIndex 最小为 spanMinCellIndex + 1，保证 Span 至少有 1 个体素的厚度
			//   - 两者都钳制到 [0, RC_SPAN_MAX_HEIGHT] 的合法范围内
			unsigned short spanMinCellIndex = (unsigned short)rcClamp((int)floorf(spanMin * inverseCellHeight), 0, RC_SPAN_MAX_HEIGHT);
			unsigned short spanMaxCellIndex = (unsigned short)rcClamp((int)ceilf(spanMax * inverseCellHeight), (int)spanMinCellIndex + 1, RC_SPAN_MAX_HEIGHT);

			// 将量化后的 Span 插入到高度场的 (x, z) 列中。
			// addSpan 会处理与已有 Span 的重叠合并逻辑：
			//   - 如果新 Span 与已有 Span 重叠且高度差 ≤ flagMergeThreshold，
			//     则合并区域 ID（取较大值，即较高优先级的区域类型）。
			//   - 如果内存分配失败（Span 池耗尽），返回 false。
			if (!addSpan(heightfield, x, z, spanMinCellIndex, spanMaxCellIndex, areaID, flagMergeThreshold))
			{
				return false;
			}
		}
	}

	// 所有网格单元处理完毕，三角形光栅化成功。
	return true;
}

bool rcRasterizeTriangle(rcContext* context,
                         const float* v0, const float* v1, const float* v2,
                         const unsigned char areaID, rcHeightfield& heightfield, const int flagMergeThreshold)
{
	rcAssert(context != NULL);

	rcScopedTimer timer(context, RC_TIMER_RASTERIZE_TRIANGLES);

	// Rasterize the single triangle.
	const float inverseCellSize = 1.0f / heightfield.cs;
	const float inverseCellHeight = 1.0f / heightfield.ch;
	if (!rasterizeTri(v0, v1, v2, areaID, heightfield, heightfield.bmin, heightfield.bmax, heightfield.cs, inverseCellSize, inverseCellHeight, flagMergeThreshold))
	{
		context->log(RC_LOG_ERROR, "rcRasterizeTriangle: Out of memory.");
		return false;
	}

	return true;
}

/// @brief 将一组三角形光栅化到高度场中（使用索引数组 + int 类型索引版本）。
///
/// 这是导航网格构建流程中的**核心光栅化步骤**，负责将三维三角形网格转换为体素化的高度场表示。
/// 该函数遍历所有输入三角形，通过索引数组从共享顶点列表中查找顶点坐标，
/// 然后逐个调用 rasterizeTri() 将每个三角形"投影"到 XZ 平面的体素网格上，
/// 生成垂直方向的 Span（实心区间）并插入高度场的对应列中。
///
/// 典型调用流程：
///   1. rcMarkWalkableTriangles() — 预先标记可行走三角形 → triAreaIDs
///   2. rcRasterizeTriangles()    — 将三角形连同区域ID光栅化到高度场（本函数）
///   3. rcFilterXxx()             — 对高度场进行过滤处理
///
/// @param[in]     context             构建上下文，用于日志记录和性能计时。不能为 NULL。
/// @param[in]     verts               共享顶点坐标数组，连续存储 [(x, y, z) * nv]。
///                                    多个三角形可共享相同的顶点。
/// @param[in]     nv                  顶点数量（当前未使用，仅用于 API 完整性）。
/// @param[in]     tris                三角形索引数组（int 类型），每 3 个索引构成一个三角形
///                                    [(v0, v1, v2) * numTris]，每个值是 verts 中的顶点索引。
/// @param[in]     triAreaIDs          三角形区域 ID 数组 [大小: numTris]。
///                                    通常由 rcMarkWalkableTriangles() 生成。
///                                    RC_WALKABLE_AREA (63) 表示可行走，RC_NULL_AREA (0) 表示不可行走。
/// @param[in]     numTris             三角形数量。
/// @param[in,out] heightfield         目标高度场。光栅化产生的 Span 会被插入到此高度场中。
///                                    高度场必须已通过 rcCreateHeightfield() 初始化。
/// @param[in]     flagMergeThreshold  Span 合并阈值。当两个重叠 Span 的最大高度差
///                                    ≤ 此值时，取较高优先级的区域 ID（数值更大的 area）。
///                                    通常设为 walkableClimb 的体素数。
///
/// @return 成功返回 true；如果在添加 Span 时内存不足返回 false。
///
/// @see rasterizeTri, addSpan, rcMarkWalkableTriangles, rcCreateHeightfield
bool rcRasterizeTriangles(rcContext* context,
                          const float* verts, const int /*nv*/,
                          const int* tris, const unsigned char* triAreaIDs, const int numTris,
                          rcHeightfield& heightfield, const int flagMergeThreshold)
{
	// 断言上下文不为空，确保日志和计时功能可用。
	rcAssert(context != NULL);

	// 创建作用域计时器，在函数进入时自动调用 context->startTimer(RC_TIMER_RASTERIZE_TRIANGLES)，
	// 函数退出时（无论正常返回还是提前返回）自动调用 context->stopTimer()，
	// 用于统计三角形光栅化的总耗时。
	rcScopedTimer timer(context, RC_TIMER_RASTERIZE_TRIANGLES);
	
	// ---- 开始光栅化所有三角形 ----

	// 预计算体素尺寸的倒数，避免在热循环中重复执行除法运算。
	// inverseCellSize = 1.0 / cs，用于将世界坐标转换为 XZ 平面上的体素坐标。
	// 例如：voxelX = (worldX - bmin[0]) * inverseCellSize
	const float inverseCellSize = 1.0f / heightfield.cs;

	// inverseCellHeight = 1.0 / ch，用于将世界 Y 坐标转换为垂直方向的体素高度索引。
	// 例如：voxelY = (worldY - bmin[1]) * inverseCellHeight
	const float inverseCellHeight = 1.0f / heightfield.ch;

	// 遍历每一个三角形，逐个进行光栅化。
	for (int triIndex = 0; triIndex < numTris; ++triIndex)
	{
		// 通过索引数组获取第 triIndex 个三角形的三个顶点坐标指针。
		// tris[triIndex * 3 + 0] 是第一个顶点在 verts 数组中的索引号，
		// 乘以 3 是因为每个顶点占 3 个 float（x, y, z），
		// 最终得到指向该顶点 (x, y, z) 起始位置的指针。
		const float* v0 = &verts[tris[triIndex * 3 + 0] * 3];
		const float* v1 = &verts[tris[triIndex * 3 + 1] * 3];
		const float* v2 = &verts[tris[triIndex * 3 + 2] * 3];

		// 调用核心光栅化函数，将单个三角形转换为高度场中的 Span。
		// rasterizeTri 内部流程：
		//   1. 计算三角形的 AABB 包围盒，与高度场包围盒做快速排斥检测
		//   2. 沿 Z 轴逐行裁剪三角形多边形（dividePoly 按 Z 轴切割）
		//   3. 沿 X 轴逐列裁剪（dividePoly 按 X 轴切割）
		//   4. 对裁剪后的多边形计算 Y 轴范围 [spanMin, spanMax]
		//   5. 量化为体素高度索引 [spanMinCellIndex, spanMaxCellIndex]
		//   6. 调用 addSpan() 将 Span 插入/合并到高度场列中
		// triAreaIDs[triIndex] 携带该三角形的区域 ID（可行走/不可行走），
		// 会被写入到生成的 Span 的 area 字段中。
		if (!rasterizeTri(v0, v1, v2, triAreaIDs[triIndex], heightfield, heightfield.bmin, heightfield.bmax, heightfield.cs, inverseCellSize, inverseCellHeight, flagMergeThreshold))
		{
			// Span 分配失败（内存不足），记录错误日志并提前返回。
			context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
			return false;
		}
	}

	// 所有三角形光栅化完成，返回成功。
	return true;
}

bool rcRasterizeTriangles(rcContext* context,
                          const float* verts, const int /*nv*/,
                          const unsigned short* tris, const unsigned char* triAreaIDs, const int numTris,
                          rcHeightfield& heightfield, const int flagMergeThreshold)
{
	rcAssert(context != NULL);

	rcScopedTimer timer(context, RC_TIMER_RASTERIZE_TRIANGLES);

	// Rasterize the triangles.
	const float inverseCellSize = 1.0f / heightfield.cs;
	const float inverseCellHeight = 1.0f / heightfield.ch;
	for (int triIndex = 0; triIndex < numTris; ++triIndex)
	{
		const float* v0 = &verts[tris[triIndex * 3 + 0] * 3];
		const float* v1 = &verts[tris[triIndex * 3 + 1] * 3];
		const float* v2 = &verts[tris[triIndex * 3 + 2] * 3];
		if (!rasterizeTri(v0, v1, v2, triAreaIDs[triIndex], heightfield, heightfield.bmin, heightfield.bmax, heightfield.cs, inverseCellSize, inverseCellHeight, flagMergeThreshold))
		{
			context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
			return false;
		}
	}

	return true;
}

bool rcRasterizeTriangles(rcContext* context,
                          const float* verts, const unsigned char* triAreaIDs, const int numTris,
                          rcHeightfield& heightfield, const int flagMergeThreshold)
{
	rcAssert(context != NULL);

	rcScopedTimer timer(context, RC_TIMER_RASTERIZE_TRIANGLES);
	
	// Rasterize the triangles.
	const float inverseCellSize = 1.0f / heightfield.cs;
	const float inverseCellHeight = 1.0f / heightfield.ch;
	for (int triIndex = 0; triIndex < numTris; ++triIndex)
	{
		const float* v0 = &verts[(triIndex * 3 + 0) * 3];
		const float* v1 = &verts[(triIndex * 3 + 1) * 3];
		const float* v2 = &verts[(triIndex * 3 + 2) * 3];
		if (!rasterizeTri(v0, v1, v2, triAreaIDs[triIndex], heightfield, heightfield.bmin, heightfield.bmax, heightfield.cs, inverseCellSize, inverseCellHeight, flagMergeThreshold))
		{
			context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
			return false;
		}
	}

	return true;
}
