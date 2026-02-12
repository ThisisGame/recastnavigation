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

#include <string.h> // for memcpy and memset

/// Sorts the given data in-place using insertion sort.
///
/// @param	data		The data to sort
/// @param	dataLength	The number of elements in @p data
static void insertSort(unsigned char* data, const int dataLength)
{
	for (int valueIndex = 1; valueIndex < dataLength; valueIndex++)
	{
		const unsigned char value = data[valueIndex];
		int insertionIndex;
		for (insertionIndex = valueIndex - 1; insertionIndex >= 0 && data[insertionIndex] > value; insertionIndex--)
		{
			// Shift over values
			data[insertionIndex + 1] = data[insertionIndex];
		}
		
		// Insert the value in sorted order.
		data[insertionIndex + 1] = value;
	}
}

// TODO (graham): This is duplicated in the ConvexVolumeTool in RecastDemo
/// Checks if a point is contained within a polygon
///
/// @param[in]	numVerts	Number of vertices in the polygon
/// @param[in]	verts		The polygon vertices
/// @param[in]	point		The point to check
/// @returns true if the point lies within the polygon, false otherwise.
static bool pointInPoly(int numVerts, const float* verts, const float* point)
{
	bool inPoly = false;
	for (int i = 0, j = numVerts - 1; i < numVerts; j = i++)
	{
		const float* vi = &verts[i * 3];
		const float* vj = &verts[j * 3];

		if ((vi[2] > point[2]) == (vj[2] > point[2]))
		{
			continue;
		}

		if (point[0] >= (vj[0] - vi[0]) * (point[2] - vi[2]) / (vj[2] - vi[2]) + vi[0])
		{
			continue;
		}
		inPoly = !inPoly;
	}
	return inPoly;
}

/// @par
///
/// 按Agent半径腐蚀可行走区域，确保Agent中心不会过于靠近墙壁或障碍物边缘。
///
/// 算法原理：
///   使用两遍 Chamfer 距离变换（也叫倒角距离变换），计算每个可行走Span到最近
///   不可行走边界的近似距离。距离值采用整数编码：正交邻居代价=2，对角邻居代价=3。
///   因此距离值 d 对应的实际体素距离约为 d/2。
///   最后将距离小于 erosionRadius*2 的Span标记为不可行走。
///
/// 为什么要腐蚀？
///   导航网格描述的是Agent中心可到达的区域。如果Agent有一定半径(如0.6m)，
///   那么Agent中心不能贴着墙走，否则身体会穿透墙壁。腐蚀就是把可行走区域
///   从边界向内收缩一个Agent半径的距离，让最终的导航网格自然地远离障碍物。
///
/// 在构建流程中的位置：
///   rcBuildCompactHeightfield() → 【rcErodeWalkableArea()】 → rcMarkConvexPolyArea() → 区域划分
///
/// @param[in]     context              构建上下文，用于记录日志和性能计时。
/// @param[in]     erosionRadius        腐蚀半径，即Agent半径（以体素为单位）。
///                                     该值来自 rcConfig::walkableRadius。
/// @param[in,out] compactHeightfield   紧凑高度场。函数会将距离边界过近的Span的
///                                     area 标记为 RC_NULL_AREA。
/// @return 成功返回 true，内存分配失败返回 false。
bool rcErodeWalkableArea(rcContext* context, const int erosionRadius, rcCompactHeightfield& compactHeightfield)
{
	rcAssert(context != NULL);

	// 获取网格在X轴和Z轴上的尺寸（体素单位）
	const int xSize = compactHeightfield.width;
	const int zSize = compactHeightfield.height;
	const int& zStride = xSize; // Z方向步长，等于X方向的宽度，用于二维索引计算

	rcScopedTimer timer(context, RC_TIMER_ERODE_AREA);

	// =========================================================================
	// 第一步：分配距离场数组
	// =========================================================================
	// 为每个Span分配一个 unsigned char 存储到边界的距离值。
	// 初始化为 0xff (255)，表示"距离未知/无穷远"。
	unsigned char* distanceToBoundary = (unsigned char*)rcAlloc(sizeof(unsigned char) * compactHeightfield.spanCount,
	                                                            RC_ALLOC_TEMP);
	if (!distanceToBoundary)
	{
		context->log(RC_LOG_ERROR, "erodeWalkableArea: Out of memory 'dist' (%d).", compactHeightfield.spanCount);
		return false;
	}
	memset(distanceToBoundary, 0xff, sizeof(unsigned char) * compactHeightfield.spanCount);
	
	// =========================================================================
	// 第二步：标记边界Span（种子初始化）
	// =========================================================================
	// 将所有"边界"Span的距离设为 0。边界的定义：
	//   1. 自身是不可行走的（area == RC_NULL_AREA）
	//   2. 自身可行走，但四个正交方向中至少有一个邻居不存在连接或不可行走
	//
	// 这些距离为 0 的Span将作为后续距离传播的"种子点"。
	//
	//   种子初始化后的示意图（俯视图，B=边界(0), ?=待计算(255), ×=不可行走(0)）：
	//
	//     × × × × × × ×
	//     × B B B B B ×
	//     × B ? ? ? B ×
	//     × B ? ? ? B ×
	//     × B ? ? ? B ×
	//     × B B B B B ×
	//     × × × × × × ×
	//
	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			const rcCompactCell& cell = compactHeightfield.cells[x + z * zStride];
			for (int spanIndex = (int)cell.index, maxSpanIndex = (int)(cell.index + cell.count); spanIndex < maxSpanIndex; ++spanIndex)
			{
				// 不可行走的Span本身就是"障碍"，距离为 0
				if (compactHeightfield.areas[spanIndex] == RC_NULL_AREA)
				{
					distanceToBoundary[spanIndex] = 0;
					continue;
				}
				const rcCompactSpan& span = compactHeightfield.spans[spanIndex];

				// 检查四个正交方向（左、上、右、下）是否都连通且可行走。
				// 方向编号：0=(-1,0)左  1=(0,+1)上  2=(+1,0)右  3=(0,-1)下
				int neighborCount = 0;
				for (int direction = 0; direction < 4; ++direction)
				{
					const int neighborConnection = rcGetCon(span, direction);
					// 如果该方向没有连接（RC_NOT_CONNECTED=0x3f），说明是边界
					if (neighborConnection == RC_NOT_CONNECTED)
					{
						break;
					}
					
					// 计算邻居的网格坐标和Span索引
					const int neighborX = x + rcGetDirOffsetX(direction);
					const int neighborZ = z + rcGetDirOffsetY(direction);
					const int neighborSpanIndex = (int)compactHeightfield.cells[neighborX + neighborZ * zStride].index + neighborConnection;
					
					// 如果邻居是不可行走的，也说明当前Span紧挨边界
					if (compactHeightfield.areas[neighborSpanIndex] == RC_NULL_AREA)
					{
						break;
					}
					neighborCount++;
				}
				
				// 只要有一个方向缺少可行走的邻居，就是边界Span，距离设为 0
				if (neighborCount != 4)
				{
					distanceToBoundary[spanIndex] = 0;
				}
			}
		}
	}
	
	// =========================================================================
	// 第三步：两遍 Chamfer 距离变换（传播距离值）
	// =========================================================================
	//
	// Chamfer 距离变换使用3x3掩码来近似欧几里得距离。
	// 正交方向（上下左右）代价为 2，对角方向代价为 3。
	// 这样 d/2 就近似于实际的体素距离（对角 3/2 ≈ 1.5 ≈ √2 ≈ 1.414）。
	//
	// 两遍扫描的掩码分布：
	//
	//   Pass 1（从左上到右下扫描）：     Pass 2（从右下到左上扫描）：
	//   检查已扫描过的邻居               检查已扫描过的邻居
	//
	//     +3  +2  .                         .  +2  +3
	//     +2  cur .                         .  cur +2
	//      .   .  .                         .   .   .
	//
	//   其中 +2=正交代价, +3=对角代价, .=不检查
	//
	// 两遍扫描覆盖全部8个方向，合起来可以正确传播距离到所有Span。
	//
	unsigned char newDistance;
	
	// ---- Pass 1：从左上角 (0,0) 向右下角 (xSize-1, zSize-1) 扫描 ----
	// 只检查方向 0(-1,0)左 和方向 3(0,-1)下 及它们的对角组合
	// 即检查左、左下、下、右下 四个已扫描的邻居
	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			const rcCompactCell& cell = compactHeightfield.cells[x + z * zStride];
			const int maxSpanIndex = (int)(cell.index + cell.count);
			for (int spanIndex = (int)cell.index; spanIndex < maxSpanIndex; ++spanIndex)
			{
				const rcCompactSpan& span = compactHeightfield.spans[spanIndex];

				// 检查方向 0: (-1, 0) — 左邻居
				if (rcGetCon(span, 0) != RC_NOT_CONNECTED)
				{
					// (-1,0) 正交邻居，代价 +2
					const int aX = x + rcGetDirOffsetX(0);  // x - 1
					const int aY = z + rcGetDirOffsetY(0);  // z
					const int aIndex = (int)compactHeightfield.cells[aX + aY * xSize].index + rcGetCon(span, 0);
					const rcCompactSpan& aSpan = compactHeightfield.spans[aIndex];
					// 取左邻居的距离 + 2（正交代价），与当前距离取最小值
					newDistance = (unsigned char)rcMin((int)distanceToBoundary[aIndex] + 2, 255);
					if (newDistance < distanceToBoundary[spanIndex])
					{
						distanceToBoundary[spanIndex] = newDistance;
					}

					// (-1,-1) 左下对角邻居，代价 +3
					// 通过左邻居 aSpan 的方向 3(0,-1) 间接访问对角格子
					// 即先走到 (x-1, z)，再从那里走到 (x-1, z-1)
					if (rcGetCon(aSpan, 3) != RC_NOT_CONNECTED)
					{
						const int bX = aX + rcGetDirOffsetX(3);  // x - 1
						const int bY = aY + rcGetDirOffsetY(3);  // z - 1
						const int bIndex = (int)compactHeightfield.cells[bX + bY * xSize].index + rcGetCon(aSpan, 3);
						// 取对角邻居的距离 + 3（对角代价），与当前距离取最小值
						newDistance = (unsigned char)rcMin((int)distanceToBoundary[bIndex] + 3, 255);
						if (newDistance < distanceToBoundary[spanIndex])
						{
							distanceToBoundary[spanIndex] = newDistance;
						}
					}
				}
				// 检查方向 3: (0, -1) — 下邻居
				if (rcGetCon(span, 3) != RC_NOT_CONNECTED)
				{
					// (0,-1) 正交邻居，代价 +2
					const int aX = x + rcGetDirOffsetX(3);  // x
					const int aY = z + rcGetDirOffsetY(3);  // z - 1
					const int aIndex = (int)compactHeightfield.cells[aX + aY * xSize].index + rcGetCon(span, 3);
					const rcCompactSpan& aSpan = compactHeightfield.spans[aIndex];
					newDistance = (unsigned char)rcMin((int)distanceToBoundary[aIndex] + 2, 255);
					if (newDistance < distanceToBoundary[spanIndex])
					{
						distanceToBoundary[spanIndex] = newDistance;
					}

					// (1,-1) 右下对角邻居，代价 +3
					// 通过下邻居 aSpan 的方向 2(+1,0) 间接访问对角格子
					// 即先走到 (x, z-1)，再从那里走到 (x+1, z-1)
					if (rcGetCon(aSpan, 2) != RC_NOT_CONNECTED)
					{
						const int bX = aX + rcGetDirOffsetX(2);  // x + 1
						const int bY = aY + rcGetDirOffsetY(2);  // z - 1
						const int bIndex = (int)compactHeightfield.cells[bX + bY * xSize].index + rcGetCon(aSpan, 2);
						newDistance = (unsigned char)rcMin((int)distanceToBoundary[bIndex] + 3, 255);
						if (newDistance < distanceToBoundary[spanIndex])
						{
							distanceToBoundary[spanIndex] = newDistance;
						}
					}
				}
			}
		}
	}

	// ---- Pass 2：从右下角 (xSize-1, zSize-1) 向左上角 (0,0) 扫描 ----
	// 只检查方向 2(+1,0)右 和方向 1(0,+1)上 及它们的对角组合
	// 即检查右、右上、上、左上 四个已扫描的邻居
	for (int z = zSize - 1; z >= 0; --z)
	{
		for (int x = xSize - 1; x >= 0; --x)
		{
			const rcCompactCell& cell = compactHeightfield.cells[x + z * zStride];
			const int maxSpanIndex = (int)(cell.index + cell.count);
			for (int spanIndex = (int)cell.index; spanIndex < maxSpanIndex; ++spanIndex)
			{
				const rcCompactSpan& span = compactHeightfield.spans[spanIndex];

				// 检查方向 2: (+1, 0) — 右邻居
				if (rcGetCon(span, 2) != RC_NOT_CONNECTED)
				{
					// (1,0) 正交邻居，代价 +2
					const int aX = x + rcGetDirOffsetX(2);  // x + 1
					const int aY = z + rcGetDirOffsetY(2);  // z
					const int aIndex = (int)compactHeightfield.cells[aX + aY * xSize].index + rcGetCon(span, 2);
					const rcCompactSpan& aSpan = compactHeightfield.spans[aIndex];
					newDistance = (unsigned char)rcMin((int)distanceToBoundary[aIndex] + 2, 255);
					if (newDistance < distanceToBoundary[spanIndex])
					{
						distanceToBoundary[spanIndex] = newDistance;
					}

					// (1,1) 右上对角邻居，代价 +3
					// 通过右邻居 aSpan 的方向 1(0,+1) 间接访问对角格子
					// 即先走到 (x+1, z)，再从那里走到 (x+1, z+1)
					if (rcGetCon(aSpan, 1) != RC_NOT_CONNECTED)
					{
						const int bX = aX + rcGetDirOffsetX(1);  // x + 1
						const int bY = aY + rcGetDirOffsetY(1);  // z + 1
						const int bIndex = (int)compactHeightfield.cells[bX + bY * xSize].index + rcGetCon(aSpan, 1);
						newDistance = (unsigned char)rcMin((int)distanceToBoundary[bIndex] + 3, 255);
						if (newDistance < distanceToBoundary[spanIndex])
						{
							distanceToBoundary[spanIndex] = newDistance;
						}
					}
				}
				// 检查方向 1: (0, +1) — 上邻居
				if (rcGetCon(span, 1) != RC_NOT_CONNECTED)
				{
					// (0,1) 正交邻居，代价 +2
					const int aX = x + rcGetDirOffsetX(1);  // x
					const int aY = z + rcGetDirOffsetY(1);  // z + 1
					const int aIndex = (int)compactHeightfield.cells[aX + aY * xSize].index + rcGetCon(span, 1);
					const rcCompactSpan& aSpan = compactHeightfield.spans[aIndex];
					newDistance = (unsigned char)rcMin((int)distanceToBoundary[aIndex] + 2, 255);
					if (newDistance < distanceToBoundary[spanIndex])
					{
						distanceToBoundary[spanIndex] = newDistance;
					}

					// (-1,1) 左上对角邻居，代价 +3
					// 通过上邻居 aSpan 的方向 0(-1,0) 间接访问对角格子
					// 即先走到 (x, z+1)，再从那里走到 (x-1, z+1)
					if (rcGetCon(aSpan, 0) != RC_NOT_CONNECTED)
					{
						const int bX = aX + rcGetDirOffsetX(0);  // x - 1
						const int bY = aY + rcGetDirOffsetY(0);  // z + 1
						const int bIndex = (int)compactHeightfield.cells[bX + bY * xSize].index + rcGetCon(aSpan, 0);
						newDistance = (unsigned char)rcMin((int)distanceToBoundary[bIndex] + 3, 255);
						if (newDistance < distanceToBoundary[spanIndex])
						{
							distanceToBoundary[spanIndex] = newDistance;
						}
					}
				}
			}
		}
	}

	// =========================================================================
	// 第四步：根据距离场执行腐蚀
	// =========================================================================
	// 将距离值小于阈值的Span标记为不可行走。
	// 阈值 = erosionRadius * 2，因为距离编码中正交代价为 2（即1个体素=距离2）。
	// 例如：erosionRadius=3 → minBoundaryDistance=6
	//       某Span的距离=4（约2个体素远）< 6 → 标记为不可行走
	const unsigned char minBoundaryDistance = (unsigned char)(erosionRadius * 2);
	for (int spanIndex = 0; spanIndex < compactHeightfield.spanCount; ++spanIndex)
	{
		if (distanceToBoundary[spanIndex] < minBoundaryDistance)
		{
			compactHeightfield.areas[spanIndex] = RC_NULL_AREA;
		}
	}

	rcFree(distanceToBoundary);
	
	return true;
}

bool rcMedianFilterWalkableArea(rcContext* context, rcCompactHeightfield& compactHeightfield)
{
	rcAssert(context);
	
	const int xSize = compactHeightfield.width;
	const int zSize = compactHeightfield.height;
	const int zStride = xSize; // For readability

	rcScopedTimer timer(context, RC_TIMER_MEDIAN_AREA);

	unsigned char* areas = (unsigned char*)rcAlloc(sizeof(unsigned char) * compactHeightfield.spanCount, RC_ALLOC_TEMP);
	if (!areas)
	{
		context->log(RC_LOG_ERROR, "medianFilterWalkableArea: Out of memory 'areas' (%d).",
		             compactHeightfield.spanCount);
		return false;
	}
	memset(areas, 0xff, sizeof(unsigned char) * compactHeightfield.spanCount);

	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			const rcCompactCell& cell = compactHeightfield.cells[x + z * zStride];
			const int maxSpanIndex = (int)(cell.index + cell.count);
			for (int spanIndex = (int)cell.index; spanIndex < maxSpanIndex; ++spanIndex)
			{
				const rcCompactSpan& span = compactHeightfield.spans[spanIndex];
				if (compactHeightfield.areas[spanIndex] == RC_NULL_AREA)
				{
					areas[spanIndex] = compactHeightfield.areas[spanIndex];
					continue;
				}

				unsigned char neighborAreas[9];
				for (int neighborIndex = 0; neighborIndex < 9; ++neighborIndex)
				{
					neighborAreas[neighborIndex] = compactHeightfield.areas[spanIndex];
				}

				for (int dir = 0; dir < 4; ++dir)
				{
					if (rcGetCon(span, dir) == RC_NOT_CONNECTED)
					{
						continue;
					}
					
					const int aX = x + rcGetDirOffsetX(dir);
					const int aZ = z + rcGetDirOffsetY(dir);
					const int aIndex = (int)compactHeightfield.cells[aX + aZ * zStride].index + rcGetCon(span, dir);
					if (compactHeightfield.areas[aIndex] != RC_NULL_AREA)
					{
						neighborAreas[dir * 2 + 0] = compactHeightfield.areas[aIndex];
					}

					const rcCompactSpan& aSpan = compactHeightfield.spans[aIndex];
					const int dir2 = (dir + 1) & 0x3;
					const int neighborConnection2 = rcGetCon(aSpan, dir2);
					if (neighborConnection2 != RC_NOT_CONNECTED)
					{
						const int bX = aX + rcGetDirOffsetX(dir2);
						const int bZ = aZ + rcGetDirOffsetY(dir2);
						const int bIndex = (int)compactHeightfield.cells[bX + bZ * zStride].index + neighborConnection2;
						if (compactHeightfield.areas[bIndex] != RC_NULL_AREA)
						{
							neighborAreas[dir * 2 + 1] = compactHeightfield.areas[bIndex];
						}
					}
				}
				insertSort(neighborAreas, 9);
				areas[spanIndex] = neighborAreas[4];
			}
		}
	}

	memcpy(compactHeightfield.areas, areas, sizeof(unsigned char) * compactHeightfield.spanCount);

	rcFree(areas);

	return true;
}

void rcMarkBoxArea(rcContext* context, const float* boxMinBounds, const float* boxMaxBounds, unsigned char areaId,
                   rcCompactHeightfield& compactHeightfield)
{
	rcAssert(context);

	rcScopedTimer timer(context, RC_TIMER_MARK_BOX_AREA);

	const int xSize = compactHeightfield.width;
	const int zSize = compactHeightfield.height;
	const int zStride = xSize; // For readability

	// Find the footprint of the box area in grid cell coordinates. 
	int minX = (int)((boxMinBounds[0] - compactHeightfield.bmin[0]) / compactHeightfield.cs);
	int minY = (int)((boxMinBounds[1] - compactHeightfield.bmin[1]) / compactHeightfield.ch);
	int minZ = (int)((boxMinBounds[2] - compactHeightfield.bmin[2]) / compactHeightfield.cs);
	int maxX = (int)((boxMaxBounds[0] - compactHeightfield.bmin[0]) / compactHeightfield.cs);
	int maxY = (int)((boxMaxBounds[1] - compactHeightfield.bmin[1]) / compactHeightfield.ch);
	int maxZ = (int)((boxMaxBounds[2] - compactHeightfield.bmin[2]) / compactHeightfield.cs);

	// Early-out if the box is outside the bounds of the grid.
	if (maxX < 0) { return; }
	if (minX >= xSize) { return; }
	if (maxZ < 0) { return; }
	if (minZ >= zSize) { return; }

	// Clamp relevant bound coordinates to the grid.
	if (minX < 0) { minX = 0; }
	if (maxX >= xSize) { maxX = xSize - 1; }
	if (minZ < 0) { minZ = 0; }
	if (maxZ >= zSize) { maxZ = zSize - 1; }

	// Mark relevant cells.
	for (int z = minZ; z <= maxZ; ++z)
	{
		for (int x = minX; x <= maxX; ++x)
		{
			const rcCompactCell& cell = compactHeightfield.cells[x + z * zStride];
			const int maxSpanIndex = (int)(cell.index + cell.count);
			for (int spanIndex = (int)cell.index; spanIndex < maxSpanIndex; ++spanIndex)
			{
				rcCompactSpan& span = compactHeightfield.spans[spanIndex];

				// Skip if the span is outside the box extents.
				if ((int)span.y < minY || (int)span.y > maxY)
				{
					continue;
				}

				// Skip if the span has been removed.
				if (compactHeightfield.areas[spanIndex] == RC_NULL_AREA)
				{
					continue;
				}

				// Mark the span.
				compactHeightfield.areas[spanIndex] = areaId;
			}
		}
	}
}

void rcMarkConvexPolyArea(rcContext* context, const float* verts, const int numVerts,
						  const float minY, const float maxY, unsigned char areaId,
						  rcCompactHeightfield& compactHeightfield)
{
	rcAssert(context);

	rcScopedTimer timer(context, RC_TIMER_MARK_CONVEXPOLY_AREA);

	const int xSize = compactHeightfield.width;
	const int zSize = compactHeightfield.height;
	const int zStride = xSize; // For readability

	// Compute the bounding box of the polygon
	float bmin[3];
	float bmax[3];
	rcVcopy(bmin, verts);
	rcVcopy(bmax, verts);
	for (int i = 1; i < numVerts; ++i)
	{
		rcVmin(bmin, &verts[i * 3]);
		rcVmax(bmax, &verts[i * 3]);
	}
	bmin[1] = minY;
	bmax[1] = maxY;

	// Compute the grid footprint of the polygon 
	int minx = (int)((bmin[0] - compactHeightfield.bmin[0]) / compactHeightfield.cs);
	int miny = (int)((bmin[1] - compactHeightfield.bmin[1]) / compactHeightfield.ch);
	int minz = (int)((bmin[2] - compactHeightfield.bmin[2]) / compactHeightfield.cs);
	int maxx = (int)((bmax[0] - compactHeightfield.bmin[0]) / compactHeightfield.cs);
	int maxy = (int)((bmax[1] - compactHeightfield.bmin[1]) / compactHeightfield.ch);
	int maxz = (int)((bmax[2] - compactHeightfield.bmin[2]) / compactHeightfield.cs);

	// Early-out if the polygon lies entirely outside the grid.
	if (maxx < 0) { return; }
    if (minx >= xSize) { return; }
    if (maxz < 0) { return; }
    if (minz >= zSize) { return; }

	// Clamp the polygon footprint to the grid
    if (minx < 0) { minx = 0; }
    if (maxx >= xSize) { maxx = xSize - 1; }
    if (minz < 0) { minz = 0; }
    if (maxz >= zSize) { maxz = zSize - 1; }

	// TODO: Optimize.
	for (int z = minz; z <= maxz; ++z)
	{
		for (int x = minx; x <= maxx; ++x)
		{
			const rcCompactCell& cell = compactHeightfield.cells[x + z * zStride];
			const int maxSpanIndex = (int)(cell.index + cell.count);
			for (int spanIndex = (int)cell.index; spanIndex < maxSpanIndex; ++spanIndex)
			{
				rcCompactSpan& span = compactHeightfield.spans[spanIndex];

				// Skip if span is removed.
				if (compactHeightfield.areas[spanIndex] == RC_NULL_AREA)
				{
					continue;
				}

				// Skip if y extents don't overlap.
				if ((int)span.y < miny || (int)span.y > maxy)
				{
					continue;
				}

				const float point[] = {
					compactHeightfield.bmin[0] + ((float)x + 0.5f) * compactHeightfield.cs,
					0,
					compactHeightfield.bmin[2] + ((float)z + 0.5f) * compactHeightfield.cs
				};
				
				if (pointInPoly(numVerts, verts, point))
				{
					compactHeightfield.areas[spanIndex] = areaId;
				}
			}
		}
	}
}

static const float EPSILON = 1e-6f;

/// Normalizes the vector if the length is greater than zero.
/// If the magnitude is zero, the vector is unchanged.
/// @param[in,out]	v	The vector to normalize. [(x, y, z)]
static void rcVsafeNormalize(float* v)
{
	const float sqMag = rcSqr(v[0]) + rcSqr(v[1]) + rcSqr(v[2]);
	if (sqMag > EPSILON)
	{
		const float inverseMag = 1.0f / rcSqrt(sqMag);
		v[0] *= inverseMag;
		v[1] *= inverseMag;
		v[2] *= inverseMag;
	}
}

int rcOffsetPoly(const float* verts, const int numVerts, const float offset, float* outVerts, const int maxOutVerts)
{
	// Defines the limit at which a miter becomes a bevel.
	// Similar in behavior to https://developer.mozilla.org/en-US/docs/Web/SVG/Attribute/stroke-miterlimit
	const float MITER_LIMIT = 1.20f;

	int numOutVerts = 0;

	for (int vertIndex = 0; vertIndex < numVerts; vertIndex++)
	{
        // Grab three vertices of the polygon.
		const int vertIndexA = (vertIndex + numVerts - 1) % numVerts;
		const int vertIndexB = vertIndex;
		const int vertIndexC = (vertIndex + 1) % numVerts;
		const float* vertA = &verts[vertIndexA * 3];
		const float* vertB = &verts[vertIndexB * 3];
		const float* vertC = &verts[vertIndexC * 3];

        // From A to B on the x/z plane
		float prevSegmentDir[3];
		rcVsub(prevSegmentDir, vertB, vertA);
		prevSegmentDir[1] = 0; // Squash onto x/z plane
		rcVsafeNormalize(prevSegmentDir);
		
        // From B to C on the x/z plane
		float currSegmentDir[3];
		rcVsub(currSegmentDir, vertC, vertB);
		currSegmentDir[1] = 0; // Squash onto x/z plane
		rcVsafeNormalize(currSegmentDir);

        // The y component of the cross product of the two normalized segment directions.
        // The X and Z components of the cross product are both zero because the two
        // segment direction vectors fall within the x/z plane.
        float cross = currSegmentDir[0] * prevSegmentDir[2] - prevSegmentDir[0] * currSegmentDir[2];

        // CCW perpendicular vector to AB.  The segment normal.
		const float prevSegmentNormX = -prevSegmentDir[2];
		const float prevSegmentNormZ = prevSegmentDir[0];

        // CCW perpendicular vector to BC.  The segment normal.
		const float currSegmentNormX = -currSegmentDir[2];
		const float currSegmentNormZ = currSegmentDir[0];

        // Average the two segment normals to get the proportional miter offset for B.
        // This isn't normalized because it's defining the distance and direction the corner will need to be
        // adjusted proportionally to the edge offsets to properly miter the adjoining edges.
		float cornerMiterX = (prevSegmentNormX + currSegmentNormX) * 0.5f;
		float cornerMiterZ = (prevSegmentNormZ + currSegmentNormZ) * 0.5f;
        const float cornerMiterSqMag = rcSqr(cornerMiterX) + rcSqr(cornerMiterZ);

        // If the magnitude of the segment normal average is less than about .69444,
        // the corner is an acute enough angle that the result should be beveled.
        const bool bevel = cornerMiterSqMag * MITER_LIMIT * MITER_LIMIT < 1.0f;

        // Scale the corner miter so it's proportional to how much the corner should be offset compared to the edges.
		if (cornerMiterSqMag > EPSILON)
		{
			const float scale = 1.0f / cornerMiterSqMag;
            cornerMiterX *= scale;
            cornerMiterZ *= scale;
		}

		if (bevel && cross < 0.0f) // If the corner is convex and an acute enough angle, generate a bevel.
		{
			if (numOutVerts + 2 > maxOutVerts)
			{
				return 0;
			}

            // Generate two bevel vertices at a distances from B proportional to the angle between the two segments.
            // Move each bevel vertex out proportional to the given offset.
			float d = (1.0f - (prevSegmentDir[0] * currSegmentDir[0] + prevSegmentDir[2] * currSegmentDir[2])) * 0.5f;

			outVerts[numOutVerts * 3 + 0] = vertB[0] + (-prevSegmentNormX + prevSegmentDir[0] * d) * offset;
			outVerts[numOutVerts * 3 + 1] = vertB[1];
			outVerts[numOutVerts * 3 + 2] = vertB[2] + (-prevSegmentNormZ + prevSegmentDir[2] * d) * offset;
			numOutVerts++;

			outVerts[numOutVerts * 3 + 0] = vertB[0] + (-currSegmentNormX - currSegmentDir[0] * d) * offset;
			outVerts[numOutVerts * 3 + 1] = vertB[1];
			outVerts[numOutVerts * 3 + 2] = vertB[2] + (-currSegmentNormZ - currSegmentDir[2] * d) * offset;
			numOutVerts++;
		}
		else
		{
			if (numOutVerts + 1 > maxOutVerts)
			{
				return 0;
			}

            // Move B along the miter direction by the specified offset.
			outVerts[numOutVerts * 3 + 0] = vertB[0] - cornerMiterX * offset;
			outVerts[numOutVerts * 3 + 1] = vertB[1];
			outVerts[numOutVerts * 3 + 2] = vertB[2] - cornerMiterZ * offset;
			numOutVerts++;
		}
	}

	return numOutVerts;
}

void rcMarkCylinderArea(rcContext* context, const float* position, const float radius, const float height,
                        unsigned char areaId, rcCompactHeightfield& compactHeightfield)
{
	rcAssert(context);

	rcScopedTimer timer(context, RC_TIMER_MARK_CYLINDER_AREA);

	const int xSize = compactHeightfield.width;
	const int zSize = compactHeightfield.height;
	const int zStride = xSize; // For readability

	// Compute the bounding box of the cylinder
	const float cylinderBBMin[] =
	{
		position[0] - radius,
		position[1],
		position[2] - radius
	};
	const float cylinderBBMax[] =
	{
		position[0] + radius,
		position[1] + height,
		position[2] + radius
	};

	// Compute the grid footprint of the cylinder
	int minx = (int)((cylinderBBMin[0] - compactHeightfield.bmin[0]) / compactHeightfield.cs);
	int miny = (int)((cylinderBBMin[1] - compactHeightfield.bmin[1]) / compactHeightfield.ch);
	int minz = (int)((cylinderBBMin[2] - compactHeightfield.bmin[2]) / compactHeightfield.cs);
	int maxx = (int)((cylinderBBMax[0] - compactHeightfield.bmin[0]) / compactHeightfield.cs);
	int maxy = (int)((cylinderBBMax[1] - compactHeightfield.bmin[1]) / compactHeightfield.ch);
	int maxz = (int)((cylinderBBMax[2] - compactHeightfield.bmin[2]) / compactHeightfield.cs);

	// Early-out if the cylinder is completely outside the grid bounds.
    if (maxx < 0) { return; }
    if (minx >= xSize) { return; }
    if (maxz < 0) { return; }
    if (minz >= zSize) { return; }

	// Clamp the cylinder bounds to the grid.
    if (minx < 0) { minx = 0; }
    if (maxx >= xSize) { maxx = xSize - 1; }
    if (minz < 0) { minz = 0; }
    if (maxz >= zSize) { maxz = zSize - 1; }

	const float radiusSq = radius * radius;

	for (int z = minz; z <= maxz; ++z)
	{
		for (int x = minx; x <= maxx; ++x)
		{
			const rcCompactCell& cell = compactHeightfield.cells[x + z * zStride];
			const int maxSpanIndex = (int)(cell.index + cell.count);

			const float cellX = compactHeightfield.bmin[0] + ((float)x + 0.5f) * compactHeightfield.cs;
			const float cellZ = compactHeightfield.bmin[2] + ((float)z + 0.5f) * compactHeightfield.cs;
			const float deltaX = cellX - position[0];
            const float deltaZ = cellZ - position[2];

			// Skip this column if it's too far from the center point of the cylinder.
            if (rcSqr(deltaX) + rcSqr(deltaZ) >= radiusSq)
            {
	            continue;
            }

			// Mark all overlapping spans
			for (int spanIndex = (int)cell.index; spanIndex < maxSpanIndex; ++spanIndex)
			{
				rcCompactSpan& span = compactHeightfield.spans[spanIndex];

				// Skip if span is removed.
				if (compactHeightfield.areas[spanIndex] == RC_NULL_AREA)
				{
					continue;
				}

				// Mark if y extents overlap.
				if ((int)span.y >= miny && (int)span.y <= maxy)
				{
					compactHeightfield.areas[spanIndex] = areaId;
				}
			}
		}
	}
}
