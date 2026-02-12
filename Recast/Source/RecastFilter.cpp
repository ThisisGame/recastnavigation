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
#include "RecastAssert.h"

#include <stdlib.h>

namespace
{
	const int MAX_HEIGHTFIELD_HEIGHT = 0xffff; // TODO (graham): Move this to a more visible constant and update usages.
}

/// @brief 过滤低矮悬挂的可行走障碍物。
///
/// 该函数是导航网格构建流程中**三个高度场过滤步骤的第一步**，
/// 用于处理光栅化阶段因保守体素化而产生的"误标记为不可行走"的情况。
///
/// 核心思想：如果一个不可行走的 Span 紧挨在一个可行走 Span 的上方，
///          且两者顶面的高度差 ≤ walkableClimb（Agent 的最大可攀爬高度），
///          则认为该 Span 是 Agent 可以"跨过/踏上"的低矮障碍物（如台阶、
///          路缘石、门槛等），应将其标记为可行走。
///
/// 典型场景示意：
///
///   保守光栅化的问题               过滤后的修正
///   ┌─────────────┐                ┌─────────────┐
///   │  area = 0   │ ← 不可行走     │  area = 63  │ ← 修正为可行走
///   │  smax = 12  │                │  smax = 12  │
///   ├─────────────┤                ├─────────────┤
///   │  area = 63  │ ← 可行走       │  area = 63  │ ← 可行走
///   │  smax = 10  │                │  smax = 10  │
///   └─────────────┘                └─────────────┘
///   高度差 = 12 - 10 = 2           2 ≤ walkableClimb(2) → 修正！
///
/// 在构建流程中的位置：
///   rcRasterizeTriangles → [rcFilterLowHangingWalkableObstacles]
///                        → rcFilterLedgeSpans
///                        → rcFilterWalkableLowHeightSpans
///                        → rcBuildCompactHeightfield
///
/// @param[in]     context       构建上下文，用于日志和计时。不可为 NULL。
/// @param[in]     walkableClimb Agent 的最大可攀爬高度（体素单位）。
///                              两个相邻 Span 顶面的高度差若不超过此值，
///                              Agent 即可跨越。通常对应 rcConfig::walkableClimb。
/// @param[in,out] heightfield   要过滤的高度场。函数将修改其中 Span 的 area 字段。
///
/// @see rcFilterLedgeSpans, rcFilterWalkableLowHeightSpans
void rcFilterLowHangingWalkableObstacles(rcContext* context, const int walkableClimb, rcHeightfield& heightfield)
{
	// 断言上下文非空，确保日志和计时功能可用。
	rcAssert(context);

	// RAII 作用域计时器：自动记录本函数的执行耗时到 RC_TIMER_FILTER_LOW_OBSTACLES 类别。
	// 构造时调用 context->startTimer()，析构时自动调用 context->stopTimer()。
	rcScopedTimer timer(context, RC_TIMER_FILTER_LOW_OBSTACLES);

	// 获取高度场的网格尺寸：
	//   xSize = X 轴方向的体素列数（对应 heightfield.width）
	//   zSize = Z 轴方向的体素行数（对应 heightfield.height）
	const int xSize = heightfield.width;
	const int zSize = heightfield.height;

	// 遍历高度场中的每一列 (x, z)。
	// 高度场共有 xSize * zSize 列，每列包含一个按 smin 升序排列的 Span 链表。
	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			// previousSpan: 当前 Span 的前一个（下方的）Span 指针。
			//               用于计算两个 Span 之间的高度差。
			rcSpan* previousSpan = NULL;

			// previousWasWalkable: 记录前一个 Span 的**原始**可行走状态（进入循环体时的值）。
			//                      这是一个关键的设计点——使用的是修改前的原始值，
			//                      而非修改后的值，原因见下方注释。
			bool previousWasWalkable = false;

			// previousAreaID: 记录前一个 Span 的区域 ID（可能已被本函数修改过）。
			//                 当需要"传播"可行走属性时，用这个值来设置当前 Span 的区域。
			unsigned char previousAreaID = RC_NULL_AREA;

			// 从下到上遍历当前列中的所有 Span。
			// 链表按 smin 升序排列，即从最低的 Span 开始向上遍历。
			// 循环步进表达式 (previousSpan = span, span = span->next) 在每次迭代结束时
			// 将当前 Span 保存为 previousSpan，并移动到下一个更高的 Span。
			for (rcSpan* span = heightfield.spans[x + z * xSize]; span != NULL; previousSpan = span, span = span->next)
			{
				// 判断当前 Span 是否为可行走区域。
				// area != RC_NULL_AREA(0) 即为可行走（包括 RC_WALKABLE_AREA(63) 和自定义区域 1~62）。
				// 注意：这里取的是**修改前**的原始值。
				const bool walkable = span->area != RC_NULL_AREA;

				// 核心过滤逻辑：满足以下三个条件时，将当前 Span 标记为可行走：
				//   条件1: !walkable — 当前 Span 本身是不可行走的
				//   条件2: previousWasWalkable — 紧邻下方的 Span 原本是可行走的
				//   条件3: span->smax - previousSpan->smax <= walkableClimb
				//          — 当前 Span 顶面与下方 Span 顶面的高度差 ≤ 最大可攀爬高度
				//
				// 注意：比较的是两个 Span 的 smax（顶面高度），而非 smin。
				// 因为 smax 代表 Span 的"可站立表面"，两个相邻表面的高度差
				// 才是 Agent 实际需要攀爬的距离。
				//
				// 示意图（满足条件的情况）：
				//
				//   Y轴（高度）
				//    ↑   ┌──────────┐
				//    │   │ current  │ smax=12, area=0 (不可行走)
				//    │   │          │ ↕ 高度差 = 12 - 10 = 2
				//    │   ├──────────┤
				//    │   │ previous │ smax=10, area=63 (可行走)
				//    │   │          │
				//    │   └──────────┘
				//    │   如果 walkableClimb ≥ 2，则 current.area 被设置为 63
				//
				if (!walkable && previousWasWalkable && (int)span->smax - (int)previousSpan->smax <= walkableClimb)
				{
					// 将当前 Span 的区域 ID 设置为前一个 Span 的区域 ID，
					// 使其"继承"下方可行走 Span 的区域类型。
					// 注意使用 previousAreaID 而非固定的 RC_WALKABLE_AREA，
					// 这样可以正确传播自定义区域类型（如草地、道路等）。
					span->area = previousAreaID;
				}

				// 关键设计：记录当前 Span 的【原始】可行走状态，而非修改后的状态。
				//
				// 为什么使用原始值（walkable）而非修改后的 span->area？
				//   这是为了防止连续多个不可行走 Span 被错误地逐级向上传播。
				//
				// 反例（如果使用修改后的值会出现的问题）：
				//
				//   Y轴   ┌──────────┐
				//    ↑     │ Span C   │ smax=16, area=0   ← 如果用修改后的值，C 也会被标记可行走！
				//    │     ├──────────┤                      因为 B 刚刚被改为了可行走
				//    │     │ Span B   │ smax=14, area=0→63 ← 被修正为可行走（正确）
				//    │     ├──────────┤
				//    │     │ Span A   │ smax=10, area=63   ← 原始可行走
				//    │     └──────────┘
				//
				//   B-A 高度差 = 4（可能 ≤ walkableClimb）→ B 被正确修正
				//   C-B 高度差 = 2（≤ walkableClimb）→ 如果 previousWasWalkable 取修改后的值，
				//     C 也会被错误修正。但实际上 C 距离真正的可行走表面 A 已经相距 6 体素了！
				//
				// 使用原始值可以确保：只有"直接紧挨在可行走表面上方"的 Span 才会被修正，
				// 不会出现逐级向上的错误传播。
				previousWasWalkable = walkable;

				// 记录当前 Span 的区域 ID（已可能被上面的逻辑修改过）。
				// 这样在下次迭代时，如果需要"传播"区域类型，
				// 传播的是修改后的值（即正确的区域 ID）。
				previousAreaID = span->area;
			}
		}
	}
}

void rcFilterLedgeSpans(rcContext* context, const int walkableHeight, const int walkableClimb, rcHeightfield& heightfield)
{
	rcAssert(context);
	
	rcScopedTimer timer(context, RC_TIMER_FILTER_BORDER);

	const int xSize = heightfield.width;
	const int zSize = heightfield.height;
	
	// Mark spans that are adjacent to a ledge as unwalkable..
	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			for (rcSpan* span = heightfield.spans[x + z * xSize]; span; span = span->next)
			{
				// Skip non-walkable spans.
				if (span->area == RC_NULL_AREA)
				{
					continue;
				}

				const int floor = (int)(span->smax);
				const int ceiling = span->next ? (int)(span->next->smin) : MAX_HEIGHTFIELD_HEIGHT;

				// The difference between this walkable area and the lowest neighbor walkable area.
				// This is the difference between the current span and all neighbor spans that have
				// enough space for an agent to move between, but not accounting at all for surface slope.
				int lowestNeighborFloorDifference = MAX_HEIGHTFIELD_HEIGHT;

				// Min and max height of accessible neighbours.
				int lowestTraversableNeighborFloor = span->smax;
				int highestTraversableNeighborFloor = span->smax;

				for (int direction = 0; direction < 4; ++direction)
				{
					const int neighborX = x + rcGetDirOffsetX(direction);
					const int neighborZ = z + rcGetDirOffsetY(direction);

					// Skip neighbours which are out of bounds.
					if (neighborX < 0 || neighborZ < 0 || neighborX >= xSize || neighborZ >= zSize)
					{
						lowestNeighborFloorDifference = -walkableClimb - 1;
						break;
					}

					const rcSpan* neighborSpan = heightfield.spans[neighborX + neighborZ * xSize];

					// The most we can step down to the neighbor is the walkableClimb distance.
					// Start with the area under the neighbor span
					int neighborCeiling = neighborSpan ? (int)neighborSpan->smin : MAX_HEIGHTFIELD_HEIGHT;

					// Skip neighbour if the gap between the spans is too small.
					if (rcMin(ceiling, neighborCeiling) - floor >= walkableHeight)
					{
						lowestNeighborFloorDifference = (-walkableClimb - 1);
						break;
					}

					// For each span in the neighboring column...
					for (; neighborSpan != NULL; neighborSpan = neighborSpan->next)
					{
						const int neighborFloor = (int)neighborSpan->smax;
						neighborCeiling = neighborSpan->next ? (int)neighborSpan->next->smin : MAX_HEIGHTFIELD_HEIGHT;

						// Only consider neighboring areas that have enough overlap to be potentially traversable.
						if (rcMin(ceiling, neighborCeiling) - rcMax(floor, neighborFloor) < walkableHeight)
						{
							// No space to traverse between them.
							continue;
						}

						const int neighborFloorDifference = neighborFloor - floor;
						lowestNeighborFloorDifference = rcMin(lowestNeighborFloorDifference, neighborFloorDifference);

						// Find min/max accessible neighbor height.
						// Only consider neighbors that are at most walkableClimb away.
						if (rcAbs(neighborFloorDifference) <= walkableClimb)
						{
							// There is space to move to the neighbor cell and the slope isn't too much.
							lowestTraversableNeighborFloor = rcMin(lowestTraversableNeighborFloor, neighborFloor);
							highestTraversableNeighborFloor = rcMax(highestTraversableNeighborFloor, neighborFloor);
						}
						else if (neighborFloorDifference < -walkableClimb)
						{
							// We already know this will be considered a ledge span so we can early-out
							break;
						}
					}
				}

				// The current span is close to a ledge if the magnitude of the drop to any neighbour span is greater than the walkableClimb distance.
				// That is, there is a gap that is large enough to let an agent move between them, but the drop (surface slope) is too large to allow it.
				// (If this is the case, then biggestNeighborStepDown will be negative, so compare against the negative walkableClimb as a means of checking
				// the magnitude of the delta)
				if (lowestNeighborFloorDifference < -walkableClimb)
				{
					span->area = RC_NULL_AREA;
				}
				// If the difference between all neighbor floors is too large, this is a steep slope, so mark the span as an unwalkable ledge.
				else if (highestTraversableNeighborFloor - lowestTraversableNeighborFloor > walkableClimb)
				{
					span->area = RC_NULL_AREA;
				}
			}
		}
	}
}

void rcFilterWalkableLowHeightSpans(rcContext* context, const int walkableHeight, rcHeightfield& heightfield)
{
	rcAssert(context);
	rcScopedTimer timer(context, RC_TIMER_FILTER_WALKABLE);

	const int xSize = heightfield.width;
	const int zSize = heightfield.height;

	// Remove walkable flag from spans which do not have enough
	// space above them for the agent to stand there.
	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			for (rcSpan* span = heightfield.spans[x + z*xSize]; span; span = span->next)
			{
				const int floor = (int)(span->smax);
				const int ceiling = span->next ? (int)(span->next->smin) : MAX_HEIGHTFIELD_HEIGHT;
				if (ceiling - floor < walkableHeight)
				{
					span->area = RC_NULL_AREA;
				}
			}
		}
	}
}
