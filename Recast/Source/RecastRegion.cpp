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

#include <float.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"

namespace
{
/// 水位线栈条目：记录一个待处理 Span 的网格坐标和索引。
/// 在 Watershed 分区算法中，用于按距离场等级分层处理 Span。
struct LevelStackEntry
{
	LevelStackEntry(int x_, int y_, int index_) : x(x_), y(y_), index(index_) {}
	int x;      ///< Span 所在的 X 轴网格坐标
	int y;      ///< Span 所在的 Z 轴网格坐标（代码中用 y 表示 Z）
	int index;  ///< Span 在紧凑高度场 spans[] 数组中的索引，-1 表示已处理
};
}  // namespace

/// 计算距离场：为每个可行走 Span 计算到最近「区域边界」的 Chamfer 近似距离。
///
/// 与 rcErodeWalkableArea 中距离场的区别：
///   - rcErodeWalkableArea 中：边界 = 不可行走的 Span（用于腐蚀）
///   - 此处：边界 = 区域类型不同的相邻 Span（用于区域划分的水位线种子）
///
/// 距离编码：正交邻居代价 = 2，对角邻居代价 = 3（与 rcErodeWalkableArea 相同）。
/// 结果存入 src[] 数组，并通过 maxDist 返回最大距离值。
///
/// @param[in,out] chf       紧凑高度场
/// @param[out]    src       距离场输出数组 [大小: chf.spanCount]
/// @param[out]    maxDist   所有 Span 中的最大距离值
static void calculateDistanceField(rcCompactHeightfield& chf, unsigned short* src, unsigned short& maxDist)
{
	const int w = chf.width;
	const int h = chf.height;
	
	// =========================================================================
	// 第一步：初始化所有距离为 0xffff（无穷远）
	// =========================================================================
	for (int i = 0; i < chf.spanCount; ++i)
		src[i] = 0xffff;
	
	// =========================================================================
	// 第二步：标记边界种子（距离 = 0）
	// =========================================================================
	// 这里的「边界」定义与 rcErodeWalkableArea 不同：
	//   不是判断邻居是否可行走，而是判断邻居的区域类型(area)是否与自身相同。
	//   如果四个正交方向中有任一个邻居的 area 不同（或未连接），则视为边界。
	//
	// 目的：距离场描述的是「到不同区域边界的距离」，后续 Watershed 算法从
	//        距离最大（最中心）的 Span 开始向外扩展，实现从中心到边缘的分区。
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				const unsigned char area = chf.areas[i];
				
				// 统计四个正交方向中「区域相同」的邻居数量
				int nc = 0;
				for (int dir = 0; dir < 4; ++dir)
				{
					if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
					{
						const int ax = x + rcGetDirOffsetX(dir);
						const int ay = y + rcGetDirOffsetY(dir);
						const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, dir);
						// 注意：这里比较的是 area（区域类型），不是是否可行走
						if (area == chf.areas[ai])
							nc++;
					}
				}
				// 只要有一个方向的邻居区域不同（或未连接），就是边界，距离 = 0
				if (nc != 4)
					src[i] = 0;
			}
		}
	}
	
			
	// =========================================================================
	// 第三步：两遍 Chamfer 距离变换（与 rcErodeWalkableArea 完全相同的算法）
	// =========================================================================
	// 正交方向代价 = 2，对角方向代价 = 3
	// Pass 1：从左上角(0,0)扫描到右下角(w-1,h-1)
	// 检查方向 0(-1,0)左、方向 3(0,-1)下 及对角组合
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				
				// 检查左邻居 (-1,0)，正交代价 +2
				if (rcGetCon(s, 0) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirOffsetX(0);
					const int ay = y + rcGetDirOffsetY(0);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 0);
					const rcCompactSpan& as = chf.spans[ai];
					if (src[ai]+2 < src[i])
						src[i] = src[ai]+2;
					
					// 通过左邻居中转访问左下对角 (-1,-1)，对角代价 +3
					if (rcGetCon(as, 3) != RC_NOT_CONNECTED)
					{
						const int aax = ax + rcGetDirOffsetX(3);
						const int aay = ay + rcGetDirOffsetY(3);
						const int aai = (int)chf.cells[aax+aay*w].index + rcGetCon(as, 3);
						if (src[aai]+3 < src[i])
							src[i] = src[aai]+3;
					}
				}
				// 检查下邻居 (0,-1)，正交代价 +2
				if (rcGetCon(s, 3) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirOffsetX(3);
					const int ay = y + rcGetDirOffsetY(3);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 3);
					const rcCompactSpan& as = chf.spans[ai];
					if (src[ai]+2 < src[i])
						src[i] = src[ai]+2;
					
					// 通过下邻居中转访问右下对角 (1,-1)，对角代价 +3
					if (rcGetCon(as, 2) != RC_NOT_CONNECTED)
					{
						const int aax = ax + rcGetDirOffsetX(2);
						const int aay = ay + rcGetDirOffsetY(2);
						const int aai = (int)chf.cells[aax+aay*w].index + rcGetCon(as, 2);
						if (src[aai]+3 < src[i])
							src[i] = src[aai]+3;
					}
				}
			}
		}
	}
	
	// Pass 2：从右下角(w-1,h-1)扫描到左上角(0,0)
	// 检查方向 2(+1,0)右、方向 1(0,+1)上 及对角组合
	for (int y = h-1; y >= 0; --y)
	{
		for (int x = w-1; x >= 0; --x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				
				// 检查右邻居 (1,0)，正交代价 +2
				if (rcGetCon(s, 2) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirOffsetX(2);
					const int ay = y + rcGetDirOffsetY(2);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 2);
					const rcCompactSpan& as = chf.spans[ai];
					if (src[ai]+2 < src[i])
						src[i] = src[ai]+2;
					
					// 通过右邻居中转访问右上对角 (1,1)，对角代价 +3
					if (rcGetCon(as, 1) != RC_NOT_CONNECTED)
					{
						const int aax = ax + rcGetDirOffsetX(1);
						const int aay = ay + rcGetDirOffsetY(1);
						const int aai = (int)chf.cells[aax+aay*w].index + rcGetCon(as, 1);
						if (src[aai]+3 < src[i])
							src[i] = src[aai]+3;
					}
				}
				// 检查上邻居 (0,1)，正交代价 +2
				if (rcGetCon(s, 1) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirOffsetX(1);
					const int ay = y + rcGetDirOffsetY(1);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 1);
					const rcCompactSpan& as = chf.spans[ai];
					if (src[ai]+2 < src[i])
						src[i] = src[ai]+2;
					
					// 通过上邻居中转访问左上对角 (-1,1)，对角代价 +3
					if (rcGetCon(as, 0) != RC_NOT_CONNECTED)
					{
						const int aax = ax + rcGetDirOffsetX(0);
						const int aay = ay + rcGetDirOffsetY(0);
						const int aai = (int)chf.cells[aax+aay*w].index + rcGetCon(as, 0);
						if (src[aai]+3 < src[i])
							src[i] = src[aai]+3;
					}
				}
			}
		}
	}	
	
	// =========================================================================
	// 第四步：统计最大距离值
	// =========================================================================
	// maxDist 将用于 Watershed 算法的起始水位线
	maxDist = 0;
	for (int i = 0; i < chf.spanCount; ++i)
		maxDist = rcMax(src[i], maxDist);
	
}

/// 对距离场进行 3×3 盒式模糊（Box Blur），平滑距离值。
///
/// 目的：消除距离场中的噪声和尖锐变化，使得后续 Watershed 分区产生更平滑的区域边界。
///
/// 算法：对每个 Span，取自身 + 4个正交邻居 + 4个对角邻居（共9个）的距离值求平均。
///        对于不存在连接的邻居，用自身距离值替代（避免边界处偏移）。
///
/// @param[in]     chf   紧凑高度场
/// @param[in]     thr   模糊阈值：距离值 ≤ thr*2 的 Span 不做模糊（保护边界种子）
/// @param[in]     src   输入距离场
/// @param[out]    dst   输出距离场
/// @return 返回 dst 指针（便于调用方判断是否需要交换 src/dst）
static unsigned short* boxBlur(rcCompactHeightfield& chf, int thr,
							   unsigned short* src, unsigned short* dst)
{
	const int w = chf.width;
	const int h = chf.height;
	
	// 阈值乘以2，与距离编码对齐（距离编码中正交代价=2）
	thr *= 2;
	
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				const unsigned short cd = src[i]; // 当前 Span 的距离值
				
				// 距离值很小的 Span（接近边界）不做模糊，直接保留
				// 这样可以保护边界种子不被平滑掉
				if (cd <= thr)
				{
					dst[i] = cd;
					continue;
				}

				// 累加 3×3 邻域的距离值（自身 + 4正交 + 4对角 = 9个采样点）
				int d = (int)cd; // 从自身距离开始累加
				for (int dir = 0; dir < 4; ++dir)
				{
					if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
					{
						// 正交邻居存在：累加其距离值
						const int ax = x + rcGetDirOffsetX(dir);
						const int ay = y + rcGetDirOffsetY(dir);
						const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, dir);
						d += (int)src[ai];
						
						// 通过正交邻居中转，访问对角邻居 (dir 和 dir+1 的对角)
						const rcCompactSpan& as = chf.spans[ai];
						const int dir2 = (dir+1) & 0x3; // 下一个方向（顺时针旋转90°）
						if (rcGetCon(as, dir2) != RC_NOT_CONNECTED)
						{
							// 对角邻居存在：累加其距离值
							const int ax2 = ax + rcGetDirOffsetX(dir2);
							const int ay2 = ay + rcGetDirOffsetY(dir2);
							const int ai2 = (int)chf.cells[ax2+ay2*w].index + rcGetCon(as, dir2);
							d += (int)src[ai2];
						}
						else
						{
							// 对角邻居不存在：用自身距离代替
							d += cd;
						}
					}
					else
					{
						// 正交邻居不存在：正交 + 对角都用自身距离代替（共 +2 次）
						d += cd*2;
					}
				}
				// 9 个采样点求平均，+5 是为了四舍五入
				dst[i] = (unsigned short)((d+5)/9);
			}
		}
	}
	return dst;
}


/// 从种子点开始，洪水填充（Flood Fill）扩展一个区域。
///
/// 这是 Watershed 分区算法的核心函数。当一个新的区域种子点被确定后，调用此函数
/// 从该种子点开始向四周扩展，将所有满足条件的相邻 Span 标记为同一区域。
///
/// 扩展条件：
///   1. 邻居的 area（区域类型）与种子点相同
///   2. 邻居的距离值 ≥ level-2（在当前水位线及以上）
///   3. 邻居尚未被分配到其他区域（srcReg == 0）
///
/// 冲突检测（8连通）：
///   在扩展过程中，会检查每个 Span 的 8 连通邻居（4正交+4对角）。
///   如果发现某个邻居已经属于另一个区域（非当前区域 r，且非边界区域），
///   则当前 Span 会被撤销标记（srcReg 重置为 0），防止区域之间的冲突。
///
/// @param[in]     x, y, i   种子点的网格坐标和 Span 索引
/// @param[in]     level     当前水位线等级（距离场阈值）
/// @param[in]     r         要分配的区域 ID
/// @param[in]     chf       紧凑高度场
/// @param[in,out] srcReg    区域 ID 数组
/// @param[in,out] srcDist   区域内部距离数组
/// @param[in,out] stack     工作栈
/// @return 如果至少成功标记了一个 Span，返回 true
static bool floodRegion(int x, int y, int i,
						unsigned short level, unsigned short r,
						rcCompactHeightfield& chf,
						unsigned short* srcReg, unsigned short* srcDist,
						rcTempVector<LevelStackEntry>& stack)
{
	const int w = chf.width;
	
	const unsigned char area = chf.areas[i]; // 种子点的区域类型，扩展时只允许相同类型
	
	// 初始化洪水填充：清空栈，将种子点入栈并标记
	stack.clear();
	stack.push_back(LevelStackEntry(x, y, i));
	srcReg[i] = r;       // 标记种子点的区域 ID
	srcDist[i] = 0;      // 种子点的内部距离为 0
	
	// lev 是扩展时的距离阈值：只有 dist >= lev 的 Span 才能被加入
	// level >= 2 时取 level-2，否则取 0
	unsigned short lev = level >= 2 ? level-2 : 0;
	int count = 0;       // 成功标记的 Span 计数
	
	while (stack.size() > 0)
	{
		LevelStackEntry& back = stack.back();
		int cx = back.x;
		int cy = back.y;
		int ci = back.index;
		stack.pop_back();
		
		const rcCompactSpan& cs = chf.spans[ci];
		
		// ================================================================
		// 冲突检测：检查 8 连通邻居是否已有其他区域
		// ================================================================
		// 如果当前 Span 的任何 8 连通邻居已经属于另一个有效区域，
		// 则当前 Span 处于两个区域的交界处，需要撤销标记。
		unsigned short ar = 0; // 发现的冲突区域 ID
		for (int dir = 0; dir < 4; ++dir)
		{
			// 检查 4 正交方向 + 4 对角方向 = 8 连通
			if (rcGetCon(cs, dir) != RC_NOT_CONNECTED)
			{
				const int ax = cx + rcGetDirOffsetX(dir);
				const int ay = cy + rcGetDirOffsetY(dir);
				const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(cs, dir);
				// 跳过不同区域类型的邻居
				if (chf.areas[ai] != area)
					continue;
				unsigned short nr = srcReg[ai];
				if (nr & RC_BORDER_REG) // 忽略边界区域
					continue;
				if (nr != 0 && nr != r) // 发现属于其他区域的邻居
				{
					ar = nr;
					break;
				}
				
				// 继续检查对角邻居（通过正交邻居中转）
				const rcCompactSpan& as = chf.spans[ai];
				
				const int dir2 = (dir+1) & 0x3; // 顺时针旋转 90°
				if (rcGetCon(as, dir2) != RC_NOT_CONNECTED)
				{
					const int ax2 = ax + rcGetDirOffsetX(dir2);
					const int ay2 = ay + rcGetDirOffsetY(dir2);
					const int ai2 = (int)chf.cells[ax2+ay2*w].index + rcGetCon(as, dir2);
					if (chf.areas[ai2] != area)
						continue;
					unsigned short nr2 = srcReg[ai2];
					if (nr2 != 0 && nr2 != r) // 对角邻居属于其他区域
					{
						ar = nr2;
						break;
					}
				}				
			}
		}
		// 发现冲突：撤销当前 Span 的区域标记，继续处理栈中其他 Span
		if (ar != 0)
		{
			srcReg[ci] = 0;
			continue;
		}
		
		count++; // 当前 Span 成功保留
		
		// ================================================================
		// 向四个正交方向扩展
		// ================================================================
		for (int dir = 0; dir < 4; ++dir)
		{
			if (rcGetCon(cs, dir) != RC_NOT_CONNECTED)
			{
				const int ax = cx + rcGetDirOffsetX(dir);
				const int ay = cy + rcGetDirOffsetY(dir);
				const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(cs, dir);
				if (chf.areas[ai] != area)
					continue;
				// 只扩展到：距离 >= lev 且尚未被标记的 Span
				if (chf.dist[ai] >= lev && srcReg[ai] == 0)
				{
					srcReg[ai] = r;
					srcDist[ai] = 0;
					stack.push_back(LevelStackEntry(ax, ay, ai));
				}
			}
		}
	}
	
	return count > 0;
}

/// 脏条目：记录在 expandRegions 迭代过程中被修改的 Span 信息。
/// 用于批量同步更新，避免在遍历过程中直接修改数据导致的并发问题。
struct DirtyEntry
{
	DirtyEntry(int index_, unsigned short region_, unsigned short distance2_)
		: index(index_), region(region_), distance2(distance2_) {}
	int index;              ///< Span 在 spans[] 数组中的索引
	unsigned short region;  ///< 要分配的区域 ID
	unsigned short distance2; ///< 到最近已标记邻居的距离
};
/// 扩展已有区域：将未标记的 Span 分配给最近的已标记邻居所属的区域。
///
/// 这是 Watershed 算法中的「水位扩展」步骤。在 floodRegion 创建了初始区域种子后，
/// 本函数将这些区域向外扩展，填充尚未被标记的 Span。
///
/// 算法：
///   反复迭代栈中的待处理 Span：
///     - 对每个未标记的 Span，检查其 4 个正交邻居
///     - 如果某个邻居已经有有效区域且距离最小，则将当前 Span 分配给该区域
///     - 直到所有 Span 都被标记，或达到最大迭代次数
///
/// @param[in]     maxIter    最大迭代次数（level>0时生效，防止无限循环）
/// @param[in]     level      当前水位线等级
/// @param[in]     chf        紧凑高度场
/// @param[in,out] srcReg     区域 ID 数组
/// @param[in,out] srcDist    区域内部距离数组
/// @param[in,out] stack      待处理的 Span 列表
/// @param[in]     fillStack  是否需要重新填充栈（true=从全局扫描收集，false=使用传入的栈）
static void expandRegions(int maxIter, unsigned short level,
					      rcCompactHeightfield& chf,
					      unsigned short* srcReg, unsigned short* srcDist,
					      rcTempVector<LevelStackEntry>& stack,
					      bool fillStack)
{
	const int w = chf.width;
	const int h = chf.height;

	if (fillStack)
	{
		// 全局扫描模式：从整个高度场中收集满足条件的未标记 Span
		// 条件：dist >= level 且 srcReg == 0 且可行走
		stack.clear();
		for (int y = 0; y < h; ++y)
		{
			for (int x = 0; x < w; ++x)
			{
				const rcCompactCell& c = chf.cells[x+y*w];
				for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
				{
					if (chf.dist[i] >= level && srcReg[i] == 0 && chf.areas[i] != RC_NULL_AREA)
					{
						stack.push_back(LevelStackEntry(x, y, i));
					}
				}
			}
		}
	}
	else // 使用传入的栈（来自上一轮的未处理项）
	{
		// 标记栈中已经获得区域 ID 的条目为已处理（index = -1）
		for (int j=0; j<stack.size(); j++)
		{
			int i = stack[j].index;
			if (srcReg[i] != 0)
				stack[j].index = -1;
		}
	}

	rcTempVector<DirtyEntry> dirtyEntries;
	int iter = 0;
	while (stack.size() > 0)
	{
		int failed = 0; // 本轮未能成功分配区域的 Span 数量
		dirtyEntries.clear();
		
		for (int j = 0; j < stack.size(); j++)
		{
			int x = stack[j].x;
			int y = stack[j].y;
			int i = stack[j].index;
			if (i < 0) // 已处理的条目
			{
				failed++;
				continue;
			}
			
			// 在 4 个正交邻居中寻找距离最小的已标记邻居
			unsigned short r = srcReg[i];        // 当前区域（通常为 0）
			unsigned short d2 = 0xffff;           // 最小距离，初始为无穷大
			const unsigned char area = chf.areas[i];
			const rcCompactSpan& s = chf.spans[i];
			for (int dir = 0; dir < 4; ++dir)
			{
				if (rcGetCon(s, dir) == RC_NOT_CONNECTED) continue;
				const int ax = x + rcGetDirOffsetX(dir);
				const int ay = y + rcGetDirOffsetY(dir);
				const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, dir);
				if (chf.areas[ai] != area) continue;  // 不同区域类型不扩展
				// 邻居已标记且不是边界区域
				if (srcReg[ai] > 0 && (srcReg[ai] & RC_BORDER_REG) == 0)
				{
					// 选择距离最小的邻居
					if ((int)srcDist[ai]+2 < (int)d2)
					{
						r = srcReg[ai];
						d2 = srcDist[ai]+2;
					}
				}
			}
			if (r) // 找到了可分配的区域
			{
				stack[j].index = -1; // 标记为已处理
				// 记录到脏列表中，稍后批量更新
				dirtyEntries.push_back(DirtyEntry(i, r, d2));
			}
			else
			{
				failed++; // 所有邻居都未被标记，本轮无法处理
			}
		}
		
		// 批量将脏条目写入 srcReg 和 srcDist（避免遍历中的读写冲突）
		for (int i = 0; i < dirtyEntries.size(); i++) {
			int idx = dirtyEntries[i].index;
			srcReg[idx] = dirtyEntries[i].region;
			srcDist[idx] = dirtyEntries[i].distance2;
		}
		
		// 如果所有条目都失败了，说明没有更多可扩展的 Span
		if (failed == stack.size())
			break;
		
		// level > 0 时限制迭代次数（level == 0 的最终扩展阶段不限制）
		if (level > 0)
		{
			++iter;
			if (iter >= maxIter)
				break;
		}
	}
}



/// 按距离场等级将未标记的 Span 分桶到不同的栈中。
///
/// 在 Watershed 算法中，水位线每次降低 2，这意味着需要大量次数的水位线扫描。
/// 为了效率，一次性将多个等级的 Span 分桶到 NB_STACKS(=8) 个栈中，
/// 每个栈对应一个距离等级范围。这样就不需要每次降低水位线时都全局扫描。
///
/// @param[in]  startLevel          当前起始水位线等级
/// @param[in]  chf                 紧凑高度场
/// @param[in]  srcReg              区域 ID 数组
/// @param[in]  nbStacks            栈的数量
/// @param[out] stacks              输出栈数组
/// @param[in]  loglevelsPerStack   每个栈包含的等级数（以位移表示，实际为 1，即 2 个等级/栈）
static void sortCellsByLevel(unsigned short startLevel,
							  rcCompactHeightfield& chf,
							  const unsigned short* srcReg,
							  unsigned int nbStacks, rcTempVector<LevelStackEntry>* stacks,
							  unsigned short loglevelsPerStack) // the levels per stack (2 in our case) as a bit shift
{
	const int w = chf.width;
	const int h = chf.height;
	// 将 startLevel 右移，转换为桶索引空间
	startLevel = startLevel >> loglevelsPerStack;

	// 清空所有栈
	for (unsigned int j=0; j<nbStacks; ++j)
		stacks[j].clear();

	// 遍历所有 Span，按距离等级分配到对应的栈中
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				// 跳过不可行走的和已经有区域 ID 的 Span
				if (chf.areas[i] == RC_NULL_AREA || srcReg[i] != 0)
					continue;

				// 计算当前 Span 应该进入哪个栈
				// sId = startLevel - level，即距离越高的 Span 进入索引越小的栈
				int level = chf.dist[i] >> loglevelsPerStack;
				int sId = startLevel - level;
				if (sId >= (int)nbStacks) // 距离太低，不在当前批次的范围内
					continue;
				if (sId < 0) // 距离比当前起始级别更高，放入第一个栈
					sId = 0;

				stacks[sId].push_back(LevelStackEntry(x, y, i));
			}
		}
	}
}


/// 将源栈中仍未被标记的条目追加到目标栈。
/// 用于将上一级水位线中未处理的剩余项传递到下一级。
///
/// @param[in]  srcStack  源栈（上一级的剩余项）
/// @param[out] dstStack  目标栈（当前级别的处理栈）
/// @param[in]  srcReg    区域 ID 数组
static void appendStacks(const rcTempVector<LevelStackEntry>& srcStack,
						 rcTempVector<LevelStackEntry>& dstStack,
						 const unsigned short* srcReg)
{
	for (int j=0; j<srcStack.size(); j++)
	{
		int i = srcStack[j].index;
		// 跳过已处理(index<0)或已有区域的条目
		if ((i < 0) || (srcReg[i] != 0))
			continue;
		dstStack.push_back(srcStack[j]);
	}
}

/// 区域信息结构体：在区域合并、过滤和布局优化阶段使用。
///
/// 每个 rcRegion 对应一个区域 ID，存储该区域的拓扑信息：
///   - 链接关系（connections）：当前区域与哪些区域相邻（边共享）
///   - 堆叠关系（floors）：当前区域与哪些区域在同一列中垂直堆叠
struct rcRegion
{
	inline rcRegion(unsigned short i) :
		spanCount(0),
		id(i),
		areaType(0),
		remap(false),
		visited(false),
		overlap(false),
		connectsToBorder(false),
		ymin(0xffff),
		ymax(0)
	{}
	
	int spanCount;					///< 属于该区域的 Span 数量（区域面积）
	unsigned short id;				///< 区域 ID，合并后可能被重新映射
	unsigned char areaType;			///< 区域类型（对应 chf.areas[] 中的值）
	bool remap;						///< 是否需要重新映射 ID（压缩 ID 时使用）
	bool visited;					///< 是否已访问（遍历连通分量时用）
	bool overlap;					///< 是否与其他区域垂直重叠（同一列有相同区域 ID）
	bool connectsToBorder;			///< 是否连接到Tile边界
	unsigned short ymin, ymax;		///< 区域的垂直范围（Y轴最小/最大值）
	rcTempVector<int> connections;	///< 邻居区域 ID 列表（绕区域轮廓一周的有序邻居序列）
	rcTempVector<int> floors;		///< 堂区域 ID 列表（同一列中垂直堆叠的区域）
};

/// 移除区域邻居连接列表中的相邻重复项。
/// 连接列表是绕轮廓一周的有序序列，相邻位置的重复 ID 表示同一段边界，
/// 合并后只保留一个代表。
static void removeAdjacentNeighbours(rcRegion& reg)
{
	// Remove adjacent duplicates.
	for (int i = 0; i < reg.connections.size() && reg.connections.size() > 1; )
	{
		int ni = (i+1) % reg.connections.size();
		if (reg.connections[i] == reg.connections[ni])
		{
			// Remove duplicate
			for (int j = i; j < reg.connections.size()-1; ++j)
				reg.connections[j] = reg.connections[j+1];
			reg.connections.pop_back();
		}
		else
			++i;
	}
}

/// 将区域的邻居列表和地板列表中的旧 ID 替换为新 ID。
/// 在区域合并时，被合并区域的 ID 需要在所有关联区域中更新。
static void replaceNeighbour(rcRegion& reg, unsigned short oldId, unsigned short newId)
{
	bool neiChanged = false;
	for (int i = 0; i < reg.connections.size(); ++i)
	{
		if (reg.connections[i] == oldId)
		{
			reg.connections[i] = newId;
			neiChanged = true;
		}
	}
	for (int i = 0; i < reg.floors.size(); ++i)
	{
		if (reg.floors[i] == oldId)
			reg.floors[i] = newId;
	}
	if (neiChanged)
		removeAdjacentNeighbours(reg);
}

/// 检查两个区域是否可以合并。
///
/// 合并条件：
///   1. 两个区域的 areaType 必须相同（不同类型的区域不能合并）
///   2. 两个区域之间的边界必须是单连通的（只有一段相邻边）
///   3. 两个区域不能在垂直方向上重叠（floors 中不能包含对方）
static bool canMergeWithRegion(const rcRegion& rega, const rcRegion& regb)
{
	if (rega.areaType != regb.areaType)
		return false;
	int n = 0;
	for (int i = 0; i < rega.connections.size(); ++i)
	{
		if (rega.connections[i] == regb.id)
			n++;
	}
	if (n > 1)
		return false;
	for (int i = 0; i < rega.floors.size(); ++i)
	{
		if (rega.floors[i] == regb.id)
			return false;
	}
	return true;
}

/// 向区域的地板列表中添加一个不重复的地板区域 ID。
/// 地板列表记录同一列中垂直堆叠的其他区域。
static void addUniqueFloorRegion(rcRegion& reg, int n)
{
	for (int i = 0; i < reg.floors.size(); ++i)
		if (reg.floors[i] == n)
			return;
	reg.floors.push_back(n);
}

/// 合并两个区域：将 regb 合并到 rega 中。
///
/// 合并逻辑：
///   两个区域的轮廓邻居列表是环形的（绕轮廓一周）。
///   合并时，找到两个列表中彼此引用的位置，将它们的邻居序列拼接起来，
///   并移除两个区域之间的共享边。
///
///   示意图（环形列表）：
///     A 的邻居: [3, B, 5, 7]    B 的邻居: [2, A, 4, 6]
///     合并后 A: [5, 7, 3, 4, 6, 2]  （删除彼此的引用，拼接其余）
///
/// @param[in,out] rega  目标区域（合并结果存入此区域）
/// @param[in,out] regb  源区域（合并后被清空）
/// @return 合并成功返回 true
static bool mergeRegions(rcRegion& rega, rcRegion& regb)
{
	unsigned short aid = rega.id;
	unsigned short bid = regb.id;
	
	// Duplicate current neighbourhood.
	rcTempVector<int> acon;
	acon.resize(rega.connections.size());
	for (int i = 0; i < rega.connections.size(); ++i)
		acon[i] = rega.connections[i];
	rcTempVector<int>& bcon = regb.connections;
	
	// Find insertion point on A.
	int insa = -1;
	for (int i = 0; i < acon.size(); ++i)
	{
		if (acon[i] == bid)
		{
			insa = i;
			break;
		}
	}
	if (insa == -1)
		return false;
	
	// Find insertion point on B.
	int insb = -1;
	for (int i = 0; i < bcon.size(); ++i)
	{
		if (bcon[i] == aid)
		{
			insb = i;
			break;
		}
	}
	if (insb == -1)
		return false;
	
	// Merge neighbours.
	rega.connections.clear();
	for (int i = 0, ni = static_cast<int>(acon.size()); i < ni-1; ++i)
	{
		rega.connections.push_back(acon[(insa+1+i) % ni]);
	}
		
	for (int i = 0, ni = static_cast<int>(bcon.size()); i < ni-1; ++i)
	{
		rega.connections.push_back(bcon[(insb+1+i) % ni]);
	}
	
	removeAdjacentNeighbours(rega);
	
	for (int j = 0; j < regb.floors.size(); ++j)
		addUniqueFloorRegion(rega, regb.floors[j]);
	rega.spanCount += regb.spanCount;
	regb.spanCount = 0;
	regb.connections.resize(0);

	return true;
}

/// 检查区域是否连接到 Tile 边界。
/// 如果区域的邻居列表中包含 ID=0（空区域/外部），则该区域接触 Tile 边界。
/// 连接到边界的区域在过滤小区域时不会被删除，因为它们的实际大小无法仅从当前 Tile 判断。
static bool isRegionConnectedToBorder(const rcRegion& reg)
{
	// Region is connected to border if
	// one of the neighbours is null id.
	for (int i = 0; i < reg.connections.size(); ++i)
	{
		if (reg.connections[i] == 0)
			return true;
	}
	return false;
}

/// 检查某个 Span 在指定方向上是否是「实体边」（区域边界）。
/// 「实体边」定义：该方向的邻居不存在或属于不同区域。
/// 用于 walkContour 中判断轮廓边缘。
///
/// @return true 表示该方向是区域边界
static bool isSolidEdge(rcCompactHeightfield& chf, const unsigned short* srcReg,
						int x, int y, int i, int dir)
{
	const rcCompactSpan& s = chf.spans[i];
	unsigned short r = 0;
	if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
	{
		const int ax = x + rcGetDirOffsetX(dir);
		const int ay = y + rcGetDirOffsetY(dir);
		const int ai = (int)chf.cells[ax+ay*chf.width].index + rcGetCon(s, dir);
		r = srcReg[ai];
	}
	if (r == srcReg[i])
		return false;
	return true;
}

/// 沿着区域轮廓行走，收集所有相邻区域的 ID。
///
/// 算法：类似「右手法则」的轮廓追踪：
///   1. 当前方向是区域边界（实体边）→ 记录邻居区域 ID，顺时针旋转 90°
///   2. 当前方向不是边界（可通行）→ 前进到邻居 Span，逆时针旋转 90°
///   3. 回到起点时停止
///
/// 结果是一个环形序列，例如 [3, 0, 5, 0, 2] 表示区域轮廓依次
/// 与区域 3、0（外部）、5、0、2 相邻。
///
/// @param[in]  x, y, i  起始 Span 的坐标和索引
/// @param[in]  dir      起始方向（必须是一个实体边方向）
/// @param[in]  chf      紧凑高度场
/// @param[in]  srcReg   区域 ID 数组
/// @param[out] cont     输出的邻居区域 ID 环形序列
static void walkContour(int x, int y, int i, int dir,
						rcCompactHeightfield& chf,
						const unsigned short* srcReg,
						rcTempVector<int>& cont)
{
	int startDir = dir;
	int starti = i;

	// 获取起始方向的邻居区域 ID
	const rcCompactSpan& ss = chf.spans[i];
	unsigned short curReg = 0;
	if (rcGetCon(ss, dir) != RC_NOT_CONNECTED)
	{
		const int ax = x + rcGetDirOffsetX(dir);
		const int ay = y + rcGetDirOffsetY(dir);
		const int ai = (int)chf.cells[ax+ay*chf.width].index + rcGetCon(ss, dir);
		curReg = srcReg[ai];
	}
	cont.push_back(curReg); // 记录第一个邻居区域
			
	int iter = 0;
	while (++iter < 40000) // 安全防护：防止无限循环
	{
		const rcCompactSpan& s = chf.spans[i];
		
		if (isSolidEdge(chf, srcReg, x, y, i, dir))
		{
			// 当前方向是边界：记录邻居区域，然后顺时针旋转 90°
			unsigned short r = 0;
			if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
			{
				const int ax = x + rcGetDirOffsetX(dir);
				const int ay = y + rcGetDirOffsetY(dir);
				const int ai = (int)chf.cells[ax+ay*chf.width].index + rcGetCon(s, dir);
				r = srcReg[ai];
			}
			if (r != curReg) // 邻居区域发生变化，记录新的邻居
			{
				curReg = r;
				cont.push_back(curReg);
			}
			
			dir = (dir+1) & 0x3;  // 顺时针旋转 90°（CW）
		}
		else
		{
			// 当前方向不是边界：前进到邻居 Span，然后逆时针旋转 90°
			int ni = -1;
			const int nx = x + rcGetDirOffsetX(dir);
			const int ny = y + rcGetDirOffsetY(dir);
			if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
			{
				const rcCompactCell& nc = chf.cells[nx+ny*chf.width];
				ni = (int)nc.index + rcGetCon(s, dir);
			}
			if (ni == -1)
			{
				// 不应该发生（isSolidEdge 已经确认了连接存在）
				return;
			}
			x = nx;
			y = ny;
			i = ni;
			dir = (dir+3) & 0x3;	// 逆时针旋转 90°（CCW）
		}
		
		// 回到起点，轮廓闭合
		if (starti == i && startDir == dir)
		{
			break;
		}
	}

	// 移除相邻重复的邻居 ID（同一段边界只保留一次）
	if (cont.size() > 1)
	{
		for (int j = 0; j < cont.size(); )
		{
			int nj = (j+1) % cont.size();
			if (cont[j] == cont[nj])
			{
				for (int k = j; k < cont.size()-1; ++k)
					cont[k] = cont[k+1];
				cont.pop_back();
			}
			else
				++j;
		}
	}
}


/// 合并和过滤区域：Watershed 和 Monotone 分区方法的后处理步骤。
///
/// 本函数执行四个关键操作：
///   1. 构建区域拓扑：遍历所有 Span，统计每个区域的大小、地板和轮廓邻居
///   2. 删除小区域：将总面积 < minRegionArea 且未连接到Tile边界的
///      连通分量全部清除（设为 ID=0）
///   3. 合并小区域：将面积 < mergeRegionSize 的区域合并到最小的邻居区域
///   4. 压缩区域 ID：重新编号使ID连续（从 1 开始）
///
/// @param[in]     ctx              构建上下文
/// @param[in]     minRegionArea    最小区域面积阈值（小于此值的连通分量被删除）
/// @param[in]     mergeRegionSize  合并区域阈值（小于此值的区域尝试合并到邻居）
/// @param[in,out] maxRegionId      输入为当前最大 ID，输出为压缩后的最大 ID
/// @param[in]     chf              紧凑高度场
/// @param[in,out] srcReg           区域 ID 数组，会被更新为合并/压缩后的新 ID
/// @param[out]    overlaps         输出发现的重叠区域 ID 列表
/// @return 成功返回 true
static bool mergeAndFilterRegions(rcContext* ctx, int minRegionArea, int mergeRegionSize,
								  unsigned short& maxRegionId,
								  rcCompactHeightfield& chf,
								  unsigned short* srcReg, rcTempVector<int>& overlaps)
{
	const int w = chf.width;
	const int h = chf.height;
	
	const int nreg = maxRegionId+1;
	rcTempVector<rcRegion> regions;
	if (!regions.reserve(nreg)) {
		ctx->log(RC_LOG_ERROR, "mergeAndFilterRegions: Out of memory 'regions' (%d).", nreg);
		return false;
	}

	// 构造区域对象数组，每个区域用其索引作为初始 ID
	for (int i = 0; i < nreg; ++i)
		regions.push_back(rcRegion((unsigned short) i));
	
	// =========================================================================
	// 第一阶段：构建区域拓扑信息
	// =========================================================================
	// 遍历所有 Span，收集每个区域的：
	//   - spanCount（区域大小）
	//   - floors（垂直堆叠的区域）
	//   - connections（轮廓邻居序列）
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				unsigned short r = srcReg[i];
				if (r == 0 || r >= nreg)
					continue;
				
				rcRegion& reg = regions[r];
				reg.spanCount++;
				
				// 更新地板关系：同一列中的其他区域是「地板」关系
				// 如果同一列中有两个相同区域 ID 的 Span，标记为 overlap
				for (int j = (int)c.index; j < ni; ++j)
				{
					if (i == j) continue;
					unsigned short floorId = srcReg[j];
					if (floorId == 0 || floorId >= nreg)
						continue;
					if (floorId == r)
						reg.overlap = true;
					addUniqueFloorRegion(reg, floorId);
				}
				
				// 已找到该区域的轮廓，跳过（每个区域只需走一次轮廓）
				if (reg.connections.size() > 0)
					continue;
				
				reg.areaType = chf.areas[i];
				
				// 找到该 Span 的一个边界方向，作为轮廓行走的起点
				int ndir = -1;
				for (int dir = 0; dir < 4; ++dir)
				{
					if (isSolidEdge(chf, srcReg, x, y, i, dir))
					{
						ndir = dir;
						break;
					}
				}
				
				if (ndir != -1)
				{
					// 该 Span 在区域边界上，沿轮廓行走收集所有邻居
					walkContour(x, y, i, ndir, chf, srcReg, reg.connections);
				}
			}
		}
	}

	// =========================================================================
	// 第二阶段：删除过小的区域
	// =========================================================================
	// 用 DFS 遍历连通分量，统计总面积。
	// 如果连通分量的总面积 < minRegionArea 且未连接到 Tile 边界，
	// 则将该连通分量中的所有区域全部清除。
	rcTempVector<int> stack(32);
	rcTempVector<int> trace(32);
	for (int i = 0; i < nreg; ++i)
	{
		rcRegion& reg = regions[i];
		if (reg.id == 0 || (reg.id & RC_BORDER_REG))
			continue;                       
		if (reg.spanCount == 0)
			continue;
		if (reg.visited)
			continue;
		
		// 统计连通分量的总面积，并跟踪是否连接到 Tile 边界
		bool connectsToBorder = false;
		int spanCount = 0;
		stack.clear();
		trace.clear();

		reg.visited = true;
		stack.push_back(i);
		
		while (stack.size())
		{
			// Pop
			int ri = stack.back(); stack.pop_back();
			
			rcRegion& creg = regions[ri];

			spanCount += creg.spanCount;
			trace.push_back(ri);

			for (int j = 0; j < creg.connections.size(); ++j)
			{
				if (creg.connections[j] & RC_BORDER_REG)
				{
					connectsToBorder = true;
					continue;
				}
				rcRegion& neireg = regions[creg.connections[j]];
				if (neireg.visited)
					continue;
				if (neireg.id == 0 || (neireg.id & RC_BORDER_REG))
					continue;
				// Visit
				stack.push_back(neireg.id);
				neireg.visited = true;
			}
		}
		
		// 总面积过小且未连接到边界：清除该连通分量的所有区域
		// 注意：连接到边界的区域不能删除，因为它们可能在相邻 Tile 中更大
		if (spanCount < minRegionArea && !connectsToBorder)
		{
			// 清除所有已访问的区域
			for (int j = 0; j < trace.size(); ++j)
			{
				regions[trace[j]].spanCount = 0;
				regions[trace[j]].id = 0;
			}
		}
	}
	
	// =========================================================================
	// 第三阶段：合并小区域到邻居
	// =========================================================================
	// 反复扫描，将面积 < mergeRegionSize 的区域合并到最小的可合并邻居。
	// 终止条件：某一轮扫描中没有发生任何合并。
	int mergeCount = 0 ;
	do
	{
		mergeCount = 0;
		for (int i = 0; i < nreg; ++i)
		{
			rcRegion& reg = regions[i];
			if (reg.id == 0 || (reg.id & RC_BORDER_REG))
				continue;
			if (reg.overlap)
				continue;
			if (reg.spanCount == 0)
				continue;
			
			// 跳过足够大且连接到边界的区域
			if (reg.spanCount > mergeRegionSize && isRegionConnectedToBorder(reg))
				continue;
			
			// 找到最小的可合并邻居区域
			int smallest = 0xfffffff;
			unsigned short mergeId = reg.id;
			for (int j = 0; j < reg.connections.size(); ++j)
			{
				if (reg.connections[j] & RC_BORDER_REG) continue;
				rcRegion& mreg = regions[reg.connections[j]];
				if (mreg.id == 0 || (mreg.id & RC_BORDER_REG) || mreg.overlap) continue;
				if (mreg.spanCount < smallest &&
					canMergeWithRegion(reg, mreg) &&
					canMergeWithRegion(mreg, reg))
				{
					smallest = mreg.spanCount;
					mergeId = mreg.id;
				}
			}
			// 找到了可合并的目标
			if (mergeId != reg.id)
			{
				unsigned short oldId = reg.id;
				rcRegion& target = regions[mergeId];
				
				// 执行合并：将当前区域合并到目标区域
				if (mergeRegions(target, reg))
				{
					// 修复所有引用旧 ID 的区域
					for (int j = 0; j < nreg; ++j)
					{
						if (regions[j].id == 0 || (regions[j].id & RC_BORDER_REG)) continue;
						// 之前已合并到 oldId 的区域，继续跟随合并到 mergeId
						if (regions[j].id == oldId)
							regions[j].id = mergeId;
						// 在所有邻居列表中替换旧 ID
						replaceNeighbour(regions[j], oldId, mergeId);
					}
					mergeCount++;
				}
			}
		}
	}
	while (mergeCount > 0);
	
	// =========================================================================
	// 第四阶段：压缩区域 ID，使其连续
	// =========================================================================
	for (int i = 0; i < nreg; ++i)
	{
		regions[i].remap = false;
		if (regions[i].id == 0) continue;       // 跳过空区域
		if (regions[i].id & RC_BORDER_REG) continue;    // 跳过边界区域
		regions[i].remap = true;
	}
	
	unsigned short regIdGen = 0;
	for (int i = 0; i < nreg; ++i)
	{
		if (!regions[i].remap)
			continue;
		unsigned short oldId = regions[i].id;
		unsigned short newId = ++regIdGen;
		for (int j = i; j < nreg; ++j)
		{
			if (regions[j].id == oldId)
			{
				regions[j].id = newId;
				regions[j].remap = false;
			}
		}
	}
	maxRegionId = regIdGen;
	
	// 将新的区域 ID 写回 srcReg 数组
	for (int i = 0; i < chf.spanCount; ++i)
	{
		if ((srcReg[i] & RC_BORDER_REG) == 0)
			srcReg[i] = regions[srcReg[i]].id;
	}

	// 收集重叠区域 ID，返回给调用方作警告
	for (int i = 0; i < nreg; ++i)
		if (regions[i].overlap)
			overlaps.push_back(regions[i].id);

	return true;
}


/// 向区域的邻居连接列表中添加一个不重复的邻居 ID。
/// 与 walkContour 产生的有序环形序列不同，此函数生成的是无序的去重集合。
/// 用于 mergeAndFilterLayerRegions 中的层级区域处理。
static void addUniqueConnection(rcRegion& reg, int n)
{
	for (int i = 0; i < reg.connections.size(); ++i)
		if (reg.connections[i] == n)
			return;
	reg.connections.push_back(n);
}

/// 合并和过滤层级区域：用于 rcBuildLayerRegions 的后处理。
///
/// 与 mergeAndFilterRegions 的区别：
///   - mergeAndFilterRegions：用于 Watershed/Monotone，处理非重叠区域
///   - mergeAndFilterLayerRegions：用于 Layer，处理可能重叠的分层区域
///
/// 本函数执行三个主要操作：
///   1. 构建区域拓扑（邻居、地板、垂直范围等）
///   2. 用 BFS 将相邻且不重叠的区域合并为同一层（layerId）
///   3. 删除小区域并压缩 ID
///
/// @param[in]     ctx           构建上下文
/// @param[in]     minRegionArea 最小区域面积阈值
/// @param[in,out] maxRegionId   输入/输出最大区域 ID
/// @param[in]     chf           紧凑高度场
/// @param[in,out] srcReg        区域 ID 数组
/// @return 成功返回 true
static bool mergeAndFilterLayerRegions(rcContext* ctx, int minRegionArea,
									   unsigned short& maxRegionId,
									   rcCompactHeightfield& chf,
									   unsigned short* srcReg)
{
	const int w = chf.width;
	const int h = chf.height;
	
	const int nreg = maxRegionId+1;
	rcTempVector<rcRegion> regions;
	
	// 构造区域对象
	if (!regions.reserve(nreg)) {
		ctx->log(RC_LOG_ERROR, "mergeAndFilterLayerRegions: Out of memory 'regions' (%d).", nreg);
		return false;
	}
	for (int i = 0; i < nreg; ++i)
		regions.push_back(rcRegion((unsigned short) i));
	
	// =========================================================================
	// 第一阶段：构建区域拓扑（邻居、地板、垂直范围）
	// =========================================================================
	rcTempVector<int> lregs(32); // 当前列中的区域 ID 临时列表
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];

			lregs.clear();
			
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				const unsigned char area = chf.areas[i];
				const unsigned short ri = srcReg[i];
				if (ri == 0 || ri >= nreg) continue;
				rcRegion& reg = regions[ri];
				
				reg.spanCount++;
				reg.areaType = area;

				reg.ymin = rcMin(reg.ymin, s.y);
				reg.ymax = rcMax(reg.ymax, s.y);
				
				// Collect all region layers.
				lregs.push_back(ri);
				
				// Update neighbours
				for (int dir = 0; dir < 4; ++dir)
				{
					if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
					{
						const int ax = x + rcGetDirOffsetX(dir);
						const int ay = y + rcGetDirOffsetY(dir);
						const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, dir);
						const unsigned short rai = srcReg[ai];
						if (rai > 0 && rai < nreg && rai != ri)
							addUniqueConnection(reg, rai);
						if (rai & RC_BORDER_REG)
							reg.connectsToBorder = true;
					}
				}
				
			}
			
			// Update overlapping regions.
			// 同一列中的不同区域互为「地板」关系（垂直重叠）
			for (int i = 0; i < lregs.size()-1; ++i)
			{
				for (int j = i+1; j < lregs.size(); ++j)
				{
					if (lregs[i] != lregs[j])
					{
						rcRegion& ri = regions[lregs[i]];
						rcRegion& rj = regions[lregs[j]];
						addUniqueFloorRegion(ri, lregs[j]);
						addUniqueFloorRegion(rj, lregs[i]);
					}
				}
			}
			
		}
	}

	// =========================================================================
	// 第二阶段：创建 2D 层（将不重叠的相邻区域合并为同一层）
	// =========================================================================
	// 用 BFS 从每个未访问的区域开始，将相邻且不重叠的区域合并为同一个 layerId。
	// 重叠检查：如果邻居区域在 root 的 floors 中，说明它们在同一列中垂直堆叠，
	//          不能合并为同一层（否则会产生非平面的区域）。
	unsigned short layerId = 1;

	// 重置所有区域 ID，准备用 layerId 重新分配
	for (int i = 0; i < nreg; ++i)
		regions[i].id = 0;

	// 用 BFS 合并单调区域为不重叠的层
	rcTempVector<int> stack(32);
	for (int i = 1; i < nreg; ++i)
	{
		rcRegion& root = regions[i];
		// Skip already visited.
		if (root.id != 0)
			continue;
		
		// Start search.
		root.id = layerId;

		stack.clear();
		stack.push_back(i);
		
		while (stack.size() > 0)
		{
			// Pop front
			rcRegion& reg = regions[stack[0]];
			for (int j = 0; j < stack.size()-1; ++j)
				stack[j] = stack[j+1];
			stack.resize(stack.size()-1);
			
			const int ncons = (int)reg.connections.size();
			for (int j = 0; j < ncons; ++j)
			{
				const int nei = reg.connections[j];
				rcRegion& regn = regions[nei];
				// Skip already visited.
				if (regn.id != 0)
					continue;
				// 跳过不同区域类型的邻居
				if (reg.areaType != regn.areaType)
					continue;
				// 跳过与 root 垂直重叠的邻居（不能合并到同一层）
				bool overlap = false;
				for (int k = 0; k < root.floors.size(); k++)
				{
					if (root.floors[k] == nei)
					{
						overlap = true;
						break;
					}
				}
				if (overlap)
					continue;
					
				// 将邻居合并到当前层
				stack.push_back(nei);
					
				// 标记层 ID
				regn.id = layerId;
				// 将邻居的地板信息合并到 root（维护全局重叠检测）
				for (int k = 0; k < regn.floors.size(); ++k)
					addUniqueFloorRegion(root, regn.floors[k]);
				root.ymin = rcMin(root.ymin, regn.ymin);
				root.ymax = rcMax(root.ymax, regn.ymax);
				root.spanCount += regn.spanCount;
				regn.spanCount = 0;
				root.connectsToBorder = root.connectsToBorder || regn.connectsToBorder;
			}
		}
		
		layerId++;
	}
	
	// =========================================================================
	// 第三阶段：删除小区域
	// =========================================================================
	for (int i = 0; i < nreg; ++i)
	{
		if (regions[i].spanCount > 0 && regions[i].spanCount < minRegionArea && !regions[i].connectsToBorder)
		{
			unsigned short reg = regions[i].id;
			for (int j = 0; j < nreg; ++j)
				if (regions[j].id == reg)
					regions[j].id = 0;
		}
	}
	
	// =========================================================================
	// 第四阶段：压缩区域 ID
	// =========================================================================
	for (int i = 0; i < nreg; ++i)
	{
		regions[i].remap = false;
		if (regions[i].id == 0) continue;				// 跳过空区域
		if (regions[i].id & RC_BORDER_REG) continue;    // 跳过边界区域
		regions[i].remap = true;
	}
	
	unsigned short regIdGen = 0;
	for (int i = 0; i < nreg; ++i)
	{
		if (!regions[i].remap)
			continue;
		unsigned short oldId = regions[i].id;
		unsigned short newId = ++regIdGen;
		for (int j = i; j < nreg; ++j)
		{
			if (regions[j].id == oldId)
			{
				regions[j].id = newId;
				regions[j].remap = false;
			}
		}
	}
	maxRegionId = regIdGen;
	
	// 将新的区域 ID 写回 srcReg 数组
	for (int i = 0; i < chf.spanCount; ++i)
	{
		if ((srcReg[i] & RC_BORDER_REG) == 0)
			srcReg[i] = regions[srcReg[i]].id;
	}
	
	return true;
}



/// @par
/// 
/// 这通常是构建完整紧凑高度场的倒数第二步。此步骤在使用
/// #rcBuildRegions 或 #rcBuildRegionsMonotone 构建区域之前是必需的。
/// 
/// 此步骤完成后，距离数据可通过 rcCompactHeightfield::maxDistance
/// 和 rcCompactHeightfield::dist 字段访问。
///
/// 算法流程：
///   1. calculateDistanceField() 计算原始距离场
///   2. boxBlur() 对距离场进行 3x3 模糊平滑
///   3. 将结果存入 chf.dist 和 chf.maxDistance
///
/// @see rcCompactHeightfield, rcBuildRegions, rcBuildRegionsMonotone
bool rcBuildDistanceField(rcContext* ctx, rcCompactHeightfield& chf)
{
	// 断言上下文指针有效
	rcAssert(ctx);
	
	// 启动距离场构建的性能计时器（用于 profiling 统计总耗时）
	rcScopedTimer timer(ctx, RC_TIMER_BUILD_DISTANCEFIELD);
	
	// 如果紧凑高度场中已存在旧的距离场数据，先释放它
	// 这允许对同一个 chf 多次调用此函数而不会内存泄漏
	if (chf.dist)
	{
		rcFree(chf.dist);  // 释放旧的距离场内存
		chf.dist = 0;      // 置空指针，防止悬空引用
	}
	
	// 分配 src 缓冲区：每个 Span 一个 unsigned short，存储距离值
	// RC_ALLOC_TEMP 表示临时分配（最终会被 chf.dist 接管或释放）
	unsigned short* src = (unsigned short*)rcAlloc(sizeof(unsigned short)*chf.spanCount, RC_ALLOC_TEMP);
	if (!src)
	{
		// 内存分配失败：记录错误日志并返回
		ctx->log(RC_LOG_ERROR, "rcBuildDistanceField: Out of memory 'src' (%d).", chf.spanCount);
		return false;
	}
	// 分配 dst 缓冲区：boxBlur 需要双缓冲（读 src 写 dst 或反之）
	unsigned short* dst = (unsigned short*)rcAlloc(sizeof(unsigned short)*chf.spanCount, RC_ALLOC_TEMP);
	if (!dst)
	{
		// dst 分配失败：先释放已分配的 src，再返回错误
		ctx->log(RC_LOG_ERROR, "rcBuildDistanceField: Out of memory 'dst' (%d).", chf.spanCount);
		rcFree(src);
		return false;
	}
	
	// maxDist 用于记录整个距离场中的最大距离值
	unsigned short maxDist = 0;

	// =========================================================================
	// 第一阶段：计算原始距离场
	// =========================================================================
	{
		// 启动距离场计算的子计时器
		rcScopedTimer timerDist(ctx, RC_TIMER_BUILD_DISTANCEFIELD_DIST);

		// 使用两遍 Chamfer 距离变换计算每个 Span 到最近区域边界的近似距离
		// 结果存入 src 数组，最大距离值存入 maxDist
		// 算法：
		//   第一遍（左→右，上→下）：传播正交邻居代价 2，对角邻居代价 3
		//   第二遍（右→左，下→上）：反向传播，取更小值
		calculateDistanceField(chf, src, maxDist);
		
		// 将最大距离值存入紧凑高度场
		// Watershed 算法会用此值作为起始水位线，从最高点逐级降水
		chf.maxDistance = maxDist;
	}

	// =========================================================================
	// 第二阶段：模糊平滑距离场
	// =========================================================================
	{
		// 启动模糊阶段的子计时器
		rcScopedTimer timerBlur(ctx, RC_TIMER_BUILD_DISTANCEFIELD_BLUR);

		// 对距离场执行一次 3×3 盒式模糊（Box Blur），平滑噪声和突变
		// 参数 thr=1 是阈值保护：距离值 ≤ thr*2（即 ≤ 2）的 Span 被视为边界种子，
		// 不参与模糊计算（保护边界精度）。
		//
		// boxBlur 内部实现：读取 src，将结果写入 dst，然后返回结果所在的指针。
		// 如果输入已经足够平滑（无需模糊），可能直接返回 src 本身。
		// 因此需要检查返回值：如果结果在 dst 中，则交换 src 和 dst 指针，
		// 确保 src 始终指向最终结果。
		if (boxBlur(chf, 1, src, dst) != src)
			rcSwap(src, dst);

		// 将 src 指针赋值给 chf.dist，由紧凑高度场接管该内存的所有权
		// 之后 chf 析构时会负责释放 chf.dist
		// 后续步骤可通过 chf.dist[spanIndex] 访问每个 Span 的距离值
		chf.dist = src;
	}
	
	// 释放 dst 缓冲区（src 已被 chf.dist 接管，不能释放）
	rcFree(dst);
	
	// 距离场构建完成
	return true;
}

/// 将指定矩形范围内的所有可行走 Span 标记为指定的区域 ID。
/// 用于标记 Tile 边界区域（regId 带有 RC_BORDER_REG 标志）。
///
/// @param[in]  minx, maxx  X 轴范围 [minx, maxx)
/// @param[in]  miny, maxy  Z 轴范围 [miny, maxy)
/// @param[in]  regId       要分配的区域 ID（通常带有 RC_BORDER_REG 标志）
/// @param[in]  chf         紧凑高度场
/// @param[out] srcReg      区域 ID 数组
static void paintRectRegion(int minx, int maxx, int miny, int maxy, unsigned short regId,
							rcCompactHeightfield& chf, unsigned short* srcReg)
{
	const int w = chf.width;	
	for (int y = miny; y < maxy; ++y)
	{
		for (int x = minx; x < maxx; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				if (chf.areas[i] != RC_NULL_AREA)
					srcReg[i] = regId;
			}
		}
	}
}


static const unsigned short RC_NULL_NEI = 0xffff; ///< 无效邻居 ID（表示当前行扫描中没有找到一致的邻居）

/// 行扫描辅助结构体：用于 Monotone/Layer 分区的行扫描算法。
///
/// Monotone 和 Layer 分区使用“行扫描”方法：逐行（沿 Z 轴）扫描，
/// 每一行内的 Span 先尝试与左邻居（-X）合并，再尝试与上一行（-Z）合并。
struct rcSweepSpan
{
	unsigned short rid;	///< 行内临时 ID（同一行内通过-X邻居连接的 Span 共享相同 rid）
	unsigned short id;	///< 最终分配的全局区域 ID
	unsigned short ns;	///< 与上一行邻居的匹配采样数
	unsigned short nei;	///< 上一行的候选邻居区域 ID（RC_NULL_NEI 表示多个不同邻居，无法合并）
};

/// @par
/// 
/// 非空区域将由连通的、非重叠的可行走 Span 组成，形成单一轮廓。
/// 轮廓将形成简单多边形。
/// 
/// 单调分区算法：
///   沿 Z 轴逐行扫描，每一行内沿 X 轴逐格扫描。
///   每个 Span 先尝试与左邻居(-X)合并，再尝试与上一行(-Z)合并。
///   如果某个行内区域的所有上行采样都指向同一个区域，则合并；
///   否则分配新的区域 ID。
///
/// 在构建流程中的位置：
///   rcBuildCompactHeightfield → rcErodeWalkableArea → rcBuildDistanceField → 【rcBuildRegionsMonotone】
///
/// @see rcCompactHeightfield, rcCompactSpan, rcBuildDistanceField, rcBuildRegions, rcConfig
bool rcBuildRegionsMonotone(rcContext* ctx, rcCompactHeightfield& chf,
							const int borderSize, const int minRegionArea, const int mergeRegionArea)
{
	rcAssert(ctx);
	
	rcScopedTimer timer(ctx, RC_TIMER_BUILD_REGIONS);
	
	const int w = chf.width;
	const int h = chf.height;
	unsigned short id = 1;
	
	rcScopedDelete<unsigned short> srcReg((unsigned short*)rcAlloc(sizeof(unsigned short)*chf.spanCount, RC_ALLOC_TEMP));
	if (!srcReg)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildRegionsMonotone: Out of memory 'src' (%d).", chf.spanCount);
		return false;
	}
	memset(srcReg,0,sizeof(unsigned short)*chf.spanCount);

	const int nsweeps = rcMax(chf.width,chf.height);
	rcScopedDelete<rcSweepSpan> sweeps((rcSweepSpan*)rcAlloc(sizeof(rcSweepSpan)*nsweeps, RC_ALLOC_TEMP));
	if (!sweeps)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildRegionsMonotone: Out of memory 'sweeps' (%d).", nsweeps);
		return false;
	}
	
	
	// =========================================================================
	// 第一步：标记边界区域
	// =========================================================================
	// 在 Tile 边界处创建四个边界区域（带 RC_BORDER_REG 标志）。
	// 边界区域不参与后续的合并和过滤，确保 Tile 拼接时的连接正确性。
	if (borderSize > 0)
	{
		// 防止 borderSize 超过网格尺寸
		const int bw = rcMin(w, borderSize);
		const int bh = rcMin(h, borderSize);
		// 绘制四个边界区域：左、右、下、上
		paintRectRegion(0, bw, 0, h, id|RC_BORDER_REG, chf, srcReg); id++;
		paintRectRegion(w-bw, w, 0, h, id|RC_BORDER_REG, chf, srcReg); id++;
		paintRectRegion(0, w, 0, bh, id|RC_BORDER_REG, chf, srcReg); id++;
		paintRectRegion(0, w, h-bh, h, id|RC_BORDER_REG, chf, srcReg); id++;
	}

	chf.borderSize = borderSize;
	
	rcTempVector<int> prev(256); // 每个全局区域在当前行的采样计数

	// =========================================================================
	// 第二步：行扫描分区
	// =========================================================================
	// 沿 Z 轴逐行扫描
	for (int y = borderSize; y < h-borderSize; ++y)
	{
		// Collect spans from this row.
		// 用 prev[] 统计每个全局区域在当前行的采样计数
		prev.resize(id+1);
		memset(&prev[0],0,sizeof(int)*id);
		unsigned short rid = 1; // 行内临时 ID 计数器
		
		for (int x = borderSize; x < w-borderSize; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				if (chf.areas[i] == RC_NULL_AREA) continue;
				
				// 尝试与左邻居(-X)合并：如果左邻居是同区域类型且非边界，继承其 ID
				unsigned short previd = 0;
				if (rcGetCon(s, 0) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirOffsetX(0);
					const int ay = y + rcGetDirOffsetY(0);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 0);
					if ((srcReg[ai] & RC_BORDER_REG) == 0 && chf.areas[i] == chf.areas[ai])
						previd = srcReg[ai];
				}
				
				if (!previd)
				{
					// 无左邻居或不可合并：分配新的行内临时 ID
					previd = rid++;
					sweeps[previd].rid = previd;
					sweeps[previd].ns = 0;
					sweeps[previd].nei = 0;
				}

				// 尝试与上一行(-Z)合并：检查上方邻居的区域
				if (rcGetCon(s,3) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirOffsetX(3);
					const int ay = y + rcGetDirOffsetY(3);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 3);
					if (srcReg[ai] && (srcReg[ai] & RC_BORDER_REG) == 0 && chf.areas[i] == chf.areas[ai])
					{
						unsigned short nr = srcReg[ai];
						if (!sweeps[previd].nei || sweeps[previd].nei == nr)
						{
							// 所有上行采样一致：继续统计
							sweeps[previd].nei = nr;
							sweeps[previd].ns++;
							prev[nr]++;
						}
						else
						{
							// 发现不同的上行邻居：标记为无法合并
							sweeps[previd].nei = RC_NULL_NEI;
						}
					}
				}

				srcReg[i] = previd;
			}
		}
		
		// 确定每个行内临时 ID 的全局 ID
		// 如果行内区域的所有上行采样都指向同一个邻居，则继承该邻居的 ID；
		// 否则分配新的全局 ID。
		for (int i = 1; i < rid; ++i)
		{
			if (sweeps[i].nei != RC_NULL_NEI && sweeps[i].nei != 0 &&
				prev[sweeps[i].nei] == (int)sweeps[i].ns)
			{
				sweeps[i].id = sweeps[i].nei;
			}
			else
			{
				sweeps[i].id = id++;
			}
		}
		
		// 将行内临时 ID 重新映射为全局 ID
		for (int x = borderSize; x < w-borderSize; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				if (srcReg[i] > 0 && srcReg[i] < rid)
					srcReg[i] = sweeps[srcReg[i]].id;
			}
		}
	}


	{
		rcScopedTimer timerFilter(ctx, RC_TIMER_BUILD_REGIONS_FILTER);

		// 合并和过滤区域
		rcTempVector<int> overlaps;
		chf.maxRegions = id;
		if (!mergeAndFilterRegions(ctx, minRegionArea, mergeRegionArea, chf.maxRegions, chf, srcReg, overlaps))
			return false;

		// 单调分区不会产生重叠区域
	}
	
	// 将结果写入紧凑高度场的 Span
	for (int i = 0; i < chf.spanCount; ++i)
		chf.spans[i].reg = srcReg[i];

	return true;
}

/// @par
/// 
/// 非空区域将由连通的、非重叠的可行走 Span 组成，形成单一轮廓。
/// 轮廓将形成简单多边形。
/// 
/// Watershed 分水岭算法：
///   核心思想：仿照地形学中的分水岭原理，以距离场为"高度"，
///   从最高的"山峰"（距离边界最远的点）开始逐级"降水"，
///   水从高处流向低处，不同"流域"的分界处就是区域边界。
///
///   步骤：
///     1. 标记边界区域（Tile 边缘）
///     2. 从最高水位线开始，每次降低 2 级：
///        a. expandRegions：将已有区域向当前水位的空白 Span 扩展
///        b. floodRegion：为剩余的未标记 Span 创建新区域
///     3. 最终扩展：处理所有剩余未标记的 Span
///     4. 合并和过滤区域
///
/// 在构建流程中的位置：
///   rcBuildCompactHeightfield → rcErodeWalkableArea → rcBuildDistanceField → 【rcBuildRegions】
///
/// @warning 必须先调用 #rcBuildDistanceField 创建距离场。
///
/// @see rcCompactHeightfield, rcCompactSpan, rcBuildDistanceField, rcBuildRegionsMonotone, rcConfig
bool rcBuildRegions(rcContext* ctx, rcCompactHeightfield& chf,
					const int borderSize, const int minRegionArea, const int mergeRegionArea)
{
	rcAssert(ctx);
	
	rcScopedTimer timer(ctx, RC_TIMER_BUILD_REGIONS);
	
	const int w = chf.width;
	const int h = chf.height;
	
	rcScopedDelete<unsigned short> buf((unsigned short*)rcAlloc(sizeof(unsigned short)*chf.spanCount*2, RC_ALLOC_TEMP));
	if (!buf)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildRegions: Out of memory 'tmp' (%d).", chf.spanCount*4);
		return false;
	}
	
	ctx->startTimer(RC_TIMER_BUILD_REGIONS_WATERSHED);

	const int LOG_NB_STACKS = 3;
	const int NB_STACKS = 1 << LOG_NB_STACKS;
	rcTempVector<LevelStackEntry> lvlStacks[NB_STACKS];
	for (int i=0; i<NB_STACKS; ++i)
		lvlStacks[i].reserve(256);

	rcTempVector<LevelStackEntry> stack;
	stack.reserve(256);
	
	unsigned short* srcReg = buf;
	unsigned short* srcDist = buf+chf.spanCount;
	
	memset(srcReg, 0, sizeof(unsigned short)*chf.spanCount);
	memset(srcDist, 0, sizeof(unsigned short)*chf.spanCount);
	
	unsigned short regionId = 1;
	// 起始水位线 = 最大距离值（向下取偶数），以确保每次降低 2 级
	unsigned short level = (chf.maxDistance+1) & ~1;

	// expandIters 控制分水岭“溢出”的程度，值越大区域边界越简化
	const int expandIters = 8;

	// =========================================================================
	// 第一步：标记 Tile 边界区域
	// =========================================================================
	if (borderSize > 0)
	{
		const int bw = rcMin(w, borderSize);
		const int bh = rcMin(h, borderSize);
		
		paintRectRegion(0, bw, 0, h, regionId|RC_BORDER_REG, chf, srcReg); regionId++;
		paintRectRegion(w-bw, w, 0, h, regionId|RC_BORDER_REG, chf, srcReg); regionId++;
		paintRectRegion(0, w, 0, bh, regionId|RC_BORDER_REG, chf, srcReg); regionId++;
		paintRectRegion(0, w, h-bh, h, regionId|RC_BORDER_REG, chf, srcReg); regionId++;
	}

	chf.borderSize = borderSize;
	
	// =========================================================================
	// 第二步：Watershed 主循环——逐级降低水位线
	// =========================================================================
	// 每次水位下降 2 级，直到达到 0。
	// 每一级先扩展已有区域，再为剩余的空白 Span 创建新区域。
	int sId = -1;
	while (level > 0)
	{
		level = level >= 2 ? level-2 : 0;
		sId = (sId+1) & (NB_STACKS-1); // 循环使用 8 个栈

//		ctx->startTimer(RC_TIMER_DIVIDE_TO_LEVELS);

		if (sId == 0)
			sortCellsByLevel(level, chf, srcReg, NB_STACKS, lvlStacks, 1); // 每 8 级重新排序
		else 
			appendStacks(lvlStacks[sId-1], lvlStacks[sId], srcReg); // 从上一级继承未处理的项

//		ctx->stopTimer(RC_TIMER_DIVIDE_TO_LEVELS);

		{
			rcScopedTimer timerExpand(ctx, RC_TIMER_BUILD_REGIONS_EXPAND);

			// 扩展已有区域：将周围空白的同水位 Span 纳入最近的区域
			expandRegions(expandIters, level, chf, srcReg, srcDist, lvlStacks[sId], false);
		}
		
		{
			rcScopedTimer timerFloor(ctx, RC_TIMER_BUILD_REGIONS_FLOOD);

			// 洪水填充：为当前水位下仍未标记的 Span 创建新区域
			for (int j = 0; j<lvlStacks[sId].size(); j++)
			{
				LevelStackEntry current = lvlStacks[sId][j];
				int x = current.x;
				int y = current.y;
				int i = current.index;
				if (i >= 0 && srcReg[i] == 0)
				{
					if (floodRegion(x, y, i, level, regionId, chf, srcReg, srcDist, stack))
					{
						if (regionId == 0xFFFF)
						{
							ctx->log(RC_LOG_ERROR, "rcBuildRegions: Region ID overflow");
							return false;
						}
						
						regionId++;
					}
				}
			}
		}
	}
	
	// 最终扩展：处理所有剩余未标记的 Span（包括距离为 0 的边界 Span）
	expandRegions(expandIters*8, 0, chf, srcReg, srcDist, stack, true);
	
	ctx->stopTimer(RC_TIMER_BUILD_REGIONS_WATERSHED);
	
	{
		rcScopedTimer timerFilter(ctx, RC_TIMER_BUILD_REGIONS_FILTER);

		// 合并和过滤区域
		rcTempVector<int> overlaps;
		chf.maxRegions = regionId;
		if (!mergeAndFilterRegions(ctx, minRegionArea, mergeRegionArea, chf.maxRegions, chf, srcReg, overlaps))
			return false;

		// 检查重叠区域（Watershed 可能产生少量重叠）
		if (overlaps.size() > 0)
		{
			ctx->log(RC_LOG_ERROR, "rcBuildRegions: %d overlapping regions.", overlaps.size());
		}
	}
		
	// 将结果写入紧凑高度场的 Span
	for (int i = 0; i < chf.spanCount; ++i)
		chf.spans[i].reg = srcReg[i];
	
	return true;
}


/// @par
///
/// 层级区域构建：使用行扫描算法分区，然后通过 mergeAndFilterLayerRegions 合并为层。
///
/// 与 Monotone 分区的区别：
///   - Monotone 使用 mergeAndFilterRegions（产生非重叠区域）
///   - Layer 使用 mergeAndFilterLayerRegions（产生可能重叠的分层区域，适合 TileCache）
///
/// 在构建流程中的位置：
///   rcBuildCompactHeightfield → rcErodeWalkableArea → 【rcBuildLayerRegions】
///   （注意：不需要 rcBuildDistanceField）
///
/// @see rcCompactHeightfield, rcCompactSpan, rcBuildRegions, rcBuildRegionsMonotone
bool rcBuildLayerRegions(rcContext* ctx, rcCompactHeightfield& chf,
						 const int borderSize, const int minRegionArea)
{
	rcAssert(ctx);
	
	rcScopedTimer timer(ctx, RC_TIMER_BUILD_REGIONS);
	
	const int w = chf.width;
	const int h = chf.height;
	unsigned short id = 1;
	
	rcScopedDelete<unsigned short> srcReg((unsigned short*)rcAlloc(sizeof(unsigned short)*chf.spanCount, RC_ALLOC_TEMP));
	if (!srcReg)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildLayerRegions: Out of memory 'src' (%d).", chf.spanCount);
		return false;
	}
	memset(srcReg,0,sizeof(unsigned short)*chf.spanCount);
	
	const int nsweeps = rcMax(chf.width,chf.height);
	rcScopedDelete<rcSweepSpan> sweeps((rcSweepSpan*)rcAlloc(sizeof(rcSweepSpan)*nsweeps, RC_ALLOC_TEMP));
	if (!sweeps)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildLayerRegions: Out of memory 'sweeps' (%d).", nsweeps);
		return false;
	}
	
	
	// 标记边界区域
	if (borderSize > 0)
	{
		const int bw = rcMin(w, borderSize);
		const int bh = rcMin(h, borderSize);
		paintRectRegion(0, bw, 0, h, id|RC_BORDER_REG, chf, srcReg); id++;
		paintRectRegion(w-bw, w, 0, h, id|RC_BORDER_REG, chf, srcReg); id++;
		paintRectRegion(0, w, 0, bh, id|RC_BORDER_REG, chf, srcReg); id++;
		paintRectRegion(0, w, h-bh, h, id|RC_BORDER_REG, chf, srcReg); id++;
	}

	chf.borderSize = borderSize;
	
	rcTempVector<int> prev(256);
	
	// 行扫描分区（与 Monotone 相同的行扫描算法）
	for (int y = borderSize; y < h-borderSize; ++y)
	{
		// 统计采样计数
		prev.resize(id+1);
		memset(&prev[0],0,sizeof(int)*id);
		unsigned short rid = 1;
		
		for (int x = borderSize; x < w-borderSize; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				if (chf.areas[i] == RC_NULL_AREA) continue;
				
				// 尝试与左邻居(-X)合并
				unsigned short previd = 0;
				if (rcGetCon(s, 0) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirOffsetX(0);
					const int ay = y + rcGetDirOffsetY(0);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 0);
					if ((srcReg[ai] & RC_BORDER_REG) == 0 && chf.areas[i] == chf.areas[ai])
						previd = srcReg[ai];
				}
				
				if (!previd)
				{
					previd = rid++;
					sweeps[previd].rid = previd;
					sweeps[previd].ns = 0;
					sweeps[previd].nei = 0;
				}
				
				// 尝试与上一行(-Z)合并
				if (rcGetCon(s,3) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirOffsetX(3);
					const int ay = y + rcGetDirOffsetY(3);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 3);
					if (srcReg[ai] && (srcReg[ai] & RC_BORDER_REG) == 0 && chf.areas[i] == chf.areas[ai])
					{
						unsigned short nr = srcReg[ai];
						if (!sweeps[previd].nei || sweeps[previd].nei == nr)
						{
							sweeps[previd].nei = nr;
							sweeps[previd].ns++;
							prev[nr]++;
						}
						else
						{
							sweeps[previd].nei = RC_NULL_NEI;
						}
					}
				}
				
				srcReg[i] = previd;
			}
		}
		
		// 确定行内临时 ID 的全局 ID
		for (int i = 1; i < rid; ++i)
		{
			if (sweeps[i].nei != RC_NULL_NEI && sweeps[i].nei != 0 &&
				prev[sweeps[i].nei] == (int)sweeps[i].ns)
			{
				sweeps[i].id = sweeps[i].nei;
			}
			else
			{
				sweeps[i].id = id++;
			}
		}
		
		// 将行内临时 ID 重新映射为全局 ID
		for (int x = borderSize; x < w-borderSize; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				if (srcReg[i] > 0 && srcReg[i] < rid)
					srcReg[i] = sweeps[srcReg[i]].id;
			}
		}
	}
	
	
	{
		rcScopedTimer timerFilter(ctx, RC_TIMER_BUILD_REGIONS_FILTER);

		// 合并单调区域为层并删除小区域
		chf.maxRegions = id;
		if (!mergeAndFilterLayerRegions(ctx, minRegionArea, chf.maxRegions, chf, srcReg))
			return false;
	}
	
	
	// 将结果写入紧凑高度场的 Span
	for (int i = 0; i < chf.spanCount; ++i)
		chf.spans[i].reg = srcReg[i];
	
	return true;
}
