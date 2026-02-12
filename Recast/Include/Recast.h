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
 
#ifndef RECAST_H
#define RECAST_H

/// The value of PI used by Recast.
static const float RC_PI = 3.14159265f;

/// Used to ignore unused function parameters and silence any compiler warnings.
template<class T> void rcIgnoreUnused(const T&) { }

/// Recast log categories.
/// @see rcContext
enum rcLogCategory
{
	RC_LOG_PROGRESS = 1,	///< A progress log entry.
	RC_LOG_WARNING,			///< A warning log entry.
	RC_LOG_ERROR			///< An error log entry.
};

/// Recast performance timer categories.
/// @see rcContext
enum rcTimerLabel
{
	/// The user defined total time of the build.
	RC_TIMER_TOTAL,
	/// A user defined build time.
	RC_TIMER_TEMP,
	/// The time to rasterize the triangles. (See: #rcRasterizeTriangle)
	RC_TIMER_RASTERIZE_TRIANGLES,
	/// The time to build the compact heightfield. (See: #rcBuildCompactHeightfield)
	RC_TIMER_BUILD_COMPACTHEIGHTFIELD,
	/// The total time to build the contours. (See: #rcBuildContours)
	RC_TIMER_BUILD_CONTOURS,
	/// The time to trace the boundaries of the contours. (See: #rcBuildContours)
	RC_TIMER_BUILD_CONTOURS_TRACE,
	/// The time to simplify the contours. (See: #rcBuildContours)
	RC_TIMER_BUILD_CONTOURS_SIMPLIFY,
	/// The time to filter ledge spans. (See: #rcFilterLedgeSpans)
	RC_TIMER_FILTER_BORDER,
	/// The time to filter low height spans. (See: #rcFilterWalkableLowHeightSpans)
	RC_TIMER_FILTER_WALKABLE,
	/// The time to apply the median filter. (See: #rcMedianFilterWalkableArea)
	RC_TIMER_MEDIAN_AREA,
	/// The time to filter low obstacles. (See: #rcFilterLowHangingWalkableObstacles)
	RC_TIMER_FILTER_LOW_OBSTACLES,
	/// The time to build the polygon mesh. (See: #rcBuildPolyMesh)
	RC_TIMER_BUILD_POLYMESH,
	/// The time to merge polygon meshes. (See: #rcMergePolyMeshes)
	RC_TIMER_MERGE_POLYMESH,
	/// The time to erode the walkable area. (See: #rcErodeWalkableArea)
	RC_TIMER_ERODE_AREA,
	/// The time to mark a box area. (See: #rcMarkBoxArea)
	RC_TIMER_MARK_BOX_AREA,
	/// The time to mark a cylinder area. (See: #rcMarkCylinderArea)
	RC_TIMER_MARK_CYLINDER_AREA,
	/// The time to mark a convex polygon area. (See: #rcMarkConvexPolyArea)
	RC_TIMER_MARK_CONVEXPOLY_AREA,
	/// The total time to build the distance field. (See: #rcBuildDistanceField)
	RC_TIMER_BUILD_DISTANCEFIELD,
	/// The time to build the distances of the distance field. (See: #rcBuildDistanceField)
	RC_TIMER_BUILD_DISTANCEFIELD_DIST,
	/// The time to blur the distance field. (See: #rcBuildDistanceField)
	RC_TIMER_BUILD_DISTANCEFIELD_BLUR,
	/// The total time to build the regions. (See: #rcBuildRegions, #rcBuildRegionsMonotone)
	RC_TIMER_BUILD_REGIONS,
	/// The total time to apply the watershed algorithm. (See: #rcBuildRegions)
	RC_TIMER_BUILD_REGIONS_WATERSHED,
	/// The time to expand regions while applying the watershed algorithm. (See: #rcBuildRegions)
	RC_TIMER_BUILD_REGIONS_EXPAND,
	/// The time to flood regions while applying the watershed algorithm. (See: #rcBuildRegions)
	RC_TIMER_BUILD_REGIONS_FLOOD,
	/// The time to filter out small regions. (See: #rcBuildRegions, #rcBuildRegionsMonotone)
	RC_TIMER_BUILD_REGIONS_FILTER,
	/// The time to build heightfield layers. (See: #rcBuildHeightfieldLayers)
	RC_TIMER_BUILD_LAYERS, 
	/// The time to build the polygon mesh detail. (See: #rcBuildPolyMeshDetail)
	RC_TIMER_BUILD_POLYMESHDETAIL,
	/// The time to merge polygon mesh details. (See: #rcMergePolyMeshDetails)
	RC_TIMER_MERGE_POLYMESHDETAIL,
	/// The maximum number of timers.  (Used for iterating timers.)
	RC_MAX_TIMERS
};

/// Provides an interface for optional logging and performance tracking of the Recast 
/// build process.
/// 
/// This class does not provide logging or timer functionality on its 
/// own.  Both must be provided by a concrete implementation 
/// by overriding the protected member functions.  Also, this class does not 
/// provide an interface for extracting log messages. (Only adding them.) 
/// So concrete implementations must provide one.
///
/// If no logging or timers are required, just pass an instance of this 
/// class through the Recast build process.
/// 
/// @ingroup recast
class rcContext
{
public:
	/// Constructor.
	///  @param[in]		state	TRUE if the logging and performance timers should be enabled.  [Default: true]
	inline rcContext(bool state = true) : m_logEnabled(state), m_timerEnabled(state) {}
	virtual ~rcContext() {}

	/// Enables or disables logging.
	///  @param[in]		state	TRUE if logging should be enabled.
	inline void enableLog(bool state) { m_logEnabled = state; }

	/// Clears all log entries.
	inline void resetLog() { if (m_logEnabled) doResetLog(); }

	/// Logs a message.
	///
	/// Example:
	/// @code
	/// // Where ctx is an instance of rcContext and filepath is a char array.
	/// ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not load '%s'", filepath);
	/// @endcode
	/// 
	/// @param[in]		category	The category of the message.
	/// @param[in]		format		The message.
	void log(const rcLogCategory category, const char* format, ...);

	/// Enables or disables the performance timers.
	///  @param[in]		state	TRUE if timers should be enabled.
	inline void enableTimer(bool state) { m_timerEnabled = state; }

	/// Clears all performance timers. (Resets all to unused.)
	inline void resetTimers() { if (m_timerEnabled) doResetTimers(); }

	/// Starts the specified performance timer.
	/// @param	label	The category of the timer.
	inline void startTimer(const rcTimerLabel label) { if (m_timerEnabled) doStartTimer(label); }

	/// Stops the specified performance timer.
	/// @param	label	The category of the timer.
	inline void stopTimer(const rcTimerLabel label) { if (m_timerEnabled) doStopTimer(label); }

	/// Returns the total accumulated time of the specified performance timer.
	/// @param	label	The category of the timer.
	/// @return The accumulated time of the timer, or -1 if timers are disabled or the timer has never been started.
	inline int getAccumulatedTime(const rcTimerLabel label) const { return m_timerEnabled ? doGetAccumulatedTime(label) : -1; }

protected:
	/// Clears all log entries.
	virtual void doResetLog();

	/// Logs a message.
	/// @param[in]		category	The category of the message.
	/// @param[in]		msg			The formatted message.
	/// @param[in]		len			The length of the formatted message.
	virtual void doLog(const rcLogCategory category, const char* msg, const int len) { rcIgnoreUnused(category); rcIgnoreUnused(msg); rcIgnoreUnused(len); }

	/// Clears all timers. (Resets all to unused.)
	virtual void doResetTimers() {}

	/// Starts the specified performance timer.
	/// @param[in]		label	The category of timer.
	virtual void doStartTimer(const rcTimerLabel label) { rcIgnoreUnused(label); }

	/// Stops the specified performance timer.
	/// @param[in]		label	The category of the timer.
	virtual void doStopTimer(const rcTimerLabel label) { rcIgnoreUnused(label); }

	/// Returns the total accumulated time of the specified performance timer.
	/// @param[in]		label	The category of the timer.
	/// @return The accumulated time of the timer, or -1 if timers are disabled or the timer has never been started.
	virtual int doGetAccumulatedTime(const rcTimerLabel label) const { rcIgnoreUnused(label); return -1; }
	
	/// True if logging is enabled.
	bool m_logEnabled;

	/// True if the performance timers are enabled.
	bool m_timerEnabled;
};

/// A helper to first start a timer and then stop it when this helper goes out of scope.
/// @see rcContext
class rcScopedTimer
{
public:
	/// Constructs an instance and starts the timer.
	///  @param[in]		ctx		The context to use.
	///  @param[in]		label	The category of the timer.
	inline rcScopedTimer(rcContext* ctx, const rcTimerLabel label) : m_ctx(ctx), m_label(label) { m_ctx->startTimer(m_label); }
	inline ~rcScopedTimer() { m_ctx->stopTimer(m_label); }

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	rcScopedTimer(const rcScopedTimer&);
	rcScopedTimer& operator=(const rcScopedTimer&);
	
	rcContext* const m_ctx;
	const rcTimerLabel m_label;
};

/// Specifies a configuration to use when performing Recast builds.
/// 指定执行 Recast 构建时使用的配置参数。
///
/// 该结构体包含导航网格构建流程中所有关键参数，分为以下几类：
///   - 网格尺寸参数：width, height, tileSize, borderSize
///   - 体素化参数：cs(XZ平面体素尺寸), ch(Y轴体素高度)
///   - 空间范围：bmin, bmax（构建区域的轴对齐包围盒）
///   - Agent参数：walkableSlopeAngle, walkableHeight, walkableClimb, walkableRadius
///   - 轮廓与多边形参数：maxEdgeLen, maxSimplificationError, maxVertsPerPoly
///   - 区域划分参数：minRegionArea, mergeRegionArea
///   - 细节网格参数：detailSampleDist, detailSampleMaxError
///
/// 单位说明：
///   - [vx] = 体素单位 (voxel units)，即以体素格子为计量的整数单位
///   - [wu] = 世界单位 (world units)，即场景中的浮点坐标单位
///
/// @ingroup recast
struct rcConfig
{
	/// The width of the field along the x-axis. [Limit: >= 0] [Units: vx]
	/// 高度场沿X轴的宽度（体素数）。
	/// 由 rcCalcGridSize() 根据包围盒和 cs 自动计算：width = ceil((bmax[0] - bmin[0]) / cs)
	int width;

	/// The height of the field along the z-axis. [Limit: >= 0] [Units: vx]
	/// 高度场沿Z轴的高度（体素数）。注意这里的"height"指XZ平面的Z方向格子数，不是Y轴。
	/// 由 rcCalcGridSize() 根据包围盒和 cs 自动计算：height = ceil((bmax[2] - bmin[2]) / cs)
	int height;
	
	/// The width/height size of tile's on the xz-plane. [Limit: >= 0] [Units: vx]
	/// 分块(Tile)模式下，每个Tile在XZ平面上的尺寸（体素数）。
	/// 仅在分块构建（如 TileMesh）时使用。若为0，表示不分块（Solo模式）。
	/// 典型值如 64、128 等。Tile越大构建越慢，但导航精度更高。
	int tileSize;
	
	/// The size of the non-navigable border around the heightfield. [Limit: >=0] [Units: vx]
	/// 高度场周围不可导航的边界大小（体素数）。
	/// 在分块模式下用于保证相邻Tile之间的无缝拼接，通常设为 walkableRadius 的值。
	/// Solo模式下通常为 0。
	int borderSize;

	/// The xz-plane cell size to use for fields. [Limit: > 0] [Units: wu] 
	/// XZ平面上每个体素格子的边长（世界单位）。
	/// 这是体素化的基础分辨率参数，值越小精度越高但内存和计算开销越大。
	/// 典型值：0.1 ~ 1.0，取决于场景规模。
	/// Agent半径通常应为 cs 的 2~4 倍，以保证合理的网格质量。
	float cs;

	/// The y-axis cell size to use for fields. [Limit: > 0] [Units: wu]
	/// Y轴方向（竖直方向）每个体素的高度（世界单位）。
	/// 影响高度场在垂直方向的分辨率。通常设为 cs 的一半（如 cs=0.3, ch=0.2）。
	/// 值越小，垂直方向的细节越精确（如台阶、斜坡的识别），但内存开销增大。
	float ch;

	/// The minimum bounds of the field's AABB. [(x, y, z)] [Units: wu]
	/// 构建区域轴对齐包围盒(AABB)的最小角坐标 (x, y, z)，世界单位。
	/// 定义了导航网格的构建范围下界。通常取自输入几何体的包围盒。
	float bmin[3]; 

	/// The maximum bounds of the field's AABB. [(x, y, z)] [Units: wu]
	/// 构建区域轴对齐包围盒(AABB)的最大角坐标 (x, y, z)，世界单位。
	/// 定义了导航网格的构建范围上界。通常取自输入几何体的包围盒。
	float bmax[3];

	/// The maximum slope that is considered walkable. [Limits: 0 <= value < 90] [Units: Degrees] 
	/// 可行走的最大坡度角（度数）。
	/// 在体素化阶段(Step 2)，通过 rcMarkWalkableTriangles() 判断每个三角形的法线与Y轴的夹角，
	/// 若夹角 <= walkableSlopeAngle，则标记该三角形为可行走(RC_WALKABLE_AREA)。
	/// 典型值：45 度。设为 0 则只有完全水平的面才可行走。
	float walkableSlopeAngle;

	/// Minimum floor to 'ceiling' height that will still allow the floor area to 
	/// be considered walkable. [Limit: >= 3] [Units: vx] 
	/// Agent的最小通行高度（体素数），即"地板到天花板"的最小净空。
	/// 若一个Span上方的净空高度小于此值，该Span将被标记为不可行走。
	/// 由 Agent身高(世界单位) 通过 ceilf(agentHeight / ch) 向上取整得到。
	/// 向上取整是为了保守估计，确保Agent能站得下。最小值为3。
	int walkableHeight;
	
	/// Maximum ledge height that is considered to still be traversable. [Limit: >=0] [Units: vx] 
	/// Agent可攀爬的最大台阶高度（体素数）。
	/// 在过滤阶段(Step 3)用于判断相邻Span之间的高度差是否可以跨越。
	/// 由 Agent最大攀爬高度(世界单位) 通过 floorf(agentMaxClimb / ch) 向下取整得到。
	/// 向下取整是为了保守估计，避免Agent攀爬过高的台阶。
	/// 此值也用于光栅化时相邻Span的合并判断。
	int walkableClimb;
	
	/// The distance to erode/shrink the walkable area of the heightfield away from 
	/// obstructions.  [Limit: >=0] [Units: vx] 
	/// Agent的半径（体素数），用于腐蚀(收缩)可行走区域。
	/// 在 Step 4 中，rcErodeWalkableArea() 会从所有不可行走区域的边界向内收缩此距离，
	/// 确保Agent中心点不会过于靠近墙壁或障碍物边缘。
	/// 由 Agent半径(世界单位) 通过 ceilf(agentRadius / cs) 向上取整得到。
	int walkableRadius;
	
	/// The maximum allowed length for contour edges along the border of the mesh. [Limit: >=0] [Units: vx] 
	/// 轮廓边的最大允许长度（体素数）。
	/// 在 Step 5 轮廓构建阶段，超过此长度的边会被细分为更短的线段，
	/// 以防止沿墙壁产生过长的直线轮廓边。值为 0 表示不限制。
	/// 由 edgeMaxLen(世界单位) / cs 转换得到。
	int maxEdgeLen;
	
	/// The maximum distance a simplified contour's border edges should deviate 
	/// the original raw contour. [Limit: >=0] [Units: vx]
	/// 轮廓简化的最大允许偏差（世界单位）。
	/// 在 Step 5 中，使用 Douglas-Peucker 算法简化轮廓时，简化后的边与原始边之间的
	/// 最大距离不超过此值。值越大简化越多（多边形越少但精度降低），值越小保留越多细节。
	/// 典型值：1.0 ~ 1.8。设为 0 则不简化。
	float maxSimplificationError;
	
	/// The minimum number of cells allowed to form isolated island areas. [Limit: >=0] [Units: vx] 
	/// 允许形成独立区域的最小面积（体素²）。
	/// 在 Step 4 区域划分时，面积小于此值的孤立区域将被移除。
	/// 用于清除由噪声或微小几何体产生的碎片区域。
	/// 注意：此值是面积，由 rcSqr(regionMinSize) 计算得到（即边长的平方）。
	int minRegionArea;
	
	/// Any regions with a span count smaller than this value will, if possible, 
	/// be merged with larger regions. [Limit: >=0] [Units: vx] 
	/// 区域合并面积阈值（体素²）。
	/// 面积小于此值的区域会尽可能被合并到相邻的更大区域中。
	/// 用于减少最终多边形数量，避免过多碎小多边形。
	/// 注意：此值是面积，由 rcSqr(regionMergeSize) 计算得到（即边长的平方）。
	int mergeRegionArea;
	
	/// The maximum number of vertices allowed for polygons generated during the 
	/// contour to polygon conversion process. [Limit: >= 3] 
	/// 轮廓转多边形时，每个多边形允许的最大顶点数。
	/// 在 Step 6 中，轮廓被三角化后，相邻三角形会合并为凸多边形，
	/// 合并后的多边形顶点数不超过此值。
	/// Detour 运行时要求此值 <= DT_VERTS_PER_POLYGON(默认为6)。
	/// 值越大，多边形越少（寻路效率越高），但多边形形状越复杂。
	/// 典型值：6。最小值：3（即纯三角形网格）。
	int maxVertsPerPoly;
	
	/// Sets the sampling distance to use when generating the detail mesh.
	/// (For height detail only.) [Limits: 0 or >= 0.9] [Units: wu] 
	/// 生成细节网格(Detail Mesh)时的采样间距（世界单位）。
	/// 在 Step 7 中，对每个多边形内部按此间距均匀采样高度点，
	/// 用于为平面多边形添加高度细节信息。
	/// 设为 0 表示禁用细节采样（多边形保持平面）。
	/// 值越小采样点越密、高度还原越精确，但计算开销越大。
	/// 实际使用时先判断：若 detailSampleDist < 0.9 则视为禁用。
	/// 典型设置：cellSize * 6（如 cs=0.3 时约 1.8）。
	float detailSampleDist;
	
	/// The maximum distance the detail mesh surface should deviate from heightfield
	/// data. (For height detail only.) [Limit: >=0] [Units: wu] 
	/// 细节网格表面与高度场数据之间允许的最大高度偏差（世界单位）。
	/// 在 Step 7 中，若某采样点处多边形表面与实际高度场的高度差超过此值，
	/// 则在该位置插入额外顶点以提升高度精度。
	/// 值越小，细节网格越贴合原始地形，但顶点和三角形数量越多。
	/// 典型设置：cellHeight * 1（如 ch=0.2 时为 0.2）。
	float detailSampleMaxError;
};

/// @brief 定义 rcSpan::smin 和 rcSpan::smax 位域所占的位数。
/// 使用 13 位可以表示 0 ~ 8191 的高度值范围，以体素单位(vx)计量。
/// 结合 ch(每个体素的世界高度)，最大可表示高度 = 8191 * ch。
static const int RC_SPAN_HEIGHT_BITS = 13;

/// @brief 定义 rcSpan::smin 和 rcSpan::smax 的最大允许值。
/// 值为 (1 << 13) - 1 = 8191，即 13 位无符号整数能表示的最大值。
static const int RC_SPAN_MAX_HEIGHT = (1 << RC_SPAN_HEIGHT_BITS) - 1;

/// @brief 每个 rcSpanPool 中预分配的 rcSpan 数量。
/// 使用内存池分配策略，一次性分配 2048 个 Span，减少频繁的堆内存分配开销。
/// 当高度场中的 Span 数量超过当前所有池的容量时，会自动创建新的 rcSpanPool。
/// @see rcSpanPool
static const int RC_SPANS_PER_POOL = 2048;

/// @brief 表示高度场中一个体素列内的一段实体跨度(Span)。
///
/// Span 是 Recast 体素化的基本单元。在高度场的每个 (x, z) 网格列中，
/// 可能存在多个沿 Y 轴排列的 Span，形成一个单链表结构。
/// 每个 Span 记录了一段被几何体占据的实体空间的高度范围 [smin, smax]。
///
/// 内存布局（位域）：
///   - smin (13 bits) + smax (13 bits) + area (6 bits) = 32 bits (一个 unsigned int)
///   - 再加上一个 next 指针（4/8 字节），整个结构体在 32 位系统上为 8 字节，
///     64 位系统上为 16 字节（含对齐填充）。
///
/// 示意图（一个网格列中的 Span 链表，从低到高排列）：
///   Y轴(高度)
///    ^  ┌──────┐
///    |  │Span C│ smin=50, smax=60 (最上层的实体)  → next = NULL
///    |  └──────┘
///    |  （开放空间: 40~50 之间为可通行区域）
///    |  ┌──────┐
///    |  │Span B│ smin=30, smax=40 (中间层的实体)  → next = Span C
///    |  └──────┘
///    |  （开放空间: 20~30 之间为可通行区域）
///    |  ┌──────┐
///    |  │Span A│ smin=0,  smax=20 (地面实体)      → next = Span B
///    |  └──────┘
///    +--→ XZ平面
///
/// @see rcHeightfield
struct rcSpan
{
	/// Span 底部高度（体素单位），即实体空间的下限。
	/// 使用位域存储，占 RC_SPAN_HEIGHT_BITS (13) 位，范围 [0, 8191]。
	/// [约束: smin < smax]
	unsigned int smin : RC_SPAN_HEIGHT_BITS;

	/// Span 顶部高度（体素单位），即实体空间的上限。
	/// 使用位域存储，占 RC_SPAN_HEIGHT_BITS (13) 位，范围 [0, 8191]。
	/// Span 的顶部(smax)到上方下一个 Span 的底部(next->smin)之间的空间，
	/// 就是开放空间（open space），用于判断是否可行走。
	/// [约束: smin < smax <= RC_SPAN_MAX_HEIGHT]
	unsigned int smax : RC_SPAN_HEIGHT_BITS;

	/// 该 Span 的区域 ID，用于标识地表类型。
	/// 使用 6 位存储，范围 [0, 63]。
	/// 常见取值：
	///   - RC_NULL_AREA (0)：不可行走区域
	///   - RC_WALKABLE_AREA (63)：可行走区域（默认标记值）
	///   - 1~62：用户自定义区域（如草地、道路、水域等）
	unsigned int area : 6;

	/// 指向同一列中下一个更高位置的 Span 的指针，形成单链表。
	/// 链表按 smin 从小到大排列（从低到高）。
	/// 若为 NULL，表示当前 Span 是该列中最高的 Span。
	rcSpan* next;
};

/// @brief 用于在高度场中快速分配 rcSpan 实例的内存池。
///
/// 为了避免在体素化过程中频繁进行堆内存分配（每次 new/malloc 一个 rcSpan），
/// Recast 采用了池化分配策略：每次需要新 Span 时从预分配的池中取出，
/// 释放时归还到空闲链表(freelist)而非释放内存。
///
/// 多个 rcSpanPool 通过 next 指针串成链表，挂在 rcHeightfield::pools 上。
/// 当所有现有池的 Span 都被用完时，会动态分配一个新的 rcSpanPool。
///
/// 内存池链表结构示意图：
///   rcHeightfield::pools → [Pool 3] → [Pool 2] → [Pool 1] → NULL
///                            ↑ 最新分配的池在链表头部
///
/// @see rcHeightfield, rcSpan, RC_SPANS_PER_POOL
struct rcSpanPool
{
	/// 指向下一个 Span 池的指针，形成单向链表。
	/// 新分配的池插入到链表头部（rcHeightfield::pools 始终指向最新的池）。
	rcSpanPool* next;

	/// 预分配的 Span 数组，每个池包含 RC_SPANS_PER_POOL (2048) 个 Span。
	/// 初始时所有 Span 通过 rcSpan::next 串成空闲链表，
	/// 使用时从 rcHeightfield::freelist 头部取出。
	rcSpan items[RC_SPANS_PER_POOL];
};

/// @brief 动态高度场，表示被几何体占据的实体空间。
///
/// rcHeightfield 是 Recast 导航网格构建流程中体素化阶段（Step 2）的核心数据结构。
/// 它将 3D 场景在 XZ 平面上划分为 width × height 的二维网格，
/// 每个网格列 (x, z) 中存储一个 rcSpan 链表，记录沿 Y 轴被几何体占据的实体跨度。
///
/// 数据结构示意图（俯视 XZ 平面，每个格子对应一个 Span 链表）：
///
///   Z轴 ↑
///        ┌───┬───┬───┬───┐
///    h-1 │   │   │   │   │  ← 每个格子 = spans[x + z * width]
///        ├───┼───┼───┼───┤     指向该列的 Span 链表头
///    ... │   │   │   │   │
///        ├───┼───┼───┼───┤
///      1 │   │   │   │   │
///        ├───┼───┼───┼───┤
///      0 │   │   │   │   │
///        └───┴───┴───┴───┘
///          0   1  ...  w-1  → X轴
///
///   每个格子的边长 = cs（XZ平面体素尺寸）
///   每个 Span 的高度量化单位 = ch（Y轴体素高度）
///
/// 典型使用流程：
///   1. rcAllocHeightfield() — 分配 rcHeightfield 对象
///   2. rcCreateHeightfield() — 初始化网格尺寸、包围盒、分配 spans 数组
///   3. rcRasterizeTriangles() — 将三角形光栅化为 Span 填入高度场
///   4. rcFilter*() — 过滤不可行走的 Span
///   5. rcBuildCompactHeightfield() — 转换为紧凑高度场继续后续处理
///   6. rcFreeHeightField() — 释放高度场
///
/// 内存管理：
///   Span 的分配采用内存池策略（pools + freelist），避免频繁的堆分配。
///   使用 rcAllocHeightfield() 创建，rcFreeHeightField() 销毁。
///   禁止拷贝（拷贝构造和赋值运算符被禁用）。
///
/// @ingroup recast
/// @see rcSpan, rcSpanPool, rcAllocHeightfield, rcFreeHeightField, rcCreateHeightfield
struct rcHeightfield
{
	/// 默认构造函数。将所有成员值初始化为零/空。
	/// 构造后需要调用 rcCreateHeightfield() 完成实际初始化（分配 spans 数组等）。
	rcHeightfield();

	/// 析构函数。释放所有已分配的内存：
	///   - 释放 spans 指针数组（rcFree）
	///   - 遍历并释放所有 rcSpanPool 内存池节点（rcFree）
	~rcHeightfield();

	/// 高度场在 X 轴方向的网格宽度（体素数）。
	/// 由 rcCreateHeightfield() 设置，值来源于 rcCalcGridSize() 的计算结果。
	/// 计算公式：width = (int)((bmax[0] - bmin[0]) / cs + 0.5f)
	int width;

	/// 高度场在 Z 轴方向的网格高度（体素数）。
	/// 注意：这里 "height" 指的是 XZ 平面上 Z 方向的格子数，而非 Y 轴的高度。
	/// 由 rcCreateHeightfield() 设置。
	/// 计算公式：height = (int)((bmax[2] - bmin[2]) / cs + 0.5f)
	int height;

	/// 高度场轴对齐包围盒 (AABB) 的最小角坐标 (x, y, z)，世界单位。
	/// 定义了高度场覆盖区域的下界。
	/// 世界坐标到体素坐标的转换：voxelX = (worldX - bmin[0]) / cs
	float bmin[3];

	/// 高度场轴对齐包围盒 (AABB) 的最大角坐标 (x, y, z)，世界单位。
	/// 定义了高度场覆盖区域的上界。
	float bmax[3];

	/// XZ 平面上每个体素格子的边长（世界单位）。
	/// 这是水平方向的分辨率参数。值越小，网格越精细，但 width * height 越大。
	/// 用于世界坐标与体素坐标之间的转换。
	float cs;

	/// Y 轴方向每个体素的高度（世界单位）。
	/// 这是垂直方向的分辨率参数。Span 的 smin/smax 以此为量化单位。
	/// 世界高度到体素高度的转换：voxelY = (worldY - bmin[1]) / ch
	float ch;

	/// Span 链表指针数组，大小为 width * height。
	/// 索引方式：spans[x + z * width] 获取 (x, z) 位置的 Span 链表头指针。
	/// 如果某个位置没有任何几何体占据，则对应指针为 NULL。
	/// 每个链表中的 Span 按 smin 从小到大（从低到高）排列。
	/// 该数组由 rcCreateHeightfield() 分配（大小为 width * height 个指针）。
	rcSpan** spans;

	// ---- 以下为 rcSpan 实例的内存池管理 ----

	/// Span 内存池链表的头指针。
	/// 每个 rcSpanPool 节点包含 RC_SPANS_PER_POOL (2048) 个预分配的 rcSpan。
	/// 新分配的池插入链表头部。析构时遍历此链表释放所有池。
	rcSpanPool* pools;

	/// 空闲 Span 链表的头指针（使用 rcSpan::next 串联）。
	/// 分配新 Span 时从此链表头部取出；释放 Span 时归还到此链表头部。
	/// 当 freelist 为 NULL 时，会分配新的 rcSpanPool 并将其中的 Span 加入空闲链表。
	rcSpan* freelist;

private:
	// 显式禁用拷贝构造函数 —— rcHeightfield 持有动态分配的资源（spans数组、内存池），
	// 浅拷贝会导致双重释放等问题，因此禁止拷贝。
	rcHeightfield(const rcHeightfield&);

	// 显式禁用拷贝赋值运算符 —— 原因同上。
	rcHeightfield& operator=(const rcHeightfield&);
};

/// Provides information on the content of a cell column in a compact heightfield. 
struct rcCompactCell
{
	unsigned int index : 24;	///< Index to the first span in the column.
	unsigned int count : 8;		///< Number of spans in the column.
};

/// Represents a span of unobstructed space within a compact heightfield.
struct rcCompactSpan
{
	unsigned short y;			///< The lower extent of the span. (Measured from the heightfield's base.)
	unsigned short reg;			///< The id of the region the span belongs to. (Or zero if not in a region.)
	unsigned int con : 24;		///< Packed neighbor connection data.
	unsigned int h : 8;			///< The height of the span.  (Measured from #y.)
};

/// A compact, static heightfield representing unobstructed space.
/// @ingroup recast
struct rcCompactHeightfield
{
	rcCompactHeightfield();
	~rcCompactHeightfield();
	
	int width;					///< The width of the heightfield. (Along the x-axis in cell units.)
	int height;					///< The height of the heightfield. (Along the z-axis in cell units.)
	int spanCount;				///< The number of spans in the heightfield.
	int walkableHeight;			///< The walkable height used during the build of the field.  (See: rcConfig::walkableHeight)
	int walkableClimb;			///< The walkable climb used during the build of the field. (See: rcConfig::walkableClimb)
	int borderSize;				///< The AABB border size used during the build of the field. (See: rcConfig::borderSize)
	unsigned short maxDistance;	///< The maximum distance value of any span within the field. 
	unsigned short maxRegions;	///< The maximum region id of any span within the field. 
	float bmin[3];				///< The minimum bounds in world space. [(x, y, z)]
	float bmax[3];				///< The maximum bounds in world space. [(x, y, z)]
	float cs;					///< The size of each cell. (On the xz-plane.)
	float ch;					///< The height of each cell. (The minimum increment along the y-axis.)
	rcCompactCell* cells;		///< Array of cells. [Size: #width*#height]
	rcCompactSpan* spans;		///< Array of spans. [Size: #spanCount]
	unsigned short* dist;		///< Array containing border distance data. [Size: #spanCount]
	unsigned char* areas;		///< Array containing area id data. [Size: #spanCount]
	
private:
	// Explicitly-disabled copy constructor and copy assignment operator.
	rcCompactHeightfield(const rcCompactHeightfield&);
	rcCompactHeightfield& operator=(const rcCompactHeightfield&);
};

/// Represents a heightfield layer within a layer set.
/// @see rcHeightfieldLayerSet
struct rcHeightfieldLayer
{
	float bmin[3];				///< The minimum bounds in world space. [(x, y, z)]
	float bmax[3];				///< The maximum bounds in world space. [(x, y, z)]
	float cs;					///< The size of each cell. (On the xz-plane.)
	float ch;					///< The height of each cell. (The minimum increment along the y-axis.)
	int width;					///< The width of the heightfield. (Along the x-axis in cell units.)
	int height;					///< The height of the heightfield. (Along the z-axis in cell units.)
	int minx;					///< The minimum x-bounds of usable data.
	int maxx;					///< The maximum x-bounds of usable data.
	int miny;					///< The minimum y-bounds of usable data. (Along the z-axis.)
	int maxy;					///< The maximum y-bounds of usable data. (Along the z-axis.)
	int hmin;					///< The minimum height bounds of usable data. (Along the y-axis.)
	int hmax;					///< The maximum height bounds of usable data. (Along the y-axis.)
	unsigned char* heights;		///< The heightfield. [Size: width * height]
	unsigned char* areas;		///< Area ids. [Size: Same as #heights]
	unsigned char* cons;		///< Packed neighbor connection information. [Size: Same as #heights]
};

/// Represents a set of heightfield layers.
/// @ingroup recast
/// @see rcAllocHeightfieldLayerSet, rcFreeHeightfieldLayerSet 
struct rcHeightfieldLayerSet
{
	rcHeightfieldLayerSet();
	~rcHeightfieldLayerSet();
	
	rcHeightfieldLayer* layers;			///< The layers in the set. [Size: #nlayers]
	int nlayers;						///< The number of layers in the set.
	
private:
	// Explicitly-disabled copy constructor and copy assignment operator.
	rcHeightfieldLayerSet(const rcHeightfieldLayerSet&);
	rcHeightfieldLayerSet& operator=(const rcHeightfieldLayerSet&);
};

/// Represents a simple, non-overlapping contour in field space.
struct rcContour
{
	int* verts;			///< Simplified contour vertex and connection data. [Size: 4 * #nverts]
	int nverts;			///< The number of vertices in the simplified contour. 
	int* rverts;		///< Raw contour vertex and connection data. [Size: 4 * #nrverts]
	int nrverts;		///< The number of vertices in the raw contour. 
	unsigned short reg;	///< The region id of the contour.
	unsigned char area;	///< The area id of the contour.
};

/// Represents a group of related contours.
/// @ingroup recast
struct rcContourSet
{
	rcContourSet();
	~rcContourSet();
	
	rcContour* conts;	///< An array of the contours in the set. [Size: #nconts]
	int nconts;			///< The number of contours in the set.
	float bmin[3];  	///< The minimum bounds in world space. [(x, y, z)]
	float bmax[3];		///< The maximum bounds in world space. [(x, y, z)]
	float cs;			///< The size of each cell. (On the xz-plane.)
	float ch;			///< The height of each cell. (The minimum increment along the y-axis.)
	int width;			///< The width of the set. (Along the x-axis in cell units.) 
	int height;			///< The height of the set. (Along the z-axis in cell units.) 
	int borderSize;		///< The AABB border size used to generate the source data from which the contours were derived.
	float maxError;		///< The max edge error that this contour set was simplified with.
	
private:
	// Explicitly-disabled copy constructor and copy assignment operator.
	rcContourSet(const rcContourSet&);
	rcContourSet& operator=(const rcContourSet&);
};

/// Represents a polygon mesh suitable for use in building a navigation mesh. 
/// @ingroup recast
struct rcPolyMesh
{
	rcPolyMesh();
	~rcPolyMesh();
	
	unsigned short* verts;	///< The mesh vertices. [Form: (x, y, z) * #nverts]
	unsigned short* polys;	///< Polygon and neighbor data. [Length: #maxpolys * 2 * #nvp]
	unsigned short* regs;	///< The region id assigned to each polygon. [Length: #maxpolys]
	unsigned short* flags;	///< The user defined flags for each polygon. [Length: #maxpolys]
	unsigned char* areas;	///< The area id assigned to each polygon. [Length: #maxpolys]
	int nverts;				///< The number of vertices.
	int npolys;				///< The number of polygons.
	int maxpolys;			///< The number of allocated polygons.
	int nvp;				///< The maximum number of vertices per polygon.
	float bmin[3];			///< The minimum bounds in world space. [(x, y, z)]
	float bmax[3];			///< The maximum bounds in world space. [(x, y, z)]
	float cs;				///< The size of each cell. (On the xz-plane.)
	float ch;				///< The height of each cell. (The minimum increment along the y-axis.)
	int borderSize;			///< The AABB border size used to generate the source data from which the mesh was derived.
	float maxEdgeError;		///< The max error of the polygon edges in the mesh.
	
private:
	// Explicitly-disabled copy constructor and copy assignment operator.
	rcPolyMesh(const rcPolyMesh&);
	rcPolyMesh& operator=(const rcPolyMesh&);
};

/// Contains triangle meshes that represent detailed height data associated 
/// with the polygons in its associated polygon mesh object.
/// @ingroup recast
struct rcPolyMeshDetail
{
	rcPolyMeshDetail();
	
	unsigned int* meshes;	///< The sub-mesh data. [Size: 4*#nmeshes] 
	float* verts;			///< The mesh vertices. [Size: 3*#nverts] 
	unsigned char* tris;	///< The mesh triangles. [Size: 4*#ntris] 
	int nmeshes;			///< The number of sub-meshes defined by #meshes.
	int nverts;				///< The number of vertices in #verts.
	int ntris;				///< The number of triangles in #tris.
	
private:
	// Explicitly-disabled copy constructor and copy assignment operator.
	rcPolyMeshDetail(const rcPolyMeshDetail&);
	rcPolyMeshDetail& operator=(const rcPolyMeshDetail&);
};

/// @name Allocation Functions
/// Functions used to allocate and de-allocate Recast objects.
/// @see rcAllocSetCustom
/// @{

/// Allocates a heightfield object using the Recast allocator.
/// @return A heightfield that is ready for initialization, or null on failure.
/// @ingroup recast
/// @see rcCreateHeightfield, rcFreeHeightField
rcHeightfield* rcAllocHeightfield();

/// Frees the specified heightfield object using the Recast allocator.
/// @param[in]		heightfield	A heightfield allocated using #rcAllocHeightfield
/// @ingroup recast
/// @see rcAllocHeightfield
void rcFreeHeightField(rcHeightfield* heightfield);

/// Allocates a compact heightfield object using the Recast allocator.
/// @return A compact heightfield that is ready for initialization, or null on failure.
/// @ingroup recast
/// @see rcBuildCompactHeightfield, rcFreeCompactHeightfield
rcCompactHeightfield* rcAllocCompactHeightfield();

/// Frees the specified compact heightfield object using the Recast allocator.
/// @param[in]		compactHeightfield		A compact heightfield allocated using #rcAllocCompactHeightfield
/// @ingroup recast
/// @see rcAllocCompactHeightfield
void rcFreeCompactHeightfield(rcCompactHeightfield* compactHeightfield);

/// Allocates a heightfield layer set using the Recast allocator.
/// @return A heightfield layer set that is ready for initialization, or null on failure.
/// @ingroup recast
/// @see rcBuildHeightfieldLayers, rcFreeHeightfieldLayerSet
rcHeightfieldLayerSet* rcAllocHeightfieldLayerSet();

/// Frees the specified heightfield layer set using the Recast allocator.
/// @param[in]		layerSet	A heightfield layer set allocated using #rcAllocHeightfieldLayerSet
/// @ingroup recast
/// @see rcAllocHeightfieldLayerSet
void rcFreeHeightfieldLayerSet(rcHeightfieldLayerSet* layerSet);

/// Allocates a contour set object using the Recast allocator.
/// @return A contour set that is ready for initialization, or null on failure.
/// @ingroup recast
/// @see rcBuildContours, rcFreeContourSet
rcContourSet* rcAllocContourSet();

/// Frees the specified contour set using the Recast allocator.
/// @param[in]		contourSet	A contour set allocated using #rcAllocContourSet
/// @ingroup recast
/// @see rcAllocContourSet
void rcFreeContourSet(rcContourSet* contourSet);

/// Allocates a polygon mesh object using the Recast allocator.
/// @return A polygon mesh that is ready for initialization, or null on failure.
/// @ingroup recast
/// @see rcBuildPolyMesh, rcFreePolyMesh
rcPolyMesh* rcAllocPolyMesh();

/// Frees the specified polygon mesh using the Recast allocator.
/// @param[in]		polyMesh	A polygon mesh allocated using #rcAllocPolyMesh
/// @ingroup recast
/// @see rcAllocPolyMesh
void rcFreePolyMesh(rcPolyMesh* polyMesh);

/// Allocates a detail mesh object using the Recast allocator.
/// @return A detail mesh that is ready for initialization, or null on failure.
/// @ingroup recast
/// @see rcBuildPolyMeshDetail, rcFreePolyMeshDetail
rcPolyMeshDetail* rcAllocPolyMeshDetail();

/// Frees the specified detail mesh using the Recast allocator.
/// @param[in]		detailMesh	A detail mesh allocated using #rcAllocPolyMeshDetail
/// @ingroup recast
/// @see rcAllocPolyMeshDetail
void rcFreePolyMeshDetail(rcPolyMeshDetail* detailMesh);

/// @}

/// Heightfield border flag.
/// If a heightfield region ID has this bit set, then the region is a border 
/// region and its spans are considered un-walkable.
/// (Used during the region and contour build process.)
/// @see rcCompactSpan::reg
static const unsigned short RC_BORDER_REG = 0x8000;

/// Polygon touches multiple regions.
/// If a polygon has this region ID it was merged with or created
/// from polygons of different regions during the polymesh
/// build step that removes redundant border vertices. 
/// (Used during the polymesh and detail polymesh build processes)
/// @see rcPolyMesh::regs
static const unsigned short RC_MULTIPLE_REGS = 0;

/// Border vertex flag.
/// If a region ID has this bit set, then the associated element lies on
/// a tile border. If a contour vertex's region ID has this bit set, the 
/// vertex will later be removed in order to match the segments and vertices 
/// at tile boundaries.
/// (Used during the build process.)
/// @see rcCompactSpan::reg, #rcContour::verts, #rcContour::rverts
static const int RC_BORDER_VERTEX = 0x10000;

/// Area border flag.
/// If a region ID has this bit set, then the associated element lies on
/// the border of an area.
/// (Used during the region and contour build process.)
/// @see rcCompactSpan::reg, #rcContour::verts, #rcContour::rverts
static const int RC_AREA_BORDER = 0x20000;

/// Contour build flags.
/// @see rcBuildContours
enum rcBuildContoursFlags
{
	RC_CONTOUR_TESS_WALL_EDGES = 0x01,	///< Tessellate solid (impassable) edges during contour simplification.
	RC_CONTOUR_TESS_AREA_EDGES = 0x02	///< Tessellate edges between areas during contour simplification.
};

/// Applied to the region id field of contour vertices in order to extract the region id.
/// The region id field of a vertex may have several flags applied to it.  So the
/// fields value can't be used directly.
/// @see rcContour::verts, rcContour::rverts
static const int RC_CONTOUR_REG_MASK = 0xffff;

/// An value which indicates an invalid index within a mesh.
/// @note This does not necessarily indicate an error.
/// @see rcPolyMesh::polys
static const unsigned short RC_MESH_NULL_IDX = 0xffff;

/// Represents the null area.
/// When a data element is given this value it is considered to no longer be 
/// assigned to a usable area.  (E.g. It is un-walkable.)
static const unsigned char RC_NULL_AREA = 0;

/// The default area id used to indicate a walkable polygon. 
/// This is also the maximum allowed area id, and the only non-null area id 
/// recognized by some steps in the build process. 
static const unsigned char RC_WALKABLE_AREA = 63;

/// The value returned by #rcGetCon if the specified direction is not connected
/// to another span. (Has no neighbor.)
static const int RC_NOT_CONNECTED = 0x3f;

/// @name General helper functions
/// @{

/// Swaps the values of the two parameters.
/// @param[in,out]	a	Value A
/// @param[in,out]	b	Value B
template<class T> inline void rcSwap(T& a, T& b) { T t = a; a = b; b = t; }

/// Returns the minimum of two values.
/// @param[in]		a	Value A
/// @param[in]		b	Value B
/// @return The minimum of the two values.
template<class T> inline T rcMin(T a, T b) { return a < b ? a : b; }

/// Returns the maximum of two values.
/// @param[in]		a	Value A
/// @param[in]		b	Value B
/// @return The maximum of the two values.
template<class T> inline T rcMax(T a, T b) { return a > b ? a : b; }

/// Returns the absolute value.
/// @param[in]		a	The value.
/// @return The absolute value of the specified value.
template<class T> inline T rcAbs(T a) { return a < 0 ? -a : a; }

/// Returns the square of the value.
/// @param[in]		a	The value.
/// @return The square of the value.
template<class T> inline T rcSqr(T a) { return a * a; }

/// Clamps the value to the specified range.
/// @param[in]		value			The value to clamp.
/// @param[in]		minInclusive	The minimum permitted return value.
/// @param[in]		maxInclusive	The maximum permitted return value.
/// @return The value, clamped to the specified range.
template<class T> inline T rcClamp(T value, T minInclusive, T maxInclusive)
{
	return value < minInclusive ? minInclusive: (value > maxInclusive ? maxInclusive : value);
}

/// Returns the square root of the value.
///  @param[in]		x	The value.
///  @return The square root of the vlaue.
float rcSqrt(float x);

/// @}
/// @name Vector helper functions.
/// @{

/// Derives the cross product of two vectors. (@p v1 x @p v2)
/// @param[out]		dest	The cross product. [(x, y, z)]
/// @param[in]		v1		A Vector [(x, y, z)]
/// @param[in]		v2		A vector [(x, y, z)]
inline void rcVcross(float* dest, const float* v1, const float* v2)
{
	dest[0] = v1[1]*v2[2] - v1[2]*v2[1];
	dest[1] = v1[2]*v2[0] - v1[0]*v2[2];
	dest[2] = v1[0]*v2[1] - v1[1]*v2[0];
}

/// Derives the dot product of two vectors. (@p v1 . @p v2)
/// @param[in]		v1	A Vector [(x, y, z)]
/// @param[in]		v2	A vector [(x, y, z)]
/// @return The dot product.
inline float rcVdot(const float* v1, const float* v2)
{
	return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
}

/// Performs a scaled vector addition. (@p v1 + (@p v2 * @p s))
/// @param[out]		dest	The result vector. [(x, y, z)]
/// @param[in]		v1		The base vector. [(x, y, z)]
/// @param[in]		v2		The vector to scale and add to @p v1. [(x, y, z)]
/// @param[in]		s		The amount to scale @p v2 by before adding to @p v1.
inline void rcVmad(float* dest, const float* v1, const float* v2, const float s)
{
	dest[0] = v1[0]+v2[0]*s;
	dest[1] = v1[1]+v2[1]*s;
	dest[2] = v1[2]+v2[2]*s;
}

/// Performs a vector addition. (@p v1 + @p v2)
/// @param[out]		dest	The result vector. [(x, y, z)]
/// @param[in]		v1		The base vector. [(x, y, z)]
/// @param[in]		v2		The vector to add to @p v1. [(x, y, z)]
inline void rcVadd(float* dest, const float* v1, const float* v2)
{
	dest[0] = v1[0]+v2[0];
	dest[1] = v1[1]+v2[1];
	dest[2] = v1[2]+v2[2];
}

/// Performs a vector subtraction. (@p v1 - @p v2)
/// @param[out]		dest	The result vector. [(x, y, z)]
/// @param[in]		v1		The base vector. [(x, y, z)]
/// @param[in]		v2		The vector to subtract from @p v1. [(x, y, z)]
inline void rcVsub(float* dest, const float* v1, const float* v2)
{
	dest[0] = v1[0]-v2[0];
	dest[1] = v1[1]-v2[1];
	dest[2] = v1[2]-v2[2];
}

/// Selects the minimum value of each element from the specified vectors.
/// @param[in,out]	mn	A vector.  (Will be updated with the result.) [(x, y, z)]
/// @param[in]		v	A vector. [(x, y, z)]
inline void rcVmin(float* mn, const float* v)
{
	mn[0] = rcMin(mn[0], v[0]);
	mn[1] = rcMin(mn[1], v[1]);
	mn[2] = rcMin(mn[2], v[2]);
}

/// Selects the maximum value of each element from the specified vectors.
/// @param[in,out]	mx	A vector.  (Will be updated with the result.) [(x, y, z)]
/// @param[in]		v	A vector. [(x, y, z)]
inline void rcVmax(float* mx, const float* v)
{
	mx[0] = rcMax(mx[0], v[0]);
	mx[1] = rcMax(mx[1], v[1]);
	mx[2] = rcMax(mx[2], v[2]);
}

/// Performs a vector copy.
/// @param[out]		dest	The result. [(x, y, z)]
/// @param[in]		v		The vector to copy. [(x, y, z)]
inline void rcVcopy(float* dest, const float* v)
{
	dest[0] = v[0];
	dest[1] = v[1];
	dest[2] = v[2];
}

/// Returns the distance between two points.
/// @param[in]		v1	A point. [(x, y, z)]
/// @param[in]		v2	A point. [(x, y, z)]
/// @return The distance between the two points.
inline float rcVdist(const float* v1, const float* v2)
{
	float dx = v2[0] - v1[0];
	float dy = v2[1] - v1[1];
	float dz = v2[2] - v1[2];
	return rcSqrt(dx*dx + dy*dy + dz*dz);
}

/// Returns the square of the distance between two points.
/// @param[in]		v1	A point. [(x, y, z)]
/// @param[in]		v2	A point. [(x, y, z)]
/// @return The square of the distance between the two points.
inline float rcVdistSqr(const float* v1, const float* v2)
{
	float dx = v2[0] - v1[0];
	float dy = v2[1] - v1[1];
	float dz = v2[2] - v1[2];
	return dx*dx + dy*dy + dz*dz;
}

/// Normalizes the vector.
/// @param[in,out]	v	The vector to normalize. [(x, y, z)]
inline void rcVnormalize(float* v)
{
	float d = 1.0f / rcSqrt(rcSqr(v[0]) + rcSqr(v[1]) + rcSqr(v[2]));
	v[0] *= d;
	v[1] *= d;
	v[2] *= d;
}

/// @}
/// @name Heightfield Functions
/// @see rcHeightfield
/// @{

/// Calculates the bounding box of an array of vertices.
/// @ingroup recast
/// @param[in]		verts		An array of vertices. [(x, y, z) * @p nv]
/// @param[in]		numVerts	The number of vertices in the @p verts array.
/// @param[out]		minBounds	The minimum bounds of the AABB. [(x, y, z)] [Units: wu]
/// @param[out]		maxBounds	The maximum bounds of the AABB. [(x, y, z)] [Units: wu]
void rcCalcBounds(const float* verts, int numVerts, float* minBounds, float* maxBounds);

/// Calculates the grid size based on the bounding box and grid cell size.
/// @ingroup recast
/// @param[in]		minBounds	The minimum bounds of the AABB. [(x, y, z)] [Units: wu]
/// @param[in]		maxBounds	The maximum bounds of the AABB. [(x, y, z)] [Units: wu]
/// @param[in]		cellSize	The xz-plane cell size. [Limit: > 0] [Units: wu]
/// @param[out]		sizeX		The width along the x-axis. [Limit: >= 0] [Units: vx]
/// @param[out]		sizeZ		The height along the z-axis. [Limit: >= 0] [Units: vx]
void rcCalcGridSize(const float* minBounds, const float* maxBounds, float cellSize, int* sizeX, int* sizeZ);

/// Initializes a new heightfield.
/// See the #rcConfig documentation for more information on the configuration parameters.
/// 
/// @see rcAllocHeightfield, rcHeightfield
/// @ingroup recast
/// 
/// @param[in,out]	context		The build context to use during the operation.
/// @param[in,out]	heightfield	The allocated heightfield to initialize.
/// @param[in]		sizeX		The width of the field along the x-axis. [Limit: >= 0] [Units: vx]
/// @param[in]		sizeZ		The height of the field along the z-axis. [Limit: >= 0] [Units: vx]
/// @param[in]		minBounds	The minimum bounds of the field's AABB. [(x, y, z)] [Units: wu]
/// @param[in]		maxBounds	The maximum bounds of the field's AABB. [(x, y, z)] [Units: wu]
/// @param[in]		cellSize	The xz-plane cell size to use for the field. [Limit: > 0] [Units: wu]
/// @param[in]		cellHeight	The y-axis cell size to use for field. [Limit: > 0] [Units: wu]
/// @returns True if the operation completed successfully.
bool rcCreateHeightfield(rcContext* context, rcHeightfield& heightfield, int sizeX, int sizeZ,
						 const float* minBounds, const float* maxBounds,
						 float cellSize, float cellHeight);

/// Sets the area id of all triangles with a slope below the specified value
/// to #RC_WALKABLE_AREA.
///
/// Only sets the area id's for the walkable triangles.  Does not alter the
/// area id's for un-walkable triangles.
/// 
/// See the #rcConfig documentation for more information on the configuration parameters.
/// 
/// @see rcHeightfield, rcClearUnwalkableTriangles, rcRasterizeTriangles
/// 
/// @ingroup recast
/// @param[in,out]	context				The build context to use during the operation.
/// @param[in]		walkableSlopeAngle	The maximum slope that is considered walkable.
/// 									[Limits: 0 <= value < 90] [Units: Degrees]
/// @param[in]		verts				The vertices. [(x, y, z) * @p nv]
/// @param[in]		numVerts			The number of vertices.
/// @param[in]		tris				The triangle vertex indices. [(vertA, vertB, vertC) * @p nt]
/// @param[in]		numTris				The number of triangles.
/// @param[out]		triAreaIDs			The triangle area ids. [Length: >= @p nt]
void rcMarkWalkableTriangles(rcContext* context, float walkableSlopeAngle, const float* verts, int numVerts,
							 const int* tris, int numTris, unsigned char* triAreaIDs); 

/// Sets the area id of all triangles with a slope greater than or equal to the specified value to #RC_NULL_AREA.
/// 
/// Only sets the area id's for the un-walkable triangles.  Does not alter the
/// area id's for walkable triangles.
/// 
/// See the #rcConfig documentation for more information on the configuration parameters.
/// 
/// @see rcHeightfield, rcClearUnwalkableTriangles, rcRasterizeTriangles
/// 
/// @ingroup recast
/// @param[in,out]	context				The build context to use during the operation.
/// @param[in]		walkableSlopeAngle	The maximum slope that is considered walkable.
/// 									[Limits: 0 <= value < 90] [Units: Degrees]
/// @param[in]		verts				The vertices. [(x, y, z) * @p nv]
/// @param[in]		numVerts			The number of vertices.
/// @param[in]		tris				The triangle vertex indices. [(vertA, vertB, vertC) * @p nt]
/// @param[in]		numTris				The number of triangles.
/// @param[out]		triAreaIDs			The triangle area ids. [Length: >= @p nt]
void rcClearUnwalkableTriangles(rcContext* context, float walkableSlopeAngle, const float* verts, int numVerts,
								const int* tris, int numTris, unsigned char* triAreaIDs); 

/// Adds a span to the specified heightfield.
/// 
/// The span addition can be set to favor flags. If the span is merged to
/// another span and the new @p spanMax is within @p flagMergeThreshold units
/// from the existing span, the span flags are merged.
/// 
/// @ingroup recast
/// @param[in,out]	context				The build context to use during the operation.
/// @param[in,out]	heightfield			An initialized heightfield.
/// @param[in]		x					The column x index where the span is to be added.
/// 									[Limits: 0 <= value < rcHeightfield::width]
/// @param[in]		z					The column z index where the span is to be added.
/// 									[Limits: 0 <= value < rcHeightfield::height]
/// @param[in]		spanMin				The minimum height of the span. [Limit: < @p spanMax] [Units: vx]
/// @param[in]		spanMax				The maximum height of the span. [Limit: <= #RC_SPAN_MAX_HEIGHT] [Units: vx]
/// @param[in]		areaID				The area id of the span. [Limit: <= #RC_WALKABLE_AREA)
/// @param[in]		flagMergeThreshold	The merge threshold. [Limit: >= 0] [Units: vx]
/// @returns True if the operation completed successfully.
bool rcAddSpan(rcContext* context, rcHeightfield& heightfield,
	           int x, int z,
               unsigned short spanMin, unsigned short spanMax,
               unsigned char areaID, int flagMergeThreshold);

/// Rasterizes a single triangle into the specified heightfield.
///
/// Calling this for each triangle in a mesh is less efficient than calling rcRasterizeTriangles
///
/// No spans will be added if the triangle does not overlap the heightfield grid.
///
/// @see rcHeightfield
/// @ingroup recast
/// @param[in,out]	context				The build context to use during the operation.
/// @param[in]		v0					Triangle vertex 0 [(x, y, z)]
/// @param[in]		v1					Triangle vertex 1 [(x, y, z)]
/// @param[in]		v2					Triangle vertex 2 [(x, y, z)]
/// @param[in]		areaID				The area id of the triangle. [Limit: <= #RC_WALKABLE_AREA]
/// @param[in,out]	heightfield			An initialized heightfield.
/// @param[in]		flagMergeThreshold	The distance where the walkable flag is favored over the non-walkable flag.
/// 									[Limit: >= 0] [Units: vx]
/// @returns True if the operation completed successfully.
bool rcRasterizeTriangle(rcContext* context,
                         const float* v0, const float* v1, const float* v2,
                         unsigned char areaID, rcHeightfield& heightfield, int flagMergeThreshold = 1);

/// Rasterizes an indexed triangle mesh into the specified heightfield.
///
/// Spans will only be added for triangles that overlap the heightfield grid.
/// 
/// @see rcHeightfield
/// @ingroup recast
/// @param[in,out]	context				The build context to use during the operation.
/// @param[in]		verts				The vertices. [(x, y, z) * @p nv]
/// @param[in]		numVerts			The number of vertices. (unused) TODO (graham): Remove in next major release
/// @param[in]		tris				The triangle indices. [(vertA, vertB, vertC) * @p nt]
/// @param[in]		triAreaIDs			The area id's of the triangles. [Limit: <= #RC_WALKABLE_AREA] [Size: @p nt]
/// @param[in]		numTris				The number of triangles.
/// @param[in,out]	heightfield			An initialized heightfield.
/// @param[in]		flagMergeThreshold	The distance where the walkable flag is favored over the non-walkable flag. 
///										[Limit: >= 0] [Units: vx]
/// @returns True if the operation completed successfully.
bool rcRasterizeTriangles(rcContext* context,
                          const float* verts, int numVerts,
                          const int* tris, const unsigned char* triAreaIDs, int numTris,
                          rcHeightfield& heightfield, int flagMergeThreshold = 1);

/// Rasterizes an indexed triangle mesh into the specified heightfield.
///
/// Spans will only be added for triangles that overlap the heightfield grid.
/// 
/// @see rcHeightfield
/// @ingroup recast
/// @param[in,out]	context				The build context to use during the operation.
/// @param[in]		verts				The vertices. [(x, y, z) * @p nv]
/// @param[in]		numVerts			The number of vertices. (unused) TODO (graham): Remove in next major release
/// @param[in]		tris				The triangle indices. [(vertA, vertB, vertC) * @p nt]
/// @param[in]		triAreaIDs			The area id's of the triangles. [Limit: <= #RC_WALKABLE_AREA] [Size: @p nt]
/// @param[in]		numTris				The number of triangles.
/// @param[in,out]	heightfield			An initialized heightfield.
/// @param[in]		flagMergeThreshold	The distance where the walkable flag is favored over the non-walkable flag. 
/// 									[Limit: >= 0] [Units: vx]
/// @returns True if the operation completed successfully.
bool rcRasterizeTriangles(rcContext* context,
                          const float* verts, int numVerts,
                          const unsigned short* tris, const unsigned char* triAreaIDs, int numTris,
                          rcHeightfield& heightfield, int flagMergeThreshold = 1);

/// Rasterizes a triangle list into the specified heightfield.
///
/// Expects each triangle to be specified as three sequential vertices of 3 floats.
///
/// Spans will only be added for triangles that overlap the heightfield grid.
/// 
/// @see rcHeightfield
/// @ingroup recast
/// @param[in,out]	context				The build context to use during the operation.
/// @param[in]		verts				The triangle vertices. [(ax, ay, az, bx, by, bz, cx, by, cx) * @p nt]
/// @param[in]		triAreaIDs			The area id's of the triangles. [Limit: <= #RC_WALKABLE_AREA] [Size: @p nt]
/// @param[in]		numTris				The number of triangles.
/// @param[in,out]	heightfield			An initialized heightfield.
/// @param[in]		flagMergeThreshold	The distance where the walkable flag is favored over the non-walkable flag. 
/// 									[Limit: >= 0] [Units: vx]
/// @returns True if the operation completed successfully.
bool rcRasterizeTriangles(rcContext* context,
                          const float* verts, const unsigned char* triAreaIDs, int numTris,
                          rcHeightfield& heightfield, int flagMergeThreshold = 1);

/// Marks non-walkable spans as walkable if their maximum is within @p walkableClimb of the span below them.
///
/// This removes small obstacles and rasterization artifacts that the agent would be able to walk over
/// such as curbs.  It also allows agents to move up terraced structures like stairs.
/// 
/// Obstacle spans are marked walkable if: <tt>obstacleSpan.smax - walkableSpan.smax < walkableClimb</tt>
/// 
/// @warning Will override the effect of #rcFilterLedgeSpans.  If both filters are used, call #rcFilterLedgeSpans only after applying this filter.
///
/// @see rcHeightfield, rcConfig
/// 
/// @ingroup recast
/// @param[in,out]	context			The build context to use during the operation.
/// @param[in]		walkableClimb	Maximum ledge height that is considered to still be traversable. 
/// 								[Limit: >=0] [Units: vx]
/// @param[in,out]	heightfield		A fully built heightfield.  (All spans have been added.)
void rcFilterLowHangingWalkableObstacles(rcContext* context, int walkableClimb, rcHeightfield& heightfield);

/// Marks spans that are ledges as not-walkable.
///
/// A ledge is a span with one or more neighbors whose maximum is further away than @p walkableClimb
/// from the current span's maximum.
/// This method removes the impact of the overestimation of conservative voxelization 
/// so the resulting mesh will not have regions hanging in the air over ledges.
/// 
/// A span is a ledge if: <tt>rcAbs(currentSpan.smax - neighborSpan.smax) > walkableClimb</tt>
/// 
/// @see rcHeightfield, rcConfig
/// 
/// @ingroup recast
/// @param[in,out]	context				The build context to use during the operation.
/// @param[in]		walkableHeight	Minimum floor to 'ceiling' height that will still allow the floor area to 
/// 								be considered walkable. [Limit: >= 3] [Units: vx]
/// @param[in]		walkableClimb	Maximum ledge height that is considered to still be traversable. 
/// 								[Limit: >=0] [Units: vx]
/// @param[in,out]	heightfield			A fully built heightfield.  (All spans have been added.)
void rcFilterLedgeSpans(rcContext* context, int walkableHeight, int walkableClimb, rcHeightfield& heightfield);

/// Marks walkable spans as not walkable if the clearance above the span is less than the specified walkableHeight.
/// 
/// For this filter, the clearance above the span is the distance from the span's 
/// maximum to the minimum of the next higher span in the same column.
/// If there is no higher span in the column, the clearance is computed as the
/// distance from the top of the span to the maximum heightfield height.
/// 
/// @see rcHeightfield, rcConfig
/// @ingroup recast
/// 
/// @param[in,out]	context			The build context to use during the operation.
/// @param[in]		walkableHeight	Minimum floor to 'ceiling' height that will still allow the floor area to 
/// 								be considered walkable. [Limit: >= 3] [Units: vx]
/// @param[in,out]	heightfield		A fully built heightfield.  (All spans have been added.)
void rcFilterWalkableLowHeightSpans(rcContext* context, int walkableHeight, rcHeightfield& heightfield);

/// Returns the number of spans contained in the specified heightfield.
///  @ingroup recast
///  @param[in,out]	context		The build context to use during the operation.
///  @param[in]		heightfield	An initialized heightfield.
///  @returns The number of spans in the heightfield.
int rcGetHeightFieldSpanCount(rcContext* context, const rcHeightfield& heightfield);

/// @}
/// @name Compact Heightfield Functions
/// @see rcCompactHeightfield
/// @{

/// Builds a compact heightfield representing open space, from a heightfield representing solid space.
///
/// This is just the beginning of the process of fully building a compact heightfield.
/// Various filters may be applied, then the distance field and regions built.
/// E.g: #rcBuildDistanceField and #rcBuildRegions
///
/// See the #rcConfig documentation for more information on the configuration parameters.
///
/// @see rcAllocCompactHeightfield, rcHeightfield, rcCompactHeightfield, rcConfig
/// @ingroup recast
/// 
/// @param[in,out]	context				The build context to use during the operation.
/// @param[in]		walkableHeight		Minimum floor to 'ceiling' height that will still allow the floor area 
/// 									to be considered walkable. [Limit: >= 3] [Units: vx]
/// @param[in]		walkableClimb		Maximum ledge height that is considered to still be traversable. 
/// 									[Limit: >=0] [Units: vx]
/// @param[in]		heightfield			The heightfield to be compacted.
/// @param[out]		compactHeightfield	The resulting compact heightfield. (Must be pre-allocated.)
/// @returns True if the operation completed successfully.
bool rcBuildCompactHeightfield(rcContext* context, int walkableHeight, int walkableClimb,
							   const rcHeightfield& heightfield, rcCompactHeightfield& compactHeightfield);

/// Erodes the walkable area within the heightfield by the specified radius.
/// 
/// Basically, any spans that are closer to a boundary or obstruction than the specified radius 
/// are marked as un-walkable.
///
/// This method is usually called immediately after the heightfield has been built.
/// 
/// @see rcCompactHeightfield, rcBuildCompactHeightfield, rcConfig::walkableRadius
/// @ingroup recast
///
/// @param[in,out]	context				The build context to use during the operation.
/// @param[in]		erosionRadius		The radius of erosion. [Limits: 0 < value < 255] [Units: vx]
/// @param[in,out]	compactHeightfield	The populated compact heightfield to erode.
/// @returns True if the operation completed successfully.
bool rcErodeWalkableArea(rcContext* context, int erosionRadius, rcCompactHeightfield& compactHeightfield);

/// Applies a median filter to walkable area types (based on area id), removing noise.
/// 
/// This filter is usually applied after applying area id's using functions
/// such as #rcMarkBoxArea, #rcMarkConvexPolyArea, and #rcMarkCylinderArea.
/// 
/// @see rcCompactHeightfield
/// @ingroup recast
/// 
/// @param[in,out]	context		The build context to use during the operation.
/// @param[in,out]	compactHeightfield		A populated compact heightfield.
/// @returns True if the operation completed successfully.
bool rcMedianFilterWalkableArea(rcContext* context, rcCompactHeightfield& compactHeightfield);

/// Applies an area id to all spans within the specified bounding box. (AABB) 
/// 
/// @see rcCompactHeightfield, rcMedianFilterWalkableArea
/// @ingroup recast
/// 
/// @param[in,out]	context				The build context to use during the operation.
/// @param[in]		boxMinBounds		The minimum extents of the bounding box. [(x, y, z)] [Units: wu]
/// @param[in]		boxMaxBounds		The maximum extents of the bounding box. [(x, y, z)] [Units: wu]
/// @param[in]		areaId				The area id to apply. [Limit: <= #RC_WALKABLE_AREA]
/// @param[in,out]	compactHeightfield	A populated compact heightfield.
void rcMarkBoxArea(rcContext* context, const float* boxMinBounds, const float* boxMaxBounds, unsigned char areaId,
				   rcCompactHeightfield& compactHeightfield);

/// Applies the area id to the all spans within the specified convex polygon. 
///
/// The value of spacial parameters are in world units.
/// 
/// The y-values of the polygon vertices are ignored. So the polygon is effectively 
/// projected onto the xz-plane, translated to @p minY, and extruded to @p maxY.
/// 
/// @see rcCompactHeightfield, rcMedianFilterWalkableArea
/// @ingroup recast
/// 
/// @param[in,out]	context				The build context to use during the operation.
/// @param[in]		verts				The vertices of the polygon [For: (x, y, z) * @p numVerts]
/// @param[in]		numVerts			The number of vertices in the polygon.
/// @param[in]		minY				The height of the base of the polygon. [Units: wu]
/// @param[in]		maxY				The height of the top of the polygon. [Units: wu]
/// @param[in]		areaId				The area id to apply. [Limit: <= #RC_WALKABLE_AREA]
/// @param[in,out]	compactHeightfield	A populated compact heightfield.
void rcMarkConvexPolyArea(rcContext* context, const float* verts, int numVerts,
						  float minY, float maxY, unsigned char areaId,
						  rcCompactHeightfield& compactHeightfield);

/// Expands a convex polygon along its vertex normals by the given offset amount.
/// Inserts extra vertices to bevel sharp corners.
///
/// Helper function to offset convex polygons for rcMarkConvexPolyArea.
///
/// @ingroup recast
/// 
/// @param[in]		verts		The vertices of the polygon [Form: (x, y, z) * @p numVerts]
/// @param[in]		numVerts	The number of vertices in the polygon.
/// @param[in]		offset		How much to offset the polygon by. [Units: wu]
/// @param[out]		outVerts	The offset vertices (should hold up to 2 * @p numVerts) [Form: (x, y, z) * return value]
/// @param[in]		maxOutVerts	The max number of vertices that can be stored to @p outVerts.
/// @returns Number of vertices in the offset polygon or 0 if too few vertices in @p outVerts.
int rcOffsetPoly(const float* verts, int numVerts, float offset, float* outVerts, int maxOutVerts);

/// Applies the area id to all spans within the specified y-axis-aligned cylinder.
/// 
/// @see rcCompactHeightfield, rcMedianFilterWalkableArea
/// 
/// @ingroup recast
/// 
/// @param[in,out]	context				The build context to use during the operation.
/// @param[in]		position			The center of the base of the cylinder. [Form: (x, y, z)] [Units: wu] 
/// @param[in]		radius				The radius of the cylinder. [Units: wu] [Limit: > 0]
/// @param[in]		height				The height of the cylinder. [Units: wu] [Limit: > 0]
/// @param[in]		areaId				The area id to apply. [Limit: <= #RC_WALKABLE_AREA]
/// @param[in,out]	compactHeightfield	A populated compact heightfield.
void rcMarkCylinderArea(rcContext* context, const float* position, float radius, float height,
						unsigned char areaId, rcCompactHeightfield& compactHeightfield);

/// Builds the distance field for the specified compact heightfield. 
/// @ingroup recast
/// @param[in,out]	ctx		The build context to use during the operation.
/// @param[in,out]	chf		A populated compact heightfield.
/// @returns True if the operation completed successfully.
bool rcBuildDistanceField(rcContext* ctx, rcCompactHeightfield& chf);

/// Builds region data for the heightfield using watershed partitioning.
/// @ingroup recast
/// @param[in,out]	ctx				The build context to use during the operation.
/// @param[in,out]	chf				A populated compact heightfield.
/// @param[in]		borderSize		The size of the non-navigable border around the heightfield.
/// 								[Limit: >=0] [Units: vx]
/// @param[in]		minRegionArea	The minimum number of cells allowed to form isolated island areas.
/// 								[Limit: >=0] [Units: vx].
/// @param[in]		mergeRegionArea	Any regions with a span count smaller than this value will, if possible,
/// 								be merged with larger regions. [Limit: >=0] [Units: vx] 
/// @returns True if the operation completed successfully.
bool rcBuildRegions(rcContext* ctx, rcCompactHeightfield& chf, int borderSize, int minRegionArea, int mergeRegionArea);

/// Builds region data for the heightfield by partitioning the heightfield in non-overlapping layers.
/// @ingroup recast
/// @param[in,out]	ctx				The build context to use during the operation.
/// @param[in,out]	chf				A populated compact heightfield.
/// @param[in]		borderSize		The size of the non-navigable border around the heightfield.
///  								[Limit: >=0] [Units: vx]
/// @param[in]		minRegionArea	The minimum number of cells allowed to form isolated island areas.
///  								[Limit: >=0] [Units: vx].
/// @returns True if the operation completed successfully.
bool rcBuildLayerRegions(rcContext* ctx, rcCompactHeightfield& chf, int borderSize, int minRegionArea);

/// Builds region data for the heightfield using simple monotone partitioning.
/// @ingroup recast 
/// @param[in,out]	ctx				The build context to use during the operation.
/// @param[in,out]	chf				A populated compact heightfield.
/// @param[in]		borderSize		The size of the non-navigable border around the heightfield.
///  								[Limit: >=0] [Units: vx]
/// @param[in]		minRegionArea	The minimum number of cells allowed to form isolated island areas.
///  								[Limit: >=0] [Units: vx].
/// @param[in]		mergeRegionArea	Any regions with a span count smaller than this value will, if possible, 
///  								be merged with larger regions. [Limit: >=0] [Units: vx] 
/// @returns True if the operation completed successfully.
bool rcBuildRegionsMonotone(rcContext* ctx, rcCompactHeightfield& chf,
							int borderSize, int minRegionArea, int mergeRegionArea);

/// Sets the neighbor connection data for the specified direction.
/// @param[in]		span			The span to update.
/// @param[in]		direction		The direction to set. [Limits: 0 <= value < 4]
/// @param[in]		neighborIndex	The index of the neighbor span.
inline void rcSetCon(rcCompactSpan& span, int direction, int neighborIndex)
{
	const unsigned int shift = (unsigned int)direction * 6;
	const unsigned int con = span.con;
	span.con = (con & ~(0x3f << shift)) | (((unsigned int)neighborIndex & 0x3f) << shift);
}

/// Gets neighbor connection data for the specified direction.
/// @param[in]		span		The span to check.
/// @param[in]		direction	The direction to check. [Limits: 0 <= value < 4]
/// @return The neighbor connection data for the specified direction, or #RC_NOT_CONNECTED if there is no connection.
inline int rcGetCon(const rcCompactSpan& span, int direction)
{
	const unsigned int shift = (unsigned int)direction * 6;
	return (span.con >> shift) & 0x3f;
}

/// Gets the standard width (x-axis) offset for the specified direction.
/// @param[in]		direction		The direction. [Limits: 0 <= value < 4]
/// @return The width offset to apply to the current cell position to move in the direction.
inline int rcGetDirOffsetX(int direction)
{
	static const int offset[4] = { -1, 0, 1, 0, };
	return offset[direction & 0x03];
}

// TODO (graham): Rename this to rcGetDirOffsetZ
/// Gets the standard height (z-axis) offset for the specified direction.
/// @param[in]		direction		The direction. [Limits: 0 <= value < 4]
/// @return The height offset to apply to the current cell position to move in the direction.
inline int rcGetDirOffsetY(int direction)
{
	static const int offset[4] = { 0, 1, 0, -1 };
	return offset[direction & 0x03];
}

/// Gets the direction for the specified offset. One of x and y should be 0.
/// @param[in]		offsetX		The x offset. [Limits: -1 <= value <= 1]
/// @param[in]		offsetZ		The z offset. [Limits: -1 <= value <= 1]
/// @return The direction that represents the offset.
inline int rcGetDirForOffset(int offsetX, int offsetZ)
{
	static const int dirs[5] = { 3, 0, -1, 2, 1 };
	return dirs[((offsetZ + 1) << 1) + offsetX];
}

/// @}
/// @name Layer, Contour, Polymesh, and Detail Mesh Functions
/// @see rcHeightfieldLayer, rcContourSet, rcPolyMesh, rcPolyMeshDetail
/// @{

/// Builds a layer set from the specified compact heightfield.
/// @ingroup recast
/// @param[in,out]	ctx				The build context to use during the operation.
/// @param[in]		chf				A fully built compact heightfield.
/// @param[in]		borderSize		The size of the non-navigable border around the heightfield. [Limit: >=0] 
///  								[Units: vx]
/// @param[in]		walkableHeight	Minimum floor to 'ceiling' height that will still allow the floor area 
///  								to be considered walkable. [Limit: >= 3] [Units: vx]
/// @param[out]		lset			The resulting layer set. (Must be pre-allocated.)
/// @returns True if the operation completed successfully.
bool rcBuildHeightfieldLayers(rcContext* ctx, const rcCompactHeightfield& chf, 
							  int borderSize, int walkableHeight,
							  rcHeightfieldLayerSet& lset);

/// Builds a contour set from the region outlines in the provided compact heightfield.
/// @ingroup recast
/// @param[in,out]	ctx			The build context to use during the operation.
/// @param[in]		chf			A fully built compact heightfield.
/// @param[in]		maxError	The maximum distance a simplified contour's border edges should deviate 
/// 							the original raw contour. [Limit: >=0] [Units: wu]
/// @param[in]		maxEdgeLen	The maximum allowed length for contour edges along the border of the mesh. 
/// 							[Limit: >=0] [Units: vx]
/// @param[out]		cset		The resulting contour set. (Must be pre-allocated.)
/// @param[in]		buildFlags	The build flags. (See: #rcBuildContoursFlags)
/// @returns True if the operation completed successfully.
bool rcBuildContours(rcContext* ctx, const rcCompactHeightfield& chf,
					 float maxError, int maxEdgeLen,
					 rcContourSet& cset, int buildFlags = RC_CONTOUR_TESS_WALL_EDGES);

/// Builds a polygon mesh from the provided contours.
/// @ingroup recast
/// @param[in,out]	ctx		The build context to use during the operation.
/// @param[in]		cset	A fully built contour set.
/// @param[in]		nvp		The maximum number of vertices allowed for polygons generated during the 
/// 						contour to polygon conversion process. [Limit: >= 3] 
/// @param[out]		mesh	The resulting polygon mesh. (Must be re-allocated.)
/// @returns True if the operation completed successfully.
bool rcBuildPolyMesh(rcContext* ctx, const rcContourSet& cset, const int nvp, rcPolyMesh& mesh);

/// Merges multiple polygon meshes into a single mesh.
///  @ingroup recast
///  @param[in,out]	ctx		The build context to use during the operation.
///  @param[in]		meshes	An array of polygon meshes to merge. [Size: @p nmeshes]
///  @param[in]		nmeshes	The number of polygon meshes in the meshes array.
///  @param[in]		mesh	The resulting polygon mesh. (Must be pre-allocated.)
///  @returns True if the operation completed successfully.
bool rcMergePolyMeshes(rcContext* ctx, rcPolyMesh** meshes, const int nmeshes, rcPolyMesh& mesh);

/// Builds a detail mesh from the provided polygon mesh.
/// @ingroup recast
/// @param[in,out]	ctx				The build context to use during the operation.
/// @param[in]		mesh			A fully built polygon mesh.
/// @param[in]		chf				The compact heightfield used to build the polygon mesh.
/// @param[in]		sampleDist		Sets the distance to use when sampling the heightfield. [Limit: >=0] [Units: wu]
/// @param[in]		sampleMaxError	The maximum distance the detail mesh surface should deviate from 
/// 								heightfield data. [Limit: >=0] [Units: wu]
/// @param[out]		dmesh			The resulting detail mesh.  (Must be pre-allocated.)
/// @returns True if the operation completed successfully.
bool rcBuildPolyMeshDetail(rcContext* ctx, const rcPolyMesh& mesh, const rcCompactHeightfield& chf,
						   float sampleDist, float sampleMaxError,
						   rcPolyMeshDetail& dmesh);

/// Copies the poly mesh data from src to dst.
/// @ingroup recast
/// @param[in,out]	ctx		The build context to use during the operation.
/// @param[in]		src		The source mesh to copy from.
/// @param[out]		dst		The resulting detail mesh. (Must be pre-allocated, must be empty mesh.)
/// @returns True if the operation completed successfully.
bool rcCopyPolyMesh(rcContext* ctx, const rcPolyMesh& src, rcPolyMesh& dst);

/// Merges multiple detail meshes into a single detail mesh.
/// @ingroup recast
/// @param[in,out]	ctx		The build context to use during the operation.
/// @param[in]		meshes	An array of detail meshes to merge. [Size: @p nmeshes]
/// @param[in]		nmeshes	The number of detail meshes in the meshes array.
/// @param[out]		mesh	The resulting detail mesh. (Must be pre-allocated.)
/// @returns True if the operation completed successfully.
bool rcMergePolyMeshDetails(rcContext* ctx, rcPolyMeshDetail** meshes, const int nmeshes, rcPolyMeshDetail& mesh);

/// @}

#endif // RECAST_H

///////////////////////////////////////////////////////////////////////////

// Due to the large amount of detail documentation for this file, 
// the content normally located at the end of the header file has been separated
// out to a file in /Docs/Extern.
