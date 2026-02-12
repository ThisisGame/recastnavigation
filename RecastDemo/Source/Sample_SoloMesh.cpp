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
#include <stdio.h>
#include <string.h>
#include "SDL.h"
#include "SDL_opengl.h"
#include "imgui.h"
#include "InputGeom.h"
#include "Sample.h"
#include "Sample_SoloMesh.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "RecastDump.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourDebugDraw.h"
#include "NavMeshTesterTool.h"
#include "NavMeshPruneTool.h"
#include "OffMeshConnectionTool.h"
#include "ConvexVolumeTool.h"
#include "CrowdTool.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif


Sample_SoloMesh::Sample_SoloMesh() :
	m_keepInterResults(true),
	m_totalBuildTimeMs(0),
	m_triareas(0),
	m_solid(0),
	m_chf(0),
	m_cset(0),
	m_pmesh(0),
	m_dmesh(0),
	m_drawMode(DRAWMODE_NAVMESH)
{
	setTool(new NavMeshTesterTool);
}
		
Sample_SoloMesh::~Sample_SoloMesh()
{
	cleanup();
}
	
void Sample_SoloMesh::cleanup()
{
	delete [] m_triareas;
	m_triareas = 0;
	rcFreeHeightField(m_solid);
	m_solid = 0;
	rcFreeCompactHeightfield(m_chf);
	m_chf = 0;
	rcFreeContourSet(m_cset);
	m_cset = 0;
	rcFreePolyMesh(m_pmesh);
	m_pmesh = 0;
	rcFreePolyMeshDetail(m_dmesh);
	m_dmesh = 0;
	dtFreeNavMesh(m_navMesh);
	m_navMesh = 0;
}

void Sample_SoloMesh::handleSettings()
{
	Sample::handleCommonSettings();

	if (imguiCheck("Keep Itermediate Results", m_keepInterResults))
		m_keepInterResults = !m_keepInterResults;

	imguiSeparator();

	imguiIndent();
	imguiIndent();

	if (imguiButton("Save"))
	{
		Sample::saveAll("solo_navmesh.bin", m_navMesh);
	}

	if (imguiButton("Load"))
	{
		dtFreeNavMesh(m_navMesh);
		m_navMesh = Sample::loadAll("solo_navmesh.bin");
		m_navQuery->init(m_navMesh, 2048);
	}

	imguiUnindent();
	imguiUnindent();
	
	char msg[64];
	snprintf(msg, 64, "Build Time: %.1fms", m_totalBuildTimeMs);
	imguiLabel(msg);

	imguiSeparator();
}

void Sample_SoloMesh::handleTools()
{
	int type = !m_tool ? TOOL_NONE : m_tool->type();
	
	if (imguiCheck("Test Navmesh", type == TOOL_NAVMESH_TESTER))
	{
		setTool(new NavMeshTesterTool);
	}
	if (imguiCheck("Prune Navmesh", type == TOOL_NAVMESH_PRUNE))
	{
		setTool(new NavMeshPruneTool);
	}
	if (imguiCheck("Create Off-Mesh Connections", type == TOOL_OFFMESH_CONNECTION))
	{
		setTool(new OffMeshConnectionTool);
	}
	if (imguiCheck("Create Convex Volumes", type == TOOL_CONVEX_VOLUME))
	{
		setTool(new ConvexVolumeTool);
	}
	if (imguiCheck("Create Crowds", type == TOOL_CROWD))
	{
		setTool(new CrowdTool);
	}
	
	imguiSeparatorLine();

	imguiIndent();

	if (m_tool)
		m_tool->handleMenu();

	imguiUnindent();

}

void Sample_SoloMesh::handleDebugMode()
{
	// Check which modes are valid.
	bool valid[MAX_DRAWMODE];
	for (int i = 0; i < MAX_DRAWMODE; ++i)
		valid[i] = false;

	if (m_geom)
	{
		valid[DRAWMODE_NAVMESH] = m_navMesh != 0;
		valid[DRAWMODE_NAVMESH_TRANS] = m_navMesh != 0;
		valid[DRAWMODE_NAVMESH_BVTREE] = m_navMesh != 0;
		valid[DRAWMODE_NAVMESH_NODES] = m_navQuery != 0;
		valid[DRAWMODE_NAVMESH_INVIS] = m_navMesh != 0;
		valid[DRAWMODE_MESH] = true;
		valid[DRAWMODE_VOXELS] = m_solid != 0;
		valid[DRAWMODE_VOXELS_WALKABLE] = m_solid != 0;
		valid[DRAWMODE_COMPACT] = m_chf != 0;
		valid[DRAWMODE_COMPACT_DISTANCE] = m_chf != 0;
		valid[DRAWMODE_COMPACT_REGIONS] = m_chf != 0;
		valid[DRAWMODE_REGION_CONNECTIONS] = m_cset != 0;
		valid[DRAWMODE_RAW_CONTOURS] = m_cset != 0;
		valid[DRAWMODE_BOTH_CONTOURS] = m_cset != 0;
		valid[DRAWMODE_CONTOURS] = m_cset != 0;
		valid[DRAWMODE_POLYMESH] = m_pmesh != 0;
		valid[DRAWMODE_POLYMESH_DETAIL] = m_dmesh != 0;
	}
	
	int unavail = 0;
	for (int i = 0; i < MAX_DRAWMODE; ++i)
		if (!valid[i]) unavail++;

	if (unavail == MAX_DRAWMODE)
		return;

	imguiLabel("Draw");
	if (imguiCheck("Input Mesh", m_drawMode == DRAWMODE_MESH, valid[DRAWMODE_MESH]))
		m_drawMode = DRAWMODE_MESH;
	if (imguiCheck("Navmesh", m_drawMode == DRAWMODE_NAVMESH, valid[DRAWMODE_NAVMESH]))
		m_drawMode = DRAWMODE_NAVMESH;
	if (imguiCheck("Navmesh Invis", m_drawMode == DRAWMODE_NAVMESH_INVIS, valid[DRAWMODE_NAVMESH_INVIS]))
		m_drawMode = DRAWMODE_NAVMESH_INVIS;
	if (imguiCheck("Navmesh Trans", m_drawMode == DRAWMODE_NAVMESH_TRANS, valid[DRAWMODE_NAVMESH_TRANS]))
		m_drawMode = DRAWMODE_NAVMESH_TRANS;
	if (imguiCheck("Navmesh BVTree", m_drawMode == DRAWMODE_NAVMESH_BVTREE, valid[DRAWMODE_NAVMESH_BVTREE]))
		m_drawMode = DRAWMODE_NAVMESH_BVTREE;
	if (imguiCheck("Navmesh Nodes", m_drawMode == DRAWMODE_NAVMESH_NODES, valid[DRAWMODE_NAVMESH_NODES]))
		m_drawMode = DRAWMODE_NAVMESH_NODES;
	if (imguiCheck("Voxels", m_drawMode == DRAWMODE_VOXELS, valid[DRAWMODE_VOXELS]))
		m_drawMode = DRAWMODE_VOXELS;
	if (imguiCheck("Walkable Voxels", m_drawMode == DRAWMODE_VOXELS_WALKABLE, valid[DRAWMODE_VOXELS_WALKABLE]))
		m_drawMode = DRAWMODE_VOXELS_WALKABLE;
	if (imguiCheck("Compact", m_drawMode == DRAWMODE_COMPACT, valid[DRAWMODE_COMPACT]))
		m_drawMode = DRAWMODE_COMPACT;
	if (imguiCheck("Compact Distance", m_drawMode == DRAWMODE_COMPACT_DISTANCE, valid[DRAWMODE_COMPACT_DISTANCE]))
		m_drawMode = DRAWMODE_COMPACT_DISTANCE;
	if (imguiCheck("Compact Regions", m_drawMode == DRAWMODE_COMPACT_REGIONS, valid[DRAWMODE_COMPACT_REGIONS]))
		m_drawMode = DRAWMODE_COMPACT_REGIONS;
	if (imguiCheck("Region Connections", m_drawMode == DRAWMODE_REGION_CONNECTIONS, valid[DRAWMODE_REGION_CONNECTIONS]))
		m_drawMode = DRAWMODE_REGION_CONNECTIONS;
	if (imguiCheck("Raw Contours", m_drawMode == DRAWMODE_RAW_CONTOURS, valid[DRAWMODE_RAW_CONTOURS]))
		m_drawMode = DRAWMODE_RAW_CONTOURS;
	if (imguiCheck("Both Contours", m_drawMode == DRAWMODE_BOTH_CONTOURS, valid[DRAWMODE_BOTH_CONTOURS]))
		m_drawMode = DRAWMODE_BOTH_CONTOURS;
	if (imguiCheck("Contours", m_drawMode == DRAWMODE_CONTOURS, valid[DRAWMODE_CONTOURS]))
		m_drawMode = DRAWMODE_CONTOURS;
	if (imguiCheck("Poly Mesh", m_drawMode == DRAWMODE_POLYMESH, valid[DRAWMODE_POLYMESH]))
		m_drawMode = DRAWMODE_POLYMESH;
	if (imguiCheck("Poly Mesh Detail", m_drawMode == DRAWMODE_POLYMESH_DETAIL, valid[DRAWMODE_POLYMESH_DETAIL]))
		m_drawMode = DRAWMODE_POLYMESH_DETAIL;
		
	if (unavail)
	{
		imguiValue("Tick 'Keep Itermediate Results'");
		imguiValue("to see more debug mode options.");
	}
}

void Sample_SoloMesh::handleRender()
{
	if (!m_geom || !m_geom->getMesh())
		return;
	
	glEnable(GL_FOG);
	glDepthMask(GL_TRUE);

	const float texScale = 1.0f / (m_cellSize * 10.0f);
	
	if (m_drawMode != DRAWMODE_NAVMESH_TRANS)
	{
		// Draw mesh
		duDebugDrawTriMeshSlope(&m_dd, m_geom->getMesh()->getVerts(), m_geom->getMesh()->getVertCount(),
								m_geom->getMesh()->getTris(), m_geom->getMesh()->getNormals(), m_geom->getMesh()->getTriCount(),
								m_agentMaxSlope, texScale);
		m_geom->drawOffMeshConnections(&m_dd);
	}
	
	glDisable(GL_FOG);
	glDepthMask(GL_FALSE);

	// Draw bounds
	const float* bmin = m_geom->getNavMeshBoundsMin();
	const float* bmax = m_geom->getNavMeshBoundsMax();
	duDebugDrawBoxWire(&m_dd, bmin[0],bmin[1],bmin[2], bmax[0],bmax[1],bmax[2], duRGBA(255,255,255,128), 1.0f);
	m_dd.begin(DU_DRAW_POINTS, 5.0f);
	m_dd.vertex(bmin[0],bmin[1],bmin[2],duRGBA(255,255,255,128));
	m_dd.end();
	
	if (m_navMesh && m_navQuery &&
		(m_drawMode == DRAWMODE_NAVMESH ||
		m_drawMode == DRAWMODE_NAVMESH_TRANS ||
		m_drawMode == DRAWMODE_NAVMESH_BVTREE ||
		 m_drawMode == DRAWMODE_NAVMESH_NODES ||
		m_drawMode == DRAWMODE_NAVMESH_INVIS))
	{
		if (m_drawMode != DRAWMODE_NAVMESH_INVIS)
			duDebugDrawNavMeshWithClosedList(&m_dd, *m_navMesh, *m_navQuery, m_navMeshDrawFlags);
		if (m_drawMode == DRAWMODE_NAVMESH_BVTREE)
			duDebugDrawNavMeshBVTree(&m_dd, *m_navMesh);
		if (m_drawMode == DRAWMODE_NAVMESH_NODES)
			duDebugDrawNavMeshNodes(&m_dd, *m_navQuery);
		duDebugDrawNavMeshPolysWithFlags(&m_dd, *m_navMesh, SAMPLE_POLYFLAGS_DISABLED, duRGBA(0,0,0,128));
	}
		
	glDepthMask(GL_TRUE);
	
	if (m_chf && m_drawMode == DRAWMODE_COMPACT)
		duDebugDrawCompactHeightfieldSolid(&m_dd, *m_chf);

	if (m_chf && m_drawMode == DRAWMODE_COMPACT_DISTANCE)
		duDebugDrawCompactHeightfieldDistance(&m_dd, *m_chf);
	if (m_chf && m_drawMode == DRAWMODE_COMPACT_REGIONS)
		duDebugDrawCompactHeightfieldRegions(&m_dd, *m_chf);
	if (m_solid && m_drawMode == DRAWMODE_VOXELS)
	{
		glEnable(GL_FOG);
		duDebugDrawHeightfieldSolid(&m_dd, *m_solid);
		glDisable(GL_FOG);
	}
	if (m_solid && m_drawMode == DRAWMODE_VOXELS_WALKABLE)
	{
		glEnable(GL_FOG);
		duDebugDrawHeightfieldWalkable(&m_dd, *m_solid);
		glDisable(GL_FOG);
	}
	if (m_cset && m_drawMode == DRAWMODE_RAW_CONTOURS)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawRawContours(&m_dd, *m_cset);
		glDepthMask(GL_TRUE);
	}
	if (m_cset && m_drawMode == DRAWMODE_BOTH_CONTOURS)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawRawContours(&m_dd, *m_cset, 0.5f);
		duDebugDrawContours(&m_dd, *m_cset);
		glDepthMask(GL_TRUE);
	}
	if (m_cset && m_drawMode == DRAWMODE_CONTOURS)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawContours(&m_dd, *m_cset);
		glDepthMask(GL_TRUE);
	}
	if (m_chf && m_cset && m_drawMode == DRAWMODE_REGION_CONNECTIONS)
	{
		duDebugDrawCompactHeightfieldRegions(&m_dd, *m_chf);
			
		glDepthMask(GL_FALSE);
		duDebugDrawRegionConnections(&m_dd, *m_cset);
		glDepthMask(GL_TRUE);
	}
	if (m_pmesh && m_drawMode == DRAWMODE_POLYMESH)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawPolyMesh(&m_dd, *m_pmesh);
		glDepthMask(GL_TRUE);
	}
	if (m_dmesh && m_drawMode == DRAWMODE_POLYMESH_DETAIL)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawPolyMeshDetail(&m_dd, *m_dmesh);
		glDepthMask(GL_TRUE);
	}
	
	m_geom->drawConvexVolumes(&m_dd);

	if (m_tool)
		m_tool->handleRender();
	renderToolStates();

	glDepthMask(GL_TRUE);
}

void Sample_SoloMesh::handleRenderOverlay(double* proj, double* model, int* view)
{
	if (m_tool)
		m_tool->handleRenderOverlay(proj, model, view);
	renderOverlayToolStates(proj, model, view);
}

void Sample_SoloMesh::handleMeshChanged(class InputGeom* geom)
{
	Sample::handleMeshChanged(geom);

	dtFreeNavMesh(m_navMesh);
	m_navMesh = 0;

	if (m_tool)
	{
		m_tool->reset();
		m_tool->init(this);
	}
	resetToolStates();
	initToolStates(this);
}


/// @brief 构建单块(Solo)导航网格的核心函数。
/// 
/// 整个构建流程分为8个步骤：
///   Step 1: 初始化构建配置 (rcConfig)
///   Step 2: 体素化光栅化 —— 将输入三角形网格光栅化为体素高度场
///   Step 3: 过滤可行走表面 —— 移除不可行走的体素 (低矮障碍物、悬崖边缘、低净空区域)
///   Step 4: 区域划分 —— 将可行走表面划分为简单区域 (支持分水岭/单调/分层三种算法)
///   Step 5: 轮廓提取与简化 —— 沿区域边界生成轮廓线
///   Step 6: 构建多边形网格 —— 将轮廓三角化为凸多边形
///   Step 7: 构建细节网格 —— 为每个多边形添加高度细节信息
///   Step 8: 创建 Detour 导航数据 —— 将 Recast 多边形网格转换为 Detour 运行时可用的导航网格
///
/// @return 构建成功返回 true，失败返回 false
bool Sample_SoloMesh::handleBuild()
{
	// 前置检查：确保已加载输入几何体
	if (!m_geom || !m_geom->getMesh())
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Input mesh is not specified.");
		return false;
	}
	
	// 清理上一次构建的所有中间结果和最终结果
	cleanup();
	
	// 获取输入网格的包围盒、顶点数据和三角形索引数据
	const float* bmin = m_geom->getNavMeshBoundsMin();  // 包围盒最小角 (世界坐标)
	const float* bmax = m_geom->getNavMeshBoundsMax();  // 包围盒最大角 (世界坐标)
	const float* verts = m_geom->getMesh()->getVerts();  // 顶点数组 (x,y,z 交错存储)
	const int nverts = m_geom->getMesh()->getVertCount(); // 顶点数量
	const int* tris = m_geom->getMesh()->getTris();       // 三角形索引数组 (每3个索引一个三角形)
	const int ntris = m_geom->getMesh()->getTriCount();   // 三角形数量
	
	//
	// Step 1. 初始化构建配置。
	// 将GUI中的参数转换为 rcConfig 结构体中的体素单位参数。
	//
	
	// 清零配置结构体，从GUI参数初始化各项配置
	memset(&m_cfg, 0, sizeof(m_cfg));
	m_cfg.cs = m_cellSize;                                  // XZ平面体素尺寸 (Cell Size)
	m_cfg.ch = m_cellHeight;                                // Y轴体素高度 (Cell Height)
	m_cfg.walkableSlopeAngle = m_agentMaxSlope;             // 可行走的最大坡度角 (度)
	// 以下参数从世界单位转换为体素单位：
	m_cfg.walkableHeight = (int)ceilf(m_agentHeight / m_cfg.ch);   // Agent身高 (向上取整，体素数)
	m_cfg.walkableClimb = (int)floorf(m_agentMaxClimb / m_cfg.ch); // Agent可攀爬高度 (向下取整，体素数)
	m_cfg.walkableRadius = (int)ceilf(m_agentRadius / m_cfg.cs);   // Agent半径 (向上取整，体素数)
	m_cfg.maxEdgeLen = (int)(m_edgeMaxLen / m_cellSize);    // 轮廓边最大长度 (体素数)
	m_cfg.maxSimplificationError = m_edgeMaxError;          // 轮廓简化最大误差 (世界单位)
	m_cfg.minRegionArea = (int)rcSqr(m_regionMinSize);		// 最小区域面积阈值 (面积 = 边长²，体素²)
	m_cfg.mergeRegionArea = (int)rcSqr(m_regionMergeSize);	// 区域合并面积阈值 (面积 = 边长²，体素²)
	m_cfg.maxVertsPerPoly = (int)m_vertsPerPoly;            // 每个多边形最大顶点数 (通常为6)
	// 细节网格采样距离：若 detailSampleDist < 0.9 则禁用采样，否则乘以cellSize转为世界单位
	m_cfg.detailSampleDist = m_detailSampleDist < 0.9f ? 0 : m_cellSize * m_detailSampleDist;
	// 细节网格采样最大误差：以cellHeight为单位缩放
	m_cfg.detailSampleMaxError = m_cellHeight * m_detailSampleMaxError;
	
	// 设置导航网格的构建区域范围。
	// 这里使用输入网格的包围盒，也可以由用户自定义一个区域盒子。
	rcVcopy(m_cfg.bmin, bmin);  // 复制包围盒最小角
	rcVcopy(m_cfg.bmax, bmax);  // 复制包围盒最大角
	// 根据包围盒和体素尺寸，计算XZ平面上的网格宽度和高度（体素数）
	rcCalcGridSize(m_cfg.bmin, m_cfg.bmax, m_cfg.cs, &m_cfg.width, &m_cfg.height);

	// 重置构建计时器
	m_ctx->resetTimers();

	// 开始计时
	m_ctx->startTimer(RC_TIMER_TOTAL);
	
	// 输出构建日志：网格尺寸、输入顶点和三角形数量
	m_ctx->log(RC_LOG_PROGRESS, "Building navigation:");
	m_ctx->log(RC_LOG_PROGRESS, " - %d x %d cells", m_cfg.width, m_cfg.height);
	m_ctx->log(RC_LOG_PROGRESS, " - %.1fK verts, %.1fK tris", nverts/1000.0f, ntris/1000.0f);
	
	//
	// Step 2. 体素化光栅化输入多边形。
	// 将输入的三角形网格(polygon soup)转换为体素高度场(heightfield)。
	// 高度场是一个2D网格，每个格子(column)中存储沿Y轴排列的Span链表，
	// 每个Span记录了该位置实体的高度范围和区域类型。
	//
	
	// 分配体素高度场结构体
	m_solid = rcAllocHeightfield();
	if (!m_solid)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
		return false;
	}
	// 创建高度场：初始化 width*height 的Span指针数组，设置包围盒和体素尺寸
	if (!rcCreateHeightfield(m_ctx, *m_solid, m_cfg.width, m_cfg.height, m_cfg.bmin, m_cfg.bmax, m_cfg.cs, m_cfg.ch))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
		return false;
	}
	
	// 分配三角形区域类型数组。每个三角形对应一个 unsigned char 标识其区域类型。
	// 如果有多个网格需要处理，需分配能容纳最大三角形数量的数组。
	m_triareas = new unsigned char[ntris];
	if (!m_triareas)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", ntris);
		return false;
	}
	
	// 先将所有三角形区域清零
	memset(m_triareas, 0, ntris*sizeof(unsigned char));
	// 根据坡度角标记可行走的三角形：
	// 计算每个三角形法线，若法线Y分量 > cos(walkableSlopeAngle) 则标记为 RC_WALKABLE_AREA
	rcMarkWalkableTriangles(m_ctx, m_cfg.walkableSlopeAngle, verts, nverts, tris, ntris, m_triareas);
	// 将标记好的三角形光栅化到高度场中：
	// 对每个三角形，按XZ网格逐格裁剪(dividePoly)，计算Y轴高度范围，
	// 调用addSpan将Span插入/合并到对应列的链表中。
	// walkableClimb 作为合并阈值：相邻Span顶部高度差 <= walkableClimb 时合并区域标记。
	if (!rcRasterizeTriangles(m_ctx, verts, nverts, tris, m_triareas, ntris, *m_solid, m_cfg.walkableClimb))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not rasterize triangles.");
		return false;
	}

	// 如果不需要保留中间结果，立即释放三角形区域数组以节省内存
	if (!m_keepInterResults)
	{
		delete [] m_triareas;
		m_triareas = 0;
	}

	return true;
	
	//
	// Step 3. 过滤可行走表面。
	// 对光栅化后的高度场进行过滤，移除由保守光栅化产生的伪可行走区域，
	// 以及Agent不可能站立的区域。
	//
	
	// 过滤低矮悬挂障碍物：如果一个不可行走Span的顶部与其下方可行走Span的顶部
	// 高度差 <= walkableClimb，则将该Span也标记为可行走（例如台阶、路缘石等）
	if (m_filterLowHangingObstacles)
		rcFilterLowHangingWalkableObstacles(m_ctx, m_cfg.walkableClimb, *m_solid);
	// 过滤悬崖边缘：检查每个可行走Span的邻居，如果与邻居之间的高度落差
	// 超过 walkableClimb，或邻居上方净空不足 walkableHeight，则标记为不可行走
	if (m_filterLedgeSpans)
		rcFilterLedgeSpans(m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, *m_solid);
	// 过滤低净空区域：如果一个Span到其上方Span之间的净空 < walkableHeight，
	// 说明Agent站不起来，标记为不可行走
	if (m_filterWalkableLowHeightSpans)
		rcFilterWalkableLowHeightSpans(m_ctx, m_cfg.walkableHeight, *m_solid);


	//
	// Step 4. 将可行走表面划分为简单区域。
	// 首先将稀疏高度场(rcHeightfield)压缩为紧凑高度场(rcCompactHeightfield)，
	// 然后进行区域划分。
	//

	// 构建紧凑高度场：
	// - 将Span链表转为连续数组存储，提升缓存友好性
	// - 计算每个可行走Span与4个邻居的连通关系
	// - 只保留可行走的Span（上方净空 >= walkableHeight 且与邻居高度差 <= walkableClimb）
	m_chf = rcAllocCompactHeightfield();
	if (!m_chf)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
		return false;
	}
	if (!rcBuildCompactHeightfield(m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, *m_solid, *m_chf))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
		return false;
	}
	
	// 紧凑高度场构建完毕后，原始高度场不再需要，可以释放以节省内存
	if (!m_keepInterResults)
	{
		rcFreeHeightField(m_solid);
		m_solid = 0;
	}
		
	// 按Agent半径腐蚀可行走区域：
	// 从所有不可行走区域的边界向内收缩 walkableRadius 个体素，
	// 确保Agent中心不会过于靠近墙壁或障碍物边缘
	if (!rcErodeWalkableArea(m_ctx, m_cfg.walkableRadius, *m_chf))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
		return false;
	}

	// (可选) 标记用户自定义的凸多边形区域。
	// 用户可在编辑器中绘制凸多边形体积来标记特殊区域（如水域、草地、道路等），
	// 这些区域会覆盖对应位置Span的区域类型。
	const ConvexVolume* vols = m_geom->getConvexVolumes();
	for (int i  = 0; i < m_geom->getConvexVolumeCount(); ++i)
		rcMarkConvexPolyArea(m_ctx, vols[i].verts, vols[i].nverts, vols[i].hmin, vols[i].hmax, (unsigned char)vols[i].area, *m_chf);

	
	// 对高度场进行区域划分，以便后续使用简单算法三角化可行走区域。
	// 支持3种分区策略，各有优缺点：
	// 1) 分水岭分区 (Watershed)：经典Recast算法，生成最美观的网格，通常最慢，
	//    大型开阔区域的最佳选择。极端情况下可能出现孔洞或重叠。
	// 2) 单调分区 (Monotone)：最快，保证无孔洞无重叠，但会产生细长多边形，
	//    可能导致路径绕行。适合需要快速生成NavMesh的场景。
	// 3) 分层分区 (Layer)：速度较快，无重叠区域，三角化质量优于单调分区，
	//    适合中小尺寸tile的分块NavMesh。
	
	if (m_partitionType == SAMPLE_PARTITION_WATERSHED)
	{
		// 分水岭分区：先计算距离场（每个可行走Span到最近边界的距离），
		// 然后基于距离场进行"泛洪"式的区域划分，生成无孔洞无重叠的区域。
		if (!rcBuildDistanceField(m_ctx, *m_chf))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build distance field.");
			return false;
		}
		
		// 基于距离场将可行走表面划分为若干简单区域。
		// 参数：0=borderSize(边界大小)，minRegionArea=最小区域面积，mergeRegionArea=合并区域面积阈值
		if (!rcBuildRegions(m_ctx, *m_chf, 0, m_cfg.minRegionArea, m_cfg.mergeRegionArea))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build watershed regions.");
			return false;
		}
	}
	else if (m_partitionType == SAMPLE_PARTITION_MONOTONE)
	{
		// 单调分区：不需要距离场，直接将可行走表面划分为无孔洞无重叠的区域。
		// 速度最快，但生成的多边形较细长，可能导致路径绕行。
		if (!rcBuildRegionsMonotone(m_ctx, *m_chf, 0, m_cfg.minRegionArea, m_cfg.mergeRegionArea))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build monotone regions.");
			return false;
		}
	}
	else // SAMPLE_PARTITION_LAYERS
	{
		// 分层分区：将可行走表面划分为不重叠的区域，
		// 依赖三角化代码处理孔洞，生成质量优于单调分区的三角形。
		if (!rcBuildLayerRegions(m_ctx, *m_chf, 0, m_cfg.minRegionArea))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build layer regions.");
			return false;
		}
	}
	
	//
	// Step 5. 提取并简化区域轮廓。
	// 沿每个区域的边界提取轮廓线，然后使用 Douglas-Peucker 算法简化轮廓，
	// 减少顶点数量，同时控制简化误差不超过 maxSimplificationError。
	//
	
	// 分配轮廓集合结构体
	m_cset = rcAllocContourSet();
	if (!m_cset)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
		return false;
	}
	// 构建轮廓：
	// - maxSimplificationError: 简化误差阈值，值越大轮廓越简化
	// - maxEdgeLen: 轮廓边最大长度，超过此长度的边会被细分
	if (!rcBuildContours(m_ctx, *m_chf, m_cfg.maxSimplificationError, m_cfg.maxEdgeLen, *m_cset))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
		return false;
	}
	
	//
	// Step 6. 从轮廓构建多边形网格。
	// 将每个轮廓三角化，然后合并三角形为凸多边形（最多 maxVertsPerPoly 个顶点）。
	// 输出的多边形网格(PolyMesh)是导航网格的核心数据结构。
	//
	
	// 分配多边形网格结构体
	m_pmesh = rcAllocPolyMesh();
	if (!m_pmesh)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmesh'.");
		return false;
	}
	// 构建多边形网格：对每个轮廓进行三角化，再将相邻三角形合并为凸多边形
	if (!rcBuildPolyMesh(m_ctx, *m_cset, m_cfg.maxVertsPerPoly, *m_pmesh))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours.");
		return false;
	}
	
	//
	// Step 7. 构建细节网格 (Detail Mesh)。
	// 多边形网格中的多边形是平面的，细节网格为每个多边形添加额外的高度采样点，
	// 使得在运行时可以获取每个多边形上任意点的近似高度值。
	//
	
	// 分配细节网格结构体
	m_dmesh = rcAllocPolyMeshDetail();
	if (!m_dmesh)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmdtl'.");
		return false;
	}

	// 构建细节网格：
	// - detailSampleDist: 采样间距，在多边形内部按此间距采样高度点
	// - detailSampleMaxError: 最大高度误差，超过此值的位置会增加额外顶点
	if (!rcBuildPolyMeshDetail(m_ctx, *m_pmesh, *m_chf, m_cfg.detailSampleDist, m_cfg.detailSampleMaxError, *m_dmesh))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build detail mesh.");
		return false;
	}

	// 细节网格构建完毕后，紧凑高度场和轮廓集合不再需要，可以释放
	if (!m_keepInterResults)
	{
		rcFreeCompactHeightfield(m_chf);
		m_chf = 0;
		rcFreeContourSet(m_cset);
		m_cset = 0;
	}

	// 至此，Recast 的导航网格数据已构建完毕，可通过 m_pmesh 访问多边形网格。
	// 参见 duDebugDrawPolyMesh 或 dtCreateNavMeshData 了解如何使用这些数据。
	
	//
	// (可选) Step 8. 从 Recast 多边形网格创建 Detour 导航数据。
	// Detour 是运行时寻路库，需要将 Recast 的构建结果转换为 Detour 可用的格式。
	//
	
	// 检查每个多边形的最大顶点数是否在 Detour 支持的范围内（DT_VERTS_PER_POLYGON=6）
	if (m_cfg.maxVertsPerPoly <= DT_VERTS_PER_POLYGON)
	{
		unsigned char* navData = 0;   // Detour 导航数据的序列化缓冲区
		int navDataSize = 0;          // 导航数据大小（字节）

		// 将区域类型(area)转换为多边形标志(flags)。
		// 区域类型决定了多边形的语义（地面、水域、门等），
		// 标志位决定了寻路时的通行能力（可行走、可游泳、可开门等）。
		for (int i = 0; i < m_pmesh->npolys; ++i)
		{
			// 将通用可行走区域 RC_WALKABLE_AREA 映射为具体的地面区域
			if (m_pmesh->areas[i] == RC_WALKABLE_AREA)
				m_pmesh->areas[i] = SAMPLE_POLYAREA_GROUND;
				
			// 地面、草地、道路区域 → 设置可行走标志
			if (m_pmesh->areas[i] == SAMPLE_POLYAREA_GROUND ||
				m_pmesh->areas[i] == SAMPLE_POLYAREA_GRASS ||
				m_pmesh->areas[i] == SAMPLE_POLYAREA_ROAD)
			{
				m_pmesh->flags[i] = SAMPLE_POLYFLAGS_WALK;
			}
			// 水域区域 → 设置可游泳标志
			else if (m_pmesh->areas[i] == SAMPLE_POLYAREA_WATER)
			{
				m_pmesh->flags[i] = SAMPLE_POLYFLAGS_SWIM;
			}
			// 门区域 → 同时设置可行走和可开门标志
			else if (m_pmesh->areas[i] == SAMPLE_POLYAREA_DOOR)
			{
				m_pmesh->flags[i] = SAMPLE_POLYFLAGS_WALK | SAMPLE_POLYFLAGS_DOOR;
			}
		}


		// 填充 Detour 导航网格创建参数
		dtNavMeshCreateParams params;
		memset(&params, 0, sizeof(params));
		// 多边形网格数据
		params.verts = m_pmesh->verts;             // 顶点数组
		params.vertCount = m_pmesh->nverts;        // 顶点数量
		params.polys = m_pmesh->polys;             // 多边形索引数组
		params.polyAreas = m_pmesh->areas;         // 多边形区域类型
		params.polyFlags = m_pmesh->flags;         // 多边形通行标志
		params.polyCount = m_pmesh->npolys;        // 多边形数量
		params.nvp = m_pmesh->nvp;                 // 每个多边形最大顶点数
		// 细节网格数据（用于高度查询）
		params.detailMeshes = m_dmesh->meshes;     // 细节子网格描述数组
		params.detailVerts = m_dmesh->verts;       // 细节顶点数组
		params.detailVertsCount = m_dmesh->nverts; // 细节顶点数量
		params.detailTris = m_dmesh->tris;         // 细节三角形数组
		params.detailTriCount = m_dmesh->ntris;    // 细节三角形数量
		// Off-Mesh连接数据（用于跳跃点、传送门等特殊连接）
		params.offMeshConVerts = m_geom->getOffMeshConnectionVerts();   // 连接端点坐标
		params.offMeshConRad = m_geom->getOffMeshConnectionRads();      // 连接半径
		params.offMeshConDir = m_geom->getOffMeshConnectionDirs();      // 连接方向（单向/双向）
		params.offMeshConAreas = m_geom->getOffMeshConnectionAreas();   // 连接区域类型
		params.offMeshConFlags = m_geom->getOffMeshConnectionFlags();   // 连接通行标志
		params.offMeshConUserID = m_geom->getOffMeshConnectionId();     // 连接用户ID
		params.offMeshConCount = m_geom->getOffMeshConnectionCount();   // 连接数量
		// Agent参数（世界单位，用于运行时查询）
		params.walkableHeight = m_agentHeight;     // Agent身高
		params.walkableRadius = m_agentRadius;     // Agent半径
		params.walkableClimb = m_agentMaxClimb;    // Agent最大攀爬高度
		// 包围盒和体素参数
		rcVcopy(params.bmin, m_pmesh->bmin);       // 导航网格包围盒最小角
		rcVcopy(params.bmax, m_pmesh->bmax);       // 导航网格包围盒最大角
		params.cs = m_cfg.cs;                      // XZ体素尺寸
		params.ch = m_cfg.ch;                      // Y体素高度
		params.buildBvTree = true;                 // 构建BVTree（用于加速空间查询）
		
		// 将所有参数序列化为 Detour 导航网格二进制数据
		if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
		{
			m_ctx->log(RC_LOG_ERROR, "Could not build Detour navmesh.");
			return false;
		}
		
		// 分配 Detour 导航网格对象
		m_navMesh = dtAllocNavMesh();
		if (!m_navMesh)
		{
			dtFree(navData);
			m_ctx->log(RC_LOG_ERROR, "Could not create Detour navmesh");
			return false;
		}
		
		dtStatus status;
		
		// 用序列化数据初始化导航网格。
		// DT_TILE_FREE_DATA 标志表示导航网格拥有 navData 的所有权，
		// 在销毁时会自动释放该内存。
		status = m_navMesh->init(navData, navDataSize, DT_TILE_FREE_DATA);
		if (dtStatusFailed(status))
		{
			dtFree(navData);
			m_ctx->log(RC_LOG_ERROR, "Could not init Detour navmesh");
			return false;
		}
		
		// 初始化导航网格查询对象，最大节点池大小为2048
		status = m_navQuery->init(m_navMesh, 2048);
		if (dtStatusFailed(status))
		{
			m_ctx->log(RC_LOG_ERROR, "Could not init Detour navmesh query");
			return false;
		}
	}
	
	// 停止总计时器
	m_ctx->stopTimer(RC_TIMER_TOTAL);

	// 输出各阶段的构建耗时统计
	duLogBuildTimes(*m_ctx, m_ctx->getAccumulatedTime(RC_TIMER_TOTAL));
	// 输出最终多边形网格的顶点数和多边形数
	m_ctx->log(RC_LOG_PROGRESS, ">> Polymesh: %d vertices  %d polygons", m_pmesh->nverts, m_pmesh->npolys);
	
	// 记录总构建耗时（毫秒），用于GUI显示
	m_totalBuildTimeMs = m_ctx->getAccumulatedTime(RC_TIMER_TOTAL)/1000.0f;
	
	// 构建完成后，重新初始化当前选中的工具（如寻路测试工具）
	if (m_tool)
		m_tool->init(this);
	// 初始化所有工具的状态
	initToolStates(this);

	return true;
}
