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
#include "DebugDraw.h"
#include "RecastDebugDraw.h"
#include "Recast.h"

void duDebugDrawTriMesh(duDebugDraw* dd, const float* verts, int /*nverts*/,
						const int* tris, const float* normals, int ntris,
						const unsigned char* flags, const float texScale)
{
	if (!dd) return;
	if (!verts) return;
	if (!tris) return;
	if (!normals) return;

	float uva[2];
	float uvb[2];
	float uvc[2];

	const unsigned int unwalkable = duRGBA(192,128,0,255);

	dd->texture(true);

	dd->begin(DU_DRAW_TRIS);
	for (int i = 0; i < ntris*3; i += 3)
	{
		const float* norm = &normals[i];
		unsigned int color;
		unsigned char a = (unsigned char)(220*(2+norm[0]+norm[1])/4);
		if (flags && !flags[i/3])
			color = duLerpCol(duRGBA(a,a,a,255), unwalkable, 64);
		else
			color = duRGBA(a,a,a,255);

		const float* va = &verts[tris[i+0]*3];
		const float* vb = &verts[tris[i+1]*3];
		const float* vc = &verts[tris[i+2]*3];
		
		int ax = 0, ay = 0;
		if (rcAbs(norm[1]) > rcAbs(norm[ax]))
			ax = 1;
		if (rcAbs(norm[2]) > rcAbs(norm[ax]))
			ax = 2;
		ax = (1<<ax)&3; // +1 mod 3
		ay = (1<<ax)&3; // +1 mod 3

		uva[0] = va[ax]*texScale;
		uva[1] = va[ay]*texScale;
		uvb[0] = vb[ax]*texScale;
		uvb[1] = vb[ay]*texScale;
		uvc[0] = vc[ax]*texScale;
		uvc[1] = vc[ay]*texScale;
		
		dd->vertex(va, color, uva);
		dd->vertex(vb, color, uvb);
		dd->vertex(vc, color, uvc);
	}
	dd->end();
	dd->texture(false);
}

void duDebugDrawTriMeshSlope(duDebugDraw* dd, const float* verts, int /*nverts*/,
							 const int* tris, const float* normals, int ntris,
							 const float walkableSlopeAngle, const float texScale)
{
	if (!dd) return;
	if (!verts) return;
	if (!tris) return;
	if (!normals) return;
	
	const float walkableThr = cosf(walkableSlopeAngle/180.0f*DU_PI);
	
	float uva[2];
	float uvb[2];
	float uvc[2];
	
	dd->texture(true);

	const unsigned int unwalkable = duRGBA(192,128,0,255);
	
	dd->begin(DU_DRAW_TRIS);
	for (int i = 0; i < ntris*3; i += 3)
	{
		const float* norm = &normals[i];
		unsigned int color;
		unsigned char a = (unsigned char)(220*(2+norm[0]+norm[1])/4);
		if (norm[1] < walkableThr)
			color = duLerpCol(duRGBA(a,a,a,255), unwalkable, 64);
		else
			color = duRGBA(a,a,a,255);
		
		const float* va = &verts[tris[i+0]*3];
		const float* vb = &verts[tris[i+1]*3];
		const float* vc = &verts[tris[i+2]*3];
		
		int ax = 0, ay = 0;
		if (rcAbs(norm[1]) > rcAbs(norm[ax]))
			ax = 1;
		if (rcAbs(norm[2]) > rcAbs(norm[ax]))
			ax = 2;
		ax = (1<<ax)&3; // +1 mod 3
		ay = (1<<ax)&3; // +1 mod 3
		
		uva[0] = va[ax]*texScale;
		uva[1] = va[ay]*texScale;
		uvb[0] = vb[ax]*texScale;
		uvb[1] = vb[ay]*texScale;
		uvc[0] = vc[ax]*texScale;
		uvc[1] = vc[ay]*texScale;
		
		dd->vertex(va, color, uva);
		dd->vertex(vb, color, uvb);
		dd->vertex(vc, color, uvc);
	}
	dd->end();

	dd->texture(false);
}

/// @brief 以实心盒子的方式绘制高度场（rcHeightfield）中的所有体素 Span。
///
/// 该函数遍历高度场中每个 (x, z) 网格列的所有 Span，并将每个 Span 渲染为一个
/// 三维立方体盒子。盒子的 XZ 大小为一个体素格子（cs × cs），Y 方向范围为
/// [smin*ch, smax*ch]（相对于高度场原点 bmin）。
///
/// 所有 Span 使用统一的白色着色，不区分可行走/不可行走区域。
/// 这是最基础的体素可视化模式，主要用于查看光栅化（体素化）后生成的原始高度场数据。
///
/// 与 duDebugDrawHeightfieldWalkable 的区别：
///   - 本函数：所有 Span 统一白色，不考虑 area 标记。
///   - Walkable 版本：根据 Span 的 area 值着不同颜色（可行走/不可行走/自定义区域）。
///
/// 数据来源：只需完成 Step 2（光栅化）即可调用本函数，无需后续过滤或紧凑化步骤。
///
/// @param[in] dd  调试绘制接口指针，用于提交顶点和绘制命令。如果为 NULL 则直接返回。
/// @param[in] hf  要绘制的高度场，包含体素网格数据。
void duDebugDrawHeightfieldSolid(duDebugDraw* dd, const rcHeightfield& hf)
{
	// 空指针检查：如果没有有效的绘制接口，直接返回
	if (!dd) return;

	// 获取高度场的世界坐标原点（AABB 最小角），用于将体素坐标转换为世界坐标
	// orig[0] = X 轴起始位置，orig[1] = Y 轴起始位置（最低高度），orig[2] = Z 轴起始位置
	const float* orig = hf.bmin;

	// cs (cell size)：XZ 平面上每个体素格子的边长（水平分辨率），单位：世界坐标
	const float cs = hf.cs;

	// ch (cell height)：Y 轴方向每个体素的高度（垂直分辨率），单位：世界坐标
	// Span 的 smin/smax 是以 ch 为单位的整数值，乘以 ch 得到实际世界高度
	const float ch = hf.ch;
	
	// w：高度场在 X 轴方向的网格列数（体素数量）
	const int w = hf.width;

	// h：高度场在 Z 轴方向的网格行数（体素数量）
	// 注意：这里的 h 是 XZ 平面的 Z 方向大小，不是 Y 轴高度
	const int h = hf.height;
		
	// 定义立方体六个面的颜色数组
	// 索引含义：[0]=顶面, [1]=底面, [2]=正面, [3]=背面, [4]=右面, [5]=左面
	// duCalcBoxColors 会根据输入的顶面颜色和侧面颜色，为每个面计算出带有明暗差异的颜色
	// （通过 duMultCol 乘以不同的亮度系数：顶面250, 底面140, 各侧面165/217）
	// 这里顶面和侧面都传入白色(255,255,255,255)，最终效果是白色基调带有光照明暗感
	unsigned int fcol[6];
	duCalcBoxColors(fcol, duRGBA(255,255,255,255), duRGBA(255,255,255,255));
	
	// 开始提交四边形（QUADS）绘制批次
	// duAppendBox 内部会为每个面提交 4 个顶点，每个面是一个四边形
	// 一个立方体 = 6 个面 = 24 个顶点
	dd->begin(DU_DRAW_QUADS);
	
	// 遍历高度场的每一个 (x, z) 网格列
	// y 对应 Z 轴方向的索引（Recast 中 y 变量名对应世界 Z 轴）
	for (int y = 0; y < h; ++y)
	{
		// x 对应 X 轴方向的索引
		for (int x = 0; x < w; ++x)
		{
			// 将体素网格索引 (x, y) 转换为世界坐标中的 XZ 位置
			// fx：当前格子左下角的世界 X 坐标
			float fx = orig[0] + x*cs;

			// fz：当前格子左下角的世界 Z 坐标
			float fz = orig[2] + y*cs;

			// 获取当前 (x, z) 列的 Span 链表头指针x|{"pl:okk jazsaa  ｝｛｝
			// spans 数组的索引方式为 x + z * width（行优先存储）
			// 链表中 Span 按 smin 从小到大排列（从低到高）
			const rcSpan* s = hf.spans[x + y*w];

			// 遍历当前列中的所有 Span（沿 Y 轴从低到高）
			while (s)
			{
				// 为当前 Span 绘制一个实心立方体盒子
				// 盒子参数（世界坐标）：
				//   最小角：(fx,                    orig[1] + smin*ch, fz     )
				//   最大角：(fx + cs,               orig[1] + smax*ch, fz + cs)
				// 其中：
				//   - fx ~ fx+cs：X 方向跨一个格子宽度
				//   - orig[1]+smin*ch ~ orig[1]+smax*ch：Y 方向从 Span 底部到顶部
				//   - fz ~ fz+cs：Z 方向跨一个格子宽度
				// fcol 为六个面的颜色数组，实现简单的明暗效果
				duAppendBox(dd, fx, orig[1]+s->smin*ch, fz, fx+cs, orig[1] + s->smax*ch, fz+cs, fcol);

				// 移动到同一列中下一个更高位置的 Span
				s = s->next;
			}
		}
	}

	// 结束绘制批次，将积累的所有四边形顶点提交给 GPU 渲染
	dd->end();
}

void duDebugDrawHeightfieldWalkable(duDebugDraw* dd, const rcHeightfield& hf)
{
	if (!dd) return;

	const float* orig = hf.bmin;
	const float cs = hf.cs;
	const float ch = hf.ch;
	
	const int w = hf.width;
	const int h = hf.height;
	
	unsigned int fcol[6];
	duCalcBoxColors(fcol, duRGBA(255,255,255,255), duRGBA(217,217,217,255));

	dd->begin(DU_DRAW_QUADS);
	
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			float fx = orig[0] + x*cs;
			float fz = orig[2] + y*cs;
			const rcSpan* s = hf.spans[x + y*w];
			while (s)
			{
				if (s->area == RC_WALKABLE_AREA)
					fcol[0] = duRGBA(64,128,160,255);
				else if (s->area == RC_NULL_AREA)
					fcol[0] = duRGBA(64,64,64,255);
				else
					fcol[0] = duMultCol(dd->areaToCol(s->area), 200);
				
				duAppendBox(dd, fx, orig[1]+s->smin*ch, fz, fx+cs, orig[1] + s->smax*ch, fz+cs, fcol);
				s = s->next;
			}
		}
	}
	
	dd->end();
}

void duDebugDrawCompactHeightfieldSolid(duDebugDraw* dd, const rcCompactHeightfield& chf)
{
	if (!dd) return;

	const float cs = chf.cs;
	const float ch = chf.ch;

	dd->begin(DU_DRAW_QUADS);
	
	for (int y = 0; y < chf.height; ++y)
	{
		for (int x = 0; x < chf.width; ++x)
		{
			const float fx = chf.bmin[0] + x*cs;
			const float fz = chf.bmin[2] + y*cs;
			const rcCompactCell& c = chf.cells[x+y*chf.width];

			for (unsigned i = c.index, ni = c.index+c.count; i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];

				const unsigned char area = chf.areas[i];
				unsigned int color;
				if (area == RC_WALKABLE_AREA)
					color = duRGBA(0,192,255,64);
				else if (area == RC_NULL_AREA)
					color = duRGBA(0,0,0,64);
				else
					color = dd->areaToCol(area);
				
				const float fy = chf.bmin[1] + (s.y+1)*ch;
				dd->vertex(fx, fy, fz, color);
				dd->vertex(fx, fy, fz+cs, color);
				dd->vertex(fx+cs, fy, fz+cs, color);
				dd->vertex(fx+cs, fy, fz, color);
			}
		}
	}
	dd->end();
}

void duDebugDrawCompactHeightfieldRegions(duDebugDraw* dd, const rcCompactHeightfield& chf)
{
	if (!dd) return;

	const float cs = chf.cs;
	const float ch = chf.ch;

	dd->begin(DU_DRAW_QUADS);

	for (int y = 0; y < chf.height; ++y)
	{
		for (int x = 0; x < chf.width; ++x)
		{
			const float fx = chf.bmin[0] + x*cs;
			const float fz = chf.bmin[2] + y*cs;
			const rcCompactCell& c = chf.cells[x+y*chf.width];
			
			for (unsigned i = c.index, ni = c.index+c.count; i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				const float fy = chf.bmin[1] + (s.y)*ch;
				unsigned int color;
				if (s.reg)
					color = duIntToCol(s.reg, 192);
				else
					color = duRGBA(0,0,0,64);

				dd->vertex(fx, fy, fz, color);
				dd->vertex(fx, fy, fz+cs, color);
				dd->vertex(fx+cs, fy, fz+cs, color);
				dd->vertex(fx+cs, fy, fz, color);
			}
		}
	}
	
	dd->end();
}


void duDebugDrawCompactHeightfieldDistance(duDebugDraw* dd, const rcCompactHeightfield& chf)
{
	if (!dd) return;
	if (!chf.dist) return;
		
	const float cs = chf.cs;
	const float ch = chf.ch;
			
	float maxd = chf.maxDistance;
	if (maxd < 1.0f) maxd = 1;
	const float dscale = 255.0f / maxd;
	
	dd->begin(DU_DRAW_QUADS);
	
	for (int y = 0; y < chf.height; ++y)
	{
		for (int x = 0; x < chf.width; ++x)
		{
			const float fx = chf.bmin[0] + x*cs;
			const float fz = chf.bmin[2] + y*cs;
			const rcCompactCell& c = chf.cells[x+y*chf.width];
			
			for (unsigned i = c.index, ni = c.index+c.count; i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				const float fy = chf.bmin[1] + (s.y+1)*ch;
				const unsigned char cd = (unsigned char)(chf.dist[i] * dscale);
				const unsigned int color = duRGBA(cd,cd,cd,255);
				dd->vertex(fx, fy, fz, color);
				dd->vertex(fx, fy, fz+cs, color);
				dd->vertex(fx+cs, fy, fz+cs, color);
				dd->vertex(fx+cs, fy, fz, color);
			}
		}
	}
	dd->end();
}

static void drawLayerPortals(duDebugDraw* dd, const rcHeightfieldLayer* layer)
{
	const float cs = layer->cs;
	const float ch = layer->ch;
	const int w = layer->width;
	const int h = layer->height;
	
	unsigned int pcol = duRGBA(255,255,255,255);
	
	const int segs[4*4] = {0,0,0,1, 0,1,1,1, 1,1,1,0, 1,0,0,0};
	
	// Layer portals
	dd->begin(DU_DRAW_LINES, 2.0f);
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const int idx = x+y*w;
			const int lh = (int)layer->heights[idx];
			if (lh == 255) continue;
			
			for (int dir = 0; dir < 4; ++dir)
			{
				if (layer->cons[idx] & (1<<(dir+4)))
				{
					const int* seg = &segs[dir*4];
					const float ax = layer->bmin[0] + (x+seg[0])*cs;
					const float ay = layer->bmin[1] + (lh+2)*ch;
					const float az = layer->bmin[2] + (y+seg[1])*cs;
					const float bx = layer->bmin[0] + (x+seg[2])*cs;
					const float by = layer->bmin[1] + (lh+2)*ch;
					const float bz = layer->bmin[2] + (y+seg[3])*cs;
					dd->vertex(ax, ay, az, pcol);
					dd->vertex(bx, by, bz, pcol);
				}
			}
		}
	}
	dd->end();
}

void duDebugDrawHeightfieldLayer(duDebugDraw* dd, const struct rcHeightfieldLayer& layer, const int idx)
{
	const float cs = layer.cs;
	const float ch = layer.ch;
	const int w = layer.width;
	const int h = layer.height;
	
	unsigned int color = duIntToCol(idx+1, 255);
	
	// Layer bounds
	float bmin[3], bmax[3];
	bmin[0] = layer.bmin[0] + layer.minx*cs;
	bmin[1] = layer.bmin[1];
	bmin[2] = layer.bmin[2] + layer.miny*cs;
	bmax[0] = layer.bmin[0] + (layer.maxx+1)*cs;
	bmax[1] = layer.bmax[1];
	bmax[2] = layer.bmin[2] + (layer.maxy+1)*cs;
	duDebugDrawBoxWire(dd, bmin[0],bmin[1],bmin[2], bmax[0],bmax[1],bmax[2], duTransCol(color,128), 2.0f);
	
	// Layer height
	dd->begin(DU_DRAW_QUADS);
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const int lidx = x+y*w;
			const int lh = (int)layer.heights[lidx];
			if (h == 0xff) continue;
			const unsigned char area = layer.areas[lidx];
			
			unsigned int col;
			if (area == RC_WALKABLE_AREA)
				col = duLerpCol(color, duRGBA(0,192,255,64), 32);
			else if (area == RC_NULL_AREA)
				col = duLerpCol(color, duRGBA(0,0,0,64), 32);
			else
				col = duLerpCol(color, dd->areaToCol(area), 32);
			
			const float fx = layer.bmin[0] + x*cs;
			const float fy = layer.bmin[1] + (lh+1)*ch;
			const float fz = layer.bmin[2] + y*cs;
			
			dd->vertex(fx, fy, fz, col);
			dd->vertex(fx, fy, fz+cs, col);
			dd->vertex(fx+cs, fy, fz+cs, col);
			dd->vertex(fx+cs, fy, fz, col);
		}
	}
	dd->end();
	
	// Portals
	drawLayerPortals(dd, &layer);
}

void duDebugDrawHeightfieldLayers(duDebugDraw* dd, const struct rcHeightfieldLayerSet& lset)
{
	if (!dd) return;
	for (int i = 0; i < lset.nlayers; ++i)
		duDebugDrawHeightfieldLayer(dd, lset.layers[i], i);
}

/*
void duDebugDrawLayerContours(duDebugDraw* dd, const struct rcLayerContourSet& lcset)
{
	if (!dd) return;
	
	const float* orig = lcset.bmin;
	const float cs = lcset.cs;
	const float ch = lcset.ch;
	
	const unsigned char a = 255;// (unsigned char)(alpha*255.0f);
	
	const int offs[2*4] = {-1,0, 0,1, 1,0, 0,-1};

	dd->begin(DU_DRAW_LINES, 2.0f);
	
	for (int i = 0; i < lcset.nconts; ++i)
	{
		const rcLayerContour& c = lcset.conts[i];
		unsigned int color = 0;

		color = duIntToCol(i, a);

		for (int j = 0; j < c.nverts; ++j)
		{
			const int k = (j+1) % c.nverts;
			const unsigned char* va = &c.verts[j*4];
			const unsigned char* vb = &c.verts[k*4];
			const float ax = orig[0] + va[0]*cs;
			const float ay = orig[1] + (va[1]+1+(i&1))*ch;
			const float az = orig[2] + va[2]*cs;
			const float bx = orig[0] + vb[0]*cs;
			const float by = orig[1] + (vb[1]+1+(i&1))*ch;
			const float bz = orig[2] + vb[2]*cs;
			unsigned int col = color;
			if ((va[3] & 0xf) != 0xf)
			{
				col = duRGBA(255,255,255,128);
				int d = va[3] & 0xf;
				
				const float cx = (ax+bx)*0.5f;
				const float cy = (ay+by)*0.5f;
				const float cz = (az+bz)*0.5f;
				
				const float dx = cx + offs[d*2+0]*2*cs;
				const float dy = cy;
				const float dz = cz + offs[d*2+1]*2*cs;
				
				dd->vertex(cx,cy,cz,duRGBA(255,0,0,255));
				dd->vertex(dx,dy,dz,duRGBA(255,0,0,255));
			}
			
			duAppendArrow(dd, ax,ay,az, bx,by,bz, 0.0f, cs*0.5f, col);
		}
	}
	dd->end();
	
	dd->begin(DU_DRAW_POINTS, 4.0f);	
	
	for (int i = 0; i < lcset.nconts; ++i)
	{
		const rcLayerContour& c = lcset.conts[i];
		unsigned int color = 0;
		
		for (int j = 0; j < c.nverts; ++j)
		{
			const unsigned char* va = &c.verts[j*4];

			color = duDarkenCol(duIntToCol(i, a));
			if (va[3] & 0x80)
				color = duRGBA(255,0,0,255);

			float fx = orig[0] + va[0]*cs;
			float fy = orig[1] + (va[1]+1+(i&1))*ch;
			float fz = orig[2] + va[2]*cs;
			dd->vertex(fx,fy,fz, color);
		}
	}
	dd->end();
}

void duDebugDrawLayerPolyMesh(duDebugDraw* dd, const struct rcLayerPolyMesh& lmesh)
{
	if (!dd) return;
	
	const int nvp = lmesh.nvp;
	const float cs = lmesh.cs;
	const float ch = lmesh.ch;
	const float* orig = lmesh.bmin;
	
	const int offs[2*4] = {-1,0, 0,1, 1,0, 0,-1};

	dd->begin(DU_DRAW_TRIS);
	
	for (int i = 0; i < lmesh.npolys; ++i)
	{
		const unsigned short* p = &lmesh.polys[i*nvp*2];
		
		unsigned int color;
		if (lmesh.areas[i] == RC_WALKABLE_AREA)
			color = duRGBA(0,192,255,64);
		else if (lmesh.areas[i] == RC_NULL_AREA)
			color = duRGBA(0,0,0,64);
		else
			color = duIntToCol(lmesh.areas[i], 255);
		
		unsigned short vi[3];
		for (int j = 2; j < nvp; ++j)
		{
			if (p[j] == RC_MESH_NULL_IDX) break;
			vi[0] = p[0];
			vi[1] = p[j-1];
			vi[2] = p[j];
			for (int k = 0; k < 3; ++k)
			{
				const unsigned short* v = &lmesh.verts[vi[k]*3];
				const float x = orig[0] + v[0]*cs;
				const float y = orig[1] + (v[1]+1)*ch;
				const float z = orig[2] + v[2]*cs;
				dd->vertex(x,y,z, color);
			}
		}
	}
	dd->end();
	
	// Draw neighbours edges
	const unsigned int coln = duRGBA(0,48,64,32);
	dd->begin(DU_DRAW_LINES, 1.5f);
	for (int i = 0; i < lmesh.npolys; ++i)
	{
		const unsigned short* p = &lmesh.polys[i*nvp*2];
		for (int j = 0; j < nvp; ++j)
		{
			if (p[j] == RC_MESH_NULL_IDX) break;
			if (p[nvp+j] & 0x8000) continue;
			const int nj = (j+1 >= nvp || p[j+1] == RC_MESH_NULL_IDX) ? 0 : j+1; 
			int vi[2] = {p[j], p[nj]};
			
			for (int k = 0; k < 2; ++k)
			{
				const unsigned short* v = &lmesh.verts[vi[k]*3];
				const float x = orig[0] + v[0]*cs;
				const float y = orig[1] + (v[1]+1)*ch + 0.1f;
				const float z = orig[2] + v[2]*cs;
				dd->vertex(x, y, z, coln);
			}
		}
	}
	dd->end();
	
	// Draw boundary edges
	const unsigned int colb = duRGBA(0,48,64,220);
	dd->begin(DU_DRAW_LINES, 2.5f);
	for (int i = 0; i < lmesh.npolys; ++i)
	{
		const unsigned short* p = &lmesh.polys[i*nvp*2];
		for (int j = 0; j < nvp; ++j)
		{
			if (p[j] == RC_MESH_NULL_IDX) break;
			if ((p[nvp+j] & 0x8000) == 0) continue;
			const int nj = (j+1 >= nvp || p[j+1] == RC_MESH_NULL_IDX) ? 0 : j+1; 
			int vi[2] = {p[j], p[nj]};
			
			unsigned int col = colb;
			if ((p[nvp+j] & 0xf) != 0xf)
			{
				const unsigned short* va = &lmesh.verts[vi[0]*3];
				const unsigned short* vb = &lmesh.verts[vi[1]*3];

				const float ax = orig[0] + va[0]*cs;
				const float ay = orig[1] + (va[1]+1+(i&1))*ch;
				const float az = orig[2] + va[2]*cs;
				const float bx = orig[0] + vb[0]*cs;
				const float by = orig[1] + (vb[1]+1+(i&1))*ch;
				const float bz = orig[2] + vb[2]*cs;
				
				const float cx = (ax+bx)*0.5f;
				const float cy = (ay+by)*0.5f;
				const float cz = (az+bz)*0.5f;
				
				int d = p[nvp+j] & 0xf;
				
				const float dx = cx + offs[d*2+0]*2*cs;
				const float dy = cy;
				const float dz = cz + offs[d*2+1]*2*cs;
				
				dd->vertex(cx,cy,cz,duRGBA(255,0,0,255));
				dd->vertex(dx,dy,dz,duRGBA(255,0,0,255));
				
				col = duRGBA(255,255,255,128);
			}
							 
			for (int k = 0; k < 2; ++k)
			{
				const unsigned short* v = &lmesh.verts[vi[k]*3];
				const float x = orig[0] + v[0]*cs;
				const float y = orig[1] + (v[1]+1)*ch + 0.1f;
				const float z = orig[2] + v[2]*cs;
				dd->vertex(x, y, z, col);
			}
		}
	}
	dd->end();
	
	dd->begin(DU_DRAW_POINTS, 3.0f);
	const unsigned int colv = duRGBA(0,0,0,220);
	for (int i = 0; i < lmesh.nverts; ++i)
	{
		const unsigned short* v = &lmesh.verts[i*3];
		const float x = orig[0] + v[0]*cs;
		const float y = orig[1] + (v[1]+1)*ch + 0.1f;
		const float z = orig[2] + v[2]*cs;
		dd->vertex(x,y,z, colv);
	}
	dd->end();
}
*/

static void getContourCenter(const rcContour* cont, const float* orig, float cs, float ch, float* center)
{
	center[0] = 0;
	center[1] = 0;
	center[2] = 0;
	if (!cont->nverts)
		return;
	for (int i = 0; i < cont->nverts; ++i)
	{
		const int* v = &cont->verts[i*4];
		center[0] += (float)v[0];
		center[1] += (float)v[1];
		center[2] += (float)v[2];
	}
	const float s = 1.0f / cont->nverts;
	center[0] *= s * cs;
	center[1] *= s * ch;
	center[2] *= s * cs;
	center[0] += orig[0];
	center[1] += orig[1] + 4*ch;
	center[2] += orig[2];
}

static const rcContour* findContourFromSet(const rcContourSet& cset, unsigned short reg)
{
	for (int i = 0; i < cset.nconts; ++i)
	{
		if (cset.conts[i].reg == reg)
			return &cset.conts[i];
	}
	return 0;
}

void duDebugDrawRegionConnections(duDebugDraw* dd, const rcContourSet& cset, const float alpha)
{
	if (!dd) return;
	
	const float* orig = cset.bmin;
	const float cs = cset.cs;
	const float ch = cset.ch;
	
	// Draw centers
	float pos[3], pos2[3];

	unsigned int color = duRGBA(0,0,0,196);

	dd->begin(DU_DRAW_LINES, 2.0f);

	for (int i = 0; i < cset.nconts; ++i)
	{
		const rcContour* cont = &cset.conts[i];
		getContourCenter(cont, orig, cs, ch, pos);
		for (int j = 0; j < cont->nverts; ++j)
		{
			const int* v = &cont->verts[j*4];
			if (v[3] == 0 || (unsigned short)v[3] < cont->reg) continue;
			const rcContour* cont2 = findContourFromSet(cset, (unsigned short)v[3]);
			if (cont2)
			{
				getContourCenter(cont2, orig, cs, ch, pos2);
				duAppendArc(dd, pos[0],pos[1],pos[2], pos2[0],pos2[1],pos2[2], 0.25f, 0.6f, 0.6f, color);
			}
		}
	}
	
	dd->end();

	unsigned char a = (unsigned char)(alpha * 255.0f);

	dd->begin(DU_DRAW_POINTS, 7.0f);

	for (int i = 0; i < cset.nconts; ++i)
	{
		const rcContour* cont = &cset.conts[i];
		unsigned int col = duDarkenCol(duIntToCol(cont->reg,a));
		getContourCenter(cont, orig, cs, ch, pos);
		dd->vertex(pos, col);
	}
	dd->end();
}

void duDebugDrawRawContours(duDebugDraw* dd, const rcContourSet& cset, const float alpha)
{
	if (!dd) return;

	const float* orig = cset.bmin;
	const float cs = cset.cs;
	const float ch = cset.ch;
	
	const unsigned char a = (unsigned char)(alpha*255.0f);
	
	dd->begin(DU_DRAW_LINES, 2.0f);
			
	for (int i = 0; i < cset.nconts; ++i)
	{
		const rcContour& c = cset.conts[i];
		unsigned int color = duIntToCol(c.reg, a);

		for (int j = 0; j < c.nrverts; ++j)
		{
			const int* v = &c.rverts[j*4];
			float fx = orig[0] + v[0]*cs;
			float fy = orig[1] + (v[1]+1+(i&1))*ch;
			float fz = orig[2] + v[2]*cs;
			dd->vertex(fx,fy,fz,color);
			if (j > 0)
				dd->vertex(fx,fy,fz,color);
		}
		// Loop last segment.
		const int* v = &c.rverts[0];
		float fx = orig[0] + v[0]*cs;
		float fy = orig[1] + (v[1]+1+(i&1))*ch;
		float fz = orig[2] + v[2]*cs;
		dd->vertex(fx,fy,fz,color);
	}
	dd->end();

	dd->begin(DU_DRAW_POINTS, 2.0f);	

	for (int i = 0; i < cset.nconts; ++i)
	{
		const rcContour& c = cset.conts[i];
		unsigned int color = duDarkenCol(duIntToCol(c.reg, a));
		
		for (int j = 0; j < c.nrverts; ++j)
		{
			const int* v = &c.rverts[j*4];
			float off = 0;
			unsigned int colv = color;
			if (v[3] & RC_BORDER_VERTEX)
			{
				colv = duRGBA(255,255,255,a);
				off = ch*2;
			}
			
			float fx = orig[0] + v[0]*cs;
			float fy = orig[1] + (v[1]+1+(i&1))*ch + off;
			float fz = orig[2] + v[2]*cs;
			dd->vertex(fx,fy,fz, colv);
		}
	}
	dd->end();
}

void duDebugDrawContours(duDebugDraw* dd, const rcContourSet& cset, const float alpha)
{
	if (!dd) return;

	const float* orig = cset.bmin;
	const float cs = cset.cs;
	const float ch = cset.ch;
	
	const unsigned char a = (unsigned char)(alpha*255.0f);
	
	dd->begin(DU_DRAW_LINES, 2.5f);
	
	for (int i = 0; i < cset.nconts; ++i)
	{
		const rcContour& c = cset.conts[i];
		if (!c.nverts)
			continue;
		const unsigned int color = duIntToCol(c.reg, a);
		const unsigned int bcolor = duLerpCol(color,duRGBA(255,255,255,a),128);
		for (int j = 0, k = c.nverts-1; j < c.nverts; k=j++)
		{
			const int* va = &c.verts[k*4];
			const int* vb = &c.verts[j*4];
			unsigned int col = (va[3] & RC_AREA_BORDER) ? bcolor : color; 
			float fx,fy,fz;
			fx = orig[0] + va[0]*cs;
			fy = orig[1] + (va[1]+1+(i&1))*ch;
			fz = orig[2] + va[2]*cs;
			dd->vertex(fx,fy,fz, col);
			fx = orig[0] + vb[0]*cs;
			fy = orig[1] + (vb[1]+1+(i&1))*ch;
			fz = orig[2] + vb[2]*cs;
			dd->vertex(fx,fy,fz, col);
		}
	}
	dd->end();

	dd->begin(DU_DRAW_POINTS, 3.0f);
	
	for (int i = 0; i < cset.nconts; ++i)
	{
		const rcContour& c = cset.conts[i];
		unsigned int color = duDarkenCol(duIntToCol(c.reg, a));
		for (int j = 0; j < c.nverts; ++j)
		{
			const int* v = &c.verts[j*4];
			float off = 0;
			unsigned int colv = color;
			if (v[3] & RC_BORDER_VERTEX)
			{
				colv = duRGBA(255,255,255,a);
				off = ch*2;
			}

			float fx = orig[0] + v[0]*cs;
			float fy = orig[1] + (v[1]+1+(i&1))*ch + off;
			float fz = orig[2] + v[2]*cs;
			dd->vertex(fx,fy,fz, colv);
		}
	}
	dd->end();
}

void duDebugDrawPolyMesh(duDebugDraw* dd, const struct rcPolyMesh& mesh)
{
	if (!dd) return;

	const int nvp = mesh.nvp;
	const float cs = mesh.cs;
	const float ch = mesh.ch;
	const float* orig = mesh.bmin;
	
	dd->begin(DU_DRAW_TRIS);
	
	for (int i = 0; i < mesh.npolys; ++i)
	{
		const unsigned short* p = &mesh.polys[i*nvp*2];
		const unsigned char area = mesh.areas[i];
		
		unsigned int color;
		if (area == RC_WALKABLE_AREA)
			color = duRGBA(0,192,255,64);
		else if (area == RC_NULL_AREA)
			color = duRGBA(0,0,0,64);
		else
			color = dd->areaToCol(area);
		
		unsigned short vi[3];
		for (int j = 2; j < nvp; ++j)
		{
			if (p[j] == RC_MESH_NULL_IDX) break;
			vi[0] = p[0];
			vi[1] = p[j-1];
			vi[2] = p[j];
			for (int k = 0; k < 3; ++k)
			{
				const unsigned short* v = &mesh.verts[vi[k]*3];
				const float x = orig[0] + v[0]*cs;
				const float y = orig[1] + (v[1]+1)*ch;
				const float z = orig[2] + v[2]*cs;
				dd->vertex(x,y,z, color);
			}
		}
	}
	dd->end();

	// Draw neighbours edges
	const unsigned int coln = duRGBA(0,48,64,32);
	dd->begin(DU_DRAW_LINES, 1.5f);
	for (int i = 0; i < mesh.npolys; ++i)
	{
		const unsigned short* p = &mesh.polys[i*nvp*2];
		for (int j = 0; j < nvp; ++j)
		{
			if (p[j] == RC_MESH_NULL_IDX) break;
			if (p[nvp+j] & 0x8000) continue;
			const int nj = (j+1 >= nvp || p[j+1] == RC_MESH_NULL_IDX) ? 0 : j+1; 
			const int vi[2] = {p[j], p[nj]};
			
			for (int k = 0; k < 2; ++k)
			{
				const unsigned short* v = &mesh.verts[vi[k]*3];
				const float x = orig[0] + v[0]*cs;
				const float y = orig[1] + (v[1]+1)*ch + 0.1f;
				const float z = orig[2] + v[2]*cs;
				dd->vertex(x, y, z, coln);
			}
		}
	}
	dd->end();
	
	// Draw boundary edges
	const unsigned int colb = duRGBA(0,48,64,220);
	dd->begin(DU_DRAW_LINES, 2.5f);
	for (int i = 0; i < mesh.npolys; ++i)
	{
		const unsigned short* p = &mesh.polys[i*nvp*2];
		for (int j = 0; j < nvp; ++j)
		{
			if (p[j] == RC_MESH_NULL_IDX) break;
			if ((p[nvp+j] & 0x8000) == 0) continue;
			const int nj = (j+1 >= nvp || p[j+1] == RC_MESH_NULL_IDX) ? 0 : j+1; 
			const int vi[2] = {p[j], p[nj]};
			
			unsigned int col = colb;
			if ((p[nvp+j] & 0xf) != 0xf)
				col = duRGBA(255,255,255,128);
			for (int k = 0; k < 2; ++k)
			{
				const unsigned short* v = &mesh.verts[vi[k]*3];
				const float x = orig[0] + v[0]*cs;
				const float y = orig[1] + (v[1]+1)*ch + 0.1f;
				const float z = orig[2] + v[2]*cs;
				dd->vertex(x, y, z, col);
			}
		}
	}
	dd->end();
	
	dd->begin(DU_DRAW_POINTS, 3.0f);
	const unsigned int colv = duRGBA(0,0,0,220);
	for (int i = 0; i < mesh.nverts; ++i)
	{
		const unsigned short* v = &mesh.verts[i*3];
		const float x = orig[0] + v[0]*cs;
		const float y = orig[1] + (v[1]+1)*ch + 0.1f;
		const float z = orig[2] + v[2]*cs;
		dd->vertex(x,y,z, colv);
	}
	dd->end();
}

void duDebugDrawPolyMeshDetail(duDebugDraw* dd, const struct rcPolyMeshDetail& dmesh)
{
	if (!dd) return;

	dd->begin(DU_DRAW_TRIS);
	
	for (int i = 0; i < dmesh.nmeshes; ++i)
	{
		const unsigned int* m = &dmesh.meshes[i*4];
		const unsigned int bverts = m[0];
		const unsigned int btris = m[2];
		const int ntris = (int)m[3];
		const float* verts = &dmesh.verts[bverts*3];
		const unsigned char* tris = &dmesh.tris[btris*4];

		unsigned int color = duIntToCol(i, 192);

		for (int j = 0; j < ntris; ++j)
		{
			dd->vertex(&verts[tris[j*4+0]*3], color);
			dd->vertex(&verts[tris[j*4+1]*3], color);
			dd->vertex(&verts[tris[j*4+2]*3], color);
		}
	}
	dd->end();

	// Internal edges.
	dd->begin(DU_DRAW_LINES, 1.0f);
	const unsigned int coli = duRGBA(0,0,0,64);
	for (int i = 0; i < dmesh.nmeshes; ++i)
	{
		const unsigned int* m = &dmesh.meshes[i*4];
		const unsigned int bverts = m[0];
		const unsigned int btris = m[2];
		const int ntris = (int)m[3];
		const float* verts = &dmesh.verts[bverts*3];
		const unsigned char* tris = &dmesh.tris[btris*4];
		
		for (int j = 0; j < ntris; ++j)
		{
			const unsigned char* t = &tris[j*4];
			for (int k = 0, kp = 2; k < 3; kp=k++)
			{
				unsigned char ef = (t[3] >> (kp*2)) & 0x3;
				if (ef == 0)
				{
					// Internal edge
					if (t[kp] < t[k])
					{
						dd->vertex(&verts[t[kp]*3], coli);
						dd->vertex(&verts[t[k]*3], coli);
					}
				}
			}
		}
	}
	dd->end();
	
	// External edges.
	dd->begin(DU_DRAW_LINES, 2.0f);
	const unsigned int cole = duRGBA(0,0,0,64);
	for (int i = 0; i < dmesh.nmeshes; ++i)
	{
		const unsigned int* m = &dmesh.meshes[i*4];
		const unsigned int bverts = m[0];
		const unsigned int btris = m[2];
		const int ntris = (int)m[3];
		const float* verts = &dmesh.verts[bverts*3];
		const unsigned char* tris = &dmesh.tris[btris*4];
		
		for (int j = 0; j < ntris; ++j)
		{
			const unsigned char* t = &tris[j*4];
			for (int k = 0, kp = 2; k < 3; kp=k++)
			{
				unsigned char ef = (t[3] >> (kp*2)) & 0x3;
				if (ef != 0)
				{
					// Ext edge
					dd->vertex(&verts[t[kp]*3], cole);
					dd->vertex(&verts[t[k]*3], cole);
				}
			}
		}
	}
	dd->end();
	
	dd->begin(DU_DRAW_POINTS, 3.0f);
	const unsigned int colv = duRGBA(0,0,0,64);
	for (int i = 0; i < dmesh.nmeshes; ++i)
	{
		const unsigned int* m = &dmesh.meshes[i*4];
		const unsigned int bverts = m[0];
		const int nverts = (int)m[1];
		const float* verts = &dmesh.verts[bverts*3];
		for (int j = 0; j < nverts; ++j)
			dd->vertex(&verts[j*3], colv);
	}
	dd->end();
}
