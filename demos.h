#pragma once
#include "NEPath-master/setup_NEPath.h"
void demo_Raster();
void demo_Zigzag();
void demo_CP();
void demo_CP_CFS();
void demo_CP_DFS();
void demo_tool_compensate();
void demo_underfill();
void demo_sharpcorner();
#if defined(IncludeIpopt) && (IncludeIpopt != 0)
void demo_IQOP();
void demo_IQOP_CFS();
void demo_IQOP_DFS();
#endif