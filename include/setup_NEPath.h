// Conditionally enable direction parallel processing for toolpath generation
#ifndef ENABLE_DIRECTION_PARALLEL
#define ENABLE_DIRECTION_PARALLEL 1
#endif

// Conditionally enable IPOPT support (open-source optimization solver)
// IPOPT replaces Gurobi for non-equidistant toolpath optimization (IQOP)
// #ifndef IncludeIpopt
// #define IncludeIpopt 1
// #endif

// Legacy Gurobi support (deprecated, use IPOPT instead)
// #ifndef IncludeGurobi
// #define IncludeGurobi 0
// #endif
