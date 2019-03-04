#include "g2o_lib_api.h"

typedef void(*StringCallbackFuncType)(const char *message);

extern "C" {
/**
 * Return codes of g2o optimization
 */
G2O_DLL_API enum G2OResult {
    Ok = 0,
    ErrorLoadingGraph = -1,
    ErrorOpeningFile = -2,
    ErrorAllocatingSolver = -3,
    ErrorGraphContainsNoVertices = -4,
    ErrorSolverCannotOptimizeGraph = -5,
    ErrorVertexNotFound = -6,
    ErrorGraphIsFixedByNode = -7,
    ErrorAddingVertex = -8,
    ErrorInitializationFailed = -9,
    ErrorCholeskyFailed = -10
};

/**
 * Optimize a factor graph with g2o
 *
 * @param argc Number of arg values
 * @param argv Configuration (commandline) parameters for the g2o run
 * @param inputFactorGraph The factor graph input as ascii string
 * @param outputFactorGraphReference The optimized output factor graph as ascii string
 * @return
 */
G2O_DLL_API int optimize(int argc, char **argv, const char *inputFactorGraph, char **outputFactorGraphReference);

/**
 * Free factor graph output buffer
 *
 * @param outputFactorGraphReference The pointer to the output factor graph
 */
G2O_DLL_API void cleanup(char *outputFactorGraphReference);
};