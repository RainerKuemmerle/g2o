#include "stream_interceptor.h"
#include "g2o_lib_api.h"

typedef void(*StringCallbackFuncType)(const char *message);

extern "C" {
/**
 * Set the callback function for the stdout interceptor
 *
 * @param callbackFunc Callback function
 */
G2O_DLL_API void setCoutListener(StringCallbackFuncType callbackFunc);

/**
 * Set the callback function for the stderr interceptor
 *
 * @param callbackFunc Callback function
 */
G2O_DLL_API void setCerrListener(StringCallbackFuncType callbackFunc);

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