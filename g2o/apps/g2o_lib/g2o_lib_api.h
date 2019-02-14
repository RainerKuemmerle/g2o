#ifndef G2O_DLL_API_H
#define G2O_DLL_API_H

#include "g2o/config.h"

#ifdef _MSC_VER
// For Microsoft compiler:
#define G2O_DLL_API __declspec(dllexport)
#else
// No Microsoft compiler so set empty:
#define G2O_DLL_API
#endif

#endif // G2O_DLL_API_H