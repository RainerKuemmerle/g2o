#ifndef G2O_TYPES_SEVEN_DOF_EXPMAP_API_H
#define G2O_TYPES_SEVEN_DOF_EXPMAP_API_H

#include "g2o/config.h"

#ifdef _MSC_VER
// We are using a Microsoft compiler:
#ifdef G2O_SHARED_LIBS
#ifdef types_sim3_EXPORTS
#define G2O_TYPES_SIM3_API __declspec(dllexport)
#else
#define G2O_TYPES_SIM3_API __declspec(dllimport)
#endif
#else
#define G2O_TYPES_SIM3_API
#endif

#else
// Not Microsoft compiler so set empty definition:
#define G2O_TYPES_SIM3_API
#endif

#endif
