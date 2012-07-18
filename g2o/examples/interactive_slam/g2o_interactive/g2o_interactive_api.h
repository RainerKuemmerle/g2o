#ifndef G2O_INTERACTIVE_API_H
#define G2O_INTERACTIVE_API_H

#include "g2o/config.h"

#ifdef _MSC_VER
// We are using a Microsoft compiler:
#ifdef G2O_SHARED_LIBS
#ifdef g2o_interactive_library_EXPORTS
#define G2O_INTERACTIVE_API __declspec(dllexport)
#else
#define G2O_INTERACTIVE_API __declspec(dllimport)
#endif
#else
#define G2O_INTERACTIVE_API
#endif

#else
// Not Microsoft compiler so set empty definition:
#define G2O_INTERACTIVE_API
#endif

#endif // G2O_CORE_API_H
