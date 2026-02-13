/**
 * sys/resource.h compatibility shim for MSVC.
 *
 * LAPKT sources conditionally include <sys/resource.h>.
 * MSVC doesn't provide this header. This empty shim prevents errors.
 */
#ifndef _MUJIN_SYS_RESOURCE_H_COMPAT
#define _MUJIN_SYS_RESOURCE_H_COMPAT

#ifdef _MSC_VER
// Nothing needed — LAPKT provides its own rusage impl for WIN32
#else
#include_next <sys/resource.h>
#endif

#endif // _MUJIN_SYS_RESOURCE_H_COMPAT
