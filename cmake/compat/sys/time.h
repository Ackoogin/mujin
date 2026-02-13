/**
 * sys/time.h compatibility shim for MSVC.
 *
 * LAPKT sources unconditionally include <sys/time.h> even on Windows.
 * MSVC doesn't provide this header. This shim provides the minimal
 * struct timeval definition that LAPKT's resources_control needs.
 */
#ifndef _MUJIN_SYS_TIME_H_COMPAT
#define _MUJIN_SYS_TIME_H_COMPAT

#ifdef _MSC_VER

#include <winsock2.h>  // provides struct timeval on Windows

#else
// On non-MSVC platforms, defer to the real header
#include_next <sys/time.h>
#endif

#endif // _MUJIN_SYS_TIME_H_COMPAT
