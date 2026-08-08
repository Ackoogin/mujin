/**
 * unistd.h compatibility shim for MSVC.
 *
 * LAPKT sources unconditionally include <unistd.h> even on Windows.
 * MSVC doesn't provide this header. This shim provides the minimal
 * definitions needed.
 */
#ifndef _AME_UNISTD_H_COMPAT
#define _AME_UNISTD_H_COMPAT

#ifdef _MSC_VER

#include <io.h>
#include <process.h>

#else
#include_next <unistd.h>
#endif

#endif // _AME_UNISTD_H_COMPAT
