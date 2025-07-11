//------------------------------------------------------------------------------
// Copyright (c) 2017 Cohda Wireless Pty Ltd
//-----------------------------------------------------------------------------

#ifndef __DEBUG_LEVELS_H__
#define __DEBUG_LEVELS_H__

#define D_MASTER 1

#ifndef D_LOCAL
//#define D_LOCAL D_ALL
#define D_LOCAL D_WARN
#endif

#include <linux/cohda/debug.h>

// Some other handy levels
#define D_API                   (D_INFO)
#define D_TST                   (D_DEBUG)
#define D_DBG                   (D_VERBOSE)

#endif //__DEBUG_LEVELS_H__
