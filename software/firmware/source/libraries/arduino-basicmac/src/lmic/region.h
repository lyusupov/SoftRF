// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

#ifndef _region_h_
#define _region_h_

#include "board.h"

// public region codes - DO NOT CHANGE!
enum {
    REGCODE_UNDEF = 0,
    REGCODE_EU868 = 1,
    REGCODE_AS923 = 2,
    REGCODE_US915 = 3,
    REGCODE_AU915 = 4,
    REGCODE_CN470 = 5,
};

// ------------------------------------------------
// EU868
#ifdef CFG_eu868

#define REG_DYN
#define REG_DRTABLE_EU

#endif


// ------------------------------------------------
// AS923
#ifdef CFG_as923

#define REG_DYN
#define REG_DRTABLE_EU

#endif


// ------------------------------------------------
// IL915
#ifdef CFG_il915

#define REG_DYN
#define REG_DRTABLE_EU

#endif


// ------------------------------------------------
// KR920
#ifdef CFG_kr920

#define REG_DYN
#define REG_DRTABLE_125kHz

#endif


// ------------------------------------------------
// US915
#ifdef CFG_us915

#define REG_FIX
#define REG_DRTABLE_US

#if MAX_FIX_CHNLS_125 < 64
#undef MAX_FIX_CHNLS_125
#define MAX_FIX_CHNLS_125 64
#endif

#if MAX_FIX_CHNLS_500 < 8
#undef MAX_FIX_CHNLS_500
#define MAX_FIX_CHNLS_500 8
#endif

#endif

// ------------------------------------------------
// AU915
#ifdef CFG_au915

#define REG_FIX
#define REG_DRTABLE_AU

#if MAX_FIX_CHNLS_125 < 64
#undef MAX_FIX_CHNLS_125
#define MAX_FIX_CHNLS_125 64
#endif

#if MAX_FIX_CHNLS_500 < 8
#undef MAX_FIX_CHNLS_500
#define MAX_FIX_CHNLS_500 8
#endif

#endif


// ------------------------------------------------
// CN470
#ifdef CFG_cn470

#define REG_FIX
#define REG_DRTABLE_125kHz

#if MAX_FIX_CHNLS_125 < 96
#undef MAX_FIX_CHNLS_125
#define MAX_FIX_CHNLS_125 96
#endif

#endif


// ------------------------------------------------
// Sanity checks

#if !defined(REG_DYN) && !defined(REG_FIX)
#error "No regions defined"
#endif


// ------------------------------------------------
// Derived values

#if defined(REG_FIX)

#ifndef MAX_FIX_CHNLS_500
#define MAX_FIX_CHNLS_500 0
#endif

#define MAX_FIX_CHNLS	(MAX_FIX_CHNLS_125 + MAX_FIX_CHNLS_500)

#endif

#endif
