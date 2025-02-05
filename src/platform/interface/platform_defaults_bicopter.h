/**
 *  _   _                     _           _     
 * | | | |                   | |         | |    
 * | |_| |_   _ _   _ _ __   | |     __ _| |__  
 * |  _  | | | | | | | '_ \  | |    / _` | '_ \ 
 * | | | | |_| | |_| | | | | | |___| (_| | |_) |
 * \_| |_/\__, |\__,_|_| |_| \_____/\__,_|_.__/ 
 *         __/ |                                
 *        |___/                                 
 * 
 * @file platform_defaults_bicopter.h
 * @author Logan Dihel
 * @brief Platform-specific default values for bicopter. This overrides the bolt defaults.
 * @details This file was modified from platform_defaults_bolt.h
 */

#pragma once

#ifndef __INCLUDED_FROM_PLATFORM_DEFAULTS__
    #pragma GCC error "Do not include this file directly, include platform_defaults.h instead."
#endif

// Defines for default values for the bicopter, overwriting the bolt defaults
#ifdef DEFAULT_BAT_LOW_VOLTAGE
    #undef DEFAULT_BAT_LOW_VOLTAGE
#endif
#define DEFAULT_BAT_LOW_VOLTAGE                   12.8f

#ifdef DEFAULT_BAT_CRITICAL_LOW_VOLTAGE
    #undef DEFAULT_BAT_CRITICAL_LOW_VOLTAGE
#endif
#define DEFAULT_BAT_CRITICAL_LOW_VOLTAGE          12.0f
