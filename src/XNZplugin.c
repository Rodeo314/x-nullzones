/*
 * XNZplugin.c
 *
 * This file is part of the x-nullzones source code.
 *
 * (C) Copyright 2020 Timothy D. Walker and others.
 *
 * All rights reserved. This program and the accompanying materials are made
 * available under the terms of the GNU General Public License (GPL) version 2
 * which accompanies this distribution (LICENSE file), and is also available at
 * http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * Contributors:
 *     Timothy D. Walker
 */

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "XPLM/XPLMDataAccess.h"
#include "XPLM/XPLMPlanes.h"
#include "XPLM/XPLMPlugin.h"
#include "XPLM/XPLMProcessing.h"
#include "XPLM/XPLMUtilities.h"

#define MPS2KTS(MPS) (MPS * 3.6f / 1.852f)
#define YAW_NZ_FACTR (2.0f)

typedef struct
{
    int i_version_simulator;
    int i_version_xplm_apis;
    XPLMFlightLoop_f f_l_cb;
    XPLMDataRef f_air_speed;
    XPLMDataRef f_grd_speed;
    XPLMDataRef f_servos[9];
    XPLMDataRef i_servos[9];
    XPLMDataRef nullzone[3];
    float prefs_nullzone[3];
    float minimum_null_zone;
    int autopilot_servos_on;
}
xnz_context;

static const char *i_servo_dataref_names[] =
{
    "sim/cockpit2/autopilot/servos_on",
    NULL,
};

static const char *f_servo_dataref_names[] =
{
    NULL,
};

xnz_context *global_context = NULL;

static   int xnz_log       (const char *format, ...);
static float callback_hdlr(float, float, int, void*);

#if IBM
#include <windows.h>
BOOL APIENTRY DllMain(HANDLE hModule,
                      DWORD  ul_reason_for_call,
                      LPVOID lpReserved)
{
    switch (ul_reason_for_call)
    {
        case DLL_PROCESS_ATTACH:
        case DLL_THREAD_ATTACH:
        case DLL_THREAD_DETACH:
        case DLL_PROCESS_DETACH:
            break;
    }
    return TRUE;
}
#endif

PLUGIN_API int XPluginStart(char *outName, char *outSig, char *outDesc)
{
    strncpy(outName,                                     "X-Nullzones", 255);
    strncpy(outSig,                                     "Rodeo314.XNZ", 255);
    strncpy(outDesc, "Dynamic nullzones and other miscellaneous stuff", 255);

    /* all good */
    XPLMDebugString("x-nullzones [info]: XPluginStart OK\n"); return 1;
}

PLUGIN_API void XPluginStop(void)
{
    return;
}

PLUGIN_API int XPluginEnable(void)
{
    /* check for unsupported versions */
    XPLMHostApplicationID outHostID;
    int outXPlaneVersion, outXPLMVersion;
    XPLMGetVersions(&outXPlaneVersion, &outXPLMVersion, &outHostID);
    if (outXPLMVersion < 210)
    {
        xnz_log("x-nullzones [error]: XPluginEnable failed (outXPLMVersion: %d < 210)\n", outXPLMVersion);
        return 0;
    }

    /* Initialize context */
    if (NULL == (global_context = malloc(sizeof(xnz_context))))
    {
        XPLMDebugString("x-nullzones [error]: XPluginEnable failed (malloc)\n"); goto fail;
    }
    if (NULL == (global_context->nullzone[0] = XPLMFindDataRef("sim/joystick/joystick_pitch_nullzone")))
    {
        XPLMDebugString("x-nullzones [error]: XPluginEnable failed (nullzone[0])\n"); goto fail;
    }
    if (NULL == (global_context->nullzone[1] = XPLMFindDataRef("sim/joystick/joystick_roll_nullzone")))
    {
        XPLMDebugString("x-nullzones [error]: XPluginEnable failed (nullzone[1])\n"); goto fail;
    }
    if (NULL == (global_context->nullzone[2] = XPLMFindDataRef("sim/joystick/joystick_heading_nullzone")))
    {
        XPLMDebugString("x-nullzones [error]: XPluginEnable failed (nullzone[2])\n"); goto fail;
    }
    if (NULL == (global_context->f_grd_speed = XPLMFindDataRef("sim/flightmodel/position/groundspeed")))
    {
        XPLMDebugString("x-nullzones [error]: XPluginEnable failed (f_grd_speed)\n"); goto fail;
    }
    if (NULL == (global_context->f_air_speed = XPLMFindDataRef("sim/flightmodel/position/indicated_airspeed")))
    {
        XPLMDebugString("x-nullzones [error]: XPluginEnable failed (f_air_speed)\n"); goto fail;
    }

    /* flight loop callback */
    XPLMRegisterFlightLoopCallback((global_context->f_l_cb = &callback_hdlr), 0, global_context);

    /* initialize arrays */
    memset(global_context->f_servos, (int)NULL, sizeof(global_context->f_servos));
    memset(global_context->i_servos, (int)NULL, sizeof(global_context->i_servos));

    /* all good */
    XPLMDebugString("x-nullzones [info]: XPluginEnable OK\n");
    global_context->i_version_simulator = outXPlaneVersion;
    global_context->i_version_xplm_apis = outXPLMVersion;
    return 1;

fail:
    if (NULL != global_context)
    {
        free(global_context);
        global_context = NULL;
    }
    return 0;
}

PLUGIN_API void XPluginDisable(void)
{
    /* flight loop callback */
    XPLMUnregisterFlightLoopCallback(global_context->f_l_cb, global_context);

    /* reset nullzones to default/preferences values */
    XPLMSetDataf(global_context->nullzone[0], global_context->prefs_nullzone[0]);
    XPLMSetDataf(global_context->nullzone[1], global_context->prefs_nullzone[1]);
    XPLMSetDataf(global_context->nullzone[2], global_context->prefs_nullzone[2]);

    /* close context */
    if (NULL != global_context)
    {
        free(global_context);
        global_context = NULL;
    }

    /* all good */
    XPLMDebugString("x-nullzones [info]: XPluginDisable OK\n");
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFromWho, long inMessage, void *inParam)
{
    switch (inMessage)
    {
        case XPLM_MSG_PLANE_LOADED:
            if (inParam == XPLM_USER_AIRCRAFT) // user's plane changing
            {
                global_context->minimum_null_zone = 0.025f;
                global_context->prefs_nullzone[0] = XPLMGetDataf(global_context->nullzone[0]);
                global_context->prefs_nullzone[1] = XPLMGetDataf(global_context->nullzone[1]);
                global_context->prefs_nullzone[2] = XPLMGetDataf(global_context->nullzone[2]);
                global_context->minimum_null_zone = fmaxf(global_context->minimum_null_zone, global_context->prefs_nullzone[0]);
                global_context->minimum_null_zone = fmaxf(global_context->minimum_null_zone, global_context->prefs_nullzone[1]);
                global_context->minimum_null_zone = fmaxf(global_context->minimum_null_zone, global_context->prefs_nullzone[2] / YAW_NZ_FACTR);
                xnz_log("x-nullzones: new aircraft: initial nullzones %.3lf %.3lf %.3lf (minimum %.3lf)\n",
                        global_context->prefs_nullzone[0],
                        global_context->prefs_nullzone[1],
                        global_context->prefs_nullzone[2],
                        global_context->minimum_null_zone);
                return;
            }
            break;

        case XPLM_MSG_PLANE_UNLOADED:
            if (inParam == XPLM_USER_AIRCRAFT) // user's plane changing
            {
                XPLMSetFlightLoopCallbackInterval(global_context->f_l_cb, 0, 1, global_context);
                memset(global_context->f_servos, (int)NULL, sizeof(global_context->f_servos));
                memset(global_context->i_servos, (int)NULL, sizeof(global_context->i_servos));
                XPLMSetDataf(global_context->nullzone[0], global_context->prefs_nullzone[0]);
                XPLMSetDataf(global_context->nullzone[1], global_context->prefs_nullzone[1]);
                XPLMSetDataf(global_context->nullzone[2], global_context->prefs_nullzone[2]);
                return;
            }
            break;

        case XPLM_MSG_WILL_WRITE_PREFS:
            XPLMSetDataf(global_context->nullzone[0], global_context->prefs_nullzone[0]);
            XPLMSetDataf(global_context->nullzone[1], global_context->prefs_nullzone[1]);
            XPLMSetDataf(global_context->nullzone[2], global_context->prefs_nullzone[2]);
            return;

        case XPLM_MSG_LIVERY_LOADED:
            if (inParam == XPLM_USER_AIRCRAFT) // wait until aircraft plugins loaded (for custom datarefs)
            {
                // check for all supported autopilot/servo datarefs
                for (size_t i = 0, j = 0, k = (sizeof(global_context->i_servos) / sizeof(global_context->i_servos[0])); i_servo_dataref_names[i] != NULL && j < k; i++)
                {
                    if (NULL != (global_context->i_servos[j] = XPLMFindDataRef(i_servo_dataref_names[i])))
                    {
                        j++;
                    }
                }
                for (size_t i = 0, j = 0, k = (sizeof(global_context->f_servos) / sizeof(global_context->f_servos[0])); f_servo_dataref_names[i] != NULL && j < k; i++)
                {
                    if (NULL != (global_context->f_servos[j] = XPLMFindDataRef(f_servo_dataref_names[i])))
                    {
                        j++;
                    }
                }
                XPLMSetFlightLoopCallbackInterval(global_context->f_l_cb, 1, 1, global_context);
                return;
            }
            break;

        default:
            break;
    }
}

#define AIRSPEED_MIN_KTS (50.000f)
#define AIRSPEED_MAX_KTS (62.500f)
#define GROUNDSP_MIN_KTS (03.125f)
#define GROUNDSP_MAX_KTS (31.250f)
static float callback_hdlr(float inElapsedSinceLastCall,
                           float inElapsedTimeSinceLastFlightLoop,
                           int   inCounter,
                           void *inRefcon)
{
    if (inRefcon)
    {
        xnz_context *ctx = inRefcon;

        /* check servos on/off */
        ctx->autopilot_servos_on = 0;
        size_t i1 = 0, j1 = sizeof(ctx->i_servos) / sizeof(ctx->i_servos[0]);
        size_t i2 = 0, j2 = sizeof(ctx->f_servos) / sizeof(ctx->f_servos[0]);
        while (ctx->autopilot_servos_on == 0 &&
               i1 < j1 && NULL != ctx->i_servos[i1])
        {
            if (XPLMGetDatai(ctx->i_servos[i1]) > 0)
            {
                ctx->autopilot_servos_on = 1;
                break;
            }
            i1++; continue;
        }
        while (ctx->autopilot_servos_on == 0 &&
               i2 < j2 && NULL != ctx->f_servos[i2])
        {
            if (XPLMGetDataf(ctx->f_servos[i2]) > 0.5f)
            {
                ctx->autopilot_servos_on = 1;
                break;
            }
            i2++; continue;
        }
        if (ctx->autopilot_servos_on)
        {
            XPLMSetDataf(ctx->nullzone[0], 0.500f);
            XPLMSetDataf(ctx->nullzone[1], 0.500f);
            XPLMSetDataf(ctx->nullzone[2], 0.500f);
        }
        else
        {
            float airspeed = XPLMGetDataf(ctx->f_air_speed);
            float groundsp = MPS2KTS(XPLMGetDataf(ctx->f_grd_speed));
            if (airspeed > AIRSPEED_MAX_KTS)
            {
                airspeed = AIRSPEED_MAX_KTS;
            }
            if (airspeed < AIRSPEED_MIN_KTS)
            {
                airspeed = AIRSPEED_MIN_KTS;
            }
            if (groundsp > GROUNDSP_MAX_KTS)
            {
                groundsp = GROUNDSP_MAX_KTS;
            }
            if (groundsp < GROUNDSP_MIN_KTS)
            {
                groundsp = GROUNDSP_MIN_KTS;
            }
            float nullzone_pitch_roll =         1.0f * 0.125f - ((0.125f - ctx->minimum_null_zone) * ((airspeed - AIRSPEED_MIN_KTS) / (AIRSPEED_MAX_KTS - AIRSPEED_MIN_KTS)));
            float nullzone_yaw_tiller = YAW_NZ_FACTR * 0.125f - ((0.125f - ctx->minimum_null_zone) * ((groundsp - GROUNDSP_MIN_KTS) / (GROUNDSP_MAX_KTS - GROUNDSP_MIN_KTS)));
            XPLMSetDataf(ctx->nullzone[0], nullzone_pitch_roll);
            XPLMSetDataf(ctx->nullzone[1], nullzone_pitch_roll);
            XPLMSetDataf(ctx->nullzone[2], nullzone_yaw_tiller);
        }
        return (1.0f / 20.0f); // run often
    }
    XPLMDebugString("x-nullzones [error]: callback_hdlr: inRefcon == NULL, disabling callback");
    return 0;
}
#undef AIRSPEED_MIN_KTS
#undef AIRSPEED_MAX_KTS
#undef GROUNDSP_MIN_KTS
#undef GROUNDSP_MAX_KTS

static int xnz_log(const char *format, ...)
{
    int ret;
    va_list ap;
    char string[1024];
    va_start(ap, format);
    ret = vsnprintf(string, sizeof(string), format, ap);
    if (ret > 0) // output is NULL-terminated
    {
        XPLMDebugString(string);
    }
    va_end(ap);
    return ret;
}

#undef MPS2KTS
#undef YAW_NZ_FACTR
