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

#include "Widgets/XPStandardWidgets.h"
#include "Widgets/XPWidgets.h"
#include "XPLM/XPLMDataAccess.h"
#include "XPLM/XPLMPlanes.h"
#include "XPLM/XPLMPlugin.h"
#include "XPLM/XPLMProcessing.h"
#include "XPLM/XPLMUtilities.h"

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wignored-attributes"
#endif
#include <stdbool.h>
#include "sharedvalue.h"
#ifdef __clang__
#pragma clang diagnostic pop
#endif

#define MPS2KTS(MPS) (MPS * 3.6f / 1.852f)
#define YAW_NZ_FACTR (2.0f)

typedef struct
{
    int i_context_init_done;
    int i_version_simulator;
    int i_version_xplm_apis;
    XPLMFlightLoop_f f_l_cb;
    XPLMDataRef f_air_speed;
    XPLMDataRef f_grd_speed;
    XPLMDataRef f_servos[9];
    XPLMDataRef i_servos[9];
    XPLMDataRef i_autoth[9];
    XPLMDataRef f_autoth[9];
    XPLMDataRef nullzone[3];
    float prefs_nullzone[3];
    float minimum_null_zone;
    XPLMDataRef acf_roll_co;
    XPLMDataRef ongroundany;
    float nominal_roll_coef;
    int use_320ultimate_api;
    SharedValuesInterface s;
    int id_f32_eng_lever_lt;
    int id_f32_eng_lever_rt;
    XPLMDataRef f_thr_array;
    XPLMDataRef f_throttall;
    float last_throttle_all;
    float show_throttle_all;
    float icecheck_required;
    char overlay_textbuf[9];
    int throttle_did_change;
    int ice_detect_positive;
    XPLMDataRef f_ice_rf[4];
    XPWidgetID  widgetid[2];
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

static const char *i_autot_dataref_names[] =
{
    "sim/cockpit2/autopilot/autothrottle_enabled",
    NULL,
};

static const char *f_autot_dataref_names[] =
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
    if (outXPLMVersion < 210) // currently pointless as we target XPLM210 at build time
    {
        xnz_log("x-nullzones [error]: XPluginEnable failed (outXPLMVersion: %d < 210)\n", outXPLMVersion);
        return 0;
    }
    if (outXPlaneVersion > 11999)
    {
        xnz_log("x-nullzones [error]: XPluginEnable failed (outXPlaneVersion: %d > 11999)\n", outXPlaneVersion);
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
    if (NULL == (global_context->acf_roll_co = XPLMFindDataRef("sim/aircraft/overflow/acf_roll_co")))
    {
        XPLMDebugString("x-nullzones [error]: XPluginEnable failed (acf_roll_co)\n"); goto fail;
    }
    if (NULL == (global_context->ongroundany = XPLMFindDataRef("sim/flightmodel/failures/onground_any")))
    {
        XPLMDebugString("x-nullzones [error]: XPluginEnable failed (ongroundany)\n"); goto fail;
    }
    if (NULL == (global_context->f_throttall = XPLMFindDataRef("sim/cockpit2/engine/actuators/throttle_ratio_all")))
    {
        XPLMDebugString("x-nullzones [error]: XPluginEnable failed (f_throt_all)\n"); goto fail;
    }
    if (NULL == (global_context->f_ice_rf[0] = XPLMFindDataRef("sim/flightmodel/failures/pitot_ice")) ||
        NULL == (global_context->f_ice_rf[1] = XPLMFindDataRef("sim/flightmodel/failures/inlet_ice")) ||
        NULL == (global_context->f_ice_rf[2] = XPLMFindDataRef("sim/flightmodel/failures/prop_ice")) ||
        NULL == (global_context->f_ice_rf[3] = XPLMFindDataRef("sim/flightmodel/failures/frm_ice")))
    {
        XPLMDebugString("x-nullzones [error]: XPluginEnable failed (f_ice_rf)\n"); goto fail;
    }
    if (!(global_context->widgetid[0] = XPCreateWidget(0, 0, 0, 0, 0, "", 1, NULL,
                                                       xpWidgetClass_MainWindow)))
    {
        XPLMDebugString("x-nullzones [error]: XPluginEnable failed (widgetid)\n"); goto fail;
    }
    if (!(global_context->widgetid[1] = XPCreateWidget(0, 0, 0, 0, 0, "", 0,
                                                       global_context->widgetid[0],
                                                       xpWidgetClass_Caption)))
    {
        XPLMDebugString("x-nullzones [error]: XPluginEnable failed (widgetid)\n"); goto fail;
    }
    XPSetWidgetProperty(global_context->widgetid[0], xpProperty_MainWindowType, xpMainWindowStyle_Translucent);
    XPSetWidgetProperty(global_context->widgetid[1], xpProperty_CaptionLit, 1);
    XPSetWidgetGeometry(global_context->widgetid[0], 00, 56, 64, 00);
    XPSetWidgetGeometry(global_context->widgetid[1], 12, 46, 56, 10);

    /* flight loop callback */
    XPLMRegisterFlightLoopCallback((global_context->f_l_cb = &callback_hdlr), 0, global_context);

    /* initialize arrays */
    memset(global_context->f_servos, (int)NULL, sizeof(global_context->f_servos));
    memset(global_context->i_servos, (int)NULL, sizeof(global_context->i_servos));
    memset(global_context->i_autoth, (int)NULL, sizeof(global_context->i_autoth));
    memset(global_context->f_autoth, (int)NULL, sizeof(global_context->f_autoth));

    /* all good */
    XPLMDebugString("x-nullzones [info]: XPluginEnable OK\n");
    global_context->i_version_simulator = outXPlaneVersion;
    global_context->i_version_xplm_apis = outXPLMVersion;
    global_context->i_context_init_done = 0;
    return 1;

fail:
    if (global_context->widgetid[0] != 0)
    {
        XPDestroyWidget(global_context->widgetid[0], 1);
    }
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
    XPLMSetDataf(global_context->acf_roll_co, global_context->nominal_roll_coef);

    /* clean up our widgets */
    if (XPIsWidgetVisible(global_context->widgetid[1]) != 0)
    {
        XPHideWidget(global_context->widgetid[0]);
        XPHideWidget(global_context->widgetid[1]);
    }
    if (global_context->widgetid[0] != 0)
    {
        XPDestroyWidget(global_context->widgetid[0], 1);
    }

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
        case XPLM_MSG_WILL_WRITE_PREFS:
            XPLMSetDataf(global_context->nullzone[0], global_context->prefs_nullzone[0]);
            XPLMSetDataf(global_context->nullzone[1], global_context->prefs_nullzone[1]);
            XPLMSetDataf(global_context->nullzone[2], global_context->prefs_nullzone[2]);
            XPLMSetDataf(global_context->acf_roll_co, global_context->nominal_roll_coef);
            return;

        case XPLM_MSG_PLANE_UNLOADED:
            if (inParam == XPLM_USER_AIRCRAFT) // user's plane changing
            {
                XPLMSetFlightLoopCallbackInterval(global_context->f_l_cb, 0, 1, global_context);
                memset(global_context->f_servos, (int)NULL, sizeof(global_context->f_servos));
                memset(global_context->i_servos, (int)NULL, sizeof(global_context->i_servos));
                memset(global_context->i_autoth, (int)NULL, sizeof(global_context->i_autoth));
                memset(global_context->f_autoth, (int)NULL, sizeof(global_context->f_autoth));
                XPLMSetDataf(global_context->nullzone[0], global_context->prefs_nullzone[0]);
                XPLMSetDataf(global_context->nullzone[1], global_context->prefs_nullzone[1]);
                XPLMSetDataf(global_context->nullzone[2], global_context->prefs_nullzone[2]);
                XPLMSetDataf(global_context->acf_roll_co, global_context->nominal_roll_coef);
                if (XPIsWidgetVisible(global_context->widgetid[1]) != 0)
                {
                    XPHideWidget(global_context->widgetid[0]);
                    XPHideWidget(global_context->widgetid[1]);
                }
                global_context->i_context_init_done = 0;
                global_context->use_320ultimate_api = 0;
                global_context->f_thr_array = NULL;
                return;
            }
            break;

        case XPLM_MSG_PLANE_LOADED:
            if (inParam == XPLM_USER_AIRCRAFT) // user's plane changing
            {
                global_context->minimum_null_zone = 0.025f;
                global_context->nominal_roll_coef = XPLMGetDataf(global_context->acf_roll_co);
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

        case XPLM_MSG_LIVERY_LOADED:
            if (inParam == XPLM_USER_AIRCRAFT && !global_context->i_context_init_done) // wait until aircraft plugins loaded (for custom datarefs)
            {
                XPLMPluginID test = XPLM_NO_PLUGIN_ID;

                // check for all supported autopilot/servo/autothrottle datarefs
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
                for (size_t i = 0, j = 0, k = (sizeof(global_context->i_autoth) / sizeof(global_context->i_autoth[0])); i_autot_dataref_names[i] != NULL && j < k; i++)
                {
                    if (NULL != (global_context->i_autoth[j] = XPLMFindDataRef(i_autot_dataref_names[i])))
                    {
                        j++;
                    }
                }
                for (size_t i = 0, j = 0, k = (sizeof(global_context->f_autoth) / sizeof(global_context->f_autoth[0])); f_autot_dataref_names[i] != NULL && j < k; i++)
                {
                    if (NULL != (global_context->f_autoth[j] = XPLMFindDataRef(f_autot_dataref_names[i])))
                    {
                        j++;
                    }
                }

                // check for custom throttle datarefs/API
                if ((XPLM_NO_PLUGIN_ID != (test = XPLMFindPluginBySignature(XPLM_FF_SIGNATURE))) && (XPLMIsPluginEnabled(test)))
                {
                    global_context->use_320ultimate_api = -1;
                }
                else if (((XPLM_NO_PLUGIN_ID != (test = XPLMFindPluginBySignature("XP10.ToLiss.A319.systems"))) && (XPLMIsPluginEnabled(test))) ||
                         ((XPLM_NO_PLUGIN_ID != (test = XPLMFindPluginBySignature("XP10.ToLiss.A321.systems"))) && (XPLMIsPluginEnabled(test))) ||
                         ((XPLM_NO_PLUGIN_ID != (test = XPLMFindPluginBySignature("XP11.ToLiss.A319.systems"))) && (XPLMIsPluginEnabled(test))) ||
                         ((XPLM_NO_PLUGIN_ID != (test = XPLMFindPluginBySignature("XP11.ToLiss.A321.systems"))) && (XPLMIsPluginEnabled(test))) ||
                         ((XPLM_NO_PLUGIN_ID != (test = XPLMFindPluginBySignature(   "ToLiSs.Airbus.systems"))) && (XPLMIsPluginEnabled(test)))) // A350v1.4.8
                {
                    global_context->f_thr_array = XPLMFindDataRef("AirbusFBW/throttle_input");
                }

                // init data, start flightloop callback
                global_context->i_context_init_done = 1;
                global_context->ice_detect_positive = 0;
                global_context->icecheck_required = 0.0f;
                global_context->show_throttle_all = 0.0f;
                global_context->last_throttle_all = XPLMGetDataf(global_context->f_throttall);
                XPLMSetFlightLoopCallbackInterval(global_context->f_l_cb, 1, 1, global_context);
                return;
            }
            break;

        default:
            break;
    }
}

#define T_ZERO           (00.001f)
#define AIRSPEED_MIN_KTS (50.000f)
#define AIRSPEED_MAX_KTS (62.500f)
#define GROUNDSP_MIN_KTS (03.125f)
#define GROUNDSP_MAX_KTS (31.250f)
#define GROUNDSP_KTS_V00 (02.500f)
#define GROUNDSP_KTS_V01 (26.250f)
#define GROUNDSP_KTS_V02 (50.000f)
#define ACF_ROLL_SET(_var, _gs, _base)                         \
{   if (_gs <= GROUNDSP_KTS_V02)                               \
    {                                                          \
        _var  = ((_gs - GROUNDSP_KTS_V00) / 950.0f) + _base;   \
    }                                                          \
    if (_gs >= GROUNDSP_KTS_V01)                               \
    {                                                          \
        _var -= ((_gs - GROUNDSP_KTS_V01) / 475.0f);           \
    }                                                          \
}
static float callback_hdlr(float inElapsedSinceLastCall,
                           float inElapsedTimeSinceLastFlightLoop,
                           int   inCounter,
                           void *inRefcon)
{
    if (inRefcon)
    {
        xnz_context *ctx = inRefcon;
        float f_throttall, array[2];
        float airspeed = XPLMGetDataf(ctx->f_air_speed);
        float groundsp = MPS2KTS(XPLMGetDataf(ctx->f_grd_speed));

        /* X-Plane 10: update ground roll friction coefficient as required */
        if (ctx->i_version_simulator < 11000)
        {
            if (XPLMGetDatai(ctx->ongroundany) && groundsp > GROUNDSP_KTS_V00 && groundsp < GROUNDSP_KTS_V02)
            {
                float arc; ACF_ROLL_SET(arc, groundsp, ctx->nominal_roll_coef);
                XPLMSetDataf(ctx->acf_roll_co, arc);
            }
            XPLMSetDataf(ctx->acf_roll_co, ctx->nominal_roll_coef);
        }

        if (ctx->use_320ultimate_api == -1)
        {
            XPLMSendMessageToPlugin(XPLMFindPluginBySignature(XPLM_FF_SIGNATURE), XPLM_FF_MSG_GET_SHARED_INTERFACE, &ctx->s);
            if (ctx->s.DataVersion != NULL && ctx->s.DataAddUpdate != NULL)
            {
                ctx->id_f32_eng_lever_lt = ctx->s.ValueIdByName("Aircraft.Cockpit.Pedestal.EngineLever1");
                ctx->id_f32_eng_lever_rt = ctx->s.ValueIdByName("Aircraft.Cockpit.Pedestal.EngineLever2");
                if (ctx->id_f32_eng_lever_lt > -1 && ctx->id_f32_eng_lever_rt > -1)
                {
                    ctx->use_320ultimate_api = 1;
                }
            }
        }

        /* check autopilot and autothrust status */
        int autopilot_servos_on = 0, autothrottle_active = 0;
        size_t ip1 = 0, ip2 = sizeof(ctx->i_servos) / sizeof(ctx->i_servos[0]);
        size_t fp1 = 0, fp2 = sizeof(ctx->f_servos) / sizeof(ctx->f_servos[0]);
        size_t ia1 = 0, ia2 = sizeof(ctx->i_autoth) / sizeof(ctx->i_autoth[0]);
        size_t fa1 = 0, fa2 = sizeof(ctx->f_autoth) / sizeof(ctx->f_autoth[0]);
        while (autopilot_servos_on == 0 && ip1 < ip2 && NULL != ctx->i_servos[ip1])
        {
            if (XPLMGetDatai(ctx->i_servos[ip1]) > 0)
            {
                autopilot_servos_on = 1;
                break;
            }
            ip1++; continue;
        }
        while (autopilot_servos_on == 0 && fp1 < fp2 && NULL != ctx->f_servos[fp1])
        {
            if (XPLMGetDataf(ctx->f_servos[fp1]) > 0.5f)
            {
                autopilot_servos_on = 1;
                break;
            }
            fp1++; continue;
        }
        // Airbus A/T doesn't move levers, status not relevant for us
        if (ctx->use_320ultimate_api <= 0 && ctx->f_thr_array == NULL)
        {
            while (autothrottle_active == 0 && ia1 < ia2 && NULL != ctx->i_autoth[ia1])
            {
                if (XPLMGetDatai(ctx->i_autoth[ia1]) > 0)
                {
                    autothrottle_active = 1;
                    break;
                }
                ia1++; continue;
            }
            while (autothrottle_active == 0 && fa1 < fa2 && NULL != ctx->f_autoth[fa1])
            {
                if (XPLMGetDataf(ctx->f_autoth[fa1]) > 0.5f)
                {
                    autothrottle_active = 1;
                    break;
                }
                fa1++; continue;
            }
        }

        /* throttle readout (overlay) */
        if (ctx->use_320ultimate_api > 0)
        {
            // Pedestal.EngineLever*: 0-20-65 (reverse-idle-max)
            ctx->s.ValueGet(ctx->id_f32_eng_lever_lt, &array[0]);
            ctx->s.ValueGet(ctx->id_f32_eng_lever_rt, &array[1]);
            if ((f_throttall = (((array[0] + array[1]) / 2.0f) - 20.0f) / 45.0f) < 0.0f)
            {
                (f_throttall = (((array[0] + array[1]) / 2.0f) - 20.0f) / 20.0f);
            }
        }
        else if (ctx->f_thr_array != NULL)
        {
            XPLMGetDatavf(ctx->f_thr_array, array, 0, 2);
            f_throttall = ((array[0] + array[1]) / 2.0f);
        }
        else
        {
            f_throttall = XPLMGetDataf(ctx->f_throttall);
        }
        if (fabsf(ctx->last_throttle_all - f_throttall) > 0.02f) // 2.0%
        {
            ctx->throttle_did_change = 1;
            ctx->show_throttle_all = 3.0f;
            ctx->last_throttle_all = f_throttall;
        }
        if (ctx->show_throttle_all < T_ZERO ||
            ctx->ice_detect_positive ||
            autothrottle_active)
        {
            ctx->throttle_did_change = 0;
            ctx->show_throttle_all = 0.0f;
        }
        else
        {
            ctx->show_throttle_all -= inElapsedSinceLastCall;
        }

        /* icing detection: every 10 seconds */
        if ((ctx->icecheck_required += inElapsedSinceLastCall) >= 10.0f)
        {
            if (XPLMGetDataf(ctx->f_ice_rf[0]) > 0.04f ||
                XPLMGetDataf(ctx->f_ice_rf[1]) > 0.04f ||
                XPLMGetDataf(ctx->f_ice_rf[2]) > 0.04f ||
                XPLMGetDataf(ctx->f_ice_rf[3]) > 0.04f)
            {
                if (ctx->ice_detect_positive == 0)
                {
                    XPSetWidgetDescriptor(ctx->widgetid[1], "ICE");
                    XPLMSpeakString("ice detected");
                }
                ctx->ice_detect_positive = 1;
                ctx->throttle_did_change = 0;
            }
            else if (XPLMGetDataf(ctx->f_ice_rf[0]) < 0.02f &&
                     XPLMGetDataf(ctx->f_ice_rf[1]) < 0.02f &&
                     XPLMGetDataf(ctx->f_ice_rf[2]) < 0.02f &&
                     XPLMGetDataf(ctx->f_ice_rf[3]) < 0.02f)
            {
                ctx->ice_detect_positive = 0;
            }
            ctx->icecheck_required = 0.0f;
        }
        if (ctx->ice_detect_positive || ctx->throttle_did_change)
        {
            if (ctx->throttle_did_change)
            {
                snprintf(ctx->overlay_textbuf, 9, "%5.3f", f_throttall);
                XPSetWidgetDescriptor(ctx->widgetid[1], ctx->overlay_textbuf);
            }
            if (XPIsWidgetVisible(ctx->widgetid[1]) == 0)
            {
                XPShowWidget(ctx->widgetid[0]);
                XPShowWidget(ctx->widgetid[1]);
            }
        }
        else if (groundsp > GROUNDSP_KTS_V00 &&
                 groundsp < GROUNDSP_KTS_V02 &&
                 XPLMGetDatai(ctx->ongroundany))
        {
            snprintf(ctx->overlay_textbuf, 9, "%2.0f kts", groundsp);
            XPSetWidgetDescriptor(ctx->widgetid[1], ctx->overlay_textbuf);
            if (XPIsWidgetVisible(ctx->widgetid[1]) == 0)
            {
                XPShowWidget(ctx->widgetid[0]);
                XPShowWidget(ctx->widgetid[1]);
            }
        }
        else
        {
            if (XPIsWidgetVisible(ctx->widgetid[1]) != 0)
            {
                XPHideWidget(ctx->widgetid[0]);
                XPHideWidget(ctx->widgetid[1]);
            }
        }

        /* variable nullzones */
        if (autopilot_servos_on)
        {
            XPLMSetDataf(ctx->nullzone[0], 0.500f);
            XPLMSetDataf(ctx->nullzone[1], 0.500f);
            XPLMSetDataf(ctx->nullzone[2], 0.500f);
        }
        else
        {
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
        ctx->last_throttle_all = f_throttall;
        return (1.0f / 20.0f); // run often
    }
    XPLMDebugString("x-nullzones [error]: callback_hdlr: inRefcon == NULL, disabling callback");
    return 0;
}
#undef AIRSPEED_MIN_KTS
#undef AIRSPEED_MAX_KTS
#undef GROUNDSP_MIN_KTS
#undef GROUNDSP_MAX_KTS
#undef GROUNDSP_KTS_V00
#undef GROUNDSP_KTS_V01
#undef GROUNDSP_KTS_V02
#undef ACF_ROLL_SET
#undef T_ZERO

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

#undef YAW_NZ_FACTR
#undef MPS2KTS
