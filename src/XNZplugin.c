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
#include "XPLM/XPLMMenus.h"
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

#define TCA_IDLE_CTR (0.28f)
#define TCA_CLMB_CTR (0.52f)
#define TCA_FLEX_CTR (0.72f)
#define TCA_DEADBAND (0.04f)

#define T_ZERO                   (00.001f)
#define AIRSPEED_MIN_KTS         (50.000f)
#define AIRSPEED_MAX_KTS         (62.500f)
#define GROUNDSP_MIN_KTS         (03.125f)
#define GROUNDSP_MAX_KTS         (31.250f)
#define GROUNDSP_KTS_MIN         (02.500f)
#define GROUNDSP_KTS_MID         (26.250f)
#define GROUNDSP_KTS_MAX         (50.000f)
#define MPS2KTS(MPS) (MPS * 3.6f / 1.852f)
#define ACF_ROLL_SET(_var, _gs, _base)                         \
{                                                              \
    if (_gs > GROUNDSP_KTS_MID)                                \
    {                                                          \
        _var  = ((_gs - GROUNDSP_KTS_MIN) / 950.0f) + _base;   \
        _var -= ((_gs - GROUNDSP_KTS_MID) / 475.0f);           \
    }                                                          \
    else                                                       \
    {                                                          \
        _var  = ((_gs - GROUNDSP_KTS_MIN) / 950.0f) + _base;   \
    }                                                          \
}

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
    char overly_txt_buf[11];
    int throttle_did_change;
    int ice_detect_positive;
    XPLMDataRef f_ice_rf[4];
    XPWidgetID  widgetid[2];

    XPLMFlightLoop_f f_l_th;
    XPLMDataRef i_stick_ass;
    XPLMDataRef f_stick_val;
    XPLMDataRef f_thr_gener;
    XPLMDataRef f_thr_tolis;
    XPLMDataRef i_prop_mode;
    XPLMDataRef i_ngine_num;
    XPLMDataRef i_ngine_typ;
    XPLMCommandRef rev_togg;
    XPLMCommandRef rev_tog1;
    XPLMCommandRef rev_tog2;
    int i_propmode_value[2];
    int id_propeller_axis_3;
    int asymmetrical_thrust;
    int ddenn_cl300_detents;

    XPLMMenuID id_th_on_off;
    int id_menu_item_on_off;
    int tca_support_enabled;
    int skip_idle_overwrite;
}
xnz_context;

static const char *i_servo_dataref_names[] =
{
    "sim/cockpit2/autopilot/servos_on", // int n boolean Are the servos on? Takes into account FD mode and control-wheel-steering, failures, etc.
    NULL,
};

static const char *f_servo_dataref_names[] =
{
    NULL,
};

static const char *i_autot_dataref_names[] =
{
    "sim/cockpit2/autopilot/autothrottle_on", // int n boolean Auto-throttle really working? Takes into account failures, esys, etc.
    NULL,
};

static const char *f_autot_dataref_names[] =
{
    NULL,
};

xnz_context *global_context = NULL;
int first_xplane_run = 1;

static   int xnz_log       (const char *format, ...);
static float callback_hdlr(float, float, int, void*);
static float throttle_hdlr(float, float, int, void*);
static void  menu_hdlr_fnc(void*,             void*);
static inline float throttle_mapping(float rawvalue);

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
    XPSetWidgetGeometry(global_context->widgetid[0], 0, 56, 64, 0);
    XPSetWidgetGeometry(global_context->widgetid[1], 7, 49, 57, 7);

    /* flight loop callback */
    XPLMRegisterFlightLoopCallback((global_context->f_l_cb = &callback_hdlr), 0, global_context);

    /* initialize arrays */
    memset(global_context->f_servos, (int)NULL, sizeof(global_context->f_servos));
    memset(global_context->i_servos, (int)NULL, sizeof(global_context->i_servos));
    memset(global_context->i_autoth, (int)NULL, sizeof(global_context->i_autoth));
    memset(global_context->f_autoth, (int)NULL, sizeof(global_context->f_autoth));

    /* TCA thrust quadrant support */
    if (NULL == (global_context->i_stick_ass = XPLMFindDataRef("sim/joystick/joystick_axis_assignments")))
    {
        XPLMDebugString("x-nullzones [error]: XPluginEnable failed (i_stick_ass)\n"); goto fail;
    }
    if (NULL == (global_context->f_stick_val = XPLMFindDataRef("sim/joystick/joystick_axis_values")))
    {
        XPLMDebugString("x-nullzones [error]: XPluginEnable failed (f_stick_val)\n"); goto fail;
    }
    if (NULL == (global_context->f_thr_gener = XPLMFindDataRef("sim/cockpit2/engine/actuators/throttle_ratio")))
    {
        XPLMDebugString("x-nullzones [error]: XPluginEnable failed (f_thr_array)\n"); goto fail;
    }
    if (NULL == (global_context->i_prop_mode = XPLMFindDataRef("sim/cockpit2/engine/actuators/prop_mode")))
    {
        XPLMDebugString("x-nullzones [error]: XPluginEnable failed (i_prop_mode)\n"); goto fail;
    }
    if (NULL == (global_context->i_ngine_num = XPLMFindDataRef("sim/aircraft/engine/acf_num_engines")))
    {
        XPLMDebugString("x-nullzones [error]: XPluginEnable failed (i_ngine_num)\n"); goto fail;
    }
    if (NULL == (global_context->i_ngine_typ = XPLMFindDataRef("sim/aircraft/prop/acf_en_type")))
    {
        XPLMDebugString("x-nullzones [error]: XPluginEnable failed (i_ngine_typ)\n"); goto fail;
    }
    if (NULL == (global_context->rev_togg = XPLMFindCommand("sim/engines/thrust_reverse_toggle")))
    {
        XPLMDebugString("x-nullzones [error]: XPluginEnable failed (rev_togg)\n"); goto fail;
    }
    if (NULL == (global_context->rev_tog1 = XPLMFindCommand("sim/engines/thrust_reverse_toggle_1")))
    {
        XPLMDebugString("x-nullzones [error]: XPluginEnable failed (rev_tog1)\n"); goto fail;
    }
    if (NULL == (global_context->rev_tog2 = XPLMFindCommand("sim/engines/thrust_reverse_toggle_2")))
    {
        XPLMDebugString("x-nullzones [error]: XPluginEnable failed (rev_tog2)\n"); goto fail;
    }
    XPLMRegisterFlightLoopCallback((global_context->f_l_th = &throttle_hdlr), 0, global_context);

    /* TCA quadrant support: toggle on/off via menu */
    if (NULL == (global_context->id_th_on_off = XPLMCreateMenu("x-nullzones", NULL, 0, &menu_hdlr_fnc, global_context)))
    {
        XPLMDebugString("x-nullzones [error]: XPluginEnable failed (XPLMCreateMenu)\n"); goto fail;
    }
    if (0 > (global_context->id_menu_item_on_off = XPLMAppendMenuItem(global_context->id_th_on_off, "TCA Throttle Quadrant", &global_context->id_menu_item_on_off, 0)))
    {
        XPLMDebugString("x-nullzones [error]: XPluginEnable failed (XPLMAppendMenuItem)\n"); goto fail;
    }
    XPLMCheckMenuItem(global_context->id_th_on_off, global_context->id_menu_item_on_off, xplm_Menu_Checked);

    /* all good */
    XPLMDebugString("x-nullzones [info]: XPluginEnable OK\n");
    global_context->i_version_simulator = outXPlaneVersion;
    global_context->i_version_xplm_apis = outXPLMVersion;
    global_context->id_propeller_axis_3 = -1;
    global_context->tca_support_enabled = 1;
    global_context->skip_idle_overwrite = 0;
    global_context->i_context_init_done = 0;
    global_context->use_320ultimate_api = 0;
    global_context->ddenn_cl300_detents = 0;
    global_context->f_thr_array = NULL;
    global_context->f_thr_tolis = NULL;
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

static void xnz_context_reset(xnz_context *ctx)
{
    if (ctx)
    {
        XPLMSetFlightLoopCallbackInterval(ctx->f_l_cb, 0, 1, ctx);
        XPLMSetFlightLoopCallbackInterval(ctx->f_l_th, 0, 1, ctx);
        memset(ctx->f_servos, (int)NULL, sizeof(ctx->f_servos));
        memset(ctx->i_servos, (int)NULL, sizeof(ctx->i_servos));
        memset(ctx->i_autoth, (int)NULL, sizeof(ctx->i_autoth));
        memset(ctx->f_autoth, (int)NULL, sizeof(ctx->f_autoth));
        XPLMSetDataf(ctx->nullzone[0], ctx->prefs_nullzone[0]);
        XPLMSetDataf(ctx->nullzone[1], ctx->prefs_nullzone[1]);
        XPLMSetDataf(ctx->nullzone[2], ctx->prefs_nullzone[2]);
        XPLMSetDataf(ctx->acf_roll_co, ctx->nominal_roll_coef);
        if (ctx->id_propeller_axis_3 >= 0)
        {
            int pr_axis_ass[2] = { 26, 27, };
            XPLMSetDatavi(ctx->i_stick_ass, pr_axis_ass, ctx->id_propeller_axis_3, 2);
        }
        if (XPIsWidgetVisible(ctx->widgetid[1]) != 0)
        {
            XPHideWidget(ctx->widgetid[0]);
            XPHideWidget(ctx->widgetid[1]);
        }
        ctx->i_context_init_done = 0;
        ctx->use_320ultimate_api = 0;
        ctx->ddenn_cl300_detents = 0;
        ctx->skip_idle_overwrite = 0;
        ctx->f_thr_array = NULL;
    }
}

PLUGIN_API void XPluginDisable(void)
{
    /* cleanup */
    xnz_context_reset(global_context);

    /* flight loop callback */
    XPLMUnregisterFlightLoopCallback(global_context->f_l_cb, global_context);
    XPLMUnregisterFlightLoopCallback(global_context->f_l_th, global_context);

    /* re-enable prop 3/4 axes */
    if (global_context->id_propeller_axis_3 >= 0)
    {
        int pr_axis_ass[2] = { 26, 27, };
        XPLMSetDatavi(global_context->i_stick_ass, pr_axis_ass, global_context->id_propeller_axis_3, 2);
    }

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

static inline void dref_read_str(XPLMDataRef ref, char *buf, size_t siz)
{
    if (ref && buf && siz)
    {
        int len = XPLMGetDatab(ref, buf, 0, siz - 1);
        if (len > 0)
        {
            buf[siz - 1] = '\0';
            return;
        }
        buf[0] = '\0';
        return;
    }
    return;
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
            if (global_context->id_propeller_axis_3 >= 0)
            {
                int pr_axis_ass[2] = { 26, 27, };
                XPLMSetDatavi(global_context->i_stick_ass, pr_axis_ass, global_context->id_propeller_axis_3, 2);
            }
            return;

        case XPLM_MSG_PLANE_UNLOADED:
            if (inParam == XPLM_USER_AIRCRAFT) // user's plane changing
            {
                return xnz_context_reset(global_context);
            }
            break;

        case XPLM_MSG_PLANE_LOADED:
            if (inParam == XPLM_USER_AIRCRAFT) // user's plane changing
            {
                global_context->i_context_init_done = 0;
                return;
            }
            break;

        case XPLM_MSG_LIVERY_LOADED:
            if (inParam == XPLM_USER_AIRCRAFT && !global_context->i_context_init_done) // wait until aircraft plugins loaded (for custom datarefs)
            {
                XPLMPluginID test = XPLM_NO_PLUGIN_ID;

                global_context->minimum_null_zone = 0.04f; // hardcoded for now
                global_context->nominal_roll_coef = XPLMGetDataf(global_context->acf_roll_co);
                global_context->prefs_nullzone[0] = XPLMGetDataf(global_context->nullzone[0]);
                global_context->prefs_nullzone[1] = XPLMGetDataf(global_context->nullzone[1]);
                global_context->prefs_nullzone[2] = XPLMGetDataf(global_context->nullzone[2]);
                xnz_log("x-nullzones: new aircraft: original nullzones %.3lf %.3lf %.3lf (minimum %.3lf)\n",
                        global_context->prefs_nullzone[0],
                        global_context->prefs_nullzone[1],
                        global_context->prefs_nullzone[2],
                        global_context->minimum_null_zone);

                if (global_context->i_version_simulator < 11000)
                {
                    float rc0; ACF_ROLL_SET(rc0, GROUNDSP_KTS_MIN, global_context->nominal_roll_coef);
                    float rc1; ACF_ROLL_SET(rc1, GROUNDSP_KTS_MID, global_context->nominal_roll_coef);
                    float rc2; ACF_ROLL_SET(rc2, GROUNDSP_KTS_MAX, global_context->nominal_roll_coef);
                    xnz_log("x-nullzones: new aircraft: original roll coefficient %.3lf (%.3lf -> %.3lf -> %.3lf)\n",
                            global_context->nominal_roll_coef, rc0, rc1, rc2);
                }

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
                         ((XPLM_NO_PLUGIN_ID != (test = XPLMFindPluginBySignature(   "ToLiSs.Airbus.systems"))) && (XPLMIsPluginEnabled(test))) || // A350v1.4.8
                         ((XPLM_NO_PLUGIN_ID != (test = XPLMFindPluginBySignature("XP11.ToLiss.Airbus.systems"))) && (XPLMIsPluginEnabled(test)))) // A350v1.6++
                {
                    global_context->f_thr_array = XPLMFindDataRef("AirbusFBW/throttle_input");
                }
                else if (XPLM_NO_PLUGIN_ID != XPLMFindPluginBySignature("1-sim.sasl"))
                {
                    XPLMDataRef ref = NULL; char auth[501], desc[261], icao[41];
                    if ((ref = XPLMFindDataRef("sim/aircraft/view/acf_author")))
                    {
                        dref_read_str(ref, auth, sizeof(auth));
                        if ((ref = XPLMFindDataRef("sim/aircraft/view/acf_descrip")))
                        {
                            dref_read_str(ref, desc, sizeof(desc));
                            if ((ref = XPLMFindDataRef("sim/aircraft/view/acf_ICAO")))
                            {
                                dref_read_str(ref, icao, sizeof(icao));
                                if (!strncasecmp(auth, "Denis 'ddenn' Krupin", strlen("Denis 'ddenn' Krupin")) &&
                                    !strncasecmp(desc, "Bombardier Challenger 300", strlen("Bombardier Challenger 300")))
                                {
                                    global_context->ddenn_cl300_detents = 1;
                                }
                            }
                        }
                    }
                }

                /* TCA thrust quadrant support */
                if (global_context->id_propeller_axis_3 < 0)
                {
                    size_t size = global_context->i_version_simulator < 11000 ? 100 : 500;
                    for (size_t i = 0; i < size - 1; i++)
                    {
                        int i_stick_ass[2]; XPLMGetDatavi(global_context->i_stick_ass, i_stick_ass, i, 2);
                        if (i_stick_ass[0] == 26 && i_stick_ass[1] == 27)
                        {
                            xnz_log("x-nullzones: found prop 3/4 axes at index (%02zd, %02zd) with assignment (%02d, %02d)\n", i, i + 1, i_stick_ass[0], i_stick_ass[1]);
                            global_context->id_propeller_axis_3 = i;
                            break;
                        }
                    }
                    if (global_context->id_propeller_axis_3 >= 0 && 0 /* debug */)
                    {
                        float last = -2.0f;
                        for (int i = 0; i <= 200; i++)
                        {
                            float input, value = throttle_mapping((input = ((float)i / 200.0f)));
                            if (value < last)
                            {
                                xnz_log("x-nullzones [debug]: non-monotonically increasing throttle mapping, %.4f -> %.4f\n", last, value);
                                exit(-1);
                            }
                            xnz_log("x-nullzones [debug]: throttle_mapping(%.3f) = %.4f\n", input, (last = value));
                        }
                    }
                }
                if (global_context->id_propeller_axis_3 >= 0)
                {
                    int acf_en_type[3], acf_en_num = XPLMGetDatai(global_context->i_ngine_num);
                    if (acf_en_num >= 2) // some Carenado have extra engine 4 special effects
                    {
                        if (acf_en_num > 3)
                        {
                            acf_en_num = 3; // whether we have at least 3 engines of same type
                        }
                        XPLMGetDatavi(global_context->i_ngine_typ, acf_en_type, 0, acf_en_num);
                        int dummy = acf_en_type[acf_en_num - 1] != acf_en_type[acf_en_num - 2];
                        global_context->asymmetrical_thrust = !(((2 != (acf_en_num - dummy))));
                    }
                    else
                    {
                        global_context->asymmetrical_thrust = 0;
                    }
                    if (global_context->id_propeller_axis_3 >= 0)
                    {
                        int no_axis_ass[2] = { 0, 0, };
                        XPLMSetDatavi(global_context->i_stick_ass, no_axis_ass, global_context->id_propeller_axis_3, 2);
                    }
                    global_context->skip_idle_overwrite = 0;
                    global_context->f_thr_tolis = global_context->f_thr_array;
                    XPLMSetFlightLoopCallbackInterval(global_context->f_l_th, 1, 1, global_context);
                    xnz_log("x-nullzones: setting TCA flightloop callback interval (enabled: %s)\n",
                            global_context->tca_support_enabled == 0 ? "no" : "yes");
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
            if (XPLMGetDatai(ctx->ongroundany) && groundsp > GROUNDSP_KTS_MIN && groundsp < GROUNDSP_KTS_MAX)
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
        if (fabsf(ctx->last_throttle_all - f_throttall) >= 0.0025f)
        {
            ctx->throttle_did_change = 1;
            ctx->show_throttle_all = 3.0f;
            ctx->last_throttle_all = f_throttall;
            if (ctx->tca_support_enabled && ctx->id_propeller_axis_3 >= 0)
            {
                ctx->show_throttle_all = 1.5f;
            }
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
                snprintf(ctx->overly_txt_buf, 11, "%7.5f", f_throttall);
                XPSetWidgetDescriptor(ctx->widgetid[1], ctx->overly_txt_buf);
            }
            if (XPIsWidgetVisible(ctx->widgetid[1]) == 0)
            {
                XPShowWidget(ctx->widgetid[0]);
                XPShowWidget(ctx->widgetid[1]);
            }
        }
        else if (groundsp > GROUNDSP_KTS_MIN &&
                 groundsp < GROUNDSP_KTS_MAX &&
                 XPLMGetDatai(ctx->ongroundany))
        {
            snprintf(ctx->overly_txt_buf, 9, "%2.0f kts", groundsp);
            XPSetWidgetDescriptor(ctx->widgetid[1], ctx->overly_txt_buf);
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
            float nullzone_pitch_roll = 0.125f - ((0.125f - ctx->minimum_null_zone) * ((airspeed - AIRSPEED_MIN_KTS) / (AIRSPEED_MAX_KTS - AIRSPEED_MIN_KTS)));
            float nullzone_yaw_tiller = 0.250f - ((0.250f - ctx->minimum_null_zone) * ((groundsp - GROUNDSP_MIN_KTS) / (GROUNDSP_MAX_KTS - GROUNDSP_MIN_KTS)));
            XPLMSetDataf(ctx->nullzone[0], nullzone_pitch_roll);
            XPLMSetDataf(ctx->nullzone[1], nullzone_pitch_roll);
            XPLMSetDataf(ctx->nullzone[2], nullzone_yaw_tiller);
        }
        if (ctx->tca_support_enabled == 0 || ctx->id_propeller_axis_3 < 0)
        {
            ctx->last_throttle_all = f_throttall;
        }
        return (1.0f / 20.0f); // run often
    }
    XPLMDebugString("x-nullzones [error]: callback_hdlr: inRefcon == NULL, disabling callback");
    return 0;
}

static inline float non_linear_standard(float linear_val)
{
    return sqrtf(linear_val);
}

static inline float non_linear_inverted(float linear_val)
{
    return 1.0f - sqrtf(1.0f - linear_val);
}

static inline float non_linear_centered(float linear_val)
{
    if (linear_val < 0.0f)
    {
        return 0.0f;
    }
    if (linear_val > 1.0f)
    {
        return 1.0f;
    }
    if (linear_val < 0.5f)
    {
        float min = 0.0f, max = 0.5f;
        float val = (linear_val - min) / (max - min);
        return min + 0.5f * non_linear_inverted(val);
    }
    if (linear_val > 0.5f)
    {
        float min = 0.5f, max = 1.0f;
        float val = (linear_val - min) / (max - min);
        return min + 0.5f * non_linear_standard(val);
    }
    return 0.5f;
}

static inline float throttle_mapping_ddcl30(float rawvalue)
{
    if (rawvalue <= (TCA_IDLE_CTR - TCA_DEADBAND))
    {
        if (rawvalue < TCA_DEADBAND)
        {
            return -1.0f;
        }
        float min = TCA_DEADBAND;
        float max = TCA_IDLE_CTR - TCA_DEADBAND;
        float val = (rawvalue - min) / (max - min);
        return 0.9f * non_linear_inverted(val) - 1.0f;
    }
    if (rawvalue > (TCA_FLEX_CTR + 0.5f * (1.0f - TCA_DEADBAND - TCA_FLEX_CTR)))
    {
        return 2.8f / 3.0f; // TO
    }
    if (rawvalue > (TCA_CLMB_CTR + 0.5f * (TCA_FLEX_CTR - TCA_CLMB_CTR)))
    {
        return 2.6f / 3.0f; // CLB
    }
    if (rawvalue > (TCA_CLMB_CTR - TCA_DEADBAND))
    {
        return 2.5f / 3.0f; // CRZ
    }
    if (rawvalue > (TCA_IDLE_CTR + TCA_DEADBAND))
    {
        float mx = TCA_CLMB_CTR - TCA_DEADBAND;
        float mn = TCA_IDLE_CTR + TCA_DEADBAND;
        float ve = (rawvalue - mn) / (mx - mn);
        return 2.4f / 3.0f * non_linear_centered(ve);
    }
    return 0.0f; // default to forward idle
}

static inline float throttle_mapping_toliss(float rawvalue)
{
    if (rawvalue <= (TCA_IDLE_CTR - TCA_DEADBAND))
    {
        if (rawvalue < TCA_DEADBAND)
        {
            return -1.0f;
        }
        float min = TCA_DEADBAND;
        float max = TCA_IDLE_CTR - TCA_DEADBAND;
        float val = (rawvalue - min) / (max - min);
        return 0.9f * non_linear_inverted(val) - 1.0f;
    }
    if (rawvalue > (TCA_FLEX_CTR + 0.5f * (1.0f - TCA_DEADBAND - TCA_FLEX_CTR)))
    {
        return 1.00f; // TOGA
    }
    if (rawvalue > (TCA_CLMB_CTR + 0.5f * (TCA_FLEX_CTR - TCA_CLMB_CTR)))
    {
        return 0.87f; // FLEX
    }
    if (rawvalue > (TCA_CLMB_CTR - TCA_DEADBAND))
    {
        return 0.69f; // CLB
    }
    if (rawvalue > (TCA_IDLE_CTR + TCA_DEADBAND))
    {
        float mx = TCA_CLMB_CTR - TCA_DEADBAND;
        float mn = TCA_IDLE_CTR + TCA_DEADBAND;
        float ve = (rawvalue - mn) / (mx - mn);
        return 0.68f * non_linear_centered(ve);
    }
    return 0.0f; // default to forward idle
}

/* ******* Example: mappings ******* *
 *************************************
 ** IDLE *** CLMB *** FLEX *** TOGA **
 *************************************
 * 0.000f | linear | 0.750f | 1.000f *
 * 0.000f | 0.700f | linear | 1.000f *
 * 0.000f | 0.750f | linear | 1.000f *
 * 0.000f | linear | 0.875f | 1.000f *
 * 0.000f | 0.500f | curved | 1.000f * <------
 *************************************
 */
static inline float throttle_mapping(float rawvalue)
{
#if 0
    return throttle_mapping_toliss(rawvalue); // debug
#endif
    if (rawvalue <= (TCA_IDLE_CTR - TCA_DEADBAND))
    {
        if (rawvalue < TCA_DEADBAND)
        {
            return -1.0f;
        }
        float min = TCA_DEADBAND;
        float max = TCA_IDLE_CTR - TCA_DEADBAND;
        float val = (rawvalue - min) / (max - min);
        return 0.9f * non_linear_inverted(val) - 1.0f;
    }
    if (rawvalue > (1.0f - TCA_DEADBAND))
    {
        return 1.0f;
    }
    if (rawvalue > (TCA_CLMB_CTR + TCA_DEADBAND))
    {
        float mx = 1.0f - TCA_DEADBAND;
        float mn = TCA_CLMB_CTR + TCA_DEADBAND;
        float ve = (rawvalue - mn) / (mx - mn);
        return 0.5f + 0.5f * non_linear_standard(ve);
    }
    if (rawvalue > (TCA_CLMB_CTR - TCA_DEADBAND))
    {
        return 0.5f; // wide detent at 50% thrust
    }
    if (rawvalue > (TCA_IDLE_CTR + TCA_DEADBAND))
    {
        float mx = TCA_CLMB_CTR - TCA_DEADBAND;
        float mn = TCA_IDLE_CTR + TCA_DEADBAND;
        float ve = (rawvalue - mn) / (mx - mn);
        return 0.5f * non_linear_inverted(ve);
    }
    return 0.0f; // default to forward idle
}

static float throttle_hdlr(float inElapsedSinceLastCall,
                           float inElapsedTimeSinceLastFlightLoop,
                           int   inCounter,
                           void *inRefcon)
{
    if (inRefcon)
    {
        float f_stick_val[2];
        xnz_context *ctx = inRefcon;
        int symmetrical_thrust = !ctx->asymmetrical_thrust;

        /* shall we be doing something? */
        if (ctx->tca_support_enabled == 0)
        {
            return (1.0f / 20.0f);
        }

        /* check autothrust status */
        if (ctx->f_thr_tolis == NULL && ctx->use_320ultimate_api < 1)
        {   // Airbus A/T doesn't move levers, status not relevant for us
            size_t ia1 = 0, ia2 = sizeof(ctx->i_autoth) / sizeof(ctx->i_autoth[0]);
            size_t fa1 = 0, fa2 = sizeof(ctx->f_autoth) / sizeof(ctx->f_autoth[0]);
            while (ia1 < ia2 && NULL != ctx->i_autoth[ia1])
            {
                if (XPLMGetDatai(ctx->i_autoth[ia1]) > 0)
                {
                    return (1.0f / 20.0f);
                }
                ia1++; continue;
            }
            while (fa1 < fa2 && NULL != ctx->f_autoth[fa1])
            {
                if (XPLMGetDataf(ctx->f_autoth[fa1]) > 0.5f)
                {
                    return (1.0f / 20.0f);
                }
                fa1++; continue;
            }
        }

        /* now we read the raw axis values */
        XPLMGetDatavf(ctx->f_stick_val, f_stick_val, ctx->id_propeller_axis_3, 2);
        if (first_xplane_run)
        {
            if (f_stick_val[0] < TCA_DEADBAND || f_stick_val[1] < TCA_DEADBAND)
            {
                return (1.0f / 20.0f); // haven't received input from hardware yet
            }
            first_xplane_run = 0;
        }
        if (fabsf(f_stick_val[0] - f_stick_val[1]) < TCA_DEADBAND)
        {
            symmetrical_thrust = 1;
        }
        if (symmetrical_thrust)
        {
            float addition = f_stick_val[0] + f_stick_val[1];
            if (0.0f == addition)
            {
                if (ctx->skip_idle_overwrite)
                {
                    return (1.0f / 20.0f);
                }
                /*
                 * Don't keep writing idle thrust over and over again.
                 * This allows simmers to use other means of controlling aircraft
                 * thrust when both TCA levers are set in an idle detent position.
                 */
                f_stick_val[0] = f_stick_val[1] = 0.0f;
                ctx->skip_idle_overwrite = 1;
            }
            else
            {
                f_stick_val[0] = addition / 2.0f;
                f_stick_val[1] = f_stick_val[0];
                ctx->skip_idle_overwrite = 0;
            }
        }
        if (ctx->use_320ultimate_api > 0)
        {
            return (1.0f / 20.0f); // TODO: implement
        }
        if (ctx->f_thr_tolis)
        {
            // note: A350 reverse thrust only works with X-Plane 11 (v1.6+)…
            f_stick_val[0] = throttle_mapping_toliss(1.0f - f_stick_val[0]);
            f_stick_val[1] = throttle_mapping_toliss(1.0f - f_stick_val[1]);
            XPLMSetDatavf(ctx->f_thr_tolis, f_stick_val, 0, 2);
            return (1.0f / 20.0f);
        }
        if (symmetrical_thrust)
        {
            XPLMGetDatavi(ctx->i_prop_mode, ctx->i_propmode_value, 0, 1);
            if (ctx->ddenn_cl300_detents)
            {
                f_stick_val[0] = throttle_mapping_ddcl30(1.0f - f_stick_val[0]);
            }
            else
            {
                f_stick_val[0] = throttle_mapping(1.0f - f_stick_val[0]);
            }
            if (f_stick_val[0] < 0.0f)
            {
                if (ctx->i_propmode_value[0] < 3)
                {
                    XPLMCommandOnce(ctx->rev_togg);
                }
            }
            else
            {
                if (ctx->i_propmode_value[0] > 1)
                {
                    XPLMCommandOnce(ctx->rev_togg);
                }
            }
            XPLMSetDataf(ctx->f_throttall, fabsf(f_stick_val[0]));
            return (1.0f / 20.0f);
        }
        XPLMGetDatavi(ctx->i_prop_mode, ctx->i_propmode_value, 0, 2);
        if (ctx->ddenn_cl300_detents)
        {
            f_stick_val[0] = throttle_mapping_ddcl30(1.0f - f_stick_val[0]);
            f_stick_val[1] = throttle_mapping_ddcl30(1.0f - f_stick_val[1]);
        }
        else
        {
            f_stick_val[0] = throttle_mapping(1.0f - f_stick_val[0]);
            f_stick_val[1] = throttle_mapping(1.0f - f_stick_val[1]);
        }
        if (f_stick_val[0] < 0.0f)
        {
            if (ctx->i_propmode_value[0] < 3)
            {
                XPLMCommandOnce(ctx->rev_tog1);
            }
        }
        else
        {
            if (ctx->i_propmode_value[0] > 1)
            {
                XPLMCommandOnce(ctx->rev_tog1);
            }
        }
        if (f_stick_val[1] < 0.0f)
        {
            if (ctx->i_propmode_value[1] < 3)
            {
                XPLMCommandOnce(ctx->rev_tog2);
            }
        }
        else
        {
            if (ctx->i_propmode_value[1] > 1)
            {
                XPLMCommandOnce(ctx->rev_tog2);
            }
        }
        f_stick_val[0] = fabsf(f_stick_val[0]);
        f_stick_val[1] = fabsf(f_stick_val[1]);
        XPLMSetDatavf(ctx->f_thr_gener, f_stick_val, 0, 2);
        return (1.0f / 20.0f);
    }
    XPLMDebugString("x-nullzones [error]: throttle_hdlr: inRefcon == NULL, disabling callback");
    return 0;
}

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

static void menu_hdlr_fnc(void *inMenuRef, void *inItemRef)
{
    if (inMenuRef)
    {
        if (inItemRef)
        {
            int *item = inItemRef;
            xnz_context *ctx = inMenuRef;
            if (*item == ctx->id_menu_item_on_off)
            {
                XPLMMenuCheck state = xplm_Menu_Checked;
                XPLMCheckMenuItemState(ctx->id_th_on_off, ctx->id_menu_item_on_off, &state);
                if (state == xplm_Menu_Checked)
                {
                    if (ctx->id_propeller_axis_3 >= 0)
                    {
                        int pr_axis_ass[2] = { 26, 27, };
                        XPLMSetDatavi(ctx->i_stick_ass, pr_axis_ass, ctx->id_propeller_axis_3, 2);
                    }
                    XPLMCheckMenuItem(ctx->id_th_on_off, ctx->id_menu_item_on_off, xplm_Menu_NoCheck);
                    xnz_log("x-nullzones [info]: menu: disabling TCA flight loop callback\n");
                    global_context->tca_support_enabled = 0;
                    global_context->skip_idle_overwrite = 0;
                    return;
                }
                if (ctx->id_propeller_axis_3 >= 0)
                {
                    int no_axis_ass[2] = { 0, 0, };
                    XPLMSetDatavi(ctx->i_stick_ass, no_axis_ass, ctx->id_propeller_axis_3, 2);
                }
                XPLMCheckMenuItem(ctx->id_th_on_off, ctx->id_menu_item_on_off, xplm_Menu_Checked);
                xnz_log("x-nullzones [info]: menu: enabling TCA flight loop callback\n");
                global_context->tca_support_enabled = 1;
                global_context->skip_idle_overwrite = 0;
                return;
            }
            return;
        }
        return;
    }
    return;
}

#undef TCA_IDLE_CTR
#undef TCA_CLMB_CTR
#undef TCA_FLEX_CTR
#undef TCA_DEADBAND

#undef AIRSPEED_MIN_KTS
#undef AIRSPEED_MAX_KTS
#undef GROUNDSP_MIN_KTS
#undef GROUNDSP_MAX_KTS
#undef GROUNDSP_KTS_MIN
#undef GROUNDSP_KTS_MID
#undef GROUNDSP_KTS_MAX
#undef ACF_ROLL_SET
#undef MPS2KTS
#undef T_ZERO
