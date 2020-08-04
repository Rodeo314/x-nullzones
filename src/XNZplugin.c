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

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "XPLM/XPLMDataAccess.h"
#include "XPLM/XPLMPlanes.h"
#include "XPLM/XPLMPlugin.h"
#include "XPLM/XPLMUtilities.h"

typedef struct
{
    XPLMDataRef f_air_speed;
    XPLMDataRef f_grd_speed;
    XPLMDataRef nullzone[3];
    float prefs_nullzone[3];
}
xnz_context;

xnz_context *context = NULL;

static int xnz_log(const char *format, va_list ap);

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
    /* Initialize context */
    if (NULL == (context = malloc(sizeof(xnz_context))))
    {
        XPLMDebugString("x-nullzones [error]: XPluginEnable failed (malloc)\n"); goto fail;
    }
    if (NULL == (context->nullzone[0] = XPLMFindDataRef("sim/joystick/joystick_pitch_nullzone")))
    {
        XPLMDebugString("x-nullzones [error]: XPluginEnable failed (ref[0])\n"); goto fail;
    }
    if (NULL == (context->nullzone[1] = XPLMFindDataRef("sim/joystick/joystick_roll_nullzone")))
    {
        XPLMDebugString("x-nullzones [error]: XPluginEnable failed (ref[1])\n"); goto fail;
    }
    if (NULL == (context->nullzone[2] = XPLMFindDataRef("sim/joystick/joystick_heading_nullzone")))
    {
        XPLMDebugString("x-nullzones [error]: XPluginEnable failed (ref[2])\n"); goto fail;
    }

    /* all good */
    XPLMDebugString("x-nullzones [info]: XPluginEnable OK\n");
    return 1;

fail:
    if (NULL != context)
    {
        free(context);
        context = NULL;
    }
    return 0;
}

PLUGIN_API void XPluginDisable(void)
{
    /* reset nullzones to default/preferences values */
    XPLMSetDataf(context->nullzone[0], context->prefs_nullzone[0]);
    XPLMSetDataf(context->nullzone[1], context->prefs_nullzone[1]);
    XPLMSetDataf(context->nullzone[2], context->prefs_nullzone[2]);

    /* close context */
    if (NULL != context)
    {
        free(context);
        context = NULL;
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
                context->prefs_nullzone[0] = XPLMGetDataf(context->nullzone[0]);
                context->prefs_nullzone[1] = XPLMGetDataf(context->nullzone[1]);
                context->prefs_nullzone[2] = XPLMGetDataf(context->nullzone[2]);
            }
            break;

        case XPLM_MSG_PLANE_UNLOADED:
            if (inParam == XPLM_USER_AIRCRAFT) // user's plane changing
            {
                XPLMSetDataf(context->nullzone[0], context->prefs_nullzone[0]);
                XPLMSetDataf(context->nullzone[1], context->prefs_nullzone[1]);
                XPLMSetDataf(context->nullzone[2], context->prefs_nullzone[2]);
            }
            break;

        case XPLM_MSG_WILL_WRITE_PREFS:
            XPLMSetDataf(context->nullzone[0], context->prefs_nullzone[0]);
            XPLMSetDataf(context->nullzone[1], context->prefs_nullzone[1]);
            XPLMSetDataf(context->nullzone[2], context->prefs_nullzone[2]);
            break;

        case XPLM_MSG_LIVERY_LOADED:
            if (inParam == XPLM_USER_AIRCRAFT) // custom plugins loaded
            {
                //fixme
            }
            break;

        default:
            break;
    }
}

static int xnz_log(const char *format, va_list ap)
{
    int ret;
    char string[1024];
    ret = vsnprintf(string, sizeof(string), format, ap);
    XPLMDebugString(string);
    return ret;
}
