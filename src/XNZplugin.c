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

#include <stdlib.h>
#include <string.h>

#include "XPLM/XPLMPlanes.h"
#include "XPLM/XPLMPlugin.h"

/* Miscellaneous data */
int xplane_first_load = 1;
void  *xnz_context = NULL;

int xnz_plugin_start(char *outName, char *outSig, char *outDesc)
{
    strncpy(outName,                                     "X-Nullzones", 255);
    strncpy(outSig,                                     "Rodeo314.XNZ", 255);
    strncpy(outDesc, "Dynamic nullzones and other miscellaneous stuff", 255);

    /* all good */
    XPLMDebugString("navP [info]: xnz_plugin_start OK\n"); return 1;
}

void xnz_plugin_stop(void)
{
    return;
}

int xnz_plugin_enable(void)
{
    /* Initialize context */
    //fixme

    /* all good */
    XPLMDebugString("navP [info]: xnz_plugin_enable OK\n"); return 1;
}

void xnz_plugin_disable(void)
{
    /* close context */
    if (xnz_context)
    {
        //fixme
    }

    /* all good */
    XPLMDebugString("navP [info]: xnz_plugin_disable OK\n");
}

void xnz_plugin_message(XPLMPluginID inFromWho,
                        long         inMessage,
                        void        *inParam)
{
    switch (inMessage)
    {
        case XPLM_MSG_PLANE_CRASHED:
            break;

        case XPLM_MSG_PLANE_LOADED:
            if (xplane_first_load)
            {
                xplane_first_load = 0;
            }
            break;

        case XPLM_MSG_AIRPORT_LOADED:
            break;

        case XPLM_MSG_SCENERY_LOADED:
            xnz_menu_reset(navpmenu_context);
            break;

        case XPLM_MSG_AIRPLANE_COUNT_CHANGED:
            break;

        case XPLM_MSG_PLANE_UNLOADED:
            if (inParam == XPLM_USER_AIRCRAFT) // user's plane changing
            {
                //fixme
            }
            break;

        case XPLM_MSG_WILL_WRITE_PREFS:
            //fixme
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
