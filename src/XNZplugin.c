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

#define T_ZERO                  (.000001f)
#define T_SMALL                 (00.0025f)
#define AIRSPEED_MIN_KTS        (50.0000f)
#define AIRSPEED_MAX_KTS        (62.5000f)
#define GROUNDSP_MIN_KTS        (03.1250f)
#define GROUNDSP_MAX_KTS        (31.2500f)
#define GROUNDSP_KTS_MIN        (02.5000f)
#define GROUNDSP_KTS_MID        (26.2500f)
#define GROUNDSP_KTS_MAX        (50.0000f)
#define MPS2KPH(MPS) (MPS * 3.6f / 1.000f)
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

enum
{
    ZONE_REV = 0,
    ZONE_CLB = 1,
    ZONE_FLX = 2,
    ZONE_TGA = 3,
    ZONE_MAX = ZONE_TGA,
};

typedef struct
{
    float   min[ZONE_MAX + 1];
    float   max[ZONE_MAX + 1];
    float   len[ZONE_MAX + 1];
    float share[ZONE_MAX + 1];
} thrust_zones;

typedef struct
{
    int xp_11_50_or_later;

    enum
    {
        XNZ_AB_ERRR =  -1,
        XNZ_AB_NONE =   0,
        XNZ_AB_XPLM =   1,
        XNZ_AB_FF32 = 320,
        XNZ_AB_TO32 = 321,
        XNZ_AB_FF35 = 350,
        XNZ_AB_IX33 = 733,
        XNZ_AB_FF75 = 757,
    } xnz_ab;

    enum
    {
        XNZ_AP_ERRR =  -1,
        XNZ_AP_NONE =   0,
        XNZ_AP_XPLM =   1,
        XNZ_AP_COMM =   2,
        XNZ_AP_TOGG =   3,
        XNZ_AP_FF32 = 320,
        XNZ_AP_XGFC = 700, // X-Plane 11.30 or later: GFC-700
    } xnz_ap;

    union
    {
        struct
        {
            XPLMCommandRef cmd_ap_disc;
            XPLMCommandRef cmd_ap_conn;
        } xplm;

        struct
        {
            XPLMCommandRef cmd_ap_disc;
            XPLMCommandRef cmd_ap_conn;
        } comm;
    } ap;

    enum
    {
        XNZ_AT_ERRR =  -1,
        XNZ_AT_NONE =   0,
        XNZ_AT_XPLM =   1,
        XNZ_AT_COMM =   2,
        XNZ_AT_TOGG =   3,
        XNZ_AT_APTO =   4,
        XNZ_AT_XP11 =  11,
        XNZ_AT_FF32 = 320,
        XNZ_AT_TOLI = 321,
    } xnz_at;

    union
    {
        struct
        {
            XPLMCommandRef cmd_at_disc;
            XPLMCommandRef cmd_at_toga;
        } comm;

        struct
        {
            XPLMCommandRef cmd_at_togg;
            XPLMDataRef    ref_at_stts;
        } togg;

        struct
        {
            XPLMCommandRef cmd_ap_toga;
        } toga;
    } at;

    enum
    {
        XNZ_BT_ERRR =  -1,
        XNZ_BT_XPLM =   1,
        XNZ_BT_COMM =   2,
        XNZ_BT_PKBR =   3,
        XNZ_BT_SIMC =   4,
        XNZ_BT_FF32 = 320,
        XNZ_BT_TO32 = 321,
        XNZ_BT_FF35 = 350,
        XNZ_BT_TBM9 = 900,
    } xnz_bt;

    union
    {
        struct
        {
            XPLMCommandRef cmd_current;
            XPLMCommandRef cmd_rgb_hld;
            XPLMCommandRef cmd_mxb_hld;
            const char *commnd_rgb_hld;
            const char *commnd_max_hld;
        } comm;

        struct
        {
        } simc;

        struct
        {
        } ff32;

        struct
        {
            int pbrak_onoff; // for manual braking: remember the pbrak state
        } ff35;

        struct
        {
            XPLMDataRef l_rgb_ratio; // AirbusFBW/BrakePedalInputLeft
            XPLMDataRef r_rgb_ratio; // AirbusFBW/BrakePedalInputRight
            XPLMDataRef br_override; // AirbusFBW/BrakePedalInputOverride
        } to32;

        struct
        {
            XPLMDataRef rbrak_array; // tbm900/controls/gear/brake_req[2]
            XPLMDataRef br_override; // tbm900/controls/gear/brake_req_ovrd
        } tbm9;
    } bt;

    enum
    {
        XNZ_ET_ERRR =  -1,
        XNZ_ET_NONE =   0,
        XNZ_ET_XPJT =   1,
        XNZ_ET_XPTP =   2,
        XNZ_ET_XPPI =   3,
        XNZ_ET_RPTP =  12, // Carenado PC-12 + REP
        XNZ_ET_DA62 =  62,
        XNZ_ET_E35L = 135,
        XNZ_ET_FF32 = 320,
        XNZ_ET_TO32 = 321,
        XNZ_ET_FF35 = 350,
        XNZ_ET_E55P = 535,
        XNZ_ET_PIPA = 540,
        XNZ_ET_EA50 = 610,
        XNZ_ET_EVIC = 617,
        XNZ_ET_CL30 = 700,
        XNZ_ET_IX73 = 733,
        XNZ_ET_FF75 = 757,
        XNZ_ET_TBM9 = 900,
    } xnz_et;

    union
    {
        struct
        {
            XPLMCommandRef cmd_e_1_onn;
            XPLMCommandRef cmd_e_1_off;
            XPLMCommandRef cmd_e_2_onn;
            XPLMCommandRef cmd_e_2_off;
            XPLMCommandRef cmd_m_12_cr;
            XPLMCommandRef cmd_m_12_no;
            XPLMCommandRef cmd_m_12_st;
        } to32;

        struct
        {
            XPLMCommandRef cmd_e_1_onn;
            XPLMCommandRef cmd_e_1_off;
            XPLMCommandRef cmd_e_2_onn;
            XPLMCommandRef cmd_e_2_off;
            XPLMCommandRef cmd_m_12_cr;
            XPLMCommandRef cmd_m_12_no;
            XPLMCommandRef cmd_m_12_st;
        } ff35;

        struct
        {
            XPLMCommandRef cmd_e_1_onn; // sim/engines/mixture_up
            XPLMCommandRef cmd_e_1_off; // sim/engines/mixture_down
            XPLMCommandRef cmd_x_12_lt; // tbm900/actuators/elec/starter_up
            XPLMCommandRef cmd_x_12_rt; // tbm900/actuators/elec/starter_down
            XPLMCommandRef cmd_m_12_cr; // tbm900/actuators/elec/ignition_off
            XPLMCommandRef cmd_m_12_no; // tbm900/actuators/elec/ignition_auto
            XPLMCommandRef cmd_m_12_st; // tbm900/actuators/elec/ignition_on
            XPLMDataRef    drf_fuelsel; // tbm900/switches/fuel/auto_man
        } tbm9;

        struct
        {
            XPLMDataRef    drf_mod_en1; // aerobask/eclipse/start_eng_0
            XPLMDataRef    drf_mod_en2; // aerobask/eclipse/start_eng_1
        } ea50;

        struct
        {
            XPLMCommandRef cmd_e_1_onn; // aerobask/eng/master1_up
            XPLMCommandRef cmd_e_1_off; // aerobask/eng/master1_dn
            XPLMCommandRef cmd_e_2_onn; // aerobask/eng/master2_up
            XPLMCommandRef cmd_e_2_off; // aerobask/eng/master2_dn
            XPLMCommandRef cmd_ecu1_up; // aerobask/eng/ecu_ab1_up
            XPLMCommandRef cmd_ecu1_dn; // aerobask/eng/ecu_ab1_dn
            XPLMCommandRef cmd_ecu2_up; // aerobask/eng/ecu_ab2_up
            XPLMCommandRef cmd_ecu2_dn; // aerobask/eng/ecu_ab2_dn
            XPLMDataRef    drf_mod_ec1; // aerobask/eng/sw_ecu_ab2
            XPLMDataRef    drf_mod_ec2; // aerobask/eng/sw_ecu_ab2
        } da62;

        struct
        {
            XPLMCommandRef cmd_m_12_no; // sim/igniters/igniter_contin_off_1
            XPLMCommandRef cmd_m_12_st; // sim/igniters/igniter_contin_on_1
            XPLMCommandRef cmd_x_12_lt; // sim/starters/engage_starter_1
            XPLMCommandRef cmd_x_12_rt; // sim/starters/shut_down_1
            XPLMCommandRef cmd_e_1_off; // sim/fuel/fuel_pump_1_off
            XPLMCommandRef cmd_e_1_onn; // sim/fuel/fuel_pump_1_on
            XPLMCommandRef cmd_e_2_tog; // aerobask/fuel_auto_toggle
            XPLMDataRef    drf_fuel_at; // aerobask/lt_fuel_auto
        } evic;

        struct
        {
            XPLMDataRef    drf_e_1_ign; // aerobask/engines/sw_ignition_1
            XPLMDataRef    drf_e_2_ign; // aerobask/engines/sw_ignition_2
            XPLMDataRef    drf_e_1_knb; // aerobask/engines/knob_start_stop_1
            XPLMDataRef    drf_e_2_knb; // aerobask/engines/knob_start_stop_2
            XPLMCommandRef cmd_ig_1_up; // aerobask/engines/ignition_1_up
            XPLMCommandRef cmd_ig_1_dn; // aerobask/engines/ignition_1_dn
            XPLMCommandRef cmd_ig_2_up; // aerobask/engines/ignition_2_up
            XPLMCommandRef cmd_ig_2_dn; // aerobask/engines/ignition_2_dn
            XPLMCommandRef cmd_e_1_lft; // aerobask/engines/knob_1_lt
            XPLMCommandRef cmd_e_1_rgt; // aerobask/engines/knob_1_rt
            XPLMCommandRef cmd_e_2_lft; // aerobask/engines/knob_2_lt
            XPLMCommandRef cmd_e_2_rgt; // aerobask/engines/knob_2_rt
        } e55p;

        struct
        {
            XPLMDataRef    drf_e_1_knb; // XCrafts/ERJ/engine1_starter_knob
            XPLMDataRef    drf_e_2_knb; // XCrafts/ERJ/engine2_starter_knob
            XPLMCommandRef cmd_e_1_lft; // XCrafts/Starter_Eng_1_down_CCW
            XPLMCommandRef cmd_e_1_rgt; // XCrafts/Starter_Eng_1_up_CW
            XPLMCommandRef cmd_e_2_lft; // XCrafts/Starter_Eng_2_down_CCW
            XPLMCommandRef cmd_e_2_rgt; // XCrafts/Starter_Eng_2_up_CW
        } e35l;

        struct
        {
            XPLMDataRef drf_e_1_cut; // ixeg/733/fuel/fuel_start_lever1_act
            XPLMDataRef drf_e_2_cut; // ixeg/733/fuel/fuel_start_lever2_act
            XPLMDataRef drf_e_1_knb; // ixeg/733/engine/eng1_start_act
            XPLMDataRef drf_e_2_knb; // ixeg/733/engine/eng1_start_act
        } ix73;

        struct
        {
            XPLMDataRef drf_e_1_cut; // 1-sim/fuel/fuelCutOffLeft
            XPLMDataRef drf_e_2_cut; // 1-sim/fuel/fuelCutOffRight
            XPLMDataRef drf_e_1_knb; // 1-sim/engine/leftStartSelector
            XPLMDataRef drf_e_2_knb; // 1-sim/engine/rightStartSelector
        } ff75;
    } et;

    enum
    {
        XNZ_FL_ERRR =  -1,
        XNZ_FL_XPLM =   1,
    } xnz_fl;

    enum
    {
        XNZ_PB_ERRR =  -1,
        XNZ_PB_XPLM =   1,
        XNZ_PB_COMM =   2,
        XNZ_PB_FF32 = 320,
        XNZ_PB_TO32 = 321,
        XNZ_PB_FF35 = 350,
        XNZ_PB_TBM9 = 900,
    } xnz_pb;

    union
    {
        struct
        {
        } comm;

        struct
        {
        } ff32;

        struct
        {
            XPLMDataRef pbrak_offon; // 1-sim/parckBrake
        } ff35;

        struct
        {
            XPLMDataRef pbrak_onoff; // AirbusFBW/ParkBrake
        } to32;

        struct
        {
            XPLMDataRef pbrak_ratio; // tbm900/switches/gear/park_brake
        } tbm9;
    } pb;

    enum
    {
        XNZ_SB_ERRR =  -1,
        XNZ_SB_XPLM =   1,
        XNZ_SB_AUTO =   2,
        XNZ_SB_E55P = 535,
    } xnz_sb;

    struct
    {
        XPLMCommandRef at_at_on; // sim/autopilot/autothrottle_on
        XPLMCommandRef at_at_no; // sim/autopilot/autothrottle_off
        XPLMCommandRef at_at_n1; // sim/autopilot/autothrottle_n1epr
        XPLMCommandRef ap_to_ga; // sim/autopilot/take_off_go_around
        XPLMCommandRef ap_cw_st; // sim/autopilot/control_wheel_steer
        XPLMCommandRef p_start1; // sim/starters/engage_starter_1
        XPLMCommandRef p_mboth1; // sim/magnetos/magnetos_both_1
        XPLMCommandRef p_m_lft1; // sim/magnetos/magnetos_left_1
        XPLMCommandRef p_m_rgt1; // sim/magnetos/magnetos_right_1
        XPLMCommandRef p_mstop1; // sim/magnetos/magnetos_off_1
        XPLMCommandRef p_start2; // sim/starters/engage_starter_2
        XPLMCommandRef p_mboth2; // sim/magnetos/magnetos_both_2
        XPLMCommandRef p_m_lft2; // sim/magnetos/magnetos_left_2
        XPLMCommandRef p_m_rgt2; // sim/magnetos/magnetos_right_2
        XPLMCommandRef p_mstop2; // sim/magnetos/magnetos_off_2
        XPLMCommandRef p_start3; // sim/starters/engage_starter_3
        XPLMCommandRef p_mboth3; // sim/magnetos/magnetos_both_3
        XPLMCommandRef p_m_lft3; // sim/magnetos/magnetos_left_3
        XPLMCommandRef p_m_rgt3; // sim/magnetos/magnetos_right_3
        XPLMCommandRef p_mstop3; // sim/magnetos/magnetos_off_3
        XPLMCommandRef p_start4; // sim/starters/engage_starter_4
        XPLMCommandRef p_mboth4; // sim/magnetos/magnetos_both_4
        XPLMCommandRef p_m_lft4; // sim/magnetos/magnetos_left_4
        XPLMCommandRef p_m_rgt4; // sim/magnetos/magnetos_right_4
        XPLMCommandRef p_mstop4; // sim/magnetos/magnetos_off_4
        XPLMDataRef auto_pil_on; // sim/cockpit2/autopilot/servos_on
        XPLMDataRef auto_vvi_on; // sim/cockpit2/autopilot/vvi_status
        XPLMDataRef auto_thr_on; // sim/cockpit2/autopilot/autothrottle_on
        XPLMDataRef eng_running; // sim/flightmodel/engine/ENGN_running
        XPLMDataRef igniters_on; // sim/cockpit2/engine/actuators/igniter_on
        XPLMDataRef auto_ignite; // sim/cockpit2/engine/actuators/auto_ignite_on
        XPLMDataRef groundspeed; // sim/flightmodel/position/groundspeed
        XPLMDataRef ongroundany; // sim/flightmodel/failures/onground_any
        XPLMDataRef l_rgb_ratio; // sim/cockpit2/controls/left_brake_ratio
        XPLMDataRef r_rgb_ratio; // sim/cockpit2/controls/right_brake_ratio
        XPLMDataRef pbrak_ratio; // sim/cockpit2/controls/parking_brake_ratio
        int         pbrak_onoff; // for manual braking: remember the pbrak state
        int         pbrakonoff2; // for xnz/brakes/regular/park
    } xp;

    XPLMCommandRef cmd_rgb_pkb; // xnz/brakes/regular/park
    XPLMCommandRef cmd_rgb_hld; // xnz/brakes/regular/hold
    XPLMCommandRef cmd_pkb_tog; // xnz/brakes/park/toggle
    XPLMCommandRef cmd_pkb_onh; // xnz/brakes/park/on/hold
    XPLMCommandRef cmd_pkb_onn; // xnz/brakes/park/on/set
    XPLMCommandRef cmd_pkb_off; // xnz/brakes/park/unset
    XPLMCommandRef cmd_at_toga; // xnz/auto/thrust/to/ga
    XPLMCommandRef cmd_a_12_lt; // xnz/tca/12/at/disc/lt
    XPLMCommandRef cmd_a_12_rt; // xnz/tca/12/at/disc/rt
    XPLMCommandRef cmd_a_34_lt; // xnz/tca/34/at/disc/lt
    XPLMCommandRef cmd_a_34_rt; // xnz/tca/34/at/disc/rt
    XPLMCommandRef cmd_x_12_lt; // xnz/tca/12/extra/lt
    XPLMCommandRef cmd_x_12_rt; // xnz/tca/12/extra/rt
    XPLMCommandRef cmd_x_34_lt; // xnz/tca/34/extra/lt
    XPLMCommandRef cmd_x_34_rt; // xnz/tca/34/extra/rt
    XPLMCommandRef cmd_m_12_ch; // xnz/tca/12/modes/crank/hold
    XPLMCommandRef cmd_m_12_cr; // xnz/tca/12/modes/crank
    XPLMCommandRef cmd_m_12_no; // xnz/tca/12/modes/norm
    XPLMCommandRef cmd_m_12_st; // xnz/tca/12/modes/start
    XPLMCommandRef cmd_m_12_sh; // xnz/tca/12/modes/start/hold
    XPLMCommandRef cmd_m_34_ch; // xnz/tca/34/modes/crank/hold
    XPLMCommandRef cmd_m_34_cr; // xnz/tca/34/modes/crank
    XPLMCommandRef cmd_m_34_no; // xnz/tca/34/modes/norm
    XPLMCommandRef cmd_m_34_st; // xnz/tca/34/modes/start
    XPLMCommandRef cmd_m_34_sh; // xnz/tca/34/modes/start/hold
    XPLMCommandRef cmd_e_1_onh; // xnz/tca/engines/1/on/hold
    XPLMCommandRef cmd_e_1_onn; // xnz/tca/engines/1/on
    XPLMCommandRef cmd_e_1_off; // xnz/tca/engines/1/off
    XPLMCommandRef cmd_e_2_onh; // xnz/tca/engines/2/on/hold
    XPLMCommandRef cmd_e_2_onn; // xnz/tca/engines/2/on
    XPLMCommandRef cmd_e_2_off; // xnz/tca/engines/2/off
    XPLMCommandRef cmd_e_3_onh; // xnz/tca/engines/3/on/hold
    XPLMCommandRef cmd_e_3_onn; // xnz/tca/engines/3/on
    XPLMCommandRef cmd_e_3_off; // xnz/tca/engines/3/off
    XPLMCommandRef cmd_e_4_onh; // xnz/tca/engines/4/on/hold
    XPLMCommandRef cmd_e_4_onn; // xnz/tca/engines/4/on
    XPLMCommandRef cmd_e_4_off; // xnz/tca/engines/4/off
} xnz_cmd_context;

static int chandler_rgb_pkb(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_rgb_hld(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_pkb_tog(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_pkb_onh(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_pkb_onn(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_pkb_off(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_at_toga(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_a_12_lt(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_a_12_rt(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_a_34_lt(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_a_34_rt(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_x_12_lt(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_x_12_rt(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_x_34_lt(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_x_34_rt(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_m_12_ch(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_m_12_cr(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_m_12_no(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_m_12_st(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_m_12_sh(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_m_34_ch(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_m_34_cr(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_m_34_no(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_m_34_st(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_m_34_sh(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_e_1_onh(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_e_1_onn(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_e_1_off(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_e_2_onh(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_e_2_onn(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_e_2_off(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_e_3_onh(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_e_3_onn(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_e_3_off(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_e_4_onh(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_e_4_onn(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_e_4_off(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);
static int chandler_printax(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon);

typedef struct
{
#ifndef PUBLIC_RELEASE_BUILD
    XPLMFlightLoop_f f_l_cb;
    XPLMDataRef f_air_speed;
    XPLMDataRef f_grd_speed;
    XPLMDataRef nullzone[3];
    float prefs_nullzone[3];
    float minimum_null_zone;
    XPLMDataRef acf_roll_co;
    XPLMDataRef ongroundany;
    float nominal_roll_coef;
    float last_throttle_all;
    float show_throttle_all;
    float icecheck_required;
    char overly_txt_buf[11];
    int throttle_did_change;
    int ice_detect_positive;
    int overly_position_set;
    XPLMDataRef f_ice_rf[4];
    XPWidgetID  widgetid[2];
    XPLMCommandRef print_ax;
#endif

    xnz_cmd_context commands;

    enum
    {
        XNZ_TT_ERRR =  -1,
        XNZ_TT_XPLM =   1,
        XNZ_TT_FF32 = 320,
        XNZ_TT_TOLI = 321,
        XNZ_TT_TBM9 = 900,
    } xnz_tt;

    union
    {
        struct
        {
            SharedValuesInterface s;
            int id_f32_eng_lever_lt;
            int id_f32_eng_lever_rt;
            int api_has_initialized;
        } ff32;

        struct
        {
            XPLMDataRef f_thr_array;
        } toli;

        struct
        {
            XPLMDataRef engn_rng;
        } tbm9;
    } tt;

    int i_got_axis_input[3];
    int i_context_init_done;
    int i_version_simulator;
    int i_version_xplm_apis;
    XPLMFlightLoop_f f_l_th;
    XPLMDataRef i_stick_ass;
    XPLMDataRef f_stick_val;
    XPLMDataRef i_prop_mode;
    XPLMDataRef i_ngine_num;
    XPLMDataRef i_ngine_typ;
    XPLMDataRef rev_info[3];
    XPLMCommandRef betto[9];
    XPLMCommandRef revto[9];
    int acft_has_rev_thrust;
    int acf_has_beta_thrust;
    int arcrft_engine_count;
    int i_propmode_value[8];
    int idx_throttle_axis_1;
    XPLMDataRef f_throttall;
    XPLMDataRef f_thr_array;

#define XNZ_THINN_NO (-1.0f)
#define XNZ_THOUT_AT (-2.0f)
#define XNZ_THOUT_SK (-3.0f)
    float avrg_throttle_inn;
    XPLMDataRef f_throt_inn;
    float avrg_throttle_out;
    XPLMDataRef f_throt_out;

    XPLMMenuID id_th_on_off;
    int id_menu_item_on_off;
    int tca_support_enabled;
    int msg_will_write_pref;
    int skip_idle_overwrite;
    thrust_zones zones_info;
}
xnz_context;

static xnz_context *global_context = NULL;

static int               xnz_log(const char *format, ...);
static float      axes_hdlr_fnc(float, float, int, void*);
static void       menu_hdlr_fnc(void*,             void*);
static inline float throttle_mapping(float, thrust_zones);

#ifndef PUBLIC_RELEASE_BUILD
static float callback_hdlr(float, float, int, void*);
#endif

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
#ifndef PUBLIC_RELEASE_BUILD
#define XNZ_XPLM_TITLE "X-Nullzones"
#define XNZ_LOG_PREFIX "x-nullzones: "
    strncpy(outName,                                    XNZ_XPLM_TITLE, 255);
    strncpy(outSig,                                     "Rodeo314.XNZ", 255);
    strncpy(outDesc, "Dynamic nullzones and other miscellaneous stuff", 255);
#else
#define XNZ_XPLM_TITLE "Quadrant314"
#define XNZ_LOG_PREFIX "Quadrant314: "
    strncpy(outName,                                    XNZ_XPLM_TITLE, 255);
    strncpy(outSig,                                     "Rodeo314.TCA", 255);
    strncpy(outDesc,   "\"Reverse on same axis\" thrust lever support", 255);
#endif

    /* all good */
    XPLMDebugString(XNZ_LOG_PREFIX"[info]: XPluginStart OK\n"); return 1;
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
        XPLMDebugString(XNZ_LOG_PREFIX);
        XPLMDebugString(string);
    }
    va_end(ap);
    return ret;
}

PLUGIN_API void XPluginStop(void)
{
    return;
}

static float XNZGetDataf(void *inRefcon) // XPLMGetDataf_f
{
    return ((float*)inRefcon)[0]; // https://developer.x-plane.com/sdk/XPLMRegisterDataAccessor/
}

#define HS_TBM9_IDLE (0.35f)

static float TCA_SYNCBAND = 0.075000f; // note: maximum L/R difference was measured slightly over 6%, but we allow for noisier hardware than mine
static float TCA_DEADBAND = 0.037500f; // half of the above
static float TCA_FLEX_CTR = 0.706744f; // print_ax
static float TCA_CLMB_CTR = 0.520279f; // print_ax
//atic float TCA_IDLE_CTR = 0.323819f; // not held
static float TCA_IDLE_CTR = 0.311589f; // averaged
//atic float TCA_IDLE_CTR = 0.299359f; // yes held

static void update_thrust_zones(thrust_zones *info)
{
    if (info)
    {
        /*
         * Re-compute zones based on the center of each hardware detent.
         */
        info->max[ZONE_TGA] = (                      1.0f - TCA_DEADBAND);
        info->min[ZONE_TGA] = (              TCA_FLEX_CTR + TCA_DEADBAND);
        info->max[ZONE_FLX] = (              TCA_FLEX_CTR - TCA_DEADBAND);
        info->min[ZONE_FLX] = (              TCA_CLMB_CTR + TCA_DEADBAND);
        info->max[ZONE_CLB] = (              TCA_CLMB_CTR - TCA_DEADBAND);
        info->min[ZONE_CLB] = (              TCA_IDLE_CTR + TCA_DEADBAND);
        info->max[ZONE_REV] = (              TCA_IDLE_CTR - TCA_DEADBAND);
        info->min[ZONE_REV] = (                      0.0f + TCA_DEADBAND);
        info->len[ZONE_TGA] = (info->max[ZONE_TGA] - info->min[ZONE_TGA]);
        info->len[ZONE_FLX] = (info->max[ZONE_FLX] - info->min[ZONE_FLX]);
        info->len[ZONE_CLB] = (info->max[ZONE_CLB] - info->min[ZONE_CLB]);
        info->len[ZONE_REV] = (info->max[ZONE_REV] - info->min[ZONE_REV]);
    }
}

static void default_throt_share(thrust_zones *info)
{
    if (info)
    {
        info->share[ZONE_CLB] = 0.500f;
        info->share[ZONE_FLX] = 0.875f - info->share[ZONE_CLB];
        info->share[ZONE_TGA] = 1.000f - info->share[ZONE_FLX] - info->share[ZONE_CLB];
    }
}

PLUGIN_API int XPluginEnable(void)
{
    /* check for unsupported versions */
    XPLMHostApplicationID outHostID;
    int outXPlaneVersion, outXPLMVersion;
    XPLMGetVersions(&outXPlaneVersion, &outXPLMVersion, &outHostID);
    if (outXPLMVersion < 210) // currently pointless as we target XPLM210 at build time
    {
        xnz_log("[error]: XPluginEnable failed (outXPLMVersion: %d < 210)\n", outXPLMVersion);
        return 0;
    }
    if (outXPlaneVersion > 11999)
    {
        xnz_log("[error]: XPluginEnable failed (outXPlaneVersion: %d > 11999)\n", outXPlaneVersion);
        return 0;
    }

    /* Initialize context */
    if (NULL == (global_context = malloc(sizeof(xnz_context))))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (malloc)\n"); goto fail;
    }
#ifndef PUBLIC_RELEASE_BUILD
    if (NULL == (global_context->print_ax = XPLMCreateCommand("xnz/print/axes/average", "")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (print_ax)\n"); goto fail;
    }
    else
    {
        XPLMRegisterCommandHandler(global_context->print_ax, &chandler_printax, 0, global_context);
    }
    if (NULL == (global_context->nullzone[0] = XPLMFindDataRef("sim/joystick/joystick_pitch_nullzone")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (nullzone[0])\n"); goto fail;
    }
    if (NULL == (global_context->nullzone[1] = XPLMFindDataRef("sim/joystick/joystick_roll_nullzone")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (nullzone[1])\n"); goto fail;
    }
    if (NULL == (global_context->nullzone[2] = XPLMFindDataRef("sim/joystick/joystick_heading_nullzone")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (nullzone[2])\n"); goto fail;
    }
    if (NULL == (global_context->f_grd_speed = XPLMFindDataRef("sim/flightmodel/position/groundspeed")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (f_grd_speed)\n"); goto fail;
    }
    if (NULL == (global_context->f_air_speed = XPLMFindDataRef("sim/flightmodel/position/indicated_airspeed")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (f_air_speed)\n"); goto fail;
    }
    if (NULL == (global_context->acf_roll_co = XPLMFindDataRef("sim/aircraft/overflow/acf_roll_co")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (acf_roll_co)\n"); goto fail;
    }
    if (NULL == (global_context->ongroundany = XPLMFindDataRef("sim/flightmodel/failures/onground_any")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (ongroundany)\n"); goto fail;
    }
    if (NULL == (global_context->f_ice_rf[0] = XPLMFindDataRef("sim/flightmodel/failures/pitot_ice")) ||
        NULL == (global_context->f_ice_rf[1] = XPLMFindDataRef("sim/flightmodel/failures/inlet_ice")) ||
        NULL == (global_context->f_ice_rf[2] = XPLMFindDataRef("sim/flightmodel/failures/prop_ice")) ||
        NULL == (global_context->f_ice_rf[3] = XPLMFindDataRef("sim/flightmodel/failures/frm_ice")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (f_ice_rf)\n"); goto fail;
    }
    if (!(global_context->widgetid[0] = XPCreateWidget(0, 0, 0, 0, 0, "", 1, NULL,
                                                       xpWidgetClass_MainWindow)))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (widgetid)\n"); goto fail;
    }
    if (!(global_context->widgetid[1] = XPCreateWidget(0, 0, 0, 0, 0, "", 0,
                                                       global_context->widgetid[0],
                                                       xpWidgetClass_Caption)))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (widgetid)\n"); goto fail;
    }
    global_context->overly_position_set = 0; // X-Plane window's actual boundaries not yet available/reliable
    XPSetWidgetProperty(global_context->widgetid[0], xpProperty_MainWindowType, xpMainWindowStyle_Translucent);
    XPSetWidgetProperty(global_context->widgetid[1], xpProperty_CaptionLit, 1);
    XPSetWidgetGeometry(global_context->widgetid[0], 0, 56 - 0, 64 - 0, 0);
    XPSetWidgetGeometry(global_context->widgetid[1], 7, 56 - 7, 64 - 7, 7);

    /* flight loop callback */
    XPLMRegisterFlightLoopCallback((global_context->f_l_cb = &callback_hdlr), 0, global_context);
#endif // PUBLIC_RELEASE_BUILD

    /* common datarefs */
    if (NULL == (global_context->f_throttall = XPLMFindDataRef("sim/cockpit2/engine/actuators/throttle_ratio_all")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (f_throt_all)\n"); goto fail;
    }

    /* TCA thrust quadrant support */
    if (NULL == (global_context->i_stick_ass = XPLMFindDataRef("sim/joystick/joystick_axis_assignments")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (i_stick_ass)\n"); goto fail;
    }
    if (NULL == (global_context->f_stick_val = XPLMFindDataRef("sim/joystick/joystick_axis_values")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (f_stick_val)\n"); goto fail;
    }
    if (NULL == (global_context->f_thr_array = XPLMFindDataRef("sim/cockpit2/engine/actuators/throttle_ratio")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (f_thr_array)\n"); goto fail;
    }
    if (NULL == (global_context->i_prop_mode = XPLMFindDataRef("sim/cockpit2/engine/actuators/prop_mode")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (i_prop_mode)\n"); goto fail;
    }
    if (NULL == (global_context->i_ngine_num = XPLMFindDataRef("sim/aircraft/engine/acf_num_engines")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (i_ngine_num)\n"); goto fail;
    }
    if (NULL == (global_context->i_ngine_typ = XPLMFindDataRef("sim/aircraft/prop/acf_en_type")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (i_ngine_typ)\n"); goto fail;
    }
    if (NULL == (global_context->rev_info[0] = XPLMFindDataRef("sim/aircraft/overflow/acf_has_beta")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (rev_info[0])\n"); goto fail;
    }
    if (NULL == (global_context->rev_info[1] = XPLMFindDataRef("sim/aircraft/prop/acf_revthrust_eq")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (rev_info[1])\n"); goto fail;
    }
    if (NULL == (global_context->rev_info[2] = XPLMFindDataRef("sim/aircraft/engine/acf_throtmax_REV")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (rev_info[2])\n"); goto fail;
    }
    if (NULL == (global_context->betto[0] = XPLMFindCommand("sim/engines/beta_toggle_1")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (betto[0])\n"); goto fail;
    }
    if (NULL == (global_context->betto[1] = XPLMFindCommand("sim/engines/beta_toggle_2")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (betto[1])\n"); goto fail;
    }
    if (NULL == (global_context->betto[2] = XPLMFindCommand("sim/engines/beta_toggle_3")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (betto[2])\n"); goto fail;
    }
    if (NULL == (global_context->betto[3] = XPLMFindCommand("sim/engines/beta_toggle_4")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (betto[3])\n"); goto fail;
    }
    if (NULL == (global_context->betto[4] = XPLMFindCommand("sim/engines/beta_toggle_5")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (betto[4])\n"); goto fail;
    }
    if (NULL == (global_context->betto[5] = XPLMFindCommand("sim/engines/beta_toggle_6")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (betto[5])\n"); goto fail;
    }
    if (NULL == (global_context->betto[6] = XPLMFindCommand("sim/engines/beta_toggle_7")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (betto[6])\n"); goto fail;
    }
    if (NULL == (global_context->betto[7] = XPLMFindCommand("sim/engines/beta_toggle_8")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (betto[7])\n"); goto fail;
    }
    if (NULL == (global_context->betto[8] = XPLMFindCommand("sim/engines/beta_toggle")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (betto[9])\n"); goto fail;
    }
    if (NULL == (global_context->revto[0] = XPLMFindCommand("sim/engines/thrust_reverse_toggle_1")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (revto[0])\n"); goto fail;
    }
    if (NULL == (global_context->revto[1] = XPLMFindCommand("sim/engines/thrust_reverse_toggle_2")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (revto[1])\n"); goto fail;
    }
    if (NULL == (global_context->revto[2] = XPLMFindCommand("sim/engines/thrust_reverse_toggle_3")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (revto[2])\n"); goto fail;
    }
    if (NULL == (global_context->revto[3] = XPLMFindCommand("sim/engines/thrust_reverse_toggle_4")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (revto[3])\n"); goto fail;
    }
    if (NULL == (global_context->revto[4] = XPLMFindCommand("sim/engines/thrust_reverse_toggle_5")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (revto[4])\n"); goto fail;
    }
    if (NULL == (global_context->revto[5] = XPLMFindCommand("sim/engines/thrust_reverse_toggle_6")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (revto[5])\n"); goto fail;
    }
    if (NULL == (global_context->revto[6] = XPLMFindCommand("sim/engines/thrust_reverse_toggle_7")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (revto[6])\n"); goto fail;
    }
    if (NULL == (global_context->revto[7] = XPLMFindCommand("sim/engines/thrust_reverse_toggle_8")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (revto[7])\n"); goto fail;
    }
    if (NULL == (global_context->revto[8] = XPLMFindCommand("sim/engines/thrust_reverse_toggle")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (revto[9])\n"); goto fail;
    }
    XPLMRegisterFlightLoopCallback((global_context->f_l_th = &axes_hdlr_fnc), 0, global_context);

#ifndef PUBLIC_RELEASE_BUILD
    /* TCA thrust quadrant support: buttons */
    if (NULL == (global_context->commands.xp.at_at_on = XPLMFindCommand("sim/autopilot/autothrottle_on")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMFindCommand failed (sim/autopilot/autothrottle_on)\n"); goto fail;
    }
    if (NULL == (global_context->commands.xp.at_at_no = XPLMFindCommand("sim/autopilot/autothrottle_off")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMFindCommand failed (sim/autopilot/autothrottle_off)\n"); goto fail;
    }
    if (NULL == (global_context->commands.xp.at_at_n1 = XPLMFindCommand("sim/autopilot/autothrottle_n1epr")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[info]: sim/autopilot/autothrottle_n1epr unavailable\n"); // new command (optional)
    }
    if (NULL == (global_context->commands.xp.ap_to_ga = XPLMFindCommand("sim/autopilot/take_off_go_around")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMFindCommand failed (sim/autopilot/take_off_go_around)\n"); goto fail;
    }
    if (NULL == (global_context->commands.xp.ap_cw_st = XPLMFindCommand("sim/autopilot/control_wheel_steer")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMFindCommand failed (sim/autopilot/control_wheel_steer)\n"); goto fail;
    }
    if (NULL == (global_context->commands.xp.p_start1 = XPLMFindCommand("sim/starters/engage_starter_1")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMFindCommand failed (sim/starters/engage_starter_1)\n"); goto fail;
    }
    if (NULL == (global_context->commands.xp.p_start2 = XPLMFindCommand("sim/starters/engage_starter_2")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMFindCommand failed (sim/starters/engage_starter_2)\n"); goto fail;
    }
    if (NULL == (global_context->commands.xp.p_start3 = XPLMFindCommand("sim/starters/engage_starter_3")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMFindCommand failed (sim/starters/engage_starter_3)\n"); goto fail;
    }
    if (NULL == (global_context->commands.xp.p_start4 = XPLMFindCommand("sim/starters/engage_starter_4")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMFindCommand failed (sim/starters/engage_starter_4)\n"); goto fail;
    }
    if (NULL == (global_context->commands.xp.p_mboth1 = XPLMFindCommand("sim/magnetos/magnetos_both_1")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMFindCommand failed (sim/magnetos/magnetos_both_1)\n"); goto fail;
    }
    if (NULL == (global_context->commands.xp.p_mboth2 = XPLMFindCommand("sim/magnetos/magnetos_both_2")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMFindCommand failed (sim/magnetos/magnetos_both_2)\n"); goto fail;
    }
    if (NULL == (global_context->commands.xp.p_mboth3 = XPLMFindCommand("sim/magnetos/magnetos_both_3")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMFindCommand failed (sim/magnetos/magnetos_both_3)\n"); goto fail;
    }
    if (NULL == (global_context->commands.xp.p_mboth4 = XPLMFindCommand("sim/magnetos/magnetos_both_4")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMFindCommand failed (sim/magnetos/magnetos_both_4)\n"); goto fail;
    }
    if (NULL == (global_context->commands.xp.p_m_lft1 = XPLMFindCommand("sim/magnetos/magnetos_left_1")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMFindCommand failed (sim/magnetos/magnetos_left_1)\n"); goto fail;
    }
    if (NULL == (global_context->commands.xp.p_m_lft2 = XPLMFindCommand("sim/magnetos/magnetos_left_2")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMFindCommand failed (sim/magnetos/magnetos_left_2)\n"); goto fail;
    }
    if (NULL == (global_context->commands.xp.p_m_lft3 = XPLMFindCommand("sim/magnetos/magnetos_left_3")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMFindCommand failed (sim/magnetos/magnetos_left_3)\n"); goto fail;
    }
    if (NULL == (global_context->commands.xp.p_m_lft4 = XPLMFindCommand("sim/magnetos/magnetos_left_4")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMFindCommand failed (sim/magnetos/magnetos_left_4)\n"); goto fail;
    }
    if (NULL == (global_context->commands.xp.p_m_rgt1 = XPLMFindCommand("sim/magnetos/magnetos_right_1")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMFindCommand failed (sim/magnetos/magnetos_right_1)\n"); goto fail;
    }
    if (NULL == (global_context->commands.xp.p_m_rgt2 = XPLMFindCommand("sim/magnetos/magnetos_right_2")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMFindCommand failed (sim/magnetos/magnetos_right_2)\n"); goto fail;
    }
    if (NULL == (global_context->commands.xp.p_m_rgt3 = XPLMFindCommand("sim/magnetos/magnetos_right_3")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMFindCommand failed (sim/magnetos/magnetos_right_3)\n"); goto fail;
    }
    if (NULL == (global_context->commands.xp.p_m_rgt4 = XPLMFindCommand("sim/magnetos/magnetos_right_4")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMFindCommand failed (sim/magnetos/magnetos_right_4)\n"); goto fail;
    }
    if (NULL == (global_context->commands.xp.p_mstop1 = XPLMFindCommand("sim/magnetos/magnetos_off_1")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMFindCommand failed (sim/magnetos/magnetos_off_1)\n"); goto fail;
    }
    if (NULL == (global_context->commands.xp.p_mstop2 = XPLMFindCommand("sim/magnetos/magnetos_off_2")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMFindCommand failed (sim/magnetos/magnetos_off_2)\n"); goto fail;
    }
    if (NULL == (global_context->commands.xp.p_mstop3 = XPLMFindCommand("sim/magnetos/magnetos_off_3")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMFindCommand failed (sim/magnetos/magnetos_off_3)\n"); goto fail;
    }
    if (NULL == (global_context->commands.xp.p_mstop4 = XPLMFindCommand("sim/magnetos/magnetos_off_4")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMFindCommand failed (sim/magnetos/magnetos_off_4)\n"); goto fail;
    }
    if (NULL == (global_context->commands.xp.auto_pil_on = XPLMFindDataRef("sim/cockpit2/autopilot/servos_on")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMFindDataRef failed (sim/cockpit2/autopilot/servos_on)\n"); goto fail;
    }
    if (NULL == (global_context->commands.xp.auto_vvi_on = XPLMFindDataRef("sim/cockpit2/autopilot/vvi_status")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMFindDataRef failed (sim/cockpit2/autopilot/vvi_status)\n"); goto fail;
    }
    if (NULL == (global_context->commands.xp.auto_thr_on = XPLMFindDataRef("sim/cockpit2/autopilot/autothrottle_on")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMFindDataRef failed (sim/cockpit2/autopilot/autothrottle_on)\n"); goto fail;
    }
    if (NULL == (global_context->commands.xp.eng_running = XPLMFindDataRef("sim/flightmodel/engine/ENGN_running")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMFindDataRef failed (sim/flightmodel/engine/ENGN_running)\n"); goto fail;
    }
    if (NULL == (global_context->commands.xp.igniters_on = XPLMFindDataRef("sim/cockpit2/engine/actuators/igniter_on")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMFindDataRef failed (sim/cockpit2/engine/actuators/igniter_on)\n"); goto fail;
    }
    if (NULL == (global_context->commands.xp.auto_ignite = XPLMFindDataRef("sim/cockpit2/engine/actuators/auto_ignite_on")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMFindDataRef failed (sim/cockpit2/engine/actuators/auto_ignite_on)\n"); goto fail;
    }
    if (NULL == (global_context->commands.xp.groundspeed = XPLMFindDataRef("sim/flightmodel/position/groundspeed")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMFindDataRef failed (sim/flightmodel/position/groundspeed)\n"); goto fail;
    }
    if (NULL == (global_context->commands.xp.ongroundany = XPLMFindDataRef("sim/flightmodel/failures/onground_any")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMFindDataRef failed (sim/flightmodel/failures/onground_any)\n"); goto fail;
    }
    if (NULL == (global_context->commands.xp.l_rgb_ratio = XPLMFindDataRef("sim/cockpit2/controls/left_brake_ratio")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMFindDataRef failed (sim/cockpit2/controls/left_brake_ratio)\n"); goto fail;
    }
    if (NULL == (global_context->commands.xp.r_rgb_ratio = XPLMFindDataRef("sim/cockpit2/controls/right_brake_ratio")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMFindDataRef failed (sim/cockpit2/controls/right_brake_ratio)\n"); goto fail;
    }
    if (NULL == (global_context->commands.xp.pbrak_ratio = XPLMFindDataRef("sim/cockpit2/controls/parking_brake_ratio")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMFindDataRef failed (sim/cockpit2/controls/parking_brake_ratio)\n"); goto fail;
    }
    if (NULL == (global_context->commands.cmd_rgb_pkb = XPLMCreateCommand("xnz/brakes/regular/park", "hold brakes regular + set parking brake")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMCreateCommand failed (xnz/brakes/regular/park)\n"); goto fail;
    }
    else
    {
        XPLMRegisterCommandHandler(global_context->commands.cmd_rgb_pkb, &chandler_rgb_pkb, 0, &global_context->commands);
    }
    if (NULL == (global_context->commands.cmd_rgb_hld = XPLMCreateCommand("xnz/brakes/regular/hold", "hold brakes regular")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMCreateCommand failed (xnz/brakes/regular/hold)\n"); goto fail;
    }
    else
    {
        XPLMRegisterCommandHandler(global_context->commands.cmd_rgb_hld, &chandler_rgb_hld, 0, &global_context->commands);
    }
    if (NULL == (global_context->commands.cmd_pkb_tog = XPLMCreateCommand("xnz/brakes/park/toggle", "parking brake toggle")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMCreateCommand failed (xnz/brakes/park/toggle)\n"); goto fail;
    }
    else
    {
        XPLMRegisterCommandHandler(global_context->commands.cmd_pkb_tog, &chandler_pkb_tog, 0, &global_context->commands);
    }
    if (NULL == (global_context->commands.cmd_pkb_onh = XPLMCreateCommand("xnz/brakes/park/on/hold", "parking brake hold")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMCreateCommand failed (xnz/brakes/park/on/hold)\n"); goto fail;
    }
    else
    {
        XPLMRegisterCommandHandler(global_context->commands.cmd_pkb_onh, &chandler_pkb_onh, 0, &global_context->commands);
    }
    if (NULL == (global_context->commands.cmd_pkb_onn = XPLMCreateCommand("xnz/brakes/park/on/set", "parking brake set")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMCreateCommand failed (xnz/brakes/park/on/set)\n"); goto fail;
    }
    else
    {
        XPLMRegisterCommandHandler(global_context->commands.cmd_pkb_onn, &chandler_pkb_onn, 0, &global_context->commands);
    }
    if (NULL == (global_context->commands.cmd_pkb_off = XPLMCreateCommand("xnz/brakes/park/unset", "parking brake release")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMCreateCommand failed (xnz/brakes/park/unset)\n"); goto fail;
    }
    else
    {
        XPLMRegisterCommandHandler(global_context->commands.cmd_pkb_off, &chandler_pkb_off, 0, &global_context->commands);
    }
    if (NULL == (global_context->commands.cmd_at_toga = XPLMCreateCommand("xnz/auto/thrust/to/ga", "A/T takeoff/go-around")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMCreateCommand failed (xnz/auto/thrust/to/ga)\n"); goto fail;
    }
    else
    {
        XPLMRegisterCommandHandler(global_context->commands.cmd_at_toga, &chandler_at_toga, 0, &global_context->commands);
    }
    if (NULL == (global_context->commands.cmd_a_12_lt = XPLMCreateCommand("xnz/tca/12/at/disc/lt", "TCA 1&2: lt A/T disconnect")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMCreateCommand failed (xnz/tca/12/at/disc/lt)\n"); goto fail;
    }
    else
    {
        XPLMRegisterCommandHandler(global_context->commands.cmd_a_12_lt, &chandler_a_12_lt, 0, &global_context->commands);
    }
    if (NULL == (global_context->commands.cmd_a_12_rt = XPLMCreateCommand("xnz/tca/12/at/disc/rt", "TCA 1&2: rt A/T disconnect")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMCreateCommand failed (xnz/tca/12/at/disc/rt)\n"); goto fail;
    }
    else
    {
        XPLMRegisterCommandHandler(global_context->commands.cmd_a_12_rt, &chandler_a_12_rt, 0, &global_context->commands);
    }
    if (NULL == (global_context->commands.cmd_x_12_lt = XPLMCreateCommand("xnz/tca/12/extra/lt", "TCA 1&2: lt extra button")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMCreateCommand failed (xnz/tca/12/extra/lt)\n"); goto fail;
    }
    else
    {
        XPLMRegisterCommandHandler(global_context->commands.cmd_x_12_lt, &chandler_x_12_lt, 0, &global_context->commands);
    }
    if (NULL == (global_context->commands.cmd_x_12_rt = XPLMCreateCommand("xnz/tca/12/extra/rt", "TCA 1&2: rt extra button")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMCreateCommand failed (xnz/tca/12/extra/rt)\n"); goto fail;
    }
    else
    {
        XPLMRegisterCommandHandler(global_context->commands.cmd_x_12_rt, &chandler_x_12_rt, 0, &global_context->commands);
    }
    if (NULL == (global_context->commands.cmd_a_34_lt = XPLMCreateCommand("xnz/tca/34/at/disc/lt", "TCA 3&4: lt A/T disconnect")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMCreateCommand failed (xnz/tca/34/at/disc/lt)\n"); goto fail;
    }
    else
    {
        XPLMRegisterCommandHandler(global_context->commands.cmd_a_34_lt, &chandler_a_34_lt, 0, &global_context->commands);
    }
    if (NULL == (global_context->commands.cmd_a_34_rt = XPLMCreateCommand("xnz/tca/34/at/disc/rt", "TCA 3&4: rt A/T disconnect")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMCreateCommand failed (xnz/tca/34/at/disc/rt)\n"); goto fail;
    }
    else
    {
        XPLMRegisterCommandHandler(global_context->commands.cmd_a_34_rt, &chandler_a_34_rt, 0, &global_context->commands);
    }
    if (NULL == (global_context->commands.cmd_x_34_lt = XPLMCreateCommand("xnz/tca/34/extra/lt", "TCA 3&4: lt extra button")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMCreateCommand failed (xnz/tca/34/extra/lt)\n"); goto fail;
    }
    else
    {
        XPLMRegisterCommandHandler(global_context->commands.cmd_x_34_lt, &chandler_x_34_lt, 0, &global_context->commands);
    }
    if (NULL == (global_context->commands.cmd_x_34_rt = XPLMCreateCommand("xnz/tca/34/extra/rt", "TCA 3&4: rt extra button")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMCreateCommand failed (xnz/tca/34/extra/rt)\n"); goto fail;
    }
    else
    {
        XPLMRegisterCommandHandler(global_context->commands.cmd_x_34_rt, &chandler_x_34_rt, 0, &global_context->commands);
    }
    if (NULL == (global_context->commands.cmd_m_12_ch = XPLMCreateCommand("xnz/tca/12/modes/crank/hold", "TCA 1&2: crank mode hold")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMCreateCommand failed (xnz/tca/12/modes/crank/hold)\n"); goto fail;
    }
    else
    {
        XPLMRegisterCommandHandler(global_context->commands.cmd_m_12_ch, &chandler_m_12_ch, 0, &global_context->commands);
    }
    if (NULL == (global_context->commands.cmd_m_12_cr = XPLMCreateCommand("xnz/tca/12/modes/crank", "TCA 1&2: crank mode set")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMCreateCommand failed (xnz/tca/12/modes/crank)\n"); goto fail;
    }
    else
    {
        XPLMRegisterCommandHandler(global_context->commands.cmd_m_12_cr, &chandler_m_12_cr, 0, &global_context->commands);
    }
    if (NULL == (global_context->commands.cmd_m_12_no = XPLMCreateCommand("xnz/tca/12/modes/norm", "TCA 1&2: norm mode set")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMCreateCommand failed (xnz/tca/12/modes/norm)\n"); goto fail;
    }
    else
    {
        XPLMRegisterCommandHandler(global_context->commands.cmd_m_12_no, &chandler_m_12_no, 0, &global_context->commands);
    }
    if (NULL == (global_context->commands.cmd_m_12_st = XPLMCreateCommand("xnz/tca/12/modes/start", "TCA 1&2: start mode set")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMCreateCommand failed (xnz/tca/12/modes/start)\n"); goto fail;
    }
    else
    {
        XPLMRegisterCommandHandler(global_context->commands.cmd_m_12_st, &chandler_m_12_st, 0, &global_context->commands);
    }
    if (NULL == (global_context->commands.cmd_m_12_sh = XPLMCreateCommand("xnz/tca/12/modes/start/hold", "TCA 1&2: start mode hold")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMCreateCommand failed (xnz/tca/12/modes/start/hold)\n"); goto fail;
    }
    else
    {
        XPLMRegisterCommandHandler(global_context->commands.cmd_m_12_sh, &chandler_m_12_sh, 0, &global_context->commands);
    }
    if (NULL == (global_context->commands.cmd_m_34_ch = XPLMCreateCommand("xnz/tca/34/modes/crank/hold", "TCA 3&4: crank mode hold")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMCreateCommand failed (xnz/tca/34/modes/crank/hold)\n"); goto fail;
    }
    else
    {
        XPLMRegisterCommandHandler(global_context->commands.cmd_m_34_ch, &chandler_m_34_ch, 0, &global_context->commands);
    }
    if (NULL == (global_context->commands.cmd_m_34_cr = XPLMCreateCommand("xnz/tca/34/modes/crank", "TCA 3&4: crank mode set")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMCreateCommand failed (xnz/tca/34/modes/crank)\n"); goto fail;
    }
    else
    {
        XPLMRegisterCommandHandler(global_context->commands.cmd_m_34_cr, &chandler_m_34_cr, 0, &global_context->commands);
    }
    if (NULL == (global_context->commands.cmd_m_34_no = XPLMCreateCommand("xnz/tca/34/modes/norm", "TCA 3&4: norm mode set")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMCreateCommand failed (xnz/tca/34/modes/norm)\n"); goto fail;
    }
    else
    {
        XPLMRegisterCommandHandler(global_context->commands.cmd_m_34_no, &chandler_m_34_no, 0, &global_context->commands);
    }
    if (NULL == (global_context->commands.cmd_m_34_st = XPLMCreateCommand("xnz/tca/34/modes/start", "TCA 3&4: start mode set")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMCreateCommand failed (xnz/tca/34/modes/start)\n"); goto fail;
    }
    else
    {
        XPLMRegisterCommandHandler(global_context->commands.cmd_m_34_st, &chandler_m_34_st, 0, &global_context->commands);
    }
    if (NULL == (global_context->commands.cmd_m_34_sh = XPLMCreateCommand("xnz/tca/34/modes/start/hold", "TCA 3&4: start mode hold")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMCreateCommand failed (xnz/tca/34/modes/start/hold)\n"); goto fail;
    }
    else
    {
        XPLMRegisterCommandHandler(global_context->commands.cmd_m_34_sh, &chandler_m_34_sh, 0, &global_context->commands);
    }
    if (NULL == (global_context->commands.cmd_e_1_onh = XPLMCreateCommand("xnz/tca/engines/1/on/hold", "TCA: engine 1 ON hold")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMCreateCommand failed (xnz/tca/engines/1/on/hold)\n"); goto fail;
    }
    else
    {
        XPLMRegisterCommandHandler(global_context->commands.cmd_e_1_onh, &chandler_e_1_onh, 0, &global_context->commands);
    }
    if (NULL == (global_context->commands.cmd_e_1_onn = XPLMCreateCommand("xnz/tca/engines/1/on", "TCA: engine 1 to ON")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMCreateCommand failed (xnz/tca/engines/1/on)\n"); goto fail;
    }
    else
    {
        XPLMRegisterCommandHandler(global_context->commands.cmd_e_1_onn, &chandler_e_1_onn, 0, &global_context->commands);
    }
    if (NULL == (global_context->commands.cmd_e_1_off = XPLMCreateCommand("xnz/tca/engines/1/off", "TCA: engine 1 to OFF")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMCreateCommand failed (xnz/tca/engines/1/off)\n"); goto fail;
    }
    else
    {
        XPLMRegisterCommandHandler(global_context->commands.cmd_e_1_off, &chandler_e_1_off, 0, &global_context->commands);
    }
    if (NULL == (global_context->commands.cmd_e_2_onh = XPLMCreateCommand("xnz/tca/engines/2/on/hold", "TCA: engine 2 ON hold")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMCreateCommand failed (xnz/tca/engines/2/on/hold)\n"); goto fail;
    }
    else
    {
        XPLMRegisterCommandHandler(global_context->commands.cmd_e_2_onh, &chandler_e_2_onh, 0, &global_context->commands);
    }
    if (NULL == (global_context->commands.cmd_e_2_onn = XPLMCreateCommand("xnz/tca/engines/2/on", "TCA: engine 2 to ON")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMCreateCommand failed (xnz/tca/engines/2/on)\n"); goto fail;
    }
    else
    {
        XPLMRegisterCommandHandler(global_context->commands.cmd_e_2_onn, &chandler_e_2_onn, 0, &global_context->commands);
    }
    if (NULL == (global_context->commands.cmd_e_2_off = XPLMCreateCommand("xnz/tca/engines/2/off", "TCA: engine 2 to OFF")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMCreateCommand failed (xnz/tca/engines/2/off)\n"); goto fail;
    }
    else
    {
        XPLMRegisterCommandHandler(global_context->commands.cmd_e_2_off, &chandler_e_2_off, 0, &global_context->commands);
    }
    if (NULL == (global_context->commands.cmd_e_3_onh = XPLMCreateCommand("xnz/tca/engines/3/on/hold", "TCA: engine 3 ON hold")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMCreateCommand failed (xnz/tca/engines/1/on/hold)\n"); goto fail;
    }
    else
    {
        XPLMRegisterCommandHandler(global_context->commands.cmd_e_3_onh, &chandler_e_3_onh, 0, &global_context->commands);
    }
    if (NULL == (global_context->commands.cmd_e_3_onn = XPLMCreateCommand("xnz/tca/engines/3/on", "TCA: engine 3 to ON")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMCreateCommand failed (xnz/tca/engines/3/on)\n"); goto fail;
    }
    else
    {
        XPLMRegisterCommandHandler(global_context->commands.cmd_e_3_onn, &chandler_e_3_onn, 0, &global_context->commands);
    }
    if (NULL == (global_context->commands.cmd_e_3_off = XPLMCreateCommand("xnz/tca/engines/3/off", "TCA: engine 3 to OFF")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMCreateCommand failed (xnz/tca/engines/3/off)\n"); goto fail;
    }
    else
    {
        XPLMRegisterCommandHandler(global_context->commands.cmd_e_3_off, &chandler_e_3_off, 0, &global_context->commands);
    }
    if (NULL == (global_context->commands.cmd_e_4_onh = XPLMCreateCommand("xnz/tca/engines/4/on/hold", "TCA: engine 4 ON hold")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMCreateCommand failed (xnz/tca/engines/4/on/hold)\n"); goto fail;
    }
    else
    {
        XPLMRegisterCommandHandler(global_context->commands.cmd_e_4_onh, &chandler_e_4_onh, 0, &global_context->commands);
    }
    if (NULL == (global_context->commands.cmd_e_4_onn = XPLMCreateCommand("xnz/tca/engines/4/on", "TCA: engine 4 to ON")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMCreateCommand failed (xnz/tca/engines/4/on)\n"); goto fail;
    }
    else
    {
        XPLMRegisterCommandHandler(global_context->commands.cmd_e_4_onn, &chandler_e_4_onn, 0, &global_context->commands);
    }
    if (NULL == (global_context->commands.cmd_e_4_off = XPLMCreateCommand("xnz/tca/engines/4/off", "TCA: engine 4 to OFF")))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPLMCreateCommand failed (xnz/tca/engines/4/off)\n"); goto fail;
    }
    else
    {
        XPLMRegisterCommandHandler(global_context->commands.cmd_e_4_off, &chandler_e_4_off, 0, &global_context->commands);
    }
    global_context->commands.xp_11_50_or_later = (outXPlaneVersion > 11499);
#endif

    /* Datarefs: axis input and XNZ-processed output */
    if (NULL == (global_context->f_throt_inn = XPLMRegisterDataAccessor("xnz/throttle/ratio/inn",
                                                                        xplmType_Float, 0,
                                                                        NULL, NULL,
                                                                        &XNZGetDataf, NULL,
                                                                        NULL, NULL,
                                                                        NULL, NULL,
                                                                        NULL, NULL,
                                                                        NULL, NULL,
                                                                        &global_context->avrg_throttle_inn, NULL)))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (XPLMRegisterDataAccessor)\n"); goto fail;
    }
    if (NULL == (global_context->f_throt_out = XPLMRegisterDataAccessor("xnz/throttle/ratio/out",
                                                                        xplmType_Float, 0,
                                                                        NULL, NULL,
                                                                        &XNZGetDataf, NULL,
                                                                        NULL, NULL,
                                                                        NULL, NULL,
                                                                        NULL, NULL,
                                                                        NULL, NULL,
                                                                        &global_context->avrg_throttle_out, NULL)))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (XPLMRegisterDataAccessor)\n"); goto fail;
    }

    /* TCA quadrant support: toggle on/off via menu */
    if (NULL == (global_context->id_th_on_off = XPLMCreateMenu(XNZ_XPLM_TITLE, NULL, 0, &menu_hdlr_fnc, global_context)))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (XPLMCreateMenu)\n"); goto fail;
    }
    if (0 > (global_context->id_menu_item_on_off = XPLMAppendMenuItem(global_context->id_th_on_off, "TCA Throttle Quadrant", &global_context->id_menu_item_on_off, 0)))
    {
        XPLMDebugString(XNZ_LOG_PREFIX"[error]: XPluginEnable failed (XPLMAppendMenuItem)\n"); goto fail;
    }
    XPLMCheckMenuItem(global_context->id_th_on_off, global_context->id_menu_item_on_off, xplm_Menu_Checked);

    /* initialize detents, corresponding zone data */
    update_thrust_zones(&global_context->zones_info);
    default_throt_share(&global_context->zones_info);

    /* all good */
    XPLMDebugString(XNZ_LOG_PREFIX"[info]: XPluginEnable OK\n");
    global_context->i_version_simulator = outXPlaneVersion;
    global_context->i_version_xplm_apis = outXPLMVersion;
    global_context->commands.xnz_ab = XNZ_AB_ERRR;
    global_context->commands.xnz_ap = XNZ_AP_ERRR;
    global_context->commands.xnz_at = XNZ_AT_ERRR;
    global_context->commands.xnz_bt = XNZ_BT_ERRR;
    global_context->commands.xnz_et = XNZ_ET_ERRR;
    global_context->commands.xnz_pb = XNZ_PB_ERRR;
    global_context->idx_throttle_axis_1 = -1;
    global_context->tca_support_enabled = 1;
    global_context->i_got_axis_input[0] = 0;
    global_context->skip_idle_overwrite = 0;
    global_context->i_context_init_done = 0;
    global_context->xnz_tt = XNZ_TT_ERRR;
    return 1;

fail:
#ifndef PUBLIC_RELEASE_BUILD
    if (global_context->widgetid[0] != 0)
    {
        XPDestroyWidget(global_context->widgetid[0], 1);
    }
#endif
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
#ifndef PUBLIC_RELEASE_BUILD
        XPLMSetFlightLoopCallbackInterval(ctx->f_l_cb, 0, 1, ctx);
        XPLMSetDataf(ctx->nullzone[0], ctx->prefs_nullzone[0]);
        XPLMSetDataf(ctx->nullzone[1], ctx->prefs_nullzone[1]);
        XPLMSetDataf(ctx->nullzone[2], ctx->prefs_nullzone[2]);
        XPLMSetDataf(ctx->acf_roll_co, ctx->nominal_roll_coef);
        if (XPIsWidgetVisible(ctx->widgetid[1]) != 0)
        {
            XPHideWidget(ctx->widgetid[0]);
            XPHideWidget(ctx->widgetid[1]);
        }
#endif
        XPLMSetFlightLoopCallbackInterval(ctx->f_l_th, 0, 1, ctx);
        default_throt_share(&ctx->zones_info);
        ctx->commands.xnz_at = XNZ_AT_ERRR;
        ctx->commands.xnz_ab = XNZ_AB_ERRR;
        ctx->commands.xnz_ap = XNZ_AP_ERRR;
        ctx->commands.xnz_bt = XNZ_BT_ERRR;
        ctx->commands.xnz_et = XNZ_ET_ERRR;
        ctx->commands.xnz_pb = XNZ_PB_ERRR;
        ctx->i_context_init_done = 0;
        ctx->skip_idle_overwrite = 0;
        ctx->xnz_tt = XNZ_TT_ERRR;
    }
}

PLUGIN_API void XPluginDisable(void)
{
    xnz_context_reset(global_context);

#ifndef PUBLIC_RELEASE_BUILD
    XPLMUnregisterFlightLoopCallback(global_context->f_l_cb, global_context);
    XPLMSetDataf(global_context->nullzone[0], global_context->prefs_nullzone[0]);
    XPLMSetDataf(global_context->nullzone[1], global_context->prefs_nullzone[1]);
    XPLMSetDataf(global_context->nullzone[2], global_context->prefs_nullzone[2]);
    XPLMSetDataf(global_context->acf_roll_co, global_context->nominal_roll_coef);
    if (XPIsWidgetVisible(global_context->widgetid[1]) != 0)
    {
        XPHideWidget(global_context->widgetid[0]);
        XPHideWidget(global_context->widgetid[1]);
    }
    if (global_context->widgetid[0] != 0)
    {
        XPDestroyWidget(global_context->widgetid[0], 1);
    }
    if (global_context->print_ax)
    {
        XPLMUnregisterCommandHandler(global_context->print_ax, &chandler_printax, 0, global_context);
    }
    if (global_context->commands.cmd_rgb_pkb) XPLMUnregisterCommandHandler(global_context->commands.cmd_rgb_pkb, &chandler_rgb_pkb, 0, &global_context->commands);
    if (global_context->commands.cmd_rgb_hld) XPLMUnregisterCommandHandler(global_context->commands.cmd_rgb_hld, &chandler_rgb_hld, 0, &global_context->commands);
    if (global_context->commands.cmd_pkb_tog) XPLMUnregisterCommandHandler(global_context->commands.cmd_pkb_tog, &chandler_pkb_tog, 0, &global_context->commands);
    if (global_context->commands.cmd_pkb_onh) XPLMUnregisterCommandHandler(global_context->commands.cmd_pkb_onh, &chandler_pkb_onh, 0, &global_context->commands);
    if (global_context->commands.cmd_pkb_onn) XPLMUnregisterCommandHandler(global_context->commands.cmd_pkb_onn, &chandler_pkb_onn, 0, &global_context->commands);
    if (global_context->commands.cmd_pkb_off) XPLMUnregisterCommandHandler(global_context->commands.cmd_pkb_off, &chandler_pkb_off, 0, &global_context->commands);
    if (global_context->commands.cmd_at_toga) XPLMUnregisterCommandHandler(global_context->commands.cmd_at_toga, &chandler_at_toga, 0, &global_context->commands);
    if (global_context->commands.cmd_a_12_lt) XPLMUnregisterCommandHandler(global_context->commands.cmd_a_12_lt, &chandler_a_12_lt, 0, &global_context->commands);
    if (global_context->commands.cmd_a_12_rt) XPLMUnregisterCommandHandler(global_context->commands.cmd_a_12_rt, &chandler_a_12_rt, 0, &global_context->commands);
    if (global_context->commands.cmd_x_12_lt) XPLMUnregisterCommandHandler(global_context->commands.cmd_x_12_lt, &chandler_x_12_lt, 0, &global_context->commands);
    if (global_context->commands.cmd_x_12_rt) XPLMUnregisterCommandHandler(global_context->commands.cmd_x_12_rt, &chandler_x_12_rt, 0, &global_context->commands);
    if (global_context->commands.cmd_a_34_lt) XPLMUnregisterCommandHandler(global_context->commands.cmd_a_34_lt, &chandler_a_34_lt, 0, &global_context->commands);
    if (global_context->commands.cmd_a_34_rt) XPLMUnregisterCommandHandler(global_context->commands.cmd_a_34_rt, &chandler_a_34_rt, 0, &global_context->commands);
    if (global_context->commands.cmd_x_34_lt) XPLMUnregisterCommandHandler(global_context->commands.cmd_x_34_lt, &chandler_x_34_lt, 0, &global_context->commands);
    if (global_context->commands.cmd_x_34_rt) XPLMUnregisterCommandHandler(global_context->commands.cmd_x_34_rt, &chandler_x_34_rt, 0, &global_context->commands);
    if (global_context->commands.cmd_m_12_ch) XPLMUnregisterCommandHandler(global_context->commands.cmd_m_12_ch, &chandler_m_12_ch, 0, &global_context->commands);
    if (global_context->commands.cmd_m_12_cr) XPLMUnregisterCommandHandler(global_context->commands.cmd_m_12_cr, &chandler_m_12_cr, 0, &global_context->commands);
    if (global_context->commands.cmd_m_12_no) XPLMUnregisterCommandHandler(global_context->commands.cmd_m_12_no, &chandler_m_12_no, 0, &global_context->commands);
    if (global_context->commands.cmd_m_12_st) XPLMUnregisterCommandHandler(global_context->commands.cmd_m_12_st, &chandler_m_12_st, 0, &global_context->commands);
    if (global_context->commands.cmd_m_12_sh) XPLMUnregisterCommandHandler(global_context->commands.cmd_m_12_sh, &chandler_m_12_sh, 0, &global_context->commands);
    if (global_context->commands.cmd_m_34_ch) XPLMUnregisterCommandHandler(global_context->commands.cmd_m_34_ch, &chandler_m_34_ch, 0, &global_context->commands);
    if (global_context->commands.cmd_m_34_cr) XPLMUnregisterCommandHandler(global_context->commands.cmd_m_34_cr, &chandler_m_34_cr, 0, &global_context->commands);
    if (global_context->commands.cmd_m_34_no) XPLMUnregisterCommandHandler(global_context->commands.cmd_m_34_no, &chandler_m_34_no, 0, &global_context->commands);
    if (global_context->commands.cmd_m_34_st) XPLMUnregisterCommandHandler(global_context->commands.cmd_m_34_st, &chandler_m_34_st, 0, &global_context->commands);
    if (global_context->commands.cmd_m_34_sh) XPLMUnregisterCommandHandler(global_context->commands.cmd_m_34_sh, &chandler_m_34_sh, 0, &global_context->commands);
    if (global_context->commands.cmd_e_1_onh) XPLMUnregisterCommandHandler(global_context->commands.cmd_e_1_onh, &chandler_e_1_onh, 0, &global_context->commands);
    if (global_context->commands.cmd_e_1_onn) XPLMUnregisterCommandHandler(global_context->commands.cmd_e_1_onn, &chandler_e_1_onn, 0, &global_context->commands);
    if (global_context->commands.cmd_e_1_off) XPLMUnregisterCommandHandler(global_context->commands.cmd_e_1_off, &chandler_e_1_off, 0, &global_context->commands);
    if (global_context->commands.cmd_e_2_onh) XPLMUnregisterCommandHandler(global_context->commands.cmd_e_2_onh, &chandler_e_2_onh, 0, &global_context->commands);
    if (global_context->commands.cmd_e_2_onn) XPLMUnregisterCommandHandler(global_context->commands.cmd_e_2_onn, &chandler_e_2_onn, 0, &global_context->commands);
    if (global_context->commands.cmd_e_2_off) XPLMUnregisterCommandHandler(global_context->commands.cmd_e_2_off, &chandler_e_2_off, 0, &global_context->commands);
    if (global_context->commands.cmd_e_3_onh) XPLMUnregisterCommandHandler(global_context->commands.cmd_e_3_onh, &chandler_e_3_onh, 0, &global_context->commands);
    if (global_context->commands.cmd_e_3_onn) XPLMUnregisterCommandHandler(global_context->commands.cmd_e_3_onn, &chandler_e_3_onn, 0, &global_context->commands);
    if (global_context->commands.cmd_e_3_off) XPLMUnregisterCommandHandler(global_context->commands.cmd_e_3_off, &chandler_e_3_off, 0, &global_context->commands);
    if (global_context->commands.cmd_e_4_onh) XPLMUnregisterCommandHandler(global_context->commands.cmd_e_4_onh, &chandler_e_4_onh, 0, &global_context->commands);
    if (global_context->commands.cmd_e_4_onn) XPLMUnregisterCommandHandler(global_context->commands.cmd_e_4_onn, &chandler_e_4_onn, 0, &global_context->commands);
    if (global_context->commands.cmd_e_4_off) XPLMUnregisterCommandHandler(global_context->commands.cmd_e_4_off, &chandler_e_4_off, 0, &global_context->commands);
#endif

    XPLMUnregisterFlightLoopCallback(global_context->f_l_th, global_context);

    /* Datarefs: axis input and XNZ-processed output */
    if (global_context->f_throt_inn)
    {
        XPLMUnregisterDataAccessor(global_context->f_throt_inn);
        global_context->f_throt_inn = NULL;
    }
    if (global_context->f_throt_out)
    {
        XPLMUnregisterDataAccessor(global_context->f_throt_out);
        global_context->f_throt_out = NULL;
    }

    /* re-enable throttle 1/2 axes */
    if (global_context->idx_throttle_axis_1 >= 0)
    {
        int th_axis_ass[2] = { 20, 21, };
        xnz_log("[info]: releasing joystick axes (XPluginDisable)\n");
        XPLMSetDatavi(global_context->i_stick_ass, th_axis_ass, global_context->idx_throttle_axis_1, 2);
    }

    /* close context */
    if (NULL != global_context)
    {
        free(global_context);
        global_context = NULL;
    }

    /* all good */
    XPLMDebugString(XNZ_LOG_PREFIX"[info]: XPluginDisable OK\n");
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

/*
 * TODO: future: XNZ_AP_COMM
 *
 * ToLiSS A319, A321
 * ctx->otto.conn.cc.name = "toliss_airbus/ap1_push";
 * ctx->otto.disc.cc.name = "toliss_airbus/ap_disc_left_stick";
 *
 * FlightFactor A350 (X-Plane 10)
 * ctx->otto.conn.cc.name = "airbus_qpac/ap1_push";
 * ctx->otto.disc.cc.name = "sim/autopilot/fdir_servos_down_one";
 *
 * FlightFactor A350 (X-Plane 11)
 * ctx->otto.conn.cc.name = "airbus_qpac/ap1_push";
 * ctx->otto.disc.cc.name = "airbus_qpac/ap_disc_left_stick";
 *
 * IXEG 737-300 Classic
 * ctx->otto.conn.cc.name = "ixeg/733/autopilot/AP_A_cmd_toggle";
 * ctx->otto.disc.cc.name = "ixeg/733/autopilot/AP_disengage";
 *
 * FlightFactor 757, 767
 * ctx->otto.conn.cc.name = NULL; // try to find a dataref maybe???
 * ctx->otto.disc.cc.name = "1-sim/comm/AP/ap_disc";
 */

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFromWho, long inMessage, void *inParam)
{
    if (global_context->msg_will_write_pref != 0)
    {
        if (global_context->idx_throttle_axis_1 >= 0 && global_context->tca_support_enabled != 0)
        {
            int no_axis_ass[2] = { 0, 0, };
            xnz_log("[info]: re-capturing joystick axes (XPLM_MSG_WILL_WRITE_PREFS done)\n");
            XPLMSetDatavi(global_context->i_stick_ass, no_axis_ass, global_context->idx_throttle_axis_1, 2);
        }
        global_context->msg_will_write_pref = 0;
    }
    switch (inMessage)
    {
        case XPLM_MSG_WILL_WRITE_PREFS:
#ifndef PUBLIC_RELEASE_BUILD
            XPLMSetDataf(global_context->nullzone[0], global_context->prefs_nullzone[0]);
            XPLMSetDataf(global_context->nullzone[1], global_context->prefs_nullzone[1]);
            XPLMSetDataf(global_context->nullzone[2], global_context->prefs_nullzone[2]);
            XPLMSetDataf(global_context->acf_roll_co, global_context->nominal_roll_coef);
#endif
            if (global_context->idx_throttle_axis_1 >= 0)
            {
                int th_axis_ass[2] = { 20, 21, };
                global_context->msg_will_write_pref = 1;
                xnz_log("[info]: releasing joystick axes (XPLM_MSG_WILL_WRITE_PREFS)\n");
                XPLMSetDatavi(global_context->i_stick_ass, th_axis_ass, global_context->idx_throttle_axis_1, 2);
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
            if (inParam != XPLM_USER_AIRCRAFT)
            {
                break; // not user aircraft, initialization not required nor desirable here
            }
            else // deferred initialization: wait for initial livery load after aircraft load -> custom aircraft plugins loaded, if any (for custom datarefs)
            {
                if (global_context->i_context_init_done)
                {
                    break; // don't re-init on subsequent livery changes
                }
#ifndef PUBLIC_RELEASE_BUILD
                global_context->minimum_null_zone = 0.04f; // hardcoded for now
                global_context->nominal_roll_coef = XPLMGetDataf(global_context->acf_roll_co);
                global_context->prefs_nullzone[0] = XPLMGetDataf(global_context->nullzone[0]);
                global_context->prefs_nullzone[1] = XPLMGetDataf(global_context->nullzone[1]);
                global_context->prefs_nullzone[2] = XPLMGetDataf(global_context->nullzone[2]);
                xnz_log("new aircraft: original nullzones %.3lf %.3lf %.3lf (minimum %.3lf)\n",
                        global_context->prefs_nullzone[0],
                        global_context->prefs_nullzone[1],
                        global_context->prefs_nullzone[2],
                        global_context->minimum_null_zone);
                if (global_context->i_version_simulator < 11000)
                {
                    float rc0; ACF_ROLL_SET(rc0, GROUNDSP_KTS_MIN, global_context->nominal_roll_coef);
                    float rc1; ACF_ROLL_SET(rc1, GROUNDSP_KTS_MID, global_context->nominal_roll_coef);
                    float rc2; ACF_ROLL_SET(rc2, GROUNDSP_KTS_MAX, global_context->nominal_roll_coef);
                    xnz_log("new aircraft: original roll coefficient %.3lf (%.3lf -> %.3lf -> %.3lf)\n",
                            global_context->nominal_roll_coef, rc0, rc1, rc2);
                }
#endif
                // check for engine count, type and related info
                if ((global_context->arcrft_engine_count = XPLMGetDatai(global_context->i_ngine_num)) > 8)
                {
                    global_context->arcrft_engine_count = 8;
                }
                else if (global_context->arcrft_engine_count < 1)
                {
                    global_context->arcrft_engine_count = 1;
                }
                int acf_en_type[8]; XPLMGetDatavi(global_context->i_ngine_typ, acf_en_type, 0, global_context->arcrft_engine_count);
                if (global_context->arcrft_engine_count >= 2)
                {
                    for (int i = global_context->arcrft_engine_count; i > 0; i--)
                    {
                        if (acf_en_type[i] != acf_en_type[0]) // some Carenado have extra engine for special effects
                        {
                            global_context->arcrft_engine_count--;
                            continue;
                        }
                        continue;
                    }
                }
                switch (acf_en_type[0])
                {
                    case 2: // turboprop
                    case 8: // turboprop
                    case 9: // turboprop (XP11+, not documented in Datarefs.txt)
                        global_context->acf_has_beta_thrust = XPLMGetDatai(global_context->rev_info[0]) != 0;
                        global_context->acft_has_rev_thrust =
                        (XPLMGetDatai(global_context->rev_info[1]) != 0 &&
                         XPLMGetDataf(global_context->rev_info[2]) >= .01f);
                        break;

                    case 4: // turbojet
                    case 5: // turbofan
                        global_context->acf_has_beta_thrust = 0;
                        global_context->acft_has_rev_thrust =
                        (XPLMGetDatai(global_context->rev_info[1]) != 0 &&
                         XPLMGetDataf(global_context->rev_info[2]) >= .01f);
                        break;

                    default:
                        global_context->acf_has_beta_thrust = 0;
                        global_context->acft_has_rev_thrust = 0;
                        break;
                }
                /*
                 * XXX: disable globally; re-enable on a case-by-case basis until
                 * I can get a better understanding of X-Plane's default behavior…
                 */
                global_context->acf_has_beta_thrust = 0;

                /* check for custom thrust datarefs/API */
                int auth_desc_icao = 0;
                XPLMDataRef ref; XPLMPluginID pid;
                char auth[501], desc[261], icao[41];
                if ((ref = XPLMFindDataRef("sim/aircraft/view/acf_author")))
                {
                    dref_read_str(ref, auth, sizeof(auth));
                    if ((ref = XPLMFindDataRef("sim/aircraft/view/acf_descrip")))
                    {
                        dref_read_str(ref, desc, sizeof(desc));
                        if ((ref = XPLMFindDataRef("sim/aircraft/view/acf_ICAO")))
                        {
                            dref_read_str(ref, icao, sizeof(icao));
                            auth_desc_icao = 1;
                        }
                    }
                }
                if (global_context->commands.xnz_ab == XNZ_AB_ERRR)
                {
                    global_context->commands.xnz_ab = XNZ_AB_NONE; // no reliable way to detect, enable on case-by-case basis below
                }
                if (global_context->commands.xnz_ap == XNZ_AP_ERRR)
                {
                    if (NULL != XPLMFindDataRef("sim/version/xplane_internal_version")) // lazy XP11+ detection
                    {
                        if ((ref = XPLMFindDataRef("sim/aircraft/autopilot/preconfigured_ap_type")))
                        {
                            if (2 == XPLMGetDatai(ref)) // 2=GFC-700
                            {
                                global_context->commands.xnz_ap = XNZ_AP_XGFC;
                            }
                            else
                            {
                                global_context->commands.xnz_ap = XNZ_AP_XPLM; // assume default X-Plane until proven otherwise
                            }
                        }
                        else
                        {
                            global_context->commands.xnz_ap = XNZ_AP_XPLM; // assume default X-Plane until proven otherwise
                        }
                    }
                    else
                    {
                        global_context->commands.xnz_ap = XNZ_AP_XPLM; // assume default X-Plane until proven otherwise
                    }
                }
                if (global_context->commands.xnz_at == XNZ_AT_ERRR)
                {
                    if (NULL != XPLMFindDataRef("sim/version/xplane_internal_version")) // lazy XP11+ detection
                    {
                        if ((ref = XPLMFindDataRef("sim/aircraft/autopilot/preconfigured_ap_type")))
                        {
                            if (1 == XPLMGetDatai(ref)) // 1=Airliner
                            {
                                global_context->commands.xnz_ap = XNZ_AT_XP11;
                            }
                            else
                            {
                                global_context->commands.xnz_at = XNZ_AT_NONE; // no reliable way to detect, enable on case-by-case basis below
                            }
                        }
                        else
                        {
                            global_context->commands.xnz_at = XNZ_AT_NONE; // no reliable way to detect, enable on case-by-case basis below
                        }
                    }
                    else
                    {
                        global_context->commands.xnz_at = XNZ_AT_NONE; // no reliable way to detect, enable on case-by-case basis below
                    }
                }
                if (global_context->commands.xnz_bt == XNZ_BT_ERRR)
                {
                    if (((XPLM_NO_PLUGIN_ID != (pid = XPLMFindPluginBySignature("com.simcoders.rep"))) && (XPLMIsPluginEnabled(pid))))
                    {
                        global_context->commands.xnz_bt = XNZ_BT_SIMC; // use parkbrake and slightly increased strength
                        global_context->commands.xp.pbrak_onoff = -1; // initialize our parking brake tracking variable
                    }
                    else
                    {
                        global_context->commands.xnz_bt = XNZ_BT_XPLM; // assume default X-Plane until proven otherwise
                        global_context->commands.xp.pbrak_onoff = -1; // initialize our parking brake tracking variable
                    }
                }
                if (global_context->commands.xnz_et == XNZ_ET_ERRR)
                {
                    if (global_context->arcrft_engine_count >= 1 && global_context->arcrft_engine_count <= 4)
                    {
                        switch (acf_en_type[0])
                        {
                            case 0:
                            case 1: // carbureted/injected recip. engines
                                global_context->commands.xnz_et = XNZ_ET_XPPI;
                                break;

                            case 2:
                            case 8: // free/fixed turbine engines
                            case 9: // turbine engine too (XP11+)
                                global_context->commands.xnz_et = XNZ_ET_XPTP;
                                break;

                            case 4:
                            case 5: // low/high bypass jet engines
                                global_context->commands.xnz_et = XNZ_ET_XPJT;
                                break;

                            default:
                                global_context->commands.xnz_et = XNZ_ET_NONE;
                                break;
                        }
                    }
                    else
                    {
                        global_context->commands.xnz_et = XNZ_ET_NONE;
                    }
                }
                if (global_context->commands.xnz_pb == XNZ_PB_ERRR)
                {
                    global_context->commands.xnz_pb = XNZ_PB_XPLM; // assume default X-Plane until proven otherwise
                }
                if (global_context->xnz_tt == XNZ_TT_ERRR)
                {
                    global_context->xnz_tt = XNZ_TT_XPLM; // assume default X-Plane until proven otherwise
                }
                if ((XPLM_NO_PLUGIN_ID != (pid = XPLMFindPluginBySignature(XPLM_FF_SIGNATURE))) && (XPLMIsPluginEnabled(pid)))
                {
                    global_context->  commands.xnz_ab = XNZ_AB_FF32;
                    global_context->  commands.xnz_ap = XNZ_AP_FF32;
                    global_context->  commands.xnz_at = XNZ_AT_FF32;
                    global_context->  commands.xnz_bt = XNZ_BT_FF32;
                    global_context->  commands.xnz_et = XNZ_ET_FF32;
                    global_context->  commands.xnz_pb = XNZ_PB_FF32;
                    global_context->           xnz_tt = XNZ_TT_FF32;
                    global_context->tt.ff32.api_has_initialized = 0;
                }
                else if (((XPLM_NO_PLUGIN_ID != (pid = XPLMFindPluginBySignature("XP10.ToLiss.A319.systems"))) && (XPLMIsPluginEnabled(pid))) ||
                         ((XPLM_NO_PLUGIN_ID != (pid = XPLMFindPluginBySignature("XP10.ToLiss.A321.systems"))) && (XPLMIsPluginEnabled(pid))) ||
                         ((XPLM_NO_PLUGIN_ID != (pid = XPLMFindPluginBySignature("XP11.ToLiss.A319.systems"))) && (XPLMIsPluginEnabled(pid))) ||
                         ((XPLM_NO_PLUGIN_ID != (pid = XPLMFindPluginBySignature("XP11.ToLiss.A321.systems"))) && (XPLMIsPluginEnabled(pid))))
                {
                    if (NULL == (global_context->         tt.toli.f_thr_array = XPLMFindDataRef("AirbusFBW/throttle_input"                         )) ||
                        NULL == (global_context->commands.pb.to32.pbrak_onoff = XPLMFindDataRef("AirbusFBW/ParkBrake"                              )) ||
// that was the bug :-( NULL == (global_context->commands.bt.to32.l_rgb_ratio = XPLMFindDataRef("AirbusFBW/BrakePedalInputleft"                    )) || // studid typo :-(
                        NULL == (global_context->commands.bt.to32.l_rgb_ratio = XPLMFindDataRef("AirbusFBW/BrakePedalInputLeft"                    )) ||
                        NULL == (global_context->commands.bt.to32.r_rgb_ratio = XPLMFindDataRef("AirbusFBW/BrakePedalInputRight"                   )) ||
                        NULL == (global_context->commands.bt.to32.br_override = XPLMFindDataRef("AirbusFBW/BrakePedalInputOverride"                )) ||
                        NULL == (global_context->commands.et.to32.cmd_e_1_onn = XPLMFindCommand("toliss_airbus/engcommands/Master1On"              )) ||
                        NULL == (global_context->commands.et.to32.cmd_e_1_off = XPLMFindCommand("toliss_airbus/engcommands/Master1Off"             )) ||
                        NULL == (global_context->commands.et.to32.cmd_e_2_onn = XPLMFindCommand("toliss_airbus/engcommands/Master2On"              )) ||
                        NULL == (global_context->commands.et.to32.cmd_e_2_off = XPLMFindCommand("toliss_airbus/engcommands/Master2Off"             )) ||
                        NULL == (global_context->commands.et.to32.cmd_m_12_cr = XPLMFindCommand("toliss_airbus/engcommands/EngineModeSwitchToCrank")) ||
                        NULL == (global_context->commands.et.to32.cmd_m_12_no = XPLMFindCommand("toliss_airbus/engcommands/EngineModeSwitchToNorm" )) ||
                        NULL == (global_context->commands.et.to32.cmd_m_12_st = XPLMFindCommand("toliss_airbus/engcommands/EngineModeSwitchToStart")))
                    {
                        xnz_log("[error]: could not self-configure for TO32\n");
                        global_context->commands.xnz_ab = XNZ_AB_ERRR;
                        global_context->commands.xnz_ap = XNZ_AP_ERRR;
                        global_context->commands.xnz_at = XNZ_AT_ERRR;
                        global_context->commands.xnz_bt = XNZ_BT_ERRR;
                        global_context->commands.xnz_et = XNZ_ET_ERRR;
                        global_context->commands.xnz_pb = XNZ_PB_ERRR;
                        global_context->         xnz_tt = XNZ_TT_ERRR;
                    }
                    else
                    {
                        XPLMSetDatai(global_context->commands.bt.to32.br_override, 1);
                        global_context->commands.xnz_ab = XNZ_AB_TO32;
//                        global_context->commands.xnz_ab = XNZ_AP_COMM;
                        global_context->commands.xnz_at = XNZ_AT_TOLI;
                        global_context->commands.xnz_bt = XNZ_BT_TO32;
                        global_context->commands.xnz_et = XNZ_ET_TO32;
                        global_context->commands.xnz_pb = XNZ_PB_TO32;
                        global_context->         xnz_tt = XNZ_TT_TOLI;
                    }
                }
                else if (((XPLM_NO_PLUGIN_ID != (pid = XPLMFindPluginBySignature("ToLiSs.Airbus.systems"))) && (XPLMIsPluginEnabled(pid)))) // A350v1.4--
                {
                    if (((XPLM_NO_PLUGIN_ID != (pid = XPLMFindPluginBySignature(  "FFSTSmousehandler")))) || // A350v1.3--
                        ((XPLM_NO_PLUGIN_ID != (pid = XPLMFindPluginBySignature("ru.ffsts.mousehandler"))))) // A350v1.4.x
                    {
                        if (NULL == (global_context->tt.toli.f_thr_array = XPLMFindDataRef("AirbusFBW/throttle_input" )) ||
                            NULL == (global_context->commands.pb.ff35.pbrak_offon = XPLMFindDataRef("1-sim/parckBrake")))
                        {
                            xnz_log("[error]: could not self-configure for FF35 (XP10)\n");
                            global_context->commands.xnz_ab = XNZ_AB_ERRR;
                            global_context->commands.xnz_ap = XNZ_AP_ERRR;
                            global_context->commands.xnz_at = XNZ_AT_ERRR;
                            global_context->commands.xnz_bt = XNZ_BT_ERRR;
                            global_context->commands.xnz_et = XNZ_ET_ERRR;
                            global_context->commands.xnz_pb = XNZ_PB_ERRR;
                            global_context->         xnz_tt = XNZ_TT_ERRR;
                        }
                        else
                        {
                            global_context->commands.xnz_ab = XNZ_AB_FF35;
//                            global_context->commands.xnz_ap = XNZ_AP_COMM;
                            global_context->commands.xnz_at = XNZ_AT_TOLI;
                            global_context->commands.xnz_bt = XNZ_BT_FF35;
                            global_context->commands.xnz_et = XNZ_ET_ERRR;
                            global_context->commands.xnz_pb = XNZ_PB_FF35;
                            global_context->         xnz_tt = XNZ_TT_TOLI;
                        }
                    }
                }
                else if (((XPLM_NO_PLUGIN_ID != (pid = XPLMFindPluginBySignature("XP11.ToLiss.Airbus.systems"))) && (XPLMIsPluginEnabled(pid)))) // A350v1.6++
                {
                    if (((XPLM_NO_PLUGIN_ID != (pid = XPLMFindPluginBySignature("ru.stsff.mousehandler"))))) // A350v1.6++
                    {
                        if (NULL == (global_context->         tt.toli.f_thr_array = XPLMFindDataRef("AirbusFBW/throttle_input"      )) ||
                            NULL == (global_context->commands.pb.ff35.pbrak_offon = XPLMFindDataRef("1-sim/parckBrake"              )) ||
                            NULL == (global_context->commands.et.ff35.cmd_e_1_onn = XPLMFindCommand("1-sim/comm/cutOnLeft"          )) ||
                            NULL == (global_context->commands.et.ff35.cmd_e_1_off = XPLMFindCommand("1-sim/comm/cutOffLeft"         )) ||
                            NULL == (global_context->commands.et.ff35.cmd_e_2_onn = XPLMFindCommand("1-sim/comm/cutOnRight"         )) ||
                            NULL == (global_context->commands.et.ff35.cmd_e_2_off = XPLMFindCommand("1-sim/comm/cutOffRight"        )) ||
                            NULL == (global_context->commands.et.ff35.cmd_m_12_cr = XPLMFindCommand("1-sim/comm/startSwitchCrank"   )) ||
                            NULL == (global_context->commands.et.ff35.cmd_m_12_no = XPLMFindCommand("1-sim/comm/startSwitchNorm"    )) ||
                            NULL == (global_context->commands.et.ff35.cmd_m_12_st = XPLMFindCommand("1-sim/comm/startSwitchStart")))
                        {
                            xnz_log("[error]: could not self-configure for FF35 (XP11)\n");
                            global_context->commands.xnz_ab = XNZ_AB_ERRR;
                            global_context->commands.xnz_ap = XNZ_AP_ERRR;
                            global_context->commands.xnz_at = XNZ_AT_ERRR;
                            global_context->commands.xnz_bt = XNZ_BT_ERRR;
                            global_context->commands.xnz_et = XNZ_ET_ERRR;
                            global_context->commands.xnz_pb = XNZ_PB_ERRR;
                            global_context->         xnz_tt = XNZ_TT_ERRR;
                        }
                        else
                        {
                            global_context->commands.xnz_ab = XNZ_AB_FF35;
//                            global_context->commands.xnz_ap = XNZ_AP_COMM;
                            global_context->commands.xnz_at = XNZ_AT_TOLI;
                            global_context->commands.xnz_bt = XNZ_BT_FF35;
                            global_context->commands.xnz_et = XNZ_ET_FF35;
                            global_context->commands.xnz_pb = XNZ_PB_FF35;
                            global_context->         xnz_tt = XNZ_TT_TOLI;
                        }
                    }
                }
                else if (XPLM_NO_PLUGIN_ID != (pid = XPLMFindPluginBySignature("ru.stsff.757767avionics")) ||
                         XPLM_NO_PLUGIN_ID != (pid = XPLMFindPluginBySignature("ru.flightfactor-steptosky.757767avionics")) ||
                         XPLM_NO_PLUGIN_ID != (pid = XPLMFindPluginBySignature("de-ru.philippmuenzel-den_rain.757avionics")))
                {
                    if (NULL == (global_context->commands.at.comm.cmd_at_disc = XPLMFindCommand("1-sim/comm/AP/at_disc")) ||
                        NULL == (global_context->commands.at.comm.cmd_at_toga = XPLMFindCommand("1-sim/comm/AP/at_toga")) ||
                        NULL == (global_context->commands.et.ff75.drf_e_1_cut = XPLMFindDataRef("1-sim/fuel/fuelCutOffLeft")) ||
                        NULL == (global_context->commands.et.ff75.drf_e_2_cut = XPLMFindDataRef("1-sim/fuel/fuelCutOffRight")) ||
                        NULL == (global_context->commands.et.ff75.drf_e_1_knb = XPLMFindDataRef("1-sim/engine/leftStartSelector")) ||
                        NULL == (global_context->commands.et.ff75.drf_e_2_knb = XPLMFindDataRef("1-sim/engine/rightStartSelector")))
                    {
                        xnz_log("[error]: could not self-configure for FlightFactor 757/767\n");
                        global_context->commands.xnz_ab = XNZ_AB_ERRR;
                        global_context->commands.xnz_ap = XNZ_AP_ERRR;
                        global_context->commands.xnz_at = XNZ_AT_ERRR;
                        global_context->commands.xnz_bt = XNZ_BT_ERRR;
                        global_context->commands.xnz_et = XNZ_ET_ERRR;
                        global_context->commands.xnz_pb = XNZ_PB_ERRR;
                        global_context->         xnz_tt = XNZ_TT_ERRR;
                    }
                    else
                    {
                        global_context->commands.xnz_ab = XNZ_AB_FF75;
//                        global_context->commands.xnz_ap = XNZ_AP_COMM;
                        global_context->commands.xnz_at = XNZ_AT_COMM;
                        global_context->commands.xnz_bt = XNZ_BT_XPLM;
                        global_context->commands.xnz_et = XNZ_ET_FF75;
                        global_context->commands.xnz_pb = XNZ_PB_XPLM;
                        global_context->         xnz_tt = XNZ_TT_XPLM;
                    }
                }
                else if (((XPLM_NO_PLUGIN_ID != (pid = XPLMFindPluginBySignature("ERJ_Functions"))) && (XPLMIsPluginEnabled(pid))))
                {
                    if (!strncasecmp(icao, "E35L", strlen("E35L")))
                    {
                        if (NULL == (global_context->commands.et.e35l.drf_e_1_knb = XPLMFindDataRef("XCrafts/ERJ/engine1_starter_knob")) ||
                            NULL == (global_context->commands.et.e35l.drf_e_2_knb = XPLMFindDataRef("XCrafts/ERJ/engine2_starter_knob")) ||
                            NULL == (global_context->commands.et.e35l.cmd_e_1_lft = XPLMFindCommand("XCrafts/Starter_Eng_1_down_CCW"  )) ||
                            NULL == (global_context->commands.et.e35l.cmd_e_2_lft = XPLMFindCommand("XCrafts/Starter_Eng_2_down_CCW"  )) ||
                            NULL == (global_context->commands.et.e35l.cmd_e_1_rgt = XPLMFindCommand("XCrafts/Starter_Eng_1_up_CW"     )) ||
                            NULL == (global_context->commands.et.e35l.cmd_e_2_rgt = XPLMFindCommand("XCrafts/Starter_Eng_2_up_CW"     )) ||
                            NULL == (global_context->commands.at.toga.cmd_ap_toga = XPLMFindCommand("XCrafts/ERJ/TOGA"               )))
                        {
                            xnz_log("[error]: could not self-configure for E35L\n");
                            global_context->commands.xnz_ab = XNZ_AB_ERRR;
                            global_context->commands.xnz_ap = XNZ_AP_ERRR;
                            global_context->commands.xnz_at = XNZ_AT_ERRR;
                            global_context->commands.xnz_bt = XNZ_BT_ERRR;
                            global_context->commands.xnz_et = XNZ_ET_ERRR;
                            global_context->commands.xnz_pb = XNZ_PB_ERRR;
                            global_context->         xnz_tt = XNZ_TT_ERRR;
                        }
                        else
                        {
                            global_context->commands.xnz_ab = XNZ_AB_NONE;
                            global_context->commands.xnz_ap = XNZ_AP_XPLM;
                            global_context->commands.xnz_at = XNZ_AT_APTO;
                            global_context->commands.xnz_bt = XNZ_BT_XPLM;
                            global_context->commands.xnz_et = XNZ_ET_E35L;
                            global_context->commands.xnz_pb = XNZ_PB_XPLM;
                            global_context->         xnz_tt = XNZ_TT_XPLM;
                        }
                    }
                }
                else if (((XPLM_NO_PLUGIN_ID != (pid = XPLMFindPluginBySignature("hotstart.tbm900"))) && (XPLMIsPluginEnabled(pid))))
                {
                    if (NULL == (global_context->            tt.tbm9.engn_rng = XPLMFindDataRef("tbm900/systems/engine/range"        )) ||
                        NULL == (global_context->commands.bt.tbm9.rbrak_array = XPLMFindDataRef("tbm900/controls/gear/brake_req"     )) ||
                        NULL == (global_context->commands.pb.tbm9.pbrak_ratio = XPLMFindDataRef("tbm900/switches/gear/park_brake"    )) ||
                        NULL == (global_context->commands.bt.tbm9.br_override = XPLMFindDataRef("tbm900/controls/gear/brake_req_ovrd")) ||
                        NULL == (global_context->commands.et.tbm9.drf_fuelsel = XPLMFindDataRef("tbm900/switches/fuel/auto_man"      )) ||
                        NULL == (global_context->commands.et.tbm9.cmd_e_1_onn = XPLMFindCommand("sim/engines/mixture_up"             )) ||
                        NULL == (global_context->commands.et.tbm9.cmd_e_1_off = XPLMFindCommand("sim/engines/mixture_down"           )) ||
                        NULL == (global_context->commands.et.tbm9.cmd_x_12_lt = XPLMFindCommand("tbm900/actuators/elec/starter_up"   )) ||
                        NULL == (global_context->commands.et.tbm9.cmd_x_12_rt = XPLMFindCommand("tbm900/actuators/elec/starter_down" )) ||
                        NULL == (global_context->commands.et.tbm9.cmd_m_12_cr = XPLMFindCommand("tbm900/actuators/elec/ignition_off" )) ||
                        NULL == (global_context->commands.et.tbm9.cmd_m_12_no = XPLMFindCommand("tbm900/actuators/elec/ignition_auto")) ||
                        NULL == (global_context->commands.et.tbm9.cmd_m_12_st = XPLMFindCommand("tbm900/actuators/elec/ignition_on")))
                    {
                        xnz_log("[error]: could not self-configure for TBM9\n");
                        global_context->commands.xnz_ab = XNZ_AB_ERRR;
                        global_context->commands.xnz_ap = XNZ_AP_ERRR;
                        global_context->commands.xnz_at = XNZ_AT_ERRR;
                        global_context->commands.xnz_bt = XNZ_BT_ERRR;
                        global_context->commands.xnz_et = XNZ_ET_ERRR;
                        global_context->commands.xnz_pb = XNZ_PB_ERRR;
                        global_context->         xnz_tt = XNZ_TT_ERRR;
                    }
                    else
                    {
                        XPLMSetDatai(global_context->commands.bt.tbm9.br_override, 1);
                        global_context->commands.xnz_ab = XNZ_AB_NONE;
//                        global_context->commands.xnz_ap = XNZ_AP_XPLM;
                        global_context->commands.xnz_at = XNZ_AT_NONE;
                        global_context->commands.xnz_bt = XNZ_BT_TBM9;
                        global_context->commands.xnz_et = XNZ_ET_TBM9;
                        global_context->commands.xnz_pb = XNZ_PB_TBM9;
                        global_context->         xnz_tt = XNZ_TT_TBM9;
                    }
                }
                else if (((XPLM_NO_PLUGIN_ID != (pid = XPLMFindPluginBySignature("1-sim Diamond_DA62"))) && (XPLMIsPluginEnabled(pid))))
                {
                    if (NULL == (global_context->commands.et.da62.drf_mod_ec1 = XPLMFindDataRef("aerobask/eng/sw_ecu_ab1")) ||
                        NULL == (global_context->commands.et.da62.drf_mod_ec2 = XPLMFindDataRef("aerobask/eng/sw_ecu_ab2")) ||
                        NULL == (global_context->commands.et.da62.cmd_ecu1_up = XPLMFindCommand("aerobask/eng/ecu_ab1_up")) ||
                        NULL == (global_context->commands.et.da62.cmd_ecu1_dn = XPLMFindCommand("aerobask/eng/ecu_ab1_dn")) ||
                        NULL == (global_context->commands.et.da62.cmd_ecu2_up = XPLMFindCommand("aerobask/eng/ecu_ab2_up")) ||
                        NULL == (global_context->commands.et.da62.cmd_ecu2_dn = XPLMFindCommand("aerobask/eng/ecu_ab2_dn")) ||
                        NULL == (global_context->commands.et.da62.cmd_e_1_onn = XPLMFindCommand("aerobask/eng/master1_up")) ||
                        NULL == (global_context->commands.et.da62.cmd_e_1_off = XPLMFindCommand("aerobask/eng/master1_dn")) ||
                        NULL == (global_context->commands.et.da62.cmd_e_2_onn = XPLMFindCommand("aerobask/eng/master2_up")) ||
                        NULL == (global_context->commands.et.da62.cmd_e_2_off = XPLMFindCommand("aerobask/eng/master2_dn")))
                    {
                        xnz_log("[error]: could not self-configure for DA62\n");
                        global_context->commands.xnz_ab = XNZ_AB_ERRR;
                        global_context->commands.xnz_ap = XNZ_AP_ERRR;
                        global_context->commands.xnz_at = XNZ_AT_ERRR;
                        global_context->commands.xnz_bt = XNZ_BT_ERRR;
                        global_context->commands.xnz_et = XNZ_ET_ERRR;
                        global_context->commands.xnz_pb = XNZ_PB_ERRR;
                        global_context->         xnz_tt = XNZ_TT_ERRR;
                    }
                    else
                    {
                        global_context->commands.xnz_ab = XNZ_AB_NONE;
                        global_context->commands.xnz_ap = XNZ_AP_XGFC;
                        global_context->commands.xnz_at = XNZ_AT_NONE;
                        global_context->commands.xnz_bt = XNZ_BT_XPLM;
                        global_context->commands.xnz_et = XNZ_ET_DA62;
                        global_context->commands.xnz_pb = XNZ_PB_XPLM;
                        global_context->         xnz_tt = XNZ_TT_XPLM;
                    }
                }
                else if (((XPLM_NO_PLUGIN_ID != (pid = XPLMFindPluginBySignature("1-sim Phenom_300"))) && (XPLMIsPluginEnabled(pid))))
                {
                    if (NULL == (global_context->commands.et.e55p.drf_e_1_ign = XPLMFindDataRef("aerobask/engines/sw_ignition_1"    )) ||
                        NULL == (global_context->commands.et.e55p.drf_e_2_ign = XPLMFindDataRef("aerobask/engines/sw_ignition_2"    )) ||
                        NULL == (global_context->commands.et.e55p.drf_e_1_knb = XPLMFindDataRef("aerobask/engines/knob_start_stop_1")) ||
                        NULL == (global_context->commands.et.e55p.drf_e_2_knb = XPLMFindDataRef("aerobask/engines/knob_start_stop_2")) ||
                        NULL == (global_context->commands.et.e55p.cmd_ig_1_up = XPLMFindCommand("aerobask/engines/ignition_1_up"    )) ||
                        NULL == (global_context->commands.et.e55p.cmd_ig_1_dn = XPLMFindCommand("aerobask/engines/ignition_1_dn"    )) ||
                        NULL == (global_context->commands.et.e55p.cmd_ig_2_up = XPLMFindCommand("aerobask/engines/ignition_2_up"    )) ||
                        NULL == (global_context->commands.et.e55p.cmd_ig_2_dn = XPLMFindCommand("aerobask/engines/ignition_2_dn"    )) ||
                        NULL == (global_context->commands.et.e55p.cmd_e_1_lft = XPLMFindCommand("aerobask/engines/knob_1_lt"        )) ||
                        NULL == (global_context->commands.et.e55p.cmd_e_1_rgt = XPLMFindCommand("aerobask/engines/knob_1_rt"        )) ||
                        NULL == (global_context->commands.et.e55p.cmd_e_2_lft = XPLMFindCommand("aerobask/engines/knob_2_lt"        )) ||
                        NULL == (global_context->commands.et.e55p.cmd_e_2_rgt = XPLMFindCommand("aerobask/engines/knob_2_rt"       )))
                    {
                        xnz_log("[error]: could not self-configure for E55P\n");
                        global_context->commands.xnz_ab = XNZ_AB_ERRR;
                        global_context->commands.xnz_ap = XNZ_AP_ERRR;
                        global_context->commands.xnz_at = XNZ_AT_ERRR;
                        global_context->commands.xnz_bt = XNZ_BT_ERRR;
                        global_context->commands.xnz_et = XNZ_ET_ERRR;
                        global_context->commands.xnz_pb = XNZ_PB_ERRR;
                        global_context->         xnz_tt = XNZ_TT_ERRR;
                    }
                    else
                    {
                        global_context->commands.xnz_ab = XNZ_AB_NONE;
                        global_context->commands.xnz_ap = XNZ_AP_XGFC;
                        global_context->commands.xnz_at = XNZ_AT_NONE; // note: CSC controlled via A/P panel only
                        global_context->commands.xnz_bt = XNZ_BT_XPLM;
                        global_context->commands.xnz_et = XNZ_ET_E55P;
                        global_context->commands.xnz_pb = XNZ_PB_XPLM;
                        global_context->         xnz_tt = XNZ_TT_XPLM;
                    }
                }
                else if (((XPLM_NO_PLUGIN_ID != (pid = XPLMFindPluginBySignature("1-sim Victory G1000"))) && (XPLMIsPluginEnabled(pid))))
                {
                    if (NULL == (global_context->commands.et.evic.drf_fuel_at = XPLMFindDataRef("aerobask/lt_fuel_auto"           )) ||
                        NULL == (global_context->commands.et.evic.cmd_e_1_onn = XPLMFindCommand("sim/fuel/fuel_pump_1_on"         )) ||
                        NULL == (global_context->commands.et.evic.cmd_e_1_off = XPLMFindCommand("sim/fuel/fuel_pump_1_off"        )) ||
                        NULL == (global_context->commands.et.evic.cmd_x_12_rt = XPLMFindCommand("sim/starters/shut_down_1"        )) ||
                        NULL == (global_context->commands.et.evic.cmd_e_2_tog = XPLMFindCommand("aerobask/fuel_auto_toggle"       )) ||
                        NULL == (global_context->commands.et.evic.cmd_x_12_lt = XPLMFindCommand("sim/starters/engage_starter_1"   )) ||
                        NULL == (global_context->commands.et.evic.cmd_m_12_st = XPLMFindCommand("sim/igniters/igniter_contin_on_1")) ||
                        NULL == (global_context->commands.et.evic.cmd_m_12_no = XPLMFindCommand("sim/igniters/igniter_contin_off_1")))
                    {
                        xnz_log("[error]: could not self-configure for EVIC\n");
                        global_context->commands.xnz_ab = XNZ_AB_ERRR;
                        global_context->commands.xnz_ap = XNZ_AP_ERRR;
                        global_context->commands.xnz_at = XNZ_AT_ERRR;
                        global_context->commands.xnz_bt = XNZ_BT_ERRR;
                        global_context->commands.xnz_et = XNZ_ET_ERRR;
                        global_context->commands.xnz_pb = XNZ_PB_ERRR;
                        global_context->         xnz_tt = XNZ_TT_ERRR;
                    }
                    else
                    {
                        global_context->commands.xnz_ab = XNZ_AB_NONE;
                        global_context->commands.xnz_ap = XNZ_AP_XGFC;
                        global_context->commands.xnz_at = XNZ_AT_NONE; // note: CSC controlled via A/P panel only
                        global_context->commands.xnz_bt = XNZ_BT_XPLM;
                        global_context->commands.xnz_et = XNZ_ET_EVIC;
                        global_context->commands.xnz_pb = XNZ_PB_XPLM;
                        global_context->         xnz_tt = XNZ_TT_XPLM;
                    }
                }
                /* must test SASL and Gizmo last */
                else if (((XPLM_NO_PLUGIN_ID != (pid = XPLMFindPluginBySignature("1-sim.sasl"))) && (XPLMIsPluginEnabled(pid))))
                {
                    if (auth_desc_icao)
                    {
                        if (!strncasecmp(auth, "Denis 'ddenn' Krupin", strlen("Denis 'ddenn' Krupin")) &&
                            !strncasecmp(desc, "Bombardier Challenger 300", strlen("Bombardier Challenger 300")))
                        {
                            global_context->commands.xnz_ab = XNZ_AB_NONE;
                            global_context->commands.xnz_ap = XNZ_AP_XPLM;
                            global_context->commands.xnz_at = XNZ_AT_NONE; // note: MACH HOLD controlled via dedicated button only
                            global_context->commands.xnz_bt = XNZ_BT_XPLM;
                            global_context->commands.xnz_et = XNZ_ET_CL30;
                            global_context->commands.xnz_pb = XNZ_PB_XPLM;
                            global_context->         xnz_tt = XNZ_TT_XPLM;
                        }
                        else if (!strncasecmp(auth, "Aerobask", strlen("Aerobask")) ||
                                 !strncasecmp(auth, "Stephane Buon", strlen("Stephane Buon")))
                        {
                            if (!strncasecmp(icao, "EA50", strlen("EA50")))
                            {
                                if (NULL == (global_context->commands.et.ea50.drf_mod_en1 = XPLMFindDataRef("aerobask/eclipse/start_eng_0")) ||
                                    NULL == (global_context->commands.et.ea50.drf_mod_en2 = XPLMFindDataRef("aerobask/eclipse/start_eng_1")))
                                {
                                    xnz_log("[error]: could not self-configure for EA50\n");
                                    global_context->commands.xnz_ab = XNZ_AB_ERRR;
                                    global_context->commands.xnz_ap = XNZ_AP_ERRR;
                                    global_context->commands.xnz_at = XNZ_AT_ERRR;
                                    global_context->commands.xnz_bt = XNZ_BT_ERRR;
                                    global_context->commands.xnz_et = XNZ_ET_ERRR;
                                    global_context->commands.xnz_pb = XNZ_PB_ERRR;
                                    global_context->         xnz_tt = XNZ_TT_ERRR;
                                }
                                else
                                {
                                    global_context->commands.xnz_ab = XNZ_AB_NONE;
                                    global_context->commands.xnz_ap = XNZ_AP_XPLM;
                                    global_context->commands.xnz_at = XNZ_AT_XPLM;
                                    global_context->commands.xnz_bt = XNZ_BT_XPLM;
                                    global_context->commands.xnz_et = XNZ_ET_EA50;
                                    global_context->commands.xnz_pb = XNZ_PB_XPLM;
                                    global_context->         xnz_tt = XNZ_TT_XPLM;
                                }
                            }
                        }
                    }
                }
                else if (((XPLM_NO_PLUGIN_ID != (pid = XPLMFindPluginBySignature("gizmo.x-plugins.com"))) && (XPLMIsPluginEnabled(pid))))
                {
                    if (auth_desc_icao)
                    {
                        if (!strncasecmp(auth, "IXEG", strlen("IXEG")) &&
                            !strncasecmp(desc, "Boeing 737-300", strlen("Boeing 737-300")))
                        {
                            if (NULL == (global_context->commands.et.ix73.drf_e_1_cut = XPLMFindDataRef("ixeg/733/fuel/fuel_start_lever1_act")) ||
                                NULL == (global_context->commands.et.ix73.drf_e_2_cut = XPLMFindDataRef("ixeg/733/fuel/fuel_start_lever2_act")) ||
                                NULL == (global_context->commands.at.comm.cmd_at_disc = XPLMFindCommand("ixeg/733/autopilot/at_disengage")) ||
                                NULL == (global_context->commands.et.ix73.drf_e_1_knb = XPLMFindDataRef("ixeg/733/engine/eng1_start_act")) ||
                                NULL == (global_context->commands.et.ix73.drf_e_2_knb = XPLMFindDataRef("ixeg/733/engine/eng2_start_act")) ||
                                NULL == (global_context->commands.at.comm.cmd_at_toga = XPLMFindCommand("sim/engines/TOGA_power")))
                            {
                                xnz_log("[error]: could not self-configure for IXEG 737 Classic\n");
                                global_context->commands.xnz_ab = XNZ_AB_ERRR;
                                global_context->commands.xnz_ap = XNZ_AP_ERRR;
                                global_context->commands.xnz_at = XNZ_AT_ERRR;
                                global_context->commands.xnz_bt = XNZ_BT_ERRR;
                                global_context->commands.xnz_et = XNZ_ET_ERRR;
                                global_context->commands.xnz_pb = XNZ_PB_ERRR;
                                global_context->         xnz_tt = XNZ_TT_ERRR;
                            }
                            else
                            {
                                global_context->commands.xnz_ab = XNZ_AB_IX33;
//                                global_context->commands.xnz_ap = XNZ_AP_COMM;
                                global_context->commands.xnz_at = XNZ_AT_COMM;
                                global_context->commands.xnz_bt = XNZ_BT_XPLM;
                                global_context->commands.xnz_et = XNZ_ET_IX73;
                                global_context->commands.xnz_pb = XNZ_PB_XPLM;
                                global_context->         xnz_tt = XNZ_TT_XPLM;
                            }
                        }
                    }
                }
                if (global_context->xnz_tt == XNZ_TT_XPLM)
                {
#ifdef PUBLIC_RELEASE_BUILD
                    // TODO: per-aircraft configuration file (initialized by user via custom commands)
#else
                    switch (global_context->commands.xnz_et)
                    {
                        case XNZ_ET_DA62:
                            global_context->zones_info.share[ZONE_CLB] = 0.50f;
                            global_context->zones_info.share[ZONE_FLX] = 0.94f - global_context->zones_info.share[ZONE_CLB]; // safely below maximum continuous thrust
                            global_context->zones_info.share[ZONE_TGA] = 1.00f - global_context->zones_info.share[ZONE_FLX] - global_context->zones_info.share[ZONE_CLB];
                            break;

                        case XNZ_ET_E35L:
                            global_context->zones_info.share[ZONE_CLB] = 0.50f;
                            global_context->zones_info.share[ZONE_FLX] = 0.75f - global_context->zones_info.share[ZONE_CLB];
                            global_context->zones_info.share[ZONE_TGA] = 1.00f - global_context->zones_info.share[ZONE_FLX] - global_context->zones_info.share[ZONE_CLB];
                            break;

                        case XNZ_ET_EA50:
                            global_context->zones_info.share[ZONE_CLB] = 0.500000f;
                            global_context->zones_info.share[ZONE_FLX] = 0.872425f - global_context->zones_info.share[ZONE_CLB]; // ~94% N1 @ PMDY/06
                            global_context->zones_info.share[ZONE_TGA] = 0.943875f - global_context->zones_info.share[ZONE_FLX] - global_context->zones_info.share[ZONE_CLB]; // ~100% N1
                            break;

                        case XNZ_ET_EVIC:
                            global_context->zones_info.share[ZONE_CLB] = 0.500000f;
                            global_context->zones_info.share[ZONE_FLX] = 0.872725f - global_context->zones_info.share[ZONE_CLB]; // ~94% N1 @ PMDY/06
                            global_context->zones_info.share[ZONE_TGA] = 0.971875f - global_context->zones_info.share[ZONE_FLX] - global_context->zones_info.share[ZONE_CLB]; // ~100% N1
                            break;

                        default:
                            break;
                    }
#endif
                }
                xnz_log("determined engine type %d\n",     global_context->commands.xnz_et);
                xnz_log("determined braking type %d\n",    global_context->commands.xnz_bt);
                xnz_log("determined a/brake type %d\n",    global_context->commands.xnz_ab);
                xnz_log("determined a/pilot type %d\n",    global_context->commands.xnz_ap);
                xnz_log("determined throttle type %d\n",   global_context->         xnz_tt);
                xnz_log("determined a/thrust type %d\n",   global_context->commands.xnz_at);
                xnz_log("determined park brake type %d\n", global_context->commands.xnz_pb);

                /* TCA thrust quadrant support */
                if (global_context->idx_throttle_axis_1 < 0) // detection: runs only once
                {
                    size_t size = global_context->i_version_simulator < 11000 ? 100 : 500;
                    for (size_t i = 0; i < size - 1; i++)
                    {
                        int i_stick_ass[2]; XPLMGetDatavi(global_context->i_stick_ass, i_stick_ass, i, 2);
                        if (i_stick_ass[0] == 20 && i_stick_ass[1] == 21)
                        {
                            xnz_log("found throttle 1/2 axes at index (%02zd, %02zd) with assignment (%02d, %02d)\n", i, i + 1, i_stick_ass[0], i_stick_ass[1]);
                            global_context->idx_throttle_axis_1 = i;
                            break;
                        }
                    }
                    if (global_context->idx_throttle_axis_1 >= 0 && 0 /* debug */)
                    {
                        xnz_log("[debug]: throttle_mapping ---------------\n");
                        float last = -2.0f;
                        int detents[3] =
                        {
                            roundf(TCA_IDLE_CTR * 200.0f),
                            roundf(TCA_CLMB_CTR * 200.0f),
                            roundf(TCA_FLEX_CTR * 200.0f),
                        };
                        for (int i = 0; i <= 200; i++)
                        {
                            float input, value = throttle_mapping((input = ((float)i / 200.0f)), global_context->zones_info);
                            if (value < last)
                            {
                                xnz_log("[debug]: non-monotonically increasing throttle mapping, %.4f -> %.4f\n", last, value);
                                exit(-1);
                            }
                            if (i == detents[0] || i == detents[1] || i == detents[2])
                            {
                                xnz_log("[debug]: throttle_mapping ---------------\n");
                            }
                            if (value < 0.0f)
                            {
                                xnz_log("[debug]: throttle_mapping(%.3f) = %.3f\n", input, (last = value));
                            }
                            else
                            {
                                xnz_log("[debug]: throttle_mapping(%.3f) = %.4f\n", input, (last = value));
                            }
                            if (i == detents[0] || i == detents[1] || i == detents[2])
                            {
                                xnz_log("[debug]: throttle_mapping ---------------\n");
                            }
                        }
                        xnz_log("[debug]: throttle_mapping ---------------\n");
                    }
                }
                if (global_context->idx_throttle_axis_1 >= 0) // capture: run every initial aircraft+livery reload
                {
#ifdef PUBLIC_RELEASE_BUILD
                    // TODO: implement A320 API support
                    if (global_context->xnz_tt == XNZ_TT_FF32)
                    {
                        int th_axis_ass[2] = { 20, 21, };
                        xnz_log("[info]: releasing joystick axes (XNZ_TT_FF32)\n");
                        XPLMSetDatavi(global_context->i_stick_ass, th_axis_ass, global_context->idx_throttle_axis_1, 2);
                    }
                    else
#endif
                    {
                        if (global_context->tca_support_enabled)
                        {
                            int no_axis_ass[2] = { 0, 0, };
                            xnz_log("[info]: capturing/re-capturing joystick axes (flight loop enabled)\n");
                            XPLMSetDatavi(global_context->i_stick_ass, no_axis_ass, global_context->idx_throttle_axis_1, 2);
                        }
                    }
                    global_context->skip_idle_overwrite = 0; XPLMSetFlightLoopCallbackInterval(global_context->f_l_th, 1, 1, global_context);
                    xnz_log("setting TCA flight loop callback interval (enabled: %d)\n",
                            global_context->tca_support_enabled == 0);
                    xnz_log("engine type %d beta %d (%d) reverse %d (%d, %f)\n",
                            acf_en_type[0],
                            global_context->acf_has_beta_thrust,
                            XPLMGetDatai(global_context->rev_info[0]),
                            global_context->acft_has_rev_thrust,
                            XPLMGetDatai(global_context->rev_info[1]),
                            XPLMGetDataf(global_context->rev_info[2]));
                }

#ifndef PUBLIC_RELEASE_BUILD
                global_context->i_context_init_done = 1;
                global_context->ice_detect_positive = 0;
                global_context->icecheck_required = 0.0f;
                global_context->show_throttle_all = 0.0f;
                global_context->last_throttle_all = XPLMGetDataf(global_context->f_throttall);
                XPLMSetFlightLoopCallbackInterval(global_context->f_l_cb, 1, 1, global_context);
                return;
#else
                global_context->i_context_init_done = 1;
                return;
#endif
            }
            break;

        default:
            break;
    }
}

static int autothrottle_active(xnz_context *ctx)
{
    switch (ctx->commands.xnz_at)
    {
        case XNZ_AT_FF32:
        case XNZ_AT_TOLI:
            return 0;

        default:
            if (ctx->xnz_tt == XNZ_TT_XPLM)
            {
                return 0 < XPLMGetDatai(ctx->commands.xp.auto_thr_on);
            }
            break;
    }
    return 0;
}

static int servos_on(xnz_context *ctx)
{
    switch (ctx->commands.xnz_ap)
    {
        default:
            break;
    }
    return 0 < XPLMGetDatai(ctx->commands.xp.auto_pil_on);
}

#ifndef PUBLIC_RELEASE_BUILD

static void overlay_show(xnz_context *ctx)
{
    if (ctx)
    {
        if (ctx->overly_position_set == 0)
        {
            /*
             * re-position throttle change/ice detection overlay for a laptop display
             * Aerobask GTN 750 ~788px high: ((1120 - 788 = 332) / 1220) = (83 / 280)
             */
            int outW, outH, outL, yOff; XPLMGetScreenSize(&outW, &outH); yOff = (outH - ((outH * 83) / 280));
            int outT, outR, outB, xOff; XPGetWidgetGeometry(ctx->widgetid[0], &outL, &outT, &outR, &outB);
            xOff = ((outW - (outR - outL)) / 2); outL += xOff; outR += xOff; outT += yOff; outB += yOff;
            XPSetWidgetGeometry(ctx->widgetid[0], outL + 0, outT - 0, outR - 0, outB + 0);
            XPSetWidgetGeometry(ctx->widgetid[1], outL + 7, outT - 7, outR - 7, outB + 7);
            xnz_log("[info]: overlay: W: %d | H: %d | X: %d -> %d | Y: %d -> %d\n", outW, outH, outL, outR, outB, outT);
            ctx->overly_position_set = 1;
        }
        if (XPIsWidgetVisible(ctx->widgetid[0]) == 0) XPShowWidget(ctx->widgetid[0]);
        if (XPIsWidgetVisible(ctx->widgetid[1]) == 0) XPShowWidget(ctx->widgetid[1]);
        return;
    }
    return;
}

static void overlay_hide(xnz_context *ctx)
{
    if (ctx)
    {
        if (XPIsWidgetVisible(ctx->widgetid[1]) != 0) XPHideWidget(ctx->widgetid[1]);
        if (XPIsWidgetVisible(ctx->widgetid[0]) != 0) XPHideWidget(ctx->widgetid[0]);
        return;
    }
    return;
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

        if (ctx->xnz_tt == XNZ_TT_FF32 && !ctx->tt.ff32.api_has_initialized)
        {
            XPLMSendMessageToPlugin(XPLMFindPluginBySignature(XPLM_FF_SIGNATURE), XPLM_FF_MSG_GET_SHARED_INTERFACE, &ctx->tt.ff32.s);
            if (ctx->tt.ff32.s.DataVersion != NULL && ctx->tt.ff32.s.DataAddUpdate != NULL)
            {
                ctx->tt.ff32.id_f32_eng_lever_lt = ctx->tt.ff32.s.ValueIdByName("Aircraft.Cockpit.Pedestal.EngineLever1");
                ctx->tt.ff32.id_f32_eng_lever_rt = ctx->tt.ff32.s.ValueIdByName("Aircraft.Cockpit.Pedestal.EngineLever2");
                if (ctx->tt.ff32.id_f32_eng_lever_lt > -1 && ctx->tt.ff32.id_f32_eng_lever_rt > -1)
                {
                    ctx->tt.ff32.api_has_initialized = 1;
                }
            }
        }

        /* throttle readout (overlay) */
        switch (ctx->xnz_tt)
        {
            case XNZ_TT_FF32:
            {
                if (ctx->tt.ff32.api_has_initialized)
                {
                    // Pedestal.EngineLever*: 0-20-65 (reverse-idle-max)
                    ctx->tt.ff32.s.ValueGet(ctx->tt.ff32.id_f32_eng_lever_lt, &array[0]);
                    ctx->tt.ff32.s.ValueGet(ctx->tt.ff32.id_f32_eng_lever_rt, &array[1]);
                    if ((f_throttall = (((array[0] + array[1]) / 2.0f) - 20.0f) / 45.0f) < 0.0f)
                    {
                        (f_throttall = (((array[0] + array[1]) / 2.0f) - 20.0f) / 20.0f);
                    }
                    break;
                }
            } // fallthrough
            case XNZ_TT_ERRR:
                f_throttall = ctx->last_throttle_all;
                break;

            case XNZ_TT_TOLI:
            {
                XPLMGetDatavf(ctx->tt.toli.f_thr_array, array, 0, 2);
                f_throttall = ((array[0] + array[1]) / 2.0f);
                break;
            }

            case XNZ_TT_TBM9:
            {
                int engn_rng = XPLMGetDatai(ctx->tt.tbm9.engn_rng);
                switch (engn_rng)
                {
                    case 3:
                    case 4:
                    case 5:
                        if (HS_TBM9_IDLE > (f_throttall = XPLMGetDataf(ctx->f_throttall)))
                        {
                            f_throttall = (0.0f - (1.0f - (f_throttall / HS_TBM9_IDLE)));
                            break;
                        }
                        f_throttall = ((f_throttall - HS_TBM9_IDLE) / (1.0f - HS_TBM9_IDLE));
                        break;
                    default:
                        f_throttall = ctx->last_throttle_all;
                        break;
                }
                break;
            }

            default:
            {
                f_throttall = XPLMGetDataf(ctx->f_throttall);
                if (ctx->acft_has_rev_thrust && f_throttall > 0.0f)
                {
                    XPLMGetDatavi(ctx->i_prop_mode, ctx->i_propmode_value, 0, 2);
                    if (ctx->i_propmode_value[0] == 3 || ctx->i_propmode_value[1] == 3)
                    {
                        f_throttall = 0.0f - f_throttall;
                        break;
                    }
                    break;
                }
                break;
            }
        }
        if (fabsf(ctx->last_throttle_all - f_throttall) >= T_SMALL)
        {
            ctx->throttle_did_change = 1;
            ctx->show_throttle_all = 3.0f;
            ctx->last_throttle_all = f_throttall;
            if (ctx->tca_support_enabled &&
                ctx->idx_throttle_axis_1 >= 0 &&
                ctx->skip_idle_overwrite == 0)
            {
                ctx->show_throttle_all = 1.0f;
            }
        }
        if (ctx->show_throttle_all < T_ZERO ||
            ctx->ice_detect_positive ||
            autothrottle_active(ctx))
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
                if (ctx->tca_support_enabled != 0 &&
                    ctx->idx_throttle_axis_1 >= 0 &&
                    ctx->skip_idle_overwrite == 0 &&
                    ctx->i_got_axis_input[0] != 0)
                {
                    snprintf(ctx->overly_txt_buf, 11, "%4.0f %%", f_throttall * 100.0f);
                }
                else if (f_throttall < (0.0f - T_ZERO))
                {
                    snprintf(ctx->overly_txt_buf, 11, "%7.4f", f_throttall);
                }
                else
                {
                    snprintf(ctx->overly_txt_buf, 11, "%7.5f", f_throttall);
                }
                XPSetWidgetDescriptor(ctx->widgetid[1], ctx->overly_txt_buf);
            }
            overlay_show(ctx);
        }
        else if (groundsp > GROUNDSP_KTS_MIN &&
                 groundsp < GROUNDSP_KTS_MAX &&
                 XPLMGetDatai(ctx->ongroundany))
        {
            snprintf(ctx->overly_txt_buf, 9, "%2.0f kts", groundsp);
            XPSetWidgetDescriptor(ctx->widgetid[1], ctx->overly_txt_buf);
            overlay_show(ctx);
        }
        else
        {
            overlay_hide(ctx);
        }

        /* variable nullzones */
        if (servos_on(ctx))
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
        if (ctx->tca_support_enabled == 0 || ctx->idx_throttle_axis_1 < 0)
        {
            ctx->last_throttle_all = f_throttall;
        }
        return (1.0f / 20.0f); // run often
    }
    XPLMDebugString(XNZ_LOG_PREFIX"[error]: callback_hdlr: inRefcon == NULL, disabling callback\n");
    return 0;
}
#endif

static inline float linear_standard(float linear_val)
{
    return linear_val;
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

static inline float jitter_protection(float input)
{
    return roundf(input / T_SMALL) * T_SMALL;
}

static inline float throttle_mapping_abe55p(float input, thrust_zones z)
{
    if (input > z.min[ZONE_TGA])
    {
        return 0.8356f; // TO
    }
//  if (input > z.max[ZONE_FLX])
//  {
//      return 0.6986f; // CLB
//  }
    if (input > z.min[ZONE_FLX])
    {
        return 0.6986f; // CLB
    }
    if (input > z.max[ZONE_CLB])
    {
        return 0.5479f; // CRZ
    }
    if (input > z.min[ZONE_CLB])
    {
        float t = (input - z.min[ZONE_CLB]) / z.len[ZONE_CLB];
        return jitter_protection(0.5205f * linear_standard(t));
    }
    return 0.0f; // no reverse
}

static inline float throttle_mapping_ddcl30(float input, thrust_zones z)
{
    if (input > z.min[ZONE_TGA])
    {
        return 2.8f / 3.0f; // TO
    }
//  if (input > z.max[ZONE_FLX])
//  {
//      return 2.6f / 3.0f; // CL
//  }
    if (input > z.min[ZONE_FLX])
    {
        return 2.6f / 3.0f; // CL
    }
    if (input > z.max[ZONE_CLB])
    {
        return 2.5f / 3.0f; // CR
    }
    if (input > z.min[ZONE_CLB])
    {
        float t = (input - z.min[ZONE_CLB]) / z.len[ZONE_CLB];
        return jitter_protection(2.4f / 3.0f * linear_standard(t));
    }
    if (input > z.max[ZONE_REV])
    {
        return 0.0f;
    }
#ifdef PUBLIC_RELEASE_BUILD
    if (input < z.min[ZONE_REV])
    {
        return -1.0f; // max reverse
    }
    float t = (input - z.min[ZONE_REV]) / z.len[ZONE_REV];
    return jitter_protection(non_linear_standard(t)-1.0f); // non-linear reverse range
#else
    if (input < (z.min[ZONE_REV] + (z.len[ZONE_REV] / 20.0f)))
    {
        return -1.0f; // max reverse
    }
    return -0.1f; // idle reverse
#endif
}

static inline float throttle_mapping_toliss(float input, thrust_zones z)
{
    if (input > z.min[ZONE_TGA])
    {
        return 1.0f; // TO/GA
    }
//  if (input > z.max[ZONE_FLX])
//  {
//      return 0.87f; // FLEX
//  }
    if (input > z.min[ZONE_FLX])
    {
        return 0.87f; // FLEX
    }
    if (input > z.max[ZONE_CLB])
    {
        return 0.69f; // CLB
    }
    if (input > z.min[ZONE_CLB])
    {
        float t = (input - z.min[ZONE_CLB]) / z.len[ZONE_CLB];
        return jitter_protection(0.68f * linear_standard(t));
    }
    if (input > z.max[ZONE_REV])
    {
        return 0.0f;
    }
#ifdef PUBLIC_RELEASE_BUILD
    if (input < z.min[ZONE_REV])
    {
        return -1.0f; // max reverse
    }
    float t = (input - z.min[ZONE_REV]) / z.len[ZONE_REV];
    return jitter_protection(non_linear_standard(t)-1.0f); // non-linear reverse range
#else
    if (input < (z.min[ZONE_REV] + (z.len[ZONE_REV] / 20.0f)))
    {
        return -1.0f; // max reverse
    }
    return -0.1f; // idle reverse
#endif
}

static inline float throttle_mapping_nl_rev(float input, thrust_zones z)
{
    if (input > z.max[ZONE_TGA])
    {
        return 1.00f; // max forward
    }
    if (input > z.min[ZONE_TGA])
    {
        float t = ((input - z.min[ZONE_TGA]) / z.len[ZONE_TGA]);
        return jitter_protection(z.share[ZONE_TGA] * linear_standard(t) + z.share[ZONE_FLX] + z.share[ZONE_CLB]);
    }
    if (input > z.max[ZONE_FLX])
    {
        return z.share[ZONE_FLX] + z.share[ZONE_CLB];
    }
    if (input > z.min[ZONE_FLX])
    {
        float t = ((input - z.min[ZONE_FLX]) / z.len[ZONE_FLX]);
        return jitter_protection(z.share[ZONE_FLX] * linear_standard(t) + z.share[ZONE_CLB]);
    }
    if (input > z.max[ZONE_CLB])
    {
        return z.share[ZONE_CLB];
    }
    if (input > z.min[ZONE_CLB])
    {
        float t = (input - z.min[ZONE_CLB]) / z.len[ZONE_CLB];
        return jitter_protection(z.share[ZONE_CLB] * linear_standard(t));
    }
    if (input > z.max[ZONE_REV])
    {
        return 0.0f;
    }
    if (input < z.min[ZONE_REV])
    {
        return -1.0f; // max reverse
    }
    float t = (input - z.min[ZONE_REV]) / z.len[ZONE_REV];
    return jitter_protection(non_linear_standard(t)-1.0f); // beta and reverse ranges
}

static inline float throttle_mapping_w_rev(float input, thrust_zones z)
{
    if (input > z.max[ZONE_TGA])
    {
        return z.share[ZONE_TGA] + z.share[ZONE_FLX] + z.share[ZONE_CLB]; // max forward (may be less than 1.0f)
    }
    if (input > z.min[ZONE_TGA])
    {
        float t = ((input - z.min[ZONE_TGA]) / z.len[ZONE_TGA]);
        return jitter_protection(z.share[ZONE_TGA] * linear_standard(t) + z.share[ZONE_FLX] + z.share[ZONE_CLB]);
    }
    if (input > z.max[ZONE_FLX])
    {
        return z.share[ZONE_FLX] + z.share[ZONE_CLB];
    }
    if (input > z.min[ZONE_FLX])
    {
        float t = ((input - z.min[ZONE_FLX]) / z.len[ZONE_FLX]);
        return jitter_protection(z.share[ZONE_FLX] * linear_standard(t) + z.share[ZONE_CLB]);
    }
    if (input > z.max[ZONE_CLB])
    {
        return z.share[ZONE_CLB];
    }
    if (input > z.min[ZONE_CLB])
    {
        float t = (input - z.min[ZONE_CLB]) / z.len[ZONE_CLB];
        return jitter_protection(z.share[ZONE_CLB] * linear_standard(t));
    }
    if (input > z.max[ZONE_REV])
    {
        return 0.0f;
    }
#ifdef PUBLIC_RELEASE_BUILD
    if (input < z.min[ZONE_REV])
    {
        return -1.0f; // max reverse
    }
    float t = (input - z.min[ZONE_REV]) / z.len[ZONE_REV];
    return jitter_protection(non_linear_standard(t)-1.0f); // non-linear reverse range
#else
    if (input < (z.min[ZONE_REV] + (z.len[ZONE_REV] / 20.0f)))
    {
        return -1.0f; // max reverse
    }
    return -0.1f; // idle reverse
#endif
}

static inline float throttle_mapping(float input, thrust_zones z)
{
    if (input > z.max[ZONE_TGA])
    {
        return z.share[ZONE_TGA] + z.share[ZONE_FLX] + z.share[ZONE_CLB]; // max forward (may be less than 1.0f)
    }
    if (input > z.min[ZONE_TGA])
    {
        float t = ((input - z.min[ZONE_TGA]) / z.len[ZONE_TGA]);
        return jitter_protection(z.share[ZONE_TGA] * linear_standard(t) + z.share[ZONE_FLX] + z.share[ZONE_CLB]);
    }
    if (input > z.max[ZONE_FLX])
    {
        return z.share[ZONE_FLX] + z.share[ZONE_CLB];
    }
    if (input > z.min[ZONE_FLX])
    {
        float t = ((input - z.min[ZONE_FLX]) / z.len[ZONE_FLX]);
        return jitter_protection(z.share[ZONE_FLX] * linear_standard(t) + z.share[ZONE_CLB]);
    }
    if (input > z.max[ZONE_CLB])
    {
        return z.share[ZONE_CLB];
    }
    if (input > z.min[ZONE_CLB])
    {
        float t = (input - z.min[ZONE_CLB]) / z.len[ZONE_CLB];
        return jitter_protection(z.share[ZONE_CLB] * linear_standard(t));
    }
    return 0.0f; // no reverse
}

static int fwd_beta_rev_thrust_for_index(xnz_context *ctx, float f_stick_val[1], int i)
{
    if (ctx->i_propmode_value[i] < 1 || // probably feathered
        ctx->i_propmode_value[i] > 3)  // should never happen
    {
        f_stick_val[0] = 0.0f;
        return 0;
    }
    if ((T_ZERO + f_stick_val[0]) < 0.0f) // reverse or beta range
    {
        if ((T_ZERO + f_stick_val[0]) >= -0.5f) // beta range or idle reverse
        {
            if (ctx->acf_has_beta_thrust) // TODO: implement
            {
                f_stick_val[0] = 0.0f;
                return 0;
            }
        }
        if (ctx->acft_has_rev_thrust)
        {
            if (ctx->i_propmode_value[i] != 3)
            {
                XPLMCommandOnce(ctx->revto[i]);
                return 1;
            }
            f_stick_val[0] = fabsf(f_stick_val[0]);
            return 0;
        }
        f_stick_val[0] = 0.0f;
        return 0;
    }
    if (ctx->acf_has_beta_thrust && ctx->i_propmode_value[i] == 2)
    {
        XPLMCommandOnce(ctx->betto[i]);
        return 1;
    }
    if (ctx->acft_has_rev_thrust && ctx->i_propmode_value[i] == 3)
    {
        XPLMCommandOnce(ctx->revto[i]);
        return 1;
    }
    return 0;
}
static int fwd_beta_rev_thrust(xnz_context *ctx, float f_stick_val[2])
{
    switch (ctx->xnz_tt)
    {
        case XNZ_TT_TOLI:
            return 0; // handled by ToLiSS plugin based on positive/begative value of f_stick_val

        case XNZ_TT_TBM9:
            if ((T_ZERO + f_stick_val[0]) < 0.0f)
            {
                if (XPLMGetDatai(ctx->tt.tbm9.engn_rng) == 3)
                {
                    XPLMSetDataf(ctx->f_throttall, HS_TBM9_IDLE - T_ZERO); // flight -> taxi range
                    XPLMCommandOnce(ctx->revto[8]); // lift gate (engn_rng goes from 3 to 4)
                    return 1;
                }
                return 0;
            }
            if (XPLMGetDatai(ctx->tt.tbm9.engn_rng) > 3)
            {
                XPLMSetDataf(ctx->f_throttall, HS_TBM9_IDLE + T_ZERO); // beta/reverse -> flight idle
                return 1;
            }
            return 0;

        default:
            break;
    }
    if (ctx->arcrft_engine_count == 2)
    {
        int l = fwd_beta_rev_thrust_for_index(ctx, &f_stick_val[0], 0);
        int r = fwd_beta_rev_thrust_for_index(ctx, &f_stick_val[1], 1);
        return (l + r) > 0;
    }
    else
    {
        int toggled_count = fwd_beta_rev_thrust_for_index(ctx, &f_stick_val[0], 0);
        for (int i = 1; i < ctx->arcrft_engine_count; i++)
        {
            toggled_count += fwd_beta_rev_thrust_for_index(ctx, &f_stick_val[0], i);
        }
        return toggled_count > 0;
    }
    return 0;
}

static int skip_idle_overwrite(xnz_context *ctx, float f_stick_val[2])
{
    if (0.0f == f_stick_val[0] && (ctx->arcrft_engine_count < 2 || f_stick_val[1] == 0.0f))
    {
        /*
         * as soon as the user overrides the throttle via commands, then
         * f_throttall is no longer zero, so we need the return 1 clause
         * before we check the latter, else we will keep overwriting any
         * throttle ratio modifications triggered by the user; hence, we
         * must have it here early, otherwise said override is pointless
         */
        if (ctx->skip_idle_overwrite > 9)
        {
            return 1;
        }
        float f_simul_val[2];
        switch (ctx->xnz_tt)
        {
            case XNZ_TT_FF32: // TODO: implement
                return ctx->skip_idle_overwrite = 0;

            case XNZ_TT_TBM9:
                if (fabsf((f_simul_val[0] = (XPLMGetDataf(ctx->f_throttall) - HS_TBM9_IDLE)) - 0.0f) < T_ZERO)
                {
                    f_simul_val[0] = 0.0f;
                }
                break;

            case XNZ_TT_TOLI:
                XPLMGetDatavf(ctx->tt.toli.f_thr_array, f_simul_val, 0, 2);
                break;

            default:
                f_simul_val[0] = f_simul_val[1] = XPLMGetDataf(ctx->f_throttall);
                break;
        }
        if (0.0f == f_simul_val[0] && (ctx->arcrft_engine_count < 2 || f_simul_val[1] == 0.0f))
        {
            for (int i = 0; i < ctx->arcrft_engine_count; i++)
            {
                switch (ctx->xnz_tt)
                {
                    case XNZ_TT_FF32: // TODO: implement
                        return ctx->skip_idle_overwrite = 0;

                    case XNZ_TT_TBM9:
                        if (XPLMGetDatai(ctx->tt.tbm9.engn_rng) != 3)
                        {
                            return ctx->skip_idle_overwrite = 0;
                        }
                        break;

                    default:
                        if (ctx->i_propmode_value[i] != 1)
                        {
                            return ctx->skip_idle_overwrite = 0;
                        }
                        break;
                }
                continue;
            }
            ctx->skip_idle_overwrite++;
            return 0;
        }
        return ctx->skip_idle_overwrite = 0;
    }
    return ctx->skip_idle_overwrite = 0;
}

static void throttle_axes(xnz_context *ctx)
{
    float f_stick_val[2], avrg_throttle_out;
    XPLMGetDatavf(ctx->f_stick_val, f_stick_val, ctx->idx_throttle_axis_1, 2);
    XPLMGetDatavi(ctx->i_prop_mode, ctx->i_propmode_value, 0, ctx->arcrft_engine_count);
    if (autothrottle_active(ctx))
    {
        ctx->avrg_throttle_inn = (1.0f - ((f_stick_val[0] + f_stick_val[1]) / 2.0f));
        ctx->avrg_throttle_out = XNZ_THOUT_AT;
        return;
    }
    if (ctx->i_got_axis_input[0] == 0)
    {
        if (f_stick_val[0] < TCA_DEADBAND || f_stick_val[1] < TCA_DEADBAND)
        {
            ctx->avrg_throttle_out = XPLMGetDataf(ctx->f_throttall);
            ctx->avrg_throttle_inn = XNZ_THINN_NO;
            return;
        }
        ctx->avrg_throttle_inn = (1.0f - ((f_stick_val[0] + f_stick_val[1]) / 2.0f));
        ctx->i_got_axis_input[0] = 1;
    }
    else
    {
        ctx->avrg_throttle_inn = (1.0f - ((f_stick_val[0] + f_stick_val[1]) / 2.0f));
    }
#ifdef PUBLIC_RELEASE_BUILD
    if (ctx->arcrft_engine_count != 2 || fabsf(f_stick_val[0] - f_stick_val[1]) < TCA_SYNCBAND)
#endif
    {
        f_stick_val[0] = f_stick_val[1] = ((f_stick_val[0] + f_stick_val[1]) / 2.0f); // cannot re-use ctx->avrg_throttle_inn (inverted)
    }
    switch (ctx->xnz_tt)
    {
        case XNZ_TT_FF32: // TODO: implement
            return;

        case XNZ_TT_TOLI:
            f_stick_val[0] = throttle_mapping_toliss(1.0f - f_stick_val[0], ctx->zones_info);
            f_stick_val[1] = throttle_mapping_toliss(1.0f - f_stick_val[1], ctx->zones_info);
            break;

        case XNZ_TT_TBM9:
            if (XPLMGetDatai(ctx->tt.tbm9.engn_rng) < 3)
            {
                XPLMSetDataf(ctx->f_throttall, HS_TBM9_IDLE);
                return;
            }
            f_stick_val[0] = throttle_mapping_nl_rev(1.0f - f_stick_val[0], ctx->zones_info);
            break;

        default:
            switch (ctx->commands.xnz_et)
            {
#ifndef PUBLIC_RELEASE_BUILD
                case XNZ_ET_CL30:
                    f_stick_val[0] = throttle_mapping_ddcl30(1.0f - f_stick_val[0], ctx->zones_info);
                    f_stick_val[1] = throttle_mapping_ddcl30(1.0f - f_stick_val[1], ctx->zones_info);
                    break;

                case XNZ_ET_E55P:
                    f_stick_val[0] = throttle_mapping_abe55p(1.0f - f_stick_val[0], ctx->zones_info);
                    f_stick_val[1] = throttle_mapping_abe55p(1.0f - f_stick_val[1], ctx->zones_info);
                    break;
#endif
                case XNZ_ET_XPTP:
                case XNZ_ET_RPTP:
                    if (ctx->acft_has_rev_thrust)
                    {
                        f_stick_val[0] = throttle_mapping_nl_rev(1.0f - f_stick_val[0], ctx->zones_info);
                        f_stick_val[1] = throttle_mapping_nl_rev(1.0f - f_stick_val[1], ctx->zones_info);
                        break;
                    } // fall through
                default:
                    if (ctx->acft_has_rev_thrust)
                    {
                        f_stick_val[0] = throttle_mapping_w_rev(1.0f - f_stick_val[0], ctx->zones_info);
                        f_stick_val[1] = throttle_mapping_w_rev(1.0f - f_stick_val[1], ctx->zones_info);
                        break;
                    }
                    f_stick_val[0] = throttle_mapping(1.0f - f_stick_val[0], ctx->zones_info);
                    f_stick_val[1] = throttle_mapping(1.0f - f_stick_val[1], ctx->zones_info);
                    break;
            }
            break;
    }
    if (skip_idle_overwrite(ctx, f_stick_val))
    {
        ctx->avrg_throttle_out = XNZ_THOUT_SK;
        return;
    }
    if (ctx->xnz_tt != XNZ_TT_TBM9)
    {
        // store before sign possibly changed by fwd_beta_rev_thrust()
        // but only set variable later (if actually writing th. ratio)
        avrg_throttle_out = ((f_stick_val[0] + f_stick_val[1]) / 2.0f);
    }
    if (fwd_beta_rev_thrust(ctx, f_stick_val))
    {
        return;
    }
    switch (ctx->xnz_tt)
    {
        case XNZ_TT_TOLI:
            XPLMSetDatavf(ctx->tt.toli.f_thr_array, f_stick_val, 0, 2);
            ctx->avrg_throttle_out = avrg_throttle_out;
            return;

        case XNZ_TT_TBM9:
            if (XPLMGetDatai(ctx->tt.tbm9.engn_rng) > 3)
            {
                ctx->avrg_throttle_out = (HS_TBM9_IDLE + ((HS_TBM9_IDLE) * f_stick_val[0]));
                XPLMSetDataf(ctx->f_throttall, ctx->avrg_throttle_out);
                return; // beta or reverse range
            }
            ctx->avrg_throttle_out = (HS_TBM9_IDLE + ((1.0f - HS_TBM9_IDLE) * f_stick_val[0]));
            XPLMSetDataf(ctx->f_throttall, ctx->avrg_throttle_out);
            return; // flight range

        default:
            ctx->avrg_throttle_out = avrg_throttle_out;
            break;
    }
    if (ctx->arcrft_engine_count == 2)
    {
        XPLMSetDatavf(ctx->f_thr_array, f_stick_val, 0, 2);
        return;
    }
    XPLMSetDataf(ctx->f_throttall, f_stick_val[0]); // sign may differ from avrg_throttle_out
    return;
}

static float axes_hdlr_fnc(float inElapsedSinceLastCall,
                           float inElapsedTimeSinceLastFlightLoop,
                           int   inCounter,
                           void *inRefcon)
{
    if (inRefcon)
    {
        /* shall we be doing something? */
        if (((xnz_context*)inRefcon)->tca_support_enabled == 0)
        {
            return (1.0f / 20.0f);
        }
        throttle_axes(inRefcon);
        return (1.0f / 20.0f);
    }
    XPLMDebugString(XNZ_LOG_PREFIX"[error]: callback_hdlr: inRefcon == NULL, disabling callback\n");
    return 0;
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
                XPLMMenuCheck s; XPLMCheckMenuItemState(ctx->id_th_on_off, ctx->id_menu_item_on_off, &s);
                if (s == xplm_Menu_Checked)
                {
                    XPLMCheckMenuItem(ctx->id_th_on_off, ctx->id_menu_item_on_off, xplm_Menu_NoCheck);
                    xnz_log("[info]: menu: disabling TCA flight loop callback\n");
                    global_context->tca_support_enabled = 0;
                    global_context->skip_idle_overwrite = 0;
#ifdef PUBLIC_RELEASE_BUILD
                    if (ctx->idx_throttle_axis_1 >= 0)
                    {
                        int th_axis_ass[2] = { 20, 21, };
                        xnz_log("[info]: releasing joystick axes (flight loop disabled)\n");
                        XPLMSetDatavi(ctx->i_stick_ass, th_axis_ass, ctx->idx_throttle_axis_1, 2);
                    }
#endif
                    return;
                }
                XPLMCheckMenuItem(ctx->id_th_on_off, ctx->id_menu_item_on_off, xplm_Menu_Checked);
                xnz_log("[info]: menu: enabling TCA flight loop callback\n");
                global_context->tca_support_enabled = 1;
                global_context->skip_idle_overwrite = 0;
#ifdef PUBLIC_RELEASE_BUILD
                if (ctx->idx_throttle_axis_1 >= 0)
                {
                    int no_axis_ass[2] = { 0, 0, };
                    XPLMSetDatavi(ctx->i_stick_ass, no_axis_ass, ctx->idx_throttle_axis_1, 2);
                    xnz_log("[info]: menu: re-capturing joystick axes (flight loop enabled)\n");
                }
#endif
                return;
            }
            return;
        }
        return;
    }
    return;
}

static int parking_brake_get(xnz_cmd_context *commands)
{
    if (commands)
    {
        switch (commands->xnz_pb)
        {
            case XNZ_PB_FF35:
                return !XPLMGetDatai(commands->pb.ff35.pbrak_offon);

            case XNZ_PB_TO32:
                return !!XPLMGetDatai(commands->pb.to32.pbrak_onoff);

            case XNZ_PB_TBM9:
                return 1.0f <= XPLMGetDataf(commands->pb.tbm9.pbrak_ratio);

            case XNZ_PB_XPLM:
                if (commands->xp.pbrak_onoff < 0)
                {
                    commands->xp.pbrak_onoff = 1.0f <= XPLMGetDataf(commands->xp.pbrak_ratio);
                }
                return !!commands->xp.pbrak_onoff;

            case XNZ_PB_ERRR:
            default:
                return -1;
        }
        return -1;
    }
    return -1;
}

static int parking_brake_set(xnz_cmd_context *commands, int set)
{
    if (commands)
    {
        switch (commands->xnz_pb)
        {
            case XNZ_PB_FF35:
                if (set)
                {
                    XPLMSetDatai(commands->pb.ff35.pbrak_offon, 0);
                    return 0;
                }
                XPLMSetDatai(commands->pb.ff35.pbrak_offon, 1);
                return 0;

            case XNZ_PB_TO32:
                if (set)
                {
                    XPLMSetDatai(commands->pb.to32.pbrak_onoff, 1);
                    return 0;
                }
                XPLMSetDatai(commands->pb.to32.pbrak_onoff, 0);
                return 0;

            case XNZ_PB_TBM9:
                if (set)
                {
                    if (commands->xnz_bt == XNZ_BT_TBM9)
                    {
                        float barray[4] = { 1.0f, 1.0f, 0.0f, 0.0f, };
                        XPLMSetDatavf(commands->bt.tbm9.rbrak_array, &barray[0], 0, 2);
                        XPLMSetDataf(commands->pb.tbm9.pbrak_ratio, 1.0f);
//                      XPLMSetDatavf(commands->bt.tbm9.rbrak_array, &barray[2], 0, 2); // tested -- resetting to zero immediately doesn't work
                        return 0;
                    }
                    XPLMSetDataf(commands->pb.tbm9.pbrak_ratio, 1.0f);
                    return 0;
                }
                if (commands->xnz_bt == XNZ_BT_TBM9)
                {
                    /*
                     * just in case we unset parking brake right after setting it,
                     * w/out calling e.g. brake hold regular or similar in between
                     */
                    float barray[4] = { 1.0f, 1.0f, 0.0f, 0.0f, };
                    XPLMSetDatavf(commands->bt.tbm9.rbrak_array, &barray[2], 0, 2);
                    XPLMSetDataf(commands->pb.tbm9.pbrak_ratio, 0.0f);
                    return 0;
                }
                XPLMSetDataf(commands->pb.tbm9.pbrak_ratio, 0.0f);
                return 0;

            case XNZ_PB_XPLM:
                if (set)
                {
                    XPLMSetDataf(commands->xp.pbrak_ratio, 1.0f);
                    commands->xp.pbrak_onoff = 1;
                    return 0;
                }
                XPLMSetDataf(commands->xp.pbrak_ratio, 0.0f);
                commands->xp.pbrak_onoff = 0;
                return 0;

            case XNZ_PB_ERRR:
            default:
                return -1;
        }
        return -1;
    }
    return -1;
}

static int chandler_rgb_pkb(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandBegin)
    {
        if ((((xnz_cmd_context*)inRefcon)->xp.pbrakonoff2 = parking_brake_get(inRefcon))) // if set, release parking brake
        {
            chandler_pkb_off(((xnz_cmd_context*)inRefcon)->cmd_pkb_off, xplm_CommandEnd, inRefcon); // use command handler for callouts
        }
        return chandler_rgb_hld(((xnz_cmd_context*)inRefcon)->cmd_rgb_hld, xplm_CommandBegin, inRefcon);
    }
    if (inPhase == xplm_CommandEnd)
    {
        if (0 == chandler_rgb_hld(((xnz_cmd_context*)inRefcon)->cmd_rgb_hld, xplm_CommandEnd, inRefcon))
        {
            if (((xnz_cmd_context*)inRefcon)->xp.pbrakonoff2 == 0) // if was NOT set on command begin, set parking brake
            {
                if (GROUNDSP_KTS_MIN > MPS2KTS(XPLMGetDataf(((xnz_cmd_context*)inRefcon)->xp.groundspeed))) // only when groundspeed is very low
                {
                    return chandler_pkb_onn(((xnz_cmd_context*)inRefcon)->cmd_pkb_onn, xplm_CommandEnd, inRefcon); // use command handler for callouts
                }
                return 0;
            }
            return 0;
        }
        return 0;
    }
    return 0;
}
static int chandler_rgb_hld(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    float barray[2];
    switch (inPhase)
    {
        case xplm_CommandBegin:
        {
            switch (((xnz_cmd_context*)inRefcon)->xnz_bt)
            {
                case XNZ_BT_XPLM:
                case XNZ_BT_PKBR:
                case XNZ_BT_SIMC:
                    ((xnz_cmd_context*)inRefcon)->xp.pbrak_onoff = parking_brake_get(inRefcon);
                    return 0; // TODO: autobrake -> manual braking

                case XNZ_BT_COMM: // deferred initialization
                    if (((xnz_cmd_context*)inRefcon)->bt.comm.cmd_mxb_hld == NULL)
                    {
                        ((xnz_cmd_context*)inRefcon)->bt.comm.cmd_mxb_hld = XPLMFindCommand(((xnz_cmd_context*)inRefcon)->bt.comm.commnd_max_hld);
                        ((xnz_cmd_context*)inRefcon)->bt.comm.cmd_mxb_hld = XPLMFindCommand(((xnz_cmd_context*)inRefcon)->bt.comm.commnd_rgb_hld);
                        ((xnz_cmd_context*)inRefcon)->xp.pbrak_onoff = parking_brake_get(inRefcon);
                        ((xnz_cmd_context*)inRefcon)->bt.comm.cmd_current = NULL;
                        return 0;
                    }
                    if (((xnz_cmd_context*)inRefcon)->bt.comm.cmd_current == ((xnz_cmd_context*)inRefcon)->bt.comm.cmd_mxb_hld ||
                        ((xnz_cmd_context*)inRefcon)->bt.comm.cmd_current == ((xnz_cmd_context*)inRefcon)->bt.comm.cmd_rgb_hld)
                    {
                        ((xnz_cmd_context*)inRefcon)->xp.pbrak_onoff = parking_brake_get(inRefcon);
                        XPLMCommandEnd(((xnz_cmd_context*)inRefcon)->bt.comm.cmd_current);
                        ((xnz_cmd_context*)inRefcon)->bt.comm.cmd_current = NULL;
                        return 0;
                    }
                    ((xnz_cmd_context*)inRefcon)->xp.pbrak_onoff = parking_brake_get(inRefcon);
                    ((xnz_cmd_context*)inRefcon)->bt.comm.cmd_current = NULL;
                    return 0; // TODO: autobrake -> manual braking

                case XNZ_BT_FF35:
                    ((xnz_cmd_context*)inRefcon)->bt.ff35.pbrak_onoff = parking_brake_get(inRefcon);
                    return 0;

                case XNZ_BT_TO32:
                case XNZ_BT_TBM9:
                case XNZ_BT_ERRR:
                default:
                    return 0;
            }
            return 0;
        }

        case xplm_CommandContinue:
        {
            float groundspeed = XPLMGetDataf(((xnz_cmd_context*)inRefcon)->xp.groundspeed);
            int speed = (50.0 < MPS2KTS(groundspeed)) ? 2 : (15.0f < MPS2KTS(groundspeed)) ? 1: 0;
            switch (((xnz_cmd_context*)inRefcon)->xnz_bt)
            {
                case XNZ_BT_XPLM:
                    switch (speed)
                    {
                        case 2:
                            XPLMSetDataf(((xnz_cmd_context*)inRefcon)->xp.l_rgb_ratio, 0.9f);
                            XPLMSetDataf(((xnz_cmd_context*)inRefcon)->xp.r_rgb_ratio, 0.9f);
                            return 0;
                        case 1:
                            XPLMSetDataf(((xnz_cmd_context*)inRefcon)->xp.l_rgb_ratio, 0.6f);
                            XPLMSetDataf(((xnz_cmd_context*)inRefcon)->xp.r_rgb_ratio, 0.6f);
                            return 0;
                        default:
                            XPLMSetDataf(((xnz_cmd_context*)inRefcon)->xp.l_rgb_ratio, 0.3f);
                            XPLMSetDataf(((xnz_cmd_context*)inRefcon)->xp.r_rgb_ratio, 0.3f);
                            return 0;
                    }

                case XNZ_BT_COMM:
                    switch (speed)
                    {
                        case 2:
                            if (((xnz_cmd_context*)inRefcon)->bt.comm.cmd_mxb_hld)
                            {
                                if (((xnz_cmd_context*)inRefcon)->bt.comm.cmd_current == NULL)
                                {
                                    XPLMCommandBegin((((xnz_cmd_context*)inRefcon)->bt.comm.cmd_current = ((xnz_cmd_context*)inRefcon)->bt.comm.cmd_mxb_hld));
                                    return 0;
                                }
                                if (((xnz_cmd_context*)inRefcon)->bt.comm.cmd_current == ((xnz_cmd_context*)inRefcon)->bt.comm.cmd_mxb_hld)
                                {
                                    return 0;
                                }
                                if (((xnz_cmd_context*)inRefcon)->bt.comm.cmd_current == ((xnz_cmd_context*)inRefcon)->bt.comm.cmd_rgb_hld)
                                {
                                    XPLMCommandEnd(((xnz_cmd_context*)inRefcon)->bt.comm.cmd_current);
                                    XPLMCommandBegin((((xnz_cmd_context*)inRefcon)->bt.comm.cmd_current = ((xnz_cmd_context*)inRefcon)->bt.comm.cmd_mxb_hld));
                                    return 0;
                                }
                                ((xnz_cmd_context*)inRefcon)->bt.comm.cmd_current = NULL;
                                return 0;
                            }
                            else
                            {
                                ((xnz_cmd_context*)inRefcon)->bt.comm.cmd_current = NULL;
                                return 0;
                            }
                        default:
                            if (((xnz_cmd_context*)inRefcon)->bt.comm.cmd_rgb_hld)
                            {
                                if (((xnz_cmd_context*)inRefcon)->bt.comm.cmd_current == NULL)
                                {
                                    XPLMCommandBegin((((xnz_cmd_context*)inRefcon)->bt.comm.cmd_current = ((xnz_cmd_context*)inRefcon)->bt.comm.cmd_rgb_hld));
                                    return 0;
                                }
                                if (((xnz_cmd_context*)inRefcon)->bt.comm.cmd_current == ((xnz_cmd_context*)inRefcon)->bt.comm.cmd_rgb_hld)
                                {
                                    return 0;
                                }
                                if (((xnz_cmd_context*)inRefcon)->bt.comm.cmd_current == ((xnz_cmd_context*)inRefcon)->bt.comm.cmd_mxb_hld)
                                {
                                    XPLMCommandEnd(((xnz_cmd_context*)inRefcon)->bt.comm.cmd_current);
                                    XPLMCommandBegin((((xnz_cmd_context*)inRefcon)->bt.comm.cmd_current = ((xnz_cmd_context*)inRefcon)->bt.comm.cmd_rgb_hld));
                                    return 0;
                                }
                                ((xnz_cmd_context*)inRefcon)->bt.comm.cmd_current = NULL;
                                return 0;
                            }
                            else
                            {
                                ((xnz_cmd_context*)inRefcon)->bt.comm.cmd_current = NULL;
                                return 0;
                            }
                    }

                case XNZ_BT_PKBR:
                    if (((xnz_cmd_context*)inRefcon)->xnz_pb == XNZ_PB_XPLM && parking_brake_get(inRefcon) == 1)
                    {
                        return 0;
                    }
                    switch (speed)
                    {
                        case 2:
                            XPLMSetDataf(((xnz_cmd_context*)inRefcon)->xp.pbrak_ratio, 0.9f);
                            return 0;
                        case 1:
                            XPLMSetDataf(((xnz_cmd_context*)inRefcon)->xp.pbrak_ratio, 0.6f);
                            return 0;
                        default:
                            XPLMSetDataf(((xnz_cmd_context*)inRefcon)->xp.pbrak_ratio, 0.3f);
                            return 0;
                    }

                case XNZ_BT_SIMC:
                    if (((xnz_cmd_context*)inRefcon)->xnz_pb == XNZ_PB_XPLM && parking_brake_get(inRefcon) == 1)
                    {
                        return 0;
                    }
                    switch (speed)
                    {
                        case 2:
                            XPLMSetDataf(((xnz_cmd_context*)inRefcon)->xp.pbrak_ratio, 1.0f);
                            return 0;
                        case 1:
                            XPLMSetDataf(((xnz_cmd_context*)inRefcon)->xp.pbrak_ratio, .75f);
                            return 0;
                        default:
                            XPLMSetDataf(((xnz_cmd_context*)inRefcon)->xp.pbrak_ratio, 0.5f);
                            return 0;
                    }

                case XNZ_BT_TO32:
                    switch (speed)
                    {
                        case 2:
                            XPLMSetDataf(((xnz_cmd_context*)inRefcon)->bt.to32.l_rgb_ratio, 0.9f);
                            XPLMSetDataf(((xnz_cmd_context*)inRefcon)->bt.to32.r_rgb_ratio, 0.9f);
                            return 0;
                        case 1:
                            XPLMSetDataf(((xnz_cmd_context*)inRefcon)->bt.to32.l_rgb_ratio, 0.6f);
                            XPLMSetDataf(((xnz_cmd_context*)inRefcon)->bt.to32.r_rgb_ratio, 0.6f);
                            return 0;
                        default:
                            XPLMSetDataf(((xnz_cmd_context*)inRefcon)->bt.to32.l_rgb_ratio, 0.3f);
                            XPLMSetDataf(((xnz_cmd_context*)inRefcon)->bt.to32.r_rgb_ratio, 0.3f);
                            return 0;
                    }

                /*
                 * sim/flight_controls/brakes_regular seems to use 1-sim/parckBrake too…
                 */
                case XNZ_BT_FF35:
                    if (0 == parking_brake_get(inRefcon))
                    {
                        return parking_brake_set(inRefcon, 1);
                    }
                    return 0;

                case XNZ_BT_TBM9:
                    switch (speed)
                    {
                        case 2:
                            barray[0] = barray[1] = 0.9f;
                            break;
                        case 1:
                            barray[0] = barray[1] = 0.6f;
                            break;
                        default:
                            barray[0] = barray[1] = 0.3f;
                            break;
                    }
                    XPLMSetDatavf(((xnz_cmd_context*)inRefcon)->bt.tbm9.rbrak_array, barray, 0, 2);
                    return 0;

                case XNZ_BT_ERRR:
                default:
                    return 0;
            }
            return 0;
        }

        case xplm_CommandEnd:
        default:
        {
            switch (((xnz_cmd_context*)inRefcon)->xnz_bt)
            {
                case XNZ_BT_XPLM:
                    XPLMSetDataf(((xnz_cmd_context*)inRefcon)->xp.l_rgb_ratio, 0.0f);
                    XPLMSetDataf(((xnz_cmd_context*)inRefcon)->xp.r_rgb_ratio, 0.0f);
                    return parking_brake_set(inRefcon, ((xnz_cmd_context*)inRefcon)->xp.pbrak_onoff);
                    return 0;

                case XNZ_BT_COMM:
                    if (((xnz_cmd_context*)inRefcon)->bt.comm.cmd_current != NULL)
                    {
                        XPLMCommandEnd(((xnz_cmd_context*)inRefcon)->bt.comm.cmd_current);
                        ((xnz_cmd_context*)inRefcon)->bt.comm.cmd_current = NULL;
                        return 0;
                    }
                    return 0;

                case XNZ_BT_PKBR:
                case XNZ_BT_SIMC:
                    if (((xnz_cmd_context*)inRefcon)->xnz_pb == XNZ_PB_XPLM)
                    {
                        return parking_brake_set(inRefcon, ((xnz_cmd_context*)inRefcon)->xp.pbrak_onoff);
                    }
                    XPLMSetDataf(((xnz_cmd_context*)inRefcon)->xp.pbrak_ratio, 0.0f);
                    return parking_brake_set(inRefcon, ((xnz_cmd_context*)inRefcon)->xp.pbrak_onoff);

                case XNZ_BT_TO32:
                    XPLMSetDataf(((xnz_cmd_context*)inRefcon)->bt.to32.l_rgb_ratio, 0.0f);
                    XPLMSetDataf(((xnz_cmd_context*)inRefcon)->bt.to32.r_rgb_ratio, 0.0f);
                    return 0;

                case XNZ_BT_FF35:
                    return parking_brake_set(inRefcon, ((xnz_cmd_context*)inRefcon)->bt.ff35.pbrak_onoff);

                case XNZ_BT_TBM9:
                    barray[0] = barray[1] = 0.0f; XPLMSetDatavf(((xnz_cmd_context*)inRefcon)->bt.tbm9.rbrak_array, barray, 0, 2);
                    return 0;

                case XNZ_BT_ERRR:
                default:
                    return 0;
            }
            return 0;
        }
    }
    return 0;
}

static int chandler_pkb_tog(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        if (parking_brake_get(inRefcon))
        {
            return chandler_pkb_off(((xnz_cmd_context*)inRefcon)->cmd_pkb_off, xplm_CommandEnd, inRefcon); // use command handler for callouts
        }
        return chandler_pkb_onn(((xnz_cmd_context*)inRefcon)->cmd_pkb_onn, xplm_CommandEnd, inRefcon); // use command handler for callouts
    }
    return 0;
}

static int chandler_pkb_onh(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
#if 0
    if (inPhase == xplm_CommandEnd)
    {
        /*
         * this may not work under X-Plane 11, they didn't
         * apply the same tweak as for engine run switches
         */
        if (((xnz_cmd_context*)inRefcon)->xp_11_50_or_later)
        {
            return parking_brake_set(inRefcon, 1);
        }
    }
#endif
    if (inPhase == xplm_CommandEnd)
    {
        return chandler_pkb_off(((xnz_cmd_context*)inRefcon)->cmd_pkb_off, xplm_CommandEnd, inRefcon); // use command handler for callouts
    }
    if (inPhase == xplm_CommandBegin)
    {
        return chandler_pkb_onn(((xnz_cmd_context*)inRefcon)->cmd_pkb_onn, xplm_CommandEnd, inRefcon); // use command handler for callouts
    }
    if (inPhase == xplm_CommandContinue)
    {
        if (0 == parking_brake_get(inRefcon))
        {
            return parking_brake_set(inRefcon, 1);
        }
        return 0;
    }
    return 0;
}

static int chandler_pkb_onn(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        XPLMSpeakString("park brake set");
        return parking_brake_set(inRefcon, 1);
    }
    return 0;
}

static int chandler_pkb_off(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        XPLMSpeakString("park brake released");
        return parking_brake_set(inRefcon, 0);
    }
    return 0;
}

static int chandler_at_toga(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandBegin)
    {
        switch (((xnz_cmd_context*)inRefcon)->xnz_at)
        {
            case XNZ_AT_APTO:
            case XNZ_AT_NONE:
                if (((xnz_cmd_context*)inRefcon)->xnz_ap == XNZ_AP_XGFC)
                {
                    XPLMCommandBegin(((xnz_cmd_context*)inRefcon)->xp.ap_cw_st);
                    return 0;
                }
                return 0;

            default:
                break;
        }
        return 0;
    }
    if (inPhase == xplm_CommandEnd)
    {
        switch (((xnz_cmd_context*)inRefcon)->xnz_at)
        {
            case XNZ_AT_XP11:
                if (((xnz_cmd_context*)inRefcon)->xp.at_at_n1)
                {
                    XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->xp.ap_to_ga);
                    XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->xp.at_at_n1);
                    return 0;
                }
            case XNZ_AT_XPLM:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->xp.ap_to_ga);
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->xp.at_at_on);
                return 0;

            case XNZ_AT_TOLI:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->xp.at_at_on);
                return 0;

            case XNZ_AT_COMM:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->at.comm.cmd_at_toga);
                return 0;

            case XNZ_AT_APTO:
            case XNZ_AT_NONE:
                if (((xnz_cmd_context*)inRefcon)->xnz_ap == XNZ_AP_XGFC)
                {
                    XPLMCommandEnd(((xnz_cmd_context*)inRefcon)->xp.ap_cw_st);
                    return 0;
                }
                return 0;

            case XNZ_AT_ERRR:
            case XNZ_AT_TOGG:
//          case XNZ_AT_DISC:
            default:
                return 0;
        }
        return 0;
    }
    return 0;
}

static int chandler_a_12_lt(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        switch (((xnz_cmd_context*)inRefcon)->xnz_at)
        {
            case XNZ_AT_XPLM:
            case XNZ_AT_TOLI:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->xp.at_at_no);
                return 0;

            case XNZ_AT_COMM:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->at.comm.cmd_at_disc);
                return 0;

//          case XNZ_AT_DISC:
//              XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->xp.at_at_no);
//              return 0;

            case XNZ_AT_APTO:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->at.toga.cmd_ap_toga);
                return 0;

            case XNZ_AT_NONE:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->xp.ap_to_ga);
                return 0;

            case XNZ_AT_ERRR:
            case XNZ_AT_TOGG:
            default:
                return 0;
        }
        return 0;
    }
    return 0;
}

static int chandler_a_12_rt(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        switch (((xnz_cmd_context*)inRefcon)->xnz_at)
        {
            case XNZ_AT_APTO:
//              XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->at.toga.cmd_ap_toga); // only map to the leftmost button (engines 1 & 2)
                return 0;

            case XNZ_AT_NONE:
//              XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->xp.ap_to_ga); // only map to the leftmost button (engines 1 & 2)
                return 0;

            default:
                break;
        }
        return chandler_a_12_lt(((xnz_cmd_context*)inRefcon)->cmd_a_12_lt, xplm_CommandEnd, inRefcon);
    }
    return 0;
}

static int chandler_a_34_lt(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        switch (((xnz_cmd_context*)inRefcon)->xnz_at)
        {
            case XNZ_AT_APTO:
//              XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->at.toga.cmd_ap_toga); // only map to the leftmost button (engines 1 & 2)
                return 0;

            case XNZ_AT_NONE:
//              XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->xp.ap_to_ga); // only map to the leftmost button (engines 1 & 2)
                return 0;

            default:
                break;
        }
        return chandler_a_12_lt(((xnz_cmd_context*)inRefcon)->cmd_a_12_lt, xplm_CommandEnd, inRefcon);
    }
    return 0;
}

static int chandler_a_34_rt(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        switch (((xnz_cmd_context*)inRefcon)->xnz_at)
        {
            case XNZ_AT_APTO:
//              XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->at.toga.cmd_ap_toga); // only map to the leftmost button (engines 1 & 2)
                return 0;

            case XNZ_AT_NONE:
//              XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->xp.ap_to_ga); // only map to the leftmost button (engines 1 & 2)
                return 0;

            default:
                break;
        }
        return chandler_a_12_lt(((xnz_cmd_context*)inRefcon)->cmd_a_12_lt, xplm_CommandEnd, inRefcon);
    }
    return 0;
}

static int chandler_x_12_lt(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandBegin)
    {
        switch (((xnz_cmd_context*)inRefcon)->xnz_et)
        {
            case XNZ_ET_TBM9:
                XPLMCommandBegin(((xnz_cmd_context*)inRefcon)->et.tbm9.cmd_x_12_lt);
                return 0;

            case XNZ_ET_DA62:
            case XNZ_ET_XPPI:
                XPLMCommandBegin(((xnz_cmd_context*)inRefcon)->xp.p_start1);
                return 0;

            case XNZ_ET_E35L:
                if (XPLMGetDatai(((xnz_cmd_context*)inRefcon)->et.e35l.drf_e_1_knb) > 0)
                {
                    XPLMCommandBegin(((xnz_cmd_context*)inRefcon)->et.e35l.cmd_e_1_rgt);
                    return 0;
                }
                return 0;

            default:
                break;
        }
        return 0;
    }
    if (inPhase == xplm_CommandEnd)
    {
        switch (((xnz_cmd_context*)inRefcon)->xnz_et)
        {
            case XNZ_ET_TBM9:
                XPLMCommandEnd(((xnz_cmd_context*)inRefcon)->et.tbm9.cmd_x_12_lt);
                return 0;

            case XNZ_ET_DA62:
            case XNZ_ET_XPPI:
                XPLMCommandEnd(((xnz_cmd_context*)inRefcon)->xp.p_start1);
                return 0;

            case XNZ_ET_EVIC:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.evic.cmd_x_12_lt);
                return 0;

            case XNZ_ET_E55P:
                if (XPLMGetDataf(((xnz_cmd_context*)inRefcon)->et.e55p.drf_e_1_knb) > 0.5f)
                {
                    XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.e55p.cmd_e_1_rgt);
                }
                return 0;

            case XNZ_ET_E35L:
                if (XPLMGetDatai(((xnz_cmd_context*)inRefcon)->et.e35l.drf_e_1_knb) > 0)
                {
                    XPLMCommandEnd(((xnz_cmd_context*)inRefcon)->et.e35l.cmd_e_1_rgt);
                    return 0;
                }
                return 0;

            case XNZ_ET_IX73:
                if (XPLMGetDatai(((xnz_cmd_context*)inRefcon)->xp.ongroundany))
                {
                    XPLMSetDataf(((xnz_cmd_context*)inRefcon)->et.ix73.drf_e_1_knb, -1.0f);
                    return 0;
                }
                XPLMSetDataf(((xnz_cmd_context*)inRefcon)->et.ix73.drf_e_1_knb, 2.0f);
                return 0;

            case XNZ_ET_FF75:
                if (XPLMGetDatai(((xnz_cmd_context*)inRefcon)->xp.ongroundany))
                {
                    XPLMSetDataf(((xnz_cmd_context*)inRefcon)->et.ff75.drf_e_1_knb, 0.0f);
                    return 0;
                }
                XPLMSetDataf(((xnz_cmd_context*)inRefcon)->et.ff75.drf_e_1_knb, 4.0f);
                return 0;

            default:
                break;
        }
        return 0;
    }
    return 0;
}

static int chandler_x_12_rt(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandBegin)
    {
        switch (((xnz_cmd_context*)inRefcon)->xnz_et)
        {
            case XNZ_ET_TBM9:
                XPLMCommandBegin(((xnz_cmd_context*)inRefcon)->et.tbm9.cmd_x_12_rt);
                return 0;

            case XNZ_ET_DA62:
            case XNZ_ET_XPPI:
                XPLMCommandBegin(((xnz_cmd_context*)inRefcon)->xp.p_start2);
                return 0;

            case XNZ_ET_E35L:
                if (XPLMGetDatai(((xnz_cmd_context*)inRefcon)->et.e35l.drf_e_2_knb) > 0)
                {
                    XPLMCommandBegin(((xnz_cmd_context*)inRefcon)->et.e35l.cmd_e_2_rgt);
                    return 0;
                }
                return 0;

            default:
                break;
        }
        return 0;
    }
    if (inPhase == xplm_CommandEnd)
    {
        switch (((xnz_cmd_context*)inRefcon)->xnz_et)
        {
            case XNZ_ET_TBM9:
                XPLMCommandEnd(((xnz_cmd_context*)inRefcon)->et.tbm9.cmd_x_12_rt);
                return 0;

            case XNZ_ET_DA62:
            case XNZ_ET_XPPI:
                XPLMCommandEnd(((xnz_cmd_context*)inRefcon)->xp.p_start2);
                return 0;

            case XNZ_ET_EVIC:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.evic.cmd_x_12_rt);
                return 0;

            case XNZ_ET_E55P:
                if (XPLMGetDataf(((xnz_cmd_context*)inRefcon)->et.e55p.drf_e_2_knb) > 0.5f)
                {
                    XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.e55p.cmd_e_2_rgt);
                }
                return 0;

            case XNZ_ET_E35L:
                if (XPLMGetDatai(((xnz_cmd_context*)inRefcon)->et.e35l.drf_e_2_knb) > 0)
                {
                    XPLMCommandEnd(((xnz_cmd_context*)inRefcon)->et.e35l.cmd_e_2_rgt);
                    return 0;
                }
                return 0;

            case XNZ_ET_IX73:
                if (XPLMGetDatai(((xnz_cmd_context*)inRefcon)->xp.ongroundany))
                {
                    XPLMSetDataf(((xnz_cmd_context*)inRefcon)->et.ix73.drf_e_2_knb, -1.0f);
                    return 0;
                }
                XPLMSetDataf(((xnz_cmd_context*)inRefcon)->et.ix73.drf_e_2_knb, 2.0f);
                return 0;

            case XNZ_ET_FF75:
                if (XPLMGetDatai(((xnz_cmd_context*)inRefcon)->xp.ongroundany))
                {
                    XPLMSetDataf(((xnz_cmd_context*)inRefcon)->et.ff75.drf_e_2_knb, 0.0f);
                    return 0;
                }
                XPLMSetDataf(((xnz_cmd_context*)inRefcon)->et.ff75.drf_e_2_knb, 4.0f);
                return 0;

            default:
                break;
        }
        return 0;
    }
    return 0;
}

static int chandler_x_34_lt(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandBegin)
    {
        switch (((xnz_cmd_context*)inRefcon)->xnz_et)
        {
            case XNZ_ET_TBM9:
                XPLMCommandBegin(((xnz_cmd_context*)inRefcon)->et.tbm9.cmd_x_12_lt);
                return 0;

            case XNZ_ET_XPPI:
                XPLMCommandBegin(((xnz_cmd_context*)inRefcon)->xp.p_start3);
                return 0;

            default:
                break;
        }
        return 0;
    }
    if (inPhase == xplm_CommandEnd)
    {
        switch (((xnz_cmd_context*)inRefcon)->xnz_et)
        {
            case XNZ_ET_TBM9:
                XPLMCommandEnd(((xnz_cmd_context*)inRefcon)->et.tbm9.cmd_x_12_lt);
                return 0;

            case XNZ_ET_XPPI:
                XPLMCommandEnd(((xnz_cmd_context*)inRefcon)->xp.p_start3);
                return 0;

            default:
                break;
        }
        return 0;
    }
    return 0;
}

static int chandler_x_34_rt(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandBegin)
    {
        switch (((xnz_cmd_context*)inRefcon)->xnz_et)
        {
            case XNZ_ET_TBM9:
                XPLMCommandBegin(((xnz_cmd_context*)inRefcon)->et.tbm9.cmd_x_12_rt);
                return 0;

            case XNZ_ET_XPPI:
                XPLMCommandBegin(((xnz_cmd_context*)inRefcon)->xp.p_start4);
                return 0;

            default:
                break;
        }
        return 0;
    }
    if (inPhase == xplm_CommandEnd)
    {
        switch (((xnz_cmd_context*)inRefcon)->xnz_et)
        {
            case XNZ_ET_TBM9:
                XPLMCommandEnd(((xnz_cmd_context*)inRefcon)->et.tbm9.cmd_x_12_rt);
                return 0;

            case XNZ_ET_XPPI:
                XPLMCommandEnd(((xnz_cmd_context*)inRefcon)->xp.p_start4);
                return 0;

            default:
                break;
        }
        return 0;
    }
    return 0;
}

static int chandler_m_12_ch(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandBegin)
    {
        if (((xnz_cmd_context*)inRefcon)->xp_11_50_or_later)
        {
            return 0;
        }
        XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->cmd_m_12_cr);
        return 0;
    }
    if (inPhase == xplm_CommandEnd)
    {
        if (((xnz_cmd_context*)inRefcon)->xp_11_50_or_later)
        {
            return chandler_m_12_cr(((xnz_cmd_context*)inRefcon)->cmd_m_12_cr, xplm_CommandEnd, inRefcon);
        }
        XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->cmd_m_12_no);
        return 0;
    }
    return 0;
}

static int chandler_m_12_cr(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        switch (((xnz_cmd_context*)inRefcon)->xnz_et)
        {
            case XNZ_ET_TO32:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.to32.cmd_m_12_cr);
                return 0;

            case XNZ_ET_FF35:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.ff35.cmd_m_12_cr);
                return 0;

            case XNZ_ET_TBM9:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.tbm9.cmd_m_12_cr);
                return 0;

            case XNZ_ET_XPPI:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->xp.p_m_lft1);
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->xp.p_m_lft2);
                return 0;

            case XNZ_ET_E55P:
                if (XPLMGetDataf(((xnz_cmd_context*)inRefcon)->et.e55p.drf_e_1_ign) > 0.5f)
                {
                    if (XPLMGetDataf(((xnz_cmd_context*)inRefcon)->et.e55p.drf_e_1_ign) > 1.5f)
                    {
                        XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.e55p.cmd_ig_1_dn);
                    }
                    XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.e55p.cmd_ig_1_dn);
                }
                if (XPLMGetDataf(((xnz_cmd_context*)inRefcon)->et.e55p.drf_e_2_ign) > 0.5f)
                {
                    if (XPLMGetDataf(((xnz_cmd_context*)inRefcon)->et.e55p.drf_e_2_ign) > 1.5f)
                    {
                        XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.e55p.cmd_ig_2_dn);
                    }
                    XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.e55p.cmd_ig_2_dn);
                }
                return 0;

            case XNZ_ET_DA62:
                if (XPLMGetDataf(((xnz_cmd_context*)inRefcon)->et.da62.drf_mod_ec1) > 0.5f)
                {
                    if (XPLMGetDataf(((xnz_cmd_context*)inRefcon)->et.da62.drf_mod_ec1) > 1.5f)
                    {
                        XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.da62.cmd_ecu1_dn);
                    }
                    XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.da62.cmd_ecu1_dn);
                }
                if (XPLMGetDataf(((xnz_cmd_context*)inRefcon)->et.da62.drf_mod_ec2) > 0.5f)
                {
                    if (XPLMGetDataf(((xnz_cmd_context*)inRefcon)->et.da62.drf_mod_ec2) > 1.5f)
                    {
                        XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.da62.cmd_ecu2_dn);
                    }
                    XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.da62.cmd_ecu2_dn);
                }
                return 0;

            case XNZ_ET_E35L:
            {
                int auto_ignite_off[2] = { 0, 0, };
                XPLMSetDatavi(((xnz_cmd_context*)inRefcon)->xp.auto_ignite, auto_ignite_off, 0, 2);
                return 0;
            }

            case XNZ_ET_FF75:
                XPLMSetDataf(((xnz_cmd_context*)inRefcon)->et.ff75.drf_e_1_knb, 2.0f);
                XPLMSetDataf(((xnz_cmd_context*)inRefcon)->et.ff75.drf_e_2_knb, 2.0f);
                return 0;

            default:
                break;
        }
        return 0;
    }
    return 0;
}

static int chandler_m_12_no(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        switch (((xnz_cmd_context*)inRefcon)->xnz_et)
        {
            case XNZ_ET_TO32:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.to32.cmd_m_12_no);
                return 0;

            case XNZ_ET_FF35:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.ff35.cmd_m_12_no);
                return 0;

            case XNZ_ET_TBM9:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.tbm9.cmd_m_12_no);
                return 0;

            case XNZ_ET_EVIC:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.evic.cmd_m_12_no);
                return 0;

            case XNZ_ET_E55P:
                if (XPLMGetDataf(((xnz_cmd_context*)inRefcon)->et.e55p.drf_e_1_ign) < 0.5f)
                {
                    XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.e55p.cmd_ig_1_up);
                }
                else if (XPLMGetDataf(((xnz_cmd_context*)inRefcon)->et.e55p.drf_e_1_ign) > 1.5f)
                {
                    XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.e55p.cmd_ig_1_dn);
                }
                if (XPLMGetDataf(((xnz_cmd_context*)inRefcon)->et.e55p.drf_e_2_ign) < 0.5f)
                {
                    XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.e55p.cmd_ig_2_up);
                }
                else if (XPLMGetDataf(((xnz_cmd_context*)inRefcon)->et.e55p.drf_e_2_ign) > 1.5f)
                {
                    XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.e55p.cmd_ig_2_dn);
                }
                return 0;

            case XNZ_ET_XPPI:
            {
                int eng_running[2]; XPLMGetDatavi(((xnz_cmd_context*)inRefcon)->xp.eng_running, eng_running, 0, 2);
                if (eng_running[0])
                {
                    XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->xp.p_mboth1);
                }
                else
                {
                    XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->xp.p_mstop1);
                }
                if (eng_running[1])
                {
                    XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->xp.p_mboth2);
                }
                else
                {
                    XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->xp.p_mstop2);
                }
                return 0;
            }

            case XNZ_ET_EA50:
            {
                int eng_running[2]; XPLMGetDatavi(((xnz_cmd_context*)inRefcon)->xp.eng_running, eng_running, 0, 2);
                if (eng_running[0])
                {
                    XPLMSetDatai(((xnz_cmd_context*)inRefcon)->et.ea50.drf_mod_en1, 1);
                }
                else
                {
                    XPLMSetDatai(((xnz_cmd_context*)inRefcon)->et.ea50.drf_mod_en1, 0);
                }
                if (eng_running[1])
                {
                    XPLMSetDatai(((xnz_cmd_context*)inRefcon)->et.ea50.drf_mod_en2, 1);
                }
                else
                {
                    XPLMSetDatai(((xnz_cmd_context*)inRefcon)->et.ea50.drf_mod_en2, 0);
                }
                return 0;
            }

            case XNZ_ET_DA62:
                if (XPLMGetDataf(((xnz_cmd_context*)inRefcon)->et.da62.drf_mod_ec1) < 0.5f)
                {
                    XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.da62.cmd_ecu1_up);
                }
                else if (XPLMGetDataf(((xnz_cmd_context*)inRefcon)->et.da62.drf_mod_ec1) > 1.5f)
                {
                    XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.da62.cmd_ecu1_dn);
                }
                if (XPLMGetDataf(((xnz_cmd_context*)inRefcon)->et.da62.drf_mod_ec2) < 0.5f)
                {
                    XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.da62.cmd_ecu2_up);
                }
                else if (XPLMGetDataf(((xnz_cmd_context*)inRefcon)->et.da62.drf_mod_ec2) > 1.5f)
                {
                    XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.da62.cmd_ecu2_dn);
                }
                return 0;

            case XNZ_ET_E35L:
            {
                int auto_ignite_on[2] = { 1, 1, };
                XPLMSetDatavi(((xnz_cmd_context*)inRefcon)->xp.auto_ignite, auto_ignite_on, 0, 2);
                return 0;
            }

            case XNZ_ET_IX73:
                XPLMSetDataf(((xnz_cmd_context*)inRefcon)->et.ix73.drf_e_1_knb, 0.0f);
                XPLMSetDataf(((xnz_cmd_context*)inRefcon)->et.ix73.drf_e_2_knb, 0.0f);
                return 0;

            case XNZ_ET_FF75:
            {
                int eng_running[2]; XPLMGetDatavi(((xnz_cmd_context*)inRefcon)->xp.eng_running, eng_running, 0, 2);
                if (eng_running[0])
                {
                    XPLMSetDataf(((xnz_cmd_context*)inRefcon)->et.ff75.drf_e_1_knb, 1.0f);
                }
                else
                {
                    XPLMSetDataf(((xnz_cmd_context*)inRefcon)->et.ff75.drf_e_1_knb, 2.0f);
                }
                if (eng_running[1])
                {
                    XPLMSetDataf(((xnz_cmd_context*)inRefcon)->et.ff75.drf_e_2_knb, 1.0f);
                }
                else
                {
                    XPLMSetDataf(((xnz_cmd_context*)inRefcon)->et.ff75.drf_e_2_knb, 2.0f);
                }
                return 0;
            }

            default:
                break;
        }
        return 0;
    }
    return 0;
}

static int chandler_m_12_st(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        switch (((xnz_cmd_context*)inRefcon)->xnz_et)
        {
            case XNZ_ET_TO32:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.to32.cmd_m_12_st);
                return 0;

            case XNZ_ET_FF35:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.ff35.cmd_m_12_st);
                return 0;

            case XNZ_ET_TBM9:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.tbm9.cmd_m_12_st);
                return 0;

            case XNZ_ET_XPPI:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->xp.p_m_rgt1);
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->xp.p_m_rgt2);
                return 0;

            case XNZ_ET_EVIC:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.evic.cmd_m_12_st);
                return 0;

            case XNZ_ET_E55P:
                if (XPLMGetDataf(((xnz_cmd_context*)inRefcon)->et.e55p.drf_e_1_ign) < 1.5f)
                {
                    if (XPLMGetDataf(((xnz_cmd_context*)inRefcon)->et.e55p.drf_e_1_ign) < 0.5f)
                    {
                        XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.e55p.cmd_ig_1_up);
                    }
                    XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.e55p.cmd_ig_1_up);
                }
                if (XPLMGetDataf(((xnz_cmd_context*)inRefcon)->et.e55p.drf_e_2_ign) < 1.5f)
                {
                    if (XPLMGetDataf(((xnz_cmd_context*)inRefcon)->et.e55p.drf_e_2_ign) < 0.5f)
                    {
                        XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.e55p.cmd_ig_2_up);
                    }
                    XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.e55p.cmd_ig_2_up);
                }
                return 0;

            case XNZ_ET_EA50:
            {
                int eng_running[2]; XPLMGetDatavi(((xnz_cmd_context*)inRefcon)->xp.eng_running, eng_running, 0, 2);
                if (eng_running[0])
                {
                    XPLMSetDatai(((xnz_cmd_context*)inRefcon)->et.ea50.drf_mod_en1, 2);
                }
                if (eng_running[1])
                {
                    XPLMSetDatai(((xnz_cmd_context*)inRefcon)->et.ea50.drf_mod_en2, 2);
                }
                return 0;
            }

            case XNZ_ET_DA62:
                if (XPLMGetDataf(((xnz_cmd_context*)inRefcon)->et.da62.drf_mod_ec1) < 1.5f)
                {
                    if (XPLMGetDataf(((xnz_cmd_context*)inRefcon)->et.da62.drf_mod_ec1) < 0.5f)
                    {
                        XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.da62.cmd_ecu1_up);
                    }
                    XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.da62.cmd_ecu1_up);
                }
                if (XPLMGetDataf(((xnz_cmd_context*)inRefcon)->et.da62.drf_mod_ec2) < 1.5f)
                {
                    if (XPLMGetDataf(((xnz_cmd_context*)inRefcon)->et.da62.drf_mod_ec2) < 0.5f)
                    {
                        XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.da62.cmd_ecu2_up);
                    }
                    XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.da62.cmd_ecu2_up);
                }
                return 0;

            case XNZ_ET_IX73:
                XPLMSetDataf(((xnz_cmd_context*)inRefcon)->et.ix73.drf_e_1_knb, 1.0f);
                XPLMSetDataf(((xnz_cmd_context*)inRefcon)->et.ix73.drf_e_2_knb, 1.0f);
                return 0;

            case XNZ_ET_FF75:
                XPLMSetDataf(((xnz_cmd_context*)inRefcon)->et.ff75.drf_e_1_knb, 3.0f);
                XPLMSetDataf(((xnz_cmd_context*)inRefcon)->et.ff75.drf_e_2_knb, 3.0f);
                return 0;

            default:
                break;
        }
        return 0;
    }
    return 0;
}

static int chandler_m_12_sh(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandBegin)
    {
        if (((xnz_cmd_context*)inRefcon)->xp_11_50_or_later)
        {
            return 0;
        }
        XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->cmd_m_12_st);
        return 0;
    }
    if (inPhase == xplm_CommandEnd)
    {
        if (((xnz_cmd_context*)inRefcon)->xp_11_50_or_later)
        {
            return chandler_m_12_st(((xnz_cmd_context*)inRefcon)->cmd_m_12_st, xplm_CommandEnd, inRefcon);
        }
        XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->cmd_m_12_no);
        return 0;
    }
    return 0;
}

static int chandler_m_34_ch(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandBegin)
    {
        if (((xnz_cmd_context*)inRefcon)->xp_11_50_or_later)
        {
            return 0;
        }
        XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->cmd_m_34_cr);
        return 0;
    }
    if (inPhase == xplm_CommandEnd)
    {
        if (((xnz_cmd_context*)inRefcon)->xp_11_50_or_later)
        {
            return chandler_m_34_cr(((xnz_cmd_context*)inRefcon)->cmd_m_34_cr, xplm_CommandEnd, inRefcon);
        }
        XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->cmd_m_34_no);
        return 0;
    }
    return 0;
}

static int chandler_m_34_cr(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        switch (((xnz_cmd_context*)inRefcon)->xnz_et)
        {
            case XNZ_ET_TO32:
            case XNZ_ET_FF35:
            case XNZ_ET_TBM9:
                return chandler_m_12_cr(((xnz_cmd_context*)inRefcon)->cmd_m_12_cr, xplm_CommandEnd, inRefcon);

            case XNZ_ET_XPPI:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->xp.p_m_lft3);
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->xp.p_m_lft4);
                return 0;

            default:
                break;
        }
        return 0;
    }
    return 0;
}

static int chandler_m_34_no(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        switch (((xnz_cmd_context*)inRefcon)->xnz_et)
        {
            case XNZ_ET_TO32:
            case XNZ_ET_FF35:
            case XNZ_ET_TBM9:
                return chandler_m_12_no(((xnz_cmd_context*)inRefcon)->cmd_m_12_no, xplm_CommandEnd, inRefcon);

            case XNZ_ET_XPPI:
            {
                int eng_running[2]; XPLMGetDatavi(((xnz_cmd_context*)inRefcon)->xp.eng_running, eng_running, 2, 2);
                if (eng_running[0])
                {
                    XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->xp.p_mboth3);
                }
                else
                {
                    XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->xp.p_mstop3);
                }
                if (eng_running[1])
                {
                    XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->xp.p_mboth4);
                }
                else
                {
                    XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->xp.p_mstop4);
                }
                return 0;
            }

            default:
                break;
        }
        return 0;
    }
    return 0;
}

static int chandler_m_34_st(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        switch (((xnz_cmd_context*)inRefcon)->xnz_et)
        {
            case XNZ_ET_TO32:
            case XNZ_ET_FF35:
            case XNZ_ET_TBM9:
                return chandler_m_12_st(((xnz_cmd_context*)inRefcon)->cmd_m_12_st, xplm_CommandEnd, inRefcon);

            case XNZ_ET_XPPI:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->xp.p_m_rgt3);
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->xp.p_m_rgt4);
                return 0;

            default:
                break;
        }
        return 0;
    }
    return 0;
}

static int chandler_m_34_sh(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandBegin)
    {
        if (((xnz_cmd_context*)inRefcon)->xp_11_50_or_later)
        {
            return 0;
        }
        XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->cmd_m_34_st);
        return 0;
    }
    if (inPhase == xplm_CommandEnd)
    {
        if (((xnz_cmd_context*)inRefcon)->xp_11_50_or_later)
        {
            return chandler_m_34_st(((xnz_cmd_context*)inRefcon)->cmd_m_34_st, xplm_CommandEnd, inRefcon);
        }
        XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->cmd_m_34_no);
        return 0;
    }
    return 0;
}

static int chandler_e_1_onh(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandBegin)
    {
        if (((xnz_cmd_context*)inRefcon)->xp_11_50_or_later)
        {
            return 0;
        }
        XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->cmd_e_1_onn);
        return 0;
    }
    if (inPhase == xplm_CommandEnd)
    {
        if (((xnz_cmd_context*)inRefcon)->xp_11_50_or_later)
        {
            return chandler_e_1_onn(((xnz_cmd_context*)inRefcon)->cmd_e_1_onn, xplm_CommandEnd, inRefcon);
        }
        XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->cmd_e_1_off);
        return 0;
    }
    return 0;
}

static int chandler_e_1_onn(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        switch (((xnz_cmd_context*)inRefcon)->xnz_et)
        {
            case XNZ_ET_TO32:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.to32.cmd_e_1_onn);
                return 0;

            case XNZ_ET_FF35:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.ff35.cmd_e_1_onn);
                return 0;

            case XNZ_ET_TBM9:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.tbm9.cmd_e_1_onn);
                return 0;

            case XNZ_ET_XPPI:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->xp.p_mboth1);
                return 0;

            case XNZ_ET_EA50:
                XPLMSetDatai(((xnz_cmd_context*)inRefcon)->et.ea50.drf_mod_en1, 1);
                return 0;

            case XNZ_ET_DA62:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.da62.cmd_e_1_onn);
                return 0;

            case XNZ_ET_EVIC:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.evic.cmd_e_1_onn);
                return 0;

            case XNZ_ET_E55P:
                if (XPLMGetDataf(((xnz_cmd_context*)inRefcon)->et.e55p.drf_e_1_knb) < 0.5f)
                {
                    XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.e55p.cmd_e_1_rgt);
                }
                return 0;

            case XNZ_ET_E35L:
                if (XPLMGetDatai(((xnz_cmd_context*)inRefcon)->et.e35l.drf_e_1_knb) < 1)
                {
                    XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.e35l.cmd_e_1_rgt);
                    return 0;
                }
                return 0;

            case XNZ_ET_IX73:
                XPLMSetDataf(((xnz_cmd_context*)inRefcon)->et.ix73.drf_e_1_cut, 1.0f);
                return 0;

            case XNZ_ET_FF75:
                XPLMSetDatai(((xnz_cmd_context*)inRefcon)->et.ix73.drf_e_1_cut, 2);
                return 0;

            default:
                break;
        }
        return 0;
    }
    return 0;
}

static int chandler_e_1_off(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        switch (((xnz_cmd_context*)inRefcon)->xnz_et)
        {
            case XNZ_ET_TO32:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.to32.cmd_e_1_off);
                return 0;

            case XNZ_ET_FF35:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.ff35.cmd_e_1_off);
                return 0;

            case XNZ_ET_TBM9:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.tbm9.cmd_e_1_off);
                return 0;

            case XNZ_ET_XPPI:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->xp.p_mstop1);
                return 0;

            case XNZ_ET_EA50:
                XPLMSetDatai(((xnz_cmd_context*)inRefcon)->et.ea50.drf_mod_en1, 0);
                return 0;

            case XNZ_ET_DA62:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.da62.cmd_e_1_off);
                return 0;

            case XNZ_ET_EVIC:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.evic.cmd_e_1_off);
                return 0;

            case XNZ_ET_E55P:
                if (XPLMGetDataf(((xnz_cmd_context*)inRefcon)->et.e55p.drf_e_1_knb) > 0.5f)
                {
                    XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.e55p.cmd_e_1_lft);
                }
                return 0;

            case XNZ_ET_E35L:
                if (XPLMGetDatai(((xnz_cmd_context*)inRefcon)->et.e35l.drf_e_1_knb) > 0)
                {
                    XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.e35l.cmd_e_1_lft);
                    return 0;
                }
                return 0;

            case XNZ_ET_IX73:
                XPLMSetDataf(((xnz_cmd_context*)inRefcon)->et.ix73.drf_e_1_cut, 0.0f);
                return 0;

            case XNZ_ET_FF75:
                XPLMSetDatai(((xnz_cmd_context*)inRefcon)->et.ix73.drf_e_1_cut, 0);
                return 0;

            default:
                break;
        }
        return 0;
    }
    return 0;
}

static int chandler_e_2_onh(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandBegin)
    {
        if (((xnz_cmd_context*)inRefcon)->xp_11_50_or_later)
        {
            return 0;
        }
        XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->cmd_e_2_onn);
        return 0;
    }
    if (inPhase == xplm_CommandEnd)
    {
        if (((xnz_cmd_context*)inRefcon)->xp_11_50_or_later)
        {
            return chandler_e_2_onn(((xnz_cmd_context*)inRefcon)->cmd_e_2_onn, xplm_CommandEnd, inRefcon);
        }
        XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->cmd_e_2_off);
        return 0;
    }
    return 0;
}

static int chandler_e_2_onn(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        switch (((xnz_cmd_context*)inRefcon)->xnz_et)
        {
            case XNZ_ET_TO32:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.to32.cmd_e_2_onn);
                return 0;

            case XNZ_ET_FF35:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.ff35.cmd_e_2_onn);
                return 0;

            case XNZ_ET_TBM9:
                XPLMSetDatai(((xnz_cmd_context*)inRefcon)->et.tbm9.drf_fuelsel, 0); // FUEL SEL switch position. 0 = AUTO, 1 = MAN.
                return 0;

            case XNZ_ET_XPPI:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->xp.p_mboth2);
                return 0;

            case XNZ_ET_EA50:
                XPLMSetDatai(((xnz_cmd_context*)inRefcon)->et.ea50.drf_mod_en2, 1);
                return 0;

            case XNZ_ET_DA62:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.da62.cmd_e_2_onn);
                return 0;

            case XNZ_ET_E55P:
                if (XPLMGetDataf(((xnz_cmd_context*)inRefcon)->et.e55p.drf_e_2_knb) < 0.5f)
                {
                    XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.e55p.cmd_e_2_rgt);
                }
                return 0;

            case XNZ_ET_EVIC:
                if (XPLMGetDatai(((xnz_cmd_context*)inRefcon)->et.evic.drf_fuel_at) == 0)
                {
                    XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.evic.cmd_e_2_tog);
                }
                return 0;

            case XNZ_ET_E35L:
                if (XPLMGetDatai(((xnz_cmd_context*)inRefcon)->et.e35l.drf_e_2_knb) < 1)
                {
                    XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.e35l.cmd_e_2_rgt);
                    return 0;
                }
                return 0;

            case XNZ_ET_IX73:
                XPLMSetDataf(((xnz_cmd_context*)inRefcon)->et.ix73.drf_e_2_cut, 1.0f);
                return 0;

            case XNZ_ET_FF75:
                XPLMSetDatai(((xnz_cmd_context*)inRefcon)->et.ix73.drf_e_2_cut, 2);
                return 0;

            default:
                break;
        }
        return 0;
    }
    return 0;
}

static int chandler_e_2_off(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        switch (((xnz_cmd_context*)inRefcon)->xnz_et)
        {
            case XNZ_ET_TO32:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.to32.cmd_e_2_off);
                return 0;

            case XNZ_ET_FF35:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.ff35.cmd_e_2_off);
                return 0;

            case XNZ_ET_TBM9:
                XPLMSetDatai(((xnz_cmd_context*)inRefcon)->et.tbm9.drf_fuelsel, 1); // FUEL SEL switch position. 0 = AUTO, 1 = MAN.
                return 0;

            case XNZ_ET_XPPI:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->xp.p_mstop2);
                return 0;

            case XNZ_ET_EA50:
                XPLMSetDatai(((xnz_cmd_context*)inRefcon)->et.ea50.drf_mod_en2, 0);
                return 0;

            case XNZ_ET_DA62:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.da62.cmd_e_2_off);
                return 0;

            case XNZ_ET_E55P:
                if (XPLMGetDataf(((xnz_cmd_context*)inRefcon)->et.e55p.drf_e_2_knb) > 0.5f)
                {
                    XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.e55p.cmd_e_2_lft);
                }
                return 0;

            case XNZ_ET_EVIC:
                if (XPLMGetDatai(((xnz_cmd_context*)inRefcon)->et.evic.drf_fuel_at) != 0)
                {
                    XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.evic.cmd_e_2_tog);
                }
                return 0;

            case XNZ_ET_E35L:
                if (XPLMGetDatai(((xnz_cmd_context*)inRefcon)->et.e35l.drf_e_2_knb) > 0)
                {
                    XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->et.e35l.cmd_e_2_lft);
                    return 0;
                }
                return 0;

            case XNZ_ET_IX73:
                XPLMSetDataf(((xnz_cmd_context*)inRefcon)->et.ix73.drf_e_2_cut, 0.0f);
                return 0;

            case XNZ_ET_FF75:
                XPLMSetDatai(((xnz_cmd_context*)inRefcon)->et.ix73.drf_e_2_cut, 0);
                return 0;

            default:
                break;
        }
        return 0;
    }
    return 0;
}

static int chandler_e_3_onh(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandBegin)
    {
        if (((xnz_cmd_context*)inRefcon)->xp_11_50_or_later)
        {
            return 0;
        }
        XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->cmd_e_3_onn);
        return 0;
    }
    if (inPhase == xplm_CommandEnd)
    {
        if (((xnz_cmd_context*)inRefcon)->xp_11_50_or_later)
        {
            return chandler_e_3_onn(((xnz_cmd_context*)inRefcon)->cmd_e_3_onn, xplm_CommandEnd, inRefcon);
        }
        XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->cmd_e_3_off);
        return 0;
    }
    return 0;
}

static int chandler_e_3_onn(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        switch (((xnz_cmd_context*)inRefcon)->xnz_et)
        {
            case XNZ_ET_XPPI:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->xp.p_mboth3);
                return 0;

            default:
                break;
        }
        return 0;
    }
    return 0;
}

static int chandler_e_3_off(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        switch (((xnz_cmd_context*)inRefcon)->xnz_et)
        {
            case XNZ_ET_XPPI:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->xp.p_mstop3);
                return 0;

            default:
                break;
        }
        return 0;
    }
    return 0;
}

static int chandler_e_4_onh(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandBegin)
    {
        if (((xnz_cmd_context*)inRefcon)->xp_11_50_or_later)
        {
            return 0;
        }
        XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->cmd_e_4_onn);
        return 0;
    }
    if (inPhase == xplm_CommandEnd)
    {
        if (((xnz_cmd_context*)inRefcon)->xp_11_50_or_later)
        {
            return chandler_e_4_onn(((xnz_cmd_context*)inRefcon)->cmd_e_4_onn, xplm_CommandEnd, inRefcon);
        }
        XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->cmd_e_4_off);
        return 0;
    }
    return 0;
}

static int chandler_e_4_onn(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        switch (((xnz_cmd_context*)inRefcon)->xnz_et)
        {
            case XNZ_ET_XPPI:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->xp.p_mboth4);
                return 0;

            default:
                break;
        }
        return 0;
    }
    return 0;
}

static int chandler_e_4_off(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        switch (((xnz_cmd_context*)inRefcon)->xnz_et)
        {
            case XNZ_ET_XPPI:
                XPLMCommandOnce(((xnz_cmd_context*)inRefcon)->xp.p_mstop4);
                return 0;

            default:
                break;
        }
        return 0;
    }
    return 0;
}

static int chandler_printax(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        if (inRefcon)
        {
            if (((xnz_context*)inRefcon)->idx_throttle_axis_1 >= 0)
            {
                float f[2]; XPLMGetDatavf(((xnz_context*)inRefcon)->f_stick_val, f, ((xnz_context*)inRefcon)->idx_throttle_axis_1, 2);
                xnz_log("[debug]: throttle axes (raw): (%.6f -- %.6f) --> (%.6f)\n", f[0], f[1], ((f[0] + f[1]) / 2.0f));
                return 0;
            }
            return 0;
        }
        return 0;
    }
    return 0;
}

#undef AIRSPEED_MIN_KTS
#undef AIRSPEED_MAX_KTS
#undef GROUNDSP_MIN_KTS
#undef GROUNDSP_MAX_KTS
#undef GROUNDSP_KTS_MIN
#undef GROUNDSP_KTS_MID
#undef GROUNDSP_KTS_MAX
#undef XNZ_LOG_PREFIX
#undef XNZ_XPLM_TITLE
#undef ACF_ROLL_SET
#undef HS_TBM9_IDLE
#undef XNZ_THINN_NO
#undef XNZ_THOUT_AT
#undef XNZ_THOUT_SK
#undef MPS2KPH
#undef MPS2KTS
#undef T_SMALL
#undef T_ZERO
