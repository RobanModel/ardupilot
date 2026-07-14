#include "Copter.h"

#if FRAME_CONFIG == HELI_FRAME

/*
 * Coordinated turn assist ("bank angle steering") for traditional helicopters.
 * See heli_bank_steer.h for an overview.
 *
 * ============================ MAINTAINER NOTES ============================
 * (for humans and future AI sessions -- read before changing anything)
 *
 * WHAT THIS IS: an optional pilot-input shaping layer.  It adds an automatic
 * yaw-rate component to the pilot's rudder command so that banked forward
 * flight is coordinated (nose follows the turn) without pilot rudder input.
 * It does NOT touch attitude/rate PID loops, navigation, or servo mixing.
 *
 * WHERE IT RUNS: called every main loop iteration from
 * Mode::get_pilot_desired_yaw_rate() in mode.cpp, but ONLY when
 * HELI_BANK_STEER=1 and the active mode opted in via
 * Mode::allows_coordinated_turn_assist() (Stabilize, AltHold, Loiter).
 * With HELI_BANK_STEER=0 this file is dead code.
 *
 * UNITS: everything in this file is SI (radians, rad/s, m/s).  The caller in
 * mode.cpp converts to/from the vehicle-side units (centidegrees/s on 4.6.x).
 *
 * PIPELINE (update_rads):
 *   1. disabled?           -> pass pilot command through, reset state
 *   2. update-gap detect   -> restart filter from zero after >0.2 s without
 *                             calls (mode switch / just enabled), no steps
 *   3. gates               -> caller passes coordination_active
 *                             (armed && spooled && !landed && dynamic_flight);
 *                             plus |commanded bank| < 80 deg here
 *   4. bank deadband ramp  -> 0 inside HELI_BANK_DB deg, linear to 1 at 2xDB
 *   5. speed fade          -> 0 below 2 m/s BODY-FORWARD groundspeed,
 *                             linear to 1 at 4 m/s (hover/sideways => no yaw)
 *   6. physics             -> ideal coordinated-turn rate g*tan(bank)/speed,
 *                             bank clamped +-60 deg, speed floored 3 m/s
 *   7. scale + clamp       -> * HELI_BANK_GAIN, clamped +-HELI_BANK_YAWMAX
 *   8. smoothing           -> 1st-order low-pass, tau 0.5 s
 *   9. pilot blend         -> out = pilot + auto*(1 - BLEND% * |stick|);
 *                             the pilot term is ALWAYS applied at full
 *                             authority, BLEND only attenuates the auto part
 *
 * SIGN CONVENTION: positive roll = right bank -> positive yaw rate = nose
 * right (clockwise from above), matching ArduPilot conventions.
 *
 * TESTS: Tools/autotest/helicopter.py BankSteerDisabled / BankSteerAssist.
 * DOCS:  arducopter-tradheli-boildown repo CLAUDE.md and the
 *        claude-session-memory/ directory in the dev workspace.
 * ==========================================================================
 */

// minimum speed used in the coordination calculation to avoid large yaw
// rate commands at low speed
#define HELI_BANK_STEER_SPEED_FLOOR_MS  3.0f

// forward speed at which the assist starts to fade in and is fully active
#define HELI_BANK_STEER_FWD_SPEED_MIN_MS  2.0f
#define HELI_BANK_STEER_FWD_SPEED_FULL_MS 4.0f

// bank angle used in the coordination formula is limited to this value
#define HELI_BANK_STEER_BANK_LIMIT_RAD  radians(60.0f)

// no assist beyond this commanded bank angle (aerobatics / inverted flight)
#define HELI_BANK_STEER_BANK_MAX_RAD    radians(80.0f)

// time constant for smoothing the automatic yaw rate command
#define HELI_BANK_STEER_TC_SEC          0.5f

const AP_Param::GroupInfo HeliBankSteer::var_info[] = {

    // @Param: STEER
    // @DisplayName: Helicopter coordinated turn assist enable
    // @Description: Enables bank angle steering (coordinated turn assist) for traditional helicopters. When enabled, an automatic yaw rate command is derived from the commanded bank angle in Stabilize, AltHold and Loiter so that turns in forward flight are coordinated without pilot rudder input. This is a pilot input assistance layer only, it is not a navigation system. It has no effect while hovering, when landed, or in any other flight mode. Pilot rudder input always retains full authority.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("STEER", 1, HeliBankSteer, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: GAIN
    // @DisplayName: Coordinated turn assist gain
    // @Description: Scale factor applied to the physically ideal coordinated turn yaw rate (GRAVITY x tan(bank) / speed). 1.0 requests the theoretically correct coordination for the current speed estimate. Reduce if the tail leads the turn, increase if the tail lags.
    // @Range: 0 2
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("GAIN", 2, HeliBankSteer, _gain, 1.0f),

    // @Param: YAWMAX
    // @DisplayName: Coordinated turn assist maximum yaw rate
    // @Description: Maximum automatic yaw rate commanded by the coordinated turn assist. Pilot rudder input can always add to this.
    // @Units: deg/s
    // @Range: 0 120
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("YAWMAX", 3, HeliBankSteer, _yaw_max_degs, 30.0f),

    // @Param: BLEND
    // @DisplayName: Coordinated turn assist pilot blend
    // @Description: Percentage of the automatic yaw command that is removed at full rudder deflection, scaling linearly with stick position. 100 means full rudder deflection completely replaces the automatic yaw command, 0 means the automatic command is purely added to pilot input. Pilot input is always applied at full authority regardless of this setting.
    // @Units: %
    // @Range: 0 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("BLEND", 4, HeliBankSteer, _blend_pct, 50.0f),

    // @Param: DB
    // @DisplayName: Coordinated turn assist bank angle deadband
    // @Description: Commanded bank angles below this produce no automatic yaw, avoiding unwanted yaw from small roll corrections. The assist ramps in linearly between this angle and twice this angle.
    // @Units: deg
    // @Range: 0 30
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("DB", 5, HeliBankSteer, _deadband_deg, 5.0f),

    AP_GROUPEND
};

HeliBankSteer::HeliBankSteer()
{
    AP_Param::setup_object_defaults(this, var_info);
}

float HeliBankSteer::update_rads(float pilot_yaw_rate_rads, float pilot_yaw_input_norm, float bank_target_rad, bool coordination_active)
{
    if (!enabled()) {
        // feature disabled: pass pilot input through unchanged
        _auto_yaw_rate_rads = 0.0f;
        _last_update_ms = 0;
        return pilot_yaw_rate_rads;
    }

    // detect gaps in updates (mode without assist, or feature just
    // enabled) and restart smoothly from zero
    const uint32_t now_ms = AP_HAL::millis();
    float dt = (now_ms - _last_update_ms) * 1.0e-3f;
    if (_last_update_ms == 0 || dt > 0.2f) {
        _auto_yaw_rate_rads = 0.0f;
        dt = 0.0f;
    }
    _last_update_ms = now_ms;

    // desired automatic coordination yaw rate
    float target_rads = 0.0f;
    if (coordination_active && fabsf(bank_target_rad) < HELI_BANK_STEER_BANK_MAX_RAD) {

        // bank angle deadband with a linear ramp to avoid a command step
        const float bank_deg = degrees(fabsf(bank_target_rad));
        const float db_deg = MAX(_deadband_deg.get(), 0.0f);
        float bank_ramp = 1.0f;
        if (bank_deg <= db_deg) {
            bank_ramp = 0.0f;
        } else if (bank_deg < 2.0f * db_deg) {
            bank_ramp = (bank_deg - db_deg) / db_deg;
        }

        if (is_positive(bank_ramp)) {
            const AP_AHRS &ahrs = AP::ahrs();

            // speed for the coordination formula: true airspeed estimate
            // if available, otherwise ground speed
            float speed_ms;
            if (!ahrs.airspeed_estimate_true(speed_ms)) {
                speed_ms = ahrs.groundspeed();
            }
            speed_ms = MAX(speed_ms, HELI_BANK_STEER_SPEED_FLOOR_MS);

            // coordination is for forward flight only: fade in with the
            // body-frame forward component of the ground velocity so that
            // hover and sideways flight produce no automatic yaw
            const Vector2f gs_vec = ahrs.groundspeed_vector();
            const float yaw_rad = ahrs.get_yaw();
            const float fwd_speed_ms = gs_vec * Vector2f(cosf(yaw_rad), sinf(yaw_rad));
            const float fwd_ramp = constrain_float((fwd_speed_ms - HELI_BANK_STEER_FWD_SPEED_MIN_MS) /
                                                   (HELI_BANK_STEER_FWD_SPEED_FULL_MS - HELI_BANK_STEER_FWD_SPEED_MIN_MS),
                                                   0.0f, 1.0f);

            // physically ideal coordinated turn yaw rate: g * tan(bank) / speed
            const float bank_lim_rad = constrain_float(bank_target_rad, -HELI_BANK_STEER_BANK_LIMIT_RAD, HELI_BANK_STEER_BANK_LIMIT_RAD);
            target_rads = _gain * bank_ramp * fwd_ramp * GRAVITY_MSS * tanf(bank_lim_rad) / speed_ms;

            const float yaw_max_rads = radians(constrain_float(_yaw_max_degs.get(), 0.0f, 120.0f));
            target_rads = constrain_float(target_rads, -yaw_max_rads, yaw_max_rads);
        }
    }

    // smooth the automatic command to avoid steps on activation and
    // state transitions
    if (is_positive(dt)) {
        const float alpha = dt / (dt + HELI_BANK_STEER_TC_SEC);
        _auto_yaw_rate_rads += (target_rads - _auto_yaw_rate_rads) * alpha;
    }

    // pilot rudder deflection attenuates the automatic component; pilot
    // input itself is always summed at full authority
    const float blend = constrain_float(_blend_pct.get() * 0.01f, 0.0f, 1.0f);
    const float atten = 1.0f - blend * constrain_float(fabsf(pilot_yaw_input_norm), 0.0f, 1.0f);

    return pilot_yaw_rate_rads + _auto_yaw_rate_rads * atten;
}

#endif  // FRAME_CONFIG == HELI_FRAME
