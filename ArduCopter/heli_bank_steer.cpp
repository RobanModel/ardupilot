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
 *                             (armed && spooled && !landed);
 *                             plus |commanded bank| < 80 deg here
 *   4. bank deadband ramp  -> 0 inside HELI_BANK_DB deg, linear to 1 at 2xDB
 *   5. speed fade (v2.4)   -> uses |body-axis longitudinal groundspeed|:
 *                             strictly 0 below HELI_BANK_SPD, full at
 *                             HELI_BANK_SPDFUL.  The shape between them is
 *                             a monotone-limited cubic (PCHIP) through
 *                             (0,0),(1,1) and three offset-from-linear
 *                             points set by HELI_BANK_SPD_25/50/75 (in %;
 *                             0/0/0 = linear).  Purely speed dependent,
 *                             no time latch.  Hover and sideways flight
 *                             => no automatic yaw.
 *   6. physics             -> ideal coordinated-turn rate g*tan(bank)/V,
 *                             V = airspeed estimate if available else
 *                             groundspeed, floored 0.5 m/s; bank clamped
 *                             +-60 deg.  Flying TAIL-FIRST the yaw is
 *                             INVERTED (v2.3, velocity-sign triggered) so
 *                             backward circles stay banked toward the
 *                             centre.  (v2 tried this with a different
 *                             speed source + the trim-bleed bug present
 *                             and was rolled back; v2.3 is the clean
 *                             retry approved after the v2.2 fix.)
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

// minimum speed used in the coordination formula: keeps g*tan(bank)/speed
// finite near the HELI_BANK_SPD threshold; the YAWMAX clamp bounds the
// result anyway, this only avoids float extremes
#define HELI_BANK_STEER_SPEED_FLOOR_MS  0.5f

// bank angle used in the coordination formula is limited to this value
#define HELI_BANK_STEER_BANK_LIMIT_RAD  radians(60.0f)

// no assist beyond this commanded bank angle (aerobatics / inverted flight)
#define HELI_BANK_STEER_BANK_MAX_RAD    radians(80.0f)

// time constant for smoothing the automatic yaw rate command
#define HELI_BANK_STEER_TC_SEC          0.5f

const AP_Param::GroupInfo HeliBankSteer::var_info[] = {

    // @Param: STEER
    // @DisplayName: Helicopter coordinated turn assist enable
    // @Description: Enables bank angle steering (coordinated turn assist) for traditional helicopters. When enabled, an automatic yaw rate command is derived from the commanded bank angle in Stabilize, AltHold and Loiter so that banked turns are coordinated without pilot rudder input, in forward and backward flight (flying tail-first the automatic yaw direction is inverted so the circle stays banked toward its centre). This is a pilot input assistance layer only, it is not a navigation system. It has no effect below HELI_BANK_SPD longitudinal speed (hover and sideways flight get no automatic yaw), when landed, or in any other flight mode. Pilot rudder input always retains full authority.
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

    // @Param: SPD
    // @DisplayName: Coordinated turn assist zero-yaw speed
    // @Description: Below this body-axis longitudinal speed (forward or backward) the assist commands strictly zero automatic yaw. The assist fades in between this speed and HELI_BANK_SPDFUL. Raise this if hover velocity noise causes unwanted yaw.
    // @Units: m/s
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("SPD", 6, HeliBankSteer, _spd_min_ms, 0.5f),

    // @Param: SPDFUL
    // @DisplayName: Coordinated turn assist full-effect speed
    // @Description: Body-axis longitudinal speed (forward or backward) at which the assist reaches full effect. Must be above HELI_BANK_SPD; values at or below it are treated as HELI_BANK_SPD + 0.1.
    // @Units: m/s
    // @Range: 0.1 15
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("SPDFUL", 7, HeliBankSteer, _spd_full_ms, 4.0f),

    // index 8 was SPDEXP (single-exponent fade shaping), replaced by the
    // three-point offset curve below in v2.4 -- do not reuse index 8

    // @Param: SPD_25
    // @DisplayName: Coordinated turn assist fade curve offset at 25%
    // @Description: Offset of the assist ratio from linear at 25% of the speed band between HELI_BANK_SPD and HELI_BANK_SPDFUL, in percent. Positive values make the assist come in earlier/stronger at low speed. 0,0,0 in all three offsets gives a linear fade. The curve always starts at 0 and ends at full effect; a shape-preserving spline through the three offset points avoids overshoot.
    // @Units: %
    // @Range: -75 75
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("SPD_25", 9, HeliBankSteer, _crv_ofs_25, 0),

    // @Param: SPD_50
    // @DisplayName: Coordinated turn assist fade curve offset at 50%
    // @Description: Offset of the assist ratio from linear at 50% of the speed band between HELI_BANK_SPD and HELI_BANK_SPDFUL, in percent.
    // @Units: %
    // @Range: -75 75
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("SPD_50", 10, HeliBankSteer, _crv_ofs_50, 0),

    // @Param: SPD_75
    // @DisplayName: Coordinated turn assist fade curve offset at 75%
    // @Description: Offset of the assist ratio from linear at 75% of the speed band between HELI_BANK_SPD and HELI_BANK_SPDFUL, in percent.
    // @Units: %
    // @Range: -75 75
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("SPD_75", 11, HeliBankSteer, _crv_ofs_75, 0),

    AP_GROUPEND
};

// rebuild the cached fade-curve interpolant when the offset parameters
// change.  Five control points at u = 0, 0.25, 0.5, 0.75, 1 with fixed
// endpoints (0,0) and (1,1); the three interior points are offset from
// the linear response by the SPD_25/50/75 parameters.  Tangents use the
// Fritsch-Carlson (PCHIP) limiter: shape preserving, no overshoot
// between points, flat neighbouring points give a flat section.
void HeliBankSteer::update_speed_curve()
{
    const float ofs[3] = { _crv_ofs_25.get(), _crv_ofs_50.get(), _crv_ofs_75.get() };
    if (is_equal(ofs[0], _crv_last[0]) && is_equal(ofs[1], _crv_last[1]) && is_equal(ofs[2], _crv_last[2])) {
        return;
    }
    _crv_last[0] = ofs[0];
    _crv_last[1] = ofs[1];
    _crv_last[2] = ofs[2];

    _crv_y[0] = 0.0f;
    _crv_y[4] = 1.0f;
    for (uint8_t i = 0; i < 3; i++) {
        const float lin = 0.25f * (i + 1);
        _crv_y[i + 1] = constrain_float(lin + ofs[i] * 0.01f, 0.0f, 1.0f);
    }

    // secant slopes of the four uniform intervals (h = 0.25)
    float d[4];
    for (uint8_t i = 0; i < 4; i++) {
        d[i] = (_crv_y[i + 1] - _crv_y[i]) * 4.0f;
    }

    // endpoint tangents equal the adjacent secants (monotone safe);
    // interior tangents use the harmonic mean, zeroed where the data
    // turns or flattens
    _crv_m[0] = d[0];
    _crv_m[4] = d[3];
    for (uint8_t i = 1; i < 4; i++) {
        if (d[i - 1] * d[i] <= 0.0f) {
            _crv_m[i] = 0.0f;
        } else {
            _crv_m[i] = 2.0f * d[i - 1] * d[i] / (d[i - 1] + d[i]);
        }
    }
}

// evaluate the cached fade curve with a cubic Hermite step in the
// interval containing u
float HeliBankSteer::apply_speed_curve(float u) const
{
    u = constrain_float(u, 0.0f, 1.0f);
    uint8_t k = (uint8_t)(u * 4.0f);
    if (k > 3) {
        k = 3;
    }
    const float t = (u - 0.25f * k) * 4.0f;
    const float t2 = t * t;
    const float t3 = t2 * t;
    const float h00 = 2.0f * t3 - 3.0f * t2 + 1.0f;
    const float h10 = t3 - 2.0f * t2 + t;
    const float h01 = -2.0f * t3 + 3.0f * t2;
    const float h11 = t3 - t2;
    const float y = h00 * _crv_y[k] + h10 * 0.25f * _crv_m[k] +
                    h01 * _crv_y[k + 1] + h11 * 0.25f * _crv_m[k + 1];
    return constrain_float(y, 0.0f, 1.0f);
}

HeliBankSteer::HeliBankSteer()
{
    AP_Param::setup_object_defaults(this, var_info);
    update_speed_curve();
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

            // SIGNED body-axis longitudinal groundspeed component:
            // positive nose-first, negative tail-first.  v2.3: the assist
            // works in forward AND backward flight; sideways flight has
            // ~zero longitudinal speed and therefore zero assist.
            const Vector2f gs_vec = ahrs.groundspeed_vector();
            const float yaw_rad = ahrs.get_yaw();
            const float lon_speed_ms = gs_vec * Vector2f(cosf(yaw_rad), sinf(yaw_rad));

            // speed fade: strictly zero below HELI_BANK_SPD, full effect
            // at HELI_BANK_SPDFUL, curve shaped by HELI_BANK_SPDEXP
            // (1 = linear).  Applies symmetrically to forward and backward
            // flight; the below-SPD region doubles as a dead zone around
            // zero speed so the direction inversion cannot chatter.
            const float spd_min = MAX(_spd_min_ms.get(), 0.0f);
            const float spd_full = MAX(_spd_full_ms.get(), spd_min + 0.1f);
            const float band_pos = constrain_float((fabsf(lon_speed_ms) - spd_min) / (spd_full - spd_min), 0.0f, 1.0f);
            update_speed_curve();
            const float spd_ramp = apply_speed_curve(band_pos);

            // speed for the coordination formula: true airspeed estimate
            // if available, otherwise ground speed (as in v1)
            float speed_ms;
            if (!ahrs.airspeed_estimate_true(speed_ms)) {
                speed_ms = ahrs.groundspeed();
            }
            speed_ms = MAX(speed_ms, HELI_BANK_STEER_SPEED_FLOOR_MS);

            // physically ideal coordinated turn yaw rate: g * tan(bank) / speed
            const float bank_lim_rad = constrain_float(bank_target_rad, -HELI_BANK_STEER_BANK_LIMIT_RAD, HELI_BANK_STEER_BANK_LIMIT_RAD);
            target_rads = _gain * bank_ramp * spd_ramp * GRAVITY_MSS * tanf(bank_lim_rad) / speed_ms;

            // tail-first flight (v2.3): invert the automatic yaw so a
            // banked backward circle stays banked toward its centre.
            // Triggered by the measured velocity sign, NOT the pitch
            // stick: while decelerating from forward flight the machine
            // still moves forward and must keep forward-sense steering.
            if (is_negative(lon_speed_ms)) {
                target_rads = -target_rads;
            }

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
