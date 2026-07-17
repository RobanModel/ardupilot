#pragma once

/*
  HeliBankSteer: optional coordinated turn assist ("bank angle steering")
  for traditional helicopters.

  When enabled, an automatic yaw (tail rotor) rate command is derived
  from the commanded bank angle so that banked turns are coordinated,
  similar to the simplified "bank steering" control style found on some
  ready-to-fly scale helicopters.  Since v2 the assist is purely speed
  dependent (fade between HELI_BANK_SPD and HELI_BANK_SPDFUL, shaped by
  HELI_BANK_SPDEXP, no time latch).  Works in forward and backward
  flight: flying tail-first the automatic yaw direction is inverted so a
  banked backward circle stays banked toward its centre (v2.3).  Hover
  and sideways flight produce no automatic yaw.
  This is an input assistance layer only: it shapes the yaw rate command
  handed to the attitude controller and does not modify attitude or rate
  control loops.

  The assist is applied by Mode::get_pilot_desired_yaw_rate_rads() in
  modes that opt in via Mode::allows_coordinated_turn_assist()
  (Stabilize, AltHold, Loiter).  Pilot rudder input is always summed at
  full authority; stick deflection additionally attenuates the automatic
  component so the pilot can override it.
 */

#include <AP_Param/AP_Param.h>

class HeliBankSteer {
public:
    HeliBankSteer();

    CLASS_NO_COPY(HeliBankSteer);

    // true if the coordinated turn assist has been enabled by parameter
    bool enabled() const { return _enable != 0; }

    // combine the pilot's yaw rate command with the automatic
    // coordinated-turn yaw rate.
    //   pilot_yaw_rate_rads: pilot's desired yaw rate (expo and rate limit already applied)
    //   pilot_yaw_input_norm: pilot's rudder stick position in the range -1..1 (deadzone applied)
    //   bank_target_rad: the roll angle currently commanded to the attitude controller
    //   coordination_active: true when the vehicle state allows coordination (flying, spooled up, in dynamic flight)
    // returns the yaw rate command in rad/s to pass to the attitude controller
    float update_rads(float pilot_yaw_rate_rads, float pilot_yaw_input_norm, float bank_target_rad, bool coordination_active);

    static const struct AP_Param::GroupInfo var_info[];

private:
    // parameters
    AP_Int8 _enable;            // HELI_BANK_STEER: 0 = disabled, 1 = enabled
    AP_Float _gain;             // HELI_BANK_GAIN: scale on the physical coordinated-turn yaw rate
    AP_Float _yaw_max_degs;     // HELI_BANK_YAWMAX: maximum automatic yaw rate in deg/s
    AP_Float _blend_pct;        // HELI_BANK_BLEND: % of automatic yaw removed at full rudder deflection
    AP_Float _deadband_deg;     // HELI_BANK_DB: bank angle deadband in degrees
    AP_Float _spd_min_ms;       // HELI_BANK_SPD: below this |longitudinal speed| the auto yaw is strictly zero
    AP_Float _spd_full_ms;      // HELI_BANK_SPDFUL: |longitudinal speed| of full assist effect
    AP_Float _crv_ofs_25;       // HELI_BANK_SPD_25: fade curve offset from linear at 25% of the band, in %
    AP_Float _crv_ofs_50;       // HELI_BANK_SPD_50: fade curve offset from linear at 50% of the band, in %
    AP_Float _crv_ofs_75;       // HELI_BANK_SPD_75: fade curve offset from linear at 75% of the band, in %

    // recompute the cached fade-curve interpolant if the offset
    // parameters changed
    void update_speed_curve();

    // evaluate the fade curve at normalized band position u (0..1),
    // returns the assist ratio 0..1
    float apply_speed_curve(float u) const;

    // state
    float _auto_yaw_rate_rads;  // low-pass filtered automatic yaw rate
    uint32_t _last_update_ms;   // time of last update, used to detect activation gaps

    // cached monotone-cubic (PCHIP) fade curve through five control
    // points at u = 0, 0.25, 0.5, 0.75, 1
    float _crv_y[5];            // control point values (clamped 0..1)
    float _crv_m[5];            // limited Hermite tangents
    float _crv_last[3] = {-1e9f, -1e9f, -1e9f};  // offsets the cache was built for
};
