#pragma once
//
// ThermalCal -- per-channel thermal-offset SHAPE for the five-hole probe.
//
// The oven-cal model gives the sensor's zero-flow offset as a function of its
// own die temperature:  off(T) = c0 + T*(c1 + T*c2)   [Horner; c2=0 => linear].
//
// It is applied as a SHAPE referenced to the temperature at which the operational
// `flow_angle null` was captured:
//
//     correction(T) = off(T) - off(T_null)
//
// so the absolute term c0 (and any baseline set by mounting stress or the power
// cycle) cancels in the difference -- that level is owned by `null`, and only the
// durable thermal SHAPE (c1, c2) comes from this table. See apply_offset().
//
// T is CLAMPED to the validated soak band [tmin,tmax] before evaluation: the
// polynomial is never extrapolated past the data it was fit on. A die temp above
// tmax (e.g. a hot avionics bay beyond the bench soak) freezes the correction at
// its hottest validated value and is surfaced by clamped_hot() in status.
//
// Fail-safe: a default / rejected cal is !valid(), and callers skip it (identity).
// Header-only, float-only (strict-flag clean), no heap, no PX4 deps.

#include <stdint.h>

class ThermalCal
{
public:
	ThermalCal() = default;

	// Validate + store. order: 1 (linear, c2 ignored) or 2 (quadratic).
	// Returns false and stays !valid() on any bad input (caller then runs
	// that channel thermally-uncalibrated -- level-only null, no shape).
	bool set(uint8_t order, float c0, float c1, float c2, float tmin, float tmax)
	{
		_valid = false;
		if ((order != 1 && order != 2) ||
		    !finite(c0) || !finite(c1) || !finite(c2) ||
		    !finite(tmin) || !finite(tmax) || !(tmax > tmin)) {
			return false;
		}
		_c0 = c0; _c1 = c1; _c2 = (order == 2) ? c2 : 0.f;
		_tmin = tmin; _tmax = tmax; _valid = true;
		return true;
	}

	bool  valid() const { return _valid; }
	float tmin()  const { return _tmin; }
	float tmax()  const { return _tmax; }

	// Absolute modelled offset at die temp t [degC], clamped to the fit band.
	float eval(float t) const
	{
		const float tc = (t < _tmin) ? _tmin : (t > _tmax) ? _tmax : t;
		return _c0 + tc * (_c1 + tc * _c2);   // Horner
	}

	// Shape correction to SUBTRACT from the reading: off(t) - off(t_ref).
	// c0 cancels here -- only curvature/slope between t and t_ref survives.
	float delta(float t, float t_ref) const { return eval(t) - eval(t_ref); }

	// True when the die is hotter than the validated band (correction frozen at
	// tmax). Drives a status flag so an out-of-band bay shows up in the log.
	bool clamped_hot(float t) const { return _valid && t > _tmax; }

private:
	static bool finite(float x) { return (x == x) && (x < 1e30f) && (x > -1e30f); }

	float   _c0{0.f}, _c1{0.f}, _c2{0.f};
	float   _tmin{0.f}, _tmax{0.f};
	bool    _valid{false};
};
