#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Joe Fernandez
"""Yaw-authority metrics for a multirotor or VTOL hover log: does the yaw
rate follow the demand, how far does heading drift hands-off, how hard is the
yaw controller working, are the motors saturating, and what does the
CW-minus-CCW motor split actually answer to.

  python3 vtol-yaw-metrics.py LOG.bin --cw 2,3 --ccw 1,4 \\
      [--phases phases.json] [--json out.json]
  python3 vtol-yaw-metrics.py --selftest

Needs numpy and pymavlink (the log is read with pymavlink; --selftest needs
only numpy). Run it with a Python that has both.

--cw and --ccw name motors by number (Motor1 = SERVOn_FUNCTION 33, Motor2 =
34, ...) as the props are physically built, which is the point: the split is
regressed against the controller's roll, pitch and yaw outputs, so a split
that loads on YOut means differential torque is the yaw actuator, and one that
loads on ROut means a roll demand also yaws the aircraft. The RCOU channel for
each motor is found from the log's own SERVOn_FUNCTION.

Windows. With --phases (written by sitl-qhover-steps.py), the hold and
step phases come from it. Without, the flight window is where every motor is
above its Q_M_SPIN_MIN (or MOT_SPIN_MIN) PWM, and the hands-off window is the
longest span inside it with roll, pitch and yaw sticks (RCIN, through
RCMAP_*) inside their RCn_DZ of RCn_TRIM.

Fields, as logged by Plane-4.7.1 and Copter-4.7.1: RATE (YDes, Y, YOut,
ROut, POut; deg/s and +/-1 outputs, AC_AttitudeControl_Logging.cpp), PIQY on
a quadplane or PIDY on a copter (I, Flags bit 0 = the mixer's yaw limit,
AP_Logger/LogFile.cpp log_PID_Flags), RCOU, ATT.Yaw, MOTB.ThLimit.

Reads positions from nothing and prints none.
"""
import argparse
import json
import sys

np = None  # imported in main() after the arguments, so --help needs no numpy

WANT = ('PARM', 'RATE', 'PIQY', 'PIDY', 'RCOU', 'RCIN', 'ATT', 'MOTB', 'MODE')


def load_log(path):
    """(data, params): the WANT messages as lists of dicts, and PARM as a dict."""
    from pymavlink import mavutil
    data = {k: [] for k in WANT}
    params = {}
    mlog = mavutil.mavlink_connection(path)
    while True:
        msg = mlog.recv_match(type=list(WANT))
        if msg is None:
            break
        t = msg.get_type()
        if t == 'PARM':
            params[msg.Name] = msg.Value
        else:
            data[t].append(msg.to_dict())
    return data, params


def analyse(data, params, CW, CCW, phases=None, window=None, label=''):
    """The metrics dict for one log. `data` maps a message type to a list of
    row dicts (the shape load_log returns), `params` maps name to value,
    `phases` is the 'phases' list of a sitl-qhover-steps.py file, `window` an
    optional (start, end) in log seconds for the no-phases flight window."""

    def arr(t, field):
        return np.array([r[field] for r in data[t]], dtype=float)

    def p(*names, default=None):
        for n in names:
            if n in params:
                return params[n]
        return default

    if not data['RATE'] or not data['RCOU']:
        sys.exit("FAIL: log has no RATE or no RCOU")
    PID = 'PIQY' if data['PIQY'] else 'PIDY'


    # motor number -> RCOU channel, from the log's own SERVOn_FUNCTION
    chan = {}
    for n in range(1, 17):
        f = params.get(f'SERVO{n}_FUNCTION')
        if f is not None and 33 <= f <= 40:
            chan.setdefault(int(f) - 32, n)
    missing = [mn for mn in CW + CCW if mn not in chan]
    if missing:
        sys.exit(f"FAIL: no SERVOn_FUNCTION for motor(s) {missing}")

    pwm_min = p('Q_M_PWM_MIN', 'MOT_PWM_MIN', default=1000) or 1000
    pwm_max = p('Q_M_PWM_MAX', 'MOT_PWM_MAX', default=2000) or 2000
    spin_min = p('Q_M_SPIN_MIN', 'MOT_SPIN_MIN', default=0.15)
    spin_max = p('Q_M_SPIN_MAX', 'MOT_SPIN_MAX', default=0.95)
    pwm_lo = pwm_min + spin_min * (pwm_max - pwm_min)
    pwm_hi = pwm_min + spin_max * (pwm_max - pwm_min)

    t_rate = arr('RATE', 'TimeUS') / 1e6
    t_rcou = arr('RCOU', 'TimeUS') / 1e6
    mot = {mn: arr('RCOU', f'C{chan[mn]}') for mn in CW + CCW}
    allm = np.vstack([mot[mn] for mn in sorted(mot)])

    # ---------------------------------------------------------------- windows
    windows = {}
    if phases:
        by = {ph['name']: (ph['start_ms'] / 1e3, ph['end_ms'] / 1e3)
              for ph in phases if ph.get('end_ms') is not None}
        windows['hold'] = by['hold']
        windows['yaw_pos'] = by['yaw_pos']
        windows['yaw_neg'] = by['yaw_neg']
        windows['yaw_steps'] = (by['yaw_pos'][0], by['yaw_neg'][1])
        windows['roll_pos'] = by['roll_pos']
        windows['roll_neg'] = by['roll_neg']
        windows['pitch_pos'] = by['pitch_pos']
        windows['pitch_neg'] = by['pitch_neg']
        windows['airborne'] = (by['hold'][0], by['settle_pitch'][1])
    else:
        spinning = np.all(allm > pwm_lo + 5, axis=0)
        if not spinning.any():
            sys.exit("FAIL: no span with every motor above spin_min")
        idx = np.flatnonzero(spinning)
        windows['airborne'] = (t_rcou[idx[0]], t_rcou[idx[-1]])
        if window:
            windows['airborne'] = (max(windows['airborne'][0], window[0]),
                                   min(windows['airborne'][1], window[1]))
        # hands-off: sticks inside their deadzones
        if data['RCIN']:
            t_in = arr('RCIN', 'TimeUS') / 1e6
            centred = np.ones(len(t_in), bool)
            for axis in ('ROLL', 'PITCH', 'YAW'):
                ch = int(p(f'RCMAP_{axis}', default={'ROLL': 1, 'PITCH': 2,
                                                      'YAW': 4}[axis]))
                trim = p(f'RC{ch}_TRIM', default=1500)
                dz = p(f'RC{ch}_DZ', default=30)
                centred &= np.abs(arr('RCIN', f'C{ch}') - trim) <= dz
            lo, hi = windows['airborne']
            centred &= (t_in >= lo) & (t_in <= hi)
            best, start = (0, None, None), None
            for i, c in enumerate(centred):
                if c and start is None:
                    start = i
                if (not c or i == len(centred) - 1) and start is not None:
                    end = i if c else i - 1
                    if t_in[end] - t_in[start] > best[0]:
                        best = (t_in[end] - t_in[start], t_in[start], t_in[end])
                    start = None
            if best[1] is not None:
                windows['hold'] = (best[1], best[2])


    def sel(t, w):
        return (t >= w[0]) & (t <= w[1])


    ydes, y = arr('RATE', 'YDes'), arr('RATE', 'Y')
    yout, rout, pout = arr('RATE', 'YOut'), arr('RATE', 'ROut'), arr('RATE', 'POut')

    out = {'label': label, 'log_rate_hz': round(1 / np.median(np.diff(t_rate)), 1),
           'rcou_hz': round(1 / np.median(np.diff(t_rcou)), 1),
           'pid_message': PID,
           'frame': {k: params.get(k) for k in ('Q_FRAME_CLASS', 'Q_FRAME_TYPE',
                                                 'FRAME_CLASS', 'FRAME_TYPE')
                     if k in params},
           'motor_channel': chan, 'cw': CW, 'ccw': CCW,
           'pwm_thrust_range': [pwm_lo, pwm_hi],
           'windows_s': {k: [round(v[0], 2), round(v[1], 2)]
                         for k, v in windows.items()}}

    # ------------------------------------------------------------ yaw tracking
    track = {}
    for w in ('airborne', 'hold', 'yaw_steps'):
        if w in windows:
            s = sel(t_rate, windows[w])
            track[w] = {'rms_err_dps': float(np.sqrt(np.mean((ydes[s] - y[s]) ** 2))),
                        'max_abs_rate_dps': float(np.max(np.abs(y[s])))}
    steps = {}
    for w in ('yaw_pos', 'yaw_neg'):
        if w in windows:
            lo, hi = windows[w]
            s = sel(t_rate, (lo + (hi - lo) / 2, hi))     # the settled half
            d, r = float(np.mean(ydes[s])), float(np.mean(y[s]))
            ratio = r / d if abs(d) > 1e-6 else float('nan')
            steps[w] = {'demand_dps': d, 'achieved_dps': r, 'ratio': ratio,
                        'followed': bool(abs(d) > 5 and ratio >= 0.7)}
    track['steps'] = steps
    out['yaw_tracking'] = track

    # --------------------------------------------------------- heading drift
    if 'hold' in windows:
        lo, hi = windows['hold']
        t_att = arr('ATT', 'TimeUS') / 1e6
        s = sel(t_att, (lo, hi))
        yaw_att = np.degrees(np.unwrap(np.radians(arr('ATT', 'Yaw')[s])))
        sr = sel(t_rate, (lo, hi))
        integ = float(np.trapezoid(y[sr], t_rate[sr])) if sr.sum() > 1 else float('nan')
        out['heading_hold'] = {
            'seconds': round(hi - lo, 1),
            'att_yaw_change_deg': float(yaw_att[-1] - yaw_att[0]) if s.sum() > 1 else None,
            'rate_y_integral_deg': integ,
            'max_abs_rate_y_dps': float(np.max(np.abs(y[sr]))) if sr.any() else None,
            'end_rate_y_dps': float(np.mean(y[sr][-30:])) if sr.any() else None}

    # ------------------------------------------------- yaw output and I term
    t_pid = arr(PID, 'TimeUS') / 1e6
    ipid = arr(PID, 'I')
    flags = arr(PID, 'Flags').astype(int) if 'Flags' in data[PID][0] else None
    yo = {}
    for w in ('airborne', 'hold', 'yaw_steps'):
        if w not in windows:
            continue
        s = sel(t_rate, windows[w])
        sp = sel(t_pid, windows[w])
        yo[w] = {'yout_mean': float(np.mean(yout[s])),
                 'yout_max_abs': float(np.max(np.abs(yout[s]))),
                 'i_start': float(ipid[sp][0]), 'i_end': float(ipid[sp][-1]),
                 'i_max_abs': float(np.max(np.abs(ipid[sp]))),
                 'yaw_limit_flag_pct': (float(100 * np.mean(flags[sp] & 1))
                                        if flags is not None else None)}
    out['yaw_effort'] = yo
    out['yaw_imax'] = p('Q_A_RAT_YAW_IMAX', 'ATC_RAT_YAW_IMAX')

    # ------------------------------------------------------- motor saturation
    sat = {}
    for w in ('airborne', 'hold', 'yaw_steps'):
        if w not in windows:
            continue
        s = sel(t_rcou, windows[w])
        m_ = allm[:, s]
        sat[w] = {'any_at_top_pct': float(100 * np.mean(np.any(m_ >= pwm_hi - 2, axis=0))),
                  'any_at_bottom_pct': float(100 * np.mean(np.any(m_ <= pwm_lo + 2, axis=0))),
                  'pwm_min_seen': float(m_.min()), 'pwm_max_seen': float(m_.max())}
        if data['MOTB']:
            # ThLimit is the throttle limit itself, 0 to 1 (1 = none), not a flag
            tm = arr('MOTB', 'TimeUS') / 1e6
            sm = sel(tm, windows[w])
            sat[w]['motb_thlimit_below_099_pct'] = (
                float(100 * np.mean(arr('MOTB', 'ThLimit')[sm] < 0.99)) if sm.any() else None)
    out['saturation'] = sat

    # -------------------------------------------- CW minus CCW split regression
    s = sel(t_rcou, windows['airborne'])
    split = (np.mean([mot[mn] for mn in CW], axis=0)
             - np.mean([mot[mn] for mn in CCW], axis=0))[s]
    X = np.column_stack([np.ones(s.sum())] +
                        [np.interp(t_rcou[s], t_rate, v) for v in (rout, pout, yout)])
    coef, *_ = np.linalg.lstsq(X, split, rcond=None)
    pred = X @ coef
    ss_res = float(np.sum((split - pred) ** 2))
    ss_tot = float(np.sum((split - split.mean()) ** 2))
    corr = {}
    for i, name in enumerate(('ROut', 'POut', 'YOut'), 1):
        c = np.corrcoef(X[:, i], split)[0, 1] if np.std(X[:, i]) > 0 else float('nan')
        corr[name] = float(c)
    # Each output's share of the split's variance it explains on its own terms:
    # |coef| x std(output), so a large coefficient on an output that never moved
    # does not read as a big effect.
    contrib = {name: float(abs(coef[i]) * np.std(X[:, i]))
               for i, name in enumerate(('ROut', 'POut', 'YOut'), 1)}
    out['split'] = {'n': int(s.sum()), 'mean_us': float(split.mean()),
                    'std_us': float(split.std()),
                    'coef_us_per_unit': {'const': float(coef[0]), 'ROut': float(coef[1]),
                                         'POut': float(coef[2]), 'YOut': float(coef[3])},
                    'r2': 1 - ss_res / ss_tot if ss_tot > 0 else float('nan'),
                    'corr': corr, 'contrib_us': contrib,
                'loads_on': split_loads_on(contrib, 1 - ss_res / ss_tot if ss_tot > 0 else float('nan'))}

    # --------------------------------------------- plant gain: accel per output
    # Angular acceleration (RATE actual, smoothed over ~20 ms and differentiated)
    # against the rate controller's outputs, at the lag in 0-150 ms that fits
    # best. Own axis: one regressor, so an output that barely moved cannot borrow
    # a huge coefficient; that number is the plant gain in rad/s^2 per unit of
    # output, the thing a rate tune is tuned against. Yaw coupling: yaw
    # acceleration on YOut and ROut together, reported only where ROut moved
    # (std > 0.01): with side-pair spins a roll demand is also a yaw torque.
    def fit_lagged(acc, regs, dt, k):
        best = None
        for lag in range(0, int(0.15 / dt) + 1, max(1, int(0.01 / dt))):
            A = acc[lag + k:len(acc) - k]
            X = np.column_stack([np.ones(len(A))] + [r[k:len(r) - k - lag] for r in regs])
            c, *_ = np.linalg.lstsq(X, A, rcond=None)
            tot = np.sum((A - A.mean()) ** 2)
            r2 = float(1 - np.sum((A - X @ c) ** 2) / tot) if tot > 0 else 0.0
            if best is None or r2 > best[0]:
                best = (r2, round(lag * dt * 1000), [float(v) for v in c[1:]])
        return best


    def plant(w):
        s = sel(t_rate, w)
        t = t_rate[s]
        if len(t) < 100:
            return None
        dt = float(np.median(np.diff(t)))
        k = max(1, int(round(0.02 / dt)))
        ker = np.ones(k) / k
        acc = {}
        for axis, rate in (('roll', arr('RATE', 'R')[s]), ('pitch', arr('RATE', 'P')[s]),
                           ('yaw', y[s])):
            acc[axis] = np.gradient(np.convolve(np.radians(rate), ker, mode='same'), t)
        res = {}
        for axis, o in (('roll', rout[s]), ('pitch', pout[s]), ('yaw', yout[s])):
            if np.std(o) < 0.005:
                res[axis] = None
                continue
            r2, lag, c = fit_lagged(acc[axis], [o], dt, k)
            res[axis] = {'gain': c[0], 'lag_ms': lag, 'r2': r2, 'out_std': float(np.std(o))}
        if np.std(rout[s]) > 0.01:
            r2, lag, c = fit_lagged(acc['yaw'], [yout[s], rout[s]], dt, k)
            res['yaw_coupling'] = {'per_YOut': c[0], 'per_ROut': c[1], 'lag_ms': lag, 'r2': r2}
        return res


    pw = {}
    for w in ('airborne', 'hold', 'roll_pos', 'pitch_pos', 'yaw_steps'):
        if w in windows:
            lo, hi = windows[w]
            if w in ('roll_pos', 'pitch_pos'):
                # the step pair and the settle after it
                nxt = {'roll_pos': 'roll_neg', 'pitch_pos': 'pitch_neg'}[w]
                hi = windows[nxt][1]
            pw[w.replace('_pos', '_steps')] = plant((lo, hi))
    out['plant_gain_rad_s2'] = pw
    return out


def split_loads_on(contrib, r2):
    """Which controller output the CW-minus-CCW split answers to: the one with
    the largest contribution (|coef| x std), or None when the three explain
    less than half of the split's variance (R2 under 0.5, or no R2)."""
    if not (r2 == r2) or r2 < 0.5:      # nan fails the first test
        return None
    return max(contrib, key=contrib.get)


SPLIT_MEANING = {
    'YOut': 'differential torque is the yaw actuator',
    'ROut': 'a roll demand also moves the yaw-axis split (same-side spin pairs)',
    'POut': 'the pitch output drives the split',
    None: 'no output explains it',
}


def print_report(out, name):
    PID, chan = out['pid_message'], out['motor_channel']
    CW, CCW = out['cw'], out['ccw']
    pwm_lo, pwm_hi = out['pwm_thrust_range']
    track = out['yaw_tracking']
    steps, yo, sat = track['steps'], out['yaw_effort'], out['saturation']
    print(f"{name}")
    print(f"  frame {out['frame']}, {PID}, RATE {out['log_rate_hz']} Hz, RCOU {out['rcou_hz']} Hz")
    print(f"  motors -> RCOU: {chan}; CW {CW}, CCW {CCW}; thrust PWM {pwm_lo:.0f}-{pwm_hi:.0f}")
    for w, v in out['windows_s'].items():
        print(f"  window {w:10} {v[1] - v[0]:6.1f} s")
    for w, v in track.items():
        if w != 'steps':
            print(f"  yaw RMS error {w:10} {v['rms_err_dps']:7.1f} deg/s  (max |Y| {v['max_abs_rate_dps']:.0f})")
    for w, v in steps.items():
        print(f"  {w}: demand {v['demand_dps']:+.1f}, achieved {v['achieved_dps']:+.1f} deg/s "
              f"(ratio {v['ratio']:.2f}) -> {'followed' if v['followed'] else 'NOT followed'}")
    if 'heading_hold' in out:
        h = out['heading_hold']
        print(f"  hold {h['seconds']} s: ATT.Yaw change {h['att_yaw_change_deg']:+.1f} deg, "
              f"integral of RATE.Y {h['rate_y_integral_deg']:+.1f} deg, end rate {h['end_rate_y_dps']:+.1f} deg/s")
    for w, v in yo.items():
        print(f"  {w:10} YOut mean {v['yout_mean']:+.3f} max|{v['yout_max_abs']:.3f}|, "
              f"I {v['i_start']:+.3f} -> {v['i_end']:+.3f} (max|{v['i_max_abs']:.3f}|, IMAX {out['yaw_imax']}), "
              f"yaw limit flag {v['yaw_limit_flag_pct']:.0f}%")
    for w, v in sat.items():
        print(f"  {w:10} motor at top {v['any_at_top_pct']:.1f}%, at bottom {v['any_at_bottom_pct']:.1f}%, "
              f"PWM {v['pwm_min_seen']:.0f}-{v['pwm_max_seen']:.0f}, MOTB ThLimit<0.99 {v.get('motb_thlimit_below_099_pct')}%")
    sp = out['split']
    c = sp['coef_us_per_unit']
    print(f"  split CW-CCW: mean {sp['mean_us']:+.1f} us, std {sp['std_us']:.1f}; "
          f"= {c['const']:+.1f} {c['ROut']:+.0f}*ROut {c['POut']:+.0f}*POut {c['YOut']:+.0f}*YOut, R2 {sp['r2']:.2f}")
    print(f"    corr R {sp['corr']['ROut']:+.2f} P {sp['corr']['POut']:+.2f} Y {sp['corr']['YOut']:+.2f}; "
          f"contribution (|coef| x std) R {sp['contrib_us']['ROut']:.1f} P {sp['contrib_us']['POut']:.1f} "
          f"Y {sp['contrib_us']['YOut']:.1f} us")
    print(f"    split loads on {sp['loads_on']}: {SPLIT_MEANING[sp['loads_on']]}")
    for w, v in out['plant_gain_rad_s2'].items():
        if not v:
            continue
        parts = []
        for axis in ('roll', 'pitch', 'yaw'):
            b = v.get(axis)
            if b:
                parts.append(f"{axis} {b['gain']:.1f} (R2 {b['r2']:.2f}, {b['lag_ms']} ms)")
        line = f"  plant gain {w:11} rad/s2 per unit: " + ', '.join(parts)
        cpl = v.get('yaw_coupling')
        if cpl:
            line += (f"; yaw accel = {cpl['per_YOut']:+.1f}*YOut {cpl['per_ROut']:+.1f}*ROut "
                     f"(R2 {cpl['r2']:.2f})")
        print(line)


def synthetic(follow=1.0, split_on='YOut', motors=(2, 3), ccw=(1, 4)):
    """(data, params, phases) for a 33 s hover log built from known signals:
    the yaw rate is `follow` x its demand, and the CW-minus-CCW motor split is
    driven by the named output ('YOut', 'ROut' or 'POut') alone."""
    t_rate = np.arange(0, 33, 0.01)
    t_rcou = np.arange(0, 33, 0.02)
    names = ['hold', 'yaw_pos', 'yaw_neg', 'settle_yaw', 'roll_pos', 'roll_neg',
             'settle_roll', 'pitch_pos', 'pitch_neg', 'settle_pitch']
    edges = [0, 10, 13, 16, 19, 21, 23, 26, 28, 30, 33]
    phases = [{'name': n, 'start_ms': edges[i] * 1000, 'end_ms': edges[i + 1] * 1000}
              for i, n in enumerate(names)]
    # step edges sit half a sample after the phase edges: the windows include
    # both of their ends, so a sample exactly on an edge belongs to the step
    # that ends there
    ydes = np.where((t_rate > 10.004) & (t_rate < 13.004), 100.0,
                    np.where((t_rate > 13.004) & (t_rate < 16.004), -100.0, 0.0))
    yout = ydes / 200.0
    rout = 0.2 * np.sin(2 * np.pi * 0.7 * t_rate)
    pout = 0.2 * np.sin(2 * np.pi * 1.3 * t_rate)
    drive = {'YOut': yout, 'ROut': rout, 'POut': pout}[split_on]
    d = np.interp(t_rcou, t_rate, drive) * 200.0     # +-us on each side of the split
    mot = {}
    for n in motors:
        mot[n] = 1500.0 + d
    for n in ccw:
        mot[n] = 1500.0 - d
    data = {k: [] for k in WANT}
    for i, t in enumerate(t_rate):
        data['RATE'].append({'TimeUS': t * 1e6, 'YDes': ydes[i], 'Y': follow * ydes[i],
                             'YOut': yout[i], 'ROut': rout[i], 'POut': pout[i],
                             'R': 0.0, 'P': 0.0})
        data['PIDY'].append({'TimeUS': t * 1e6, 'I': 0.01, 'Flags': 0})
        data['ATT'].append({'TimeUS': t * 1e6, 'Yaw': 90.0})
        data['MOTB'].append({'TimeUS': t * 1e6, 'ThLimit': 1.0})
    for i, t in enumerate(t_rcou):
        data['RCOU'].append({'TimeUS': t * 1e6, **{f'C{n}': mot[n][i] for n in mot}})
    params = {f'SERVO{n}_FUNCTION': 32 + n for n in range(1, 5)}
    return data, params, phases


def selftest():
    """Offline checks on synthetic rows; no log file. Needs numpy."""
    n_ok = n_bad = 0

    def check(name, cond):
        nonlocal n_ok, n_bad
        print(f"{'PASS' if cond else 'FAIL'}  {name}")
        if cond:
            n_ok += 1
        else:
            n_bad += 1

    # a yaw rate that follows its demand exactly
    data, params, phases = synthetic(follow=1.0, split_on='YOut')
    out = analyse(data, params, [2, 3], [1, 4], phases)
    tr = out['yaw_tracking']
    check("exact follow: RMS error 0 over the whole flight", tr['airborne']['rms_err_dps'] < 1e-9)
    check("exact follow: RMS error 0 over the yaw steps", tr['yaw_steps']['rms_err_dps'] < 1e-9)
    for w, demand in (('yaw_pos', 100.0), ('yaw_neg', -100.0)):
        st = tr['steps'][w]
        check(f"exact follow: {w} demand {demand:+.0f}, achieved equal, ratio 1.00, followed",
              abs(st['demand_dps'] - demand) < 1e-9 and abs(st['ratio'] - 1.0) < 1e-9
              and st['followed'])
    check("exact follow: max |Y| over the steps is 100 deg/s",
          abs(tr['yaw_steps']['max_abs_rate_dps'] - 100.0) < 1e-9)
    hh = out['heading_hold']
    check("exact follow: hands-off hold, no heading change and no yaw rate",
          abs(hh['att_yaw_change_deg']) < 1e-9 and abs(hh['rate_y_integral_deg']) < 1e-9
          and abs(hh['end_rate_y_dps']) < 1e-9)
    check("windows come from the phases file",
          out['windows_s']['hold'] == [0.0, 10.0] and out['windows_s']['yaw_pos'] == [10.0, 13.0])
    check("motors found by SERVOn_FUNCTION", out['motor_channel'] == {1: 1, 2: 2, 3: 3, 4: 4})

    # a split that loads only on the yaw output
    sp = out['split']
    check("yaw-driven split: loads on YOut", sp['loads_on'] == 'YOut')
    check("yaw-driven split: R2 about 1", sp['r2'] > 0.999)
    check("yaw-driven split: YOut coefficient 400 us per unit (2 x 200)",
          abs(sp['coef_us_per_unit']['YOut'] - 400.0) < 1.0)
    check("yaw-driven split: ROut and POut coefficients about 0",
          abs(sp['coef_us_per_unit']['ROut']) < 5.0 and abs(sp['coef_us_per_unit']['POut']) < 5.0)
    check("yaw-driven split: YOut contribution dominates",
          sp['contrib_us']['YOut'] > 20 * max(sp['contrib_us']['ROut'], sp['contrib_us']['POut']))
    check("yaw-driven split: correlation with YOut about +1", sp['corr']['YOut'] > 0.999)

    # a yaw rate that follows only 30 percent of the demand
    data, params, phases = synthetic(follow=0.3, split_on='YOut')
    out = analyse(data, params, [2, 3], [1, 4], phases)
    st = out['yaw_tracking']['steps']['yaw_pos']
    check("30 percent follow: ratio 0.30, NOT followed",
          abs(st['ratio'] - 0.3) < 1e-9 and not st['followed'])
    check("30 percent follow: RMS error 70 deg/s over the steps on the settled half",
          65 < out['yaw_tracking']['yaw_steps']['rms_err_dps'] < 75)
    check("sign: a negative step is scored against its own demand",
          abs(out['yaw_tracking']['steps']['yaw_neg']['ratio'] - 0.3) < 1e-9)

    # splits that load on roll or pitch are not called yaw-actuated
    for src in ('ROut', 'POut'):
        data, params, phases = synthetic(split_on=src)
        out = analyse(data, params, [2, 3], [1, 4], phases)
        check(f"{src}-driven split: loads on {src}, not YOut",
              out['split']['loads_on'] == src and out['split']['r2'] > 0.999)
    # swapping the CW and CCW lists flips the sign of the coefficient
    data, params, phases = synthetic(split_on='YOut')
    out = analyse(data, params, [1, 4], [2, 3], phases)
    check("CW and CCW swapped: YOut coefficient -400",
          abs(out['split']['coef_us_per_unit']['YOut'] + 400.0) < 1.0)

    # the verdict function itself
    check("verdict: largest contribution wins",
          split_loads_on({'ROut': 1.0, 'POut': 2.0, 'YOut': 30.0}, 0.9) == 'YOut')
    check("verdict: low R2 is not blamed on anything",
          split_loads_on({'ROut': 1.0, 'POut': 2.0, 'YOut': 30.0}, 0.2) is None)
    check("verdict: nan R2 is not blamed on anything",
          split_loads_on({'ROut': 1.0, 'POut': 2.0, 'YOut': 30.0}, float('nan')) is None)

    # without --phases: the hands-off hold is found from the sticks
    data, params, phases = synthetic(split_on='YOut')
    n = len(data['RCOU'])
    data['RCIN'] = [{'TimeUS': r['TimeUS'], 'C1': 1500, 'C2': 1500, 'C4': 1500 if r['TimeUS'] < 9e6 or r['TimeUS'] > 12e6 else 1800}
                    for r in data['RCOU']]
    out = analyse(data, params, [2, 3], [1, 4], None)
    hold = out['windows_s']['hold']
    check(f"no phases: longest centred-stick span is the 12-33 s one ({hold})",
          abs(hold[0] - 12.0) < 0.1 and abs(hold[1] - 32.98) < 0.1)
    check(f"no phases: {n} RCOU rows, airborne window is the whole log",
          out['windows_s']['airborne'][0] == 0.0 and out['windows_s']['airborne'][1] > 32.9)
    print(f"{n_ok} passed, {n_bad} failed")
    return 0 if n_bad == 0 else 1


def main():
    global np
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('log', nargs='?', help='dataflash .bin log of a hover')
    ap.add_argument('--cw', help='motor numbers spinning CW, e.g. 2,3 (required with a log)')
    ap.add_argument('--ccw', help='motor numbers spinning CCW, e.g. 1,4 (required with a log)')
    ap.add_argument('--phases', help='phases JSON from sitl-qhover-steps.py (--phases-out)')
    ap.add_argument('--json', help='write the metrics here as JSON')
    ap.add_argument('--window', nargs=2, type=float, metavar=('START', 'END'),
                    help='without --phases: limit the flight window to these log '
                         'seconds (TimeUS / 1e6), e.g. the part before a spin-up')
    ap.add_argument('--label', default='', help='name printed in place of the log file name')
    ap.add_argument('--selftest', action='store_true',
                    help='run the offline checks on synthetic rows and exit; needs numpy and '
                         'no log file (run it with a Python that has numpy)')
    a = ap.parse_args()
    try:
        import numpy
    except ImportError:
        sys.exit("numpy not found. Try: pip install numpy pymavlink")
    np = numpy
    if a.selftest:
        return selftest()
    if not (a.log and a.cw and a.ccw):
        ap.error("needs LOG, --cw and --ccw (or --selftest)")
    CW = [int(x) for x in a.cw.split(',')]
    CCW = [int(x) for x in a.ccw.split(',')]
    phases = None
    if a.phases:
        with open(a.phases) as fh:
            phases = json.load(fh)['phases']
        if not phases:
            sys.exit(f"{a.phases}: phases file holds no phases")
    data, params = load_log(a.log)
    out = analyse(data, params, CW, CCW, phases, a.window, a.label)
    print_report(out, a.label or a.log)
    if a.json:
        with open(a.json, 'w') as fh:
            json.dump(out, fh, indent=1, default=float)
    return 0


if __name__ == '__main__':
    sys.exit(main())
