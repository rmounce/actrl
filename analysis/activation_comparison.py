"""Offline three-policy event replay; production code and deployment untouched.

Reads the export_history CSVs, causally forward-fills onto a 10 s grid,
warms the full closed-loop model before activation, then optionally anchors
measured temperatures and room PID outputs to the last pre-event observations.
The anchor preserves the model's measured/bulk temperature difference; it does
not establish the true bulk state. Use the unanchored arm to expose warmup drift.

Example:
  uv run python analysis/activation_comparison.py --data-dir /tmp/actrl-review-history

Output CSVs are gitignored. Report sensor (Tm), bulk (T), and effective (E)
temperatures separately. Solar fraction and initial thermal lead sensitivity
are assumptions, not fitted parameters. No recorded HVAC/temperature forcing
is applied after activation; the policies can change capacity and airflow.
"""
from __future__ import annotations

import argparse
import copy
import sys
from contextlib import contextmanager
from pathlib import Path
from unittest.mock import patch

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'tests'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
from sim.closed_loop import ClosedLoop, ROOMS
from sim.solar import AZ_NE, AZ_NW, vertical_irradiance
from scenarios import base_world, room_climates
import actrl
from control import MyPID, MyDeriv

EVENT = pd.Timestamp('2026-09-09 08:33:10', tz='Australia/Adelaide')
END = pd.Timestamp('2026-09-09 10:16:00', tz='Australia/Adelaide')


def seed_proportional(app, errors):
    """Analysis-only: establish P+D-relative priority using leader's integral.

    Project the ordinary update on copies (including re-enable clearing),
    select one pre-seeding raw leader, and transfer its I to eligible rooms
    only when that increases their output. Ordinary production update then
    runs once. Existing D histories and other rooms' relative PIDs survive.
    """
    projected = {}
    for room, error in errors[app.mode].items():
        pid = copy.deepcopy(app.pids[room])
        if not app.rooms_enabled[room]:
            pid.clear()
        pid.update(error, actrl.mode_sign[app.mode] * app.targets[app.mode][room])
        projected[room] = pid
    leader = max(projected, key=lambda r: projected[r].get_output())
    for room in sorted(app.activation_steps[app.mode] & projected.keys()):
        if errors[app.mode][room] < actrl.room_activation_error - 1e-9:
            continue
        boost = max(0.0, projected[leader].i_term - projected[room].i_term)
        # A newly re-enabled room has no comparable requested-target history
        # in the event workflow; keep this helper safe for that case too.
        if boost and app.rooms_enabled[room]:
            app.pids[room].adjust_integral(boost)
    app.activation_steps[app.mode] = set()


def seed_match(app, errors):
    """Analysis-only historical candidate: tie activated raw output to leader."""
    projected = {}
    for room, error in errors[app.mode].items():
        pid = copy.deepcopy(app.pids[room])
        if not app.rooms_enabled[room]:
            pid.clear()
        pid.update(error, actrl.mode_sign[app.mode] * app.targets[app.mode][room])
        projected[room] = pid
    leader_output = max(pid.get_output() for pid in projected.values())
    for room in sorted(app.activation_steps[app.mode] & projected.keys()):
        if errors[app.mode][room] < actrl.room_activation_error - 1e-9:
            continue
        boost = max(0.0, leader_output - projected[room].get_output())
        if boost and app.rooms_enabled[room]:
            app.pids[room].adjust_integral(boost)
    app.activation_steps[app.mode] = set()


@contextmanager
def policy(name):
    original = actrl.Actrl._calculate_pid_outputs

    def calculate(app, errors):
        if name == 'original' or not getattr(app, 'comparison_enabled', True):
            app.activation_steps[app.mode] = set()
        elif name == 'match':
            seed_match(app, errors)
        elif name == 'proportional':
            seed_proportional(app, errors)
        return original(app, errors)

    if name not in ('original', 'match', 'proportional'):
        raise ValueError(name)
    with patch.object(actrl.Actrl, '_calculate_pid_outputs', calculate):
        yield


def load(data_dir, start, end):
    index = pd.date_range(start, end, freq='10s').tz_convert('UTC')
    columns = {}

    def field(measurement, entity, field_name='value'):
        files = sorted((data_dir / 'raw').glob(f'*/{measurement}__{entity}.csv.gz'))
        if not files:
            raise ValueError(f'missing {measurement}/{entity}')
        frames = [pd.read_csv(p) for p in files]
        points = pd.concat(frames, ignore_index=True)
        points = points[points['field'] == field_name]
        times = pd.to_datetime(points['time'], format='ISO8601', utc=True)
        s = pd.Series(points['value'].astype(float).values, index=times)
        s = s[~s.index.duplicated(keep='last')].sort_index()
        if s.empty or s.index.max() < index[-1] - pd.Timedelta('15min'):
            # Targets may legitimately remain unchanged for hours.
            if field_name not in ('target_temp_low', 'target_temp_high', 'temperature'):
                raise ValueError(f'stale {entity}/{field_name}: {s.index.max()}')
        return s.reindex(index, method='ffill')

    for r in ROOMS:
        columns[f'Tm_{r}'] = field('sensor__temperature', f'{r}_average_temperature')
        columns[f'RH_{r}'] = field('sensor__humidity', f'{r}_average_humidity')
        columns[f'low_{r}'] = field('climate', f'{r}_aircon', 'target_temp_low')
        columns[f'high_{r}'] = field('climate', f'{r}_aircon', 'target_temp_high')
        columns[f'climate_{r}'] = field('climate', f'{r}_aircon', 'current_temperature')
        columns[f'pid_{r}'] = field('input_number', f'{r}_pid')
    columns['outdoor'] = field('sensor__temperature', 'm5atom_outside_temp')
    columns['setpoint'] = field('climate', 'm5atom_climate', 'temperature')
    columns['p_kw'] = field('sensor__power', 'shellyem_ec64c9c6932b_channel_1_power') / 1000
    columns['increment'] = field('input_number', 'aircon_comp_speed')
    frame = pd.DataFrame(columns, index=index)
    required = [c for c in frame if not c.startswith('pid_')]
    if frame[required].isna().any().any():
        raise ValueError(f'unseeded inputs: {frame[required].columns[frame[required].isna().any()].tolist()}')
    for r in ROOMS:
        t, rh = frame[f'Tm_{r}'], frame[f'RH_{r}']
        frame[f'offset_{r}'] = 0.33 * rh / 100 * 6.105 * np.exp(17.27*t/(237.7+t)) - 4
        frame[f'E_{r}'] = t + frame[f'offset_{r}']
    return frame


def run(frame, name, anchor=True, solar=0.5, lead_scale=1.0, cancel=False):
    first = frame.iloc[0]
    temps = {r: float(first[f'Tm_{r}']) for r in ROOMS}
    targets = {r: (float(first[f'low_{r}']), float(first[f'high_{r}'])) for r in ROOMS}
    world = base_world(setpoint=float(first['setpoint']), static_pressure='2')
    room_climates(world, 'heat_cool', targets, temps)
    loop = ClosedLoop(world, temps, setpoint=float(first['setpoint']))
    rows = []
    topup = {}
    original_inflate = actrl.min_airflow_inflation

    def inflate(pids, outputs, weights, minimum):
        before = sum(actrl.damper_share(o)*weights[r] for r,o in outputs.items())
        original_inflate(pids, outputs, weights, minimum)
        after = sum(actrl.damper_share(o)*weights[r] for r,o in outputs.items())
        topup.update(topup=max(0,after-before), min_flow=minimum/2)

    try:
        with policy(name), patch.object(actrl, 'min_airflow_inflation', inflate):
            for i, (ts, rec) in enumerate(frame.iterrows()):
                loop.app.comparison_enabled = ts >= EVENT
                if ts == EVENT and anchor:
                    prev = frame.iloc[i-1]
                    loop.app.mode = 'heat'
                    for r in ROOMS:
                        lead = loop.house.temps_measured[r] - loop.house.temps[r]
                        loop.house.temps_measured[r] = float(prev[f'Tm_{r}'])
                        loop.house.temps[r] = float(prev[f'Tm_{r}']) - lead_scale*lead
                        # Historical published PID includes airflow top-up.
                        # Valid only if that top-up was inactive at this boundary;
                        # assess this assumption in the report, not as true raw I.
                        # Reconstruct P/D from observed history, not the
                        # drifting simulated warmup. I is then the residual.
                        pid = MyPID(actrl.room_kp, actrl.room_ki*10,
                                    actrl.room_deriv_factor/actrl.interval,
                                    int(actrl.room_deriv_window/actrl.interval))
                        deriv = MyDeriv(int(actrl.global_temp_deriv_window/actrl.interval),
                                        actrl.global_temp_deriv_factor/actrl.interval)
                        for _, past in frame.iloc[:i].iterrows():
                            pid.update(float(past[f'low_{r}']-past[f'E_{r}']), -float(past[f'low_{r}']))
                            deriv.set(float(past[f'E_{r}']),0)
                        if np.isfinite(prev[f'pid_{r}']):
                            pid.set_integral(float(prev[f'pid_{r}']) - pid.p_term - pid.deriv.get())
                        loop.app.pids[r] = pid
                        loop.app.temp_derivs[r] = deriv
                        loop.app.rooms_enabled[r] = True
                        for mode, field_name in [('heat','low'),('cool','high')]:
                            loop.app.targets[mode][r] = float(prev[f'{field_name}_{r}'])
                            loop.app.previous_requested_targets[mode][r] = float(prev[f'{field_name}_{r}'])
                        position = 5*round(100*min(1,actrl.damper_share(float(prev[f'pid_{r}'])))/5)
                        loop.app.damper_pos[r] = position
                        loop.world.update(f'cover.{r}', {'attributes':{'current_position':position}})
                    speed = int(prev['increment'])
                    loop.app.capacity.guesstimated_comp_speed = speed
                    loop.unit.comp_speed = speed
                    loop._p_lag.reset(float(prev['p_kw']))
                updates = {
                    'climate.m5atom_climate': {'attributes': {'temperature': float(rec['setpoint'])}},
                    **{f'climate.{r}_aircon': {'attributes': {
                        'target_temp_low': 16.0 if cancel and r=='study' and ts>=EVENT+pd.Timedelta('10min') else float(rec[f'low_{r}']),
                        'target_temp_high': float(rec[f'high_{r}'])}} for r in ROOMS},
                }
                if anchor and ts < EVENT:
                    updates.update({
                        f'sensor.{r}_average_temperature': {'state':str(float(rec[f'E_{r}']))}
                        for r in ROOMS
                    })
                topup.clear()
                row = loop.step(
                    t_out=float(rec['outdoor']), updates=updates,
                    sun_ne=solar*vertical_irradiance(ts, AZ_NE),
                    sun_nw=solar*vertical_irradiance(ts, AZ_NW),
                    ctrl_offsets={r:float(rec[f'offset_{r}']) for r in ROOMS},
                )
                row.update(topup=topup.get('topup',0), min_flow=topup.get('min_flow',0))
                row['capacity_step'] = loop.app.capacity.guesstimated_comp_speed
                row['fan'] = loop.world.entities['climate.m5atom_climate']['attributes']['fan_mode']
                for r in ROOMS:
                    row[f'E_{r}'] = row[f'Tm_{r}'] + float(rec[f'offset_{r}'])
                rows.append(row)
    finally:
        loop.close()
    return pd.DataFrame(rows,index=frame.index)


def metrics(frame, sim):
    post = sim.loc[EVENT:]
    target = frame.loc[EVENT:,'low_study']
    error = post['E_study']-target
    hits = post.index[error>=0]
    peak = post['E_study'].idxmax()
    return {
        'target_min': (hits[0]-EVENT).total_seconds()/60 if len(hits) else None,
        'study_peak_effective': post['E_study'].max(),
        'study_peak_min': (peak-EVENT).total_seconds()/60,
        'study_deficit_Kmin': (-error).clip(lower=0).sum()/6,
        'kitchen_excess_Kmin': (post['E_kitchen']-frame.loc[EVENT:,'low_kitchen']).clip(lower=0).sum()/6,
        'peak_kW': post['p_kw'].max(),
        'kWh': post['p_kw'].sum()/360,
        'peak_increment': post['increment'].max(),
        'study_RMSE': np.sqrt(((post['Tm_study']-frame.loc[EVENT:,'Tm_study'])**2).mean()),
        'kitchen_RMSE': np.sqrt(((post['Tm_kitchen']-frame.loc[EVENT:,'Tm_kitchen'])**2).mean()),
        'topup_min': (post['topup']>1e-6).sum()/6 if 'topup' in post else None,
        'topup_duct_min': post['topup'].sum()/6 if 'topup' in post else None,
        'first_study_damper': post['damper_study'].iloc[0] if 'damper_study' in post else None,
        'first_kitchen_damper': post['damper_kitchen'].iloc[0] if 'damper_kitchen' in post else None,
    }


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-dir',type=Path,required=True)
    parser.add_argument('--out',type=Path,default=Path('analysis/out/activation_20260909'))
    args=parser.parse_args()
    frame=load(args.data_dir, EVENT-pd.Timedelta('3h'),END)
    args.out.mkdir(parents=True,exist_ok=True)
    frame.to_csv(args.out/'observed.csv')
    summaries=[]
    for anchor,solar,lead in [(False,.5,1),(True,.5,1),(True,0,1),(True,1,1),(True,.5,0),(True,.5,2)]:
        for name in ['original','match','proportional']:
            sim=run(frame,name,anchor,solar,lead)
            label=f'{name}_anchor{int(anchor)}_solar{solar}_lead{lead}'
            sim.to_csv(args.out/f'{label}.csv')
            result={'arm':name,'anchor':anchor,'solar':solar,'lead_scale':lead,**metrics(frame,sim)}
            summaries.append(result)
            print(label, {k:round(v,3) if isinstance(v,(float,np.floating)) else v for k,v in result.items() if k in ('target_min','kWh','peak_kW','study_RMSE','first_study_damper','first_kitchen_damper','topup_min')},flush=True)
    pd.DataFrame(summaries).to_csv(args.out/'summary.csv',index=False)
    print('Actual',metrics(frame,frame))


if __name__=='__main__':
    main()
