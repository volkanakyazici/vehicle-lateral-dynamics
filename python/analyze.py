"""Analyze exported CSVs. This script does not simulate the vehicle.

Usage from the repository root:
    python3 python/analyze.py --input results/simulink
    python3 python/analyze.py --input results/preview
"""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

PREVIEW = 'PYTHON_NUMERICAL_PREVIEW_NOT_SIMULINK'
EXPECTED = [f'{kind}_{u}' for kind in ('step', 'uncontrolled', 'track') for u in (10, 20, 30)]
EXPECTED += ['sine_20', 'offset_20', 'limited_20']
COLORS = {10: '#2878b5', 20: '#db7c26', 30: '#008b79'}


def read_case(folder, name):
    meta = json.loads((folder / f'{name}.json').read_text())
    expected_name = f"{meta['case']}_{meta['speed_mps']:g}"
    if name != expected_name:
        raise ValueError(f'{name}: filename does not match case metadata')
    data = np.genfromtxt(folder / f'{name}.csv', delimiter=',', names=True)
    required = {'time_s', 'X_m', 'Y_m', 'Y_ref_m', 'ey_m', 'vy_mps',
                'yaw_rate_radps', 'psi_rad', 'road_heading_rad', 'body_heading_ref_rad',
                'beta_rad', 'delta_request_rad', 'delta_rad', 'alpha_f_rad',
                'alpha_r_rad', 'ay_mps2', 'curvature_1pm'}
    if not required.issubset(data.dtype.names or ()):
        raise ValueError(f'{name}: missing CSV columns')
    if data.ndim != 1 or len(data) < 2:
        raise ValueError(f'{name}: expected a sampled time series')
    if not all(np.isfinite(data[c]).all() for c in data.dtype.names):
        raise ValueError(f'{name}: nonfinite data')
    dt = meta['sample_time_s']
    if dt <= 0 or not np.allclose(np.diff(data['time_s']), dt, rtol=1e-7, atol=1e-10):
        raise ValueError(f'{name}: inconsistent time grid')
    duration = (10. if meta['case'] in ('step', 'sine', 'zero')
                else meta['config']['path_end_m'] / meta['speed_mps'])
    expected_samples = int(np.floor(duration / dt + .5)) + 1
    if len(data) != expected_samples or not np.isclose(data['time_s'][0], 0., atol=1e-10):
        raise ValueError(f'{name}: incomplete simulation history')
    if not np.allclose(data['X_m'], meta['speed_mps'] * data['time_s'], atol=1e-7):
        raise ValueError(f'{name}: X does not match U*t')
    if not np.allclose(data['ey_m'], data['Y_m'] - data['Y_ref_m'], atol=1e-8):
        raise ValueError(f'{name}: inconsistent lateral error')
    if meta['source'] not in ('SIMULINK', PREVIEW):
        raise ValueError(f'{name}: unknown simulation source')
    if meta['source'] == 'SIMULINK' and meta.get('status') != 'PASS':
        raise ValueError(f'{name}: Simulink run did not pass its checks')
    rate = np.diff(np.r_[0., data['delta_rad']]) / dt
    if np.max(abs(data['delta_rad'])) > meta['steering_limit_rad'] + 1e-8:
        raise ValueError(f'{name}: angle bound violated')
    if np.max(abs(rate)) > meta['steering_rate_limit_radps'] + 1e-7:
        raise ValueError(f'{name}: rate bound violated')
    return data, meta


def metrics(name, d, meta):
    tracking_task = meta['case'] in ('track', 'offset', 'limited', 'uncontrolled')
    dt = meta['sample_time_s']
    p = meta['config']
    limit = meta['steering_limit_rad']
    bounded = np.clip(d['delta_request_rad'], -limit, limit)
    previous = np.r_[0., d['delta_rad'][:-1]]
    rate = (d['delta_rad'] - previous) / dt
    peak_slip = np.rad2deg(max(np.max(abs(d['alpha_f_rad'])), np.max(abs(d['alpha_r_rad']))))
    peak_heading = np.rad2deg(np.max(abs(d['psi_rad'])))
    peak_ay = np.max(abs(d['ay_mps2'])) / p['gravity_mps2']
    return {
        'case': name, 'source': meta['source'], 'speed_mps': meta['speed_mps'],
        'sample_time_s': dt, 'duration_s': d['time_s'][-1] - d['time_s'][0],
        # Use left endpoints: each sample represents one equally long interval.
        'path_rmse_m': np.sqrt(np.mean(d['ey_m'][:-1]**2)) if tracking_task else '',
        'peak_path_error_m': np.max(abs(d['ey_m'])) if tracking_task else '',
        'final_path_error_m': d['ey_m'][-1] if tracking_task else '',
        'peak_steering_deg': np.rad2deg(np.max(abs(d['delta_rad']))),
        'peak_steering_rate_degps': np.rad2deg(np.max(abs(rate))),
        'peak_axle_slip_deg': peak_slip, 'peak_body_heading_deg': peak_heading,
        'peak_lateral_acceleration_g': peak_ay,
        'angle_limited_duration_s': np.count_nonzero(abs(d['delta_request_rad'][:-1]) > limit + 1e-9) * dt,
        'rate_limited_duration_s': np.count_nonzero(abs(bounded[:-1]-previous[:-1]) > meta['steering_rate_limit_radps']*dt + 1e-9) * dt,
        'small_angle_screen_passed': bool(peak_slip <= p['screen_slip_deg'] and
                                          peak_heading <= p['screen_heading_deg'] and
                                          peak_ay <= p['screen_lateral_accel_g']),
    }


def make_figures(cases, folder, source):
    folder.mkdir(exist_ok=True)
    provenance = ('PYTHON NUMERICAL PREVIEW — NOT SIMULINK' if source == PREVIEW
                  else 'SIMULINK CSV RESULTS')
    plt.rcParams.update({'font.size': 10, 'axes.spines.top': False,
                         'axes.spines.right': False, 'lines.linewidth': 1.8})

    def canvas(title):
        fig, axes = plt.subplots(2, 2, figsize=(12, 8), layout='constrained')
        fig.suptitle(f'{title}\n{provenance}', fontsize=13, fontweight='bold')
        for ax in axes.flat:
            ax.grid(alpha=.22)
        return fig, axes.ravel()

    def finish(fig, axes, name):
        for ax in axes:
            handles, _ = ax.get_legend_handles_labels()
            if handles:
                ax.legend(fontsize=9, loc='best')
        fig.savefig(folder / name, dpi=170)
        plt.close(fig)

    fig, axes = canvas('Steering input response at 20 m/s')
    for name, color in [('step_20', '#2878b5'), ('sine_20', '#db7c26')]:
        d, _ = cases[name]
        for ax, col, ylabel, scale in zip(axes,
                ['delta_rad', 'yaw_rate_radps', 'beta_rad', 'ay_mps2'],
                ['Applied steering [deg]', 'Yaw rate [deg/s]', 'Body sideslip [deg]', 'Lateral acceleration [m/s²]'],
                [180/np.pi, 180/np.pi, 180/np.pi, 1]):
            ax.plot(d['time_s'], d[col]*scale, label=name.split('_')[0], color=color)
            ax.set(xlabel='Time [s]', ylabel=ylabel)
    finish(fig, axes, '01_steering_response.png')

    amplitude = cases['step_20'][1]['config']['steering_amplitude_deg']
    fig, axes = canvas(f'Same {amplitude:g}° steering pulse at three speeds')
    for u in (10, 20, 30):
        d, meta = cases[f'step_{u}']
        for ax, col, ylabel in zip(axes[:3], ['yaw_rate_radps', 'beta_rad', 'ay_mps2'],
                                  ['Yaw rate [deg/s]', 'Body sideslip [deg]', 'Lateral acceleration [m/s²]']):
            scale = 1 if col == 'ay_mps2' else 180/np.pi
            ax.plot(d['time_s'], d[col]*scale, label=f'{u} m/s', color=COLORS[u])
            ax.set(xlabel='Time [s]', ylabel=ylabel)
        i = np.argmin(abs(d['time_s']-4.99))
        axes[3].scatter(u, d['yaw_rate_radps'][i]/d['delta_rad'][i], color=COLORS[u], zorder=3, label=f'CSV at {u} m/s')
    p = meta['config']
    a, b = p['front_distance_m'], p['rear_distance_m']
    ku = p['mass_kg']/(a+b)*(b/p['front_axle_stiffness_Nprad'] - a/p['rear_axle_stiffness_Nprad'])
    speeds = np.linspace(5, 35, 200)
    axes[3].plot(speeds, speeds/(a+b+ku*speeds**2), '--', color='#555555', label='Analytic steady-state gain')
    axes[3].set(xlabel='Speed [m/s]', ylabel='Steady yaw gain r/δ [1/s]')
    finish(fig, axes, '02_speed_comparison.png')

    shift = cases['track_20'][1]['config']['path_shift_m']
    fig, axes = canvas(f'Tracking the same {shift:g} m spatial lane shift')
    ref, _ = cases['track_20']
    axes[0].plot(ref['X_m'], ref['Y_ref_m'], 'k--', linewidth=2.5, label='Reference')
    base, _ = cases['uncontrolled_20']
    axes[0].plot(base['X_m'], base['Y_m'], ':', color='#777777', label='Zero steering')
    for u in (10, 20, 30):
        d, meta = cases[f'track_{u}']
        for ax, col, ylabel, scale in zip(axes,
                ['Y_m', 'ey_m', 'delta_rad', 'ay_mps2'],
                ['Lateral position Y [m]', 'Lateral error [cm]', 'Applied steering [deg]', 'Lateral acceleration [g]'],
                [1, 100, 180/np.pi, 1/meta['config']['gravity_mps2']]):
            ax.plot(d['X_m'], d[col]*scale, label=f'{u} m/s', color=COLORS[u])
            ax.set(xlabel='Longitudinal position X [m]', ylabel=ylabel)
    finish(fig, axes, '03_path_tracking.png')

    fig, axes = canvas('Initial offset and steering constraints at 20 m/s')
    axes[0].plot(ref['X_m'], ref['Y_ref_m'], 'k--', label='Reference')
    for name, label, color in [('track_20','Nominal','#2878b5'),
                                ('offset_20','Initial Y = 0.5 m','#008b79'),
                                ('limited_20','Steering limited to ±0.15°','#b53a53')]:
        d, meta = cases[name]
        axes[0].plot(d['X_m'], d['Y_m'], color=color, label=label)
        axes[1].plot(d['X_m'], d['ey_m'], color=color, label=label)
    axes[0].set(xlabel='X [m]', ylabel='Lateral position Y [m]')
    axes[1].set(xlabel='X [m]', ylabel='Lateral error [m]')
    d, meta = cases['limited_20']
    axes[2].plot(d['X_m'], np.rad2deg(d['delta_request_rad']), '--', color='#b53a53', label='Requested')
    axes[2].plot(d['X_m'], np.rad2deg(d['delta_rad']), color='#2878b5', label='Applied')
    for sign in (-1, 1):
        axes[2].axhline(sign*np.rad2deg(meta['steering_limit_rad']), color='#777777', linestyle=':',
                        label='Angle bounds' if sign == 1 else None)
    axes[2].set(xlabel='X [m]', ylabel='Limited-case steering [deg]')
    zoom = axes[2].inset_axes([.07, .12, .36, .28])
    zoom.plot(d['X_m'], np.rad2deg(d['delta_rad']), color='#2878b5', linewidth=1.2)
    for sign in (-1, 1):
        zoom.axhline(sign*np.rad2deg(meta['steering_limit_rad']), color='#777777', linestyle=':', linewidth=1)
    zoom.set(title='Applied [deg]', xticks=[0, 120, 240], yticks=[-.15, 0, .15], ylim=(-.18, .18))
    zoom.tick_params(labelsize=7)
    zoom.title.set_fontsize(9)
    d, meta = cases['offset_20']
    rate = np.rad2deg(np.diff(np.r_[0., d['delta_rad']])/meta['sample_time_s'])
    axes[3].plot(d['time_s'], rate, color='#008b79', label='Offset recovery')
    for sign in (-1, 1):
        axes[3].axhline(sign*np.rad2deg(meta['steering_rate_limit_radps']), color='#777777', linestyle=':',
                        label='Rate bounds' if sign == 1 else None)
    axes[3].set(xlabel='Time [s]', ylabel='Applied steering rate [deg/s]', xlim=(0, 2))
    finish(fig, axes, '04_constraints_and_offset.png')

    fig, axes = canvas('Body heading, sideslip and tire slip during tracking')
    d, _ = cases['track_30']
    for col, label, style in [('road_heading_rad','Road tangent','--'),
                               ('body_heading_ref_rad','Body-heading reference',':'),
                               ('psi_rad','Actual body heading','-')]:
        axes[0].plot(d['X_m'], np.rad2deg(d[col]), style, label=label)
    axes[0].set(xlabel='X [m]', ylabel='Heading at 30 m/s [deg]')
    for u in (10, 20, 30):
        d, _ = cases[f'track_{u}']
        for ax, col, label in zip(axes[1:], ['beta_rad','alpha_f_rad','alpha_r_rad'],
                                  ['Body sideslip [deg]','Front axle slip [deg]','Rear axle slip [deg]']):
            ax.plot(d['X_m'], np.rad2deg(d[col]), color=COLORS[u], label=f'{u} m/s')
            ax.set(xlabel='X [m]', ylabel=label)
    finish(fig, axes, '05_heading_and_slip.png')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True, help='Folder containing all 12 paired CSV/JSON results')
    args = parser.parse_args()
    folder = args.input.resolve()
    cases = {name: read_case(folder, name) for name in EXPECTED}
    sources = {meta['source'] for _, meta in cases.values()}
    if len(sources) != 1:
        raise ValueError('Refusing to mix preview and Simulink results')
    source = sources.pop()
    configs = {json.dumps(meta['config'], sort_keys=True) for _, meta in cases.values()}
    if len(configs) != 1:
        raise ValueError('Cases use different configurations; rerun the full experiment suite')
    status_name = 'execution_status.json' if source == 'SIMULINK' else 'preflight_report.json'
    status = json.loads((folder/status_name).read_text())
    if status.get('status') != 'PASS':
        raise ValueError('The selected simulation suite must complete successfully before analysis')
    if source == 'SIMULINK':
        records = status.get('cases', [])
        recorded = {f"{m['case']}_{m['speed_mps']:g}": m for m in records}
        if len(records) != len(EXPECTED) or set(recorded) != set(EXPECTED):
            raise ValueError('Full-suite report does not contain the expected 12 cases')
        if any(recorded[name] != meta for name, (_, meta) in cases.items()):
            raise ValueError('Case metadata differs from the full-suite report; rerun run_project')
    rows = [metrics(name, *cases[name]) for name in EXPECTED]
    with (folder/'metrics.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    make_figures(cases, folder/'figures', source)
    print('Source:', source)
    print(f'Analyzed {len(cases)} cases; wrote metrics.csv and 5 figures in {folder}')
    for row in rows:
        if row['case'].startswith(('track_', 'offset_', 'limited_')):
            print(f"{row['case']}: RMSE={row['path_rmse_m']:.6f} m; screen={row['small_angle_screen_passed']}")


if __name__ == '__main__':
    main()
