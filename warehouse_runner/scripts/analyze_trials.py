#!/usr/bin/env python3
"""Stage 7: turn trials.csv into statistics, a Markdown summary and charts.

  ros2 run warehouse_runner analyze_trials ~/ros2_ws/trials/<run>
  ros2 run warehouse_runner analyze_trials            # newest run
Writes summary.md, summary.json, chart_time.png, chart_outcomes.png into the run folder.
"""
import csv
import json
import statistics as st
import sys
from pathlib import Path


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def describe(values):
    v = [x for x in values if x is not None]
    if not v:
        return {'mean': None, 'sd': None, 'min': None, 'max': None}
    return {'mean': round(st.mean(v), 2), 'sd': round(st.stdev(v), 2) if len(v) > 1 else 0.0,
            'min': round(min(v), 2), 'max': round(max(v), 2)}


def main():
    if len(sys.argv) > 1:
        run = Path(sys.argv[1]).expanduser()
    else:
        runs = sorted((Path.home() / 'ros2_ws' / 'trials').glob('*/trials.csv'))
        if not runs:
            print('No trials found.'); return 1
        run = runs[-1].parent
    rows = list(csv.DictReader(open(run / 'trials.csv')))
    scen = {}
    for r in rows:
        scen.setdefault(r['scenario'], []).append(r)

    summary = {'run': str(run), 'trials': len(rows),
               'as_expected_rate': round(sum(r['as_expected'] == 'True' for r in rows) / len(rows), 3),
               'scenarios': {}}
    for name, rs in scen.items():
        summary['scenarios'][name] = {
            'n': len(rs), 'expected': rs[0]['expected'],
            'outcomes': {o: sum(r['outcome'] == o for r in rs) for o in sorted({r['outcome'] for r in rs})},
            'as_expected_rate': round(sum(r['as_expected'] == 'True' for r in rs) / len(rs), 3),
            'time_s': describe([num(r['total_seconds']) for r in rs]),
            'distance_m': describe([num(r['total_distance_m']) for r in rs]),
            'recoveries': describe([num(r['recoveries']) for r in rs]),
            'retries': describe([num(r['retries']) for r in rs]),
            'min_clearance_m': describe([num(r['min_clearance_m']) for r in rs]),
            'true_goal_error_m': describe([num(r.get('max_true_goal_error_m')) for r in rs]),
            'amcl_error_m': describe([num(r.get('max_amcl_error_m')) for r in rs]),
            'false_successes': sum(int(num(r.get('false_successes')) or 0) for r in rs),
        }
    (run / 'summary.json').write_text(json.dumps(summary, indent=2))

    def fmt(d, unit=''):
        return '-' if d['mean'] is None else f"{d['mean']}{unit} ± {d['sd']}"
    lines = [f'# Delivery trials: {run.name}', '',
             f"{len(rows)} trials. Behaved as expected: **{summary['as_expected_rate']:.0%}**", '',
             '| Scenario | n | Expected | Outcomes | As expected | Time (s) | Distance (m) | Nav2 recoveries | Retries | Min clearance (m) |',
             '|---|---|---|---|---|---|---|---|---|---|']
    for name, s in summary['scenarios'].items():
        outs = ', '.join(f'{k} {v}' for k, v in s['outcomes'].items())
        lines.append(f"| {name} | {s['n']} | {s['expected']} | {outs} | {s['as_expected_rate']:.0%} | "
                     f"{fmt(s['time_s'])} | {fmt(s['distance_m'])} | {fmt(s['recoveries'])} | "
                     f"{fmt(s['retries'])} | {s['min_clearance_m']['min'] if s['min_clearance_m']['min'] is not None else '-'} |")
    lines += ['', '**Ground truth check (simulation only, from Gazebo):**', '',
              '| Scenario | True distance to goal at "success" (m, worst leg) | AMCL error (m, worst leg) | False successes (>0.5 m off) |',
              '|---|---|---|---|']
    for name, s in summary['scenarios'].items():
        lines.append(f"| {name} | {fmt(s['true_goal_error_m'])} | {fmt(s['amcl_error_m'])} | {s['false_successes']} |")
    lines += ['', 'Time and distance cover the whole mission (out and back to HOME). Values are mean ± standard deviation.',
              'Min clearance is the closest laser reading during the mission (lowest across trials).',
              'A false success = Nav2 reported the goal reached, but Gazebo shows the robot more than 0.5 m away.']
    (run / 'summary.md').write_text('\n'.join(lines) + '\n')

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        names = list(scen)
        fig, ax = plt.subplots(figsize=(8, 4))
        data = [[num(r['total_seconds']) for r in scen[n] if num(r['total_seconds']) is not None] for n in names]
        ax.boxplot(data, labels=names, showmeans=True)
        for k, d in enumerate(data, 1):
            ax.scatter([k] * len(d), d, s=14, c='#2a6fdb', zorder=3)
        ax.set_ylabel('mission time (s)'); ax.set_title('Mission time per scenario (each dot = one trial)')
        ax.grid(axis='y', alpha=.3); fig.tight_layout(); fig.savefig(run / 'chart_time.png', dpi=110)

        fig, ax = plt.subplots(figsize=(8, 3.6))
        outcomes = sorted({r['outcome'] for r in rows})
        colors = {'success': '#2ca25f', 'partial': '#f0a202', 'failed': '#d7301f', 'cancelled': '#888'}
        bottom = [0] * len(names)
        for o in outcomes:
            vals = [sum(r['outcome'] == o for r in scen[n]) for n in names]
            ax.bar(names, vals, bottom=bottom, label=o, color=colors.get(o, '#555'))
            bottom = [b + v for b, v in zip(bottom, vals)]
        ax.set_ylabel('trials'); ax.set_title('Outcomes per scenario'); ax.legend(fontsize=8)
        fig.tight_layout(); fig.savefig(run / 'chart_outcomes.png', dpi=110)
    except Exception as e:  # noqa: BLE001
        print('charts skipped:', e)

    print((run / 'summary.md').read_text())
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
