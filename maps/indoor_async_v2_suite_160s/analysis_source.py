#!/usr/bin/env python3
"""Plot saved indoor benchmark measurements without rerunning the planner."""
import argparse
import hashlib
import html
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from classroom_voxel_scene import ROOT
from indoor_scenario_suite import LABELS, SPAWNS


def summarize(folder):
    summary = json.loads((folder/'summary.json').read_text())
    rows = [r for r in summary['cases'] if r['status'] == 'verified']
    colors = ('#62b7ff', '#f4b052', '#8bd9ad')
    font_path = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    title, label, small = [ImageFont.truetype(font_path, n) for n in (30, 22, 18)]
    canvas = Image.new('RGB', (1600, 1350), '#111923')
    draw = ImageDraw.Draw(canvas)
    draw.text((32, 20), 'Indoor A1 Algorithm / reduction of unknown voxels after the initial scan', fill='white', font=title)
    for i, spawn in enumerate(SPAWNS):
        draw.line((36+i*500, 83, 84+i*500, 83), fill=colors[i], width=4)
        draw.text((96+i*500, 68), spawn, fill='#e2ebf7', font=label)
    aggregates = []
    for n, (layout, name) in enumerate(LABELS.items()):
        left, top = 95+(n%2)*785, 180+(n//2)*370
        width, height = 650, 255
        draw.text((left-55, top-48), name, fill='white', font=label)
        for pct in range(0, 101, 25):
            y = top+height*(1-pct/100)
            draw.line((left, y, left+width, y), fill='#344253')
            draw.text((left-52, y-12), f'{pct}%', fill='#aab8c8', font=small)
        for sec in np.linspace(0, summary['duration_cap'], 5):
            x = left+width*sec/summary['duration_cap']
            draw.line((x, top, x, top+height), fill='#263344')
            draw.text((x-12, top+height+12), f'{sec:g}', fill='#aab8c8', font=small)
        selected = [r for r in rows if r['layout'] == layout]
        for r in selected:
            color = colors[list(SPAWNS).index(r['spawn'])]
            points = [(left+width*p['time']/summary['duration_cap'],
                       top+height*p['unknown']/r['initial_unknown']) for p in r['coverage_curve']]
            if len(points) >= 2:
                draw.line(points, fill=color, width=3)
        if selected:
            aggregates.append(dict(layout=layout, verified=len(selected),
                unknown_reduction_percent_range=[min(r['unknown_reduction_percent'] for r in selected),
                                                 max(r['unknown_reduction_percent'] for r in selected)],
                final_unknown_range=[min(r['final_unknown'] for r in selected), max(r['final_unknown'] for r in selected)],
                mean_planning_seconds=float(np.mean([r['mean_planning_seconds'] for r in selected])),
                worst_case_p95_seconds=max(r['p95_planning_seconds'] for r in selected),
                max_post_startup_idle_seconds=max(max(r['post_startup_idle_seconds']) for r in selected),
                minimum_sampled_separation_m=min(r['sampled_min_robot_separation_m'] for r in selected)))
    draw.text((32, 1310), 'Time (s). One trial per layout/start. Progress is not observable-space completion.',
              fill='#aab8c8', font=label)
    canvas.save(folder/'coverage_curves.png')
    flagged = [dict(id=r['id'], minimum_sampled_separation_m=r['sampled_min_robot_separation_m'],
                    post_startup_idle_seconds=r['post_startup_idle_seconds']) for r in rows
               if r['sampled_min_robot_separation_m'] < .5 or max(r['post_startup_idle_seconds']) > 5.]
    milestones = []
    for r in rows:
        crossings = [p['time'] for p in r['coverage_curve'] if p['unknown'] <= .1*r['initial_unknown']]
        at_80 = min(r['coverage_curve'], key=lambda p: abs(p['time']-80.))
        milestones.append(dict(id=r['id'], first_recorded_90_percent_reduction_seconds=min(crossings) if crossings else None,
                               checkpoint_time=at_80['time'], unknown_at_checkpoint=at_80['unknown'],
                               reduction_at_checkpoint_percent=100*(1-at_80['unknown']/r['initial_unknown'])))
    sensitivity = []
    for layout in LABELS:
        entries = [m for m in milestones if m['id'].startswith(layout+'__')
                   and m['first_recorded_90_percent_reduction_seconds'] is not None]
        if len(entries) > 1:
            entries.sort(key=lambda m: m['first_recorded_90_percent_reduction_seconds'])
            sensitivity.append(dict(layout=layout, fastest=entries[0]['id'], slowest=entries[-1]['id'],
                time_to_90_percent_range=[entries[0]['first_recorded_90_percent_reduction_seconds'],
                                          entries[-1]['first_recorded_90_percent_reduction_seconds']]))
    result = dict(planned=summary['planned'], verified=summary['verified'],
                  actual_exploration_completions=sum(r['complete'] for r in rows),
                  total_simulated_seconds=sum(r['seconds'] for r in rows),
                  by_layout=aggregates, motion_attention_cases=flagged, progress_milestones=milestones,
                  spawn_sensitivity=sensitivity,
                  analysis_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (folder/'aggregate.json').write_text(json.dumps(result, indent=2)+'\n')
    table = []
    for a in aggregates:
        lo, hi = a['unknown_reduction_percent_range']
        table.append(f'<tr><td>{html.escape(a["layout"])}</td><td>{a["verified"]}</td>'
                     f'<td>{lo:.2f}–{hi:.2f}%</td><td>{a["mean_planning_seconds"]:.3f}</td>'
                     f'<td>{a["worst_case_p95_seconds"]:.3f}</td>'
                     f'<td>{a["max_post_startup_idle_seconds"]:.2f}</td>'
                     f'<td>{a["minimum_sampled_separation_m"]:.2f}</td></tr>')
    slow_note = ''
    if sensitivity:
        worst = max(sensitivity, key=lambda s: s['time_to_90_percent_range'][1]-s['time_to_90_percent_range'][0])
        lo, hi = worst['time_to_90_percent_range']
        slow_note = (f'<p><b>本批出生点影响最明显的布局：{worst["layout"]}。</b>'
                     f'首次记录到未知减少 90% 的时间为 {lo:.1f}–{hi:.1f} 秒；最慢为 {worst["slowest"]}。'
                     '这是推进效率差异，不能据此认定存在任务遗漏。</p>')
    section = ('<section id="aggregate"><h2>场景汇总</h2>'+slow_note+'<p>同一布局的三组出生点；规划均值为三次均值的平均。'
               'P95、空闲取最差的一组，车距取最小采样值。</p><table cellpadding="10">'
               '<tr><th>布局</th><th>核验通过</th><th>未知减少范围</th><th>规划均值 s</th>'
               '<th>最差 P95 s</th><th>最长累计启动后空闲 s</th><th>最小采样车距 m</th></tr>'
               +''.join(table)+'</table><p><a href="aggregate.json">汇总数据及需关注案例</a></p>'
               '<img src="coverage_curves.png" alt="Unknown reduction over time"></section>')
    index = folder/'index.html'
    text = index.read_text()
    if '<section id="aggregate">' in text:
        start = text.index('<section id="aggregate">')
        end = text.index('</section>', start)+len('</section>')
        text = text[:start]+text[end:]
    index.write_text(text.replace('<main>', section+'<main>', 1))
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', nargs='?', type=Path, default=ROOT/'maps/indoor_async_v2_suite_160s')
    summarize(parser.parse_args().folder)
