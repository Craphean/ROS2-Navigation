#!/usr/bin/env python3
"""Generate deterministic SDF indoor scenes and validated two-robot starts."""
import copy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw, ImageFont
import xacro
import yaml

from classroom_voxel_scene import ROOT, DEFAULT_WORLD, scene

SUITE = ROOT/'config/indoor_suite'
WORLDS = ROOT/'worlds/indoor_suite'
BASE_CONFIG = ROOT/'config/voxel360_async_v2_params.yaml'
SPAWNS = {
    'front_split': ((-6.5, 4.8, 0.), (6.5, 4.8, math.pi)),
    'rear_together': ((5.5, -5.3, math.pi/2), (6.8, -5.3, math.pi/2)),
    'diagonal': ((-6.5, -3.5, 0.), (6.5, 3.5, math.pi)),
}
LABELS = {
    'classroom': 'Classroom / original desks',
    'office': 'Office / desk pods and screens',
    'meeting': 'Meeting room / central table',
    'library': 'Library / tall shelf aisles',
    'laboratory': 'Laboratory / benches and partitions',
    'multiroom': 'Four rooms / corridor and doorways',
}


def clone_model(template, name, x, y, yaw=0.):
    model = copy.deepcopy(template)
    model.set('name', name)
    pose = model.find('pose')
    if pose is None:
        pose = ET.SubElement(model, 'pose')
    pose.text = f'{x} {y} 0 0 0 {yaw}'
    return model


def box_model(name, center, size, color='0.42 0.49 0.58 1'):
    model = ET.Element('model', name=name)
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = ' '.join(map(str, (*center, 0, 0, 0)))
    link = ET.SubElement(model, 'link', name='body')
    for kind in ('collision', 'visual'):
        item = ET.SubElement(link, kind, name=kind)
        ET.SubElement(ET.SubElement(ET.SubElement(item, 'geometry'), 'box'), 'size').text = ' '.join(map(str, size))
        if kind == 'visual':
            material = ET.SubElement(item, 'material')
            for field in ('ambient', 'diffuse'):
                ET.SubElement(material, field).text = color
    return model


def worlds():
    original = ET.fromstring(xacro.process_file(str(DEFAULT_WORLD), mappings={'headless': 'true'}).toxml())
    models = {m.get('name'): m for m in original.find('world').findall('model')}
    result = {'classroom': original}
    for name in LABELS:
        if name == 'classroom':
            continue
        root = copy.deepcopy(original)
        world = root.find('world')
        world.set('name', name)
        for model in list(world.findall('model')):
            if model.get('name') not in ('classroom_ground', 'classroom_walls', 'whiteboard'):
                world.remove(model)

        def desk(label, x, y, yaw=0.):
            world.append(clone_model(models['desk_c1_r1'], label, x, y, yaw))
            # Chair behind the desk, transformed with the same yaw.
            world.append(clone_model(models['chair_c1_r1'], label+'_chair',
                                     x+.65*math.sin(yaw), y-.65*math.cos(yaw), yaw+math.pi))

        def block(label, x, y, sx, sy, height, z=None):
            world.append(box_model(label, (x, y, height/2 if z is None else z), (sx, sy, height)))

        if name == 'office':
            for col, x in enumerate((-4., -1.9, 1.9, 4.)):
                for row, y in enumerate((-2.7, 1.3)):
                    desk(f'workstation_{col}_{row}', x, y)
            for i, x in enumerate((-3., 3.)):
                block(f'pod_screen_{i}', x, -.7, 3.6, .12, 1.45)
            block('storage_west', -7.3, 0., .65, 2.4, 1.9)
            block('storage_east', 7.3, 0., .65, 2.4, 1.9)
        elif name == 'meeting':
            block('conference_top', 0., 0., 5., 2., .10, .78)
            for x in (-2., 2.):
                for y in (-.7, .7):
                    block(f'table_leg_{x}_{y}', x, y, .16, .16, .76)
            for i, x in enumerate((-2., -.7, .7, 2.)):
                for side in (-1, 1):
                    world.append(clone_model(models['chair_c1_r1'], f'chair_{i}_{side}', x, side*1.6,
                                             0. if side == 1 else math.pi))
            for side in (-1, 1):
                world.append(clone_model(models['chair_c1_r1'], f'end_chair_{side}', side*3.1, 0., side*math.pi/2))
            block('presentation_cabinet', -4.3, 4., 2.1, .6, 1.4)
            block('rear_cabinet', 0., -5.3, 3., .6, 1.9)
        elif name == 'library':
            for col, x in enumerate((-4., 0., 4.)):
                for row, y in enumerate((-2.2, 2.2)):
                    block(f'bookcase_{col}_{row}', x, y, .7, 2.8, 2.2)
            desk('reading_west', -6., 1.3, math.pi/2)
            desk('reading_east', 6., -1.3, -math.pi/2)
        elif name == 'laboratory':
            for col, x in enumerate((-4., 0., 4.)):
                for row, y in enumerate((-2.3, 1.8)):
                    block(f'bench_{col}_{row}', x, y, 2., 1., .9)
                    block(f'bench_screen_{col}_{row}', x, y+.45, 1.6, .1, .6, 1.2)
            block('divider_west', -2., -.1, .16, 2.6, 1.8)
            block('divider_east', 2., -.1, .16, 2.6, 1.8)
            block('equipment', 0., 4.5, 1.5, .7, 2.)
        elif name == 'multiroom':
            # Two 2.5 m-wide corridor walls, each with two 1.4 m doorways.
            for side in (-1, 1):
                for part, (a, b) in enumerate(((-7.9, -4.7), (-3.3, 3.3), (4.7, 7.9))):
                    block(f'corridor_{side}_{part}', (a+b)/2, side*1.25, b-a, .16, 2.8)
                block(f'room_divider_{side}', 0., side*3.575, .16, 4.65, 2.8)
                for col, x in enumerate((-3., 3.)):
                    desk(f'room_desk_{side}_{col}', x, side*3.4)
                block(f'room_cabinet_{side}', -7.35, side*2.5, .6, 1.5, 1.9)
        result[name] = root
    return result


def render_layout(truth, starts, title, path):
    """Ground truth preview for scene review only; never fed to the planner."""
    occupied = truth.state == 1
    z = truth.origin[2]+(np.arange(occupied.shape[0])+.5)*truth.resolution
    height = np.max(occupied*z[:, None, None], axis=0)
    shade = np.clip(220-height*52, 65, 220).astype(np.uint8)
    pixels = np.stack((shade, shade, shade), axis=-1)
    pixels[height == 0] = (243, 245, 247)
    canvas = Image.new('RGB', (800, 690), '#151c25')
    canvas.paste(Image.fromarray(pixels[::-1]).resize((800, 600), Image.Resampling.NEAREST), (0, 60))
    draw = ImageDraw.Draw(canvas)
    font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 20)
    draw.text((18, 18), title, font=font, fill='white')
    colors = ('#e45858', '#2279dd')
    for r, xy in enumerate(starts):
        x = (xy[0]+8)*50
        y = 60+(6-xy[1])*50
        draw.ellipse((x-12, y-12, x+12, y+12), fill=colors[r], outline='white', width=2)
        draw.text((x+15, y-12), f'R{r+1}', font=font, fill=colors[r])
    draw.text((18, 664), 'Scene truth preview / 16 x 12 m / darker = taller obstacle', font=font, fill='#bcc9d9')
    canvas.save(path)


def generate():
    SUITE.mkdir(parents=True, exist_ok=True)
    WORLDS.mkdir(parents=True, exist_ok=True)
    base = yaml.safe_load(BASE_CONFIG.read_text())
    cases = []
    for layout, root in worlds().items():
        world_path = WORLDS/f'{layout}.sdf'
        ET.indent(root)
        ET.ElementTree(root).write(world_path, encoding='utf-8', xml_declaration=True)
        for spawn_id, poses in SPAWNS.items():
            case_id = f'{layout}__{spawn_id}'
            config = copy.deepcopy(base)
            p = config['rolling_observation_planner']['ros__parameters']
            p['robot1_init_pose'], p['robot2_init_pose'] = [list(x) for x in poses]
            config_path = SUITE/f'{case_id}.yaml'
            config_path.write_text(yaml.safe_dump(config, sort_keys=False))
            truth, _, ground, starts, _, boxes, _ = scene(config_path, world_path)
            distance = ground.distances(starts[0])
            assert np.isfinite(distance[ground.cell(starts[1])]), case_id
            assert math.dist(*starts) >= 1., case_id
            if layout == 'multiroom':
                for xy in ((-4., 3.), (4., 3.), (-4., -3.), (4., -3.), (0., 0.)):
                    assert np.isfinite(distance[ground.cell(xy)]), (case_id, xy)
            preview = SUITE/f'{case_id}.png'
            render_layout(truth, starts, LABELS[layout]+' / '+spawn_id, preview)
            cases.append(dict(id=case_id, layout=layout, spawn=spawn_id, starts=poses,
                              world=str(world_path.relative_to(ROOT)), config=str(config_path.relative_to(ROOT)),
                              preview=str(preview.relative_to(ROOT)), collision_boxes=len(boxes),
                              world_sha256=hashlib.sha256(world_path.read_bytes()).hexdigest(),
                              ground_sha256=hashlib.sha256(ground.data.tobytes()).hexdigest(),
                              safe_cells=int(ground.safe.sum()), reachable_safe_cells=int(np.isfinite(distance).sum()),
                              spawn_distance_m=float(distance[ground.cell(starts[1])]),
                              voxel_count=int(truth.state.size), spawn_validated=True))
    result = dict(algorithm='async_v2 + retained detail fill + idle search',
                  base_config=str(BASE_CONFIG.relative_to(ROOT)), cases=cases)
    (SUITE/'suite.json').write_text(json.dumps(result, indent=2)+'\n')
    print(f'Generated and checked {len(cases)} cases in {SUITE}', flush=True)
    return result


if __name__ == '__main__':
    generate()
