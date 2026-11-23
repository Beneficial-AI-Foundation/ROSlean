#!/usr/bin/env python3
"""Writes the world used for the demo video from the demo's apartment.sdf.

Differences from the demo world, none of which touch the mux or its inputs:

- The furniture is built from boxes and cylinders instead of being fetched
  from Gazebo Fuel, so the world loads without a network connection.
- A fixed camera sensor films the room; its frames are what the video shows.
- The real-time factor is lowered so that software rendering keeps up. All
  nodes run on simulated time, so their behaviour does not depend on it.

Usage: make_world.py APARTMENT_SDF OUT_SDF [WIDTH HEIGHT]
"""

import re
import sys

# Camera: position, the point it looks at, horizontal field of view (rad).
CAM_POS = (1.9, -3.45, 3.5)
CAM_LOOK = (-0.7, -1.2, 0.0)
CAM_HFOV = 1.25
FPS = 30

WOOD = (0.50, 0.33, 0.20)
DARK_WOOD = (0.30, 0.20, 0.13)
FABRIC = (0.26, 0.36, 0.46)
FABRIC_LIGHT = (0.36, 0.47, 0.57)
WHITE = (0.92, 0.92, 0.90)
STEEL = (0.70, 0.72, 0.74)
BLACK = (0.12, 0.12, 0.13)
LINEN = (0.88, 0.86, 0.80)
BLANKET = (0.30, 0.42, 0.38)


def mat(c):
    r, g, b = c
    return (f'<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>'
            '<specular>0.08 0.08 0.08 1</specular></material>')


class Model:
    """A static model made of parts, each given in the model's own frame."""

    def __init__(self, name, x, y, yaw=0.0):
        self.name, self.pose, self.parts = name, (x, y, yaw), []

    def box(self, x, y, z, sx, sy, sz, color, collide=True):
        """A box of size (sx, sy, sz) whose bottom centre is at (x, y, z)."""
        geom = f'<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>'
        self._add(x, y, z + sz / 2, geom, color, collide)
        return self

    def cyl(self, x, y, z, radius, length, color, collide=True):
        geom = (f'<geometry><cylinder><radius>{radius}</radius><length>{length}</length>'
                '</cylinder></geometry>')
        self._add(x, y, z + length / 2, geom, color, collide)
        return self

    def ball(self, x, y, z, radius, color):
        """A sphere centred at (x, y, z); decoration only."""
        self._add(x, y, z, f'<geometry><sphere><radius>{radius}</radius></sphere></geometry>',
                  color, False)
        return self

    def legs(self, half_x, half_y, height, color, thick=0.05):
        for sx in (-1, 1):
            for sy in (-1, 1):
                self.box(sx * half_x, sy * half_y, 0, thick, thick, height, color)
        return self

    def _add(self, x, y, z, geom, color, collide):
        n = len(self.parts)
        col = f'<collision name="c{n}"><pose>{x} {y} {z} 0 0 0</pose>{geom}</collision>'
        vis = f'<visual name="v{n}"><pose>{x} {y} {z} 0 0 0</pose>{geom}{mat(color)}</visual>'
        self.parts.append((col if collide else '') + vis)

    def sdf(self):
        x, y, yaw = self.pose
        return (f'<model name="{self.name}"><static>true</static><pose>{x} {y} 0 0 0 {yaw}</pose>'
                f'<link name="link">{"".join(self.parts)}</link></model>\n')


def chair(name, x, y, yaw, seat=WOOD):
    """A chair that faces +x in its own frame."""
    return (Model(name, x, y, yaw)
            .legs(0.18, 0.18, 0.42, DARK_WOOD, 0.04)
            .box(0, 0, 0.42, 0.44, 0.44, 0.05, seat)
            .box(-0.20, 0, 0.47, 0.04, 0.44, 0.45, seat))


def furniture():
    out = []
    # Living room. The sofa and the armchair face the coffee table.
    out.append(Model('sofa', -2.6, 3.0)
               .box(0, 0, 0.08, 2.1, 0.9, 0.34, FABRIC)
               .box(0, 0.36, 0.42, 2.1, 0.18, 0.42, FABRIC)
               .box(-0.96, -0.02, 0.42, 0.18, 0.74, 0.22, FABRIC)
               .box(0.96, -0.02, 0.42, 0.18, 0.74, 0.22, FABRIC)
               .box(-0.44, -0.05, 0.42, 0.84, 0.66, 0.08, FABRIC_LIGHT, False)
               .box(0.44, -0.05, 0.42, 0.84, 0.66, 0.08, FABRIC_LIGHT, False))
    out.append(Model('armchair', -4.0, 1.5)
               .box(0, 0, 0.08, 0.85, 0.9, 0.34, FABRIC)
               .box(-0.34, 0, 0.42, 0.18, 0.9, 0.42, FABRIC)
               .box(0.05, -0.37, 0.42, 0.70, 0.16, 0.20, FABRIC)
               .box(0.05, 0.37, 0.42, 0.70, 0.16, 0.20, FABRIC)
               .box(0.07, 0, 0.42, 0.62, 0.56, 0.08, FABRIC_LIGHT, False))
    out.append(Model('coffee_table', -2.6, 1.7)
               .legs(0.45, 0.25, 0.38, DARK_WOOD)
               .box(0, 0, 0.38, 1.05, 0.62, 0.04, WOOD))
    shelf = Model('bookshelf', -4.3, -0.6).box(0, 0, 0, 0.32, 0.95, 1.25, DARK_WOOD)
    books = [(0.62, 0.24, 0.20), (0.20, 0.38, 0.55), (0.80, 0.68, 0.30), (0.25, 0.45, 0.35)]
    for row, z in enumerate((0.12, 0.50, 0.88)):
        for k in range(6):
            shelf.box(0.14, -0.36 + 0.145 * k, z, 0.06, 0.11, 0.26 - 0.02 * ((k + row) % 3),
                      books[(k + row) % 4], False)
    out.append(shelf)

    # Dining area and kitchen.
    out.append(Model('dining_table', 2.2, 1.4)
               .legs(0.62, 0.36, 0.72, DARK_WOOD, 0.06)
               .box(0, 0, 0.72, 1.45, 0.90, 0.05, WOOD))
    for k, (x, y, yaw) in enumerate([(1.25, 1.9, 0.0), (3.15, 1.9, 3.1416),
                                     (1.25, 0.9, 0.0), (3.15, 0.9, 3.1416)]):
        out.append(chair(f'dining_chair_{k + 1}', x, y, yaw))
    out.append(Model('refrigerator', 4.05, 3.05)
               .box(0, 0, 0, 0.72, 0.72, 1.50, STEEL)
               .box(-0.37, -0.25, 0.55, 0.02, 0.03, 0.60, BLACK, False))
    out.append(Model('kitchen_counter', 2.55, 3.18)
               .box(0, 0, 0, 1.9, 0.62, 0.86, WHITE)
               .box(0, 0, 0.86, 1.94, 0.66, 0.04, BLACK)
               .box(0.3, 0, 0.90, 0.5, 0.36, 0.01, STEEL, False))
    out.append(Model('cabinet', 0.9, 3.25)
               .box(0, 0, 0, 0.95, 0.45, 1.05, WOOD)
               .box(-0.03, -0.23, 0.45, 0.02, 0.02, 0.30, BLACK, False)
               .box(0.03, -0.23, 0.45, 0.02, 0.02, 0.30, BLACK, False))

    # Study corner.
    out.append(Model('desk', -3.2, -3.05)
               .legs(0.65, 0.27, 0.72, DARK_WOOD)
               .box(0, 0, 0.72, 1.45, 0.66, 0.04, WOOD)
               .box(0, 0.12, 0.76, 0.22, 0.14, 0.02, BLACK, False)
               .box(0, 0.14, 0.78, 0.03, 0.03, 0.14, BLACK, False)
               .box(0, 0.14, 0.86, 0.62, 0.03, 0.36, BLACK, False))
    out.append(Model('office_chair', -3.2, -2.3)
               .cyl(0, 0, 0, 0.26, 0.04, BLACK)
               .cyl(0, 0, 0.04, 0.03, 0.40, STEEL)
               .box(0, 0, 0.44, 0.46, 0.46, 0.07, BLACK)
               .box(0, 0.21, 0.51, 0.44, 0.05, 0.48, BLACK))

    # Bedroom.
    out.append(Model('bed', 3.2, -2.4)
               .box(0, 0, 0, 1.7, 2.1, 0.28, DARK_WOOD)
               .box(0, 0, 0.28, 1.6, 2.0, 0.20, LINEN)
               .box(0, 0.35, 0.48, 1.62, 1.32, 0.03, BLANKET, False)
               .box(-0.40, -0.72, 0.48, 0.60, 0.38, 0.10, WHITE, False)
               .box(0.40, -0.72, 0.48, 0.60, 0.38, 0.10, WHITE, False)
               .box(0, -1.08, 0, 1.7, 0.06, 0.95, DARK_WOOD))

    # Decoration.
    leaf, pot = (0.22, 0.42, 0.24), (0.80, 0.78, 0.74)
    for name, x, y in (('plant_1', -4.1, 3.1), ('plant_2', -4.1, -1.75)):
        out.append(Model(name, x, y)
                   .cyl(0, 0, 0, 0.17, 0.32, pot)
                   .ball(0, 0, 0.62, 0.26, leaf).ball(0.14, 0.08, 0.84, 0.20, leaf)
                   .ball(-0.12, -0.06, 0.92, 0.17, leaf).ball(0.0, 0.1, 1.05, 0.13, leaf))
    out.append(Model('lamp', -1.1, 3.2)
               .cyl(0, 0, 0, 0.15, 0.03, BLACK).cyl(0, 0, 0.03, 0.015, 1.35, BLACK)
               .cyl(0, 0, 1.30, 0.20, 0.28, (0.96, 0.90, 0.74), False))
    return ''.join(m.sdf() for m in out)


def textures(folder):
    """Paints the floorboards and the rug; returns the two file names."""
    import os
    import random
    from PIL import Image, ImageDraw, ImageFilter
    rnd = random.Random(7)
    scale = 440                                   # pixels per metre
    w, h = 9 * scale, 7 * scale
    floor = Image.new('RGB', (w, h))
    draw = ImageDraw.Draw(floor)
    board, length = int(0.11 * scale), int(1.3 * scale)
    for row in range(h // board + 1):
        x = -rnd.randrange(length)
        while x < w:
            n = int(length * rnd.uniform(0.7, 1.3))
            k = rnd.uniform(0.90, 1.06)
            tone = rnd.uniform(-0.02, 0.02)
            colour = tuple(int(255 * min(1, c * k + tone)) for c in (0.74, 0.60, 0.44))
            draw.rectangle([x, row * board, x + n, (row + 1) * board], fill=colour)
            for _ in range(5):                    # grain
                gy = row * board + rnd.randrange(board)
                shade = tuple(int(c * rnd.uniform(0.93, 0.98)) for c in colour)
                draw.line([x, gy, x + n, gy + rnd.randint(-3, 3)], fill=shade, width=2)
            draw.line([x + n, row * board, x + n, (row + 1) * board], fill=(120, 94, 66), width=3)
            x += n
        draw.line([0, row * board, w, row * board], fill=(120, 94, 66), width=3)
    floor = floor.filter(ImageFilter.GaussianBlur(1.2))
    floor.save(os.path.join(folder, 'floor.png'))

    rw, rh = int(2.6 * 300), int(1.8 * 300)
    rug = Image.new('RGB', (rw, rh), (58, 78, 96))
    draw = ImageDraw.Draw(rug)
    for inset, colour, width in ((24, (214, 204, 184), 10), (54, (214, 204, 184), 3)):
        draw.rectangle([inset, inset, rw - inset, rh - inset], outline=colour, width=width)
    rug.save(os.path.join(folder, 'rug.png'))
    return os.path.join(folder, 'floor.png'), os.path.join(folder, 'rug.png')


def camera(width, height):
    import math
    (x, y, z), (lx, ly, lz) = CAM_POS, CAM_LOOK
    dx, dy, dz = lx - x, ly - y, lz - z
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, math.hypot(dx, dy))
    return f'''
    <model name="video_camera">
      <static>true</static>
      <pose>{x} {y} {z} 0 {pitch:.5f} {yaw:.5f}</pose>
      <link name="link">
        <sensor name="camera" type="camera">
          <always_on>true</always_on>
          <update_rate>{FPS}</update_rate>
          <topic>video/image</topic>
          <camera>
            <horizontal_fov>{CAM_HFOV}</horizontal_fov>
            <image><width>{width}</width><height>{height}</height><format>R8G8B8</format></image>
            <clip><near>0.1</near><far>60</far></clip>
            <anti_aliasing>4</anti_aliasing>
          </camera>
        </sensor>
      </link>
    </model>
'''


def main():
    src, dst = sys.argv[1], sys.argv[2]
    width, height = (int(sys.argv[3]), int(sys.argv[4])) if len(sys.argv) > 4 else (2880, 1620)
    rtf = sys.argv[5] if len(sys.argv) > 5 else '0.1'
    world = open(src).read()
    world = re.sub(r'\s*<include>.*?</include>', '', world, flags=re.S)
    world = world.replace('<real_time_factor>1.0</real_time_factor>',
                          f'<real_time_factor>{rtf}</real_time_factor>')
    # A slightly warmer, softer look than the demo world's defaults.
    world = world.replace('<background>0.75 0.8 0.85 1</background>',
                          '<background>0.09 0.11 0.15 1</background>')
    world = world.replace('<size>40 40</size></plane></geometry>\n          <material>'
                          '<ambient>0.5 0.5 0.5 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse>',
                          '<size>40 40</size></plane></geometry>\n          <material>'
                          '<ambient>0.10 0.12 0.16 1</ambient><diffuse>0.10 0.12 0.16 1</diffuse>')
    assert '<include>' not in world

    import os
    floor, rug = textures(os.path.dirname(os.path.abspath(dst)))

    def textured(world, colour, image):
        old = (f'<material><ambient>{colour} 1</ambient><diffuse>{colour} 1</diffuse>'
               '<specular>0.1 0.1 0.1 1</specular></material>')
        assert world.count(old) == 1, colour
        return world.replace(old, '<material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>'
                             '<specular>0.05 0.05 0.05 1</specular><pbr><metal>'
                             f'<albedo_map>{image}</albedo_map><roughness>0.8</roughness>'
                             '<metalness>0</metalness></metal></pbr></material>')
    world = textured(world, '0.62 0.46 0.30', floor)
    world = textured(world, '0.45 0.20 0.20', rug)
    world = world.replace('<direction>-0.4 0.3 -0.9</direction>',
                          '<direction>-0.45 0.25 -1.0</direction>')
    world = world.replace('<ambient>0.6 0.6 0.6 1</ambient>', '<ambient>0.8 0.8 0.8 1</ambient>')
    world = world.replace('</world>', furniture() + camera(width, height) + '  </world>')
    open(dst, 'w').write(world)


if __name__ == '__main__':
    main()
