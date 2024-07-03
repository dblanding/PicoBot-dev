# arena.py
"""Represent the lines bounding arena (dimensions in m)"""

width = 1.500
height = 1.500
cutout_width = 0.500
cutout_height = 0.500

boundary_lines = [
    [(0,0), (0, height)],
    [(0, height), (width, height)],
    [(width, height), (width, cutout_height)],
    [(width, cutout_height), (width - cutout_width, cutout_height)],
    [(width - cutout_width, cutout_height), (width - cutout_width, 0)],
    [(width - cutout_width, 0), (0, 0)],
    ]
