import os
import sys
import argparse

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import yaml

parser = argparse.ArgumentParser()
parser.add_argument('--input', type=str)
parser.add_argument('--output', type=str, default='observation.mp4')

args = parser.parse_args()

FN = args.input
OUT = args.output

if not os.path.isfile(FN):
    print(f"{FN} not found")
    sys.exit(1)

DATA = yaml.safe_load(open(FN))

map_name_id = {
    DATA["key_frame"][i]["name"]: i
    for i in DATA["key_frame"].keys()
}
frames = [map_name_id[n] for n in sorted(list(map_name_id.keys()))]

pts = list(zip(*[(p["pos"][0], p["pos"][1]) for p in DATA["mark"].values()]))
kfs = list(zip(*[(p["tf"][0], p["tf"][1])
                 for p in DATA["key_frame"].values()]))


def animate(idx, line):
    # print(idx, DATA["key_frame"][idx]["name"])
    pos1 = DATA["key_frame"][idx]["tf"][:2]
    pos2 = list(
        zip(*[
            DATA["mark"][i_pt]["pos"][:2]
            for i_pt in DATA["key_frame"][idx]["has"]
        ]))
    pos_x = [
        pos2[0][i // 2] if i % 2 == 0 else pos1[0]
        for i in range(len(pos2[0]) * 2)
    ]
    pos_y = [
        pos2[1][i // 2] if i % 2 == 0 else pos1[1]
        for i in range(len(pos2[1]) * 2)
    ]
    line.set_xdata(pos_x)
    line.set_ydata(pos_y)
    line.set_color("g")
    line.set_alpha(0.7)
    return line,


xs = kfs[0] + pts[0]
ys = kfs[1] + pts[1]

fig1 = plt.figure()
plt.xlim(min(xs) - 0.3, max(xs) + 0.3)
plt.ylim(min(ys) - 0.3, max(ys) + 0.3)
plt.xlabel("X")
plt.ylabel("Y")
plt.grid(1)
plt.gca().set_aspect('equal', 'box')
plt.plot(pts[0], pts[1], ".r", alpha=0.3)
plt.plot(kfs[0], kfs[1], ".b", alpha=0.3)
line, = plt.plot([])

writer = animation.writers['ffmpeg'](fps=2, bitrate=1800)
animation.FuncAnimation(fig1, animate, (i for i in frames),
                        fargs=(line, )).save(OUT, writer=writer)
