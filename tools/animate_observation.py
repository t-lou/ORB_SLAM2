import os
import sys
import argparse
import re

import numpy
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import yaml

parser = argparse.ArgumentParser()
parser.add_argument('--input', type=str)
parser.add_argument('--output', type=str, default='observation.mp4')
parser.add_argument('--gt', type=str)

args = parser.parse_args()

FN = args.input
OUT = args.output
GT = args.gt

gt_tf = None
if GT is not None and os.path.isfile(GT):
    gt_tf = numpy.genfromtxt(GT, delimiter=" ", skip_header=3)


def interp_tf(t0, tf0, t1, tf1, t):
    assert t0 <= t <= t1, f"time disorder {t0} {t1} {t}"
    if t0 == t:
        return tf0
    elif t1 == t:
        return tf1
    else:
        p = (t - t0) / (t1 - t0)
        tr = list(
            map(lambda i: p * i[1] + (1 - p) * i[0],
                list(zip(tf0, tf1))[:3]))
        rot = Slerp([t0, t1], R.from_quat([tf0[3:],
                                           tf1[3:]]))(t).as_quat().tolist()
        return tr + rot
    return None


def get_gt_tf(timestamp: float):
    if gt_tf[0, 0] < timestamp < gt_tf[-1, 0]:
        id_p = numpy.where(gt_tf[:, 0] <= timestamp)[0][-1]
        id_n = numpy.where(gt_tf[:, 0] >= timestamp)[0][0]
        return interp_tf(gt_tf[id_p, 0], gt_tf[id_p, 1:], gt_tf[id_n, 0],
                         gt_tf[id_n, 1:], timestamp)
    else:
        return None


def get_gt_tf_from_name(name: str):
    floats = [float(m) for m in re.findall("\d+\.\d+", name)]
    if len(floats) == 1:
        return get_gt_tf(floats[0])
    else:
        print(f"name {name} should contain one float")


if not os.path.isfile(FN):
    print(f"{FN} not found")
    sys.exit(1)

DATA = yaml.safe_load(open(FN))

map_name_id = {
    DATA["key_frame"][i]["name"]: i for i in DATA["key_frame"].keys()
}
sorted_frame_name = sorted(list(map_name_id.keys()))
frames = [map_name_id[n] for n in sorted_frame_name]

# reset the ground truth tf with the first key-frame time
if gt_tf is not None:
    matched_gt_tf = None
    for frame_name in sorted_frame_name:
        matched_gt_tf = get_gt_tf_from_name(frame_name)
        if matched_gt_tf is not None:
            matched_tf = DATA["key_frame"][map_name_id[frame_name]]["tf"]
            break
    if matched_gt_tf is None:
        print("ground truth and key frames not matched in time")
        sys.exit(1)

    def tf2mat(tf):
        re = numpy.eye(4)
        re[0, 3], re[1, 3], re[2, 3] = tf[0], tf[1], tf[2]
        re[:3, :3] = R.from_quat(tf[3:]).as_matrix()
        return re

    def mat2tf(mat):
        tl = [mat[0, 3], mat[1, 3], mat[2, 3]]
        rot = R.from_matrix(mat[:3, :3]).as_quat().tolist()
        return tl + rot

    conv = tf2mat(matched_tf) @ numpy.linalg.pinv(tf2mat(matched_gt_tf))

    for i in range(gt_tf.shape[0]):
        gt_tf[i, 1:] = numpy.array(mat2tf(conv @ tf2mat(gt_tf[i, 1:].tolist())))

pts = list(zip(*[(p["pos"][0], p["pos"][1]) for p in DATA["mark"].values()]))
kfs = list(zip(*[(p["tf"][0], p["tf"][1]) for p in DATA["key_frame"].values()]))


def animate(idx, line, gt_line):
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

    if gt_tf is not None:
        gt_pos = get_gt_tf_from_name(DATA["key_frame"][idx]["name"])
        if gt_pos is not None:
            gt_line.set_xdata(gt_pos[0])
            gt_line.set_ydata(gt_pos[1])
            gt_line.set_color("black")
            gt_line.set_marker("o")
            gt_line.set_alpha(1.0)
    return line, gt_line


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
gt_line, = plt.plot([])

writer = animation.writers['ffmpeg'](fps=2, bitrate=1800)
animation.FuncAnimation(fig1,
                        animate, (i for i in frames),
                        fargs=(line, gt_line)).save(OUT, writer=writer)
