import os
import argparse
import re
import sys

import cv2
import numpy
import yaml

parser = argparse.ArgumentParser()
parser.add_argument('--yaml', type=str)
parser.add_argument('--in-dir', type=str)
parser.add_argument('--out-dir', type=str)

args = parser.parse_args()

if any(a is None for a in (args.yaml, args.in_dir, args.out_dir)):
    parser.print_help()
    sys.exit(1)

DATA = yaml.safe_load(open(args.yaml))
map_id_feature = {
    re.findall("\d+\.\d+", DATA["key_frame"][i]["name"])[0]:
    DATA["key_frame"][i]["px"]
    for i in DATA["key_frame"].keys()
}

if os.path.isdir(args.out_dir):
    os.system(f"rm -rf {args.out_dir}")
os.mkdir(args.out_dir)

files = sorted([
    fn for fn in os.listdir(args.in_dir)
    if re.findall("\d+\.\d+", fn)[0] in map_id_feature
])
for i in range(1, len(files)):
    img0 = cv2.imread(os.path.join(args.in_dir, files[i - 1]))
    img1 = cv2.imread(os.path.join(args.in_dir, files[i]))

    n0 = re.findall("\d+\.\d+", files[i - 1])[0]
    n1 = re.findall("\d+\.\d+", files[i])[0]

    i0 = set(map_id_feature[n0].keys())
    i1 = set(map_id_feature[n1].keys())
    shared = i0.intersection(i1)
    # i1 = i1.difference(shared)
    # i0 = i0.difference(shared)

    canvas = numpy.zeros([img0.shape[0] * 2, img0.shape[1], img0.shape[2]],
                         dtype=img0.dtype)
    canvas[:img0.shape[0], :, :] = img0
    canvas[img0.shape[0]:, :, :] = img1

    for i in map_id_feature[n0]:
        pt = map_id_feature[n0][i]
        cv2.circle(canvas, (int(round(pt[0])), int(round(pt[1]))),
                   3,
                   color=(255, 0, 0))
    for i in map_id_feature[n1]:
        pt = map_id_feature[n1][i]
        cv2.circle(canvas,
                   (int(round(pt[0])), int(round(pt[1] + img0.shape[0]))),
                   3,
                   color=(255, 0, 0))
    for i in shared:
        pt0 = map_id_feature[n0][i]
        pt1 = map_id_feature[n1][i]
        cv2.line(canvas, (int(round(pt0[0])), int(round(pt0[1]))),
                 (int(round(pt1[0])), int(round(pt1[1] + img0.shape[0]))),
                 thickness=1,
                 color=(0, 0, 255))

    cv2.imwrite(os.path.join(args.out_dir, n0 + n1 + ".png"), canvas)
    # cv2.imshow("", canvas)
    # cv2.waitKey(100)
