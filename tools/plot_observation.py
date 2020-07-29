import os
import sys

import matplotlib.pyplot as plt
import yaml

MIN_OCCUR = 20

FN = sys.argv[1]

if not os.path.isfile(FN):
    print(f"{FN} not found")
    sys.exit(1)

DATA = yaml.safe_load(open(FN))

# landmarks which show in more than MIN_OCCUR frames
shared_marks = set(i_pt for i_pt in DATA["mark"]
                   if len(DATA["mark"][i_pt]["in"]) > MIN_OCCUR)

for i_kf in DATA["key_frame"]:
    pos1 = DATA["key_frame"][i_kf]["tf"][:2]
    plt.plot(pos1[0], pos1[1], "ob")
    for i_pt in set(DATA["key_frame"][i_kf]["has"]).intersection(shared_marks):
        pos2 = DATA["mark"][i_pt]["pos"][:2]
        plt.plot([pos1[0], pos2[0]], [pos1[1], pos2[1]], "g", alpha=0.3)

plt.gca().set_aspect('equal', 'box')
plt.grid(1)
plt.show()
