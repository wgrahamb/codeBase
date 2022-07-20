import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib.animation as ma
from matplotlib.lines import Line2D
import pickle
import os
import pandas

fig = plt.figure(tight_layout=True)
ax = fig.add_subplot(111, projection='3d')
ax.view_init(elev=10, azim=315)
ax.invert_xaxis()
ax.set_xlim(left=20000,
            right=0)
ax.set_xlabel("East")
ax.set_ylim(top=20000,
            bottom=0)
ax.set_ylabel("North")
ax.set_zlim(top=20000,
            bottom=0)
ax.set_zlabel("Up")

dirname = os.getcwd()
simdata = pickle.load(open(r"{}\NOVICE.pickle".format(dirname), "rb"))

timeindex = []
for packet in simdata:
    time = (round(float(packet), 1))
    if time not in timeindex and time <= float(packet):
        timeindex.append(time)

artists = []

for asset in simdata["0.05"]["assets"]:
    line, = ax.plot([], [], [], marker="s", markersize=10, color="blue")
    artists.append(line, )

for launcher in simdata["0.05"]["launchers"]:
    line, = ax.plot([], [], [], marker="v", markersize=10, color="green")
    artists.append(line,)

for threat in simdata["0.05"]["threats"]:
    line, = ax.plot([], [], [], marker="2", markersize=20, color="red")
    artists.append(line,)

for soldier in simdata["0.05"]["soldiers"]:
    line, = ax.plot([], [], [], marker="1", markersize=20, color="blue")
    artists.append(line,)
    for evaluation in soldier["evaluation"]:
        line, = ax.plot([], [], [], color="red", linewidth=0.5)
        artists.append(line,)
    for tca in soldier["tca"]:
        line, = ax.plot([], [], [], color="red", linewidth=0.5)
        artists.append(line, )
    for projectory in soldier["projectory"]:
        line, = ax.plot([], [], [], color="blue", linewidth=0.5)
        artists.append(line,)
    for ldc in soldier["ldc"]:
        line, = ax.plot([], [], [], color="blue", linewidth=0.5)
        artists.append(line,)

def update(frames, data):
    returns = []
    index = -1
    for a in data["{}".format(frames)]["assets"]:
        index += 1
        artists[index].set_data(a["pos"][0],
                                a["pos"][1])
        artists[index].set_3d_properties(a["pos"][2])
        returns.append(artists[index])
    for la in data["{}".format(frames)]["launchers"]:
        index += 1
        artists[index].set_data(la["pos"][0],
                                la["pos"][1])
        artists[index].set_3d_properties(la["pos"][2])
        returns.append(artists[index])
    for th in data["{}".format(frames)]["threats"]:
        index += 1
        artists[index].set_data(th["pos"][0],
                                th["pos"][1])
        artists[index].set_3d_properties(th["pos"][2])
        returns.append(artists[index])
    for so in data["{}".format(frames)]["soldiers"]:
        index += 1
        artists[index].set_data(so["initpos"][0],
                                so["initpos"][1])
        artists[index].set_3d_properties(so["initpos"][2])
        returns.append(artists[index])
        for ev in so["evaluation"]:
            index += 1
            artists[index].set_data(ev.iloc[:, 0],
                                    ev.iloc[:, 1])
            artists[index].set_3d_properties(ev.iloc[:, 2])
            returns.append(artists[index])
        for tshape in so["tca"]:
            index += 1
            artists[index].set_data(tshape.iloc[:, 1],
                                    tshape.iloc[:, 2])
            artists[index].set_3d_properties(tshape.iloc[:, 3])
            returns.append(artists[index])
        for pr in so["projectory"]:
            index += 1
            artists[index].set_data(pr.iloc[:, 0],
                                    pr.iloc[:, 1])
            artists[index].set_3d_properties(pr.iloc[:, 2])
            returns.append(artists[index])
        for lshape in so["ldc"]:
            index += 1
            artists[index].set_data(lshape.iloc[:, 1],
                                    lshape.iloc[:, 2])
            artists[index].set_3d_properties(lshape.iloc[:, 3])
            returns.append(artists[index])
    title = ax.set_title(label=frames)
    returns.append(title)
    return returns

animation = FuncAnimation(
    fig=fig,
    func=update,
    frames=timeindex,
    fargs=([simdata]),
    repeat=True,
    repeat_delay=1500,
    blit=True,
    interval=1
)

plt.tight_layout()
plt.show()
