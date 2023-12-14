"""
BSD 3-Clause License

Copyright (c) 2022, VicenteQueiroz
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""

from matplotlib.widgets import Slider, Button
import matplotlib as mpl
from matplotlib import pyplot as plt
import numpy as np
import math

# import dubins
from dubins_path import dubins_path, optimized_dubins_path

# Initial conditions for plot
theta0 = 0
theta1 = 160

p0 = (2, 2, theta0)
p1 = (8, 8, theta1)
turning_radius = 1
step_size = 0.5

# dubins_xx, dubins_yy, dubins_yaws = dubins_path(p0, p1, turning_radius)
dubins_xx, dubins_yy, dubins_yaws = optimized_dubins_path(p0, p1)

N = 2
xmax = 10
x = [p0[0], p1[0]]
yvals = [p0[1], p1[1]]
thetavals = [theta0, theta1]

# figure.subplot.right
mpl.rcParams["figure.subplot.right"] = 0.8

# set up a plot
fig, axes = plt.subplots(1, 1, figsize=(9.0, 8.0), sharex=True)
ax1 = axes
pind = None  # active point
epsilon = 10  # max pixel distance


def update(val):
    global x
    global yvals
    global thetavals
    global arrow0
    global arrow1

    # update value of sliders
    for i in np.arange(N):
        thetavals[i] = sliders[i].val
    turning_radius = sliders[len(sliders) - 1].val

    # update p0 and p1 positions
    l.set_xdata(x)
    l.set_ydata(yvals)

    # update the dubins curve
    # dubins_xx, dubins_yy, dubins_yaws = dubins_path(
    #     [x[0], yvals[0], thetavals[0]],
    #     [x[1], yvals[1], thetavals[1]],
    #     1 / turning_radius,
    # )
    dubins_xx, dubins_yy, dubins_yaws = optimized_dubins_path(
        [x[0], yvals[0], thetavals[0]], [x[1], yvals[1], thetavals[1]]
    )
    m.set_xdata(dubins_xx)
    m.set_ydata(dubins_yy)

    # clean arrows and update them
    try:
        arrow0.remove()
        arrow1.remove()

        arrow0 = ax1.arrow(
            x[0],
            yvals[0],
            math.cos(math.radians(thetavals[0])),
            math.sin(math.radians(thetavals[0])),
            head_width=0.05,
            head_length=0.1,
            fc="k",
            ec="k",
        )
        arrow1 = ax1.arrow(
            x[1],
            yvals[1],
            math.cos(math.radians(thetavals[1])),
            math.sin(math.radians(thetavals[1])),
            head_width=0.05,
            head_length=0.1,
            fc="k",
            ec="k",
        )
    except:
        print("No arrow to remove")

    # redraw canvas while idle
    # fig.canvas.draw_idle()


# Update the turning radius slider
def turningRadiusUpdate(val):
    global turning_radius

    turning_radius = sliders[len(sliders) - 1].val

    # dubins_xx, dubins_yy, dubins_yaws = dubins_path(
    #     [x[0], yvals[0], thetavals[0]],
    #     [x[1], yvals[1], thetavals[1]],
    #     1 / turning_radius,
    # )
    dubins_xx, dubins_yy, dubins_yaws = optimized_dubins_path(
        [x[0], yvals[0], thetavals[0]],
        [x[1], yvals[1], thetavals[1]],
    )
    m.set_xdata(dubins_xx)
    m.set_ydata(dubins_yy)


def reset(event):
    global x
    global thetavals
    global yvals
    global arrow0
    global arrow1

    # Clean arrows
    arrow0.remove()
    arrow1.remove()

    # reset the values
    x = [p0[0], p1[0]]
    yvals = [p0[1], p1[1]]
    thetavals = [theta0, theta1]
    turning_radius = 1

    for i in np.arange(len(sliders)):
        sliders[i].reset()
    # spline = inter.InterpolatedUnivariateSpline(x, yvals)
    l.set_xdata(x)
    l.set_ydata(yvals)

    # dubins_xx, dubins_yy, dubins_yaws = dubins_path(
    #     [x[0], yvals[0], thetavals[0]],
    #     [x[1], yvals[1], thetavals[1]],
    #     1 / turning_radius,
    # )
    dubins_xx, dubins_yy, dubins_yaws = optimized_dubins_path(
        [x[0], yvals[0], thetavals[0]],
        [x[1], yvals[1], thetavals[1]],
    )

    m.set_xdata(dubins_xx)
    m.set_ydata(dubins_yy)

    arrow0 = ax1.arrow(
        x[0],
        yvals[0],
        math.cos(math.radians(thetavals[0])),
        math.sin(math.radians(thetavals[0])),
        head_width=0.05,
        head_length=0.1,
        fc="k",
        ec="k",
    )
    arrow1 = ax1.arrow(
        x[1],
        yvals[1],
        math.cos(math.radians(thetavals[1])),
        math.sin(math.radians(thetavals[1])),
        head_width=0.05,
        head_length=0.1,
        fc="k",
        ec="k",
    )


def button_press_callback(event):
    "whenever a mouse button is pressed"
    global pind
    if event.inaxes is None:
        return
    if event.button != 1:
        return
    pind = get_ind_under_point(event)


def button_release_callback(event):
    "whenever a mouse button is released"
    global pind
    if event.button != 1:
        return
    pind = None


def get_ind_under_point(event):
    "get the index of the vertex under point if within epsilon tolerance"

    # display coords
    # print('display x is: {0}; display y is: {1}'.format(event.x,event.y))
    t = ax1.transData.inverted()
    tinv = ax1.transData
    xy = t.transform([event.x, event.y])
    # print('data x is: {0}; data y is: {1}'.format(xy[0],xy[1]))
    xr = np.reshape(x, (np.shape(x)[0], 1))
    yr = np.reshape(yvals, (np.shape(yvals)[0], 1))
    xy_vals = np.append(xr, yr, 1)
    xyt = tinv.transform(xy_vals)
    xt, yt = xyt[:, 0], xyt[:, 1]
    d = np.hypot(xt - event.x, yt - event.y)
    (indseq,) = np.nonzero(d == d.min())
    ind = indseq[0]

    # print(d[ind])
    if d[ind] >= epsilon:
        ind = None

    # print(ind)
    return ind


def motion_notify_callback(event):
    "on mouse movement"
    global x
    global yvals
    global thetavals

    if pind is None:
        return
    if event.inaxes is None:
        return
    if event.button != 1:
        return

    # update x and yvals
    # print('motion x: {0}; y: {1}'.format(event.xdata,event.ydata))
    x[pind] = event.xdata
    yvals[pind] = event.ydata

    # update theha of p0 and p1
    sliders[pind].set_val(thetavals[pind])
    fig.canvas.draw_idle()


# Draggable Points
(l,) = ax1.plot(
    x,
    yvals,
    color="k",
    linestyle="none",
    marker="o",
    markersize=8,
)

arrow0 = ax1.arrow(
    x[0],
    yvals[0],
    math.cos(math.radians(thetavals[0])),
    math.sin(math.radians(thetavals[0])),
    head_width=0.05,
    head_length=0.1,
    fc="k",
    ec="k",
)
arrow1 = ax1.arrow(
    x[1],
    yvals[1],
    math.cos(math.radians(thetavals[1])),
    math.sin(math.radians(thetavals[1])),
    head_width=0.05,
    head_length=0.1,
    fc="k",
    ec="k",
)

# Dubins curve
(m,) = ax1.plot(dubins_xx, dubins_yy, "r-", label="dubins")

ax1.set_yscale("linear")
ax1.set_xlim(0, xmax)
ax1.set_ylim(0, xmax)
ax1.set_xlabel("x")
ax1.set_ylabel("y")
ax1.grid(True)
ax1.yaxis.grid(True, which="minor", linestyle="--")
ax1.legend(loc=2, prop={"size": 12})

sliders = []

# Define the p0 and p1 angle sliders
for i in np.arange(2):
    axamp = plt.axes([0.84, 0.8 - (i * 0.05), 0.12, 0.02])
    # Slider
    s = Slider(axamp, "θ{0}".format(i), 0, 360, valinit=thetavals[i])
    sliders.append(s)


for i in np.arange(2):
    sliders[i].on_changed(update)

# Add turning radius slider
axamp = plt.axes([0.84, 0.8 - (3 * 0.05), 0.12, 0.02])
s = Slider(axamp, "curv", 0.1, 5, valinit=turning_radius)
sliders.append(s)
sliders[len(sliders) - 1].on_changed(turningRadiusUpdate)

axres = plt.axes([0.84, 0.8 - ((N) * 0.05), 0.12, 0.02])
bres = Button(axres, "Reset")
bres.on_clicked(reset)

fig.canvas.set_window_title("Dubins Visualizer")
fig.canvas.mpl_connect("button_press_event", button_press_callback)
fig.canvas.mpl_connect("button_release_event", button_release_callback)
fig.canvas.mpl_connect("motion_notify_event", motion_notify_callback)

plt.show()
