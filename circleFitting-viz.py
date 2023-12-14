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

# get the circle equation (x - a)**2 + (y - b)**2 = r**2
# then add the points of the circle
def circle_path(p1, p2, p3):
    # get the b coordinate:
    b_denominator = 2 * (
        p2[1] * p1[1]
        - p2[0] * p3[1]
        + p1[0] * p3[1]
        - p1[0] * p1[1]
        + p3[0] * p2[1]
        - p1[0] * p2[1]
    )

    if b_denominator == 0:
        print("Can't get circle")
        return

    b_nominator = (
        p3[0] * (p2[0] ** 2 - p1[0] ** 2 + p2[1] ** 2 - p1[1] ** 2)
        + p1[0] * (2 * p1[1] ** 2 + p3[0] ** 2 + p3[1] ** 2 - p2[0] ** 2 - p2[1] ** 2)
        + p2[0] * (p1[0] ** 2 + p1[1] ** 2 - p3[0] ** 2 - p3[1] ** 2)
    )

    b = b_nominator / b_denominator

    # get a coordinate

    a_denominator = 2 * (p3[0] - p1[0])

    if a_denominator == 0:
        print("Can't get a circle")

    a_nominator = (
        p3[0] ** 2 + p3[1] ** 2 - 2 * p3[1] * b + 2 * p1[1] - p1[0] ** 2 - p1[1] ** 2
    )

    a = a_nominator / a_denominator

    # Get the radius:
    r = math.sqrt((p1[0] - a) ** 2 + (p1[1] - b) ** 2)

    # Generate the points of the circle, use parametric coordinates:
    # x = r * cos(t) + a  ,  y = r * sin(t) + b

    # number of points do generate
    n = 100
    xx_points = []
    yy_points = []

    for x in range(0, n + 1):
        xx_points.append(math.cos(2 * math.pi / n * x) * r + a)
        yy_points.append(math.sin(2 * math.pi / n * x) * r + b)

    return xx_points, yy_points


# Initial conditions for plot

p0 = (2, 2)
p1 = (8, 8)
p2 = (10, 10)

circle_xx, circle_yy = circle_path(p0, p1, p2)

N = 2
xmax = 20
x = [p0[0], p1[0], p2[0]]
yvals = [p0[1], p1[1], p2[1]]

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

    # update p0 and p1 positions
    l.set_xdata(x)
    l.set_ydata(yvals)

    # update the dubins curve
    circle_xx, circle_yy = circle_path(
        [x[0], yvals[0]], [x[1], yvals[1]], [x[2], yvals[2]]
    )
    m.set_xdata(circle_xx)
    m.set_ydata(circle_yy)

    # redraw canvas while idle
    # fig.canvas.draw_idle()


def reset(event):
    global x
    global yvals

    # reset the values
    x = [p0[0], p1[0], p2[0]]
    yvals = [p0[1], p1[1], p2[1]]

    l.set_xdata(x)
    l.set_ydata(yvals)

    circle_xx, circle_yy = circle_path(
        [x[0], yvals[0]], [x[1], yvals[1]], [x[2], yvals[2]]
    )
    m.set_xdata(circle_xx)
    m.set_ydata(circle_yy)


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
    # print("display x is: {0}; display y is: {1}".format(event.x, event.y))
    t = ax1.transData.inverted()
    tinv = ax1.transData
    xy = t.transform([event.x, event.y])
    # print("data x is: {0}; data y is: {1}".format(xy[0], xy[1]))
    print("x: ", x)
    xr = np.reshape(x, (np.shape(x)[0], 1))
    print("xr: ", xr)
    yr = np.reshape(yvals, (np.shape(yvals)[0], 1))
    xy_vals = np.append(xr, yr, 1)
    xyt = tinv.transform(xy_vals)
    xt, yt = xyt[:, 0], xyt[:, 1]
    print("xt: ", xt)
    d = np.hypot(xt - event.x, yt - event.y)
    (indseq,) = np.nonzero(d == d.min())
    print("indseq: ", indseq)
    ind = indseq[0]

    # print(d[ind])
    if d[ind] >= epsilon:
        ind = None

    print(ind)
    return ind


def motion_notify_callback(event):
    "on mouse movement"
    global x
    global yvals

    if pind is None:
        return
    if event.inaxes is None:
        return
    if event.button != 1:
        return

    # update x and yvals
    # print("motion x: {0}; y: {1}".format(event.xdata, event.ydata))
    x[pind] = event.xdata
    yvals[pind] = event.ydata

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

# Circle curve
(m,) = ax1.plot(circle_xx, circle_yy, "r-", label="circle")

ax1.set_yscale("linear")
ax1.set_xlim(0, xmax)
ax1.set_ylim(0, xmax)
ax1.set_xlabel("x")
ax1.set_ylabel("y")
ax1.grid(True)
ax1.yaxis.grid(True, which="minor", linestyle="--")
ax1.legend(loc=2, prop={"size": 12})


axres = plt.axes([0.84, 0.8 - ((N) * 0.05), 0.12, 0.02])
bres = Button(axres, "Reset")
bres.on_clicked(reset)

fig.canvas.set_window_title("Circle Fitting")
fig.canvas.mpl_connect("button_press_event", button_press_callback)
fig.canvas.mpl_connect("button_release_event", button_release_callback)
fig.canvas.mpl_connect("motion_notify_event", motion_notify_callback)

plt.show()