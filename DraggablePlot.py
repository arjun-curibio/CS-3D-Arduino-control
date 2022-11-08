import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backend_bases import MouseEvent


class DraggablePlot():
    u""" An example of plot with draggable markers """

    def __init__(self):
        self._figure, self._axes, self._line = None, None, None
        self._dragging_point = None
        self._points = {0:0, 40:50, 60:50, 70:0}
        
        self._init_plot()
        self._update_plot()
        
        

    def _init_plot(self):
        self._figure = plt.figure("Example plot")
        axes = plt.subplot(1, 1, 1)
        axes.set_xlim(0, 100)
        axes.set_ylim(0, 100)
        axes.grid(which="both")
        self._axes = axes

        self._figure.canvas.mpl_connect('button_press_event', self._on_click)
        self._figure.canvas.mpl_connect('button_release_event', self._on_release)
        self._figure.canvas.mpl_connect('motion_notify_event', self._on_motion)
        plt.show()

    def _update_plot(self):
        if not self._points:
            self._line.set_data([], [])
            print('1')
        else:
            x, y = zip(*sorted(self._points.items()))
            # print(x)
            print(y)
            x_points_list = list(x)
            if x_points_list[-1] > 90:
                x_points_list[-1] = 90
            x = tuple(x_points_list)
            for i in range(1,len(x)):
                x_distance = x[i]-x[i-1]
                if x_distance < 10:
                    x_list = list(x)
                    x_list[0] = 0
                    x_list[i] = x_list[i-1]+10
                    x = tuple(x_list)
            # Add new plot
            if not self._line:
                self._line, = self._axes.plot(x, y, "b", marker="o", markersize=10)
            # Update current plot
            else:
                self._points = dict(zip(x,y))
                self._line.set_data(x, y)
        
        
        self._figure.canvas.draw()
        print(self._line.get_data())
        # print(x)
        self._axes.set_title(x)

    def _add_point(self, x, y=None):
        if isinstance(x, MouseEvent):
            x, y = int(x.xdata), int(x.ydata)
        self._points[x] = y
        return x, y

    def _remove_point(self, x, _):
        if x in self._points:
            self._points.pop(x)

    def _find_neighbor_point(self, event):
        u""" Find point around mouse position
        :rtype: ((int, int)|None)
        :return: (x, y) if there are any point around mouse else None
        """
        distance_threshold = 10
        nearest_point = None
        min_distance = math.sqrt(2 * (100 ** 2))
        for x, y in self._points.items():
            distance = math.hypot(event.xdata - x, event.ydata - y)
            if distance < min_distance:
                min_distance = distance
                nearest_point = (x, y)
        if min_distance < distance_threshold:
            return nearest_point
        return None

    def _on_click(self, event):
        u""" callback method for mouse click event
        :type event: MouseEvent
        """
        # left click
        if event.button == 1 and event.inaxes in [self._axes]:
            point = self._find_neighbor_point(event)
            if point:
                self._dragging_point = point
                # print(self._dragging_point)
                hold_y = self._dragging_point[1]
                # print(hold_y)
            else:
                # self._add_point(event)
                pass
            self._update_plot()
        # right click
        # elif event.button == 3 and event.inaxes in [self._axes]:
        #     point = self._find_neighbor_point(event)
        #     if point:
        #         self._remove_point(*point)
        #         self._update_plot()

    def _on_release(self, event):
        u""" callback method for mouse release event
        :type event: MouseEvent
        """
        if event.button == 1 and event.inaxes in [self._axes] and self._dragging_point:
            self._dragging_point = None
            self._update_plot()

    def _on_motion(self, event):
        u""" callback method for mouse motion event
        :type event: MouseEvent
        """
        if not self._dragging_point:
            return
        if event.xdata is None or event.ydata is None:
            return
        event.ydata = self._dragging_point[1]
        if event.xdata > 90:
            event.xdata = 90
        
        print(event)
        self._remove_point(*self._dragging_point)
        self._dragging_point = self._add_point(event)
        
        self._update_plot()

if __name__ == '__main__':
    fig = DraggablePlot()