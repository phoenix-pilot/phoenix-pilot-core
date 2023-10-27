from dataclasses import dataclass

from matplotlib.artist import Artist
from matplotlib.axes import Axes


@dataclass
class _HideablePlot:
    plot: Artist
    annotations: list[Artist]


class InteractiveLegendFactory:
    def __init__(self, ax: Axes) -> None:
        self._hideables: list[_HideablePlot] = []
        self.ax = ax

        # Maps legend elements with plots
        self._legend_map: dict[Artist, _HideablePlot] = {}

    def add_hideable_plot(self, plot: Artist, annotations: list[Artist] = []):
        self._check_axes(plot)

        for annotation in annotations:
            self._check_axes(annotation)

        self._hideables.append(_HideablePlot(plot, annotations))

    def _check_axes(self, artist: Artist):
        if artist.axes is None:
            artist.axes = self.ax
        elif artist.axes is not self.ax:
            raise Exception("Incorrect axes instance")

    def generate_legend(self):
        all_plots = [hideable.plot for hideable in self._hideables]

        legend = self.ax.legend(handles=all_plots)

        for legend_line, hideable in zip(legend.legendHandles, self._hideables):
            # Enable clicking on line
            legend_line.set_picker(True)
            legend_line.set_pickradius(5)

            self._legend_map[legend_line] = hideable

        self.ax.figure.canvas.mpl_connect('pick_event', self.__on_pick_event)

    def __on_pick_event(self, event):
        legend_line = event.artist

        if legend_line not in self._legend_map.keys():
            return

        hideable = self._legend_map[legend_line]
        visible = not hideable.plot.get_visible()

        hideable.plot.set_visible(visible)
        for annotation in hideable.annotations:
            annotation.set_visible(visible)

        legend_line.set_alpha(1.0 if visible else 0.2)
        legend_line.axes.figure.canvas.draw()
