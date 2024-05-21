import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import pandas as pd
import argparse
import os

from proto.track_pb2 import Track, SegmentType, CornerDirection
from proto.visualization_config_pb2 import VisualizationConfig
from utils import readProtoText


class Path(object):
    DIST_COLUMN = 'position'
    SEGMENT_COLUMN = 'segmentNumber'

    def __init__(self, track, car_data, parameter, subdivision, colormap, thickness):
        self.track = track
        self.car_data = car_data
        self.parameter = parameter
        self.subdivision = subdivision
        self.colormap = colormap
        self.thickness = thickness
        self.position = np.array((0, 0))
        self.direction = 0
        self.total_distance = 0
        self.color_scalarmappable = self.get_color_scalarmappable()

    def get_color_scalarmappable(self):
        col = self.car_data.loc[:, self.parameter]
        norm = mpl.colors.Normalize(vmin=np.min(col), vmax=np.max(col))
        sm = plt.cm.ScalarMappable(norm=norm, cmap=self.colormap)
        return sm

    def draw(self):
        for index, segment in enumerate(self.track.segment):
            self.sync_total_distance(index)

            if segment.type == SegmentType.STRAIGHT:
                self.draw_straight(segment)
            elif segment.type == SegmentType.CORNER:
                self.draw_arc(segment)

    # gets distance at start of segment number
    # and puts in total_distance
    def sync_total_distance(self, seg_index):
        if seg_index == 0:
            return 0

        lo = 0
        hi = self.car_data.shape[0] - 1
        while lo < hi - 1:
            mid = lo + (hi - lo) // 2
            if seg_index <= self.car_data.at[mid, Path.SEGMENT_COLUMN]:
                hi = mid
            else:
                lo = mid

        self.total_distance = self.car_data.at[hi, Path.DIST_COLUMN]

    # expects distance column to be non-decreasing
    # expects dist to be within lowest and highest distance in data
    # returns interpolation and stopping row
    def interpolate_parameter(self, dist, parameter):

        dist = max(
            self.car_data.at[0, Path.DIST_COLUMN],
            min(dist, self.car_data.iloc[-1][Path.DIST_COLUMN]))

        lo = 0
        hi = self.car_data.shape[0] - 1
        while lo < hi - 1:
            mid = lo + (hi - lo) // 2
            if dist < self.car_data.iloc[mid][Path.DIST_COLUMN]:
                hi = mid
            else:
                lo = mid

        rng = self.car_data.at[hi, Path.DIST_COLUMN] - self.car_data.at[lo, Path.DIST_COLUMN]
        diff = dist - self.car_data.at[lo, Path.DIST_COLUMN]
        t = diff / rng
        return (1 - t) * self.car_data.at[lo, parameter] \
            + t * self.car_data.at[hi, parameter]

    # by default translates in current direction
    def translate_position(self, length, direction=None):
        if direction is None:
            direction = self.direction

        return self.position + length * np.array(
            (np.cos(direction),
             np.sin(direction)))

    def draw_segment(self, p1, p2):
        parameter_val = self.interpolate_parameter(self.total_distance, self.parameter)
        color = self.map_color(parameter_val)
        plt.plot((p1[0], p2[0]), (p1[1], p2[1]), color=color, linewidth=self.thickness)

    # returns new position
    # this is super scuffed but better than copy and pasting within draw_track
    def draw_arc_step(self, cumulative_step_angle, last_pos, radius):

        disp_direction = self.direction + cumulative_step_angle / 2

        # law of cosines moment 😩
        disp_distance = radius * np.sqrt(2 - 2 * np.cos(cumulative_step_angle))

        next_pos = self.translate_position(disp_distance, disp_direction)

        self.draw_segment(last_pos, next_pos)

        return next_pos

    def draw_straight(self, segment):
        steps = int(segment.length / self.subdivision)
        last_pos = self.position
        for i in range(1, steps + 1):
            next_pos = self.translate_position(i * self.subdivision)
            self.draw_segment(last_pos, next_pos)
            last_pos = next_pos
            self.total_distance += self.subdivision

        remaining_dist = segment.length - steps * self.subdivision
        next_pos = self.translate_position(segment.length)
        self.draw_segment(last_pos, next_pos)
        self.total_distance += remaining_dist
        self.position = next_pos

    def draw_arc(self, segment):
        turn_angle = segment.angle
        arc_step = self.subdivision / segment.radius
        steps = int(turn_angle / arc_step)

        if segment.direction == CornerDirection.RIGHT:
            turn_angle *= -1
            arc_step *= -1

        last_pos = self.position
        for i in range(1, steps + 1):
            step_angle = i * arc_step
            last_pos = self.draw_arc_step(step_angle, last_pos, segment.radius)
            self.total_distance += self.subdivision

        remaining_angle = turn_angle - steps * arc_step
        arc_distance = remaining_angle * segment.radius

        self.position = self.draw_arc_step(turn_angle, last_pos, segment.radius)

        self.total_distance += arc_distance
        self.direction += turn_angle

    def map_color(self, val):
        return self.color_scalarmappable.to_rgba(val)

    def draw_colorbar(self):
        plt.colorbar(self.color_scalarmappable)


def proto_color_to_tuple(proto_color):
    return proto_color.r, proto_color.g, proto_color.b, proto_color.a


def generate_visualizations(car_data, track, vis_config, file_path):
    print("\nGenerating visualizations...")

    os.makedirs(file_path, exist_ok=True)
    for plot in vis_config.plot:
        path = Path(track, car_data, plot.parameter, plot.subdivision, plot.colormap, plot.thickness)

        plt.gca().axis("equal")
        plt.tick_params(
            which='both',
            bottom=False,
            top=False,
            left=False,
            right=False,
            labelbottom=False,
            labelleft=False)
        path.draw()
        path.draw_colorbar()
        plt.title(plot.name)
        plt.savefig(os.path.join(file_path, plot.name + ".png"), dpi=300)
        plt.clf()

    print("Done.")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "path",
        nargs="?",
        type=str,
        default=".",
        help="Output directory"
    )

    parser.add_argument(
        "--data",
        type=str,
        help="Vehicle data CSV"
    )

    parser.add_argument(
        "--track",
        type=str,
        help="Track config pbtxt",
    )

    parser.add_argument(
        "--vis_config",
        type=str,
        help="Visualization config pbtxt",
    )

    args = parser.parse_args()
    path = args.path
    car_data = pd.read_csv(args.data, index_col=False)
    track = readProtoText(args.track, Track)
    vis_config = readProtoText(args.vis_config, VisualizationConfig)

    generate_visualizations(car_data, track, vis_config, path)
