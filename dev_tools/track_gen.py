import argparse
import numpy as np
import ezdxf

from proto.track_pb2 import Track, Segment, SegmentType, CornerDirection
from utils import writeProtoText


def main(args):
    track = Track()
    track.name = args.name

    doc = ezdxf.readfile(args.dxf)
    msp = doc.modelspace()

    tangent = 0
    position = np.array((0, 0))
    for i, e in enumerate(msp):
        segment = Segment()

        if e.dxftype() == "LINE":
            segment.type = SegmentType.STRAIGHT
            start = np.array((e.dxf.start.x, e.dxf.start.y))
            end = np.array((e.dxf.end.x, e.dxf.end.y))
            vec = end - start
            segment.length = np.linalg.norm(vec)

            position = end
            tangent = np.arctan2(vec[1], vec[0])

        elif e.dxftype() == "ARC":
            segment.type = SegmentType.CORNER
            segment.radius = e.dxf.radius

            start_angle = np.radians(e.dxf.start_angle)
            end_angle = np.radians(e.dxf.end_angle)
            angle_delta = end_angle - start_angle
            segment.angle = angle_delta

            center = np.array((e.dxf.center.x, e.dxf.center.y))
            start_point = center + e.dxf.radius * np.array(
                (np.cos(start_angle),
                 np.sin(start_angle)))
            end_point = center + e.dxf.radius * np.array(
                (np.cos(end_angle),
                 np.sin(end_angle)))

            if np.linalg.norm(start_point - position) \
                    > np.linalg.norm(end_point - position):
                new_position = start_point
            else:
                new_position = end_point

            tangent_vec = np.array(
                (np.cos(tangent),
                 np.sin(tangent)))
            cross = np.cross(tangent_vec, new_position - position)

            if cross > 0:
                segment.direction = CornerDirection.LEFT
                tangent += angle_delta
            else:
                segment.direction = CornerDirection.RIGHT
                tangent -= angle_delta

            position = new_position

        else:
            raise Exception("Invalid entity in DXF! Aborting...")

        track.segment.append(segment)

    writeProtoText(f"tracks/{args.name}.pbtxt", track)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--dxf",
        type=str,
        help="DXF file with track data"
    )

    parser.add_argument(
        "--name",
        type=str,
        help="Track name"
    )

    args = parser.parse_args()

    main(args)
