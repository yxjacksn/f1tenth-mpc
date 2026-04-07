"""
RViz / Foxglove MarkerArray publishers for MPC debugging.
"""

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import numpy as np


def make_predicted_trajectory_markers(pred_z: np.ndarray, frame_id: str = 'map',
                                       stamp=None) -> MarkerArray:
    """Green line strip showing MPC predicted path."""
    ma = MarkerArray()

    m = Marker()
    m.header.frame_id = frame_id
    if stamp:
        m.header.stamp = stamp
    m.ns = 'mpc_predicted'
    m.id = 0
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.scale.x = 0.08
    m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.9)

    for k in range(len(pred_z)):
        p = Point()
        p.x = float(pred_z[k, 0])
        p.y = float(pred_z[k, 1])
        p.z = 0.1
        m.points.append(p)

    ma.markers.append(m)
    return ma


def make_reference_markers(ref: np.ndarray, frame_id: str = 'map',
                           stamp=None) -> MarkerArray:
    """Blue line strip showing the reference the MPC is tracking."""
    ma = MarkerArray()

    m = Marker()
    m.header.frame_id = frame_id
    if stamp:
        m.header.stamp = stamp
    m.ns = 'mpc_reference'
    m.id = 0
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.scale.x = 0.06
    m.color = ColorRGBA(r=0.3, g=0.3, b=1.0, a=0.8)

    for k in range(len(ref)):
        p = Point()
        p.x = float(ref[k, 0])
        p.y = float(ref[k, 1])
        p.z = 0.05
        m.points.append(p)

    ma.markers.append(m)
    return ma


def make_waypoint_markers(waypoints: np.ndarray, frame_id: str = 'map',
                          stamp=None, step: int = 10) -> MarkerArray:
    """Red dots showing every `step`-th waypoint (full track)."""
    ma = MarkerArray()

    m = Marker()
    m.header.frame_id = frame_id
    if stamp:
        m.header.stamp = stamp
    m.ns = 'mpc_waypoints'
    m.id = 0
    m.type = Marker.POINTS
    m.action = Marker.ADD
    m.scale.x = 0.08
    m.scale.y = 0.08
    m.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.5)

    for k in range(0, len(waypoints), step):
        p = Point()
        p.x = float(waypoints[k, 0])
        p.y = float(waypoints[k, 1])
        p.z = 0.0
        m.points.append(p)

    ma.markers.append(m)
    return ma
