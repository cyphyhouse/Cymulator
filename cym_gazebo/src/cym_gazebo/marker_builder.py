import abc
from typing import Optional, Tuple

import rospy
from cym_marker.msg import Marker, Material, Script
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from std_msgs.msg import Header


# TODO move this file into cym_marker package
class MarkerBase(abc.ABC):
    def __init__(self):
        super().__init__()
        self.ns = ""
        self.id = None

    @abc.abstractmethod
    def build(self) -> Optional[Marker]:
        pass


class DeleteAll(MarkerBase):
    """
    Delete all markers in the given namespace ns.
    If ns is None or the empty string, delete all markers.
    """
    def __init__(self):
        super().__init__()

    def build(self) -> Optional[Marker]:
        return Marker(action=Marker.DELETE_ALL, ns=self.ns)


class DeleteMarker(MarkerBase):
    """
    Delete the marker with the given id in the given namespace ns
    """
    def __init__(self):
        super().__init__()

    def build(self) -> Optional[Marker]:
        if self.id is None:
            rospy.logwarn("Marker id is not set for deleting a marker. Skip.")
            return None
        return Marker(action=Marker.DELETE_MARKER, ns=self.ns, id=self.id)


class PutMarkerBase(MarkerBase):
    """
    Base class to build a marker message for adding or modifying the marker
    with the given id.
    """

    DEFAULT_POSE = Pose(position=Point(x=0, y=0, z=0),
                        orientation=Quaternion(x=0, y=0, z=0, w=1))
    DEFAULT_SCALE = Vector3(x=1, y=1, z=1)
    DEFAULT_MATERIAL = Material(script=Script(name="Gazebo/YellowTransparent"))

    def __init__(self):
        super().__init__()
        # Fields not exposed to user
        self._type = Marker.NONE
        self._layer = None  # Not supported yet
        self._lifetime = None  # Not supported yet

        self.pose = self.DEFAULT_POSE
        self.scale = self.DEFAULT_SCALE
        # TODO easier way to set material than constructing Material
        #  from scratch
        self.material = self.DEFAULT_MATERIAL
        self.parent = ""
        self.visible_to_sensors = False

    @abc.abstractmethod
    def build(self) -> Optional[Marker]:
        if self.id is None:
            rospy.logwarn("Marker id is not set for putting the marker. Skip.")
            return None
        if self._type == Marker.NONE:
            rospy.logwarn("Marker type is set to NONE for putting the marker. "
                          "Skip.")
            return None
        # TODO check if the quaternion is valid or convert to valid one
        if self.pose.orientation == Quaternion():
            rospy.logwarn("Invalid pose is given for putting a marker. Skip.")
            return None
        # TODO check if the scale makes sense
        if self.scale.x <= 0 or self.scale.y <= 0 or self.scale.z <= 0:
            rospy.logwarn("Invalid scale is given for putting a marker. Skip.")
            return None

        visibility = Marker.GUI if not self.visible_to_sensors else Marker.ALL

        kwargs = {
            "header": Header(frame_id="world"),
            "action": Marker.ADD_MODIFY,
            "type": self._type,
            "id": self.id,
            "ns": self.ns,
            "pose": self.pose,
            "scale": self.scale,
            "material": self.material,
            "parent": self.parent,
            "visibility": visibility
        }
        # Not all derived classes have following fields
        for attr_str in ["point", "text"]:
            value = getattr(self, attr_str, None)
            kwargs[attr_str] = value

        return Marker(**kwargs)

    def use_material(self, name: str):
        self.material.script.name = name


class PutBox(PutMarkerBase):
    def build(self) -> Optional[Marker]:
        self._type = Marker.BOX
        return super().build()

    def from_corners(self,
                     p0: Tuple[float, float, float],
                     p1: Tuple[float, float, float]):
        self.pose.position.x = (p0[0] + p1[0]) / 2
        self.pose.position.y = (p0[1] + p1[1]) / 2
        self.pose.position.z = (p0[2] + p1[2]) / 2

        self.scale.x = abs((p0[0] - p1[0])) / 2
        self.scale.y = abs((p0[1] - p1[1])) / 2
        self.scale.z = abs((p0[2] - p1[2])) / 2


class PutCylinder(PutMarkerBase):
    def build(self) -> Optional[Marker]:
        self._type = Marker.CYLINDER
        return super().build()


class PutSphere(PutMarkerBase):
    def build(self) -> Optional[Marker]:
        self._type = Marker.SPHERE
        return super().build()


class PutText(PutMarkerBase):
    def __init__(self):
        super().__init__()
        self.text = "Hello World!"

    def build(self) -> Optional[Marker]:
        if not self.text:
            rospy.logwarn("Empty string is given for putting text marker. "
                          "Skip.")
            return None

        self._type = Marker.TEXT
        return super().build()


class PutMarkerListBase(PutMarkerBase):
    def __init__(self):
        super().__init__()
        self.point = []


class PutLineList(PutMarkerListBase):
    def build(self):
        self._type = Marker.LINE_LIST
        return super().build()

    def add_hollow_box(self,
                       p0: Tuple[float, float, float],
                       p1: Tuple[float, float, float]):
        pair = (p0, p1)
        z_lines = (Point(x=pair[i][0], y=pair[j][1], z=pair[k][2])
                   for i in range(2) for j in range(2) for k in range(2))
        self.point.extend(z_lines)
        y_lines = (Point(x=pair[j][0], y=pair[k][1], z=pair[i][2])
                   for i in range(2) for j in range(2) for k in range(2))
        self.point.extend(y_lines)

        x_lines = (Point(x=pair[k][0], y=pair[i][1], z=pair[j][2])
                   for i in range(2) for j in range(2) for k in range(2))
        self.point.extend(x_lines)


class PutPoints(PutMarkerListBase):
    def build(self):
        self._type = Marker.POINTS
        return super().build()


# TODO Other shapes that use lists of points

# TODO Move below to test files
if __name__ == "__main__":
    builder = PutLineList()
    builder.id = 0
    builder.add_hollow_box((0, 0, 0), (1, 1, 1))
    print(builder.build())
