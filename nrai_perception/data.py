from dataclasses import dataclass, field

@dataclass
class Header:
    stamp: float = 0.0 # Timestamp in seconds
    frame_id: str = ""

@dataclass
class Point:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

@dataclass
class Quaternion:
    x: float  = 0.0
    y: float = 0.0 
    z: float = 0.0
    w: float = 0.0

@dataclass
class Pose:
    position: Point = field(default_factory=Point)
    orientation: Quaternion = field(default_factory=Quaternion)

@dataclass
class PoseWithCovariance:
    pose: Pose = field(default_factory=Pose)
    covariance: list[float] = field(default_factory=lambda: [0.0] * 36)  # 6x6 covariance matrix flattened

@dataclass
class PoseWithCovarianceStamped:
    header: Header = field(default_factory=Header)
    pose: PoseWithCovariance = field(default_factory=PoseWithCovariance)
