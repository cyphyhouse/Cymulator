# Marker type
uint8 NONE=0
uint8 BOX=1
uint8 CYLINDER=2
uint8 LINE_LIST=4
uint8 LINE_STRIP=3
uint8 POINTS=5
uint8 SPHERE=6
uint8 TEXT=7
uint8 TRIANGLE_FAN=8
uint8 TRIANGLE_LIST=9
uint8 TRIANGLE_STRIP=10

# Visilibity defines what cameras render the marker.
uint8 GUI=0  # Only cameras for user interaction render the visual.
uint8 ALL=1   # All cameras including camera sensors render the visual.

# Marker action
uint8 ADD_MODIFY=0
uint8 DELETE_MARKER=1
uint8 DELETE_ALL=2

Header header
uint8 action
string ns
uint64 id
int32 layer
uint8 type
duration lifetime
geometry_msgs/Pose pose
geometry_msgs/Vector3 scale
Material material

# Relevant Type: LINE_STRIP, LINE_LIST, POINTS, TRIANGLE_FAN, TRIANGLE_LIST,
#                TRIANGLE_STRIP
geometry_msgs/Point[] point

# Relevant Type: TEXT
string text

string parent

uint8 visibility

