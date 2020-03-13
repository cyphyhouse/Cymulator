# Material shader type
uint8 VERTEX=1
uint8 PIXEL=2
uint8 NORMAL_MAP_OBJECT_SPACE=3
uint8 NORMAL_MAP_TANGENT_SPACE=4

Header header

Script script
uint8 shader_type
string normal_map
std_msgs/ColorRGBA ambient
std_msgs/ColorRGBA diffuse
std_msgs/ColorRGBA specular
std_msgs/ColorRGBA emissive
bool lighting

