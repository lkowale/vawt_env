cd image_transport_ws/src/
catkin_create_pkg dynamic_tutorials rospy roscpp dynamic_reconfigure

#Create a configuration file, saved it as your_package/cfg/MyStuff.cfg:
mkdir cfg



#create config file
#!/usr/bin/env python
PACKAGE = "dynamic_tutorials"

from dynamic_reconfigure.parameter_generator_catkin import *

gen = ParameterGenerator()

gen.add("int_param",    int_t,    0, "An Integer parameter", 50,  0, 100)
gen.add("double_param", double_t, 0, "A double parameter",    .5, 0,   1)
gen.add("str_param",    str_t,    0, "A string parameter",  "Hello World")
gen.add("bool_param",   bool_t,   0, "A Boolean parameter",  True)

size_enum = gen.enum([ gen.const("Small",      int_t, 0, "A small constant"),
                       gen.const("Medium",     int_t, 1, "A medium constant"),
                       gen.const("Large",      int_t, 2, "A large constant"),
                       gen.const("ExtraLarge", int_t, 3, "An extra large constant")],
                     "An enum to set size")

gen.add("size", int_t, 0, "A size parameter which is edited via an enum", 1, 0, 3, edit_method=size_enum)

exit(gen.generate(PACKAGE, "dynamic_tutorials", "Tutorials"))




chmod a+x cfg/Tutorials.cfg
#build
cd ~/image_transport_ws
catkin_make

# after build file is made to be imported
# under path
# /home/aa/image_transport_ws/devel/lib/python2.7/dist-packages/dynamic_tutorials/cfg/TutorialsConfig.py


from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'name': 'Default', 'type': '', 'state': True, 'cstate': 'true', 'id': 0, 'parent': 0, 'parameters':
    [
        {'name': 'int_param', 'type': 'int', 'default': 50, 'level': 0, 'description': 'An Integer parameter', 'min': 0, 'max': 100, 'srcline': 290, 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'int', 'cconsttype': 'const int'},
        {'name': 'double_param', 'type': 'double', 'default': 0.5, 'level': 0, 'description': 'A double parameter', 'min': 0.0, 'max': 1.0, 'srcline': 290, 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'},
        {'name': 'str_param', 'type': 'str', 'default': 'Hello World', 'level': 0, 'description': 'A string parameter', 'min': '', 'max': '', 'srcline': 290, 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'std::string', 'cconsttype': 'const char * const'},
        {'name': 'bool_param', 'type': 'bool', 'default': True, 'level': 0, 'description': 'A Boolean parameter', 'min': False, 'max': True, 'srcline': 290, 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'bool', 'cconsttype': 'const bool'},
        {'name': 'size', 'type': 'int', 'default': 1, 'level': 0, 'description': 'A size parameter which is edited via an enum', 'min': 0, 'max': 3, 'srcline': 290, 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': "{'enum': [{'name': 'Small', 'type': 'int', 'value': 0, 'srcline': 13, 'srcfile': '/home/aa/image_transport_ws/src/dynamic_tutorials/cfg/Tutorials.cfg', 'description': 'A small constant', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'Medium', 'type': 'int', 'value': 1, 'srcline': 14, 'srcfile': '/home/aa/image_transport_ws/src/dynamic_tutorials/cfg/Tutorials.cfg', 'description': 'A medium constant', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'Large', 'type': 'int', 'value': 2, 'srcline': 15, 'srcfile': '/home/aa/image_transport_ws/src/dynamic_tutorials/cfg/Tutorials.cfg', 'description': 'A large constant', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'ExtraLarge', 'type': 'int', 'value': 3, 'srcline': 16, 'srcfile': '/home/aa/image_transport_ws/src/dynamic_tutorials/cfg/Tutorials.cfg', 'description': 'An extra large constant', 'ctype': 'int', 'cconsttype': 'const int'}], 'enum_description': 'An enum to set size'}", 'ctype': 'int', 'cconsttype': 'const int'}
    ],
                      'groups': [], 'srcline': 245, 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'class': 'DEFAULT', 'parentclass': '', 'parentname': 'Default', 'field': 'default', 'upper': 'DEFAULT', 'lower': 'groups'}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']

Tutorials_Small = 0
Tutorials_Medium = 1
Tutorials_Large = 2
Tutorials_ExtraLarge = 3

# that after import and used as Server() paramter is to be used as dynamic configure like this

#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from cfg import TutorialsConfig


def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {int_param}, {double_param},\ 
          {str_param}, {bool_param}, {size}""".format(**config))
    return config


if __name__ == "__main__":
    rospy.init_node("dynamic_tutorials", anonymous=False)

    srv = Server(TutorialsConfig, callback)
    rospy.spin()