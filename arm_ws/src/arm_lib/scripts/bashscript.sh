rostopic pub --once /xyz_coordinates arm_lib/xyz_coordinates -- {3.45,0.42,0.375} 
rostopic pub --once /command_chatter arm_lib/cmd_message catch
rostopic pub --once /xyz_coordinates arm_lib/xyz_coordinates -- {3.45,-1.5,1.5}
rostopic pub --once /command_chatter arm_lib/cmd_message release
rostopic pub --once /xyz_coordinates arm_lib/xyz_coordinates -- {0,2.116,5.077}