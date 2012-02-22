echo "started robot motion"
rosrun pr2_impeded_joint_detector wiggle &
sleep 0.25
echo "starting recording"
rostopic echo -p --nostr -n 1000 /r_arm_controller/state  > controller_data_right.rtp &  
rostopic echo -p --nostr -n 1000 /l_arm_controller/state  > controller_data_left.rtp 
echo "done recording"
echo "auto-generating yaml file"
octave genyaml.m
echo "done generating. copying to ../launch"
mv arm_controller_error_limits.yaml ../launch/
echo "process complete"