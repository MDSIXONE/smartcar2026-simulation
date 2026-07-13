cmake_minimum_required(VERSION 2.8.3)

project(car3)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

catkin_install_python(PROGRAMS
  scripts/gripper_mimic.py
  scripts/grasp_attach.py
  scripts/arm_pose_tuner.py
  scripts/keyboard_drive.py
  scripts/projected_cube_grasp_test.py
  scripts/set_arm_initial_pose.py
  scripts/task3_pick_deliver.py
  scripts/spawn_cubes.py
  scripts/odom_tf_broadcaster.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
