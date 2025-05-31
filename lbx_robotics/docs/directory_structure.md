lbx-Franka-Teach/ (Git Repository Root)
├── lbx_robotics/ (ROS2 Workspace Root - This will be self-contained)
│ ├── src/
│ │ ├── lbx_interfaces/ (Custom ROS messages/services)
│ │ │ ├── package.xml
│ │ │ ├── CMakeLists.txt
│ │ │ └── msg/
│ │ │ ├── VRInput.msg
│ │ │ └── RobotCommand.msg
│ │ │
│ │ ├── lbx_utils/ (Common Python utilities, ROS-agnostic if possible)
│ │ │ ├── package.xml
│ │ │ ├── setup.py
│ │ │ └── lbx_utils/
│ │ │ ├── **init**.py
│ │ │ └── general_utils.py
│ │ │
│ │ ├── lbx_input_oculus/ (ROS Node for Oculus VR input)
│ │ │ ├── package.xml
│ │ │ ├── setup.py
│ │ │ ├── launch/
│ │ │ │ └── oculus_input.launch.py
│ │ │ └── lbx_input_oculus/
│ │ │ ├── **init**.py
│ │ │ └── oculus_node.py
│ │ │
│ │ ├── lbx_vision_realsense/ (ROS Node for Intel RealSense cameras)
│ │ │ ├── package.xml
│ │ │ ├── setup.py
│ │ │ ├── launch/
│ │ │ │ └── realsense.launch.py
│ │ │ ├── config/
│ │ │ │ └── realsense_cameras.yaml
│ │ │ └── lbx_vision_realsense/
│ │ │ ├── **init**.py
│ │ │ └── realsense_node.py
│ │ │
│ │ ├── lbx_franka_description/ (ROS Package for Franka URDFs and robot model)
│ │ │ ├── package.xml
│ │ │ ├── CMakeLists.txt
│ │ │ ├── launch/
│ │ │ │ └── robot_state_publisher.launch.py
│ │ │ ├── urdf/
│ │ │ └── meshes/
│ │ │
│ │ ├── lbx_franka_moveit/ (MoveIt2 configuration and server launch for FR3)
│ │ │ ├── package.xml
│ │ │ ├── CMakeLists.txt
│ │ │ ├── launch/
│ │ │ │ └── moveit_server.launch.py
│ │ │ └── config/
│ │ │
│ │ ├── lbx_franka_control/ (ROS Package for Franka teleoperation and MoveIt control)
│ │ │ ├── package.xml
│ │ │ ├── setup.py
│ │ │ ├── launch/
│ │ │ │ ├── franka_teleop.launch.py
│ │ │ │ └── franka_moveit_bringup.launch.py
│ │ │ ├── config/
│ │ │ │ └── teleop_config.yaml
│ │ │ └── lbx_franka_control/
│ │ │ ├── **init**.py
│ │ │ ├── teleop_node.py
│ │ │ ├── moveit_interface_node.py
│ │ │ └── system_health_node.py
│ │ │
│ │ └── lbx_data_recorder/ (ROS Package for MCAP recording)
│ │ ├── package.xml
│ │ ├── setup.py
│ │ ├── launch/
│ │ │ └── recorder.launch.py
│ │ └── lbx_data_recorder/
│ │ ├── **init**.py
│ │ ├── mcap_recorder_node.py
│ │ └── mcap_utils.py
│ │
│ ├── configs/ (Workspace-level configs, copied from root/configs if applicable)
│ │ ├── deoxys_params.yaml
│ │ └── general_teleop_settings.yaml
│ │
│ ├── robot_urdf_models/ (Copied from root/robot_urdf_models/)
│ │ ├── fr3.urdf.xacro
│ │ └── fr3_franka_hand_snug.urdf
│ │
│ ├── docs/ (Centralized documentation directory)
│ │ ├── README.md (Overall workspace: build, launch main components)
│ │ ├── directory_structure.md (This file)
│ │ ├── lbx_input_oculus.md (Package-specific docs)
│ │ ├── lbx_vision_realsense.md
│ │ ├── lbx_franka_control.md
│ │ ├── lbx_data_recorder.md
│ │ └── architecture.md (Overview of how nodes interact)
│ │
│ ├── requirements.txt (Consolidated Python dependencies for this workspace)
│ ├── Dockerfile (Optional)
│ ├── docker-compose.yml (Optional)
│ └── .dockerignore (Optional)
│
├── configs/ (Original root-level configs - WILL REMAIN)
├── franka-env/ (Original - WILL REMAIN)
├── frankateach/ (Original - WILL REMAIN, files copied from here)
├── oculus_reader_app/ (Original - WILL REMAIN)
├── robot_urdf_models/ (Original - WILL REMAIN, files copied from here)
├── ros2_moveit_franka/ (Original - WILL REMAIN, files copied from here)
└── ... (Other original root-level files - WILL REMAIN)
