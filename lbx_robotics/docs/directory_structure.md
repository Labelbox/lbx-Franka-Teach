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
│ │ │ └── lbx_vision_realsense/
│ │ │ ├── **init**.py
│ │ │ └── realsense_node.py
│ │ │
│ │ ├── lbx_franka_description/ (ROS Package for Franka URDFs and robot model)
│ │ │ ├── package.xml
│ │ │ ├── CMakeLists.txt
│ │ │ ├── launch/
│ │ │ │ └── robot_state_publisher.launch.py
│ │ │ ├── urdf/ # Custom URDFs/XACROs, potentially referencing official franka_description assets
│ │ │ └── meshes/ # Custom meshes
│ │ │
│ │ ├── lbx_franka_moveit/ (MoveIt2 server launch for FR3)
│ │ │ ├── package.xml
│ │ │ ├── CMakeLists.txt
│ │ │ ├── launch/
│ │ │ │ └── moveit_server.launch.py # Loads configs from lbx_robotics/configs/moveit
│ │ │ └── lbx_franka_moveit/ # Python module for any helper scripts/nodes
│ │ │ └── **init**.py
│ │ │
│ │ ├── lbx_franka_control/ (ROS Package for Franka teleoperation)
│ │ │ ├── package.xml
│ │ │ ├── setup.py
│ │ │ ├── launch/
│ │ │ │ └── franka_teleop.launch.py # Loads configs from lbx_robotics/configs/control
│ │ │ └── lbx_franka_control/
│ │ │ ├── **init**.py
│ │ │ ├── teleop_node.py
│ │ │ └── moveit_interface_node.py
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
│ ├── configs/ (Centralized workspace configurations)
│ │ ├── moveit/ # All MoveIt related .yaml, .srdf, .rviz files
│ │ │ └── fr3_moveit_config.rviz # Example
│ │ │ └── fr3.srdf # Example
│ │ │ └── kinematics.yaml # Example
│ │ │ └── joint_limits.yaml # Example
│ │ │ └── ompl_planning.yaml # Example
│ │ ├── sensors/
│ │ │ └── realsense_cameras.yaml # Example camera configuration
│ │ ├── control/
│ │ │ └── teleop_config.yaml # Example teleoperation settings
│ │ ├── urdf/ # Centralized URDF/XACRO if not part of lbx_franka_description
│ │ │ └── fr3_with_hand_camera.urdf.xacro # Example
│ │ └── workspace_settings.yaml # General workspace settings, e.g. global parameters
│ │
│ ├── robot_urdf_models/ (Original URDFs - for reference, active ones in lbx_franka_description or configs/urdf)
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
│ │ ├── lbx_franka_moveit.md
│ │ └── architecture.md (Overview of how nodes interact)
│ │
│ ├── requirements.txt (Consolidated Python dependencies for this workspace)
│ ├── setup_environment.sh (Script to setup conda, ROS, and franka_ros2)
│ ├── environment.yaml (Conda environment definition)
│ ├── Dockerfile (Optional)
│ ├── docker-compose.yml (Optional)
│ └── .dockerignore (Optional)
│
├── configs/ (Original root-level configs - RETAINED for reference or non-ROS parts)
├── franka-env/ (Original - RETAINED)
├── frankateach/ (Original - RETAINED)
├── oculus_reader_app/ (Original - RETAINED)
├── robot_urdf_models/ (Original - RETAINED for reference)
├── ros2_moveit_franka/ (Original - RETAINED for reference or until full migration)
└── ... (Other original root-level files - RETAINED)
