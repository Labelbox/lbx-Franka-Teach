#!/usr/bin/env python3
"""
Generate a self-contained FR3 URDF with embedded geometry for Foxglove visualization
"""

import os
from pathlib import Path

def generate_fr3_urdf_embedded():
    """Generate FR3 URDF with embedded simple geometry instead of mesh files"""
    
    urdf_content = """<?xml version="1.0" ?>
<robot name="fr3">
  <!-- FR3 Robot Model with Embedded Geometry for Foxglove -->
  <!-- This is a simplified version with primitive shapes instead of meshes -->
  
  <!-- Base link -->
  <link name="world"/>
  
  <link name="base"/>
  
  <joint name="world_to_base" type="fixed">
    <parent link="world"/>
    <child link="base"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
  
  <!-- Link 0 -->
  <link name="fr3_link0">
    <visual>
      <geometry>
        <cylinder length="0.15" radius="0.08"/>
      </geometry>
      <origin xyz="0 0 0.075" rpy="0 0 0"/>
      <material name="white">
        <color rgba="0.9 0.9 0.9 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.15" radius="0.08"/>
      </geometry>
      <origin xyz="0 0 0.075" rpy="0 0 0"/>
    </collision>
    <inertial>
      <origin xyz="-0.0172 0.0004 0.0745" rpy="0 0 0"/>
      <mass value="2.3966"/>
      <inertia ixx="0.009" ixy="0.0" ixz="0.002" iyy="0.0115" iyz="0.0" izz="0.0085"/>
    </inertial>
  </link>
  
  <joint name="fr3_base_joint" type="fixed">
    <parent link="base"/>
    <child link="fr3_link0"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
  
  <!-- Link 1 -->
  <link name="fr3_link1">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.07"/>
      </geometry>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.07"/>
      </geometry>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
    </collision>
    <inertial>
      <origin xyz="0.0000004128 -0.0181251324 -0.0386035970" rpy="0 0 0"/>
      <mass value="2.9274653454"/>
      <inertia ixx="0.023927316485107913" ixy="1.3317903455714081e-05" ixz="-0.00011404774918616684" iyy="0.0224821613275756" iyz="-0.0019950320628240115" izz="0.006350098258530016"/>
    </inertial>
  </link>
  
  <joint name="fr3_joint1" type="revolute">
    <origin xyz="0 0 0.333" rpy="0 0 0"/>
    <parent link="fr3_link0"/>
    <child link="fr3_link1"/>
    <axis xyz="0 0 1"/>
    <limit effort="87.0" lower="-2.7437" upper="2.7437" velocity="2.62"/>
    <safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-2.7437" soft_upper_limit="2.7437"/>
    <dynamics D="1" K="7000" damping="0.003" friction="0.2" mu_coulomb="0" mu_viscous="16"/>
  </joint>
  
  <!-- Link 2 -->
  <link name="fr3_link2">
    <visual>
      <geometry>
        <box size="0.12 0.08 0.2"/>
      </geometry>
      <origin xyz="0 -0.04 0" rpy="0 0 0"/>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.12 0.08 0.2"/>
      </geometry>
      <origin xyz="0 -0.04 0" rpy="0 0 0"/>
    </collision>
    <inertial>
      <origin xyz="0.0031828864 -0.0743221644 0.0088146084" rpy="0 0 0"/>
      <mass value="2.9355370338"/>
      <inertia ixx="0.041938946257609425" ixy="0.00020257331521090626" ixz="0.004077784227179924" iyy="0.02514514885014724" iyz="-0.0042252158006570156" izz="0.06170214472888839"/>
    </inertial>
  </link>
  
  <joint name="fr3_joint2" type="revolute">
    <origin xyz="0 0 0" rpy="-1.570796326794897 0 0"/>
    <parent link="fr3_link1"/>
    <child link="fr3_link2"/>
    <axis xyz="0 0 1"/>
    <limit effort="87.0" lower="-1.7837" upper="1.7837" velocity="2.62"/>
    <safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-1.7837" soft_upper_limit="1.7837"/>
    <dynamics D="1" K="7000" damping="0.003" friction="0.2" mu_coulomb="0" mu_viscous="16"/>
  </joint>
  
  <!-- Link 3 -->
  <link name="fr3_link3">
    <visual>
      <geometry>
        <cylinder length="0.15" radius="0.06"/>
      </geometry>
      <origin xyz="0.04 0 -0.03" rpy="0 1.5708 0"/>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.15" radius="0.06"/>
      </geometry>
      <origin xyz="0.04 0 -0.03" rpy="0 1.5708 0"/>
    </collision>
    <inertial>
      <origin xyz="0.0407015686 -0.0048200565 -0.0289730823" rpy="0 0 0"/>
      <mass value="2.2449013699"/>
      <inertia ixx="0.02410142547240885" ixy="0.002404694559042109" ixz="-0.002856269270114313" iyy="0.01974053266708178" iyz="-0.002104212683891874" izz="0.019044494482244823"/>
    </inertial>
  </link>
  
  <joint name="fr3_joint3" type="revolute">
    <origin xyz="0 -0.316 0" rpy="1.570796326794897 0 0"/>
    <parent link="fr3_link2"/>
    <child link="fr3_link3"/>
    <axis xyz="0 0 1"/>
    <limit effort="87.0" lower="-2.9007" upper="2.9007" velocity="2.62"/>
    <safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-2.9007" soft_upper_limit="2.9007"/>
    <dynamics D="1" K="7000" damping="0.003" friction="0.2" mu_coulomb="0" mu_viscous="16"/>
  </joint>
  
  <!-- Link 4 -->
  <link name="fr3_link4">
    <visual>
      <geometry>
        <box size="0.12 0.08 0.2"/>
      </geometry>
      <origin xyz="-0.04 0.04 0" rpy="0 0 0"/>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.12 0.08 0.2"/>
      </geometry>
      <origin xyz="-0.04 0.04 0" rpy="0 0 0"/>
    </collision>
    <inertial>
      <origin xyz="-0.0459100965 0.0630492960 -0.0085187868" rpy="0 0 0"/>
      <mass value="2.6155955791"/>
      <inertia ixx="0.03452998321913202" ixy="0.01322552265982813" ixz="0.01015142998484113" iyy="0.028881621933049058" iyz="-0.0009762833870704552" izz="0.04125471171146641"/>
    </inertial>
  </link>
  
  <joint name="fr3_joint4" type="revolute">
    <origin xyz="0.0825 0 0" rpy="1.570796326794897 0 0"/>
    <parent link="fr3_link3"/>
    <child link="fr3_link4"/>
    <axis xyz="0 0 1"/>
    <limit effort="87.0" lower="-3.0421" upper="-0.1518" velocity="2.62"/>
    <safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-3.0421" soft_upper_limit="-0.1518"/>
    <dynamics D="1" K="7000" damping="0.003" friction="0.2" mu_coulomb="0" mu_viscous="16"/>
  </joint>
  
  <!-- Link 5 -->
  <link name="fr3_link5">
    <visual>
      <geometry>
        <cylinder length="0.12" radius="0.06"/>
      </geometry>
      <origin xyz="0 0.03 -0.1" rpy="0 0 0"/>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.12" radius="0.06"/>
      </geometry>
      <origin xyz="0 0.03 -0.1" rpy="0 0 0"/>
    </collision>
    <inertial>
      <origin xyz="-0.0016039605 0.0292536262 -0.0972965990" rpy="0 0 0"/>
      <mass value="2.3271207594"/>
      <inertia ixx="0.051610278463662895" ixy="-0.005715173387783472" ixz="-0.0035673167625872135" iyy="0.04787729713371481" iyz="0.010673985108535986" izz="0.016423625579357254"/>
    </inertial>
  </link>
  
  <joint name="fr3_joint5" type="revolute">
    <origin xyz="-0.0825 0.384 0" rpy="-1.570796326794897 0 0"/>
    <parent link="fr3_link4"/>
    <child link="fr3_link5"/>
    <axis xyz="0 0 1"/>
    <limit effort="12.0" lower="-2.8065" upper="2.8065" velocity="5.26"/>
    <safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-2.8065" soft_upper_limit="2.8065"/>
    <dynamics D="1" K="7000" damping="0.003" friction="0.2" mu_coulomb="0" mu_viscous="16"/>
  </joint>
  
  <!-- Link 6 -->
  <link name="fr3_link6">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <origin xyz="0.06 -0.04 0" rpy="0 1.5708 0"/>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <origin xyz="0.06 -0.04 0" rpy="0 1.5708 0"/>
    </collision>
    <inertial>
      <origin xyz="0.0597131221 -0.0410294666 -0.0101692726" rpy="0 0 0"/>
      <mass value="1.8170376524"/>
      <inertia ixx="0.005412333594383447" ixy="0.006193456360285834" ixz="0.0014219289306117652" iyy="0.014058329545509979" iyz="-0.0013140753741120031" izz="0.016080817924212554"/>
    </inertial>
  </link>
  
  <joint name="fr3_joint6" type="revolute">
    <origin xyz="0 0 0" rpy="1.570796326794897 0 0"/>
    <parent link="fr3_link5"/>
    <child link="fr3_link6"/>
    <axis xyz="0 0 1"/>
    <limit effort="12.0" lower="0.5445" upper="4.5169" velocity="4.18"/>
    <safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="0.5445" soft_upper_limit="4.5169"/>
    <dynamics D="1" K="7000" damping="0.003" friction="0.2" mu_coulomb="0" mu_viscous="16"/>
  </joint>
  
  <!-- Link 7 -->
  <link name="fr3_link7">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.04"/>
      </geometry>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.04"/>
      </geometry>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
    </collision>
    <inertial>
      <origin xyz="0.0045225817 0.0086261921 -0.0161633251" rpy="0 0 0"/>
      <mass value="0.6271432862"/>
      <inertia ixx="0.00021092389150104718" ixy="-2.433299114461931e-05" ixz="4.564480393778983e-05" iyy="0.00017718568002411474" iyz="8.744070223226438e-05" izz="5.993190599659971e-05"/>
    </inertial>
  </link>
  
  <joint name="fr3_joint7" type="revolute">
    <origin xyz="0.088 0 0" rpy="1.570796326794897 0 0"/>
    <parent link="fr3_link6"/>
    <child link="fr3_link7"/>
    <axis xyz="0 0 1"/>
    <limit effort="12.0" lower="-3.0159" upper="3.0159" velocity="5.26"/>
    <safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-3.0159" soft_upper_limit="3.0159"/>
    <dynamics D="1" K="7000" damping="0.003" friction="0.2" mu_coulomb="0" mu_viscous="16"/>
  </joint>
  
  <!-- Link 8 (flange) -->
  <link name="fr3_link8"/>
  
  <joint name="fr3_joint8" type="fixed">
    <origin xyz="0 0 0.107" rpy="0 0 0"/>
    <parent link="fr3_link7"/>
    <child link="fr3_link8"/>
  </joint>
  
  <!-- Hand -->
  <link name="fr3_hand">
    <visual>
      <geometry>
        <box size="0.08 0.08 0.06"/>
      </geometry>
      <origin xyz="0 0 0.03" rpy="0 0 0"/>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.08 0.08 0.06"/>
      </geometry>
      <origin xyz="0 0 0.03" rpy="0 0 0"/>
    </collision>
    <inertial>
      <origin xyz="-0.0000376 0.0119128 0.0207260" rpy="0 0 0"/>
      <mass value="0.6544"/>
      <inertia ixx="0.00186" ixy="0.0" ixz="0.0" iyy="0.0003" iyz="-2e-05" izz="0.00174"/>
    </inertial>
  </link>
  
  <joint name="fr3_hand_joint" type="fixed">
    <parent link="fr3_link8"/>
    <child link="fr3_hand"/>
    <origin xyz="0 0 0" rpy="0 0 -0.7853981633974483"/>
  </joint>
  
  <!-- Hand TCP -->
  <link name="fr3_hand_tcp"/>
  
  <joint name="fr3_hand_tcp_joint" type="fixed">
    <origin xyz="0 0 0.1034" rpy="0 0 0"/>
    <parent link="fr3_hand"/>
    <child link="fr3_hand_tcp"/>
  </joint>
  
  <!-- Left finger -->
  <link name="fr3_leftfinger">
    <visual>
      <geometry>
        <box size="0.02 0.04 0.08"/>
      </geometry>
      <origin xyz="0 0.02 0.04" rpy="0 0 0"/>
      <material name="grey"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.02 0.04 0.08"/>
      </geometry>
      <origin xyz="0 0.02 0.04" rpy="0 0 0"/>
    </collision>
    <inertial>
      <origin xyz="0 0.0152850 0.0219675" rpy="0 0 0"/>
      <mass value="0.0291"/>
      <inertia ixx="8.49e-06" ixy="0.0" ixz="0.0" iyy="8.53e-06" iyz="-1.06e-06" izz="1.77e-06"/>
    </inertial>
  </link>
  
  <!-- Right finger -->
  <link name="fr3_rightfinger">
    <visual>
      <geometry>
        <box size="0.02 0.04 0.08"/>
      </geometry>
      <origin xyz="0 0.02 0.04" rpy="0 0 0"/>
      <material name="grey"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.02 0.04 0.08"/>
      </geometry>
      <origin xyz="0 0.02 0.04" rpy="0 0 0"/>
    </collision>
    <inertial>
      <origin xyz="0 0.0152850 0.0219675" rpy="0 0 0"/>
      <mass value="0.0291"/>
      <inertia ixx="8.49e-06" ixy="0.0" ixz="0.0" iyy="8.53e-06" iyz="-1.06e-06" izz="1.77e-06"/>
    </inertial>
  </link>
  
  <joint name="fr3_finger_joint1" type="prismatic">
    <parent link="fr3_hand"/>
    <child link="fr3_leftfinger"/>
    <origin xyz="0 0 0.0584" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="100" lower="0.0" upper="0.04" velocity="0.2"/>
    <dynamics damping="0.3"/>
  </joint>
  
  <joint name="fr3_finger_joint2" type="prismatic">
    <parent link="fr3_hand"/>
    <child link="fr3_rightfinger"/>
    <origin xyz="0 0 0.0584" rpy="0 0 3.141592653589793"/>
    <axis xyz="0 1 0"/>
    <limit effort="100" lower="0.0" upper="0.04" velocity="0.2"/>
    <mimic joint="fr3_finger_joint1"/>
    <dynamics damping="0.3"/>
  </joint>
</robot>
"""
    
    return urdf_content

def main():
    # Generate the embedded URDF
    urdf_content = generate_fr3_urdf_embedded()
    
    # Save to file
    output_path = Path("robot_urdf_models/fr3_embedded.urdf")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    with open(output_path, 'w') as f:
        f.write(urdf_content)
    
    print(f"Generated embedded FR3 URDF: {output_path}")
    print("This URDF uses primitive shapes instead of mesh files")
    print("It will work in Foxglove without needing external mesh files")

if __name__ == "__main__":
    main() 