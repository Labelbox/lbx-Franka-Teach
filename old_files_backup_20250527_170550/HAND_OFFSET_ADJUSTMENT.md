# Franka Hand Offset and Rotation Adjustment Guide

## Problem
The default URDF configuration may have a gap between the Franka hand and the FR3 arm receptacle, and the hand orientation may need adjustment. This can cause issues with:
- Visual appearance in simulations
- Potential mechanical stress in real hardware
- Inaccurate collision detection
- Incorrect grasping orientation

## Solution
Adjust the `xyz_ee` parameter to bring the hand closer to the arm flange and the `rpy_ee` parameter to rotate the hand.

## Current Configuration
- **Offset**: 105mm closer (Z = -0.105m)
- **Rotation**: 15° CCW from default orientation

## Methods to Adjust Hand Offset and Rotation

### Method 1: Using Launch File (Recommended for Testing)
```bash
roslaunch franka_description fr3_with_snug_hand.launch hand_z_offset:=-0.105
```

Adjust the `hand_z_offset` parameter:
- `0.0` - Default position (may have gap)
- `-0.050` - 50mm closer
- `-0.105` - 105mm closer (current setting)
- `-0.150` - 150mm closer

### Method 2: Apply Script (Recommended)
Use the provided script to apply both offset and rotation:

```bash
# Apply current configuration (105mm closer, 15° CCW)
./apply_hand_offset.py

# Apply custom values
./apply_hand_offset.py --offset -0.12 --rotation 20

# Show current configuration
./apply_hand_offset.py --show

# Restore original
./apply_hand_offset.py --restore
```

### Method 3: Direct URDF Parameter
When loading the URDF, specify both parameters:

```xml
<param name="robot_description" 
       command="$(find xacro)/xacro '$(find franka_description)/robots/fr3/fr3.urdf.xacro'
                hand:=true
                ee_id:=franka_hand
                xyz_ee:='0 0 -0.105'
                rpy_ee:='0 0 ${-pi/4 + 0.261799}'" />
```

### Method 4: Permanent Change
Modify the default values in `franka_description/end_effectors/franka_hand/franka_hand_arguments.xacro`:

```xml
<!-- Position offset between ee and parent frame -->
<xacro:arg name="xyz_ee" default="0 0 -0.105" />

<!-- Rotation offset between ee and parent frame -->
<xacro:arg name="rpy_ee" default="0 0 ${-pi/4 + 0.261799}" />
```

## Technical Details

- The FR3 arm's link8 (flange) is positioned 0.107m from link7
- The hand attaches to link8 with the `xyz_ee` offset
- Negative Z values bring the hand closer to the arm
- The hand's TCP (Tool Center Point) is 0.1034m from the hand base
- Default hand rotation is -45° (-π/4 radians)
- Additional rotation is added to the default

## Verification

After adjusting the offset and rotation:
1. Visualize in RViz to check appearance
2. Check for collision warnings if using self-collision checking
3. Verify that gripper operation is not affected
4. Test grasping orientation with real objects
5. Test with real hardware carefully before deployment

## MCAP Recording

The MCAP recorder uses a pre-patched URDF from `robot_urdf_models/fr3_franka_hand_snug.urdf` which includes:
- Joint8 Z position: 0.002m (105mm closer than default)
- Hand joint rotation: -30° (default -45° + 15° CCW)

To update MCAP recordings with new values:
```bash
./patch_mcap_urdf.py
cp franka_description/urdfs/fr3_franka_hand.urdf robot_urdf_models/fr3_franka_hand_snug.urdf
```

## Notes

- The optimal offset and rotation may vary slightly between simulation and real hardware
- Consider mechanical tolerances when setting the offset
- Test rotation changes with your specific end-effector tasks
- Document your chosen values for consistency across deployments 