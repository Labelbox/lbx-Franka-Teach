# Franka Hand Adjustment Tools

Quick reference for adjusting the Franka hand position and rotation.

## Current Configuration
- **Offset**: 110mm closer to arm (Z = -0.003m)
- **Rotation**: -45° (45 degrees CCW)

## How to Adjust Hand Position and Rotation

### Understanding the Values

#### Hand Offset (Position)
- **Parameter**: `--offset` 
- **Units**: Meters (negative values bring hand closer)
- **Default**: 0.0m (107mm gap between link7 and hand)
- **Typical Range**: -0.050m to -0.150m
- **Examples**:
  - `-0.050` = 50mm closer
  - `-0.105` = 105mm closer (current)
  - `-0.110` = 110mm closer
  - `-0.150` = 150mm closer (very snug)

#### Hand Rotation
- **Parameter**: `--rotation`
- **Units**: Degrees (from default -45° orientation)
- **Default**: 0° (which results in -45° total rotation)
- **Direction**: Negative = CCW, Positive = CW (looking down Z-axis)
- **Typical Range**: -90° to +90°
- **Examples**:
  - `0` = Default orientation (-45° total)
  - `45` = Straight/aligned (0° total) 
  - `-45` = 90° CCW from default (-90° total)
  - `90` = 45° CW from default (+45° total)

### Step-by-Step Adjustment Process

1. **Adjust the values in xacro**:
   ```bash
   # Example: Set hand 110mm closer with 45° CCW rotation
   ./apply_hand_offset.py --offset -0.110 --rotation 45
   ```

2. **Update the MCAP URDF**:
   ```bash
   ./patch_mcap_urdf.py
   cp franka_description/urdfs/fr3_franka_hand.urdf robot_urdf_models/fr3_franka_hand_snug.urdf
   ```

3. **Verify the changes**:
   ```bash
   ./debug_hand_rotation.py
   ```

4. **Test in visualization** (record new MCAP or restart RViz)

### Quick Rotation Testing

For rapid iteration on rotation only:
```bash
# Test different rotations directly in URDF
./test_hand_rotation.py -45    # 45° CCW
./test_hand_rotation.py 0      # No rotation  
./test_hand_rotation.py 45     # 45° CW
```

## Available Tools

### 1. `apply_hand_offset.py`
Main tool for adjusting hand offset and rotation in the xacro files.

```bash
# Apply specific offset and rotation
./apply_hand_offset.py --offset -0.110 --rotation 45

# Show current configuration
./apply_hand_offset.py --show

# Restore original configuration
./apply_hand_offset.py --restore
```

### 2. `patch_mcap_urdf.py`
Updates the URDF file used by MCAP recorder after xacro changes.

```bash
# Apply current xacro settings to MCAP URDF
./patch_mcap_urdf.py

# Then copy to robot models directory
cp franka_description/urdfs/fr3_franka_hand.urdf robot_urdf_models/fr3_franka_hand_snug.urdf
```

### 3. `test_hand_rotation.py`
Quick tool to test different rotation values directly in the URDF.

```bash
# Set specific rotation
./test_hand_rotation.py -45    # 45 degrees CCW
./test_hand_rotation.py 0      # No rotation
./test_hand_rotation.py 45     # 45 degrees CW
```

### 4. `debug_hand_rotation.py`
Check all rotation values in the current URDF.

```bash
./debug_hand_rotation.py
```

## Important Files

- **URDF for MCAP**: `robot_urdf_models/fr3_franka_hand_snug.urdf`
- **Xacro config**: `franka_description/end_effectors/franka_hand/franka_hand_arguments.xacro`
- **Config file**: `franka_hand_offset_config.yaml`

## Notes

- Rotation is applied at `joint8` (between link7 and link8)
- Negative rotation values = CCW, Positive = CW (when looking down Z-axis)
- Always record a new MCAP file after changes to see updates in Foxglove
- Restart RViz or reload robot model to see changes 