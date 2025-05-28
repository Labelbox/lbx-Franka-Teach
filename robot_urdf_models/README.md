# Robot URDF Models

This directory contains modified URDF models used by the MCAP recording system.

## Files

### fr3_franka_hand_snug.urdf
- **Description**: Modified FR3 robot URDF with hand positioned closer and rotated
- **Modifications**: 
  - Joint8 Z position changed from 0.107m to -0.003m (110mm closer)
  - Hand joint rotation: 0° (default -45° + 45° adjustment)
  - This brings the Franka hand 110mm closer to the arm receptacle and rotates it 45° from default
- **Used by**: MCAP data recorder (`frankateach/mcap_data_recorder.py`)

## Current Configuration

- **Hand Offset**: 110mm closer to arm (Z = -0.003m)
- **Hand Rotation**: 45° from default orientation (total yaw = 0°)

## Usage

The MCAP recorder automatically loads the URDF from this directory when starting a recording:

```python
# In frankateach/mcap_data_recorder.py
urdf_path = Path(__file__).parent.parent / "robot_urdf_models" / "fr3_franka_hand_snug.urdf"
```

## Updating the URDF

To update the hand offset and rotation:

1. Modify the offset and rotation in `franka_description/end_effectors/franka_hand/franka_hand_arguments.xacro`
2. Run `./patch_mcap_urdf.py` to patch the generated URDF
3. Copy the updated URDF to this directory:
   ```bash
   cp franka_description/urdfs/fr3_franka_hand.urdf robot_urdf_models/fr3_franka_hand_snug.urdf
   ```

Or use the convenience script:
```bash
./apply_hand_offset.py --offset -0.110 --rotation 45
```

## Original URDF

The original unmodified URDF can be found at:
- `franka_description/urdfs/fr3_franka_hand.urdf.backup`

## Notes

- All MCAP recordings will use the URDF from this directory
- The URDF is embedded in each MCAP file for visualization in Foxglove Studio
- Mesh files are referenced from GitHub for web accessibility 