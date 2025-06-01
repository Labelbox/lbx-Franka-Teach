# 🎮 VR CONNECTION GUIDE

This guide helps you connect your Meta Quest to the LBX Robotics system for VR teleoperation.

## ⚠️ VR Graceful Fallback Active

When you see "VR Graceful Fallback Active", it means:

- ✅ The robotics system is running normally
- ✅ All features except VR input are functional
- ✅ Recording, cameras, and robot control work
- ❌ VR controller input is not available

## 🔧 Quick Setup

### USB Connection (Recommended)

1. **Enable Developer Mode on Quest**

   - Open Meta Quest app on your phone
   - Go to Settings → Developer Mode → Enable

2. **Connect USB Cable**

   - Use the included USB-C cable
   - Connect Quest to computer
   - Put on headset when prompted for USB debugging

3. **Verify Connection**
   ```bash
   adb devices
   # Should show your Quest device
   ```

### Network Connection (Advanced)

1. **Enable ADB over Network**

   - In Quest: Settings → Developer → ADB over Network
   - Note the IP address shown

2. **Connect via Network**

   ```bash
   adb connect YOUR_QUEST_IP:5555
   ```

3. **Launch with Network Mode**
   ```bash
   ./unified_launch.sh --network-vr YOUR_QUEST_IP
   ```

## 🔍 Troubleshooting

### Common Issues

**"No ADB devices found"**

- Ensure Developer Mode is enabled
- Try a different USB cable
- Restart ADB: `adb kill-server && adb start-server`

**"Device unauthorized"**

- Put on Quest headset
- Look for USB debugging prompt
- Select "Allow" and check "Always allow"

**"pure-python-adb not installed"**

```bash
pip install pure-python-adb
```

**Connection drops during use**

- Check USB cable connection
- Try moving closer to router (network mode)
- Restart the Quest device

### Advanced Debugging

1. **Check ADB Status**

   ```bash
   adb devices -l
   # Shows detailed device info
   ```

2. **Test VR App Installation**

   ```bash
   adb shell pm list packages | grep teleop
   # Should show the VR teleoperation app
   ```

3. **Monitor VR Logs**
   ```bash
   adb logcat | grep wE9ryARX
   # Shows VR controller data stream
   ```

## 🎯 VR System Status

The system provides clear feedback about VR status:

- 🎮 **VR: Active** - Controller input detected and robot responding
- 🎮 **VR: Ready** - Connected but not actively controlling
- 🎮 **VR: Fallback** - Not connected, using graceful fallback

## 🔄 Hot-Plugging Support

The system supports hot-plugging VR:

- ✅ Connect VR while system is running
- ✅ Automatic detection and reconnection
- ✅ No restart required
- ✅ Seamless transition between modes

## 📋 System Requirements

- **Meta Quest 2/3/Pro** with Developer Mode
- **ADB (Android Debug Bridge)** - usually installed with Android SDK
- **USB-C cable** or **Network connection**
- **Python package**: `pure-python-adb`

## 🆘 Getting Help

If VR connection issues persist:

1. **Check System Logs**

   - Look for detailed error messages in oculus_node logs
   - Check the VR connection banner for specific instructions

2. **Alternative Control Methods**

   - Keyboard/mouse interfaces (if available)
   - Direct robot programming
   - Data recording without VR input

3. **Contact Support**
   - Provide ADB device output
   - Include system log files
   - Mention Quest model and software version

---

💡 **Pro Tip**: The robotics system works perfectly without VR connected. Use this mode for:

- System testing and validation
- Data recording setup
- Robot calibration and safety checks
- Development and debugging

The VR teleoperation is an enhancement - the core robotics functionality remains fully operational!
