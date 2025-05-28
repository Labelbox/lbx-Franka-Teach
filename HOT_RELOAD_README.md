# Hot Reload Feature for Oculus VR Server

## Overview

The hot reload feature allows the Oculus VR Server to automatically restart when code changes are detected, making development and debugging much faster.

## Usage

### Method 1: Using run_server.sh (Recommended)
```bash
./run_server.sh --hot-reload
```

This will:
- Kill any existing servers
- Start the server with hot reload, performance mode, and data verification
- Automatically restart when Python files or configs change

### Method 2: Direct command
```bash
python oculus_vr_server.py --hot-reload --performance --verify-data
```

### Method 3: Hot reload with custom options
```bash
./run_server.sh --hot-reload --debug --left-controller
```

## How It Works

1. **File Watching**: The hot reload wrapper monitors:
   - `*.py` files (all Python code)
   - `configs/*.yaml` (configuration files)
   - `frankateach/*.py` (package files)
   - `simulation/*.py` (simulation files)

2. **Automatic Restart**: When changes are detected:
   - Current server process is gracefully stopped
   - Brief pause (0.5s) to ensure clean shutdown
   - Server restarts with the same arguments

3. **Ignored Files**: The following are ignored:
   - `__pycache__` directories
   - `.pyc`, `.pyo` files
   - Swap files (`.swp`, `.swo`, `~`)
   - Git directories
   - Log and output directories

## Features

- **Preserves Arguments**: All command-line arguments are passed to the restarted server
- **Graceful Shutdown**: Uses SIGTERM for clean shutdown, SIGKILL as fallback
- **Process Monitoring**: Automatically restarts if server crashes
- **Cooldown Period**: 1-second minimum between restarts to prevent rapid cycling

## Development Workflow

1. Start the server with hot reload:
   ```bash
   ./run_server.sh --hot-reload
   ```

2. Make changes to any Python file

3. Save the file - server automatically restarts

4. Test your changes immediately

## Troubleshooting

### Server not restarting?
- Check if the file type is monitored (Python or YAML)
- Ensure the file is not in an ignored directory
- Look for error messages in the console

### Too many restarts?
- The 1-second cooldown prevents rapid restarts
- Check if multiple files are being saved simultaneously

### Dependencies
Requires the `watchdog` library:
```bash
pip install watchdog
```

## Architecture

```
run_server.sh
    ↓
oculus_vr_server.py --hot-reload
    ↓
oculus_vr_server_hotreload.py (wrapper)
    ├── File watcher (watchdog)
    └── Server process (oculus_vr_server.py)
```

The hot reload wrapper runs as a parent process that:
- Monitors file system changes
- Manages the actual server as a subprocess
- Handles signals and cleanup

## Best Practices

1. **Use with Debug Mode**: Great for testing without robot
   ```bash
   ./run_server.sh --hot-reload --debug
   ```

2. **Disable for Production**: Don't use hot reload in production
   ```bash
   ./run_server.sh  # No --hot-reload
   ```

3. **Save Frequently**: The server only restarts on file save

4. **Check Console**: Always check for startup errors after reload 