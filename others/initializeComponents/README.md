# RoboComp Component Launcher

A Python script to automatically start, monitor, and manage multiple RoboComp components with a real-time monitoring interface.

## Features

- 🚀 **Automatic startup** of multiple RoboComp components
- 🔍 **Real-time monitoring** with live CPU and memory usage
- 🤖 **Automatic Webots detection** and startup
- 🔧 **Automatic rcnode management** - starts if not already running
- 📊 **ICE connectivity checks** to verify components are alive
- 📝 **Error logging** to `output/` directory for debugging
- ⚡ **Clean shutdown** with graceful termination on Ctrl+C

## Requirements

- Python 3
- Conda environment named `robotica` (or modify the script)
- Required Python packages:
  - `psutil`
  - `rich`
  - `toml`
  - `zeroc-ice` (Ice for Python)

## Installation

```bash
pip install psutil rich toml zeroc-ice
```

## Configuration

Components are defined in a TOML file (default: `sub.toml`). Example structure:

```toml
[[components]]
name = "bridge"
cwd = "~/robocomp/components/webots-bridge"
cmd = "bin/Webots2Robocomp etc/config"
ice_name = "Webots2Robocomp:tcp -h localhost -p 10006"

[[components]]
name = "camera"
cwd = "~/robocomp/components/robocomp-robolab/components/hardware/camera/ricoh_omni"
cmd = "killall -9 RicohOmni; bin/RicohOmni etc/config_wb"
ice_name = "Camera360RGB:tcp -h localhost -p 10097"
```

### Configuration Fields

- **name**: Component identifier (shown in the monitor)
- **cwd**: Working directory where the component should run
- **cmd**: Command to execute (shell command)
- **ice_name**: ICE proxy string for connectivity checks (optional)


## Usage

### Basic Usage

```bash
python subcognitive.py sub.toml
```

### With Custom Config File

```bash
python subcognitive.py config.toml
```

### What Happens When You Run It

1. ✅ Checks if Webots is running (starts it if not)
2. ✅ Checks if rcnode is running (starts it if not)
3. ✅ Loads components from TOML file
4. ✅ Displays component table with configuration
5. ✅ Starts all components in sequence
6. ✅ Shows real-time monitoring dashboard with:
   - Component status (Running/Stopped/Alive)
   - Uptime
   - Memory usage
   - CPU usage with visual bars
   - ICE connectivity status

### Stopping the Script

Press **Ctrl+C** to gracefully shutdown all components. The script will:
1. Send termination signal to all processes
2. Wait up to 3 seconds for graceful shutdown
3. Force kill any processes that don't respond
4. Exit cleanly

## Monitoring Dashboard

The dashboard updates in real-time and shows:

| Column | Description |
|--------|-------------|
| **Name** | Component name from TOML file |
| **Status** | ✅ Alive (ICE responds), ❌ Stopped (process died), ⚠️ No ICE (running but no ICE check) |
| **Uptime** | How long the component has been running (HH:MM:SS) |
| **Memory** | RAM usage in MB |
| **CPU** | CPU percentage with visual bar (🟩 <50%, 🟨 50-80%, 🟥 >80%) |

## Debugging

### Output Directory

Error messages are logged to the `output/` directory:
- `<component_name>.err` - Contains stderr output for debugging

Standard output (stdout) is discarded to prevent massive log files.

### Common Issues

**Component shows "Stopped"**
- Check `output/<component>.err` for error messages
- Verify the working directory (`cwd`) exists
- Ensure the binary path in `cmd` is correct

**Component shows "Down" (red X)**
- ICE proxy is not responding
- Check if rcnode is running: `ps aux | grep rcnode`
- Verify ICE ports are not in use: `netstat -tlnp | grep <port>`

**"rcnode: not found" error**
- The script handles this automatically, but if you see it:
- Make sure rcnode.sh exists at `/home/cofer/robocomp/tools/rcnode/rcnode.sh`
- Or update the path in `subcognitive.py`

**Commands with `&&` fail**
- Use `;` instead of `&&` for commands like `killall` that may fail
- Example: `killall -9 MyComp; bin/MyComp etc/config`

## Advanced Usage

### Starting Without Webots

If you don't want automatic Webots startup, comment out the Webots check in `subcognitive.py`.

### Adding New Components

1. Add a new `[[components]]` section to your TOML file
2. Fill in name, cwd, cmd, and optionally ice_name
3. Restart the script

### Monitoring Only (No Auto-start)

The script automatically monitors Webots even if it was started externally. Just run the script after starting your components manually to monitor them.

## File Structure

```
initializeComponents/
├── subcognitive.py     # Main script
├── config.toml           # Component configuration
├── README.md          # This file
└── output/            # Error logs (auto-created)
    ├── bridge.err
    ├── camera.err
    └── ...
```

## Troubleshooting

### Script hangs on exit
- Should be fixed in the latest version
- Uses timeout-based termination with force kill

### Huge log files
- Fixed: stdout is now redirected to /dev/null
- Only stderr (errors) are saved to `output/` directory

### Conda environment issues
- Make sure you activate the `robotica` environment before running
- Or modify the shebang in the script to use the correct Python

## Tips

- Keep the terminal window open to monitor component status
- Check `output/*.err` files when components fail to start
- Use descriptive names in the TOML file for easy identification
- Components start in the order they appear in the TOML file

## License

Part of the RoboComp project.
