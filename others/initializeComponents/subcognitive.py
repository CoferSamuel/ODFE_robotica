#!/usr/bin/env python3

import subprocess
import time
import os
import shutil
from pathlib import Path
import psutil
from rich.console import Console
from rich.table import Table
from rich.live import Live
from rich import box
import toml
import argparse
import threading
import pprint
import sys
import tty
import termios
from concurrent.futures import ThreadPoolExecutor, as_completed

# ── Optional Ice (ZeroC) import ───────────────────────────────────────────────
# If Ice is not installed the script still works; ICE status will show ⚠️ Running
# instead of ✅ Alive, but all process management continues normally.
try:
    import Ice
    _ICE_AVAILABLE = True
except ImportError:
    _ICE_AVAILABLE = False

parser = argparse.ArgumentParser(description="RoboComp subcognitive monitor")
parser.add_argument("file_name", type=str, default="sub.toml", help="Path to TOML components file")
args = parser.parse_args()
console = Console()

# ── Portable path resolution ──────────────────────────────────────────────────
# Read optional overrides from config early (before components are loaded).
def _early_config() -> dict:
    try:
        return toml.load(os.path.expanduser(args.file_name))
    except Exception:
        return {}

_cfg_early = _early_config()

def _find_webots() -> str:
    """Resolve Webots binary: config override > which webots > /usr/local/bin/webots."""
    from_cfg = _cfg_early.get("webots_bin", "").strip()
    if from_cfg:
        return os.path.expanduser(from_cfg)
    found = shutil.which("webots")
    if found:
        return found
    return "/usr/local/bin/webots"

def _find_rcnode() -> list:
    """Resolve rcnode command: config override > robocomp_root default > which rcnode."""
    from_cfg = _cfg_early.get("rcnode_script", "").strip()
    if from_cfg:
        return ["bash", os.path.expanduser(from_cfg)]
    robocomp_root = os.path.expanduser(_cfg_early.get("robocomp_root", "~/robocomp"))
    default_script = os.path.join(robocomp_root, "tools", "rcnode", "rcnode.sh")
    if os.path.exists(default_script):
        return ["bash", default_script]
    found = shutil.which("rcnode")
    if found:
        return [found]
    return ["bash", default_script]  # best guess even if not yet present

WEBOTS_BIN = _find_webots()
RCNODE_CMD = _find_rcnode()

# ── Freeze-prevention tuning ──────────────────────────────────────────────────
# Max system-wide CPU % before we'll launch the next component.
# If the system is above this, we wait (up to CPU_HEADROOM_TIMEOUT seconds).
CPU_HEADROOM_THRESHOLD = 60.0   # %
CPU_HEADROOM_TIMEOUT   = 90     # seconds

# If a single component stays above this CPU % for WATCHDOG_STRIKES consecutive
# 5-second samples, it gets auto-paused (user can resume with its number key).
WATCHDOG_CPU_THRESHOLD = 85.0   # %
WATCHDOG_STRIKES       = 2      # samples before auto-pause
# ─────────────────────────────────────────────────────────────────────────────

# Check if Webots is running, start it if not
def check_and_start_webots():
    webots_running = False
    webots_proc = None
    
    # First check if Webots is already running
    for proc in psutil.process_iter(['name', 'pid']):
        try:
            if 'webots' in proc.info['name'].lower():
                webots_running = True
                webots_proc = proc
                break
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass
    
    if not webots_running:
        console.print("[yellow]Webots not detected. Starting Webots...[/yellow]")
        try:
            # Start Webots detached (won't block the script)
            proc = subprocess.Popen(
                [WEBOTS_BIN],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True  # Detach from parent process
            )
            console.print("[green]✓ Webots started successfully[/green]")
            time.sleep(2)  # Give Webots a moment to initialize
            
            # Find the actual Webots process
            for p in psutil.process_iter(['name', 'pid']):
                try:
                    if 'webots' in p.info['name'].lower():
                        webots_proc = p
                        break
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass
        except Exception as e:
            console.print(f"[red]Failed to start Webots: {e}[/red]")
            return None
    else:
        console.print("[green]✓ Webots is already running[/green]")
    
    return webots_proc

webots_process = check_and_start_webots()

# Check if rcnode is running, start it if not
def check_and_start_rcnode():
    rcnode_running = False
    rcnode_proc = None
    
    # Check if rcnode (icebox) is already running
    for proc in psutil.process_iter(['name', 'cmdline', 'pid']):
        try:
            if 'icebox' in proc.info['name'].lower() and proc.info['cmdline']:
                # Check if it's the rcnode icebox process
                cmdline_str = ' '.join(proc.info['cmdline'])
                if 'rcnode' in cmdline_str.lower():
                    rcnode_running = True
                    rcnode_proc = proc
                    break
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass
    
    if not rcnode_running:
        console.print("[yellow]rcnode not detected. Starting rcnode...[/yellow]")
        try:
            # Start rcnode detached
            proc = subprocess.Popen(
                RCNODE_CMD,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True
            )
            console.print("[green]✓ rcnode started successfully[/green]")
            time.sleep(2)  # Give rcnode time to initialize
            
            # Find the actual icebox process
            for p in psutil.process_iter(['name', 'cmdline', 'pid']):
                try:
                    if 'icebox' in p.info['name'].lower() and p.info['cmdline']:
                        cmdline_str = ' '.join(p.info['cmdline'])
                        if 'rcnode' in cmdline_str.lower():
                            rcnode_proc = p
                            break
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass
        except Exception as e:
            console.print(f"[red]Failed to start rcnode: {e}[/red]")
            return None
    else:
        console.print("[green]✓ rcnode is already running[/green]")
    
    return rcnode_proc

rcnode_process = check_and_start_rcnode()

def cpu_usage_bar(cpu_percent, width=10):
    """Return a colored CPU usage bar."""
    filled = int((cpu_percent / 100) * width)
    empty = width - filled
    if cpu_percent < 50:
        color = "green"
    elif cpu_percent < 80:
        color = "yellow"
    else:
        color = "red"
    bar = f"[{color}]" + ("█" * filled + "░" * empty) + "[/]"
    return bar

# Load components and global settings from TOML file
def toml_loader():
    config_path = os.path.expanduser(args.file_name)
    if not os.path.exists(config_path):
        console.print(f"[red]Missing file with components at {config_path}[/red]")
        exit(1)

    config = toml.load(args.file_name)
    components = config["components"]
    startup_delay = config.get("startup_delay", 0)  # seconds between component launches

    # Expand ${robocomp_root} in all cwd/cmd values so the config is portable
    robocomp_root = os.path.expanduser(config.get("robocomp_root", "~/robocomp"))
    for comp in components:
        for key in ("cwd", "cmd"):
            if comp.get(key):
                comp[key] = comp[key].replace("${robocomp_root}", robocomp_root)

    return components, startup_delay

# Single persistent ICE communicator — created once, reused for all pings
if _ICE_AVAILABLE:
    _ice_communicator = Ice.initialize()
else:
    _ice_communicator = None
    console.print("[yellow]⚠ ZeroC Ice not found — ICE status checks disabled (install python3-zeroc-ice to enable)[/yellow]")

def ping_proxy(ice_string, timeout_ms=1000):
    """Ping an ICE proxy using the shared communicator. Returns True if alive."""
    if not _ICE_AVAILABLE or _ice_communicator is None:
        return False
    try:
        proxy = _ice_communicator.stringToProxy(ice_string)
        proxy = proxy.ice_timeout(timeout_ms)
        proxy.ice_ping()
        return True
    except Exception:
        return False

# Thread pool for parallel background ICE pings (one thread per component)
_ping_executor = ThreadPoolExecutor(max_workers=8, thread_name_prefix="ice_ping")

def green(text):
    return f"\033[92m{text}\033[0m"

def red(text):
    return f"\033[91m{text}\033[0m"

def format_uptime(seconds):
    hrs, rem = divmod(int(seconds), 3600)
    mins, secs = divmod(rem, 60)
    return f"{hrs:02}:{mins:02}:{secs:02}"

processes = {}

# Add Webots to monitoring if it's running
if webots_process:
    try:
        webots_process.cpu_percent(interval=None)  # Initialize CPU tracking
        processes["Webots"] = {
            "process": None,  # We don't have a subprocess.Popen object for Webots
            "psutil_proc": webots_process,
            "ice_name": None,
            "start_time": time.time(),
            "is_webots": True,
            "paused": False,
            "mem_last": 0.0,
            "cpu_last": 0.0,
        }
    except Exception:
        pass

# Add rcnode to monitoring if it's running
if rcnode_process:
    try:
        rcnode_process.cpu_percent(interval=None)  # Initialize CPU tracking
        processes["rcnode"] = {
            "process": None,  # We don't have a subprocess.Popen object for rcnode
            "psutil_proc": rcnode_process,
            "ice_name": None,
            "start_time": time.time(),
            "is_rcnode": True,
            "paused": False,
            "mem_last": 0.0,
            "cpu_last": 0.0,
        }
    except Exception:
        pass

def expand_path(p):
    return os.path.expanduser(p) if p else None

def monitor_and_rotate_log(pipe, log_path, max_size=10 * 1024 * 1024):
    """
    Reads from the pipe and writes to log_path.
    Rotates the log file when it exceeds max_size.
    Keeps one backup file (.old).
    """
    try:
        current_file = open(log_path, 'wb')
        while True:
            # Read in chunks to handle both binary data and massive lines safely
            data = pipe.read(4096)
            if not data:
                break
            
            current_file.write(data)

            current_file.flush()
            
            # Check size
            if current_file.tell() > max_size:
                current_file.close()
                timestamp = int(time.time())
                backup_name = f"{log_path}.old"
                
                # Windows support (rewrite implied by simple rename on Unix)
                if os.path.exists(backup_name):
                    try:
                        os.remove(backup_name)
                    except OSError:
                        pass
                        
                try:
                    os.rename(log_path, backup_name)
                except OSError:
                    pass
                
                current_file = open(log_path, 'wb')
                
        current_file.close()
    except Exception as e:
        # Safely ignore errors during shutdown or file ops to prevent crashing the monitor
        pass


# Start all components
def launch_process(command, cwd=None, name=None, nice_level=None):
    # Create output directory in the project folder
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.join(script_dir, "output")
    os.makedirs(output_dir, exist_ok=True)

    stderr_path = os.path.join(output_dir, f"{name}.err") if name else os.devnull

    # Replace 'rcnode' alias with the resolved portable script path
    if 'rcnode' in command and not command.startswith('bash /'):
        command = command.replace('rcnode', ' '.join(RCNODE_CMD))

    # Prepend nice if requested — lowers OS scheduling priority so desktop stays responsive
    if nice_level is not None:
        command = f"nice -n {nice_level} {command}"

    proc = subprocess.Popen(
        command,
        cwd=cwd,
        shell=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,  # Capture stderr
        env=os.environ.copy(),
        executable='/bin/bash'
    )

    # Start the log rotator thread
    if name:
        t = threading.Thread(target=monitor_and_rotate_log, args=(proc.stderr, stderr_path), daemon=True)
        t.start()

    time.sleep(0.3)  # brief wait for child process to appear
    try:
        children = psutil.Process(proc.pid).children()
        ps_proc = children[0] if children else psutil.Process(proc.pid)
    except Exception:
        ps_proc = psutil.Process(proc.pid)
    return proc, ps_proc

components, startup_delay = toml_loader()

def wait_for_cpu_headroom(max_cpu=CPU_HEADROOM_THRESHOLD, timeout=CPU_HEADROOM_TIMEOUT):
    """Block until system-wide CPU drops below max_cpu%, or timeout expires."""
    deadline = time.time() + timeout
    warned = False
    while time.time() < deadline:
        usage = psutil.cpu_percent(interval=1.0)  # 1-second blocking sample
        if usage < max_cpu:
            if warned:
                console.print(f"[green]✓ CPU at {usage:.0f}% — proceeding[/green]")
            return
        warned = True
        remaining = int(deadline - time.time())
        console.print(
            f"[yellow]⏳ System CPU at {usage:.0f}% (>{max_cpu:.0f}%) — "
            f"waiting for headroom... ({remaining}s left)[/yellow]"
        )
    console.print("[red]⚠ CPU headroom timeout reached — launching next component anyway[/red]")

def print_components_table(components):
    table = Table(title="🧠 Loaded Components", box=box.SIMPLE_HEAVY)
    table.add_column("Name", style="bold cyan")
    table.add_column("CWD", style="dim")
    table.add_column("Command", style="magenta")
    table.add_column("Nice", justify="right", style="dim")

    for comp in components:
        nice = comp.get("nice")
        table.add_row(
            comp["name"],
            comp.get("cwd", "-"),
            comp.get("cmd", "-"),
            str(nice) if nice is not None else "-"
        )

    console.print(table)

print_components_table(components)

if startup_delay > 0:
    console.print(f"[dim]Startup delay: {startup_delay}s between components[/dim]")

for i, comp in enumerate(components):
    # Wait for CPU headroom before every component (skip only the very first)
    if i > 0:
        wait_for_cpu_headroom()
        if startup_delay > 0:
            time.sleep(startup_delay)

    cwd = expand_path(comp.get("cwd"))
    command = comp["cmd"]
    nice_level = comp.get("nice")  # optional per-component nice level
    console.print(f"Starting [cyan]{comp['name']}[/cyan]...")
    proc, ps_proc = launch_process(command, cwd=cwd, name=comp["name"], nice_level=nice_level)
    ps_proc.cpu_percent(interval=None)  # initialise CPU tracking
    processes[comp["name"]] = {
        "process": proc,
        "psutil_proc": ps_proc,
        "ice_name": comp.get("ice_name"),
        "start_time": time.time(),
        "ice_status": "[yellow]⏳ Starting...[/yellow]",
        # Keep original config so we can restart after pause
        "_cwd": cwd,
        "_cmd": command,
        "_nice": nice_level,
        "paused": False,
    }

# Ordered list of component names the user can toggle (includes Webots and rcnode)
_toggleable = []
if webots_process:
    _toggleable.append("Webots")
if rcnode_process:
    _toggleable.append("rcnode")
_toggleable += [comp["name"] for comp in components]

# ── Watchdog: auto-pause runaway components ───────────────────────────────────
_high_cpu_counts: dict[str, int] = {}  # name -> consecutive high-CPU sample count

def watchdog_loop():
    """Every 5s, check for components hogging CPU. Auto-pause after WATCHDOG_STRIKES hits."""
    while True:
        time.sleep(5)
        for name, info in list(processes.items()):
            # Skip paused or system processes
            if info.get("paused") or info.get("is_webots") or info.get("is_rcnode"):
                _high_cpu_counts.pop(name, None)
                continue
            cpu = info.get("cpu_last", 0.0)
            # Use per-component max_cpu if configured (supports multi-threaded/OpenMP
            # components that legitimately exceed 100% by using multiple cores).
            comp_cfg = next((c for c in components if c["name"] == name), {})
            threshold = comp_cfg.get("max_cpu", WATCHDOG_CPU_THRESHOLD)
            if cpu > threshold:
                count = _high_cpu_counts.get(name, 0) + 1
                _high_cpu_counts[name] = count
                console.print(
                    f"[yellow]⚠ {name} at {cpu:.0f}% CPU "
                    f"({count}/{WATCHDOG_STRIKES} strikes)[/yellow]"
                )
                if count >= WATCHDOG_STRIKES:
                    console.print(
                        f"[red]🚨 {name} exceeded {threshold:.0f}% CPU "
                        f"for {WATCHDOG_STRIKES} samples — auto-pausing! "
                        f"(press its key to resume)[/red]"
                    )
                    if name in _toggleable:
                        toggle_component(_toggleable.index(name))
                    _high_cpu_counts[name] = 0
            else:
                _high_cpu_counts.pop(name, None)
# ─────────────────────────────────────────────────────────────────────────────

def toggle_component(index):
    """Stop or restart the component at position `index` (0-based) in _toggleable."""
    if index >= len(_toggleable):
        return
    name = _toggleable[index]
    info = processes.get(name)
    if info is None:
        return

    is_system = info.get("is_webots", False) or info.get("is_rcnode", False)

    if info["paused"]:
        # --- Resume: relaunch the process ---
        if is_system:
            console.print(f"[green]Starting {name}...[/green]")
            if info.get("is_webots"):
                proc = subprocess.Popen(
                    [WEBOTS_BIN],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    start_new_session=True
                )
                time.sleep(2)
                ps_proc = None
                for p in psutil.process_iter(['name', 'pid']):
                    try:
                        if 'webots' in p.info['name'].lower():
                            ps_proc = p
                            break
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        pass
                if ps_proc is None:
                    ps_proc = psutil.Process(proc.pid)
            else:  # rcnode
                proc = subprocess.Popen(
                    RCNODE_CMD,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    start_new_session=True
                )
                time.sleep(2)
                ps_proc = None
                for p in psutil.process_iter(['name', 'cmdline', 'pid']):
                    try:
                        if 'icebox' in p.info['name'].lower() and p.info['cmdline']:
                            if 'rcnode' in ' '.join(p.info['cmdline']).lower():
                                ps_proc = p
                                break
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        pass
                if ps_proc is None:
                    ps_proc = psutil.Process(proc.pid)
            ps_proc.cpu_percent(interval=None)
            info["psutil_proc"] = ps_proc
            info["start_time"] = time.time()
            info["mem_last"] = 0.0
            info["cpu_last"] = 0.0
            info["paused"] = False
        else:
            console.print(f"[green]Resuming {name}...[/green]")
            proc, ps_proc = launch_process(
                info["_cmd"], cwd=info["_cwd"], name=name, nice_level=info["_nice"]
            )
            ps_proc.cpu_percent(interval=None)
            info["process"] = proc
            info["psutil_proc"] = ps_proc
            info["start_time"] = time.time()
            info["ice_status"] = "[yellow]⏳ Starting...[/yellow]"
            info["mem_last"] = 0.0
            info["cpu_last"] = 0.0
            info["paused"] = False
    else:
        # --- Pause/Stop: terminate the process ---
        if is_system:
            console.print(f"[yellow]Stopping {name}...[/yellow]")
            ps_proc = info["psutil_proc"]
            try:
                ps_proc.terminate()
                ps_proc.wait(timeout=3)
            except psutil.TimeoutExpired:
                try:
                    ps_proc.kill()
                except Exception:
                    pass
            except Exception:
                pass
            info["mem_last"] = 0.0
            info["cpu_last"] = 0.0
            info["paused"] = True
        else:
            console.print(f"[yellow]Pausing {name}...[/yellow]")
            proc = info["process"]
            try:
                proc.terminate()
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=1)
            except Exception:
                pass
            info["ice_status"] = "[dim]⏸ Paused[/dim]"
            info["mem_last"] = 0.0
            info["cpu_last"] = 0.0
            info["paused"] = True

def read_keys():
    """Background thread: reads single keypresses and toggles components."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)  # read one char at a time, no echo
        while True:
            ch = sys.stdin.read(1)
            if ch.isdigit():
                idx = int(ch) - 1  # '1' -> index 0
                if 0 <= idx < len(_toggleable):
                    toggle_component(idx)
    except Exception:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

#Monitor loop
def build_table():
    table = Table(title="🧠 RoboComp Component Monitor", box=box.SIMPLE_HEAVY)
    table.add_column("Key", justify="center", style="dim", no_wrap=True)
    table.add_column("Name", style="bold cyan")
    table.add_column("Status", style="bold")
    table.add_column("Uptime", justify="right")
    table.add_column("Memory", justify="right")
    table.add_column("CPU", justify="right")

    toggle_idx = 1  # key number shown to the user (1-based)
    for name, info in processes.items():
        ice_name = info["ice_name"]
        status = "[yellow]⏳ Checking...[/yellow]"

        # Assign key hint only to toggleable components
        is_toggleable = name in _toggleable
        key_label = f"[bold yellow][{toggle_idx}][/bold yellow]" if is_toggleable else "[dim] - [/dim]"
        if is_toggleable:
            toggle_idx += 1

        # Special handling for Webots and rcnode
        if info.get("is_webots", False) or info.get("is_rcnode", False):
            if info.get("paused", False):
                status = "[dim]⏸  Stopped[/dim]"
            else:
                try:
                    if info["psutil_proc"].is_running():
                        status = "[green]✅ Running[/green]"
                    else:
                        status = "[red]❌ Stopped[/red]"
                except Exception:
                    status = "[red]❌ Stopped[/red]"
        elif info.get("paused", False):
            status = "[dim]⏸  Paused[/dim]"
        else:
            # Regular component handling
            proc = info["process"]
            running = proc.poll() is None

            if running:
                # Use the pre-computed ICE status from the background thread
                status = info.get("ice_status", "[yellow]⏳ Starting...[/yellow]")
            else:
                status = "[red]❌ Stopped[/red]"

        # Uptime
        uptime = time.time() - info["start_time"]
        uptime_str = format_uptime(uptime)

        # Resource usage
        try:
            mem = info.get("mem_last", 0.0)
            cpu = info.get("cpu_last", 0.0)
        except Exception:
            mem, cpu = 0, 0

        table.add_row(
            key_label,
            name,
            status,
            uptime_str,
            f"{mem:6.1f} MB",
            f"{cpu:5.1f}% {cpu_usage_bar(cpu)}"
        )

    # Footer hint
    hints = "  ".join(
        f"[bold yellow][{i+1}][/bold yellow] {n}"
        for i, n in enumerate(_toggleable)
    )
    table.caption = f"[dim]Toggle: {hints}   │   Ctrl+C to quit[/dim]"

    return table

def _ping_one(name, info):
    """Ping a single component's ICE endpoint and update its status. Runs in thread pool."""
    if info.get("paused", False):
        return  # don't ping paused components
    ice_name = info.get("ice_name")
    if ice_name and info.get("process") and info["process"].poll() is None:
        if ping_proxy(ice_name, timeout_ms=1000):
            info["ice_status"] = "[green]✅ Alive[/green]"
        else:
            info["ice_status"] = "[yellow]⚠️  Running[/yellow]"
    elif not ice_name and info.get("process") and info["process"].poll() is None:
        info["ice_status"] = "[blue]🔵 Running[/blue]"

def update_cpu_mem():
    while True:
        # CPU/memory update — fast, no blocking
        for info in processes.values():
            if info.get("paused", False):
                continue  # skip paused components
            try:
                proc = info["psutil_proc"]
                if proc.is_running():
                    info["mem_last"] = proc.memory_info().rss / (1024 ** 2)
                    info["cpu_last"] = proc.cpu_percent(interval=0.0)
                else:
                    info["mem_last"] = 0
                    info["cpu_last"] = 0
            except Exception:
                info["mem_last"] = 0
                info["cpu_last"] = 0

        # ICE pings — run all components in parallel so total wait ≤ 1s timeout
        futures = {
            _ping_executor.submit(_ping_one, name, info): name
            for name, info in processes.items()
        }
        # Wait for all pings to finish (max 2s safety cap)
        for future in as_completed(futures, timeout=2):
            pass  # results are written directly into info dict by _ping_one

        time.sleep(1)

threading.Thread(target=update_cpu_mem, daemon=True).start()
threading.Thread(target=read_keys,      daemon=True).start()
threading.Thread(target=watchdog_loop,  daemon=True).start()

try:
    with Live(build_table(), refresh_per_second=1, console=console, screen=False) as live:
        while True:
            time.sleep(1)
            live.update(build_table())
except KeyboardInterrupt:
    console.print("\n[yellow]Exiting. Terminating all processes...[/yellow]")
    
    # First, try graceful termination
    for name, info in processes.items():
        if info["process"]:  # Only terminate if we have a subprocess object
            try:
                info["process"].terminate()
                console.print(f"[dim]Terminating {name}...[/dim]")
            except Exception as e:
                console.print(f"[dim red]Error terminating {name}: {e}[/dim red]")
    
    # Wait up to 3 seconds for processes to terminate
    for name, info in processes.items():
        if info["process"]:
            try:
                info["process"].wait(timeout=3)
                console.print(f"[dim green]✓ {name} terminated[/dim green]")
            except subprocess.TimeoutExpired:
                console.print(f"[yellow]⚠ {name} didn't respond, force killing...[/yellow]")
                try:
                    info["process"].kill()
                    info["process"].wait(timeout=1)
                    console.print(f"[dim green]✓ {name} killed[/dim green]")
                except Exception as e:
                    console.print(f"[red]✗ Failed to kill {name}: {e}[/red]")
    
    console.print("[green]All processes terminated.[/green]")
    # Clean up the shared ICE communicator
    if _ice_communicator is not None:
        try:
            _ice_communicator.destroy()
        except Exception:
            pass
