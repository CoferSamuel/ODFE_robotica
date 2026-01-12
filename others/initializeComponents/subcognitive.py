#!/usr/bin/env python3

import subprocess
import time
import os
from pathlib import Path
import Ice
import psutil
from rich.console import Console
from rich.table import Table
from rich.live import Live
from rich import box
import toml
import argparse
import threading
import pprint

parser = argparse.ArgumentParser(description="RoboComp subcognitive monitor")
parser.add_argument("file_name", type=str, default="sub.toml", help="Path to TOML components file")
args = parser.parse_args()
console = Console()

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
                ["/usr/local/bin/webots"],
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
                ["bash", "/home/cofer/robocomp/tools/rcnode/rcnode.sh"],
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

# Load components from TOML file
def toml_loader():
    config_path = os.path.expanduser(args.file_name)
    if not os.path.exists(config_path):
        console.print(f"[red]Missing file with components at {config_path}[/red]")
        exit(1)

    components = toml.load(args.file_name)["components"]
    return components

def ping_proxy(ice_string):
    try:
        # Initialize Ice communicator if not already done
        with Ice.initialize() as communicator:
            proxy = communicator.stringToProxy(ice_string)
            # Narrow to Ice::Object (the base interface)
            obj = proxy.ice_ping()
            return True
    except Exception as e:
        return False

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
            "is_webots": True
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
            "is_rcnode": True
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
def launch_process(command, cwd=None, name=None):
    # Create output directory in the project folder
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.join(script_dir, "output")
    os.makedirs(output_dir, exist_ok=True)
    
    # Only capture stderr for errors, discard stdout to prevent huge files
    stdout_path = os.devnull  # Discard stdout to prevent massive log files
    stderr_path = os.path.join(output_dir, f"{name}.err") if name else os.devnull

    # stdout = open(stdout_path, "w")
    # stderr = open(stderr_path, "w")
    
    # We will pipe stderr to our rotation function
    # stdout is still discarded to devnull

    # Replace 'rcnode' alias with actual script path
    if 'rcnode' in command and not command.startswith('bash /'):
        command = command.replace('rcnode', 'bash /home/cofer/robocomp/tools/rcnode/rcnode.sh')

    proc = subprocess.Popen(
        command,
        cwd=cwd,
        shell=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,  # Capture stderr
        env=os.environ.copy(),
        executable='/bin/bash'  # Use bash instead of sh to handle the command properly
    )

    # Start the log rotator thread if we have a name (and thus a log file)
    if name:
        t = threading.Thread(target=monitor_and_rotate_log, args=(proc.stderr, stderr_path), daemon=True)
        t.start()

    time.sleep(0.3)  # wait for child process to start
    try:
        children = psutil.Process(proc.pid).children()
        if children:
            ps_proc = children[0]  # use real process, not shell
        else:
            ps_proc = psutil.Process(proc.pid)
    except Exception:
        ps_proc = psutil.Process(proc.pid)
    return proc, ps_proc

components = toml_loader()
def print_components_table(components):
    table = Table(title="🧠 Loaded Components", box=box.SIMPLE_HEAVY)
    table.add_column("Name", style="bold cyan")
    table.add_column("CWD", style="dim")
    table.add_column("Command", style="magenta")

    for comp in components:
        table.add_row(
            comp["name"],
            comp.get("cwd", "-"),
            comp.get("cmd", "-")
        )

    console.print(table)

print_components_table(components)

for comp in components:
    cwd = expand_path(comp.get("cwd"))
    command = comp["cmd"]
    print(f"Starting {comp['name']}...")
    proc, ps_proc = launch_process(command, cwd=cwd, name=comp["name"])
    ps_proc.cpu_percent(interval=None)  # <<< Add this here
    processes[comp["name"]] = {
        "process": proc,
        "psutil_proc": ps_proc,
        "ice_name": comp.get("ice_name"),
        "start_time": time.time()
    }

#Monitor loop
def build_table():
    table = Table(title="🧠 RoboComp Component Monitor", box=box.SIMPLE_HEAVY)
    table.add_column("Name", style="bold cyan")
    table.add_column("Status", style="bold")
    table.add_column("Uptime", justify="right")
    table.add_column("Memory", justify="right")
    table.add_column("CPU", justify="right")

    for name, info in processes.items():
        ice_name = info["ice_name"]
        status = "[yellow]⏳ Checking...[/yellow]"
        
        # Special handling for Webots and rcnode (system processes we don't control directly)
        if info.get("is_webots", False) or info.get("is_rcnode", False):
            try:
                if info["psutil_proc"].is_running():
                    status = "[green]✅ Running[/green]"
                else:
                    status = "[red]❌ Stopped[/red]"
            except Exception:
                status = "[red]❌ Stopped[/red]"
        else:
            # Regular component handling
            proc = info["process"]
            running = proc.poll() is None

            if running:
                # Ice ping
                try:
                    if ice_name:
                        ping_proxy(ice_name)
                        status = "[green]✅ Alive[/green]"
                    else:
                        status = "[blue]⚠️ No ICE[/blue]"
                except Exception:
                    status = "[red]❌ Down[/red]"
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
            name,
            status,
            uptime_str,
            f"{mem:6.1f} MB",
            #f"{cpu:5.1f}%"
            f"{cpu:5.1f}% {cpu_usage_bar(cpu)}"
        )

    return table

def update_cpu_mem():
    while True:
        for info in processes.values():
            try:
                proc = info["psutil_proc"]
                if proc.is_running():
                    info["mem_last"] = proc.memory_info().rss / (1024 ** 2)
                    info["cpu_last"] = proc.cpu_percent(interval=0.0)
            except Exception:
                info["mem_last"] = 0
                info["cpu_last"] = 0
        time.sleep(1)

threading.Thread(target=update_cpu_mem, daemon=True).start()

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

