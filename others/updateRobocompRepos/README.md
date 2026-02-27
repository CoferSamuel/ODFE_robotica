# updateRobocompRepos

A small utility script to keep all local RoboComp-related git repositories up to date with a single command.

## What it does

`update_robocomp_repos.sh` iterates over every subdirectory of a target directory and executes `git pull` on each one that is a git repository. Non-git directories are skipped with a notice.

## Usage

Run from anywhere — including directly from this project's repo:

```bash
# Default target: ~/robocomp
bash others/updateRobocompRepos/update_robocomp_repos.sh

# Custom target directory
bash others/updateRobocompRepos/update_robocomp_repos.sh /path/to/repos
```

Or make it executable first:

```bash
chmod +x others/updateRobocompRepos/update_robocomp_repos.sh
./others/updateRobocompRepos/update_robocomp_repos.sh
```

## Output

- ✅ Green — successful pull
- ❌ Red — failed pull (e.g. local conflicts)
- 🟡 Yellow — directory skipped (not a git repo)
