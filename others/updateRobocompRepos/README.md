# updateRobocompRepos

A small utility script to keep all local RoboComp-related git repositories up to date with a single command.

## What it does

`update_robocomp_repos.sh` recursively searches a target directory (up to depth 4) for git repositories and executes `git pull` on each one. This ensures nested repos (e.g. under `components/`) are not missed.

- **Untracked files** (build artifacts like `CMakeCache.txt`) are ignored and never block pulls.
- **Tracked modifications** cause the repo to be skipped (unless `--force` is used).

## Usage

```bash
# Default target: ~/robocomp
bash others/updateRobocompRepos/update_robocomp_repos.sh

# Custom target directory
bash others/updateRobocompRepos/update_robocomp_repos.sh /path/to/repos

# Discard tracked changes and pull anyway
bash others/updateRobocompRepos/update_robocomp_repos.sh --force

# Custom target + force
bash others/updateRobocompRepos/update_robocomp_repos.sh /path/to/repos --force
```

## Output

- ✅ Green — successful pull
- ❌ Red — failed pull (e.g. network issues)
- 🟡 Yellow — tracked changes detected, skipped (or discarded with `--force`)

