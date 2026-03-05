#!/bin/bash

# Define colors for better readability
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Target directory: first argument or default to ~/robocomp
TARGET_DIR="${1:-$HOME/robocomp}"

if [ ! -d "$TARGET_DIR" ]; then
    echo -e "${RED}Error: directory '${TARGET_DIR}' does not exist.${NC}"
    exit 1
fi

echo -e "${GREEN}Starting recursive git pull in: ${TARGET_DIR}${NC}"

# Find all git repos recursively (up to depth 4) and update each one
while IFS= read -r gitdir; do
    dir="$(dirname "$gitdir")"
    echo -e "\n${CYAN}Checking repository: ${dir}${NC}"
    (
        cd "$dir" || exit

        # Discard any tracked modifications before pulling
        TRACKED_CHANGES=$(git status --porcelain | grep -v '^??')
        if [ -n "$TRACKED_CHANGES" ]; then
            echo -e "${YELLOW}⚠ Discarding tracked changes...${NC}"
            echo "$TRACKED_CHANGES" | sed 's/^/  /'
            git checkout -- . 2>/dev/null
            git reset HEAD . >/dev/null 2>&1
        fi

        if git pull; then
            echo -e "${GREEN}Successfully updated ${dir}${NC}"
        else
            echo -e "${RED}Failed to update ${dir}. Check network or remote status.${NC}"
        fi
    )
done < <(find "$TARGET_DIR" -maxdepth 4 -name ".git" -type d 2>/dev/null | sort)

echo -e "\n${GREEN}Process complete.${NC}"

# Prevent the window from closing
echo -e "${NC}---------------------------------------"
read -p "Press [Enter] key to close this window..."

