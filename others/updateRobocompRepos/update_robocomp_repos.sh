#!/bin/bash

# Define colors for better readability
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Target directory: first argument or default to ~/robocomp
TARGET_DIR="${1:-$HOME/robocomp}"

if [ ! -d "$TARGET_DIR" ]; then
    echo -e "${RED}Error: directory '${TARGET_DIR}' does not exist.${NC}"
    exit 1
fi

echo -e "${GREEN}Starting recursive git pull in: ${TARGET_DIR}${NC}"

# Iterate through each subdirectory of TARGET_DIR
for dir in "$TARGET_DIR"/*/; do
    if [ -d "$dir/.git" ]; then
        echo -e "\n${GREEN}Updating repository: ${dir}${NC}"
        (
            cd "$dir" || exit
            if git pull; then
                echo -e "${GREEN}Successfully updated ${dir}${NC}"
            else
                echo -e "${RED}Failed to update ${dir}. Check for local conflicts.${NC}"
            fi
        )
    else
        echo -e "${YELLOW}Skipping ${dir}: Not a git repository.${NC}"
    fi
done

echo -e "\n${GREEN}Process complete.${NC}"

# Prevent the window from closing
echo -e "${NC}---------------------------------------"
read -p "Press [Enter] key to close this window..."
