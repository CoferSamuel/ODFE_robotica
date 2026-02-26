#!/bin/bash

# Define colors for better readability
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}Starting recursive git pull...${NC}"

# Iterate through each directory
for dir in */; do
    if [ -d "$dir/.git" ]; then
        echo -e "\n${GREEN}Updating repository: ${dir}${NC}"
        # Use subshell to avoid permanent directory changes if cd fails
        (
            cd "$dir" || exit
            if git pull; then
                echo -e "${GREEN}Successfully updated ${dir}${NC}"
            else
                echo -e "${RED}Failed to update ${dir}. Check for local conflicts.${NC}"
            fi
        )
    else
        echo -e "Skipping ${dir}: Not a git repository."
    fi
done

echo -e "\n${GREEN}Process complete.${NC}"

# Prevent the window from closing
echo -e "${NC}---------------------------------------"
read -p "Press [Enter] key to close this window..."
