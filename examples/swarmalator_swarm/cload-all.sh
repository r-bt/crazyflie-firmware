#!/usr/bin/env bash
# Reset

COLOR_RESET='\033[0m'       # Text Reset
YELLOW='\033[0;33m'         # Yellow
RED='\033[0;31m'            # Red

# Bash ranges can be continious like in {1..9} or non continious like in 2 {4..5}
# Change the defconfig to match the platform
for i in {1..10}; do
    hex=$(printf "%02X" "$i")
    printf "${YELLOW}Flashing ${RED}CF ${YELLOW} ${hex}${COLOR_RESET}\n"
    CLOAD_CMDS="-w radio://0/80/2M/E7E7E7E7${hex}" make cload
done