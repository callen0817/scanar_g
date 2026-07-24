#!/bin/bash

# Get directory of current script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Run dependency checker
python3 "$DIR/check_dependencies.py"
exit $?
