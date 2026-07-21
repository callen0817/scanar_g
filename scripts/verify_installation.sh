#!/bin/bash

# Run validation checks
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
python3 "$DIR/check_dependencies.py"
exit $?
