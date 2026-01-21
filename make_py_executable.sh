#!/bin/bash

echo "Searching for Python scripts in all packages..."

# Find all .py files in any scripts/ directory and make them executable
find . -type f -path "*/scripts/*.py" -exec chmod +x {} \;

# Also make this script executable for next time
chmod +x make_executable.sh

echo "✅ All Python scripts are now executable!"
echo ""
echo "Made executable:"
find . -type f -path "*/scripts/*.py" -executable -ls | awk '{print $NF}'