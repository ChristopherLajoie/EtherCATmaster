#!/bin/bash

# Enable strict error handling
set -e

# Colorize output
GREEN='\e[0;32m'
YELLOW='\e[1;33m'
RED='\e[0;31m'
NC='\e[0m' # No Color

# Specify exact directories to search
SEARCH_DIRS=(
    "$HOME/EtherCAT_master/src"
    "$HOME/EtherCAT_master"
)

# Finding the .clang-format file
echo "=== Finding .clang-format file... ==="
CLANG_FORMAT_FILE=$(find "${SEARCH_DIRS[@]}" -name '.clang-format' -print -quit 2>/dev/null)
if [ -z "$CLANG_FORMAT_FILE" ]; then
    echo -e "${RED}[FAILED] .clang-format file not found in the specified directories!${NC}"
    exit 1
else
    echo -e "${GREEN}[OK]${NC} .clang-format file found at $CLANG_FORMAT_FILE"
fi

# Debug function to show found files
debug_find_files() {
    echo -e "${YELLOW}Debug: Searching for source files in specified directories${NC}"
    
    # Find files and show their full paths
    for dir in "${SEARCH_DIRS[@]}"; do
        echo -e "${YELLOW}Searching in: $dir${NC}"
        find "$dir" \( -name "*.c" -o -name "*.h" -o -name "*.cpp" -o -name "*.hpp" \) \
            -not -path "*/build/*" \
            -not -path "*/SOEM/*" | while read -r file; do
            echo -e "${GREEN}Found file:${NC} $file"
        done
    done
}

# Debug: Show full file list
debug_find_files

# Count files to be processed
echo "=== Counting files to process... ==="
FILE_COUNT=0
for dir in "${SEARCH_DIRS[@]}"; do
    count=$(find "$dir" \( -name "*.c" -o -name "*.h" -o -name "*.cpp" -o -name "*.hpp" \) \
        -not -path "*/build/*" \
        -not -path "*/SOEM/*" | wc -l)
    FILE_COUNT=$((FILE_COUNT + count))
done
echo -e "${YELLOW}Found $FILE_COUNT source files${NC}"

# Linting process
echo "=== Applying lint... ==="
START_TIME=$(date +%s)

# Use parallel processing if available
if command -v parallel &> /dev/null; then
    echo "Using GNU Parallel for faster processing..."
    for dir in "${SEARCH_DIRS[@]}"; do
        find "$dir" \( -name "*.c" -o -name "*.h" -o -name "*.cpp" -o -name "*.hpp" \) \
            -not -path "*/build/*" \
            -not -path "*/SOEM/*" | parallel -j$(nproc) clang-format -style=file -i
    done
else
    # Fallback to sequential processing with progress indicator
    for dir in "${SEARCH_DIRS[@]}"; do
        find "$dir" \( -name "*.c" -o -name "*.h" -o -name "*.cpp" -o -name "*.hpp" \) \
            -not -path "*/build/*" \
            -not -path "*/SOEM/*" -print0 | xargs -0 -I {} sh -c '
            clang-format -style=file -i "{}"
            printf "."
        '
    done
    echo  # New line after progress dots
fi

END_TIME=$(date +%s)
DURATION=$((END_TIME - START_TIME))

echo -e "\n${GREEN}[SUCCESS]${NC} Linting applied successfully!"
echo -e "${YELLOW}Total processing time: $DURATION seconds${NC}"