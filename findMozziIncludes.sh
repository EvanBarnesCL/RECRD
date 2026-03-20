#!/bin/bash
# Crawl all .h, .hpp, and .cpp files in the project and print every
# #include line that references a Mozzi header, along with the file
# and line number it was found on.

PROJECT_ROOT="${1:-.}"

echo "Searching in: $PROJECT_ROOT"
echo "----------------------------------------"

grep -rn \
    --include="*.h" \
    --include="*.hpp" \
    --include="*.cpp" \
    "#include" \
    "$PROJECT_ROOT/src" \
    "$PROJECT_ROOT/include" \
    "$PROJECT_ROOT/lib" \
    "$PROJECT_ROOT/.pio/libdeps" \
    2>/dev/null \
| grep -i "mozzi" \
| sort

echo "----------------------------------------"
echo "Done."