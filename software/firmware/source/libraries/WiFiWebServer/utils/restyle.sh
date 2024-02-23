#!/bin/bash

for dir in . ; do
    find $dir -type f \( -name "*.c" -o -name "*.h" -o -name "*.cpp" -o -name "*.ino" \) -exec astyle --suffix=none --options=./utils/astyle_library.conf \{\} \;
done

