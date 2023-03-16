# simple script to update all examples' flash_config.h based on the file in this folder

import os
import sys
import glob
import shutil

new_file = 'tools/flash_config.h'

all_files = list(glob.iglob('examples/*/flash_config.h'))

for f in all_files:
    shutil.copyfile(new_file, f)