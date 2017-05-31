#!/bin/bash

# clear previous frame images
rm -rf tmp*

set -o xtrace

name=$(date -d "today" +"%Y%m%d%H%M%S")
mkdir tmp${name}
./extract_frames.sh $1 tmp${name}

./theseus/src/Theseus tmp${name}/*.png --output $2.png --ba ray --max_iter 100 --expos_comp gain --seam gc_color --blend multiband --blend_strength 50 --wave_correct yes --crop yes #--show_matches yes
