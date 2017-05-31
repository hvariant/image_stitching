#!/bin/bash

set -o xtrace

NFRAMES=3
output=${2:result}

# clear previous frame images
rm -rf tmp*

# extract frames
name=$(date -d "today" +"%Y%m%d%H%M%S")
mkdir tmp${name}
./extract_frames.sh $1 ${NFRAMES} tmp${name}

# perform stitching
./theseus/src/Theseus tmp${name}/*.png --output $2.png --ba ray --max_iter 100 --expos_comp gain --seam gc_color --blend multiband --blend_strength 50 --wave_correct yes --crop yes #--show_matches yes
