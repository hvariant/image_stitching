#!/bin/bash

# -r followed by how many frames per second
ffmpeg -i $1 -r 3 $2/keyi%03d.png
