#!/bin/bash

# -r followed by how many frames per second
ffmpeg -i $1 -r $2 $3/keyi%03d.png
