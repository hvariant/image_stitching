# ENGN6528 Project: Panoramic Stitching

This is the repository for the term project of ENGN6528 at the ANU.
For more details please read the [report](https://www.dropbox.com/s/l85lbb0brclcwg9/report.pdf?dl=0).

## Dependencies

- [opencv3](https://github.com/opencv/opencv)
- [opencv\_contrib](https://github.com/opencv/opencv_contrib)
- [cmake](https://cmake.org/)
- [ffmpeg](https://ffmpeg.org/)

## Build

In `theseus/src`, run

```
cmake .
make
```

## Run

```
./run.sh video.mp4 [output]
```

To change stitching parameters edit the last line in [run.sh](run.sh).

Usage:
```
$ ./Theseus

./Theseus img1 img2 [...imgN] [flags]

Flags:

Motion Estimation Flags:
  --work_megapix <float>
      Resolution for image registration step. The default is 0.6 Mpx.
  --features (surf|orb)
      Type of features used for images matching. The default is surf.
  --adjacent_matches <k>
      the number of adjacent images to do feature matching with, default is 3
  --match_conf <float>
      Confidence for feature matching step. The default is 0.65 for surf and 0.3 for orb.
  --show_matches yes|no
      Enable the program to show feature matches between images
  --conf_thresh <float>
      Threshold for two images are from the same panorama confidence.
      The default is 1.0.
  --ba (no|reproj|ray)
      Bundle adjustment cost function. The default is ray.
  --ba_refine_mask (mask)
      Set refinement mask for bundle adjustment. It looks like 'x_xxx',
      where 'x' means refine respective parameter and '_' means don't
      refine one, and has the following format:
      <fx><skew><ppx><aspect><ppy>. The default mask is 'xxxxx'.
      the respective flag is ignored.
      Ignored by ray bundle adjustment.
  --max_iter <n>
      the maximum number of iterations for bundle adjustment
  --wave_correct (no|yes)
      Perform wave effect correction.

Compositing Flags:
  --warp (affine|plane|cylindrical|spherical|stereographic)
      Warp surface type. The default is 'spherical'.
  --seam_megapix <float>
      Resolution for seam estimation step. The default is 0.1 Mpx.
  --seam (no|voronoi|gc_color|gc_colorgrad)
      Seam estimation method. The default is 'gc_color'.
  --expos_comp (no|gain|gain_blocks)
      Exposure compensation method. The default is 'gain'.
  --blend (no|multiband)
      Blending method. The default is 'multiband'.
  --blend_strength <float>
      Blending strength from [0,100] range. The default is 5.
  --crop yes|no
      Enable to allow the program to crop the output. The default is yes.
  --output <result_img>
      Name of output image. The default is 'result.png'.
```

# Contributors

- Zhansong Li (u5844206)
- Jiawei Wang (u5719594)
