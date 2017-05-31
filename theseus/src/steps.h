#ifndef __STEPS_H__
#define __STEPS_H__

#include "theseus.h"
#include "param.h"

int step1(
    const Param& param,
    //input
    const vector<String>& img_names,
    //output
    vector<Mat>& images, vector<Size>& full_img_sizes,
    double& work_scale, double& seam_scale,
    double& seam_work_aspect
);

int step2(
    const Param& param,
    //input/output
    vector<Mat>& images,
    //input
    double work_scale, double seam_work_aspect,
    //output
    vector<ImageFeatures>& features
);

int step3(
    const Param& param,
    //input
    const vector<ImageFeatures>& features,
    //output
    vector<MatchesInfo>& pairwise_matches
);

int step4(
    const Param& param,
    //input
    const vector<ImageFeatures>& features,
    const vector<MatchesInfo>& pairwise_matches,
    //output
    vector<CameraParams>& cameras
);

int step5(
    const Param& param,
    //input/output
    vector<CameraParams>& cameras,
    //output
    double& warped_image_scale
);

int step6(
    const Param& param,
    //input
    const vector<CameraParams>& cameras, const vector<Mat>& images,
    double warped_image_scale, double seam_work_aspect,
    //output
    vector<UMat>& images_warped,
    vector<UMat>& images_warped_f,
    vector<Point>& corners,
    vector<UMat>& masks_warped,
    Ptr<WarperCreator>& warper_creator
);

int step7(
    const Param& param,
    //input/output
    vector<UMat>& masks_warped,
    //input
    const vector<Point>& corners,
    const vector<UMat>& images_warped,
    const vector<UMat>& images_warped_f,
    //output
    Ptr<ExposureCompensator>& compensator
);

int step8(
    const Param& param,
    //input/output
    vector<CameraParams>& cameras,
    //input
    const vector<Size>& full_img_sizes,
    const vector<UMat>& masks_warped,
    const Ptr<WarperCreator>& warper_creator,
    const Ptr<ExposureCompensator>& compensator,
    double warped_image_scale, double work_scale,
    //output
    Mat& result, Mat& result_mask
);

#endif
