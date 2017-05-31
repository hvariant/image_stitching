
#include "theseus.h"
#include "param.h"
#include "steps.h"

int main(int argc, char* argv[])
{
#if ENABLE_LOG
    int64 app_start_time = getTickCount();
#endif

    Param param;

#if 0
    cv::setBreakOnError(true);
#endif

    int retval = param.parseCmdArgs(argc, argv);
    if (retval < 0)
        return retval;

    vector<String> img_names = param.getImageNames();
    // Check if have enough images
    size_t num_images = img_names.size();
    if (num_images < 2)
    {
        LOGLN("Need more images");
        return -1;
    }

/************************************************************************************************
* read in images and calculate work_scale and seam_scale
*************************************************************************************************/

    vector<Mat> images;
    vector<Size> full_img_sizes;
    double seam_work_aspect = 1;
    double work_scale = 1, seam_scale = 1;

    int status =
        step1(
            param,
            // input
            img_names,
            // output
            images, full_img_sizes,
            work_scale, seam_scale,
            seam_work_aspect
        );
    if(status < 0) return -1;

/************************************************************************************************
* find image features
*************************************************************************************************/

    LOGLN("Finding features...");
#if ENABLE_LOG
    int64 t = getTickCount();
#endif

    vector<ImageFeatures> features;
    status =
        step2(
            param,
            // input/output
            images,
            //input
            work_scale, seam_work_aspect,
            //output
            features
        );
    if(status < 0) return -1;

    LOGLN("Finding features, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");

/************************************************************************************************
* pairwise feature matching
*************************************************************************************************/

    LOG("Pairwise feature matching ...");
#if ENABLE_LOG
    t = getTickCount();
#endif

    vector<MatchesInfo> pairwise_matches;
    status =
        step3(
            param,
            //input
            features,
            //output
            pairwise_matches
        );
    if(status < 0) return -1;

    LOGLN("Pairwise feature matching, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");

    // visualise features
    bool show_matches = param.getBoolParam("show_matches");
    if(show_matches){
        visualiseMatches(img_names, features, pairwise_matches, work_scale);
    }

/*********************************************************************************************************
* do pairwise camera parameters estimation and then bundle adjustment
**********************************************************************************************************/

    LOG("camera parameter estimation and bundle adjustment ...");
#if ENABLE_LOG
    t = getTickCount();
#endif

    vector<CameraParams> cameras;
    status =
        step4(
            param,
            //input
            features, pairwise_matches,
            //output
            cameras
        );
    if(status < 0) return -1;

    LOGLN("camera parameter estimation and bundle adjustment, time:" << ((getTickCount() - t) / getTickFrequency()) << " sec");

/************************************************************************************************
* get initial warped_image_scale and then do wave correction
*************************************************************************************************/

    LOG("Wave correction ...");
#if ENABLE_LOG
    t = getTickCount();
#endif

    double warped_image_scale;
    status =
        step5(
            param,
            //input/output
            cameras,
            //output
            warped_image_scale
        );
    if(status < 0) return -1;

    LOGLN("wave correction, time:" << ((getTickCount() - t) / getTickFrequency()) << " sec");

/************************************************************************************************
* warping images and masks
************************************************************************************************/

    LOGLN("Warping images and masks ... ");
#if ENABLE_LOG
    t = getTickCount();
#endif

    vector<UMat> images_warped;
    vector<UMat> images_warped_f;
    vector<Point> corners;
    vector<UMat> masks_warped;
    Ptr<WarperCreator> warper_creator;

    status = step6(
                param,
                //input
                cameras, images,
                warped_image_scale, seam_work_aspect,
                //output
                images_warped,
                images_warped_f,
                corners,
                masks_warped,
                warper_creator
            );
    if(status < 0) return -1;

    // Release unused memory
    images.clear();

    LOGLN("Warping images and masks, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");

/************************************************************************************************
* use warped masks to find seam, initialize exposure compensator
*************************************************************************************************/

    LOGLN("Find seams and initialize exposure compensator");
#if ENABLE_LOG
    t = getTickCount();
#endif

    // initialize exposure compensator
    Ptr<ExposureCompensator> compensator;
    
    status = step7(
                param,
                //input/output
                masks_warped,
                //input
                corners,
                images_warped,
                images_warped_f,
                //output
                compensator
            );
    if(status < 0) return -1;

    // Release unused memory
    images_warped.clear();
    images_warped_f.clear();

    LOGLN("Find seams and initialize exposure compensator, time:" << ((getTickCount() - t) / getTickFrequency()) << " sec");

/************************************************************************************************
* composition (warping, exposure compensation and blending)
*************************************************************************************************/

    LOGLN("Compositing...");
#if ENABLE_LOG
    t = getTickCount();
#endif

    Mat result, result_mask;
    status = step8(
                param,
                //input/output
                cameras,
                //input
                full_img_sizes, masks_warped,
                warper_creator, compensator,
                warped_image_scale, work_scale,
                //output
                result, result_mask
            );
    if(status < 0) return -1;

    LOGLN("Compositing, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");

/************************************************************************************************
* cropping (or not, depending on the argument)
*************************************************************************************************/

    bool crop = param.getBoolParam("crop");
    if(crop){
        cropImage(result_mask, result);
    }

/************************************************************************************************
* output result to file
*************************************************************************************************/

    string result_name = param.getStringParam("result_name");
    imwrite(result_name, result);
    LOGLN("Finished, total time: " << ((getTickCount() - app_start_time) / getTickFrequency()) << " sec");
    return 0;
}
