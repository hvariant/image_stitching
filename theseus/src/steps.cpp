#include "steps.h"

int step1(
    const Param& param,
    //input
    const vector<String>& img_names,
    //output
    vector<Mat>& images, vector<Size>& full_img_sizes,
    double& work_scale, double& seam_scale,
    double& seam_work_aspect)
{
    //params
    double work_megapix = param.getDoubleParam("work_megapix");
    double seam_megapix = param.getDoubleParam("seam_megapix");

    if(readImages(img_names, images) < 0){
        return -1; // error occured when reading images
    }

    for(int i=0;i<images.size();i++){
        full_img_sizes.push_back(images[i].size());
    }

    work_scale = scaleFromMegapix(full_img_sizes[0], work_megapix);
    seam_scale = scaleFromMegapix(full_img_sizes[0], seam_megapix);
    seam_work_aspect = seam_scale / work_scale;

    return 0;
}

int step2(
    const Param& param,
    //input/output
    vector<Mat>& images,
    //input
    double work_scale, double seam_work_aspect,
    //output
    vector<ImageFeatures>& features)
{
    string features_type = param.getStringParam("features_type");

    Ptr<FeaturesFinder> finder;
    if (features_type == "surf")
    {
        finder = makePtr<SurfFeaturesFinder>();
    }
    else if (features_type == "orb")
    {
        finder = makePtr<OrbFeaturesFinder>();
    }
    else
    {
        cout << "Unknown 2D features type: '" << features_type << "'.\n";
        return -1;
    }

    // resize to work scale before finding features
    resizeImages(images,work_scale);
    findFeatures(finder, images, features);

    // resize to seam scale for later processsing
    resizeImages(images,seam_work_aspect);

    return 0;
}

int step3(
    const Param& param,
    //input
    const vector<ImageFeatures>& features,
    //output
    vector<MatchesInfo>& pairwise_matches)
{
    double match_conf = param.getDoubleParam("match_conf");
    double conf_thresh = param.getDoubleParam("conf_thresh");
    vector<String> img_names = param.getImageNames();
    int adjacent_matches = param.getIntParam("adjacent_matches");

    // make sure we only match adjacent images!!!!!!! This greatly improves performance of bundle adjustment!!!!!!
    int num_images = features.size();
    UMat matching_mask = UMat::zeros(num_images,num_images, CV_8U);
    for(int k=1;k<=adjacent_matches;k++){
        for(int i=0;i<num_images-k;i++){
            for(int j=i+1;j<=i+k;j++){
                matching_mask.getMat(cv::ACCESS_WRITE).at<uchar>(i,j) = 1;
            }
        }
    }

    Ptr<FeaturesMatcher> matcher;
    matcher = makePtr<BestOf2NearestMatcher>(false, match_conf);
    (*matcher)(features, pairwise_matches, matching_mask);
    matcher->collectGarbage();

    return 0;
}

int step4(
    const Param& param,
    //input
    const vector<ImageFeatures>& features,
    const vector<MatchesInfo>& pairwise_matches,
    //output
    vector<CameraParams>& cameras)
{
    string ba_cost_func = param.getStringParam("ba_cost_func");
    double conf_thresh = param.getDoubleParam("conf_thresh");
    string ba_refine_mask = param.getStringParam("ba_refine_mask");
    int max_iter = param.getIntParam("max_iter");

    Ptr<Estimator> estimator;
    estimator = makePtr<HomographyBasedEstimator>();
    if (!(*estimator)(features, pairwise_matches, cameras))
    {
        cout << "Homography estimation failed.\n";
        return -1;
    }

    for (size_t i = 0; i < cameras.size(); ++i)
    {
        Mat R;
        cameras[i].R.convertTo(R, CV_32F);
        cameras[i].R = R;
        //LOGLN("Initial camera intrinsics #" << indices[i]+1 << ":\nK:\n" << cameras[i].K() << "\nR:\n" << cameras[i].R);
        LOGLN("Initial camera intrinsics #" << i+1 << ":\nK:\n" << cameras[i].K() << "\nR:\n" << cameras[i].R);
    }

    Ptr<detail::BundleAdjusterBase> adjuster;
    if (ba_cost_func == "reproj") adjuster = makePtr<detail::BundleAdjusterReproj>();
    else if (ba_cost_func == "ray") adjuster = makePtr<detail::BundleAdjusterRay>();
    else if (ba_cost_func == "affine") adjuster = makePtr<detail::BundleAdjusterAffinePartial>();
    else if (ba_cost_func == "no") adjuster = makePtr<detail::NoBundleAdjuster>();
    else
    {
        cout << "Unknown bundle adjustment cost function: '" << ba_cost_func << "'.\n";
        return -1;
    }
    adjuster->setConfThresh(conf_thresh);
    Mat_<uchar> refine_mask = Mat::zeros(3, 3, CV_8U);
    if (ba_refine_mask[0] == 'x') refine_mask(0,0) = 1;
    if (ba_refine_mask[1] == 'x') refine_mask(0,1) = 1;
    if (ba_refine_mask[2] == 'x') refine_mask(0,2) = 1;
    if (ba_refine_mask[3] == 'x') refine_mask(1,1) = 1;
    if (ba_refine_mask[4] == 'x') refine_mask(1,2) = 1;
    adjuster->setRefinementMask(refine_mask);
    adjuster->setTermCriteria(cv::TermCriteria(cv::TermCriteria::MAX_ITER, max_iter, 0));
    if (!(*adjuster)(features, pairwise_matches, cameras))
    {
        cout << "Camera parameters adjusting failed.\n";
        return -1;
    }

    return 0;
}

int step5(
    const Param& param,
    //input/output
    vector<CameraParams>& cameras,
    //output
    double& warped_image_scale)
{
    bool do_wave_correct = param.getBoolParam("do_wave_correct");
    WaveCorrectKind wave_correct = static_cast<WaveCorrectKind>(param.getIntParam("wave_correct"));

    // Find median focal length
    vector<double> focals;
    for (size_t i = 0; i < cameras.size(); ++i)
    {
        //LOGLN("Camera #" << indices[i]+1 << ":\nK:\n" << cameras[i].K() << "\nR:\n" << cameras[i].R);
        //LOGLN("Camera #" << i+1 << ":\nK:\n" << cameras[i].K() << "\nR:\n" << cameras[i].R);
        focals.push_back(cameras[i].focal);
    }
    sort(focals.begin(), focals.end());
    if (focals.size() % 2 == 1)
        warped_image_scale = static_cast<float>(focals[focals.size() / 2]);
    else
        warped_image_scale = static_cast<float>(focals[focals.size() / 2 - 1] + focals[focals.size() / 2]) * 0.5f;

    if (do_wave_correct)
    {
        vector<Mat> rmats;
        for (size_t i = 0; i < cameras.size(); ++i)
            rmats.push_back(cameras[i].R.clone());
        waveCorrect(rmats, wave_correct);
        for (size_t i = 0; i < cameras.size(); ++i)
            cameras[i].R = rmats[i];
    }

    return 0;
}

int step6(
    const Param& param,
    //input
    const vector<CameraParams>& cameras,
    const vector<Mat>& images,
    double warped_image_scale, double seam_work_aspect,
    //output
    vector<UMat>& images_warped,
    vector<UMat>& images_warped_f,
    vector<Point>& corners,
    vector<UMat>& masks_warped,
    Ptr<WarperCreator>& warper_creator
)
{
    string warp_type = param.getStringParam("warp_type");
    size_t num_images = images.size();

    // Warp images and their masks
    if (warp_type == "plane")
        warper_creator = makePtr<cv::PlaneWarper>();
    else if (warp_type == "affine")
        warper_creator = makePtr<cv::AffineWarper>();
    else if (warp_type == "cylindrical")
        warper_creator = makePtr<cv::CylindricalWarper>();
    else if (warp_type == "spherical")
        warper_creator = makePtr<cv::SphericalWarper>();

    if (!warper_creator)
    {
        cout << "Can't create the following warper '" << warp_type << "'\n";
        return -1;
    }
    Ptr<RotationWarper> warper = warper_creator->create(static_cast<float>(warped_image_scale * seam_work_aspect));

    warpImagesAndMasks(images, cameras, warper, seam_work_aspect,
            images_warped, corners, masks_warped);

    for (int i = 0; i < num_images; ++i){
        UMat umat;
        images_warped[i].convertTo(umat, CV_32F);
        images_warped_f.push_back(umat.clone());
    }

    return 0;
}


int step7(
    const Param& param,
    //input/output
    vector<UMat>& masks_warped,
    //input
    const vector<Point>& corners,
    const vector<UMat>& images_warped,
    const vector<UMat>& images_warped_f,
    //output
    Ptr<ExposureCompensator>& compensator)
{
    int expos_comp_type = param.getIntParam("expos_comp_type");
    string seam_find_type = param.getStringParam("seam_find_type");

    compensator = ExposureCompensator::createDefault(expos_comp_type);
    compensator->feed(corners, images_warped, masks_warped);

    // find seam and save in masks_warped
    Ptr<SeamFinder> seam_finder;
    if (seam_find_type == "no")
        seam_finder = makePtr<detail::NoSeamFinder>();
    else if (seam_find_type == "voronoi")
        seam_finder = makePtr<detail::VoronoiSeamFinder>();
    else if (seam_find_type == "gc_color")
    {
        seam_finder = makePtr<detail::GraphCutSeamFinder>(detail::GraphCutSeamFinderBase::COST_COLOR);
    }
    else if (seam_find_type == "gc_colorgrad")
    {
        seam_finder = makePtr<detail::GraphCutSeamFinder>(detail::GraphCutSeamFinderBase::COST_COLOR_GRAD);
    }
    else if (seam_find_type == "dp_color")
        seam_finder = makePtr<detail::DpSeamFinder>(detail::DpSeamFinder::COLOR);
    else if (seam_find_type == "dp_colorgrad")
        seam_finder = makePtr<detail::DpSeamFinder>(detail::DpSeamFinder::COLOR_GRAD);

    if (!seam_finder)
    {
        cout << "Can't create the following seam finder '" << seam_find_type << "'\n";
        return -1;
    }
    seam_finder->find(images_warped_f, corners, masks_warped);

    return 0;
}

int step8(
    const Param& param,
    //input
    vector<CameraParams>& cameras,
    const vector<Size>& full_img_sizes,
    const vector<UMat>& masks_warped,
    const Ptr<WarperCreator>& warper_creator,
    const Ptr<ExposureCompensator>& compensator,
    double warped_image_scale, double work_scale,
    //output
    Mat& result, Mat& result_mask)
{
    vector<String> img_names = param.getImageNames();
    int blend_type = param.getIntParam("blend_type");
    double blend_strength = param.getDoubleParam("blend_strength");

    LOGLN("blend_type:" << blend_type);

    size_t num_images = img_names.size();
    vector<Point> corners(num_images);
    vector<Size> sizes(num_images);

    Mat img_warped, img_warped_s;
    Mat dilated_mask, seam_mask, mask, mask_warped;
    Ptr<Blender> blender;
    Mat img;

    warped_image_scale /= work_scale;
    Ptr<RotationWarper> warper = warper_creator->create(warped_image_scale);
    for (int i = 0; i < num_images; ++i)
    {
        // update intrinsics
        cameras[i].focal /= work_scale;
        cameras[i].ppx /= work_scale;
        cameras[i].ppy /= work_scale;

        // update corner and size
        Size sz = full_img_sizes[i];
        Mat K;
        cameras[i].K().convertTo(K, CV_32F);
        Rect roi = warper->warpRoi(sz, K, cameras[i].R);
        corners[i] = roi.tl();
        sizes[i] = roi.size();
    }
    for (int img_idx = 0; img_idx < num_images; ++img_idx)
    {
        //LOGLN("Compositing image #" << indices[img_idx]+1);
        LOGLN("Compositing image #" << img_idx+1);

        // Read image and resize it if necessary
        img = cv::imread(img_names[img_idx]);
        Size img_size = img.size();

        Mat K;
        cameras[img_idx].K().convertTo(K, CV_32F);

        // Warp the current image
        warper->warp(img, K, cameras[img_idx].R, cv::INTER_LINEAR, cv::BORDER_REFLECT, img_warped);

        // Warp the current image mask
        mask.create(img_size, CV_8U);
        mask.setTo(Scalar::all(255));
        warper->warp(mask, K, cameras[img_idx].R, cv::INTER_NEAREST, cv::BORDER_CONSTANT, mask_warped);

        // Compensate exposure
        compensator->apply(img_idx, corners[img_idx], img_warped, mask_warped);

        img_warped.convertTo(img_warped_s, CV_16S);
        img_warped.release();
        img.release();
        mask.release();

        dilate(masks_warped[img_idx], dilated_mask, Mat());
        resize(dilated_mask, seam_mask, mask_warped.size());
        mask_warped = seam_mask & mask_warped;

        if (!blender)
        {
            blender = Blender::createDefault(blend_type, false);
            Size dst_sz = detail::resultRoi(corners, sizes).size();
            float blend_width = sqrt(static_cast<float>(dst_sz.area())) * blend_strength / 100.f;
            if (blend_width < 1.f)
                blender = Blender::createDefault(Blender::NO, false);
            else if (blend_type == Blender::MULTI_BAND)
            {
                MultiBandBlender* mb = dynamic_cast<MultiBandBlender*>(blender.get());
                mb->setNumBands(static_cast<int>(ceil(log(blend_width)/log(2.)) - 1.));
                LOGLN("Multi-band blender, number of bands: " << mb->numBands());
            }
            else if (blend_type == Blender::FEATHER)
            {
                FeatherBlender* fb = dynamic_cast<FeatherBlender*>(blender.get());
                fb->setSharpness(1.f/blend_width);
                LOGLN("Feather blender, sharpness: " << fb->sharpness());
            }
            blender->prepare(corners, sizes);
        }

        // Blend the current image
        blender->feed(img_warped_s, mask_warped, corners[img_idx]);
    }

    blender->blend(result, result_mask);

    return 0;
}
