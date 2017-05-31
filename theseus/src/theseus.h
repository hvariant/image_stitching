#ifndef __THESEUS_H__
#define __THESEUS_H__

#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include "opencv2/opencv_modules.hpp"
#include <opencv2/core/utility.hpp>
#include <opencv2/core/types.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/stitching/detail/autocalib.hpp"
#include "opencv2/stitching/detail/blenders.hpp"
#include "opencv2/stitching/detail/camera.hpp"
#include "opencv2/stitching/detail/exposure_compensate.hpp"
#include "opencv2/stitching/detail/matchers.hpp"
#include "opencv2/stitching/detail/motion_estimators.hpp"
#include "opencv2/stitching/detail/seam_finders.hpp"
#include "opencv2/stitching/detail/warpers.hpp"
#include "opencv2/stitching/warpers.hpp"

using std::map;
using std::vector;
using std::string;
using std::cout;

using cv::Mat;
using cv::Mat_;
using cv::UMat;
using cv::Point;
using cv::String;
using cv::Size;
using cv::Scalar;
using cv::Rect;

using cv::Ptr;
using cv::makePtr;

using cv::getTickCount;
using cv::getTickFrequency;

using cv::WarperCreator;

namespace detail = cv::detail;
using detail::FeaturesFinder;
using detail::ImageFeatures;
using detail::WaveCorrectKind;
using detail::ExposureCompensator;
using detail::FeaturesFinder;
using detail::SurfFeaturesFinder;
using detail::OrbFeaturesFinder;

using detail::FeaturesMatcher;
using detail::MatchesInfo;
using detail::AffineBestOf2NearestMatcher;
using detail::BestOf2NearestMatcher;
using detail::BestOf2NearestRangeMatcher;

using detail::CameraParams;
using detail::Estimator;
using detail::AffineBasedEstimator;
using detail::HomographyBasedEstimator;

using detail::RotationWarper;

using detail::SeamFinder;

using detail::Blender;
using detail::MultiBandBlender;
using detail::FeatherBlender;

#define ENABLE_LOG 1
#define LOG(msg) std::cout << msg << std::flush
#define LOGLN(msg) std::cout << msg << std::endl << std::flush

// read in image and returns image matrix
int readImages(const vector<String>& image_names, vector<Mat>& images);

// resizes images_on and puts them in images_out
void resizeImages(const vector<Mat>& images_in, vector<Mat>& images_out, double scale);

// resizes images_in in place
void resizeImages(vector<Mat>& images_in, double scale);

// calculate scaling factor from megapix and actual image size
double scaleFromMegapix(Size img_size, double megapix);

// feature extraction
void findFeatures(Ptr<FeaturesFinder> feature_finder, const vector<Mat>& images, vector<ImageFeatures>& features);

// get warped images and masks
void warpImagesAndMasks(const vector<Mat>& images, const vector<CameraParams>& cameras, Ptr<RotationWarper> warper, double seam_work_aspect,
    vector<UMat>& images_warped, vector<Point>& corners, vector<UMat>& masks_warped);

// crop image according to mask
void cropImage(const Mat& result_mask, Mat& result);

// visualize feature matches
void visualiseMatches(const vector<String>& img_names, const vector<ImageFeatures>& features, const vector<MatchesInfo>& matches, double work_scale, int N=10);

#endif
