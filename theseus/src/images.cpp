#include "theseus.h"

int readImages(const vector<String>& image_names, vector<Mat>& images){
    Mat img;

    for(size_t i=0;i<image_names.size();i++){
        img = imread(image_names[i]);

        LOGLN("reading image " << image_names[i]);

        if (img.empty())
        {
            LOGLN("Can't open image " << image_names[i]);
            return -1;
        }

        images.push_back(img.clone());
    }

    //test
    for(size_t i=0;i<images.size();i++){
        LOGLN("images[" << i << "].size() = " << images[i].size());
    }

    img.release();
    return 0;
}

void resizeImages(const vector<Mat>& images_in, vector<Mat>& images_out, double scale){
    for(size_t i=0;i<images_in.size();i++){
        images_out.push_back(images_in[i].clone());
    }

    resizeImages(images_out, scale);
}

void resizeImages(vector<Mat>& images_in, double scale){
    Mat full_img, img;
    for(size_t i=0;i<images_in.size();i++){
        full_img = images_in[i];
        resize(full_img, img, Size(), scale, scale);
        images_in[i] = img.clone();
    }

    img.release();
    full_img.release();
}

double scaleFromMegapix(Size img_size, double megapix){
    if(megapix < 0) return 1.0;
    return cv::min(1.0, sqrt(megapix * 1e6 / img_size.area()));
}

void warpImagesAndMasks(
        //input
        const vector<Mat>& images, const vector<CameraParams>& cameras,
        Ptr<RotationWarper> warper, double seam_work_aspect,
        //output
        vector<UMat>& images_warped, vector<Point>& corners, vector<UMat>& masks_warped)
{
    size_t num_images = images.size();
    vector<UMat> masks(num_images);

    // Preapre images masks
    for (int i = 0; i < num_images; ++i)
    {
        masks[i].create(images[i].size(), CV_8U);
        masks[i].setTo(Scalar::all(255));
    }

    for (int i = 0; i < num_images; ++i)
    {
        Mat_<float> K;
        cameras[i].K().convertTo(K, CV_32F);
        float swa = (float)seam_work_aspect;
        K(0,0) *= swa; K(0,2) *= swa;
        K(1,1) *= swa; K(1,2) *= swa;

        UMat umat;
        corners.push_back(warper->warp(images[i], K, cameras[i].R, cv::INTER_LINEAR, cv::BORDER_REFLECT, umat));
        images_warped.push_back(umat.clone());

        UMat umat2;
        warper->warp(masks[i], K, cameras[i].R, cv::INTER_NEAREST, cv::BORDER_CONSTANT, umat2);
        masks_warped.push_back(umat2.clone());
    }

    masks.clear();
}

int getBestSide(const Mat& result_mask, const Rect& rect){
    int best_count = 0, best_dir = -1;
    int count;

    // left
    count = 0;
    for(int j=rect.y;j<rect.y+rect.height;j++){
        if(result_mask.at<uchar>(j,rect.x) == 0) count++;
    }
    if(count > best_count){
        best_count = count;
        best_dir = 1;
    }

    // right
    count = 0;
    for(int j=rect.y;j<rect.y+rect.height;j++){
        if(result_mask.at<uchar>(j,rect.x+rect.width-1) == 0) count++;
    }
    if(count > best_count){
        best_count = count;
        best_dir = 2;
    }

    // top
    count = 0;
    for(int i=rect.x;i<rect.x+rect.width;i++){
        if(result_mask.at<uchar>(rect.y,i) == 0) count++;
    }
    if(count > best_count){
        best_count = count;
        best_dir = 3;
    }

    // bottom
    count = 0;
    for(int i=rect.x;i<rect.x+rect.width;i++){
        if(result_mask.at<uchar>(rect.y+rect.height-1,i) == 0) count++;
    }
    if(count > best_count){
        best_count = count;
        best_dir = 4;
    }

    //LOGLN("best_count" << best_count);
    return best_dir;
}

void cropImage(const Mat& result_mask, Mat& result){
    Rect inRect;
    inRect.x = inRect.y = 0;
    inRect.width = result_mask.size().width;
    inRect.height = result_mask.size().height;

    int best_dir = getBestSide(result_mask, inRect);
    while(best_dir != -1){
        switch(best_dir){
            case 1:
            {
                inRect.x++;
                break;
            }
            case 2:
            {
                inRect.width--;
                break;
            }
            case 3:
            {
                inRect.y++;
                break;
            }
            case 4:
            {
                inRect.height--;
                break;
            }
        }

        best_dir = getBestSide(result_mask, inRect);
    }

    result = result(inRect);
}
