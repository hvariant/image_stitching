#include "theseus.h"

void findFeatures(Ptr<FeaturesFinder> finder, const vector<Mat>& images, vector<ImageFeatures>& features){
    Mat img;

    for (size_t i = 0; i < images.size(); ++i)
    {
        img = images[i];

        ImageFeatures feature;
        (*finder)(img, feature);
        feature.img_idx = i;

        features.push_back(feature);
        LOGLN("Features in image #" << i+1 << ": " << features[i].keypoints.size());
    }

    finder->collectGarbage();
    img.release();
}

void visualiseMatches(const vector<String>& img_names, const vector<ImageFeatures>& features, const vector<MatchesInfo>& matches, double work_scale, int N){
    int i=0,k=0;
    while(k < N && i < matches.size()){
        if(matches[i].src_img_idx < 0 || matches[i].dst_img_idx < 0){ //empty match
            i++;
            continue;
        }
        if(matches[i].src_img_idx >= matches[i].dst_img_idx){ // non-unique matches
            i++;
            continue;
        }

        int src = matches[i].src_img_idx;
        int dst = matches[i].dst_img_idx;

        Mat img_source_full = imread(img_names[src]);
        Mat img_dst_full = imread(img_names[dst]);
        Mat img_source, img_dst;
        resize(img_source_full, img_source, Size(), work_scale, work_scale);
        resize(img_dst_full, img_dst, Size(), work_scale, work_scale);

        img_source_full.release();
        img_dst_full.release();

        Mat img_matches;
        vector<cv::DMatch> first_matches = vector<cv::DMatch>(matches[i].matches.begin(), matches[i].matches.begin() + 20);
        cv::drawMatches(img_source, features[src].keypoints, img_dst, features[dst].keypoints, first_matches, img_matches,
                Scalar::all(-1), Scalar::all(-1), vector<char>()
                //,cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
                );
        //cv::drawMatches(images[i], features[i].keypoints, images[i+1], features[i+1].keypoints, matches[i].matches, img_matches);
        
        img_source.release();
        img_dst.release();

        cv::imshow("Display window", img_matches);
        cv::waitKey(0);

        std::ostringstream fn;
        fn << "feature_match_" << src << "_" << dst << ".png";
        imwrite(fn.str(), img_matches);
        
        k++; i++;
    }
}
