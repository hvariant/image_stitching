#include "param.h"

Param::Param(){
    createBoolParamMap();
    createIntParamMap();
    createStringParamMap();
    createDoubleParamMap();
}

void Param::printUsage(char* argv[])
{
    cout <<
        "Rotation model images stitcher.\n\n"
        << argv[0] <<
        " img1 img2 [...imgN] [flags]\n\n"
        "Flags:\n"
        "\nMotion Estimation Flags:\n"
        "  --work_megapix <float>\n"
        "      Resolution for image registration step. The default is 0.6 Mpx.\n"
        "  --features (surf|orb)\n"
        "      Type of features used for images matching. The default is surf.\n"
        "  --adjacent_matches <k>\n"
        "      the number of adjacent images to do feature matching with, default is 3\n"
        "  --match_conf <float>\n"
        "      Confidence for feature matching step. The default is 0.65 for surf and 0.3 for orb.\n"
        "  --show_matches yes|no\n"
        "      Enable the program to show feature matches between images\n"
        "  --conf_thresh <float>\n"
        "      Threshold for two images are from the same panorama confidence.\n"
        "      The default is 1.0.\n"
        "  --ba (no|reproj|ray)\n"
        "      Bundle adjustment cost function. The default is ray.\n"
        "  --ba_refine_mask (mask)\n"
        "      Set refinement mask for bundle adjustment. It looks like 'x_xxx',\n"
        "      where 'x' means refine respective parameter and '_' means don't\n"
        "      refine one, and has the following format:\n"
        "      <fx><skew><ppx><aspect><ppy>. The default mask is 'xxxxx'.\n"
        "      the respective flag is ignored.\n"
        "      Ignored by ray bundle adjustment.\n"
        "  --max_iter <n>\n"
        "      the maximum number of iterations for bundle adjustment\n"
        "  --wave_correct (no|yes)\n"
        "      Perform wave effect correction.\n"
        "\nCompositing Flags:\n"
        "  --warp (affine|plane|cylindrical|spherical|stereographic)\n"
        "      Warp surface type. The default is 'spherical'.\n"
        "  --seam_megapix <float>\n"
        "      Resolution for seam estimation step. The default is 0.1 Mpx.\n"
        "  --seam (no|voronoi|gc_color|gc_colorgrad)\n"
        "      Seam estimation method. The default is 'gc_color'.\n"
        "  --expos_comp (no|gain|gain_blocks)\n"
        "      Exposure compensation method. The default is 'gain'.\n"
        "  --blend (no|multiband)\n"
        "      Blending method. The default is 'multiband'.\n"
        "  --blend_strength <float>\n"
        "      Blending strength from [0,100] range. The default is 5.\n"
        "  --crop yes|no\n"
        "      Enable to allow the program to crop the output. The default is yes.\n"
        "  --output <result_img>\n"
        "      Name of output image. The default is 'result.png'.\n";
}

int Param::parseCmdArgs(int argc, char** argv)
{
    if (argc == 1)
    {
        printUsage(argv);
        return -1;
    }
    for (int i = 1; i < argc; ++i)
    {
        if (string(argv[i]) == "--help" || string(argv[i]) == "/?")
        {
            printUsage(argv);
            return -1;
        }
        else if (string(argv[i]) == "--work_megapix")
        {
            work_megapix = atof(argv[i + 1]);
            i++;
        }
        else if (string(argv[i]) == "--seam_megapix")
        {
            seam_megapix = atof(argv[i + 1]);
            i++;
        }
        else if (string(argv[i]) == "--result")
        {
            result_name = argv[i + 1];
            i++;
        }
        else if (string(argv[i]) == "--features")
        {
            features_type = argv[i + 1];
            if (features_type == "orb")
                match_conf = 0.3f;
            i++;
        }
        else if (string(argv[i]) == "--match_conf")
        {
            match_conf = static_cast<float>(atof(argv[i + 1]));
            i++;
        }
        else if (string(argv[i]) == "--conf_thresh")
        {
            conf_thresh = static_cast<float>(atof(argv[i + 1]));
            i++;
        }
        else if (string(argv[i]) == "--max_iter")
        {
            max_iter = static_cast<int>(atoi(argv[i + 1]));
            i++;
        }
        else if (string(argv[i]) == "--ba")
        {
            ba_cost_func = argv[i + 1];
            i++;
        }
        else if(string(argv[i]) == "--crop")
        {
            if (string(argv[i + 1]) == "yes")
                crop = true;
            else
            {
                crop = false;
            }
            i++;
        }
        else if(string(argv[i]) == "--show_matches")
        {
            if (string(argv[i + 1]) == "yes")
                show_matches = true;
            else
            {
                show_matches = false;
            }
            i++;
        }
        else if (string(argv[i]) == "--ba_refine_mask")
        {
            ba_refine_mask = argv[i + 1];
            if (ba_refine_mask.size() != 5)
            {
                cout << "Incorrect refinement mask length.\n";
                return -1;
            }
            i++;
        }
        else if (string(argv[i]) == "--adjacent_matches")
        {
            adjacent_matches = static_cast<int>(atoi(argv[i + 1]));
            i++;
        }
        else if (string(argv[i]) == "--wave_correct")
        {
            if (string(argv[i + 1]) == "no")
                do_wave_correct = false;
            else if (string(argv[i + 1]) == "yes")
            {
                do_wave_correct = true;
                wave_correct = detail::WAVE_CORRECT_HORIZ;
            }
            else
            {
                cout << "Bad --wave_correct flag value\n";
                return -1;
            }
            i++;
        }
        else if (string(argv[i]) == "--warp")
        {
            warp_type = string(argv[i + 1]);
            i++;
        }
        else if (string(argv[i]) == "--expos_comp")
        {
            if (string(argv[i + 1]) == "no")
                expos_comp_type = ExposureCompensator::NO;
            else if (string(argv[i + 1]) == "gain")
                expos_comp_type = ExposureCompensator::GAIN;
            else if (string(argv[i + 1]) == "gain_blocks")
                expos_comp_type = ExposureCompensator::GAIN_BLOCKS;
            else
            {
                cout << "Bad exposure compensation method\n";
                return -1;
            }
            i++;
        }
        else if (string(argv[i]) == "--seam")
        {
            if (string(argv[i + 1]) == "no" ||
                string(argv[i + 1]) == "voronoi" ||
                string(argv[i + 1]) == "gc_color" ||
                string(argv[i + 1]) == "gc_colorgrad" ||
                string(argv[i + 1]) == "dp_color" ||
                string(argv[i + 1]) == "dp_colorgrad")
                seam_find_type = argv[i + 1];
            else
            {
                cout << "Bad seam finding method\n";
                return -1;
            }
            i++;
        }
        else if (string(argv[i]) == "--blend")
        {
            if (string(argv[i + 1]) == "no")
                blend_type = Blender::NO;
            else if (string(argv[i + 1]) == "multiband")
                blend_type = Blender::MULTI_BAND;
            else
            {
                cout << "Bad blending method\n";
                return -1;
            }
            i++;

            LOGLN("blend_type in param.cpp:" << this->blend_type);
        }
        else if (string(argv[i]) == "--blend_strength")
        {
            blend_strength = static_cast<float>(atof(argv[i + 1]));
            i++;
        }
        else if (string(argv[i]) == "--output")
        {
            result_name = argv[i + 1];
            i++;
        }
        else
            img_names.push_back(argv[i]);
    }

    createBoolParamMap();
    createIntParamMap();
    createStringParamMap();
    createDoubleParamMap();

    return 0;
}

#define insertIntoMap(s,type) \
    type##_param_map[#s] = s;

void Param::createBoolParamMap(){
    insertIntoMap(do_wave_correct, bool);
    insertIntoMap(crop, bool);
    insertIntoMap(show_matches, bool);
}

void Param::createIntParamMap(){
    insertIntoMap(blend_type, int);
    insertIntoMap(expos_comp_type, int);
    int_param_map.insert(std::pair<string,int>("wave_correct",static_cast<int>(wave_correct)));
    insertIntoMap(max_iter, int);
    insertIntoMap(adjacent_matches, int);
}

void Param::createDoubleParamMap(){
    insertIntoMap(work_megapix, double);
    insertIntoMap(seam_megapix, double);
    insertIntoMap(conf_thresh, double);
    insertIntoMap(match_conf, double);
    insertIntoMap(blend_strength, double);
}

void Param::createStringParamMap(){
    insertIntoMap(features_type, string);
    insertIntoMap(ba_cost_func, string);
    insertIntoMap(ba_refine_mask, string);
    insertIntoMap(warp_type, string);
    insertIntoMap(seam_find_type, string);
    insertIntoMap(result_name, string);
}

bool Param::getBoolParam(string key) const{
    bool ret = bool_param_map.at(key);
    return ret;
}

int Param::getIntParam(string key) const{
    int ret = int_param_map.at(key);
    return ret;
}

string Param::getStringParam(string key) const{
    string ret = string_param_map.at(key);
    return ret;
}

double Param::getDoubleParam(string key) const{
    double ret = double_param_map.at(key);
    return ret;
}

const std::vector<String>& Param::getImageNames() const{
    return img_names;
}
