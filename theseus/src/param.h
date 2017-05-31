#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "theseus.h"
class Param{
public:
    Param();
    int parseCmdArgs(int argc, char* argv[]);
    void printUsage(char* argv[]);
bool getBoolParam(string key) const;
    int getIntParam(string key) const;
    string getStringParam(string key) const;
    double getDoubleParam(string key) const;

    const std::vector<String>& getImageNames() const;

private:
    // Default command line args
    vector<String> img_names;

    int adjacent_matches = 3;
    double work_megapix = 0.4;
    double seam_megapix = 0.08;
    double conf_thresh = 1.f;
    string features_type = "surf";
    string ba_cost_func = "ray";
    string ba_refine_mask = "xxxxx";
    bool do_wave_correct = true;
    WaveCorrectKind wave_correct = detail::WAVE_CORRECT_HORIZ;
    string warp_type = "cylindrical";
    int expos_comp_type = ExposureCompensator::GAIN;
    double match_conf = 0.3f;
    string seam_find_type = "gc_color";
    int blend_type = Blender::MULTI_BAND;
    double blend_strength = 5;
    string result_name = "result.png";
    int range_width = -1;
    int max_iter = 20;
    bool crop = false;
    bool show_matches = false;

    void createBoolParamMap();
    void createIntParamMap();
    void createDoubleParamMap();
    void createStringParamMap();

    map<string, bool> bool_param_map;
    map<string, int> int_param_map;
    map<string, double> double_param_map;
    map<string, string> string_param_map;
};

#endif
