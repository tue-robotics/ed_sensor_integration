#include "ed_sensor_integration/kinect/segmodules/sam_seg_module.h"
#include "segmentation.h"
#include "detection.h"

std::vector<cv::Mat> DetectTest(const cv::Mat& img)
{


    ////////////////////////// YOLO //////////////////////////////////////
    std::unique_ptr<YOLO_V8> yoloDetector;
    DL_INIT_PARAM params;
    std::tie(yoloDetector, params) = Initialize();
    ////////////////////////// SAM //////////////////////////////////////

    std::vector<std::unique_ptr<SAM>> samSegmentors;
    SEG::DL_INIT_PARAM params_encoder;
    SEG::DL_INIT_PARAM params_decoder;
    std::vector<SEG::DL_RESULT> resSam;
    SEG::DL_RESULT res;
    std::tie(samSegmentors, params_encoder, params_decoder, res, resSam) = Initializer();


    ////////////////////////// YOLO //////////////////////////////////////
    std::vector<DL_RESULT> resYolo;
    resYolo = Detector(yoloDetector, img);

    ////////////////////////// SAM //////////////////////////////////////
    // Make sure we have at least one result

    // for (const auto& result : resYolo) {
    //     if (result.confidence < 0.5) {
    //         std::cout << "Confidence is too low: " << result.confidence << std::endl;
    //         std::cout << "Class ID is: " << yoloDetector->classes[result.classId] << std::endl;
    //         continue;
    //     }
    //     std::cout << "Confidence is OKOK: " << result.confidence << std::endl;
    //         std::cout << "Class ID is: " << yoloDetector->classes[result.classId] << std::endl;
    //     res.boxes.push_back(result.box);
    // }

    for (const auto& result : resYolo) {
        if (result.classId == 60) {
            std::cout << "Class ID is: " << yoloDetector->classes[result.classId] << " So we dont append"<< std::endl;
            continue;
        }
        res.boxes.push_back(result.box);
        std::cout << "Confidence is OKOK: " << result.confidence << std::endl;
        std::cout << "Class is: " << yoloDetector->classes[result.classId] << std::endl;
        std::cout << "Class ID is: " << result.classId << std::endl;
    }

    SegmentAnything(samSegmentors, params_encoder, params_decoder, img, resSam, res);

    return std::move(res.masks);
    }
