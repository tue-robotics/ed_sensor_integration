#include <ros/ros.h>

// Services
#include <ed_sensor_integration_msgs/GetImage.h>

// Write to file
#include <fstream>

// Read RGBD
#include <tue/serialization/input_archive.h>
#include <tue/serialization/conversions.h>
#include <rgbd/serialization.h>

// Time
//#include <stdio.h>
#include <time.h>

// Visualization
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>


// YOLO functions
std::vector<std::string> load_class_list()
{
    std::vector<std::string> class_list;
    std::ifstream ifs("Yolo_config/classes.txt");
    std::string line;
    while (getline(ifs, line))
    {
        class_list.push_back(line);
    }
    return class_list;
}

void load_net(cv::dnn::Net &net, bool is_cuda)
{
    std::cout << "reading net" << std::endl;
    auto result = cv::dnn::readNetFromONNX("/home/amigo/Yolo_config/yolov5s.onnx");
    std::cout << "net loaded" << std::endl;
    if (is_cuda)
    {
        std::cout << "Attempty to use CUDA\n";
        result.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        result.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);
    }
    else
    {
        std::cout << "Running on CPU\n";
        result.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        result.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }
    net = result;
}

const std::vector<cv::Scalar> colors = {cv::Scalar(255, 255, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 255, 255), cv::Scalar(255, 0, 0)};

const float INPUT_WIDTH = 640.0;
const float INPUT_HEIGHT = 640.0;
const float SCORE_THRESHOLD = 0.2;
const float NMS_THRESHOLD = 0.4;
const float CONFIDENCE_THRESHOLD = 0.4;

struct Detection
{
    int class_id;
    float confidence;
    cv::Rect box;
};

cv::Mat format_yolov5(const cv::Mat &source) {
    int col = source.cols;
    int row = source.rows;
    int _max = MAX(col, row);
    cv::Mat result = cv::Mat::zeros(_max, _max, CV_8UC3);
    source.copyTo(result(cv::Rect(0, 0, col, row)));
    return result;
}

//YOLO object detection
void detect(cv::Mat &image, cv::dnn::Net &net, std::vector<Detection> &output, const std::vector<std::string> &className) {
    cv::Mat blob;

    auto input_image = format_yolov5(image);
    
    cv::dnn::blobFromImage(input_image, blob, 1./255., cv::Size(INPUT_WIDTH, INPUT_HEIGHT), cv::Scalar(), true, false);
    net.setInput(blob);
    std::vector<cv::Mat> outputs;
    net.forward(outputs, net.getUnconnectedOutLayersNames());

    float x_factor = input_image.cols / INPUT_WIDTH;
    float y_factor = input_image.rows / INPUT_HEIGHT;
    
    float *data = (float *)outputs[0].data;

    const int dimensions = 85;
    const int rows = 25200;
    
    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    for (int i = 0; i < rows; ++i) {

        float confidence = data[4];
        if (confidence >= CONFIDENCE_THRESHOLD) {

            float * classes_scores = data + 5;
            cv::Mat scores(1, className.size(), CV_32FC1, classes_scores);
            cv::Point class_id;
            double max_class_score;
            minMaxLoc(scores, 0, &max_class_score, 0, &class_id);
            if (max_class_score > SCORE_THRESHOLD) {

                confidences.push_back(confidence);

                class_ids.push_back(class_id.x);

                float x = data[0];
                float y = data[1];
                float w = data[2];
                float h = data[3];
                int left = int((x - 0.5 * w) * x_factor);
                int top = int((y - 0.5 * h) * y_factor);
                int width = int(w * x_factor);
                int height = int(h * y_factor);
                boxes.push_back(cv::Rect(left, top, width, height));
            }

        }

        data += dimensions;

    }

    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD, nms_result);
    for (uint i = 0; i < nms_result.size(); i++) {
        int idx = nms_result[i];
        Detection result;
        result.class_id = class_ids[idx];
        result.confidence = confidences[idx];
        result.box = boxes[idx];
        output.push_back(result);
    }
}




// ----------------------------------------------------------------------------------------------------

// Get current date/time, format is YYYY-MM-DD-HH-mm-ss
std::string currentDateTime()
{
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);

    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d-%H-%M-%S", &tstruct);

    return buf;
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Initialize
    ros::init(argc, argv, "ed_yolo_viewer");

    ros::NodeHandle nh;
    ros::ServiceClient cl_get_image = nh.serviceClient<ed_sensor_integration_msgs::GetImage>("ed/kinect/get_image");

    rgbd::Image image;
    ed_sensor_integration_msgs::GetImage srv;
    
    std::vector<std::string> class_list = load_class_list();
    cv::Mat frame;
    
    cv::dnn::Net net;
    load_net(net, false);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Start loop
    
    while (ros::ok())
    {
        // - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Retrieve image

        // Try to retrieve the current kinect image
        srv.request.filename = currentDateTime();
        if (!cl_get_image.call(srv))
        {
            std::cout << "Could not call service '" << cl_get_image.getService() << "'." << std::endl;
            ros::Duration(1.0).sleep();
            continue;
        }
    
        // - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Deserialize rgbd image

        std::stringstream stream;
        tue::serialization::convert(srv.response.rgbd_data, stream);
        tue::serialization::InputArchive a_in(stream);
        rgbd::deserialize(a_in, image);

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Show rgbd image

        frame = image.getRGBImage().clone();

        if (frame.cols == 0 || frame.rows == 0)
        {
//            std::cout << "Image size 0, 0, skipping" << std::endl;
            continue;
        }
        // Run YOLO
        std::vector<Detection> output;
        detect(frame, net, output, class_list);

        int detections = output.size();
        

        // Draw bounding boxes
        for (int i = 0; i < detections; ++i)
        {

            auto detection = output[i];
            auto box = detection.box;
            auto classId = detection.class_id;
            const auto color = colors[classId % colors.size()];
            cv::rectangle(frame, box, color, 3);

            cv::rectangle(frame, cv::Point(box.x, box.y - 20), cv::Point(box.x + box.width, box.y), color, cv::FILLED);
            cv::putText(frame, class_list[classId].c_str(), cv::Point(box.x, box.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
        }

        cv::imshow("Output", frame);
        
    }

    return 0;
}
