#include "data/depth_data.h"

#include <blackboard/ValueUpdate.h>
#include <blackboard/blackboard.h>

#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv)
{

    bb::Serializer* s = new DepthDataSerializer;

    blackboard::ValueUpdate msg;

    {
        DepthData d;
        cv::Mat rgb;

        cv::Mat depth(480, 640, CV_32FC1, 0.5);
        for(unsigned int i = 0; i < depth.cols * depth.rows; ++i)
        {
            if (i % 10 == 0)
            {
                depth.at<float>(i) = 9;
            }
        }

        geo::DepthCamera cam_model;
        d.image = rgbd::Image(rgb, depth, cam_model, "frame", 0);
        d.sensor_pose.t = geo::Vector3(1, 2, 3);
        bb::Variant v = d;

        bb::ROSWBytes bytes(msg);
        s->serialize(v, bytes);
    }

    std::cout << "message size: " << msg.data.size() << std::endl;

    {
        bb::Variant v;
        s->deserialize(bb::ROSRBytes(msg), v);
        const DepthData& d = v.getValue<DepthData>();
        std::cout << d.sensor_pose.t << std::endl;

        cv::imshow("depth", d.image.getDepthImage() / 10);
        cv::waitKey();
    }

    return 0;
}
