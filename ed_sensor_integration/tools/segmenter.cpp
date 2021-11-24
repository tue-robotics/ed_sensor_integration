#include <ed/models/model_loader.h>
#include <ed/world_model.h>
#include <ed/update_request.h>
#include <ed/serialization/serialization.h>
#include <ed/io/json_reader.h>
#include <ed/kinect/updater.h>

#include <tue/filesystem/crawler.h>

#include <rgbd/image.h>
#include <rgbd/serialization.h>
#include <rgbd/view.h>

#include <opencv2/highgui/highgui.hpp>

#include <tue/config/read.h>
#include <tue/config/reader.h>
#include <tue/config/data_pointer.h>

#include <ed/entity.h>

#include <fstream>
#include <vector>

#include "ed_sensor_integration/tools/snapshot.h"

/**
 * @brief visualizeSegmentation paint segmentation result over an image
 * @param[in] snapshot snapshot containing the image that was segmented
 * @param[in] res segmentation result on the image
 * @param[out] canvas canvas to paint to.
 */
void visualizeSegmentation(const ed_sensor_integration::Snapshot& snapshot, const UpdateResult& res, cv::Mat canvas)
{
    cv::Vec3b fill_colour(0, 0, 255); // red
    cv::Scalar outline_colour_1(255, 255, 255); // white
    cv::Scalar outline_colour_2(0, 0, 0); // black

    int depth_width = snapshot.image->getDepthImage().cols;
    double f = (double)canvas.cols / depth_width;

    for(unsigned int i = 0; i < res.entity_updates.size(); ++i)
    {
        const EntityUpdate& e_update = res.entity_updates[i];
        if (e_update.pixel_indices.empty())
            continue;

        unsigned i_pxl = e_update.pixel_indices[0];
        cv::Point bb_min(i_pxl % depth_width, i_pxl / depth_width);
        cv::Point bb_max(i_pxl % depth_width, i_pxl / depth_width);

        for(std::vector<unsigned int>::const_iterator it = e_update.pixel_indices.begin(); it != e_update.pixel_indices.end(); ++it)
        {
            int x = *it % depth_width;
            int y = *it / depth_width;

            bb_min.x = std::min(bb_min.x, x);
            bb_min.y = std::min(bb_min.y, y);
            bb_max.x = std::max(bb_max.x, x);
            bb_max.y = std::max(bb_max.y, y);

            for(double x2 = f * x; x2 < (f * (x + 1)); ++x2)
                for(double y2 = f * y; y2 < (f * (y + 1)); ++y2)
                    canvas.at<cv::Vec3b>(y2, x2) = fill_colour;
        }

        cv::Point d(2, 2);
        cv::rectangle(canvas, f * bb_min, f * bb_max, cv::Scalar(255, 255, 255), 2);
        cv::rectangle(canvas, f * bb_min - d, f * bb_max + d, cv::Scalar(0, 0, 0), 2);
        cv::rectangle(canvas, f * bb_min + d, f * bb_max - d, cv::Scalar(0, 0, 0), 2);
    }
}

/**
 * @brief usage: instruct user how to use the executable
 */
void usage()
{
    std::cout << "Usage: ed_segmenter IMAGE-FILE-OR-DIRECTORY" << std::endl;
}


int main(int argc, char **argv)
{
    if (argc != 2)
    {
        usage();
        return 1;
    }

//    std::string model_name = argv[2];

//    ed::WorldModel world_model;
//    if (!ed_sensor_integration::loadWorldModel(model_name, world_model))
//        return 1;

    tue::filesystem::Path path = argv[1];
    if (!path.exists())
    {
        std::cerr << "Path '" << path << "' does not exist." << std::endl;
        return 1;
    }

    ed_sensor_integration::SnapshotCrawler crawler(path);

    Updater updater;

    std::vector<ed_sensor_integration::Snapshot> snapshots;
    unsigned int i_snapshot = 0;

    while(true)
    {
        ed_sensor_integration::Snapshot& snapshot = crawler.current();

        ed::UpdateRequest update_req;
        UpdateResult res(update_req);

        UpdateRequest kinect_update_request;
        kinect_update_request.area_description = snapshot.area_description;
        updater.update(snapshot.world_model, snapshot.image, snapshot.sensor_pose, kinect_update_request, res);

        std::cout << update_req.measurements.size() << std::endl;

        cv::Mat canvas = snapshot.image->getRGBImage().clone();

        std::string error_msg = res.error.str();
        if (error_msg.empty())
        {
            visualizeSegmentation(snapshot, res, canvas);
        }
        else
        {
            std::cerr << error_msg << std::endl;
            cv::putText(canvas, "Segmentation failed", cv::Point(10, 20), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255, 255, 255), 1);
        }

        cv::imshow("RGB", canvas);
        char key = cv::waitKey();

        if (key == 81)  // Left arrow
        {
            crawler.previous();
        }
        else if (key == 83) // Right arrow
        {
            crawler.next();
        }
        else if (key == 'q')
        {
            break;
        }

//        std::cout << (int)key << std::endl;
    }

    return 0;
}
