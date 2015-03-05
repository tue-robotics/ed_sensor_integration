#include "visualization.h"

#include <ed/world_model.h>
#include <ed/entity.h>
#include <ed/measurement.h>

#include <rgbd/Image.h>

#include <tue/config/reader.h>

// ----------------------------------------------------------------------------------------------------

int getHash(const std::string& id)
{
    const char* str = id.c_str();

//    unsigned long hash = 5381;
    int hash = 5381;
    int c;

    while ((c = *str++))
        hash = ((hash << 5) + hash) + c; /* hash * 33 + c */

    return hash;// % max_val;
}


// ----------------------------------------------------------------------------------------------------

void getColor(unsigned int id, float& r, float& g, float& b)
{
    static float colors[8][3] = { {1,0,0} , {0,1,0} , {0,0,1} , {1,1,0} , {1,0,1} , {0,1,1} , {1,1,1} , {0,0,0} };
    unsigned int color_id = id % 8;

    //float red_id = (id % 100)/100.0;
    //float green_id = ((id/2) % 100)/100.0;
    //float blue_id = ((id/3) % 100)/100.0;

    //c.r = red_id;
    //c.g = green_id;
    //c.b = blue_id;

    r = colors[color_id][0];
    g = colors[color_id][1];
    b = colors[color_id][2];
}

// ----------------------------------------------------------------------------------------------------

void drawImageWithMeasurements(const ed::WorldModel& world_model, rgbd::ImageConstPtr rgbd_image, cv::Mat& color_img)
{
    color_img = rgbd_image->getRGBImage().clone() * 0.2;
    for (ed::WorldModel::const_iterator it = world_model.begin(); it != world_model.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;

        if (e && !e->shape()) //! if it has no shape
        {
            if (e->lastMeasurement())
            {
                ed::MeasurementConstPtr m = e->lastMeasurement();

                if (m->timestamp() == rgbd_image->getTimestamp() && e->measurementSeq() > 5)
                {
                    std::vector<cv::Point2i> pnts;
                    for (ed::ImageMask::const_iterator mit = m->imageMask().begin(color_img.cols); mit != m->imageMask().end(); ++mit)
                    {
                        color_img.at<cv::Vec3b>(*mit) = rgbd_image->getRGBImage().at<cv::Vec3b>(*mit);
                        pnts.push_back(*mit);
                    }

                    // get the bounding rectangle of the mask
                    cv::Rect bounding_rect = cv::boundingRect(pnts);

                    // calculate color components
                    float r, g, b;
                    getColor(getHash(e->id().str()), r, g, b);
                    int red = r * 255;
                    int green = g * 255;
                    int blue = b * 255;

                    // create Scalar color with a minimum
                    cv::Scalar color(std::max(red, 80), std::max(green, 80), std::max(blue, 80));

                    // draw bounding box rectangle
                    cv::rectangle(color_img, bounding_rect, color, 2);

                    //                    std::vector<cv::Point2i> chull;
                    //                    cv::convexHull(pnts,chull);
                    //                    std::vector<std::vector<cv::Point2i> > contours; contours.push_back(chull);
                    //                    // draw convex hull contours
                    //                    cv::drawContours(color_img,contours,0,color, 1);

                    tue::config::Reader config(e->data());
                    std::string type;
                    std::string info ;//= e->id().substr(0,4);
                    float score = 0;

                    // update type given by perception modules type and certainty
                    if (config.readGroup("perception_result", tue::config::OPTIONAL))
                    {
                        if (config.readGroup("type_aggregator", tue::config::OPTIONAL))
                        {
                            if (config.value("type", type, tue::config::OPTIONAL) &&
                                    config.value("score", score, tue::config::OPTIONAL)){
                                info = boost::str(boost::format("%.2f") % score);
                            }
                        }
                        config.endGroup(); // close type_aggregator group
                    }
                    config.endGroup();  // close perception_result group

                    // if no type was read, use the default and the UID
                    if (type.empty()){
                        type = e->type();
                        info = e->id().str().substr(0,4);
                    }else if (type.compare("unknown") == 0){
                        type = "";
                        info = e->id().str().substr(0,4);
                    }else if (type.compare("human") == 0){
                        // in case type is human, replace by name
                        if (config.readGroup("perception_result", tue::config::OPTIONAL)){
                            if (config.readGroup("face_recognizer", tue::config::OPTIONAL))
                            {
                                std::string person_name;
                                if (config.value("label", person_name, tue::config::OPTIONAL) &&
                                        config.value("score", score, tue::config::OPTIONAL)){
                                    if (!person_name.empty() && score > 0){
                                        type = person_name;
                                        info = boost::str(boost::format("%.2f") % score);
                                    }
                                }
                            }
                            config.endGroup(); // close type_aggregator group
                        }
                        config.endGroup();  // close perception_result group
                    }

                    // draw name background rectangle
                    cv::rectangle(color_img, cv::Point(bounding_rect.x, bounding_rect.y) + cv::Point(0, -22),
                                  cv::Point(bounding_rect.x, bounding_rect.y) + cv::Point(((type.size() + 6) * 10), -2),
                                  color - cv::Scalar(140, 140, 140), CV_FILLED);

                    // draw name and ID
                    cv::putText(color_img, type + "(" + info + ")",
                                cv::Point(bounding_rect.x, bounding_rect.y) + cv::Point(5, -8),
                                1, 1.0, color, 1, CV_AA);
                }
            }
        }
    }
}
