#ifndef ED_SENSOR_INTEGRATION_TOOLS_SNAPSHOT_H_
#define ED_SENSOR_INTEGRATION_TOOLS_SNAPSHOT_H_

#include <ed/models/model_loader.h>
#include <ed/world_model.h>
#include <ed/update_request.h>
#include <ed/serialization/serialization.h>
#include <ed/io/json_reader.h>

#include <tue/filesystem/crawler.h>

#include <tue/config/read.h>
#include <tue/config/reader.h>
#include <tue/config/data_pointer.h>

#include <rgbd/image.h>
#include <rgbd/serialization.h>

#include <vector>

namespace ed_sensor_integration
{

struct Snapshot
{
    rgbd::ImagePtr image;
    geo::Pose3D sensor_pose;
};

bool readImage(const std::string& filename, rgbd::ImagePtr& image, geo::Pose3D& sensor_pose)
{
    tue::config::DataPointer meta_data;

    try
    {
        meta_data = tue::config::fromFile(filename);
    }
    catch (tue::config::ParseException& e)
    {
        std::cerr << "Could not open '" << filename << "'.\n\n" << e.what() << std::endl;
        return false;
    }

    tue::config::Reader r(meta_data);

    // Read image
    std::string rgbd_filename;
    if (r.value("rgbd_filename", rgbd_filename))
    {
        tue::filesystem::Path abs_rgbd_filename = tue::filesystem::Path(filename).parentPath().join(rgbd_filename);

        std::ifstream f_rgbd;
        f_rgbd.open(abs_rgbd_filename.string().c_str(), std::ifstream::binary);

        if (!f_rgbd.is_open())
        {
            std::cerr << "Could not open '" << filename << "'." << std::endl;
            return false;
        }

        image.reset(new rgbd::Image);

        tue::serialization::InputArchive a_in(f_rgbd);
        rgbd::deserialize(a_in, *image);
    }

    // Read sensor pose
    if (!ed::deserialize(r, "sensor_pose", sensor_pose))
    {
        std::cerr << "No field 'sensor_pose' specified." << std::endl;
        return false;
    }

    return true;
}

bool loadWorldModel(const std::string& model_name, ed::WorldModel& world_model)
{
    ed::UpdateRequest req;

    ed::models::ModelLoader model_loader;

    std::stringstream error;
    if (!model_loader.create("_root", model_name, req, error, true))
    {
        std::cerr << "Model '" << model_name << "' could not be loaded:" << std::endl << std::endl;
        std::cerr << error.str() << std::endl;
        return false;
    }

    // Reset world
    world_model = ed::WorldModel();

    // Update world
    world_model.update(req);

    return true;
}

class SnapshotCrawler
{

public:

    SnapshotCrawler(tue::filesystem::Path path)
    {
        if (path.isDirectory())
            crawler.setRootPath(path);
        else
            crawler.setRootPath(path.parentPath());

        // load first snapshot
        if (path.isRegularFile())
            loadSnapshot(path);
        else
            loadNewSnapshot();
    }

    inline Snapshot& current() { return snapshots[i_current]; } //TODO pass by reference
    inline Snapshot& getSnapshot(uint i) { return snapshots[i]; } //TODO replace with overloading of indexing notation []

    void previous()
    {
        if (i_current >0)
            --i_current;
    }

    void next()
    {
        ++i_current;

        //load new image if size of vector is exceeded
        if (i_current >= snapshots.size())
        {
            if (!loadNewSnapshot())
            {
                i_current = snapshots.size() - 1;
            }
        }
    }

    bool loadNewSnapshot()
    {
        bool file_found = false;
        tue::filesystem::Path filename;

        while (crawler.nextPath(filename))
        {
            if (filename.extension() == ".json")
            {
                file_found = true;
                break;
            }
        }

        if (!file_found)
            return false;


        if (!loadSnapshot(filename))
            return false;

        return true;
    }

    bool loadSnapshot(tue::filesystem::Path filename)
    {
        i_current = snapshots.size();
        snapshots.push_back(ed_sensor_integration::Snapshot());
        ed_sensor_integration::Snapshot& snapshot = snapshots.back();

        std::cout << "loading " << filename << std::endl;
        if (!readImage(filename.string(), snapshot.image, snapshot.sensor_pose))
        {
            std::cerr << "Could not read " << filename << std::endl;
            snapshots.pop_back();
            return false;
        }
        return true;
    }

    uint i_current;
    std::vector<Snapshot> snapshots;

    tue::filesystem::Crawler crawler;
};


}
#endif // ED_SENSOR_INTEGRATION_TOOLS_SNAPSHOT_H_
