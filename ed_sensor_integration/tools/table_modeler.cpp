#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char **argv) {

    if (argc < 2)
        {
            std::cout << "Usage:\n\n   table_modeler FILENAME1 FILENAME2 ...\n\n";
            return 1;
        }

        pcl::PointCloud<pcl::PointXYZRGB> inputs [argc-1];

        for (int i = 1; i < argc; ++i)
        {
            std::string name = std::string(argv[i]);

            // open pcd file
            if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (name, inputs[i-1]) == -1) //* load the file
            {
                //PCL_ERROR ("Couldn't read file" + name + "\n");
            }
            else
            {

            // read json metadata


            // align to world model coordinates


            // align to previous file

            }
        }


    return 0;
}
