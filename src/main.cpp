#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include "sequence_viewer.h"

int main(int argc, char *argv[])
{
    namespace bops = boost::program_options;
    namespace bfs = boost::filesystem;

    bops::options_description description("options");
    description.add_options()
        ("help,h", "show help")
        ("pcd_path,",
        bops::value<std::string>(),
        "path to pcd file or directory in which pcd files exist")
        ("annotation_path,",
        bops::value<std::string>(),
        "path to directory in which annotation files exist")
        ("cameraparam_path,",
        bops::value<std::string>(),
        "path to camera pose parameters of view point")
        ("cameraparam_save_path,",
        bops::value<std::string>(),
        "where to save camera pose parameters of view point, default : [current_directory]/cameraparam.cam");

    bops::variables_map vm;
    try
    {
        bops::store(bops::parse_command_line(argc, argv, description), vm);
    }
    catch (const bops::error_with_no_option_name &e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    bops::notify(vm);

    std::string pcd_path;
    if (vm.count("help"))
    {
        std::cout << description << std::endl;
        return 0;
    }
    else if (vm.count("pcd_path"))
    {
        try
        {
            pcd_path = vm["pcd_path"].as<std::string>();
        }
        catch (const boost::bad_any_cast &e)
        {
            std::cerr << e.what() << std::endl;
            return 1;
        }
    }
    else
    {
        std::cout << "usage:\n"
                  << argv[0] << "--pcd_path [path/to/pcd_file]" << std::endl;
        std::cout << description << std::endl;
    }

    std::string annot_path;
    if (vm.count("annotation_path"))
    {
        annot_path = vm["annotation_path"].as<std::string>();
    } else {
        annot_path = "";
    }

    std::string cameraparam_path;
    if (vm.count("cameraparam_path"))
    {
        cameraparam_path = vm["cameraparam_path"].as<std::string>();
    } else {
        cameraparam_path = "";
    }

    std::string cameraparam_save_path;
    if (vm.count("cameraparam_save_path"))
    {
        cameraparam_save_path = vm["cameraparam_save_path"].as<std::string>();

        bfs::path bfs_p(cameraparam_save_path);

        if (cameraparam_save_path.empty() or !bfs::exists(bfs_p))
        {
            std::string message = (
                boost::format("An argument 'cameraparam_save_path : %1%' is empty or doesn't exist!") % cameraparam_save_path).str();
            throw std::runtime_error(message);
        } else if (bfs::is_directory(bfs_p))
        {
            cameraparam_save_path = (bfs_p / "camerapose.cam").string();
        }
    } else {
        cameraparam_save_path = "camerapose.cam";
    }

    SequenceViewer viewer(pcd_path, annot_path, cameraparam_path, cameraparam_save_path);
    int res;
    res = viewer.run();
    return res;
}