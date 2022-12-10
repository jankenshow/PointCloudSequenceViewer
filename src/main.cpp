#include <boost/program_options.hpp>

#include "sequence_viewer.h"

int main(int argc, char *argv[])
{
    namespace bops = boost::program_options;

    bops::options_description description("options");
    description.add_options()
        ("help,h", "show help")
        ("pcd_path,",
        bops::value<std::string>(),
        "path to pcd file or directory in which pcd files exist")
        ("annotation_path,",
        bops::value<std::string>(),
        "path to directory in which annotation files exist");;

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
    bool flag_annot;
    if (vm.count("annotation_path"))
    {
        annot_path = vm["annotation_path"].as<std::string>();
        flag_annot = true;
    }
    else 
    {
        annot_path = "";
        flag_annot = false;
    }

    SequenceViewer viewer(pcd_path, flag_annot, annot_path);
    int res;
    res = viewer.run();
    return res;
}