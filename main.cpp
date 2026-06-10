#include "rosbag2nuscenes/Bag2Scenes.hpp"
#include <iostream>
#include <cstdlib>

int main(int argc, char** argv) {
    if (argc != 5) {
        std::cerr << "Usage: " << argv[0]
                  << " <rosbag_directory> <parameter_file> <output_directory> <num_workers>"
                  << std::endl;
        if (argc > 1) {
            std::cerr << "Got:";
            for (int i = 0; i < argc; i++) {
                std::cerr << " " << argv[i];
            }
            std::cerr << std::endl;
        }
        return EXIT_FAILURE;
    }

    int num_workers = 0;
    try {
        num_workers = std::stoi(argv[4]);
    } catch (const std::exception& e) {
        std::cerr << "Error: num_workers must be an integer, got '" << argv[4] << "'" << std::endl;
        return EXIT_FAILURE;
    }

    if (num_workers < 1) {
        std::cerr << "Error: num_workers must be >= 1, got " << num_workers << std::endl;
        return EXIT_FAILURE;
    }

    try {
        Bag2Scenes converter(argv[1], argv[2], argv[3], num_workers);
        converter.writeScene();
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
