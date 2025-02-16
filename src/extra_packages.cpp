#include "phntm_bridge/extra_packages.hpp"
#include "phntm_bridge/const.hpp"
#include "phntm_bridge/lib.hpp"
#include <cstdlib>
#include <iostream>
#include <filesystem>
#include <fstream>

#include <yaml-cpp/yaml.h>
#include "tinyxml2.h"

bool runCmd(std::string cmd) {
    std::array<char, 128> buffer;
    std::string err_out = "/tmp/phntm_pkg_cmd_err.txt";
    cmd += " 2>" + err_out;
    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) {
        std::cout << RED << "     " << "Error calling " << cmd << CLR << std::endl;
        return false;
    }
    while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
        auto res = trim(std::string(buffer.data()));
        if (!res.empty()) {
            std::cout << GRAY << "     " << res << CLR << std::endl;
        }
    }
    pclose(pipe);

    bool err = false;
    FILE* pipe_err = fopen(err_out.c_str(), "r");
    if (pipe_err) {
        while (fgets(buffer.data(), buffer.size(), pipe_err) != nullptr) {
            auto res = trim(std::string(buffer.data()));
            if (!res.empty()) {
                std::cout << RED << "     " << res << CLR << std::endl;
            }
            err = true;
        }
        fclose(pipe_err);
        std::filesystem::remove(err_out);
    }

    return err ? false : true;
}

bool checkPackage(std::string extra_pkg) {

    auto ros_distro = std::getenv("ROS_DISTRO");
    std::string cmd;

    if (extra_pkg[0] == '/') { // dir

        std::cout << "  => directory, checking package.xml" << std::endl;
        auto pkg_xml_path = std::filesystem::path(extra_pkg) /  "package.xml";

        if (!std::filesystem::exists(pkg_xml_path)) {
            std::cout << RED << "     " << pkg_xml_path << " not found, skipping" << CLR << std::endl;
            return false;
        }

        tinyxml2::XMLDocument xml_tree;
        xml_tree.LoadFile(pkg_xml_path.c_str());
        
        std::cout << "     " << "parsed " << pkg_xml_path.c_str() << std::endl;

        // auto xml_root = xml_tree.getroot()
        tinyxml2::XMLElement* xml_root = xml_tree.FirstChildElement("package");
        if (!xml_root) {
            std::cout << RED << "     " << "XML root not found, skipping" << CLR << std::endl;
            return false;
        }
        tinyxml2::XMLElement* xml_export = xml_root->FirstChildElement("export");
        if (!xml_export) {
            std::cout << RED << "     " << "package/export not found in XML, skipping" << CLR << std::endl;
            return false;
        }
        tinyxml2::XMLElement* xml_build_type = xml_export->FirstChildElement("build_type");
        if (!xml_build_type) {
            std::cout << RED << "     " << "package/export/build_type not found in XML, skipping" << CLR << std::endl;
            return false;
        }
        tinyxml2::XMLElement* xml_pkg_name = xml_root->FirstChildElement("name");
        if (!xml_pkg_name) {
            std::cout << RED << "     " << "package/pkg_name not found in XML, skipping" << CLR << std::endl;
            return false;
        }
        std::cout << "     " << "name: " << xml_pkg_name->GetText() << std::endl;
        std::cout << "     " << "build_type: " << xml_build_type->GetText() << std::endl;

        std::cout << "     " << "installing deps..." << std::endl;
        if (!runCmd("/usr/bin/rosdep install -i --from-path " + extra_pkg + " --rosdistro "+ ros_distro + " -y"))
            return false;
        
        std::cout << "     " << "building..." << std::endl;
        if (!runCmd("/usr/bin/colcon build --symlink-install --base-path " + extra_pkg))
            return false;
        
        std::cout << "     " << "Done." << std::endl;
        return true;

    } else { // package name
        
        auto pkg_name = "ros-" + std::string(ros_distro) + "-" + replace(extra_pkg, "_", "-");

        std::cout << "  => package, installing " << pkg_name << std::endl;

        if (!runCmd("/usr/bin/apt-get install -y " + pkg_name))
            return false;
        
        std::cout << "     " << "Done." << std::endl;
        return true;
    }
}

bool installExtraPackages() {

    auto checked_packages_file_path = "/.checked_packages.yaml";
    auto config_path = "/ros2_ws/phntm_bridge_params.yaml";

    YAML::Node checked_packages_log; // don't re-process, unless forced in config
    if (std::filesystem::exists(checked_packages_file_path)) {
        checked_packages_log = YAML::LoadFile(checked_packages_file_path); 
    }

    if (!std::filesystem::exists(config_path)) {
        std::cerr << RED << "Config file not found at " << config_path << CLR << std::endl;  
        return false;
    }

    YAML::Node node_config = YAML::LoadFile(config_path);
    auto extra_packages = node_config["/**"]["ros__parameters"]["extra_packages"];
    if (!extra_packages.IsDefined() || !extra_packages.IsSequence()) {
        return false; //nothing to do
    }
    
    std::cout << MAGENTA << "Checking extra packages..." << CLR << std::endl;

    for (const auto& item : extra_packages) {

        auto extra_pkg = item.as<std::string>();
        bool force = false;

        if (extra_pkg.find(':') != std::string::npos) {
            auto parts = split(extra_pkg, ':');
            extra_pkg = parts[0];
            if (parts.size() > 1 && parts[1] == "rebuild") {
                force = true;
            }
        }

        bool checked_before = false;
        if (checked_packages_log["packages"].IsSequence()) {
            for (const auto& one : checked_packages_log["packages"]) {
                if (one.as<std::string>() == extra_pkg) {
                    checked_before = true;
                    break;
                }   
            }
        }

        if (checked_before && !force) {
            std::cout << "Skipping package: '" << extra_pkg << "' (checked before)" << std::endl;
            continue;
        }

        if (!checked_packages_log["packages"].IsDefined())
            checked_packages_log["packages"] = YAML::Node(YAML::NodeType::Sequence);

        std::cout << YELLOW << "Checking package" << (force ? " (FORCE)" : "") << ": '" << extra_pkg << "'..." << CLR << std::endl;
        
        auto changed = checkPackage(extra_pkg);

        if (!checked_before)
            checked_packages_log["packages"].push_back(extra_pkg);
    }

    if (checked_packages_log["packages"].IsDefined()) {
        std::ofstream fout(checked_packages_file_path);
        fout << checked_packages_log;
        fout.close();
    }
    
    return true; // restart
}