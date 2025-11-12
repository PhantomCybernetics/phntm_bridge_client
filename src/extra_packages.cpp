#include "phntm_bridge/extra_packages.hpp"
#include "phntm_bridge/const.hpp"
#include "phntm_bridge/lib.hpp"
#include <cstdlib>
#include <iostream>
#include <filesystem>
#include <fstream>

#include <yaml-cpp/yaml.h>
#include "tinyxml2.h"

namespace phntm {

    bool runCmd(std::string cmd) {
        std::array<char, 128> buffer;
        std::string err_out = "/tmp/phntm_pkg_cmd_err.log";
        cmd = std::string(". /root/ros2_py_venv/bin/activate && ") // must call from inside of the python venv
            + cmd + " 2>" + err_out;
        FILE* pipe = popen(cmd.c_str(), "r");
        if (!pipe) {
            log("     Error calling " + cmd, true);
            return false;
        }
        while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
            auto res = trim(std::string(buffer.data()));
            if (!res.empty()) {
                log(GRAY + "     " + res + CLR);
            }
        }
        auto status = pclose(pipe);

        FILE* pipe_err = fopen(err_out.c_str(), "r");
        if (pipe_err) {
            while (fgets(buffer.data(), buffer.size(), pipe_err) != nullptr) {
                auto res = trim(std::string(buffer.data()));
                if (!res.empty()) {
                    log("     " + res, true);
                }
            }
            fclose(pipe_err);
            std::filesystem::remove(err_out);
        }

        // did it exit normally?
        if (WIFEXITED(status)) {
            int exitCode = WEXITSTATUS(status);
            if (exitCode != 0) {
                log("Command '" + cmd + "' exited with code " + std::to_string(exitCode), true);
                return false;
            }
            return true; // success
        }
        // was it killed by a signal?
        if (WIFSIGNALED(status)) {
            int sig = WTERMSIG(status);
            log("Command '" + cmd + "' killed by signal " + std::to_string(sig), true);
            return false;
        }

        log("Command '" + cmd + "' did not terminate normally", true);
        return false;
    }

    bool checkPackage(std::string extra_pkg, bool &apt_updated) {

        auto ros_distro = std::getenv("ROS_DISTRO");
        std::string cmd;

        if (extra_pkg[0] == '/') { // dir

            log("  => directory, checking package.xml");
            auto pkg_xml_path = std::filesystem::path(extra_pkg) /  "package.xml";

            if (!std::filesystem::exists(pkg_xml_path)) {
                log("     " + pkg_xml_path.string() + " not found, skipping", true);
                return false;
            }

            tinyxml2::XMLDocument xml_tree;
            xml_tree.LoadFile(pkg_xml_path.c_str());
            
            log("     parsed " + pkg_xml_path.string());

            // auto xml_root = xml_tree.getroot()
            tinyxml2::XMLElement* xml_root = xml_tree.FirstChildElement("package");
            if (!xml_root) {
                log("     XML root not found, skipping", true);
                return false;
            }
            tinyxml2::XMLElement* xml_export = xml_root->FirstChildElement("export");
            if (!xml_export) {
                log("     package/export not found in XML, skipping", true);
                return false;
            }
            tinyxml2::XMLElement* xml_build_type = xml_export->FirstChildElement("build_type");
            if (!xml_build_type) {
                log("     package/export/build_type not found in XML, skipping", true);
                return false;
            }
            tinyxml2::XMLElement* xml_pkg_name = xml_root->FirstChildElement("name");
            if (!xml_pkg_name) {
                log("     package/pkg_name not found in XML, skipping", true);
                return false;
            }
            log("     name: " + std::string(xml_pkg_name->GetText()));
            log("     build_type: " + std::string(xml_build_type->GetText()));

            log("     installing deps...");
            if (!runCmd("/usr/bin/rosdep install -i --from-path " + extra_pkg + " --rosdistro "+ ros_distro + " -y"))
                return false;
            
            log("     building...");
            if (!runCmd("/usr/bin/colcon build --symlink-install --base-path " + extra_pkg))
                return false;
            
            log("     Done.");
            return true;

        } else { // package name -> apt
            
            if (!apt_updated) {
                log("Running apt-update...");
                if (!runCmd("/usr/bin/apt-get update"))
                    return false;
                apt_updated = true; //only once
            }

            auto pkg_name = "ros-" + std::string(ros_distro) + "-" + replace(extra_pkg, "_", "-");

            log("  => package, installing " + pkg_name);

            if (!runCmd("/usr/bin/apt-get install -y " + pkg_name))
                return false;
            
            log("     Done.");
            return true;
        }
    }

    bool installExtraPackages() {

        // if rebooted after some changes were made, remove the lock and carry on
        if (std::filesystem::exists(IGNORE_PACKAGES_CHECK_FILE)) {
            std::filesystem::remove(IGNORE_PACKAGES_CHECK_FILE);
            return false;
        }

        YAML::Node checked_packages_log; // don't re-process, unless forced in config
        if (std::filesystem::exists(CHECKED_PACKAGES_FILE_PATH)) {
            checked_packages_log = YAML::LoadFile(CHECKED_PACKAGES_FILE_PATH); 
        }

        if (!std::filesystem::exists(CONFIG_PATH)) {
            log("Config file not found at " + std::string(CONFIG_PATH), true);  
            return false;
        }

        YAML::Node node_config = YAML::LoadFile(CONFIG_PATH);
        auto extra_packages = node_config["/**"]["ros__parameters"]["extra_packages"];
        if (!extra_packages.IsDefined() || !extra_packages.IsSequence()) {
            return false; //nothing to do
        }
        
        log(MAGENTA + "Checking extra packages..." + CLR);

        auto packages_changed= false;
        bool apt_updated = false;
        for (const auto& item : extra_packages) {

            auto extra_pkg = item.as<std::string>();
            bool force = false;
            
            // if package name is followed by :FORCE_CHECK, it will be checked (re-compiled) on every start
            if (extra_pkg.find(':') != std::string::npos) {
                auto parts = split(extra_pkg, ':');
                extra_pkg = parts[0];
                if (parts.size() > 1 && parts[1] == "FORCE_CHECK") {
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
                log("Skipping package: '" + extra_pkg + "' (checked before)");
                continue;
            }

            if (!checked_packages_log["packages"].IsDefined())
                checked_packages_log["packages"] = YAML::Node(YAML::NodeType::Sequence);

            log(YELLOW + "Checking package" + (force ? " (FORCE)" : "") + ": '" + extra_pkg + "'..." + CLR);
            
            packages_changed = checkPackage(extra_pkg, apt_updated) || packages_changed;

            if (!checked_before)
                checked_packages_log["packages"].push_back(extra_pkg);
        }

        if (checked_packages_log["packages"].IsDefined()) {
            std::ofstream fout(CHECKED_PACKAGES_FILE_PATH);
            fout << checked_packages_log;
            fout.close();
        }

        if (packages_changed) { // add lock file so that this check is skipped on the next start
            std::ofstream file(IGNORE_PACKAGES_CHECK_FILE);
            file.close();
        }
        
        return packages_changed; // restart
    }
    
}