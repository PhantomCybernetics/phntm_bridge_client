#include "phntm_bridge/introspection.hpp"
#include "phntm_bridge/sio.hpp"
#include "phntm_bridge/wrtc_peer.hpp"
#include "sio_message.h"
#include <mutex>
#include <ostream>
#include <iostream>
#include <phntm_interfaces/msg/detail/docker_status__struct.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
// #include <ament_index_cpp/has_resource.hpp>
// JAZZY:
#include <ament_index_cpp/has_resource.hpp>
#include <string>
#include <filesystem>
#include <fstream>

namespace phntm {

    Introspection* Introspection::instance = nullptr;
    const std::string Introspection::L = "[I] ";
    std::mutex Introspection::mutex;

    Introspection::Introspection(std::shared_ptr<PhntmBridge> node, std::shared_ptr<BridgeConfig> config) {

        this->node = node;
        this->config = config;
        this->running = false;

        // auto my_callback_group = rclcpp::create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        if (!this->config->docker_monitor_topic.empty()) {
            auto qos = rclcpp::QoS(1);
            qos.best_effort();
            qos.durability_volatile();
            qos.lifespan(rclcpp::Duration::max());
            // rclcpp::SubscriptionOptions options;
            // options.callback_group = my_callback_group;
            log(L + "Subscribing to " + this->config->docker_monitor_topic);
            this->docker_sub = this->node->create_subscription<phntm_interfaces::msg::DockerStatus>(this->config->docker_monitor_topic, qos,
                std::bind(&Introspection::onDockerMonitorMessage, this, std::placeholders::_1));
        }
    }

    void Introspection::init(std::shared_ptr<PhntmBridge> node, std::shared_ptr<BridgeConfig> config) {
        if (Introspection::instance != nullptr) {
            return;
        }
        Introspection::instance = new Introspection(node, config);
    }

    void Introspection::start() {
        auto instance = Introspection::instance;
        if (instance->running)
            return;

        instance->running = true;
        RCLCPP_WARN(instance->node->get_logger(), "%s Introspection starting...", L.c_str());
        instance->reportRunningState();
        
        instance->start_time = instance->node->now();
        instance->introspection_in_progress = false;
        instance->timer = instance->node->create_wall_timer(
            std::chrono::milliseconds((uint) instance->config->discovery_period_sec * 1000),
            std::bind(&Introspection::runIntrospection, instance),
            instance->node->introspection_reentrant_group
        );
    }

    void Introspection::stop() {
        auto instance = Introspection::instance;
        if (!instance->running)
            return;

        instance->running = false;
        RCLCPP_WARN(instance->node->get_logger(), "%s Introspection stopped", L.c_str());

        instance->timer->cancel();
        instance->reportRunningState();
    }

    void Introspection::runIntrospection() {
        std::lock_guard<std::mutex> introspection_lock(Introspection::mutex);
        
        if (!this->running || this->introspection_in_progress) {
            return;
        }

        this->introspection_in_progress = true;
        if (this->config->introspection_verbose)
            log(GRAY + L + "Introspecting..." + CLR);

        bool nodes_changed = false;
        bool topics_changed = false;
        bool idls_changed = false;
        bool services_changed = false;

        // fresh nodes
        std::vector<std::string> observed_nodes;
        try {
            observed_nodes = this->node->get_node_names();
        } catch (const std::runtime_error & ex) {
            RCLCPP_ERROR(this->node->get_logger(), "%s Failed to get fresh nodes, skipping introspection...", L.c_str());
            this->introspection_in_progress = false;
            return;
        }

        // split namespace from node name
        std::map<std::string, std::string> nodes_and_namespaces;
        for (auto n : observed_nodes) {
            size_t pos = n.rfind('/');
            std::string ns, node;
            if (pos != std::string::npos) {
                ns = n.substr(0, pos); // must not include the trailing /
                node = n.substr(pos+1);
            } else {
                ns = "";
                node = n;
            }
            if (ns.empty())
                ns = "/";
            nodes_and_namespaces.emplace(node, ns);
        }

        // remove not found nodes
        for (auto it = this->discovered_nodes.begin(); it != this->discovered_nodes.end();) {
            if (nodes_and_namespaces.find(it->first) == nodes_and_namespaces.end()) {
                RCLCPP_INFO(this->node->get_logger(), "%s Lost node %s", L.c_str(), it->first.c_str());
                if (this->discovered_file_extractors.find(it->first) != this->discovered_file_extractors.end()) {
                    this->discovered_file_extractors.erase(it->first);
                }
                it = this->discovered_nodes.erase(it);
                nodes_changed = true;
            } else {
                ++it;
            }
        }

        // add newly found nodes
        for (auto n : nodes_and_namespaces) {
            if (this->discovered_nodes.find(n.first) == this->discovered_nodes.end()) {
                RCLCPP_INFO(this->node->get_logger(), "%s Discovered node %s", L.c_str(), n.first.c_str());
                DiscoveredNode node;
                node.ns = n.second;
                this->discovered_nodes.emplace(n.first, node);
                nodes_changed = true;
            }

            // reset
            this->discovered_nodes[n.first].tmp_publishers.clear();
            this->discovered_nodes[n.first].tmp_subscribers.clear();
            this->discovered_nodes[n.first].needs_subscribers_qos_check = false;
        }

        // fresh topics
        std::map<std::string, std::vector<std::string>> observed_topics_and_types;
        try {
            observed_topics_and_types = this->node->get_topic_names_and_types();
        } catch (const std::runtime_error & ex) {
            RCLCPP_ERROR(this->node->get_logger(), "%s Failed to get fresh topics, skipping  introslection...", L.c_str());
            this->introspection_in_progress = false;
            return;
        }
        
        // remove no longer observed topics
        for (auto it = this->discovered_topics.begin(); it != this->discovered_topics.end();) {
            if (observed_topics_and_types.find(it->first) == observed_topics_and_types.end()) {
                RCLCPP_INFO(this->node->get_logger(), "%s Lost topic %s", L.c_str(), it->first.c_str());
                it = this->discovered_topics.erase(it);
                topics_changed = true;
            } else {
                ++it;
            }
        }

        // add newly found topics
        for (auto topic : observed_topics_and_types) {

            if (std::find(this->config->blacklist_topics.begin(), this->config->blacklist_topics.end(), topic.first) != this->config->blacklist_topics.end()) {
                continue; // topic blacklisted
            }
            if (std::find(this->config->blacklist_topics.begin(), this->config->blacklist_topics.end(), topic.second[0]) != this->config->blacklist_topics.end()) {
                continue; // topic type blacklisted
            }

            // add new topic
            if (this->discovered_topics.find(topic.first) == this->discovered_topics.end()) {
                RCLCPP_INFO(this->node->get_logger(), "%s Discovered topic %s {%s}", L.c_str(), topic.first.c_str(), topic.second[0].c_str());
                this->discovered_topics.emplace(topic.first, topic.second[0]);
                topics_changed = true;
                idls_changed = this->collectIDLs(topic.second[0]) || idls_changed;
            }

            // topic publishers to nodes
            std::vector<rclcpp::TopicEndpointInfo> observed_topic_publishers;
            try {
                observed_topic_publishers = node->get_publishers_info_by_topic(topic.first);
            } catch (const std::runtime_error & ex) {
                RCLCPP_ERROR(this->node->get_logger(), "%s Failed getting publishers of %s, skipping...", L.c_str(), topic.first.c_str());
                continue;
            }
            for (auto pub : observed_topic_publishers) {
                if (this->discovered_nodes.find(pub.node_name()) == this->discovered_nodes.end()) {
                    RCLCPP_ERROR(this->node->get_logger(), "%s Publisher node %s not found for %s", L.c_str(), pub.node_name().c_str(), topic.first.c_str());
                    continue;
                }

                auto node_tmp_publishers = &this->discovered_nodes.at(pub.node_name()).tmp_publishers;
                if (node_tmp_publishers->find(topic.first) == node_tmp_publishers->end()) {

                    std::vector<rclcpp::QoS> qos_list;
                    qos_list.push_back(pub.qos_profile());
                    NodePubTopic info = { topic.second[0], qos_list };
                    node_tmp_publishers->emplace(topic.first, info);

                } else { // another publisher to the same topic on this node
                    node_tmp_publishers->at(topic.first).qos.push_back(pub.qos_profile());
                }
            }

            // topic subscribers to nodes
            std::vector<rclcpp::TopicEndpointInfo> observed_topic_subscribers;
            try {
                observed_topic_subscribers = node->get_subscriptions_info_by_topic(topic.first);
            } catch (const std::runtime_error & ex) {
                RCLCPP_ERROR(this->node->get_logger(), "%s Failed getting subscribers of %s, skipping...", L.c_str(), topic.first.c_str());
                continue;
            }
            for (auto sub : observed_topic_subscribers) {
                if (this->discovered_nodes.find(sub.node_name()) == this->discovered_nodes.end()) {
                    RCLCPP_ERROR(this->node->get_logger(), "%s Subscriber node %s not found for %s", L.c_str(), sub.node_name().c_str(), topic.first.c_str());
                    continue;
                }

                auto node_tmp_subscribers = &this->discovered_nodes.at(sub.node_name()).tmp_subscribers;
                if (node_tmp_subscribers->find(topic.first) == node_tmp_subscribers->end()) {
                    std::vector<SubQoS> qos_list;
                    SubQoS qos = { sub.qos_profile(), "", "" };
                    qos_list.push_back(qos);
                    NodeSubTopic info = { topic.second[0], qos_list};
                    node_tmp_subscribers->emplace(topic.first, info);
                    
                } else {  // another subscriber to the same topic on this node
                    SubQoS qos = { sub.qos_profile(), "", "" };
                    node_tmp_subscribers->at(topic.first).qos.push_back(qos);
                }
            }
        }

        // check nodes for pub/sub changes
        for (auto &n : this->discovered_nodes) {

            auto pubs_changed = false;
            auto subs_changed = false;
            auto node_name = n.first;

            // check new pubs
            for (auto pub : n.second.tmp_publishers) {
                auto pub_topic = pub.first;
                if (n.second.publishers.find(pub_topic) == n.second.publishers.end()) {
                    pubs_changed = true;
                    log(L + node_name + GREEN + " >> " + CLR + pub_topic + GRAY + " {" + pub.second.msg_type + "}" + CLR);
                }
            }

            // check existing pubs
            for (auto pub : n.second.publishers) {
                auto pub_topic = pub.first;
                if (n.second.tmp_publishers.find(pub_topic) == n.second.tmp_publishers.end()) {
                    pubs_changed = true;
                    RCLCPP_INFO(this->node->get_logger(), "%s %s Stopped publishing >> %s", L.c_str(), node_name.c_str(), pub_topic.c_str());
                } else { // publister existed
                    auto tmp = n.second.tmp_publishers[pub_topic];
                    auto old = pub.second;
                    if (tmp.msg_type != old.msg_type) {
                        pubs_changed = true;
                        RCLCPP_WARN(this->node->get_logger(), "%s Message type changed for %s >> %s from {%s} to {%s}", L.c_str(), node_name.c_str(), pub_topic.c_str(), old.msg_type.c_str(), tmp.msg_type.c_str());
                    } else if (tmp.qos.size() != old.qos.size()) {
                        pubs_changed = true;
                        RCLCPP_INFO(this->node->get_logger(), "%s Changed number of publishers %s >> %s {%s} %s", L.c_str(), node_name.c_str(), pub_topic.c_str(), tmp.msg_type.c_str(), tmp.qos.size() > 1 ? ("[" + std::to_string(tmp.qos.size()) + "]").c_str() : "");
                    } else {
                        for (size_t i = 0; i < tmp.qos.size(); i++) {
                            if (tmp.qos[i] != old.qos[i]) {
                                pubs_changed = true;
                                RCLCPP_INFO(this->node->get_logger(), "%s Changed publisger QoS %s of %s >> %s {%s}", L.c_str(), tmp.qos.size() > 1 ?(std::to_string(i)+"/"+std::to_string(tmp.qos.size())).c_str() : "", node_name.c_str(), pub_topic.c_str(), tmp.msg_type.c_str());
                            }
                        }
                    }
                }
            }

            // check new subs
            for (auto sub : n.second.tmp_subscribers) {
                auto sub_topic = sub.first;
                if (n.second.subscribers.find(sub_topic) == n.second.subscribers.end()) {
                    subs_changed = true;
                    log(L +  node_name  + YELLOW + " << " + CLR + sub_topic + GRAY + " {" + sub.second.msg_type + "}" + CLR);
                }
            }

            // check existing subs
            for (auto sub : n.second.subscribers) {
                auto sub_topic = sub.first;
                if (n.second.tmp_subscribers.find(sub_topic) == n.second.tmp_subscribers.end()) {
                    subs_changed = true;
                    RCLCPP_INFO(this->node->get_logger(), "%s Lost subscriber %s << %s", L.c_str(), node_name.c_str(), sub_topic.c_str());
                } else { // subscriber existed
                    auto tmp = n.second.tmp_subscribers[sub_topic];
                    auto old = sub.second;
                    if (tmp.msg_type != old.msg_type) {
                        subs_changed = true;
                        RCLCPP_WARN(this->node->get_logger(), "%s Message type changed for %s << %s from {%s} to {%s}", L.c_str(), node_name.c_str(), sub_topic.c_str(), old.msg_type.c_str(), tmp.msg_type.c_str());
                    } else if (tmp.qos.size() != old.qos.size()) {
                        subs_changed = true;
                        RCLCPP_INFO(this->node->get_logger(), "%s Changed number of subscribers %s << %s {%s} %s", L.c_str(), node_name.c_str(), sub_topic.c_str(), tmp.msg_type.c_str(), tmp.qos.size() > 1 ? ("[" + std::to_string(tmp.qos.size()) + "]").c_str() : "");
                    } else {
                        for (size_t i = 0; i < tmp.qos.size(); i++) {
                            if (tmp.qos[i].qos != old.qos[i].qos) {
                                subs_changed = true;
                                RCLCPP_INFO(this->node->get_logger(), "%s Changed subscriber QoS %s of %s << %s {%s}", L.c_str(), tmp.qos.size() > 1 ? (std::to_string(i)+"/"+std::to_string(tmp.qos.size())).c_str() : "", node_name.c_str(), sub_topic.c_str(), tmp.msg_type.c_str());
                            } else {
                                tmp.qos[i].error = old.qos[i].error; // copy on no change
                                tmp.qos[i].warning = old.qos[i].warning;
                            }
                        }
                    }
                }
            }

            n.second.publishers.clear();
            for (auto pub : n.second.tmp_publishers)
                n.second.publishers.emplace(pub.first, pub.second);
            n.second.subscribers.clear();
            for (auto sub : n.second.tmp_subscribers)
                n.second.subscribers.emplace(sub.first, sub.second);

            if (subs_changed)
                n.second.needs_subscribers_qos_check = true; // check when all other nodes have publishers updated

            if (subs_changed || pubs_changed)
                nodes_changed = true; // generateupdate
        
        }

        // check node subscriber QoS
        for (auto &n : this->discovered_nodes) {
            if (!n.second.needs_subscribers_qos_check)
                continue;
            this->checkSubscriberQos(n.first,&n.second);
            n.second.needs_subscribers_qos_check = false;
        }

        // force adding Byte to IDLs as we use it on the client to send heartbeat (but don't produce into a topic so it might not get discovered)
        idls_changed = this->collectIDLs("std_msgs/msg/Byte") || idls_changed;

        // services 
        for (auto &n : this->discovered_nodes) {

            auto node_name = n.first;

            // fresh services
            std::map<std::string, std::vector<std::string>> observed_node_services;
            try {
                observed_node_services = this->node->get_service_names_and_types_by_node(node_name, n.second.ns);
            } catch (const std::runtime_error & ex) {
                RCLCPP_ERROR(this->node->get_logger(), "%s Failed getting services of %s, skipping...", L.c_str(), node_name.c_str());
                continue;
            }

            for (auto s : observed_node_services) {

                auto service_type = s.second[0];

                // find agent nodes with file extraction service enabled (before blacklisting)
                if (service_type == "phntm_interfaces/srv/FileRequest") {
                    if (this->discovered_file_extractors.find(node_name) == this->discovered_file_extractors.end()) {
                        auto client = node->create_client<phntm_interfaces::srv::FileRequest>(s.first);
                        this->discovered_file_extractors.emplace(node_name, client); //id node => srv id
                        RCLCPP_INFO(this->node->get_logger(), "%s Discovered file extractor node %s %s", L.c_str(), node_name.c_str(), s.first.c_str());
                    }
                }

                bool blacklisted = false;
                if (service_type == "rcl_interfaces/srv/ListParameters" ||
                    service_type == "rcl_interfaces/srv/DescribeParameters" ||
                    service_type == "rcl_interfaces/srv/GetParameters" ||
                    service_type == "rcl_interfaces/srv/GetParameterTypes" ||
                    service_type == "rcl_interfaces/srv/SetParameters" ||
                    service_type == "rcl_interfaces/srv/SetParametersAtomically"
                ) {
                    if (!this->config->enable_node_parameters_read) {
                        blacklisted = true; // no read => ignore all param services
                    } else if (!this->config->enable_node_parameters_write && (service_type == "rcl_interfaces/srv/SetParameters" || service_type == "rcl_interfaces/srv/SetParametersAtomically")) {
                        blacklisted = true;
                    } else {
                        for (size_t i = 0; i < this->config->blacklist_parameter_services.size(); i++) {
                            if (startsWith(node_name, config->blacklist_parameter_services[i])) {
                                blacklisted = true;
                                break;
                            }
                        }
                    }
                }
                if (blacklisted)
                    continue;

                if (std::find(this->config->blacklist_services.begin(), this->config->blacklist_services.end(), s.first) != this->config->blacklist_services.end()) {
                    continue; // service blacklisted
                }
                if (std::find(this->config->blacklist_services.begin(), this->config->blacklist_services.end(), service_type) != this->config->blacklist_services.end()) {
                    continue; // service type blacklisted
                }

                if (n.second.services.find(s.first) == n.second.services.end()) { // new service
                    n.second.services.emplace(s.first, service_type);
                    services_changed = true;
                    idls_changed = this->collectIDLs(service_type) || idls_changed;
                    RCLCPP_INFO(this->node->get_logger(), "%s Discovered srv %s: %s {%s}", L.c_str(), node_name.c_str(), s.first.c_str(), service_type.c_str());
                } else if (n.second.services.at(s.first) != service_type) {
                    n.second.services[s.first] = service_type;
                    services_changed = true;
                    idls_changed = this->collectIDLs(service_type) || idls_changed;
                    RCLCPP_WARN(this->node->get_logger(), "%s Message type changed for srv %s: %s {%s}", L.c_str(), node_name.c_str(), s.first.c_str(), service_type.c_str());
                }
            }
        }

        if (idls_changed) {
            this->reportIDLs(); // report defs before others
        }

        if (nodes_changed || topics_changed || services_changed) {
            this->reportNodes();
        }

        if (topics_changed) {
            log(GRAY + L + "Topics changed, processing peerSubscriptions..." + CLR);
            WRTCPeer::processAllPeerSubscriptions();
        }

        this->introspection_in_progress = false;

        // auto stop
        if (this->config->stop_discovery_after_sec > 0.0f) {
            rclcpp::Duration duration = node->now() - start_time;
            if (duration.seconds() > this->config->stop_discovery_after_sec) {
                Introspection::stop();
            }
        }    
    }

    void Introspection::checkSubscriberQos(std::string id_subscriber, DiscoveredNode *subscriber_node) {
        for (auto &sub : subscriber_node->subscribers) {
            auto sub_topic = sub.first;

            for (auto & sub_qos : sub.second.qos) {
                sub_qos.error = ""; // reset
                sub_qos.warning = "";

                for (auto &pub_node : this->discovered_nodes) {
                    if (pub_node.second.publishers.find(sub_topic) == pub_node.second.publishers.end())
                        continue; // not publishing topic

                    for (auto pub_qos : pub_node.second.publishers.at(sub_topic).qos) {
                        auto res = rclcpp::qos_check_compatible(pub_qos, sub_qos.qos);

                        switch (res.compatibility) {
                            case rclcpp::QoSCompatibility::Error:
                                sub_qos.warning = "";
                                sub_qos.error = res.reason;
                                RCLCPP_WARN(this->node->get_logger(), "%s Detected QoS compatilibility error for %s << %s: %s", L.c_str(), id_subscriber.c_str(), sub_topic.c_str(), res.reason.c_str());
                                break;
                            case rclcpp::QoSCompatibility::Warning:
                                sub_qos.warning = res.reason;
                                sub_qos.error = "";
                                RCLCPP_WARN(this->node->get_logger(), "%s Detected QoS compatilibility warnning for %s << %s: %s", L.c_str(), id_subscriber.c_str(), sub_topic.c_str(), res.reason.c_str());
                                break;
                            default: // ignore (last error or warning will be reported)
                                break;
                        }
                    }
                }
            }
        }
    }

    std::map<std::string, rclcpp::Client<phntm_interfaces::srv::FileRequest>::SharedPtr> Introspection::getFileExtractors() {
        if (instance == nullptr)
            return {};
        return instance->discovered_file_extractors;
    }

    std::string Introspection::getInterfaceIDLPath(std::string interface_name) {
        auto parts = split(interface_name, '/');
        if (parts.size() < 2) {
            RCLCPP_WARN(this->node->get_logger(), "%s[IDL] Invalid name '%s'. Expected at least two parts separated by '/'", Introspection::L.c_str(), interface_name.c_str());
            return "";
        }
        auto all_non_empty = std::all_of(parts.begin(), parts.end(), [](const std::string& s) { return !s.empty(); });
        if (!all_non_empty) {
            RCLCPP_WARN(this->node->get_logger(), "%s[IDL] Invalid name '%s'. Must not contain empty part", Introspection::L.c_str(), interface_name.c_str());
            return "";
        }
        if (std::find(parts.begin(), parts.end(), "..") != parts.end()) {
            RCLCPP_WARN(this->node->get_logger(), "%s[IDL] Invalid name '%s'. Must not contain '..'", Introspection::L.c_str(), interface_name.c_str());
            return "";
        }
        std::string prefix_path;
        if (!ament_index_cpp::has_resource("packages", parts[0], &prefix_path)) {
            RCLCPP_WARN(this->node->get_logger(), "%s[IDL] Unknown package '%s'", Introspection::L.c_str(), parts[0].c_str());
            return "";
        }

        std::filesystem::path interface_path = std::filesystem::path(prefix_path) / "share" / interface_name;

        if (rsplit(parts[parts.size()-1], '.', 1).size() == 1) {
            interface_path += ".idl";
        }

        if (!std::filesystem::exists(interface_path)) {
            RCLCPP_WARN(this->node->get_logger(), "%s[IDL] Could not find the interface '%s'", Introspection::L.c_str(), interface_path.string().c_str());
            return "";
        }
        
        return interface_path;
    }

    bool Introspection::collectIDLs(std::string msg_type) {

        if (std::find(this->config->blacklist_msg_types.begin(), this->config->blacklist_msg_types.end(), msg_type) != this->config->blacklist_msg_types.end())
            return false; // type blacklisted

        if (this->discovered_idls.find(msg_type) != this->discovered_idls.end())
            return false; // already processed, no change

        auto idl_path = this->getInterfaceIDLPath(msg_type);
        if (idl_path.empty()) {
            RCLCPP_WARN(this->node->get_logger(), "%s[IDL] Path not found for '%s'", Introspection::L.c_str(), msg_type.c_str());
            return false;
        }

        std::ifstream file(idl_path);
        std::string idl_raw((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

        std::string idl_clear = "";
        std::vector<std::string> inluded_message_types;

        bool first_valid_line = false;
        std::istringstream iss(idl_raw);
        std::string line;
        while (std::getline(iss, line)) {
            if (line.find("//") == 0) // skipping comments
                continue;
            if (line.empty() && !first_valid_line)
                continue;
            if (!line.empty())
                first_valid_line = true;

            idl_clear += line + "\n";

            // detect includes
            if (line.find("#include") == 0 && line.find(".idl") != std::string::npos) {
                auto inc_path = replace(line, "#include", "");
                inc_path = trim(inc_path, " \t\n\r\f\v\"'"); //remove whitespaces " & '
                auto inc_msg_type = replace(inc_path, ".idl", "");
                inluded_message_types.push_back(inc_msg_type);
            }
        }

        this->discovered_idls.emplace(msg_type, idl_clear);
        RCLCPP_INFO(this->node->get_logger(), "%s[IDL] Loaded '%s'", Introspection::L.c_str(), idl_path.c_str());

        // load includes
        for (auto inc_msg_type : inluded_message_types) {
            this->collectIDLs(inc_msg_type);
        }
        
        return true;
    }

    void Introspection::onDockerMonitorMessage(phntm_interfaces::msg::DockerStatus const & msg) {

        auto host = !msg.header.frame_id.empty() ? "phntm_agent_" + msg.header.frame_id : "phntm_agent"; // # default frame id is empty
        auto docker_containers_changed = false;

        if (this->discovered_docker_containers.find(host) == this->discovered_docker_containers.end()) { // new host reporting
            docker_containers_changed = true;
            
            // try looking for hosts we have and match id by container names
            for (size_t i = 0; i < msg.containers.size(); i++) {
                for (auto p : this->discovered_docker_containers) {
                    for (size_t j = 0; j < p.second.containers.size(); j++) {
                        if (msg.containers[i].id == p.second.containers[j].id) { // known container id found under new host
                            RCLCPP_INFO(this->node->get_logger(), "%s Agent host changed from %s to %s", Introspection::L.c_str(), p.first.c_str(), host.c_str());
                            this->discovered_docker_containers.erase(p.first);
                            break;
                        }
                    }
                }
            }

        } else { // host seen before
            if (this->discovered_docker_containers[host].containers.size() != msg.containers.size()) {
                docker_containers_changed = true;
            } else {
                for (size_t i = 0; i < msg.containers.size(); i++) {
                    if (msg.containers[i].id != this->discovered_docker_containers[host].containers[i].id ||
                        msg.containers[i].name != this->discovered_docker_containers[host].containers[i].name ||
                        msg.containers[i].status != this->discovered_docker_containers[host].containers[i].status) {
                        docker_containers_changed = true;
                        break;
                    }
                }
            }
        }

        this->discovered_docker_containers.insert_or_assign(host, msg);

        if (docker_containers_changed) {
            this->reportDocker();

            if (!this->running) {
                Introspection::start();
            }
        }
    }

    void Introspection::report() {
        auto instance = Introspection::instance;
        instance->reportIDLs(); // report defs before others
        instance->reportNodes();
        instance->reportDocker();
        instance->reportRunningState();
    }

    void Introspection::reportIDLs() {
        auto msg = sio::object_message::create();

        for (auto &idl : this->discovered_idls) {
            auto one_idl = sio::string_message::create(idl.second);
            msg->get_map().emplace(idl.first, one_idl);
        }
       
        log(GRAY + L + "Reporting " + std::to_string(this->discovered_idls.size()) + " IDLs" + CLR);
        if (this->config->introspection_verbose)
            log(GRAY + BridgeSocket::printMessage(msg) + CLR);
        BridgeSocket::emit("idls", { msg } , nullptr);
    }

    sio::message::ptr createQoSMessage(rclcpp::QoS qos) {
        auto qos_msg = sio::object_message::create();

        qos_msg->get_map().emplace("depth", sio::int_message::create(static_cast<int>(qos.depth())));
        qos_msg->get_map().emplace("history", sio::int_message::create(static_cast<int>(qos.history())));
        qos_msg->get_map().emplace("reliability", sio::int_message::create(static_cast<int>(qos.reliability())));
        qos_msg->get_map().emplace("durability", sio::int_message::create(static_cast<int>(qos.durability())));
        qos_msg->get_map().emplace("lifespan", sio::int_message::create((int) (qos.lifespan() == rclcpp::Duration::max() ? -1 : qos.lifespan().nanoseconds())));
        qos_msg->get_map().emplace("deadline", sio::int_message::create((int) (qos.deadline() == rclcpp::Duration::max() ? -1 : qos.deadline().nanoseconds())));

        return qos_msg;
    }

    void Introspection::reportNodes() {

        auto msg = sio::object_message::create();

        for (auto &n : this->discovered_nodes) {

            auto n_msg = sio::object_message::create();
            n_msg->get_map().emplace("namespace", sio::string_message::create(n.second.ns));

            auto n_pub_msg = sio::object_message::create();
            for (auto &pub : n.second.publishers) {
                auto t_msg = sio::object_message::create();
                t_msg->get_map().emplace("msg_type", sio::string_message::create(pub.second.msg_type));
                auto qos_list = sio::array_message::create(); //allowing multiple pubs to one topic from the same node
                for (auto qos : pub.second.qos) {
                    qos_list->get_vector().push_back(createQoSMessage(qos));
                }
                t_msg->get_map().emplace("qos", qos_list);
                n_pub_msg->get_map().emplace(pub.first, t_msg);
            }
            n_msg->get_map().emplace("publishers", n_pub_msg);

            auto n_sub_msg = sio::object_message::create();
            for (auto &sub : n.second.subscribers) {
                auto t_msg = sio::object_message::create();
                t_msg->get_map().emplace("msg_type", sio::string_message::create(sub.second.msg_type));
                auto qos_list = sio::array_message::create(); //allowing multiple subs to one topic from the same node
                for (auto qos : sub.second.qos) {
                    auto q_msg = sio::object_message::create();
                    q_msg->get_map().emplace("qos", createQoSMessage(qos.qos));
                    if (!qos.error.empty())
                        q_msg->get_map().emplace("error", sio::string_message::create(qos.error));
                    if (!qos.warning.empty())
                        q_msg->get_map().emplace("warning", sio::string_message::create(qos.warning));
                    qos_list->get_vector().push_back(q_msg);
                }           
                t_msg->get_map().emplace("qos", qos_list);
                n_sub_msg->get_map().emplace(sub.first, t_msg);
            }
            n_msg->get_map().emplace("subscribers", n_sub_msg);

            auto n_srv_msg = sio::object_message::create();
            for (auto &s : n.second.services) {
                n_srv_msg->get_map().emplace(s.first, sio::string_message::create(s.second));
            }
            n_msg->get_map().emplace("services", n_srv_msg);

            msg->get_map().emplace(n.first, n_msg);
        }
        
        if (this->discovered_nodes.size()) {
            log(GRAY + L + "Reporting " + std::to_string(this->discovered_nodes.size()) + " nodes" + CLR);
            if (this->config->introspection_verbose)
                log(GRAY + BridgeSocket::printMessage(msg) + CLR);
        } else {
            log(GRAY + L + "Reporting empty nodes" + CLR);
        }
        BridgeSocket::emit("nodes", { msg }, nullptr);
    }

    void Introspection::reportDocker() {
        auto msg = sio::object_message::create();
        std::vector<std::string> hosts;

        for (auto p : this->discovered_docker_containers) {
            hosts.push_back(p.first);

            auto host_msg = sio::object_message::create();

            auto header = sio::object_message::create();
            header->get_map().emplace("frame_id", sio::string_message::create(p.second.header.frame_id));
            auto stamp = sio::object_message::create();
            stamp->get_map().emplace("sec", sio::int_message::create(p.second.header.stamp.sec));
            stamp->get_map().emplace("nanosec", sio::int_message::create(p.second.header.stamp.nanosec));
            header->get_map().emplace("stamp", stamp);
            host_msg->get_map().emplace("header", header);

            auto containers = sio::array_message::create();
            for (size_t i = 0; i < p.second.containers.size(); i++) {
                auto container = sio::object_message::create();
                container->get_map().emplace("block_io_read_bytes", sio::int_message::create(p.second.containers[i].block_io_read_bytes));
                container->get_map().emplace("block_io_write_bytes", sio::int_message::create(p.second.containers[i].block_io_write_bytes));
                container->get_map().emplace("cpu_percent",  sio::double_message::create(p.second.containers[i].cpu_percent));
                container->get_map().emplace("id", sio::string_message::create(p.second.containers[i].id));
                container->get_map().emplace("name", sio::string_message::create(p.second.containers[i].name));
                container->get_map().emplace("pids", sio::int_message::create(p.second.containers[i].pids));
                container->get_map().emplace("status", sio::int_message::create(p.second.containers[i].status));
                containers->get_vector().push_back(container);
            }
            host_msg->get_map().emplace("containers", containers);

            msg->get_map().emplace(p.first, host_msg);
        }
        
        if (hosts.size()) {
            log(GRAY + L + "Reporting " + std::to_string(this->discovered_docker_containers.size()) + " Docker containers for " + join(hosts) + CLR);
            if (this->config->introspection_verbose)
                log(GRAY + BridgeSocket::printMessage(msg) + CLR);
        } else {
            log(GRAY + L + "Reporting empty Docker containers" + CLR);
        }
        BridgeSocket::emit("docker", { msg }, nullptr);
    }

    void Introspection::reportRunningState() {
        auto msg = sio::bool_message::create(this->running);
        BridgeSocket::emit("introspection", { msg }, nullptr);
    }

    std::string Introspection::getService(std::string service) {
        auto instance = Introspection::instance;
        for (auto n : instance->discovered_nodes) {
            for (auto s : n.second.services) {
                if (s.first == service) {
                    return s.second;
                }
            }
        }
        return ""; //not found
    }

    std::string Introspection::getTopic(std::string topic) {
        auto instance = Introspection::instance;
        for (auto it = instance->discovered_topics.begin(); it != instance->discovered_topics.end(); it++) {
            if (it->first == topic) {
                return it->second;
            }
        }
        return ""; //not found
    }

    Introspection::~Introspection() {
        instance = nullptr;
    }

}