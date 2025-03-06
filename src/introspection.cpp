#include "phntm_bridge/introspection.hpp"
#include "phntm_bridge/sio.hpp"
#include "sio_message.h"
#include <ostream>
#include <iostream>
#include <phntm_interfaces/msg/detail/docker_status__struct.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include <ament_index_cpp/has_resource.hpp>
#include <string>
#include <filesystem>
#include <fstream>

Introspection* Introspection::instance = nullptr;

Introspection::Introspection(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<BridgeSocket> sio, std::shared_ptr<BridgeConfig> config) {

    if (instance != nullptr) {
        std::cerr << RED << "Introspection instance already created" << CLR << std::endl;
        return;
    }

    this->node = node;
    this->sio = sio;
    this->config = config;
    this->running = false;

    instance = this;

    // auto my_callback_group = rclcpp::create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    if (!this->config->docker_monitor_topic.empty()) {
        auto qos = rclcpp::QoS(1);
        qos.best_effort();
        qos.durability_volatile();
        qos.lifespan(rclcpp::Duration::max());
        // rclcpp::SubscriptionOptions options;
        // options.callback_group = my_callback_group;
        std::cout << "Subscribing to " << this->config->docker_monitor_topic << std::endl;
        this->docker_sub = this->node->create_subscription<phntm_interfaces::msg::DockerStatus>(this->config->docker_monitor_topic, qos,
            std::bind(&Introspection::onDockerMonitorMessage, this, std::placeholders::_1));
    }
}

void Introspection::start() {
    if (this->running)
        return;

    this->running = true;
    std::cout << CYAN << "Introspection starting..." << CLR << std::endl;
    this->reportRunningState();

    this->start_time = node->now();
    this->introspection_in_progress = false;
    this->timer = this->node->create_wall_timer(
        std::chrono::milliseconds((uint) this->config->discovery_period_sec * 1000),
        std::bind(&Introspection::runIntrospection, this)
    );
}

void Introspection::stop() {
    if (!this->running)
        return;

    this->running = false;
    std::cout << CYAN << "Introspection stopped." << CLR << std::endl;

    this->timer->cancel();
    this->reportRunningState();
}

void Introspection::runIntrospection() {
    if (!this->running || this->introspection_in_progress) {
        return;
    }

    this->introspection_in_progress = true;
    if (this->config->introspection_verbose)
        std::cout << GRAY << "Introspecting..." << CLR << std::endl;

    bool nodes_changed = false;
    bool topics_changed = false;
    bool idls_changed = false;
    bool services_changed = false;

    // fresh nodes
    auto nodes = this->node->get_node_names();

    // split namespace from node name
    std::map<std::string, std::string> nodes_and_namespaces;
    for (auto n : nodes) {
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
            std::cout << GRAY << "Lost node " << it->first << CLR << std::endl;
            it = this->discovered_nodes.erase(it);
            nodes_changed = true;
        } else {
            ++it;
        }
    }

    // add newly found nodes
    for (auto n : nodes_and_namespaces) {
        if (this->discovered_nodes.find(n.first) == this->discovered_nodes.end()) {
            std::cout << "Discovered node " << MAGENTA << n.first << CLR << " ns=" << n.second << std::endl;
            DiscoveredNode node;
            node.ns = n.second;
            this->discovered_nodes.emplace(n.first, node);
            nodes_changed = true;
        }
    }

    // fresh topics
    auto topics_and_types = this->node->get_topic_names_and_types();

    // remove no longer observed topics
    for (auto it = this->discovered_topics.begin(); it != this->discovered_topics.end();) {
        if (topics_and_types.find(it->first) == topics_and_types.end()) {
            std::cout << GRAY << "Lost topic " << it->first << CLR << std::endl;
            it = this->discovered_topics.erase(it);
            topics_changed = true;
        } else {
            ++it;
        }
    }

    // add newly found topics
    for (auto topic : topics_and_types) {

        if (std::find(this->config->blacklist_topics.begin(), this->config->blacklist_topics.end(), topic.first) != this->config->blacklist_topics.end()) {
            continue; // topic blacklisted
        }

        if (this->discovered_topics.find(topic.first) == this->discovered_topics.end()) {
            std::cout << "Discovered topic " << CYAN << topic.first << CLR << GRAY << " [" << topic.second[0] << "]" << CLR << std::endl;
            this->discovered_topics.emplace(topic.first, topic.second[0]);
            topics_changed = true;
            idls_changed = this->collectIDLs(topic.second[0]) || idls_changed;
        }

        // publishers
        auto publishers_info = node->get_publishers_info_by_topic(topic.first);
        for (auto pub : publishers_info) {
            if (this->discovered_nodes.find(pub.node_name()) == this->discovered_nodes.end()) {
                std::cout << RED << "Publisher node " << pub.node_name() << " not found for " << topic.first << CLR << std::endl;
                continue;
            }

            auto node_publishers = &this->discovered_nodes.at(pub.node_name()).publishers;
            if (node_publishers->find(topic.first) == node_publishers->end()) {
                NodePubTopic info = { topic.second[0], pub.qos_profile() };
                node_publishers->emplace(topic.first, info);
                nodes_changed = true;
                std::cout << "  " << pub.node_name() << GREEN << " >> " << CLR << topic.first << GRAY << " [" << topic.second[0] << "]" << CLR << std::endl;
            } else { //check qos & type
                if (node_publishers->at(topic.first).msg_type != topic.second[0]) {
                    node_publishers->at(topic.first).msg_type = topic.second[0];
                    nodes_changed = true;
                }
                if (node_publishers->at(topic.first).qos != pub.qos_profile()) {
                    node_publishers->at(topic.first).qos = pub.qos_profile();
                    nodes_changed = true;
                }
            }            
        }

        // subscribers
        auto subscribers_info = node->get_subscriptions_info_by_topic(topic.first);
        for (auto sub : subscribers_info) {
            if (this->discovered_nodes.find(sub.node_name()) == this->discovered_nodes.end()) {
                std::cout << RED << "Subscriber node " << sub.node_name() << " not found for " << topic.first << CLR << std::endl;
                continue;
            }

            auto node_subscribers = &this->discovered_nodes.at(sub.node_name()).subscribers;
            if (node_subscribers->find(topic.first) == node_subscribers->end()) {
                NodeSubTopic info = { topic.second[0], sub.qos_profile(), "", "" };
                node_subscribers->emplace(topic.first, info);
                nodes_changed = true;
                std::cout << "  " << sub.node_name() << YELLOW << " << " << CLR << topic.first << GRAY << " [" << topic.second[0] << "]" << CLR << std::endl;
            } else { //check qos & type
                if (node_subscribers->at(topic.first).msg_type != topic.second[0]) {
                    node_subscribers->at(topic.first).msg_type = topic.second[0];
                    nodes_changed = true;
                }
                if (node_subscribers->at(topic.first).qos != sub.qos_profile()) {
                    node_subscribers->at(topic.first).qos = sub.qos_profile();
                    nodes_changed = true;
                }
            }
        }
    }

    // force adding Byte to IDLs as we use it on the client to send heartbeat (but don't produce into a topic so it might not get discovered)
    idls_changed = this->collectIDLs("std_msgs/msg/Byte") || idls_changed;

    // services 
    for (auto &n : this->discovered_nodes) {

        // fresh services
        auto node_services = this->node->get_service_names_and_types_by_node(n.first, n.second.ns);

        for (auto s : node_services) {

            if (std::find(this->config->blacklist_services.begin(), this->config->blacklist_services.end(), s.first) != this->config->blacklist_services.end()) {
                continue; // service blacklisted
            }

            if (n.second.services.find(s.first) == n.second.services.end()) { // new service
                n.second.services.emplace(s.first, s.second[0]);
                services_changed = true;
                idls_changed = this->collectIDLs(s.second[0]) || idls_changed;
                std::cout << "Discovered srv " << n.first << ": " << GREEN << s.first << CLR << GRAY << " [" << s.second[0] << "]" << CLR << std::endl;
            } else if (n.second.services.at(s.first) != s.second[0]) {
                n.second.services[s.first] = s.second[0];
                services_changed = true;
                idls_changed = this->collectIDLs(s.second[0]) || idls_changed;
            }
        }
    }

    if (nodes_changed || topics_changed) {
        this->checkSubscriberQos();
    }

    if (nodes_changed || topics_changed || services_changed) {
        this->reportNodes();
    }

    if (idls_changed) {
        this->reportIDLs();
    }

    this->introspection_in_progress = false;
    
    // auto stop
    if (this->config->stop_discovery_after_sec > 0.0f) {
        rclcpp::Duration duration = node->now() - start_time;
        if (duration.seconds() > this->config->stop_discovery_after_sec) {
            this->stop();
        }
    }    
}

void Introspection::checkSubscriberQos() {
    for (auto &n : this->discovered_nodes) {
        for (auto &t : n.second.subscribers) {
            for (auto &nn : this->discovered_nodes) {
                if (nn.second.publishers.find(t.first) == nn.second.publishers.end())
                    continue;
                auto pub_qos = nn.second.publishers.at(t.first).qos;
                auto sub_qos = n.second.subscribers.at(t.first).qos;

                auto res = rclcpp::qos_check_compatible(pub_qos, sub_qos);
                switch (res.compatibility) {
                    case rclcpp::QoSCompatibility::Error:
                        n.second.subscribers.at(t.first).qos_warning = "";
                        n.second.subscribers.at(t.first).qos_error = res.reason;
                        std::cout << RED << "Detected QoS compatilibility error for " << n.first << " << " << t.first << ": " << res.reason << CLR << std::endl;
                        break;
                    case rclcpp::QoSCompatibility::Warning:
                        n.second.subscribers.at(t.first).qos_warning = res.reason;
                        n.second.subscribers.at(t.first).qos_error = "";
                        std::cout << RED << "Detected QoS compatilibility warnning for " << n.first << " << " << t.first << ": " << res.reason << CLR << std::endl;
                        break;
                    default: //ok
                        n.second.subscribers.at(t.first).qos_warning = "";
                        n.second.subscribers.at(t.first).qos_error = "";
                        break;
                }
            }
        }
    }
}

std::string getInterfaceIDLPath(std::string interface_name) {
    auto parts = split(interface_name, '/');
    if (parts.size() < 2) {
        std::cerr << RED << "Invalid name '" << interface_name << "'. Expected at least two parts separated by '/'" << CLR << std::endl;
        return "";
    }
    auto all_non_empty = std::all_of(parts.begin(), parts.end(), [](const std::string& s) { return !s.empty(); });
    if (!all_non_empty) {
        std::cerr << RED << "Invalid name '" << interface_name << "'. Must not contain empty part" << CLR << std::endl;
        return "";
    }
    if (std::find(parts.begin(), parts.end(), "..") != parts.end()) {
        std::cerr << RED << "Invalid name '" << interface_name << "'. Must not contain '..'" << CLR << std::endl;
        return "";
    }
    std::string prefix_path;
    if (!ament_index_cpp::has_resource("packages", parts[0], &prefix_path)) {
        std::cerr << RED << "Unknown package '" << parts[0] << "'" << CLR << std::endl;
        return "";
    }

    std::filesystem::path interface_path = std::filesystem::path(prefix_path) / "share" / interface_name;

    if (rsplit(parts[parts.size()-1], '.', 1).size() == 1) {
        interface_path += ".idl";
    }

    if (!std::filesystem::exists(interface_path)) {
        std::cerr << RED << "Could not find the interface '" << interface_path << "'" << CLR << std::endl;
        return "";
    }
    
    return interface_path;
}

bool Introspection::collectIDLs(std::string msg_type) {

    if (std::find(this->config->blacklist_msg_types.begin(), this->config->blacklist_msg_types.end(), msg_type) != this->config->blacklist_msg_types.end())
        return false; // type blacklisted

    if (this->discovered_idls.find(msg_type) != this->discovered_idls.end())
        return false; // already processed, no change

    auto idl_path = getInterfaceIDLPath(msg_type);
    if (idl_path.empty()) {
        std::cerr << RED << "IDL path not found for '" << msg_type << "'" << CLR << std::endl;
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
    std::cout << YELLOW << "Loaded " << idl_path << CLR << std::endl;

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
                        std::cout << "Agent host changed from " << p.first << " to " << host << std::endl;
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
            this->start();
        }
    }
}

void Introspection::report() {
    this->reportIDLs();
    this->reportNodes();
    this->reportDocker();
    this->reportRunningState();
}

void Introspection::reportIDLs() {
    auto msg = sio::object_message::create();

    for (auto &idl : this->discovered_idls) {
        auto one_idl = sio::string_message::create(idl.second);
        msg->get_map().emplace(idl.first, one_idl);
    }

    std::cout << GRAY << "Reporting " << this->discovered_idls.size() << " IDLs" << CLR << std::endl;
    if (this->config->introspection_verbose)
            std::cout << GRAY << BridgeSocket::PrintMessage(msg) << CLR << std::endl;

    this->sio->emit("idls", { msg } , nullptr);
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
            t_msg->get_map().emplace("qos", createQoSMessage(pub.second.qos));
            n_pub_msg->get_map().emplace(pub.first, t_msg);
        }
        n_msg->get_map().emplace("publishers", n_pub_msg);

        auto n_sub_msg = sio::object_message::create();
        for (auto &sub : n.second.subscribers) {
            auto t_msg = sio::object_message::create();
            t_msg->get_map().emplace("msg_type", sio::string_message::create(sub.second.msg_type));
            t_msg->get_map().emplace("qos", createQoSMessage(sub.second.qos));
            if (!sub.second.qos_error.empty())
                t_msg->get_map().emplace("qos_error", sio::string_message::create(sub.second.qos_error));
            if (!sub.second.qos_warning.empty())
                t_msg->get_map().emplace("qos_warning", sio::string_message::create(sub.second.qos_warning));
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
        std::cout << GRAY << "Reporting " << this->discovered_nodes.size() << " nodes" << CLR << std::endl;
        if (this->config->introspection_verbose)
            std::cout << GRAY << BridgeSocket::PrintMessage(msg) << CLR << std::endl;
    } else {
        std::cout << GRAY << "Reporting empty nodes" << CLR << std::endl;
    }
    this->sio->emit("nodes", { msg }, nullptr);
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
        std::cout << GRAY << "Reporting " << this->discovered_docker_containers.size() << " Docker containers for " << join(hosts) << CLR << std::endl;
        if (this->config->introspection_verbose)
            std::cout << GRAY << BridgeSocket::PrintMessage(msg) << CLR << std::endl;
    } else {
        std::cout << GRAY << "Reporting empty Docker containers" << CLR << std::endl;
    }
    
    this->sio->emit("docker", { msg }, nullptr);
}

void Introspection::reportRunningState() {
    auto msg = sio::bool_message::create(this->running);
    this->sio->emit("introspection", { msg }, nullptr);
}

std::string Introspection::getService(std::string service) {
    for (auto n : this->discovered_nodes) {
        for (auto s : n.second.services) {
            if (s.first == service) {
                return s.second;
            }
        }
    }
    return ""; //not found
}

Introspection::~Introspection() {
    instance = nullptr;
}
