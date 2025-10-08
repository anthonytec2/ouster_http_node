#include <rclcpp/rclcpp.hpp>
#include <ouster_http_node/msg/time_status.hpp>
#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>
#include <chrono>

using json = nlohmann::json;
namespace ouster_http_node{
class OusterHttpNode : public rclcpp::Node {
public:
     explicit OusterHttpNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("ouster_http_node", options), running_(true) {
        // Declare configurable URL parameter
        this->declare_parameter<std::string>("url", "http://169.254.50.111/api/v1/time/sensor");
        this->get_parameter("url", url_);

        RCLCPP_INFO(this->get_logger(), "Starting Ouster HTTP Node with URL: %s", url_.c_str());

        publisher_ = this->create_publisher<ouster_http_node::msg::TimeStatus>("ouster_time_status", 10);

        // Start worker thread
        worker_thread_ = std::thread(&OusterHttpNode::loop, this);
    }

    ~OusterHttpNode() override {
        running_ = false;
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
    }

private:
    std::string url_;
    std::atomic<bool> running_;
    rclcpp::Publisher<ouster_http_node::msg::TimeStatus>::SharedPtr publisher_;
    std::thread worker_thread_;

    static size_t WriteCallback(void *contents, size_t size, size_t nmemb, std::string *output) {
        size_t total_size = size * nmemb;
        output->append((char *)contents, total_size);
        return total_size;
    }

    void loop() {
        CURL *curl = curl_easy_init();
        if (!curl) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize libcurl");
            return;
        }

        while (rclcpp::ok() && running_) {
            std::string response;
            curl_easy_setopt(curl, CURLOPT_URL, url_.c_str());
            curl_easy_setopt(curl, CURLOPT_TIMEOUT, 5L);
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

            CURLcode res = curl_easy_perform(curl);
            if (res != CURLE_OK) {
                RCLCPP_WARN(this->get_logger(), "HTTP request failed: %s", curl_easy_strerror(res));
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            long http_code = 0;
            curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
            // RCLCPP_INFO(this->get_logger(), "HTTP Response (%ld): %s", http_code, response.c_str());

            try {
                auto j = json::parse(response);

                ouster_http_node::msg::TimeStatus msg;
                msg.stamp = this->get_clock()->now();

                // Safely check and parse JSON
                    auto &sensor = j;

                    if (sensor.contains("timestamp")) {
                        auto &timestamp = sensor["timestamp"];
                        msg.timestamp_time = static_cast<uint64_t>(timestamp.value("time", 0.0) * 1e9);

                        if (timestamp.contains("time_options")) {
                            msg.time_options_sync_pulse_in =
                                static_cast<uint64_t>(timestamp["time_options"].value("sync_pulse_in", 0));
                        } else {
                            msg.time_options_sync_pulse_in = 0;
                        }
                    } else {
                        msg.timestamp_time = 0;
                        msg.time_options_sync_pulse_in = 0;
                    }

                    if (sensor.contains("sync_pulse_in")) {
                        auto &sync_pulse_in = sensor["sync_pulse_in"];
                        msg.sync_pulse_locked = sync_pulse_in.value("locked", 0) != 0;

                        if (sync_pulse_in.contains("diagnostics")) {
                            auto &diagnostics = sync_pulse_in["diagnostics"];
                            msg.diagnostics_count = diagnostics.value("count", 0);
                            msg.diagnostics_count_unfiltered = diagnostics.value("count_unfiltered", 0);
                        } else {
                            msg.diagnostics_count = 0;
                            msg.diagnostics_count_unfiltered = 0;
                        }
                    } else {
                        msg.sync_pulse_locked = false;
                        msg.diagnostics_count = 0;
                        msg.diagnostics_count_unfiltered = 0;
                    }

                publisher_->publish(msg);

            } catch (const json::parse_error &e) {
                RCLCPP_ERROR(this->get_logger(), "JSON parse error: %s", e.what());
            } catch (const json::type_error &e) {
                RCLCPP_ERROR(this->get_logger(), "JSON type error: %s", e.what());
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Unexpected error: %s", e.what());
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        curl_easy_cleanup(curl);
    }
};
}
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ouster_http_node::OusterHttpNode)

