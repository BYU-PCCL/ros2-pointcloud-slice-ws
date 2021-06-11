#include "pcl_conversions/pcl_conversions.h"
#include "pcl/common/transforms.h"
#include "pcl/filters/crop_box.h"
#include "websocketpp/config/asio_no_tls_client.hpp"
#include "websocketpp/server.hpp"
#include "websocketpp/common/asio.hpp"
#include "png++/png.hpp"
#include "boost/iostreams/device/array.hpp"
#include "random"

using namespace std::chrono_literals;

std::mutex queue_mutex;
// TODO: See if we're doing some copying when adding images to the queue
std::queue<png::image<png::gray_pixel_1>> message_queue;

class CloudSliceListener : public rclcpp::Node
{
public:
	CloudSliceListener() : Node("cloud_slice")
	{
		this->declare_parameter<std::string>("cloud_topic", {});
		this->get_parameter("cloud_topic", point_cloud_topic);

		RCLCPP_INFO(this->get_logger(), "Adding subscription for point cloud at path: %s", point_cloud_topic.c_str());

		subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
		    point_cloud_topic, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)),
		    std::bind(&CloudSliceListener::point_cloud_callback, this, std::placeholders::_1));
	}

private:
	void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
		pcl::moveFromROSMsg(*msg, *pcl_cloud);
		pcl::CropBox<pcl::PointXYZRGB> boxFilter;

		// Note that this should be a little bit faster once we crop out walls and such _in the multi cloud publisher._
		// The idea is to isolate people as much as possible, and the wide open space we're targeting should make that much easier.

		// TODO: Make these parameters
		boxFilter.setMin(Eigen::Vector4f(-4.0, -1.0, 0.0, 1.0));
		boxFilter.setMax(Eigen::Vector4f(4.0, 1.0, 2.0, 1.0));

		boxFilter.setInputCloud(pcl_cloud);
		filtered_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
		boxFilter.filter(*filtered_cloud);

		publish_slice();
	}
	void publish_slice()
	{
		// TODO: Make these all parameters
		double physical_height = 3.0861;
		double physical_width = 5.4864;
		double width = 640;
		double height = 360;

		png::image<png::gray_pixel_1> image(width, height);

		for (auto point : filtered_cloud->points) {
			unsigned int x = (point.x + (physical_width/2.0)) * (width/physical_width);
			unsigned int y = (physical_height - point.z) * (height/physical_height);
			if (x >= image.get_width() || y >= image.get_height()) {
				continue;
			}
			image[y][x] = png::gray_pixel_1(1);
		}

		const std::lock_guard<std::mutex> lock(queue_mutex);
		message_queue.push(image);
	}
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription;
	std::string point_cloud_topic;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud;
};

class SliceWebSocketServer : public std::enable_shared_from_this<SliceWebSocketServer>
{
public:
	SliceWebSocketServer(websocketpp::lib::asio::io_service *io)
	{
		ws_server.init_asio(io);
		ws_server.set_open_handler(bind(&SliceWebSocketServer::on_open, this, std::placeholders::_1));
		ws_server.set_close_handler(bind(&SliceWebSocketServer::on_close, this, std::placeholders::_1));

		ws_server.set_access_channels(websocketpp::log::alevel::none);
	}

	void send_queue()
	{
		png::image<png::gray_pixel_1> image;
		{
			const std::lock_guard<std::mutex> lock(queue_mutex);
			if (message_queue.empty()) {
				return;
			}
			image = message_queue.front();
			message_queue.pop();
		}

		std::stringstream stream(std::ios::out | std::ios::binary);
		image.write_stream(stream);
		// TODO: Do this more efficiently (don't copy so much, so don't use a stringstream)
		auto string = stream.str();
		auto raw_data = string.c_str();
		
		for (auto it: connections) {
			ws_server.send(it, raw_data, string.size(), websocketpp::frame::opcode::binary);
		}
	}

	// TODO: I don't think on_{open, close} need to be open
	void on_open(websocketpp::connection_hdl hdl)
	{
		connections.insert(hdl);
	}

	void on_close(websocketpp::connection_hdl hdl)
	{
		connections.erase(hdl);
	}

	void run(uint16_t port)
	{
		ws_server.set_reuse_addr(true);
		ws_server.listen(port);
		ws_server.start_accept();
	}

	void stop() {
		ws_server.stop_listening();
	}

private:
	websocketpp::server<websocketpp::config::asio_client> ws_server;
	std::set<websocketpp::connection_hdl, std::owner_less<websocketpp::connection_hdl>> connections;
};

void run_publisher(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<SliceWebSocketServer> ws_server, boost::asio::steady_timer *t)
{
	// TODO: See if there's a better way to let rclcpp spin independently, because this code looks like it will do more blocking than we'd like
	rclcpp::spin_some(node);
	ws_server->send_queue();
	t->expires_at(t->expiry() + boost::asio::chrono::milliseconds(30));
	t->async_wait(boost::bind(run_publisher, node, ws_server, t));
}

std::shared_ptr<SliceWebSocketServer> ws_server;

void signal_handler(int signum) {
	ws_server->stop();
	exit(signum);  
}

int main(int argc, char *argv[])
{
	signal(SIGINT, signal_handler);

	rclcpp::init(argc, argv);
	auto slice_listener = std::make_shared<CloudSliceListener>();

	websocketpp::lib::asio::io_service service;

	ws_server = std::make_shared<SliceWebSocketServer>(&service);

	boost::asio::steady_timer listener_timer(service, boost::asio::chrono::milliseconds(30));
	listener_timer.async_wait(boost::bind(&run_publisher, slice_listener, ws_server, &listener_timer));

	// TODO: Make this port a ROS parameter
	ws_server->run(9002);
	service.run();

	ws_server->stop();
	service.stop();

	rclcpp::shutdown();
	return 0;
}
