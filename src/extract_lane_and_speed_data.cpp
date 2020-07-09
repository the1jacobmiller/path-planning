#include <iostream>
#include <fstream>
#include <string>
#include <ros/ros.h>
#include <cmath>
#include <pwd.h>
#include <signal.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <dbw_mkz_msgs/SteeringReport.h>

using namespace std;

// lane data globals
std::vector<long double> lanes_lat;
std::vector<long double> lanes_lon;
std::vector<double> vehicle_speed;

string file_name;
string csv_file_name;

long double min_distance_interval = 3.0;
long double prev_lon = 0;
long double prev_lat = 0;
double current_speed = -1.0;
bool add_stop_point = false;
int accel_point_count = 10;
double max_speed = 30.0;

int g_counter = 0;

void read_configs() {
  if (!ros::param::get("/path_planning/file_name", file_name)) {
      ROS_ERROR_STREAM("Failed to read ROS configs from main");
      ros::shutdown();
  }
	csv_file_name = file_name + ".csv";
}

void writeFile (int sig) {
	ofstream myfile(csv_file_name);
	for (int i = 0; i < lanes_lon.size(); i++) {
		myfile << std::setprecision(8) << lanes_lon.at(i) << "," << lanes_lat.at(i) << "," << vehicle_speed.at(i) << "," << 0 << endl;
	}
	myfile.close();
	ros::shutdown();
	exit(0);
}

void processPosition(long double lat, long double lon) {
	long double dist = sqrt(pow(lon - prev_lon, 2) + pow(lat - prev_lat, 2));

	if (dist >= min_distance_interval) {
			lanes_lon.push_back(lon);
			lanes_lat.push_back(lat);

			double csv_speed;
			if (add_stop_point) {
				vehicle_speed.push_back(0.0);
				add_stop_point = false;
				csv_speed = 0.0;
			}
			else {
				double vehicle_rounded_speed;
				int remainder = (int)current_speed % 5;
				int q = (int)current_speed / 5;
				if (remainder > 2) {
					vehicle_rounded_speed = q * 5 + 5;
				}
				else {
					vehicle_rounded_speed = q * 5;
				}

				if (vehicle_rounded_speed == 0) vehicle_rounded_speed = 5;
				vehicle_speed.push_back(vehicle_rounded_speed);
				csv_speed = vehicle_rounded_speed;
			}

			printf("%d. lon %Lf, lat %Lf, speed %f\n", g_counter, lon, lat, csv_speed);
			g_counter++;

			prev_lon = lon;
			prev_lat = lat;
	}
}

void recvPosition(const geometry_msgs::PoseStamped msg) {
	if(current_speed == -1.0) return;
	geometry_msgs::Pose pose = msg.pose;
	geometry_msgs::Point position = pose.position;
	geometry_msgs::Quaternion orientation = pose.orientation;

	// translating pose position back to lat and lon.
	long double lon = position.x; //((position.x - lon_translation)/lon_mult_factor) - center_lon;
	long double lat = position.y; //((position.y - lat_translation)/lat_mult_factor) - center_lat;

	processPosition(lat, lon);
}

void recvSteeringReport(const dbw_mkz_msgs::SteeringReport::ConstPtr& msg) {
	// updates current_speed global
	current_speed = msg->speed * 2.23694;
	if (!vehicle_speed.empty() && current_speed == 0.0 && vehicle_speed[vehicle_speed.size()-1] != 0.0){
	 add_stop_point = true;
	}
}

int main( int argc, char** argv ){
	// getting config settings
	ros::init(argc, argv, "extract_lane_data", ros::init_options::NoSigintHandler);
	signal(SIGINT, writeFile);

	ros::NodeHandle n;
	ros::Rate r(30);

	read_configs();
	cout<<"Writing lane data to "<< csv_file_name << endl;

	ros::Subscriber sub_localization = n.subscribe("/gnss_pose", 1, recvPosition);
	ros::Subscriber sub_steering_report = n.subscribe("/vehicle/steering_report", 1, recvSteeringReport);

  ros::MultiThreadedSpinner spinner(1);
  spinner.spin();
	return 0;
}
