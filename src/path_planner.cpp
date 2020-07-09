#include <ros/ros.h>
#include <fstream>

#include <visualization_msgs/MarkerArray.h>
#include <geographic_msgs/GeographicMap.h>

using namespace std;

struct point {
    double x;
    double y;
    double speed;

    point() {}
    point(double x_, double y_) {
        x=x_;
        y=y_;
    }
    point(double x_, double y_, double speed_) {
        x=x_;
        y=y_;
        speed=speed_;
    }
};

ros::Publisher rviz_path_pub, geographic_path_pub;
ros::Timer path_timer;
vector<point> target_path;

void process_CSV(string csv_file_name) {
  ROS_INFO_STREAM("Path Planning - Processing " << csv_file_name);
	char* pEnd;
  string::size_type sz;
	std::ifstream infile;
	infile.open(csv_file_name);

	string x,y,speed,blinker;
	while(!infile.eof()) {
		getline(infile,x, ',');
		getline(infile,y, ',');
    getline(infile,speed);

		if(strtold(x.c_str(),&pEnd)==0.0 || strtold(y.c_str(),&pEnd)==0.0) {
			break;
		}

		point new_point;
		new_point.x = strtold(x.c_str(),&pEnd);
		new_point.y = strtold(y.c_str(),&pEnd);
    new_point.speed = 0.44704 * stod(speed, &sz);
		target_path.push_back(new_point);
	}

	infile.close();
  ROS_INFO_STREAM("Path Planning - Finished processing path of size " << target_path.size());
}

void publishRvizPath() {
  // ROS_INFO_STREAM("Path Planning - Publishing VSI Path");
  visualization_msgs::Marker path_marker;
  path_marker.header.frame_id = "/world";
  path_marker.header.stamp = ros::Time::now();
  path_marker.ns = "points_and_lines";
  path_marker.action = visualization_msgs::Marker::ADD;
  path_marker.pose.orientation.w = 1.0;
  path_marker.id = 0;
  path_marker.type = visualization_msgs::Marker::POINTS;
  path_marker.scale.x = 0.8;
  path_marker.scale.y = 0.8;
  path_marker.color.r = 1;
  path_marker.color.g = 0;
  path_marker.color.b = 0;
  path_marker.color.a = 1.0;

  for (int i = 0; i < target_path.size(); i++) {
    geometry_msgs::Point p;
    p.x = target_path[i].x;
    p.y = target_path[i].y;
    p.z = 0;
    path_marker.points.push_back(p);
  }

  rviz_path_pub.publish(path_marker);
  // ROS_INFO_STREAM("Path Planning - Finsihed publishing VSI Path");
}

void publishGeographicPath() {
  geographic_msgs::GeographicMap path;

  for (auto pt : target_path) {
    geographic_msgs::WayPoint waypoint;
    waypoint.position.longitude = pt.x;
    waypoint.position.latitude = pt.y;

    geographic_msgs::KeyValue speed_prop;
    speed_prop.key = "speed";
    speed_prop.value = to_string(pt.speed);
    waypoint.props.push_back(speed_prop);

    path.points.push_back(waypoint);
  }

  geographic_path_pub.publish(path);
}

void timerCallback(const ros::TimerEvent& event) {
  publishRvizPath();
  publishGeographicPath();
}

int main(int argc, char **argv) {
  // initialize ros node
  ros::init(argc, argv, "path_planner_node");
  ros::NodeHandle n;
  ros::Rate r(30);

  string file_name, csv_file_name;
  if (!ros::param::get("/path_planning/file_name", file_name)) {
    ROS_ERROR_STREAM("Failed to read ROS configs in main");
    ros::shutdown();
  }
  csv_file_name = file_name + ".csv";
  process_CSV(csv_file_name);

  // Publishers
  rviz_path_pub = n.advertise<visualization_msgs::Marker>("/path_planning/path_marker", 1);
  geographic_path_pub = n.advertise<geographic_msgs::GeographicMap>("/path_planning/path", 1);

  path_timer = n.createTimer(ros::Duration(1.0), timerCallback);

  ros::MultiThreadedSpinner spinner(0); // use one thread for each CPU core
  spinner.spin();

}
