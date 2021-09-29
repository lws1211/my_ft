#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_broadcaster");
    tf::TransformBroadcaster broadcast;
    tf::StampedTransform stamptf_;
    ros::Rate rate(50);

    
    stamptf_.frame_id_ = "map";
    stamptf_.child_frame_id_ = "odom";
    ros::NodeHandle node;
    while (ros::ok()){
        stamptf_.stamp_ = ros::Time::now();
        stamptf_.setOrigin(tf::Vector3(0.0f,0.0f,0.0f));
        stamptf_.setRotation(tf::Quaternion(0.0f,0.0f,0.0f));

        broadcast.sendTransform(stamptf_);
        ros::spinOnce();
        rate.sleep();
    }
    
}