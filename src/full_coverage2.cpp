
#include "full_coverage.hpp"


namespace lws_nav {
  full_coverage::full_coverage() : ac("move_base", true)
  {

  }
  full_coverage::full_coverage(ros::NodeHandle *nh) : nh_(*nh) , ac("move_base", true){
    goal_col = 0;
    map_client = nh_.serviceClient<nav_msgs::GetMap>("static_map2");
    status_subscriber = nh_.subscribe("/move_base/status", 1, &full_coverage::status_callback, this);
    odom_subscriber = nh_.subscribe("/odom", 1, &full_coverage::odom_callback, this);
    publish_markers = nh_.advertise<visualization_msgs::MarkerArray>("/my_markers", 1000);
    nav_msgs::GetMap map_req;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    try
    {
        transformStamped = tfBuffer.lookupTransform("odom", "map", ros::Time(0), ros::Duration(3.0));
        
    }
    catch (tf2::TransformException &ex) 
    {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    
    ROS_INFO("requesting map from map server");
    if (map_client.call(map_req))
    {
        ROS_INFO("requested map from map server");
        bool_map = parse_map(map_req.response.map);
        bool2goals(bool_map, goals, map_req.response.map);
        goals2visualize(my_marker_array, goals);
    }
    else
    {
        ROS_ERROR("Failed to call map service");
    }
    // ros::Rate rate(1);
    // while (ros::ok()){
    std::cout << "publishing markets, size of markers:"<< my_marker_array.markers.size() << std::endl;
    publish_markers.publish(my_marker_array);     
    std::cout << "publishing markets, size of markers:"<< my_marker_array.markers.size() << std::endl;
      // ros::spinOnce();
      // rate.sleep();
    // }
    std::cout << "Constructing first goal"<< "\n";
    std::cout << "x:"<<goals[goal_row][goal_col].x << ", y: "<< goals[goal_row][goal_col].y <<"\n";
    goal_odom = constructgoal(goals[goal_row][goal_col]);
    ROS_INFO("SUCCESSFULLY construct out first goal");
    
    ac.sendGoal(goal_odom, boost::bind(&full_coverage::doneCb, this, _1), MoveBaseClient::SimpleActiveCallback());
    ros::spin();
  }
  full_coverage::~full_coverage(){

  }
  
void full_coverage::status_callback(actionlib_msgs::GoalStatusArray status){
  if (status.status_list.size() > 0){
    std::cout << "status:" << static_cast<int16_t>(status.status_list[0].status ) << "\n";
  }
  else{
    // ac.sendGoal(goal, boost::bind(&full_coverage::doneCb, this, _1), MoveBaseClient::SimpleActiveCallback());
  }    
}
void full_coverage::odom_callback(nav_msgs::Odometry odometry){
  odomet = odometry;
}
void full_coverage::doneCb(const actionlib::SimpleClientGoalState& state){
  ROS_INFO("DONECB: Finished in state [%s]", state.toString().c_str());
  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
        // do something as goal was reached         
        if (switc){
          
          int last_col = goals[goal_row].size() - 1;
          
          if (!rotated){
            std::cout << "Rotating, this row size" << static_cast<int16_t>(goals[goal_row].size() ) << "\n";
            float angle = atan2(goals[goal_row][last_col].y - goals[goal_row][goal_col].y,goals[goal_row][last_col].x - goals[goal_row][goal_col].x);
            rotated = true;
            goal_odom = constructgoal(goals[goal_row][goal_col], angle);
            ac.sendGoal(goal_odom, boost::bind(&full_coverage::doneCb, this, _1), MoveBaseClient::SimpleActiveCallback());
            std::cout << "Going " << goal_odom.target_pose.pose.position.x << ", "<< goal_odom.target_pose.pose.position.y << "\n";
          }
          else{
            goal_odom = constructgoal(goals[goal_row][last_col]);
            rotated = false;
            switc = false;
            ac.sendGoal(goal_odom, boost::bind(&full_coverage::doneCb, this, _1), MoveBaseClient::SimpleActiveCallback());
            std::cout << "Going straight, x,y:" << goal_odom.target_pose.pose.position.x << ", "<< goal_odom.target_pose.pose.position.y << "\n";
            goal_row ++;
          }
  
        }        
        else {
          int last_col = goals[goal_row].size() - 1;
          if (!rotated){
            std::cout << "Rotating, this row size" << static_cast<int16_t>(goals[goal_row].size() ) << "\n";
            float angle = atan2(goals[goal_row][goal_col].y - goals[goal_row][last_col].y, goals[goal_row][goal_col].x -goals[goal_row][last_col].x );
            rotated = true;
            goal_odom = constructgoal(goals[goal_row][last_col], angle);
            ac.sendGoal(goal_odom, boost::bind(&full_coverage::doneCb, this, _1), MoveBaseClient::SimpleActiveCallback());
            std::cout << "Going " << goal_odom.target_pose.pose.position.x << ", "<< goal_odom.target_pose.pose.position.y << "\n";
          }
          else{
            goal_odom = constructgoal(goals[goal_row][goal_col]);
            rotated = false;
            switc = true;
            ac.sendGoal(goal_odom, boost::bind(&full_coverage::doneCb, this, _1), MoveBaseClient::SimpleActiveCallback());
            std::cout << "Going straight, x,y:" << goal_odom.target_pose.pose.position.x << ", "<< goal_odom.target_pose.pose.position.y << "\n";
            goal_row ++;
          }
        }
  }
  if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
  {
      // do something as goal was canceled   
      ac.sendGoal(goal_odom);
  } 

}    
move_base_msgs::MoveBaseGoal full_coverage::constructgoal(point Point, float yaw ){
  // goal.target_pose.header.frame_id = "map";
  // goal.target_pose.header.stamp = ros::Time::now();
  move_base_msgs::MoveBaseGoal my_goal;
  goal.position.x = Point.x;
  goal.position.y = -Point.y;
  goal.position.z = 2.6;

  // tf2::Quaternion rot;
  // rot.setRPY(0,0,yaw);
  // goal.target_pose.pose.orientation.x = rot.x();
  // goal.target_pose.pose.orientation.y = rot.y();
  // goal.target_pose.pose.orientation.z = rot.z();
  // goal.target_pose.pose.orientation.w = rot.w();
  tf2::doTransform(goal, geo_odom, transformStamped);
  geometry_msgs::Point stamp;
  my_goal.target_pose.header.frame_id = "odom";
    my_goal.target_pose.header.stamp = ros::Time::now();

    my_goal.target_pose.pose.position.x = geo_odom.position.x;
    my_goal.target_pose.pose.position.y = geo_odom.position.y;
    my_goal.target_pose.pose.position.z = 2.6;

    tf2::Quaternion rot;
    rot.setRPY(0, 0, yaw);

    my_goal.target_pose.pose.orientation.x = rot.x();
    my_goal.target_pose.pose.orientation.y = rot.y();
    my_goal.target_pose.pose.orientation.z = rot.z();
    my_goal.target_pose.pose.orientation.w = rot.w();
    std::cout << "goal x:"<< geo_odom.position.x << ", y: "<< geo_odom.position.y << "\n";
    return my_goal;
}   

void full_coverage::goals2visualize(visualization_msgs::MarkerArray & markers, std::vector<std::vector<point>> goals){
  std::cout << "visualize size rows:" << goals.size() << ", "<< "size cols"<< goals[0].size() << std::endl;
  int id = 0;
  for (size_t i =0; i < goals.size(); i++){
    for (size_t j =0; j < goals[i].size(); j++){
      visualization_msgs::Marker mark;
      mark.header.frame_id = "map";
      mark.header.stamp = ros::Time::now();
      mark.id = id;
      mark.type = visualization_msgs::Marker::SPHERE;
      mark.action = visualization_msgs::Marker::ADD;
      mark.scale.x = 0.05;
      mark.scale.y = 0.05;
      mark.scale.z = 0.05;
      mark.color.a = 1.0; // Don't forget to set the alpha!
      mark.color.r = 255.0;
      mark.pose.position.x = goals[i][j].x;
      mark.pose.position.y = -goals[i][j].y;
      mark.pose.position.z = 0;
      mark.pose.orientation.x = 0.0;
      mark.pose.orientation.y = 0.0;
      mark.pose.orientation.z = 0.0;
      mark.pose.orientation.w = 1.0;
      mark.lifetime = ros::Duration(0);
      markers.markers.push_back(mark);
      id++; 
    }
  }
  std::cout << "size of markers:"<< markers.markers.size() << std::endl;
}

void full_coverage::bool2goals(std::vector<std::vector<bool>> bool_mp, std::vector<std::vector<point>> & goals, nav_msgs::OccupancyGrid const& cpp_grid_){
  // std::cout << "grid rows:" << bool_mp.size() << ", "<< "grid cols"<< bool_mp[0].size() << std::endl;
  // std::cout << "height:" << cpp_grid_.info.height << ", "<<"width: "<< cpp_grid_.info.width<< "\n";
  float x, y;
  int height_centre_pix = cpp_grid_.info.height / 2; 
  int width_centre_pix = cpp_grid_.info.width / 2; 
  int step_h = cpp_grid_.info.height / bool_mp.size(); 
  int step_w = cpp_grid_.info.width / bool_mp[0].size(); 
  float resolution = cpp_grid_.info.resolution;
  for (int nrow = 0 ; nrow < bool_mp.size(); nrow ++){
    std::vector<point> tmp_poses_;
    for (int ncol = 0; ncol < bool_mp[0].size(); ncol++){
      if (bool_mp[nrow][ncol] == false){
        if (!_initialized_row) {
        _initialized_row = true;
        goal_row = nrow;
      }
        point tmp_pose_;
        x = (ncol * step_w -width_centre_pix) * resolution;
        y = (height_centre_pix - nrow*step_h) * resolution;
        tmp_pose_.x = x;
        tmp_pose_.y = y;
        tmp_poses_.push_back(tmp_pose_);
      }
    }
    goals.push_back(tmp_poses_);
  }
}
std::vector<std::vector<bool>> full_coverage::parse_map(nav_msgs::OccupancyGrid const& cpp_grid_, 
                            float robotRadius, float toolRadius){

    std::vector<std::vector<bool>> grid;
    int ix, iy, nodeRow, nodeColl;
  uint32_t nodeSize = dmax(floor(toolRadius / cpp_grid_.info.resolution), 1);  // Size of node in pixels/units
  uint32_t robotNodeSize = dmax(floor(robotRadius / cpp_grid_.info.resolution), 1);  // RobotRadius in pixels/units
  uint32_t nRows = cpp_grid_.info.height, nCols = cpp_grid_.info.width;
  int count = 0;
  ROS_INFO("nRows: %u nCols: %u nodeSize: %d", nRows, nCols, nodeSize);

  if (nRows == 0 || nCols == 0)
  {
    ROS_WARN("empty rows and columns");
  }

  float tile_size_ = nodeSize * cpp_grid_.info.resolution;  // Size of a tile in meters
  float x1 = cpp_grid_.info.origin.position.x;  // x-origin in meters
  float y1 = cpp_grid_.info.origin.position.y;  // y-origin in meters
  std::cout << "origin is at: "<< x1<<", "<< y1 << std::endl;
  for (iy = 0; iy < nRows; iy = iy + nodeSize)
  {
    std::vector<bool> gridRow;
    for (ix = 0; ix < nCols; ix = ix + nodeSize)
    {
      bool nodeOccupied = false;
      for (nodeRow = 0; (nodeRow < robotNodeSize) && ((iy + nodeRow) < nRows) && (nodeOccupied == false); ++nodeRow)
      {
        for (nodeColl = 0; (nodeColl < robotNodeSize) && ((ix + nodeColl) < nCols); ++nodeColl)
        {
          int index_grid = dmax((iy + nodeRow - ceil(static_cast<float>(robotNodeSize - nodeSize) / 2.0))
                            * nCols + (ix + nodeColl - ceil(static_cast<float>(robotNodeSize - nodeSize) / 2.0)), 0);
          if (cpp_grid_.data[index_grid] > 65 || cpp_grid_.data[index_grid] < 0)
          {
            nodeOccupied = true;
            break;
          }
        }
        if (!nodeOccupied)
        // std::cout <<"free space at:" <<iy<<"," <<ix <<"),(";
        count ++;
        
      }
      
      gridRow.push_back(nodeOccupied);
    }
    grid.push_back(gridRow);
  }
  return grid;
}
} //lws_nav