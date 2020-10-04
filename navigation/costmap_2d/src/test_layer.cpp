#include<costmap_2d/test_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(test_layer_namespace::TestLayer, costmap_2d::Layer)
//using the LETHAL cost parameter
using costmap_2d::LETHAL_OBSTACLE;
//declaring the layer namespace (ns in the launch files)
namespace test_layer_namespace
{

TestLayer::TestLayer() : dsrv_(NULL)
{
}

TestLayer::~TestLayer()
{
    if (dsrv_!=NULL)
        delete dsrv_;
}

void TestLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::TestLayerConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::TestLayerConfig>::CallbackType cb = boost::bind(
      &TestLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}


void TestLayer::reconfigureCB(costmap_2d::TestLayerConfig) &config, uint32_t level)
{
  enabled_  = config.enabled;
  distance_ = config.distance;
}
//first part of the updating algorithm : UpdateBounds
void TestLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;
//marks a point at the specified distance from the robot
  mark_x_ = robot_x + distance_*cos(robot_yaw);
  mark_y_ = robot_y + distance_*sin(robot_yaw);

  *min_x = std::min(*min_x, mark_x_);
  *min_y = std::min(*min_y, mark_y_);
  *max_x = std::max(*max_x, mark_x_);
  *max_y = std::max(*max_y, mark_y_);
}
//second part of the updating algorithm : UpdateCosts
void TestLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{//we set the value of the point 
  if (!enabled_)
    return;
  unsigned int mx;
  unsigned int my;
  if(master_grid.worldToMap(mark_x_, mark_y_, mx, my)){
    master_grid.setCost(mx, my, LETHAL_OBSTACLE);
  }
}

} // end namespace
