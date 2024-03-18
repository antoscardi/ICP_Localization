#include "localizer2d.h"

#include "icp/eigen_icp_2d.h"

#include <ros/ros.h> //Aggiunto da me per ROS_INFO

using namespace std; //Aggiunto da me 

Localizer2D::Localizer2D()
    : _map(nullptr),
      _laser_in_world(Eigen::Isometry2f::Identity()),
      _obst_tree_ptr(nullptr) {}

/**
 * @brief Set the internal map reference and constructs the KD-Tree containing
 * obstacles coordinates for fast access.
 *
 * @param map_
 */
void Localizer2D::setMap(std::shared_ptr<Map> map_) {
  // Set the internal map pointer
  _map = map_;
  /** TODO
   * If the map is initialized, fill the _obst_vect vector with world
   * coordinates of all cells representing obstacles.
   * Finally instantiate the KD-Tree (obst_tree_ptr) on the vector.
   */
  if (_map->initialized()){
    ROS_INFO("Map size: (%d,%d)", _map->size().height, _map->size().width);
    
    for (auto row = 1; row <= _map->rows(); ++row) {
        for (auto col = 1; col <= _map->cols(); ++col) {
          int grid_element = _map->operator()(row,col);
          //ROS_INFO("Value of the cell at row:%d col:%d is: %d",row, col, grid_element); 
          // Converts  points in the grid map into world coordinates.
          cv::Point2i point_in_image(row,col);
          //ROS_INFO("Point at :%d x, and :%d y:",row, col);
          Eigen::Vector2f point_in_world = _map->grid2world(point_in_image);
          // Fill the obstacle vector with world coordinates of all cells representing obstacles.
          _obst_vect.push_back(point_in_world);
        }
    }
   /*
   vector<int8_t> grid_vec = _map->grid();
   for (int elem = 1; _map->size().height*_map->size().width; ++elem) {
    int grid_elem_value = grid_vec[elem];
    ROS_INFO("Value at i-th elem of vector:%d  is: %d", elem, grid_elem_value);
   }
   */
  }
  /** TODO
   *  Create KD-Tree
   */
  // Create KD-Tree pointer which points the custum TreeType.
  _obst_tree_ptr = make_shared<TreeType>(_obst_vect.begin(), _obst_vect.end());
  // Check the pointer has been created correctly.
  if (!_obst_tree_ptr) {
    ROS_ERROR("Failed to construct KD-Tree of obstacles");
    }
}

/**
 * @brief Set the current estimate for laser_in_world
 *
 * @param initial_pose_
 */
void Localizer2D::setInitialPose(const Eigen::Isometry2f& initial_pose_) {
   /** TODO
   * Set the current estimate for laser_in_world
   */
  _laser_in_world = initial_pose_;
  // Print current estimate.
  ROS_INFO("Localizer informed:\n%41s   | %10s\n%36.2f %2.2f %10.2f\n%36.2f %2.2f %10.2f",
         "Rotation:", "Translation:",
         _laser_in_world.linear()(0, 0), _laser_in_world.linear()(0, 1),
         _laser_in_world.translation()(0),
         _laser_in_world.linear()(1, 0), _laser_in_world.linear()(1, 1),
         _laser_in_world.translation()(1));
}

/**
 * @brief Process the input scan.
 * First creates a prediction using the current laser_in_world estimate
 *
 * @param scan_
 */
void Localizer2D::process(const ContainerType& scan_) {
  ROS_INFO("Processing scan ...");
  // TODO Use initial pose to get a synthetic scan to compare with scan_
  

  /** TODO
   * Align prediction and scan_ using ICP.
   * Set the current estimate of laser in world as initial guess (replace the
   * solver X before running ICP)
   */

  /** TODO
   * Store the solver result (X) as the new laser_in_world estimate
   *
   */
}

/**
 * @brief Set the parameters of the laser scanner. Used to predict
 * measurements.
 * These parameters should be taken from the incoming sensor_msgs::LaserScan
 * message
 *
 * For further documentation, refer to:
 * http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html
 *
 *
 * @param range_min_
 * @param range_max_
 * @param angle_min_
 * @param angle_max_
 * @param angle_increment_
 */
void Localizer2D::setLaserParams(float range_min_, float range_max_,
                                 float angle_min_, float angle_max_,
                                 float angle_increment_) {
  _range_min = range_min_;
  _range_max = range_max_;
  _angle_min = angle_min_;
  _angle_max = angle_max_;
  _angle_increment = angle_increment_;
}

/**
 * @brief Computes the predicted scan at the current laser_in_world pose
 * estimate.
 *
 * @param dest_ Output predicted scan
 */
void Localizer2D::getPrediction(ContainerType& prediction_) {
  prediction_.clear();
  /** TODO
   * To compute the prediction, query the KD-Tree and search for all points
   * around the current laser_in_world estimate.
   * You may use additional sensor's informations to refine the prediction.
   */
}