//
// Created by alsarmi on 04/04/22.
//

#ifndef PATH_PLANNER_RRT_STAR_PLANNER_H
#define PATH_PLANNER_RRT_STAR_PLANNER_H

#include <chrono>
#include <deque>
#include <exception>
#include <limits>
#include <memory>
#include <random>
#include <ufo/map/occupancy_map_color.h>
using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::seconds;
using std::chrono::steady_clock;

namespace globalPlanner::RRT {
template <typename Map, typename BoundingVolume, typename Coordinates,
          typename Path>
class RRTStar {

public:
  // Constructors
  RRTStar(Map *map_, double radius_);

  RRTStar(Map *map_, Coordinates const &mapMinBoundary_,
          Coordinates const &mapMaxBoundary_, double step, double epsilon_,
          double threshold_, double searchRadius_, double radius_);

  RRTStar(Map *map_, Coordinates const &mapMinBoundary_,
          Coordinates const &mapMaxBoundary_, double step, double epsilon_,
          double threshold_, double searchRadius_, long timeout_,
          double radius_);

  // Copy constructor and Copy assignment are not allowed
  RRTStar(RRTStar const &src) = delete;
  RRTStar &operator=(RRTStar const &src) = delete;
  // Move assignment and move constructor
  RRTStar(RRTStar &&src) noexcept;
  RRTStar &operator=(RRTStar &&src) noexcept;
  // Destructor
  ~RRTStar();
  // Public member functions
  void setMaxIterations(int iterations_) { timeout = iterations_; }
  void setMapBoundaries(Coordinates const &mapMinBoundary_,
                        Coordinates const &mapMaxBoundary_) {
    mapMinBoundary = mapMinBoundary_;
    mapMaxBoundary = mapMaxBoundary_;
  }
  Path *computePath(Coordinates const &start_, Coordinates const &goal_);
  RRTStar() = default;
private:
  // Default constructor for derived templates

  // Data  structure for the tree
  struct Node {
    explicit Node(Coordinates point_) : point(point_){};
    explicit Node() = default;
    Coordinates point;
    Node *parent = nullptr;
    Node *child = nullptr;
    double costToParent{0.0};
  };
  // Protected members
  std::vector<std::unique_ptr<Node>> tree;
  std::vector<Node *> nearby;
  Path path{};
  Map *map{nullptr};
  long timeout{30};         // 30 seconds default
  double epsilon{0.0};      // 0.01 mts default
  double threshold{0.0};    // 0.5 default
  double searchRadius{0.0}; // 0.3 mts default
  double deltaStep{0.0};    // in meters
  double radius{0.0};       // in meters
  Coordinates mapMinBoundary;
  Coordinates mapMaxBoundary;
  // Random number generator:
  std::mt19937 randomEngine;
  std::uniform_real_distribution<> xrand;
  std::uniform_real_distribution<> yrand;
  std::uniform_real_distribution<> zrand;
  std::uniform_real_distribution<> rnd;

  // Private methods
  // For Move Semantics
  friend void swap(RRTStar &src, RRTStar &dst) {
    using std::swap;
    swap(src.tree, dst.tree);
    swap(src.nearby, dst.nearby);
    swap(src.timeout, dst.timeout);
    swap(src.map, dst.map);
    swap(src.path, dst.path);
    swap(src.epsilon, dst.epsilon);
    swap(src.threshold, dst.threshold);
    swap(src.searchRadius, dst.searchRadius);
    swap(src.deltaStep, dst.deltaStep);
    swap(src.radius, dst.radius);
    swap(src.mapMinBoundary, dst.mapMinBoundary);
    swap(src.mapMaxBoundary, dst.mapMaxBoundary);
    swap(src.randomEngine, dst.randomEngine);
    swap(src.xrand, dst.xrand);
    swap(src.yrand, dst.yrand);
    swap(src.zrand, dst.zrand);
    swap(src.rnd, dst.rnd);
  }
  // Define map limits if they were not provided
  void getMapLimits();
  // Core component of RRTStar algorithm
  void expandTree(Node *qNear, Coordinates qNew);
  // RRTStar* components
  void rewire();
  void chooseParent(Node *qNear, Node *newVertex);
  // 3D collision checking
  // To check if point is in occupied or unknown space.
  bool isInCollision(Coordinates const &center);
  // To check if path from A to B is collision free.
  bool isInCollision(Coordinates const &goal_, Coordinates const &position_);
  void getClosestVoxel(Coordinates const &src, Coordinates &dst);
  // Random point generation with validation check( is within free space).
  Coordinates generateRandomPoint();
  // Get the nearest vertex in the tree to a Node based on Euclidean distance
  Node *nearestVertex(Node *qrand);
  // Check if path between vertices is possible, following dynamic constraints.
  Coordinates steer(Coordinates const &nearestVertex,
                    Coordinates const &randomVertex);
  // Generate the final path to be returned.
  void traceBack();
};
using Sphere = ufo::geometry::Sphere;
using Point = ufo::math::Vector3;
using ColorMap = ufo::map::OccupancyMapColor;
using NavPath = std::vector<Point>;
// Public template definitions
// Constructors
template <class Map, class BoundingVolume, class Coordinates, class Path>
RRTStar<Map, BoundingVolume, Coordinates, Path>::RRTStar(Map *map_,
                                                         double radius_)
    : map(map_), timeout(30), deltaStep(0.05), epsilon(0.01), threshold(0.5),
      searchRadius(0.3), radius(radius_),
      randomEngine((std::random_device())()),
      rnd(std::uniform_real_distribution<>(0.0, 1.0)) {
  // Get Map limits
  getMapLimits();
  // Initialize the uniform distributions for random coordinate generation
  xrand = std::uniform_real_distribution<>(-10.0, 10.0);
  yrand = std::uniform_real_distribution<>(-10.0, 10.0);
  zrand =
      std::uniform_real_distribution<>(mapMinBoundary.z(), mapMaxBoundary.z());
  // Initialize path frame_id
  std::cout << "RRTStar Planner created!" << std::endl;
}
template <class Map, class BoundingVolume, class Coordinates, class Path>
RRTStar<Map, BoundingVolume, Coordinates, Path>::RRTStar(
    Map *map_, Coordinates const &mapMinBoundary_,
    Coordinates const &mapMaxBoundary_, double step, double epsilon_,
    double threshold_, double searchRadius_, double radius_)
    : map(map_), mapMinBoundary(mapMinBoundary_),
      mapMaxBoundary(mapMaxBoundary_), deltaStep(step), epsilon(epsilon_),
      threshold(threshold_), searchRadius(searchRadius_), timeout(30),
      radius(radius_), randomEngine((std::random_device())()),
      xrand(std::uniform_real_distribution<>(mapMinBoundary.x(),
                                             mapMaxBoundary.x())),
      yrand(std::uniform_real_distribution<>(mapMinBoundary.y(),
                                             mapMaxBoundary.y())),
      zrand(std::uniform_real_distribution<>(mapMinBoundary.z(),
                                             mapMaxBoundary.z())),
      rnd(std::uniform_real_distribution<>(0.0, 1.0)) {
  // Initialize path frame_id

  std::cout << "RRTStar Planner created!" << std::endl;
}
template <class Map, class BoundingVolume, class Coordinates, class Path>
RRTStar<Map, BoundingVolume, Coordinates, Path>::RRTStar(
    Map *map_, Coordinates const &mapMinBoundary_,
    Coordinates const &mapMaxBoundary_, double step, double epsilon_,
    double threshold_, double searchRadius_, long timeout_, double radius_)
    : map(map_), mapMinBoundary(mapMinBoundary_),
      mapMaxBoundary(mapMaxBoundary_), deltaStep(step), timeout(timeout_),
      epsilon(epsilon_), threshold(threshold_), searchRadius(searchRadius_),
      radius(radius_), randomEngine((std::random_device())()),
      xrand(std::uniform_real_distribution<>(mapMinBoundary.x(),
                                             mapMaxBoundary.x())),
      yrand(std::uniform_real_distribution<>(mapMinBoundary.y(),
                                             mapMaxBoundary.y())),
      zrand(std::uniform_real_distribution<>(mapMinBoundary.z(),
                                             mapMaxBoundary.z())),
      rnd(std::uniform_real_distribution<>(0.0, 1.0)) {

  std::cout << "RRTStar Planner created!" << std::endl;
}
// Destructor
template <class Map, class BoundingVolume, class Coordinates, class Path>
RRTStar<Map, BoundingVolume, Coordinates, Path>::~RRTStar() = default;
// Move constructor and Move assignment
template <class Map, class BoundingVolume, class Coordinates, class Path>
RRTStar<Map, BoundingVolume, Coordinates, Path>::RRTStar(
    RRTStar &&src) noexcept {
  swap(src, *this);
}

template <class Map, class BoundingVolume, class Coordinates, class Path>
RRTStar<Map, BoundingVolume, Coordinates, Path> &
RRTStar<Map, BoundingVolume, Coordinates, Path>::operator=(
    RRTStar &&src) noexcept {
  swap(src, *this);
  return *this;
}
// Private method definitions
template <class Map, class BoundingVolume, class Coordinates, class Path>
bool RRTStar<Map, BoundingVolume, Coordinates, Path>::isInCollision(
    Coordinates const &center) {
  return false;
}
// Template specialization for collision checking considering a specific
// bounding volume.
template <>
bool RRTStar<
    ufo::map::OccupancyMapColor, ufo::geometry::Sphere, ufo::math::Vector3,
    std::vector<ufo::math::Vector3>>::isInCollision(Point const &center) {
  ufo::geometry::Sphere sphere(center, radius);
  // Iterate through all leaf nodes that intersects the bounding volume
  // Check intersection with          //occupied/free/unknown space
  for (auto it = map->beginLeaves(sphere, true, false, false, false, 0),
            it_end = map->endLeaves();
       it != it_end; ++it) {
    // Is in collision since a leaf node intersects the bounding volume.
    return true;
  }
  // No leaf node intersects the bounding volume.
  return false;
}
template <class Map, class BoundingVolume, class Coordinates, class Path>
bool RRTStar<Map, BoundingVolume, Coordinates, Path>::isInCollision(
    Coordinates const &goal_, Coordinates const &position_) {
  Coordinates direction = goal_ - position_;
  Coordinates center = position_ + (direction / 2.0);
  double distance = direction.norm();
  direction /= distance; // Distance normalization
  // Calculate yaw, pitch and roll for oriented bounding box
  double yaw = -atan2(direction[1], direction[0]);
  double pitch = -asin(direction[2]);
  double roll = 0;
  // Create an oriented bounding box between position and goal, with a size
  // radius.
  ufo::geometry::OBB obb(center,
                         ufo::math::Vector3(distance / 2.0, radius, radius),
                         ufo::math::Quaternion(roll, pitch, yaw));

  // Iterate through all leaf nodes that intersects the bounding volume
  // Check intersection with       //occupied/free/unknown space
  for (auto it = map->beginLeaves(obb, true, false, false, false, 0),
            it_end = map->endLeaves();
       it != it_end; ++it) {
    // Is in collision since a leaf node intersects the bounding volume.
    return true;
  }

  // No leaf node intersects the bounding volume.
  return false;
}
template <class Map, class BoundingVolume, class Coordinates, class Path>
void RRTStar<Map, BoundingVolume, Coordinates, Path>::getClosestVoxel(
    const Coordinates &src, Coordinates &dst) {
  for (auto it = map->beginNNLeaves(src, false, true, false, false, 0),
            it_end = map->endNNLeaves();
       it != it_end; ++it) {
    dst[0] = it.getX();
    dst[1] = it.getY();
    dst[2] = it.getZ();
    break;
  }
}

template <class Map, class BoundingVolume, class Coordinates, class Path>
void RRTStar<Map, BoundingVolume, Coordinates, Path>::getMapLimits() {
  getClosestVoxel(map->getMin(), mapMinBoundary);
  getClosestVoxel(map->getMax(), mapMaxBoundary);
}
template <class Map, class BoundingVolume, class Coordinates, class Path>
Coordinates
RRTStar<Map, BoundingVolume, Coordinates, Path>::generateRandomPoint() {
  Coordinates rndPoint(xrand(randomEngine), yrand(randomEngine),
                       zrand(randomEngine));

  rndPoint[0] = xrand(randomEngine);
  rndPoint[1] = yrand(randomEngine);
  rndPoint[2] = zrand(randomEngine);

  return rndPoint;
}
template <class Map, class BoundingVolume, class Coordinates, class Path>
typename RRTStar<Map, BoundingVolume, Coordinates, Path>::Node *
RRTStar<Map, BoundingVolume, Coordinates, Path>::nearestVertex(
    RRTStar::Node *qrand) {

  double previousDistance{std::numeric_limits<double>::max()};
  double currentDistance{0.0};
  Node *nearest = nullptr;
  // We could use the in-built Point difference and norm method as well.
  auto norm = [&](const auto &A, const auto &B) mutable {
    currentDistance = std::sqrt(((A.x() - B.x()) * (A.x() - B.x())) +
                                ((A.y() - B.y()) * (A.y() - B.y())) +
                                ((A.z() - B.z()) * (A.z() - B.z())));
  };
  for (auto &vertex : tree) {
    // Compute the euclidean distance to the random point
    norm(qrand->point, vertex->point);
    if (currentDistance < previousDistance) {
      previousDistance = currentDistance;
      nearest = vertex.get();
    }
  }
  return nearest;
}

template <class Map, class BoundingVolume, class Coordinates, class Path>
Coordinates RRTStar<Map, BoundingVolume, Coordinates, Path>::steer(
    Coordinates const &nearestVertex, Coordinates const &randomVertex) {
  // Assuming holonomic motion model.
  Coordinates difference = randomVertex - nearestVertex;
  double norm = difference.norm();
  if (norm > deltaStep) {
    // Random point is beyond our step size
    difference /= norm; // unit vector;
    return nearestVertex +
           difference * deltaStep; // return a point at step distance in
                                   // direction of the random point.
  }
  return randomVertex;
}
template <class Map, class BoundingVolume, class Coordinates, class Path>
void RRTStar<Map, BoundingVolume, Coordinates, Path>::chooseParent(
    Node *qNear, Node *newVertex) {
  double cost{0.0};
  double limit =
      searchRadius *
      std::pow(std::log((double)tree.size()) / (double)tree.size(), 1.0 / 3.0);
  nearby.clear();
  auto norm = [](const auto &A, const auto &B) {
    return std::sqrt(((A.x() - B.x()) * (A.x() - B.x())) +
                     ((A.y() - B.y()) * (A.y() - B.y())) +
                     ((A.z() - B.z()) * (A.z() - B.z())));
  };
  for (auto &vertex : tree)
    if (vertex.get() != qNear && norm(vertex->point, newVertex->point) <= limit)
      nearby.emplace_back(vertex.get());
  for (auto &vertex : nearby)
    if (!isInCollision(vertex->point, newVertex->point)) {
      cost = norm(vertex->point, newVertex->point) + vertex->costToParent;
      if (cost < newVertex->costToParent) {
        newVertex->parent = vertex;
        vertex->child = newVertex;
        newVertex->costToParent = cost;
      }
    }
}
template <class Map, class BoundingVolume, class Coordinates, class Path>
void RRTStar<Map, BoundingVolume, Coordinates, Path>::rewire() {
  double cost{0.0};
  auto norm = [](const auto &A, const auto &B) {
    return std::sqrt(((A.x() - B.x()) * (A.x() - B.x())) +
                     ((A.y() - B.y()) * (A.y() - B.y())) +
                     ((A.z() - B.z()) * (A.z() - B.z())));
  };
  auto qNew = tree.back().get();
  for (auto &vertex : nearby) {
    cost = qNew->costToParent + norm(qNew->point, vertex->point);
    if (cost < vertex->costToParent) {
      qNew->child = vertex;
      vertex->parent->child = nullptr;
      vertex->parent = qNew;
      vertex->costToParent = cost;
    }
  }
}

template <class Map, class BoundingVolume, class Coordinates, class Path>
void RRTStar<Map, BoundingVolume, Coordinates, Path>::expandTree(
    Node *qNear, Coordinates qNew) {

  auto norm = [](const auto &A, const auto &B) {
    return std::sqrt(((A.x() - B.x()) * (A.x() - B.x())) +
                     ((A.y() - B.y()) * (A.y() - B.y())) +
                     ((A.z() - B.z()) * (A.z() - B.z())));
  };
  std::unique_ptr<Node> newVertex = std::make_unique<Node>(std::move(qNew));
  qNear->child = newVertex.get();
  newVertex->parent = qNear;
  newVertex->costToParent = norm(qNear->point, newVertex->point) +
                            qNear->costToParent; // cost so far.
  // Look for the nearest neighbor vertices of the new point and choose the best
  // parent based on cost.
  chooseParent(qNear, newVertex.get());
  tree.emplace_back(std::move(newVertex));
  // Rewire the tree
  rewire();
}
template <class Map, class BoundingVolume, class Coordinates, class Path>
void RRTStar<Map, BoundingVolume, Coordinates, Path>::traceBack() {

  auto node = tree.back().get(); // Get goal

  while (node != nullptr) { // get the path in one vector
    path.emplace_back(node->point);
    node = node->parent;
  }
  std::reverse(path.begin(), path.end());
}

template <class Map, class BoundingVolume, class Coordinates, class Path>
Path *RRTStar<Map, BoundingVolume, Coordinates, Path>::computePath(
    Coordinates const &start_, Coordinates const &goal_) {
  // Main RRTStar algorithm body
  try {
    if (isInCollision(start_))
      throw std::runtime_error(
          "Start coordinates are in Occupied or Unknown space!");
    if (isInCollision(goal_))
      throw std::runtime_error(
          "Goal coordinates are in Occupied or Unknown space!");
  } catch (const std::exception &e) {
    std::cout << "Cannot compute path, reason: " << e.what() << std::endl;
    return nullptr;
  }
  std::cout << "RRT* Path planning started" << std::endl;

  tree.clear(); // Useful if running the globalPlanner more than once.
  path.clear();

  double distanceToGoal = std::numeric_limits<double>::max();
  // Initialize nodes
  Coordinates qNew;
  Node *qNearest = nullptr;

  auto qStart = std::make_unique<Node>(start_);
  auto qRand = std::make_unique<Node>();
  tree.emplace_back(std::move(qStart)); // Add start node.
  auto start = steady_clock::now();
  auto elapsedTime = std::numeric_limits<long>::min();
  while ((elapsedTime < timeout) && distanceToGoal >= epsilon) {
    if (rnd(randomEngine) > threshold)
      qRand->point = goal_;
    else
      qRand->point = generateRandomPoint();
    qNearest = nearestVertex(qRand.get());
    qNew = steer(qNearest->point, qRand->point);
    if (!isInCollision(qNearest->point, qNew)) {
      // Add point to tree if it is collision free
      expandTree(qNearest, qNew);
      distanceToGoal = (goal_ - qNew).norm(); // Update the distance
      // left.
    }
    elapsedTime = duration_cast<seconds>(steady_clock::now() - start).count();
  }
  // We need to connect the goal to the closest vertex if distanceToGoal <=
  // epsilon and n>0
  std::unique_ptr<Node> qGoal = std::make_unique<Node>(goal_);
  qNearest = nearestVertex(qGoal.get());
  if (!isInCollision(qNearest->point,
                     qGoal->point)) { // We can get to the goal directly
    qNearest->child = qGoal.get();
    qGoal->parent = qNearest;
    tree.emplace_back(std::move(qGoal));
  } else {
    // We did not reach the goal, we return the path that was the closest.
    for (auto it = tree.begin(); it != tree.end(); it++)
      // move the closest vertex to the goal to the end of the vector,
      // so it is easier to backtrack.
      if (it->get() == qNearest)
        std::rotate(it, it + 1, tree.end());
  }
  traceBack();
  return (path.empty()) ? nullptr : &path;
}
} // namespace globalPlanner::RRT
#endif // PATH_PLANNER_RRT_STAR_PLANNER_H