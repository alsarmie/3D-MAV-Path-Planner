//
// Created by alsarmi on 04/04/22.
//

#include "rrt_planner.h"

// Public method definitions
// Constructors
template <>
RRT<ColorMap, Point, NavPath>::RRT(ColorMap &map_, double radius_)
    : map(map_), iterations(std::numeric_limits<int>::max()), deltaStep(0.20),
      radius(radius_), randomEngine(randomDevice()) {
  // Get Map limits
  getMapLimits();
  // Initialize the uniform distributions for random coordinate generation
  xrand = std::uniform_real_distribution<>(mapLBoundary.x(), mapUBoundary.x());
  yrand = std::uniform_real_distribution<>(mapLBoundary.y(), mapUBoundary.y());
  zrand = std::uniform_real_distribution<>(mapLBoundary.z(), mapUBoundary.z());
  // Initialize path frame_id
  std::cout << "RRT Planner created!" << std::endl;
}
template <>
RRT<ColorMap, Point, NavPath>::RRT(ColorMap &map_, Point const &mapMinBoundary,
                                   Point const &mapMaxBoundary, double step,
                                   double radius_)
    : map(map_), mapLBoundary(mapMinBoundary), mapUBoundary(mapMaxBoundary),
      deltaStep(step), iterations(std::numeric_limits<int>::max()),
      radius(radius_), randomEngine(randomDevice()),
      xrand(
          std::uniform_real_distribution<>(mapLBoundary.x(), mapUBoundary.x())),
      yrand(
          std::uniform_real_distribution<>(mapLBoundary.y(), mapUBoundary.y())),
      zrand(std::uniform_real_distribution<>(mapLBoundary.z(),
                                             mapUBoundary.z())) {
  // Initialize path frame_id

  std::cout << "RRT Planner created!" << std::endl;
}
template <>
RRT<ColorMap, Point, NavPath>::RRT(ColorMap &map_, Point const &mapMinBoundary,
                                   Point const &mapMaxBoundary, double step,
                                   int iterations_, double radius_)
    : map(map_), mapLBoundary(mapMinBoundary), mapUBoundary(mapMaxBoundary),
      deltaStep(step), iterations(iterations_), radius(radius_),
      randomEngine(randomDevice()), xrand(std::uniform_real_distribution<>(
                                        mapLBoundary.x(), mapUBoundary.x())),
      yrand(
          std::uniform_real_distribution<>(mapLBoundary.y(), mapUBoundary.y())),
      zrand(std::uniform_real_distribution<>(mapLBoundary.z(),
                                             mapUBoundary.z())) {
  // Initialize path frame_id

  std::cout << "RRT Planner created!" << std::endl;
}
// Destructor
template <> RRT<ColorMap, Point, NavPath>::~RRT() = default;

// Private method definitions
template <>
bool RRT<ColorMap, Point, NavPath>::isInCollision(Point const &center) {
  Sphere sphere(center, radius);
  // Iterate through all leaf nodes that intersects the bounding volume
  // Check intersection with          //occupied/free/unknown space
  for (auto it = map.beginLeaves(sphere, true, false, false, false, 0),
            it_end = map.endLeaves();
       it != it_end; ++it) {
    // Is in collision since a leaf node intersects the bounding volume.
    return true;
  }
  // No leaf node intersects the bounding volume.
  return false;
}
template <>
bool RRT<ColorMap, Point, NavPath>::isInCollision(Point const &goal_,
                                                  Point const &position_) {
  Point direction = goal_ - position_;
  Point center = position_ + (direction / 2.0);
  double distance = direction.norm();
  direction /= distance; // Distance normalization
  // Calculate yaw, pitch and roll for oriented bounding box
  double yaw = atan2(direction[1], direction[0]);
  double pitch = asin(direction[2]);
  double roll = 0;
  // Create an oriented bounding box between position and goal, with a size
  // radius.
  ufo::geometry::OBB obb(center,
                         ufo::math::Vector3(distance / 2.0, radius, radius),
                         ufo::math::Quaternion(roll, pitch, yaw));

  // Iterate through all leaf nodes that intersects the bounding volume
  // Check intersection with       //occupied/free/unknown space
  for (auto it = map.beginLeaves(obb, true, false, false, false, 0),
            it_end = map.endLeaves();
       it != it_end; ++it) {
    // Is in collision since a leaf node intersects the bounding volume.
    return true;
  }
  // No leaf node intersects the bounding volume.
  return false;
}
template <>
void RRT<ColorMap, Point, NavPath>::getClosestVoxel(const Point &src,
                                                    Point &dst) {
  for (auto it = map.beginNNLeaves(src, false, true, false, false, 0),
            it_end = map.endNNLeaves();
       it != it_end; ++it) {
    std::cout << "Limit: " << it.getX() << " " << it.getY() << " " << it.getZ()
              << std::endl;
    dst[0] = it.getX();
    dst[1] = it.getY();
    dst[2] = it.getZ();
    break;
  }
}

template <> void RRT<ColorMap, Point, NavPath>::getMapLimits() {
  getClosestVoxel(map.getMin(), mapLBoundary);
  getClosestVoxel(map.getMax(), mapUBoundary);
}
template <> Point RRT<ColorMap, Point, NavPath>::generateRandomPoint() {
  Point rndPoint(xrand(randomEngine), yrand(randomEngine), zrand(randomEngine));

  while (isInCollision(rndPoint)) {
    rndPoint[0] = xrand(randomEngine);
    rndPoint[1] = yrand(randomEngine);
    rndPoint[2] = zrand(randomEngine);
  }
  return rndPoint;
}
template <>
RRT<ColorMap, Point, NavPath>::Node *
RRT<ColorMap, Point, NavPath>::nearestVertex(RRT::Node *qrand) {

  double previousDistance{std::numeric_limits<double>::max()};
  double currentDistance{0.0};
  Node *nearest = nullptr;
  // We could use the in-built Point difference and norm method as well.
  auto norm = [&currentDistance](const auto &A, const auto &B) {
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

template <>
Point RRT<ColorMap, Point, NavPath>::steer(Point const &nearestVertex,
                                           Point const &randomVertex) {
  // Assuming holonomic motion model.
  Point difference = randomVertex - nearestVertex;
  double norm = difference.norm();
  if (norm > deltaStep) {
    // Random point is beyond our step size
    difference /= norm;            // unit vector;
    return difference * deltaStep; // return a point at step distance in
                                   // direction of the random point.
  }
  return randomVertex;
}
template <>
void RRT<ColorMap, Point, NavPath>::expandTree(Node *qNear, Point qNew) {
  //
  std::unique_ptr<Node> newVertex = std::make_unique<Node>(std::move(qNew));
  qNear->child = newVertex.get();
  newVertex->parent = qNear;
  tree.emplace_back(std::move(newVertex));
}
template <> void RRT<ColorMap, Point, NavPath>::traceBack() {
  auto node = tree.back().get(); // Get goal

  while (node != nullptr) { // get the path in one vector
    path.emplace_back(node->point);
    node = node->parent;
  }
  std::reverse(path.begin(), path.end());
}

template <>
NavPath *RRT<ColorMap, Point, NavPath>::computePath(Point start_, Point goal_) {
  // Main RRT algorithm body
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

  tree.clear(); // Useful if running the planner more than once.
  path.clear();

  double n = iterations;
  double distanceToGoal = std::numeric_limits<double>::max();
  // Initialize nodes
  Point qNew;
  Node *qNearest = nullptr;
  auto qStart = std::make_unique<Node>(std::move(start_));
  auto qRand = std::make_unique<Node>();
  tree.emplace_back(std::move(qStart)); // Add start node.

  while ((n-- > 0) && distanceToGoal >= epsilon) {
    qRand->point = generateRandomPoint();
    qNearest = nearestVertex(qRand.get());
    qNew = steer(qNearest->point, qRand->point);
    if (!isInCollision(qNearest->point,
                       qNew)) { // Add point to tree if it is collision free
      expandTree(qNearest, qNew);
      distanceToGoal = (goal_ - qNew).norm(); // Update the distance
                                              // left.
    }
  }
  // We need to connect the goal to the closest vertex if distanceToGoal <=
  // epsilon and n>0
  std::unique_ptr<Node> qGoal = std::make_unique<Node>(std::move(goal_));
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
  return &path;
}