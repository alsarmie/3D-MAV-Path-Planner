//
// Created by alsarmi on 6/05/22.
//

#ifndef B_RRT_STAR_PLANNER_H
#define B_RRT_STAR_PLANNER_H

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
class BRRT {

public:
  // Constructors
  BRRT() = default;
  BRRT(Map *map_, double radius_);

  BRRT(Map *map_, Coordinates const &mapMinBoundary_,
       Coordinates const &mapMaxBoundary_, double step, double searchRadius_,
       double radius_, long iterations_);

  BRRT(Map *map_, Coordinates const &mapMinBoundary_,
       Coordinates const &mapMaxBoundary_, double step, double searchRadius_,
       double radius_);

  // Copy constructor and Copy assignment are not allowed
  BRRT(BRRT const &src) = delete;
  BRRT &operator=(BRRT const &src) = delete;
  // Move assignment and move constructor
  BRRT(BRRT &&src) noexcept;
  BRRT &operator=(BRRT &&src) noexcept;
  // Destructor
  ~BRRT();
  // Public member functions
  void setMapBoundaries(Coordinates const &mapMinBoundary_,
                        Coordinates const &mapMaxBoundary_) {
    mapMinBoundary = mapMinBoundary_;
    mapMaxBoundary = mapMaxBoundary_;
  }
  Path *computePath(Coordinates const &start_, Coordinates const &goal_);


private:
  // Data  structure for the tree
  struct Node {
    explicit Node(Coordinates point_) : point(point_){};
    explicit Node() = default;
    Coordinates point;
    Node *parent = nullptr;
    double costToParent{0.0};
  };
  // Private members
  std::vector<std::unique_ptr<Node>> treeA;
  std::vector<std::unique_ptr<Node>> treeB;
  std::vector<std::unique_ptr<Node>> *treeAPtr;
  std::vector<std::unique_ptr<Node>> *treeBPtr;

  double sigmaF{std::numeric_limits<double>::max()};

  std::vector<Node *> nearby;
  std::vector<std::pair<double, Node *>> ls;
  Path path{};
  Map *map{nullptr};
  long iterations{10000};   // 30 seconds default
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
  friend void swap(BRRT &src, BRRT &dst) {
    using std::swap;
    swap(src.treeA, dst.treeA);
    swap(src.treeB, dst.treeB);
    swap(src.treeAPtr, dst.treeAPtr);
    swap(src.treeBPtr, dst.treeBPtr);
    swap(src.ls, dst.ls);
    swap(src.nearby, dst.nearby);
    swap(src.iterations, dst.iterations);
    swap(src.map, dst.map);
    swap(src.path, dst.path);
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

  /*! RRTStar components */
  // Random point generation
  Coordinates sample();
  // Get the nearest vertex in the tree to a Node based on Euclidean distance
  Node *nearestVertex(Coordinates *xRand,
                      std::vector<std::unique_ptr<Node>> *tree);
  // Get the set of nearby vertices to a point in a given radius.
  void nearVertices(Coordinates *xRand,
                    std::vector<std::unique_ptr<Node>> *tree);
  // Select the best parent for the xNew node based on cost-distance.
  Node *chooseBestParent(Coordinates *xRand,
                         std::vector<std::unique_ptr<Node>> *tree);
  void rewire(std::vector<std::unique_ptr<Node>> *tree);
  // Expand returns a new node xNew ∈ R^n such that xNew is closer to x2
  // than x1 in the direction from x1 to x2.
  Coordinates extend(Coordinates *x1, Coordinates *x2);
  void expandTree(Node *xMin, Coordinates *xNew,
                  std::vector<std::unique_ptr<Node>> *tree);
  void connect(Coordinates *x1, Coordinates *x2,
               std::vector<std::unique_ptr<Node>> *tree);

  // 3D collision checking
  // To check if point is in occupied or unknown space.
  bool isInCollision(Coordinates const &center);
  // To check if path from A to B is collision free.
  bool isInCollision(Coordinates const &goal_, Coordinates const &position_);
  void getClosestVoxel(Coordinates const &src, Coordinates &dst);

  // Check if path between vertices is possible, following dynamic constraints.
  Coordinates steer(Coordinates const &nearestVertex,
                    Coordinates const &randomVertex);
  // Generate the final path to be returned.
  Path traceBack(Node *node, double &cost);
};

// Definitions

// Public template definitions
// Constructors
template <class Map, class BoundingVolume, class Coordinates, class Path>
BRRT<Map, BoundingVolume, Coordinates, Path>::BRRT(Map *map_, double radius_)
    : map(map_), deltaStep(0.1), iterations(10000), searchRadius(0.6),
      radius(radius_), randomEngine((std::random_device())()),
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
BRRT<Map, BoundingVolume, Coordinates, Path>::BRRT(
    Map *map_, Coordinates const &mapMinBoundary_,
    Coordinates const &mapMaxBoundary_, double step, double searchRadius_,
    double radius_, long iterations_)
    : map(map_), mapMinBoundary(mapMinBoundary_),
      mapMaxBoundary(mapMaxBoundary_), deltaStep(step),
      searchRadius(searchRadius_), iterations(iterations_), radius(radius_),
      randomEngine((std::random_device())()),
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
BRRT<Map, BoundingVolume, Coordinates, Path>::BRRT(
    Map *map_, Coordinates const &mapMinBoundary_,
    Coordinates const &mapMaxBoundary_, double step, double searchRadius_,
    double radius_)
    : map(map_), mapMinBoundary(mapMinBoundary_),
      mapMaxBoundary(mapMaxBoundary_), deltaStep(step), iterations(10000),
      searchRadius(searchRadius_), radius(radius_),
      randomEngine((std::random_device())()),
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
BRRT<Map, BoundingVolume, Coordinates, Path>::~BRRT() = default;
// Move constructor and Move assignment
template <class Map, class BoundingVolume, class Coordinates, class Path>
BRRT<Map, BoundingVolume, Coordinates, Path>::BRRT(BRRT &&src) noexcept {
  swap(src, *this);
}

template <class Map, class BoundingVolume, class Coordinates, class Path>
BRRT<Map, BoundingVolume, Coordinates, Path> &
BRRT<Map, BoundingVolume, Coordinates, Path>::operator=(BRRT &&src) noexcept {
  swap(src, *this);
  return *this;
}
// Private method definitions
template <class Map, class BoundingVolume, class Coordinates, class Path>
void BRRT<Map, BoundingVolume, Coordinates, Path>::getClosestVoxel(
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
void BRRT<Map, BoundingVolume, Coordinates, Path>::getMapLimits() {
  getClosestVoxel(map->getMin(), mapMinBoundary);
  getClosestVoxel(map->getMax(), mapMaxBoundary);
}

// Template  for collision checking considering a specific
// bounding volume.
template <class Map, class BoundingVolume, class Coordinates, class Path>
bool BRRT<Map, BoundingVolume, Coordinates, Path>::isInCollision(
    Coordinates const &center) {
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
bool BRRT<Map, BoundingVolume, Coordinates, Path>::isInCollision(
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
/**
 *                  Core IB-RRT* components
 * */
template <class Map, class BoundingVolume, class Coordinates, class Path>
Coordinates BRRT<Map, BoundingVolume, Coordinates, Path>::sample() {
  return {xrand(randomEngine), yrand(randomEngine), zrand(randomEngine)};
}
template <class Map, class BoundingVolume, class Coordinates, class Path>
Coordinates
BRRT<Map, BoundingVolume, Coordinates, Path>::extend(Coordinates *x1,
                                                     Coordinates *x2) {
  // Assuming holonomic motion model.
  Coordinates difference = *x2 - *x1;
  double norm = difference.norm();
  // Random point is beyond our step size
  difference /= norm; // unit vector;
  return *x1 + difference *
                   deltaStep; // norm *0.6 return a point closer to x2 (xRand)
                              //  in the direction from x1 to x2
}

template <class Map, class BoundingVolume, class Coordinates, class Path>
typename BRRT<Map, BoundingVolume, Coordinates, Path>::Node *
BRRT<Map, BoundingVolume, Coordinates, Path>::nearestVertex(
    Coordinates *xRand, std::vector<std::unique_ptr<Node>> *tree) {
  double previousDistance{std::numeric_limits<double>::max()};
  double currentDistance{0.0};
  Node *nearest = nullptr;
  // We could use the in-built Point difference and norm method as well.
  auto norm = [&](const auto &A, const auto &B) mutable {
    currentDistance = std::sqrt(((A.x() - B.x()) * (A.x() - B.x())) +
                                ((A.y() - B.y()) * (A.y() - B.y())) +
                                ((A.z() - B.z()) * (A.z() - B.z())));
  };
  for (auto &vertex : *tree) {
    // Compute the euclidean distance to the random point
    norm(*xRand, vertex->point);
    if (currentDistance < previousDistance) {
      previousDistance = currentDistance;
      nearest = vertex.get();
    }
  }
  return nearest;
}
template <class Map, class BoundingVolume, class Coordinates, class Path>
void BRRT<Map, BoundingVolume, Coordinates, Path>::nearVertices(
    Coordinates *xRand, std::vector<std::unique_ptr<Node>> *tree) {
  nearby.clear();
  double limit = searchRadius *
                 std::pow(std::log((double)tree->size()) / (double)tree->size(),
                          1.0 / 3.0);
  auto norm = [](const auto &A, const auto &B) {
    return std::sqrt(((A.x() - B.x()) * (A.x() - B.x())) +
                     ((A.y() - B.y()) * (A.y() - B.y())) +
                     ((A.z() - B.z()) * (A.z() - B.z())));
  };
  for (auto &vertex : *tree)
    if (norm(vertex->point, *xRand) <= limit)
      nearby.emplace_back(vertex.get());
}
template <class Map, class BoundingVolume, class Coordinates, class Path>
Coordinates BRRT<Map, BoundingVolume, Coordinates, Path>::steer(
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
typename BRRT<Map, BoundingVolume, Coordinates, Path>::Node *
BRRT<Map, BoundingVolume, Coordinates, Path>::chooseBestParent(
    Coordinates *xRand, std::vector<std::unique_ptr<Node>> *tree) {
  double cost{0.0};
  auto norm = [](const auto &A, const auto &B) {
    return std::sqrt(((A.x() - B.x()) * (A.x() - B.x())) +
                     ((A.y() - B.y()) * (A.y() - B.y())) +
                     ((A.z() - B.z()) * (A.z() - B.z())));
  };
  ls.clear();
  std::pair<double, Node *> n;
  // Get sorted List
  for (auto &vertex : nearby) {
    auto xNew = *xRand; // steer(vertex->point,
                        //*xRand); //
    cost = norm((*tree)[0]->point, vertex->point) + norm(vertex->point, xNew);
    n.first = cost;
    n.second = vertex;
    ls.emplace_back(n);
  }
  std::sort(ls.begin(), ls.end(),
            [](const auto &a, const auto &b) { return a.first < b.first; });
  // Choose best parent
  for (auto &vertex : ls)
    if (!isInCollision(vertex.second->point, *xRand))
      return vertex.second;
  return nullptr;
}

template <class Map, class BoundingVolume, class Coordinates, class Path>
void BRRT<Map, BoundingVolume, Coordinates, Path>::rewire(
    std::vector<std::unique_ptr<Node>> *tree) {
  double cost{0.0};
  auto norm = [](const auto &A, const auto &B) {
    return std::sqrt(((A.x() - B.x()) * (A.x() - B.x())) +
                     ((A.y() - B.y()) * (A.y() - B.y())) +
                     ((A.z() - B.z()) * (A.z() - B.z())));
  };
  auto xRand = tree->back().get();
  auto xInit = (*tree)[0].get();
  for (auto &vertex : ls) {
    cost = norm(xInit->point, xRand->point) +
           norm(xRand->point, vertex.second->point);
    if (cost < norm(xInit->point, vertex.second->point))
      if (!isInCollision(xRand->point, vertex.second->point)) {
        vertex.second->parent = xRand;
        vertex.second->costToParent = norm(xRand->point, vertex.second->point);
      }
  }
}

template <class Map, class BoundingVolume, class Coordinates, class Path>
void BRRT<Map, BoundingVolume, Coordinates, Path>::expandTree(
    Node *xMin, Coordinates *xNew, std::vector<std::unique_ptr<Node>> *tree) {
  auto norm = [](const auto &A, const auto &B) {
    return std::sqrt(((A.x() - B.x()) * (A.x() - B.x())) +
                     ((A.y() - B.y()) * (A.y() - B.y())) +
                     ((A.z() - B.z()) * (A.z() - B.z())));
  };
  std::unique_ptr<Node> newVertex = std::make_unique<Node>(*xNew);
  newVertex->parent = xMin;
  newVertex->costToParent = norm(xMin->point, newVertex->point); // cost so far.
  tree->emplace_back(std::move(newVertex));
  // Rewire the tree
  rewire(tree);
}
template <typename Map, typename BoundingVolume, typename Coordinates,
          typename Path>
void BRRT<Map, BoundingVolume, Coordinates, Path>::connect(
    Coordinates *x1, Coordinates *x2,
    std::vector<std::unique_ptr<Node>> *tree) {
  double costA{0.0};
  double costB{0.0};
  double sigmaNew{0.0};
  Path pathA;
  Path pathB;
  Path pathT;
  std::vector<std::thread> threads;
  auto xNew = std::make_unique<Coordinates>();
  auto norm = [](const auto &A, const auto &B) {
    return std::sqrt(((A.x() - B.x()) * (A.x() - B.x())) +
                     ((A.y() - B.y()) * (A.y() - B.y())) +
                     ((A.z() - B.z()) * (A.z() - B.z())));
  };

  *xNew = extend(x2, x1);
  // Get the nearby vertices.
  nearVertices(xNew.get(), treeBPtr);
  // Sort nearby vertices by cost and select the best parent option.
  auto xMin = chooseBestParent(x1, treeBPtr);
  if (xMin != nullptr && !isInCollision(xMin->point, *x1)) {
    // Generate a path
    /*    pathA = traceBack(treeAPtr->back().get(), costA);
        pathB = traceBack(xMin, costB);*/
    for (int i = 0; i < 2; i++)
      threads.emplace_back(std::thread([&, i]() {
        (i == 0) ? (pathA = traceBack(treeAPtr->back().get(), costA))
                 : (pathB = traceBack(xMin, costB));
      }));
    // Wait for the threads to finish
    for (auto &t : threads)
      t.join();
    sigmaNew = costA + costB + norm(xMin->point, *x1);
    std::reverse(pathB.begin(), pathB.end());
    if (sigmaNew < sigmaF) {
      path.clear();
      sigmaF = sigmaNew;
      path.resize(pathA.size() + pathB.size());
      std::move(pathA.begin(), pathA.end(), path.begin());
      std::move(pathB.begin(), pathB.end(), path.begin() + pathA.size());
    }
  }
}
template <class Map, class BoundingVolume, class Coordinates, class Path>
Path BRRT<Map, BoundingVolume, Coordinates, Path>::traceBack(Node *node,
                                                             double &cost) {
  Path p;
  // Get goal
  while (node != nullptr) { // get the path in one vector
    p.emplace_front(node->point);
    cost += node->costToParent;
    node = node->parent;
  }
  return p;
}

template <class Map, class BoundingVolume, class Coordinates, class Path>
Path *BRRT<Map, BoundingVolume, Coordinates, Path>::computePath(
    Coordinates const &start_, Coordinates const &goal_) {
  // Main B-RRTStar algorithm body
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
  std::cout << "Path planning started" << std::endl;
  // Initialize nodes
  Node *xNearest = nullptr;
  Node *xConn = nullptr;
  Node *xMin = nullptr;
  // Useful if running the globalPlanner more than once.
  treeA.clear();
  treeB.clear();
  path.clear();
  treeAPtr = &treeA;
  treeBPtr = &treeB;
  //
  auto xRand = std::make_unique<Coordinates>();
  auto xNew = std::make_unique<Coordinates>();
  auto startPoint = std::make_unique<Node>(start_);
  auto goalPoint = std::make_unique<Node>(goal_);
  //
  treeA.emplace_back(std::move(startPoint)); // Add start node.
  treeB.emplace_back(std::move(goalPoint));  // Add goal node.
  //
  sigmaF = std::numeric_limits<double>::max();
  auto start = steady_clock::now();
  auto elapsedTime = std::numeric_limits<long>::min();
  int iter = iterations;
  // Main Loop
  while (iter--) { //(elapsedTime < timeout)) {
    // Sample
    *xRand = sample();
    // Get nearest vertex
    xNearest = nearestVertex(xRand.get(), treeAPtr);
    *xNew = extend(&(xNearest->point), xRand.get());
    // Get the nearby vertices.
    nearVertices(xNew.get(), treeAPtr);
    if (nearby.empty())
      nearby.emplace_back(xNearest);
    // Sort nearby vertices by cost and select the best parent option.
    xMin = chooseBestParent(xNew.get(), treeAPtr);
    if (xMin != nullptr) {
      expandTree(xMin, xNew.get(), treeAPtr);
      xConn = nearestVertex(xNew.get(), treeBPtr);
      connect(xNew.get(), &(xConn->point), treeBPtr);
    }
    // Swap Trees
    std::swap(treeAPtr, treeBPtr);

    elapsedTime = duration_cast<seconds>(steady_clock::now() - start).count();
  }
  return (path.empty()) ? nullptr : &path;
}

} // namespace globalPlanner::RRT

#endif // B_RRT_STAR_PLANNER_H