#ifndef IB_RRT_STAR_PLANNER_H
#define IB_RRT_STAR_PLANNER_H

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
class IBRRT {

public:
  // Constructors
  IBRRT() = default;
  IBRRT(Map *map_, double radius_);

  IBRRT(Map *map_, Coordinates const &mapMinBoundary_,
        Coordinates const &mapMaxBoundary_, double step, double searchRadius_,
        double radius_, long iterations_);

  IBRRT(Map *map_, Coordinates const &mapMinBoundary_,
        Coordinates const &mapMaxBoundary_, double step, double searchRadius_,
        double radius_);

  // Copy constructor and Copy assignment are not allowed
  IBRRT(IBRRT const &src) = delete;
  IBRRT &operator=(IBRRT const &src) = delete;
  // Move assignment and move constructor
  IBRRT(IBRRT &&src) noexcept;
  IBRRT &operator=(IBRRT &&src) noexcept;
  // Destructor
  ~IBRRT();
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
  bool connection_{false};
  bool insertInA{false};
  std::vector<Node *> xNearA;
  std::vector<Node *> xNearB;
  std::vector<std::pair<double, Node *>> listA;
  std::vector<std::pair<double, Node *>> listB;
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
  friend void swap(IBRRT &src, IBRRT &dst) {
    using std::swap;
    swap(src.treeA, dst.treeA);
    swap(src.treeB, dst.treeB);
    swap(src.treeAPtr, dst.treeAPtr);
    swap(src.treeBPtr, dst.treeBPtr);
    swap(src.listA, dst.listA);
    swap(src.listB, dst.listB);
    swap(src.xNearA, dst.xNearA);
    swap(src.xNearB, dst.xNearB);
    swap(src.insertInA, dst.insertInA);
    swap(src.connection_, dst.connection_);
    swap(src.sigmaF, dst.sigmaF);
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
  void nearestVertex(Coordinates *xRand);
  // Get the set of nearby vertices to a point in a given radius.
  void nearVertices(Coordinates *xRand);
  // Sort nearby vertices by cost.
  void getSortedList(Coordinates *xRand);
  // Select the best parent for the xNew node based on cost-distance.
  typename IBRRT<Map, BoundingVolume, Coordinates, Path>::Node *
  getBestTreeParent(bool connection, Coordinates *xRand);
  void rewire(std::vector<std::unique_ptr<Node>> *tree);
  // Expand returns a new node xNew ∈ R^n such that xNew is closer to x2
  // than x1 in the direction from x1 to x2.
  Coordinates extend(Coordinates *x1, Coordinates *x2);
  void expandTree(Node *xMin, Coordinates *xNew,
                  std::vector<std::unique_ptr<Node>> *tree);

  void connectTrees(Node *xMinA, Node *xMinB, Coordinates *xRand);
  // 3D collision checking
  // To check if point is in occupied or unknown space.
  bool isInCollision(Coordinates const &center);
  // To check if path from A to B is collision free.
  bool isInCollision(Coordinates const &goal_, Coordinates const &position_);
  void getClosestVoxel(Coordinates const &src, Coordinates &dst);

  // Generate the final path to be returned.
  Path traceBack(Node *node, double &cost);
  // Utilities
  double norm(Coordinates const &x1, Coordinates const &x2);
};

// Public template definitions
// Constructors
template <class Map, class BoundingVolume, class Coordinates, class Path>
IBRRT<Map, BoundingVolume, Coordinates, Path>::IBRRT(Map *map_, double radius_)
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
IBRRT<Map, BoundingVolume, Coordinates, Path>::IBRRT(
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
IBRRT<Map, BoundingVolume, Coordinates, Path>::IBRRT(
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
IBRRT<Map, BoundingVolume, Coordinates, Path>::~IBRRT() = default;
// Move constructor and Move assignment
template <class Map, class BoundingVolume, class Coordinates, class Path>
IBRRT<Map, BoundingVolume, Coordinates, Path>::IBRRT(IBRRT &&src) noexcept {
  swap(src, *this);
}

template <class Map, class BoundingVolume, class Coordinates, class Path>
IBRRT<Map, BoundingVolume, Coordinates, Path> &
IBRRT<Map, BoundingVolume, Coordinates, Path>::operator=(IBRRT &&src) noexcept {
  swap(src, *this);
  return *this;
}
// Private method definitions
template <class Map, class BoundingVolume, class Coordinates, class Path>
void IBRRT<Map, BoundingVolume, Coordinates, Path>::getClosestVoxel(
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
void IBRRT<Map, BoundingVolume, Coordinates, Path>::getMapLimits() {
  getClosestVoxel(map->getMin(), mapMinBoundary);
  getClosestVoxel(map->getMax(), mapMaxBoundary);
}

// Template  for collision checking considering a specific
// bounding volume.
template <class Map, class BoundingVolume, class Coordinates, class Path>
bool IBRRT<Map, BoundingVolume, Coordinates, Path>::isInCollision(
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
bool IBRRT<Map, BoundingVolume, Coordinates, Path>::isInCollision(
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
// Utilities
template <typename Map, typename BoundingVolume, typename Coordinates,
          typename Path>
double
IBRRT<Map, BoundingVolume, Coordinates, Path>::norm(Coordinates const &x1,
                                                    Coordinates const &x2) {
  return std::sqrt(((x1.x() - x2.x()) * (x1.x() - x2.x())) +
                   ((x1.y() - x2.y()) * (x1.y() - x2.y())) +
                   ((x1.z() - x2.z()) * (x1.z() - x2.z())));
}

/**
 *                  Core IB-RRT* components
 * */
template <class Map, class BoundingVolume, class Coordinates, class Path>
Coordinates IBRRT<Map, BoundingVolume, Coordinates, Path>::sample() {
  return {xrand(randomEngine), yrand(randomEngine), zrand(randomEngine)};
}
template <class Map, class BoundingVolume, class Coordinates, class Path>
Coordinates
IBRRT<Map, BoundingVolume, Coordinates, Path>::extend(Coordinates *x1,
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
void IBRRT<Map, BoundingVolume, Coordinates, Path>::nearestVertex(
    Coordinates *xRand) {

  xNearA.clear();
  xNearB.clear();
  auto nearest = [&](const auto *tree, auto &xNear) mutable {
    Node *nearest{nullptr};
    double previousDistance{std::numeric_limits<double>::max()};
    double currentDistance{0.0};
    for (auto &vertex : *tree) {
      // Compute the euclidean distance to the random point
      norm(*xRand, vertex->point);
      if (currentDistance < previousDistance) {
        previousDistance = currentDistance;
        nearest = vertex.get();
      }
    }
    xNear.emplace_back(std::move(nearest));
  };

  nearest(treeAPtr, xNearA);
  nearest(treeBPtr, xNearB);
}
template <class Map, class BoundingVolume, class Coordinates, class Path>
void IBRRT<Map, BoundingVolume, Coordinates, Path>::nearVertices(
    Coordinates *xRand) {

  xNearA.clear();
  xNearB.clear();
  auto limit = [&](const auto &size) {
    return searchRadius *
           std::pow(std::log((double)size) / (double)size, 1.0 / 3.0);
  };
  auto near = [&](const auto *tree, auto &xNear) mutable {
    for (auto &vertex : *tree)
      if (norm(vertex->point, *xRand) <= limit(tree->size()))
        xNear.emplace_back(vertex.get());
  };
  near(treeAPtr, xNearA);
  near(treeBPtr, xNearB);
}
template <typename Map, typename BoundingVolume, typename Coordinates,
          typename Path>
void IBRRT<Map, BoundingVolume, Coordinates, Path>::getSortedList(
    Coordinates *xRand) {
  listA.clear();
  listB.clear();
  double cost{0.0};
  std::pair<double, Node *> n;
  auto sortList = [&](const auto *tree, auto &list, const auto &nearby,
                      const auto *xRand) mutable {
    // Get sorted List
    for (auto &vertex : nearby) {
      cost =
          norm((*tree)[0]->point, vertex->point) + norm(vertex->point, *xRand);
      n.first = cost;
      n.second = vertex;
      list.emplace_back(std::move(n));
    }
    std::sort(list.begin(), list.end(),
              [](const auto &a, const auto &b) { return a.first < b.first; });
  };
  sortList(treeAPtr, listA, xNearA, xRand);
  sortList(treeBPtr, listB, xNearB, xRand);
}
template <class Map, class BoundingVolume, class Coordinates, class Path>
typename IBRRT<Map, BoundingVolume, Coordinates, Path>::Node *
IBRRT<Map, BoundingVolume, Coordinates, Path>::getBestTreeParent(
    bool connection, Coordinates *xRand) {
  insertInA = true;
  double minCostA{0.0};
  double minCostB{0.0};
  Node *xMin = nullptr;
  auto getBestParent = [&](const auto &list, auto &cost) mutable {
    // Choose best parent
    for (auto &vertex : list)
      if (!isInCollision(vertex.second->point, *xRand)) {
        cost = vertex.first;
        return vertex.second;
      }
    return (Node *)nullptr;
  };
  auto xMinA = getBestParent(listA, minCostA);
  auto xMinB = getBestParent(listB, minCostB);
  if (xMinA != nullptr && xMinB != nullptr) {
    if (minCostA <= minCostB) {
      xMin = xMinA;
    } else if (minCostB < minCostA) {
      xMin = xMinB;
      insertInA = false;
    }
    if (connection)
      connectTrees(xMinA, xMinB, xRand);
  }else if (xMinA != nullptr) {
    xMin = xMinA;
  } else if (xMinB != nullptr) {
    xMin = xMinB;
    insertInA = false;
  }
  return xMin;
}

template <class Map, class BoundingVolume, class Coordinates, class Path>
void IBRRT<Map, BoundingVolume, Coordinates, Path>::rewire(
    std::vector<std::unique_ptr<Node>> *tree) {
  double cost{0.0};
  auto xRand = tree->back().get();
  auto xInit = (*tree)[0].get();
  for (auto &vertex : (insertInA) ? listA : listB) {
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
void IBRRT<Map, BoundingVolume, Coordinates, Path>::expandTree(
    Node *xMin, Coordinates *xNew, std::vector<std::unique_ptr<Node>> *tree) {
  auto expand =[&](const auto *randomVertex,const auto* nearestVertex){
    Coordinates difference = *randomVertex - *nearestVertex;
    double norm = difference.norm();
    if (norm > deltaStep) {
      // Random point is beyond our step size
      difference /= norm; // unit vector;
      return *nearestVertex +
             difference * deltaStep; // return a point at step distance in
                                     // direction of the random point.
    }
    return *randomVertex;
  };
  std::unique_ptr<Node> newVertex = std::make_unique<Node>( *xNew);//expand(xNew,&xMin->point));
  newVertex->parent = xMin;
  newVertex->costToParent = norm(xMin->point, newVertex->point); // cost so far.
  tree->emplace_back(std::move(newVertex));
  // Rewire the tree
  rewire(tree);
}
template <typename Map, typename BoundingVolume, typename Coordinates,
          typename Path>
void IBRRT<Map, BoundingVolume, Coordinates, Path>::connectTrees(
    Node *xMinA, Node *xMinB, Coordinates *xRand) {
  double costA{0.0};
  double costB{0.0};
  double sigmaAB{0.0};
  Path pathA;
  Path pathB;
  pathA = traceBack(xMinA, costA);
  pathB = traceBack(xMinB, costB);
  std::reverse(pathB.begin(), pathB.end());
  pathB.emplace_front(*xRand);
  sigmaAB =
      costA + costB + norm(xMinA->point, *xRand) + norm(xMinB->point, *xRand);
  if (sigmaAB < sigmaF) { // Store new path.
    path.clear();
    sigmaF = sigmaAB;
    path.resize(pathA.size() + pathB.size());
    std::move(pathA.begin(), pathA.end(), path.begin());
    std::move(pathB.begin(), pathB.end(), path.begin() + pathA.size());
  }
}

template <class Map, class BoundingVolume, class Coordinates, class Path>
Path IBRRT<Map, BoundingVolume, Coordinates, Path>::traceBack(Node *node,
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
Path *IBRRT<Map, BoundingVolume, Coordinates, Path>::computePath(
    Coordinates const &start_, Coordinates const &goal_) {
  // Main IB-RRTStar algorithm body
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

  Node *xMin = nullptr;
  // Useful if running the globalPlanner more than once.
  treeA.clear();
  treeB.clear();
  path.clear();
  treeAPtr = &treeA;
  treeBPtr = &treeB;
  //
  auto xRand = std::make_unique<Coordinates>();
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
    connection_ = true;
    // Sample
    *xRand = sample();
    // Get the nearby vertices for both trees.
    nearVertices(xRand.get());
    if (xNearA.empty() || xNearB.empty()) {
      nearestVertex(xRand.get());
      connection_ = false;
    }
    // Sort group of near vertices.
    getSortedList(xRand.get());
    xMin = getBestTreeParent(connection_, xRand.get());
    if (xMin != nullptr)
      expandTree(xMin, xRand.get(), (insertInA) ? treeAPtr : treeBPtr);

    // elapsedTime = duration_cast<seconds>(steady_clock::now() -
    // start).count();
  }

  return (path.empty()) ? nullptr : &path;
}

} // namespace globalPlanner::RRT

#endif