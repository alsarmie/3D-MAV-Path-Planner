/**
 *  @brief The implementation of IB-RRT* is based on Intelligent
 *  bidirectional rapidly-exploring random trees for optimal motion planning in
 *  complex cluttered environments: https://arxiv.org/pdf/1703.08944.pdf
 * */

#ifndef IB_RRT_STAR_PLANNER_H
#define IB_RRT_STAR_PLANNER_H

#include <chrono>
#include <deque>
#include <exception>
#include <limits>
#include <memory>
#include <random>
#include <ufo/map/occupancy_map_color.h>

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
  Path *computePath(Coordinates const &start_, Coordinates const &goal_);

private:
  // Data  structure for the tree
  /**
   *
   */
  struct Node {
    explicit Node(Coordinates point_) : point(point_){};
    explicit Node() = default;
    Coordinates point;
    Node *parent = nullptr;
    double costToParent{0.0};
  };
  // Private members
  std::vector<Node *> xNearA;
  std::vector<Node *> xNearB;
  std::vector<std::unique_ptr<Node>> treeA;
  std::vector<std::unique_ptr<Node>> treeB;
  std::vector<std::unique_ptr<Node>> *treeAPtr;
  std::vector<std::unique_ptr<Node>> *treeBPtr;
  std::vector<std::pair<double, Node *>> listA;
  std::vector<std::pair<double, Node *>> listB;

  Path path{};
  Map *map{nullptr};
  Coordinates mapMinBoundary;
  Coordinates mapMaxBoundary;

  bool connection_{false};
  bool insertInA{false};
  long iterations{10000}; //   default
  double sigmaF{std::numeric_limits<double>::max()};
  double searchRadius{0.0}; // in meters
  double deltaStep{0.0};    // in meters
  double radius{0.0};       // in meters

  // Random number generator:
  std::mt19937 randomEngine;
  std::uniform_real_distribution<> xrand;
  std::uniform_real_distribution<> yrand;
  std::uniform_real_distribution<> zrand;
  std::uniform_real_distribution<> rnd;

  // Private methods
  // For Move Semantics
  /**
   * @brief  Friend function that swaps the values of two RRTStar objects.
   * @param src Source IB-RRT * instance
   * @param dst Destination IB-RRT * instance
   */
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
/**
 * @brief Constructor of IB-RRT * class
 * @tparam Map Template parameter, specifies the Map type.
 * @tparam BoundingVolume Template parameter, specifies the type of bounding
 * volume to check for collisions in a map.
 * @tparam Coordinates Template parameter, specifies the type of coordinates
 * data structure (2D/3D)
 * @tparam Path Template parameter, specifies path type, default std::deque of
 * ufo::math::Vector3
 * @param map_ Pointer to map resource.
 * @param radius_ Radius of the agent (sphere). The sphere represents the volume
 * occupied by a robot/MAV.
 */
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
  std::cout << "IB-RRT * Planner created!" << std::endl;
}
/**
 * @brief Constructor of IB-RRT * class
 * @tparam Map Template parameter, specifies the Map type.
 * @tparam BoundingVolume Template parameter, specifies the type of bounding
 * volume to check for collisions in a map.
 * @tparam Coordinates Template parameter, specifies the type of coordinates
 * data structure (2D/3D)
 * @tparam Path Template parameter, specifies path type, default std::deque of
 * ufo::math::Vector3
 * @param map_ Pointer to map resource.
 * @param mapMinBoundary_ 3D boundaries of provided map (x,y,z)
 * @param mapMaxBoundary_ 3D boundaries of the provided map (x,y,z)
 * @param step Exploration step (in mts) for IB-RRT * algorithm.
 * @param searchRadius_ Radius used to search for near vertices, centered at
 * random sample.
 * @param radius_ Radius of the agent (sphere). The sphere represents the volume
 * occupied by a robot/MAV.
 * @param iterations_ maximum number of iterations for the IB-RRT * planner to
 * run.
 */
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

  std::cout << "IB-RRT * Planner created!" << std::endl;
}
/**
 * @brief Constructor of IB-RRT * class.
 * @tparam Map Template parameter, specifies the Map type.
 * @tparam BoundingVolume Template parameter, specifies the type of bounding
 * volume to check for collisions in a map.
 * @tparam Coordinates Template parameter, specifies the type of coordinates
 * data structure (2D/3D)
 * @tparam Path Template parameter, specifies path type, default std::deque of
 * ufo::math::Vector3
 * @param map_ Pointer to map resource.
 * @param mapMinBoundary_ 3D boundaries of provided map (x,y,z)
 * @param mapMaxBoundary_ 3D boundaries of the provided map (x,y,z)
 * @param step Exploration step (in mts) for IB-RRT * algorithm.
 * @param searchRadius_ Radius used to search for near vertices, centered at
 * random sample.
 * @param radius_ Radius of the agent (sphere). The sphere represents the volume
 * occupied by a robot/MAV.
 */
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

  std::cout << "IB-RRT * Planner created!" << std::endl;
}
// Destructor
/**
 * @brief Default destructor of IB-RRT * class.
 * @tparam Map Template parameter, specifies the Map type.
 * @tparam BoundingVolume Template parameter, specifies the type of bounding
 * volume to check for collisions in a map.
 * @tparam Coordinates Template parameter, specifies the type of coordinates
 * data structure (2D/3D)
 * @tparam Path Template parameter, specifies path type, default std::deque of
 * ufo::math::Vector3
 */
template <class Map, class BoundingVolume, class Coordinates, class Path>
IBRRT<Map, BoundingVolume, Coordinates, Path>::~IBRRT() = default;
// Move constructor and Move assignment
/**
 * @brief Move constructor for IB-RRT * class
 * @tparam Map Template parameter, specifies the Map type.
 * @tparam BoundingVolume Template parameter, specifies the type of bounding
 * volume to check for collisions in a map.
 * @tparam Coordinates Template parameter, specifies the type of coordinates
 * data structure (2D/3D)
 * @tparam Path Template parameter, specifies path type, default std::deque of
 * ufo::math::Vector3
 * @param src Source R-value reference object of type  IB-RRT *.
 */
template <class Map, class BoundingVolume, class Coordinates, class Path>
IBRRT<Map, BoundingVolume, Coordinates, Path>::IBRRT(IBRRT &&src) noexcept {
  swap(src, *this);
}
/**
 * @brief Move assignment operator for the  IB-RRT * class.
 * @tparam Map Template parameter, specifies the Map type.
 * @tparam BoundingVolume Template parameter, specifies the type of bounding
 * volume to check for collisions in a map.
 * @tparam Coordinates Template parameter, specifies the type of coordinates
 * data structure (2D/3D)
 * @tparam Path Template parameter, specifies path type, default std::deque of
 * ufo::math::Vector3
 * @param src Source R-value reference object of type  IB-RRT *.
 * @return Reference to new object of class  IB-RRT *
 */
template <class Map, class BoundingVolume, class Coordinates, class Path>
IBRRT<Map, BoundingVolume, Coordinates, Path> &
IBRRT<Map, BoundingVolume, Coordinates, Path>::operator=(IBRRT &&src) noexcept {
  swap(src, *this);
  return *this;
}
// Private method definitions
/**
 * @brief Finding the closest voxel to the source point.
 * @tparam Map Template parameter, specifies the Map type.
 * @tparam BoundingVolume Template parameter, specifies the type of bounding
 * volume to check for collisions in a map.
 * @tparam Coordinates Template parameter, specifies the type of coordinates
 * data structure (2D/3D)
 * @tparam Path Template parameter, specifies path type, default std::deque of
 * ufo::math::Vector3
 * @param src Origin Coordinate
 * @param dst Coordinate of free space type closest to the src coordinate.
 */
template <class Map, class BoundingVolume, class Coordinates, class Path>
void IBRRT<Map, BoundingVolume, Coordinates, Path>::getClosestVoxel(
    const Coordinates &src, Coordinates &dst) {
  // Finding the nearest neighbor of src in the map.
  for (auto it = map->beginNNLeaves(src, false, true, false, false, 0),
            it_end = map->endNNLeaves();
       it != it_end; ++it) {
    dst[0] = it.getX();
    dst[1] = it.getY();
    dst[2] = it.getZ();
    break;
  }
}

/**
 * @brief This function tries to get the map boundaries from the UfoMap.
 * @tparam Map Template parameter, specifies the Map type.
 * @tparam BoundingVolume Template parameter, specifies the type of bounding
 * volume to check for collisions in a map.
 * @tparam Coordinates Template parameter, specifies the type of coordinates
 * data structure (2D/3D)
 * @tparam Path Template parameter, specifies path type, default std::deque of
 * ufo::math::Vector3
 */
template <class Map, class BoundingVolume, class Coordinates, class Path>
void IBRRT<Map, BoundingVolume, Coordinates, Path>::getMapLimits() {
  getClosestVoxel(map->getMin(), mapMinBoundary);
  getClosestVoxel(map->getMax(), mapMaxBoundary);
}

// Template  for collision checking considering a specific
// bounding volume.
/**
 * @brief Collision checking function considering a
 * specific bounding volume.
 * @tparam Map Template parameter, specifies the Map type.
 * @tparam BoundingVolume Template parameter, specifies the type of bounding
 * volume to check for collisions in a map.
 * @tparam Coordinates Template parameter, specifies the type of coordinates
 * data structure (2D/3D)
 * @tparam Path Template parameter, specifies path type, default std::deque of
 * ufo::math::Vector3
 * @param center Coordinate of the center of a sphere of radius R.
 * @return Boolean. True if there is a collision with occupied space.
 */
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
/**
 * @brief Checks if the linear path between the goal and the position is
 * in collision with occupied space.
 * @tparam Map Template parameter, specifies the Map type.
 * @tparam BoundingVolume Template parameter, specifies the type of bounding
 * volume to check for collisions in a map.
 * @tparam Coordinates Template parameter, specifies the type of coordinates
 * data structure (2D/3D)
 * @tparam Path Template parameter, specifies path type, default std::deque of
 * ufo::math::Vector3
 * @param goal_ Target coordinate
 * @param position_ Current coordinate
 * @return Boolean. True if there is a collision with occupied space along the
 * trajectory between position_ and goal_.
 */
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
/*
 * Utilities
 * */
/**
 * @brief This function returns the Euclidean distance between two
 * Coordinates x1 and x2 in 3D space.
 * @tparam Map Template parameter, specifies the Map type.
 * @tparam BoundingVolume Template parameter, specifies the type of bounding
 * volume to check for collisions in a map.
 * @tparam Coordinates Template parameter, specifies the type of coordinates
 * data structure (2D/3D)
 * @tparam Path Template parameter, specifies path type, default std::deque of
 * ufo::math::Vector3
 * @param x1 Coordinate in 3D space.
 * @param x2 Coordinate in 3D space.
 * @return Distance as a double type number.
 */
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
/**
 * @brief Main sample function used in B-RRT * algorithm. Returns an
 * independent and uniformly distributed random sample from the map space, i.e.,
 * xrand ∈ Xmap.
 * @tparam Map Template parameter, specifies the Map type.
 * @tparam BoundingVolume Template parameter, specifies the type of bounding
 * volume to check for collisions in a map.
 * @tparam Coordinates Template parameter, specifies the type of coordinates
 * data structure (2D/3D)
 * @tparam Path Template parameter, specifies path type, default std::deque of
 * ufo::math::Vector3
 * @return Independent and uniformly distributed random sample from the map free
 * space.
 */
template <class Map, class BoundingVolume, class Coordinates, class Path>
Coordinates IBRRT<Map, BoundingVolume, Coordinates, Path>::sample() {
  return {xrand(randomEngine), yrand(randomEngine), zrand(randomEngine)};
}
/**
 * @brief This function returns the nearest vertex in the tree from any
given state x ∈ X. Given the tree T = (V, E), the nearest vertex procedure can
be defined as: Nearest(T, x) := argminv∈V d(x, v) |→ xmin.
 * @tparam Map Template parameter, specifies the Map type.
 * @tparam BoundingVolume Template parameter, specifies the type of bounding
 * volume to check for collisions in a map.
 * @tparam Coordinates Template parameter, specifies the type of coordinates
 * data structure (2D/3D)
 * @tparam Path Template parameter, specifies path type, default std::deque of
 * ufo::math::Vector3
 * @param xRand Coordinate pointer to the random vertex generated by the sample
* process of IB-RRT * algorithm.
 */
template <class Map, class BoundingVolume, class Coordinates, class Path>
void IBRRT<Map, BoundingVolume, Coordinates, Path>::nearestVertex(
    Coordinates *xRand) {
  // Finding the nearest node to the random point in each tree.
  xNearA.clear();
  xNearB.clear();
  auto nearest = [&](const auto *tree, auto &xNear) mutable {
    Node *nearest{nullptr};
    double previousDistance{std::numeric_limits<double>::max()};
    double currentDistance{0.0};
    for (auto &vertex : *tree) {
      // Compute the euclidean distance to the random point
      currentDistance = norm(*xRand, vertex->point);
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
/**
 * @brief This function populates the nearby A or nearby B std::vector
 * with  the set of near vertices to a given sample coordinate (xRand) within a
 * ball of radius R centered at xRand.
 * @tparam Map Template parameter, specifies the Map type.
 * @tparam BoundingVolume Template parameter, specifies the type of bounding
 * volume to check for collisions in a map.
 * @tparam Coordinates Template parameter, specifies the type of coordinates
 * data structure (2D/3D)
 * @tparam Path Template parameter, specifies path type, default std::deque of
 * ufo::math::Vector3
 * @param xRand  Random coordinate pointer, coordinate  generated by the sample
 * function.
 */
template <class Map, class BoundingVolume, class Coordinates, class Path>
void IBRRT<Map, BoundingVolume, Coordinates, Path>::nearVertices(
    Coordinates *xRand) {
  // Finding the near vertices to the random point within a ball or radius R.
  xNearA.clear();
  xNearB.clear();
  auto limit = [&](const auto &size) {
    return searchRadius *
           std::pow(std::log((double)size) / (double)size, 1.0 / 3.0);
  };
  auto near = [&](const auto *tree, auto &xNear) mutable {
    for (auto &vertex : *tree)
      // If the point is within the search sphere centered at xRand
      if (norm(vertex->point, *xRand) <= limit(tree->size()))
        xNear.emplace_back(vertex.get());
  };
  near(treeAPtr, xNearA);
  near(treeBPtr, xNearB);
}
/**
 * @brief Sorts the list of nearby nodes to xRand in xNearA (Tree A) and
 * xNearB (Tree B), based on ascending cost to xRand.
 * @tparam Map Template parameter, specifies the Map type.
 * @tparam BoundingVolume Template parameter, specifies the type of bounding
 * volume to check for collisions in a map.
 * @tparam Coordinates Template parameter, specifies the type of coordinates
 * data structure (2D/3D)
 * @tparam Path Template parameter, specifies path type, default std::deque of
 * ufo::math::Vector3
 * @param xRand Random coordinate pointer, coordinate  generated by the sample
 * function.
 */
template <typename Map, typename BoundingVolume, typename Coordinates,
          typename Path>
void IBRRT<Map, BoundingVolume, Coordinates, Path>::getSortedList(
    Coordinates *xRand) {
  // Sort
  listA.clear();
  listB.clear();
  auto sortList = [&](const auto *tree, auto &list, const auto &nearby,
                      const auto *xRand) mutable {
    double cost{0.0};
    std::pair<double, Node *> n;
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
/**
 * @brief Replaces the ChooseBestParent procedure from B-RRT *. This
function computes the best parent vertex from both tree A and B for vertex
xRand. Next, The best parent vertex between tree A and tree B is selected based
on cost (distance to xRand).
 * @tparam Map Template parameter, specifies the Map type.
 * @tparam BoundingVolume Template parameter, specifies the type of bounding
 * volume to check for collisions in a map.
 * @tparam Coordinates Template parameter, specifies the type of coordinates
 * data structure (2D/3D)
 * @tparam Path Template parameter, specifies path type, default std::deque of
 * ufo::math::Vector3
 * @param connection boolean flag to signal if the function should try to
connect tree A and tree B to generate a path.
 * @param xRand Random coordinate pointer, coordinate  generated by the sample
function.
 * @return Node pointer to the best parent vertex in tree A or B for xRand.
 */
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
  } else if (xMinA != nullptr) {
    xMin = xMinA;
  } else if (xMinB != nullptr) {
    xMin = xMinB;
    insertInA = false;
  }
  return xMin;
}
/**
 * @brief Rewires the tree of vertices(Nodes). After successfully
 * inserting a new vertex into the tree, this function sets the new vertex as
 * parent of nearby vertices if the cost of traveling to the nearby vertices is
 * lower through the new vertex than through its current parent.
 * @tparam Map Template parameter, specifies the Map type.
 * @tparam BoundingVolume Template parameter, specifies the type of bounding
 * volume to check for collisions in a map.
 * @tparam Coordinates Template parameter, specifies the type of coordinates
 * data structure (2D/3D)
 * @tparam Path Template parameter, specifies path type, default std::deque of
 * ufo::math::Vector3
 * @param tree Pointer to current tree of Nodes.
 */
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
/**
 * @brief Expands current tree (A or B). Inserts xNew coordinates into a
 * new node (vertex) and into the tree. xMin is set as xNew best parent. A
 * rewire process is run on the tree to determine if the new node serves as the
 * best parent for near nodes.
 * @tparam Map Template parameter, specifies the Map type.
 * @tparam BoundingVolume Template parameter, specifies the type of bounding
 * volume to check for collisions in a map.
 * @tparam Coordinates Template parameter, specifies the type of coordinates
 * data structure (2D/3D)
 * @tparam Path Template parameter, specifies path type, default std::deque of
 * ufo::math::Vector3
 * @param xMin Pointer to the best parent vertex.
 * @param xNew Coordinates of the new Node to be added to the current tree.
 * @param tree Pointer to current tree of Nodes.
 */
template <class Map, class BoundingVolume, class Coordinates, class Path>
void IBRRT<Map, BoundingVolume, Coordinates, Path>::expandTree(
    Node *xMin, Coordinates *xNew, std::vector<std::unique_ptr<Node>> *tree) {

  std::unique_ptr<Node> newVertex =
      std::make_unique<Node>(*xNew); // expand(xNew,&xMin->point));
  newVertex->parent = xMin;
  newVertex->costToParent = norm(xMin->point, newVertex->point); // cost so far.
  tree->emplace_back(std::move(newVertex));
  // Rewire the tree
  rewire(tree);
}
/**
 * @brief Heuristic function employed by IB-RRT*, main responsible for
 * generating a path. This procedure updates the end-to-end collision-free path
between tree A and tree B through a common vertex xRand if and only if there is
collision free path between the best parent vertex on tree A with xRand and the
best parent vertex on tree B with xRand. The path is updated if the cost of
concatenated paths, c(σa|σb), is found to be less than the cost of the existing
end-to-end path c(σf). Connection between the trees is only
successful if the boolean variable connection is true.
 * @tparam Map Template parameter, specifies the Map type.
 * @tparam BoundingVolume Template parameter, specifies the type of bounding
 * volume to check for collisions in a map.
 * @tparam Coordinates Template parameter, specifies the type of coordinates
 * data structure (2D/3D)
 * @tparam Path Template parameter, specifies path type, default std::deque of
 * ufo::math::Vector3
 * @param xMinA Node pointer to the best parent of xRand vertex/coordinates in
tree A.
 * @param xMinB Node pointer to the best parent of xRand vertex/coordinates in
tree B.
 * @param xRand Random coordinate generated by the sample function, added as a
new Node to either tree A or B.
 */
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
  pathA.emplace_back(*xRand);
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
/**
 * @brief Track back the path from the provided Node pointer to the last
 * available parent.
 * @tparam Map Template parameter, specifies the Map type.
 * @tparam BoundingVolume Template parameter, specifies the type of bounding
 * volume to check for collisions in a map.
 * @tparam Coordinates Template parameter, specifies the type of coordinates
 * data structure (2D/3D)
 * @tparam Path Template parameter, specifies path type, default std::deque of
 * ufo::math::Vector3
 * @param node Pointer to a Node/vertex in tree A or B.
 * @param cost Reference to cost variable. Used to store the cost of the tracked
 * path.
 * @return  A path of points/coordinates of type Path.
 */
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
/**
 * @brief Implementation of the IB-RRT* algorithm's main body.
 * @tparam Map Template parameter, specifies the Map type.
 * @tparam BoundingVolume Template parameter, specifies the type of bounding
 * volume to check for collisions in a map.
 * @tparam Coordinates Template parameter, specifies the type of coordinates
 * data structure (2D/3D)
 * @tparam Path Template parameter, specifies path type, default std::deque of
 * ufo::math::Vector3
 * @param start_ Start coordinate (2D/3D)
 * @param goal_ Goal coordinate (2D/3D)
 * @return A pointer to generated path between start and goal coordinates.
 */
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
  int iter = iterations;
  // Main Loop
  while (iter--) {
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
  }

  return (path.empty()) ? nullptr : &path;
}

} // namespace globalPlanner::RRT

#endif