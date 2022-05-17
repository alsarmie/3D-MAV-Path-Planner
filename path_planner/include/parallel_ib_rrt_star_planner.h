/**

 *  @brief The implementation of Parallel IB-RRT* is based on Intelligent
 *  bidirectional rapidly-exploring random trees for optimal motion planning in
 *  complex cluttered environments: https://arxiv.org/pdf/1703.08944.pdf
 * */

#ifndef PARALLEL_IB_RRT_STAR_PLANNER_H
#define PARALLEL_IB_RRT_STAR_PLANNER_H

#include "thread_pool.h"
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
class PIBRRT {

public:
  // Constructors
  PIBRRT();
  PIBRRT(Map *map_, double radius_);

  PIBRRT(Map *map_, Coordinates const &mapMinBoundary_,
         Coordinates const &mapMaxBoundary_, double step, double searchRadius_,
         double radius_, long iterations_, int workers_);

  PIBRRT(Map *map_, Coordinates const &mapMinBoundary_,
         Coordinates const &mapMaxBoundary_, double step, double searchRadius_,
         double radius_, int workers_);

  // Copy constructor and Copy assignment are not allowed
  PIBRRT(PIBRRT const &src) = delete;
  PIBRRT &operator=(PIBRRT const &src) = delete;
  // Move assignment and move constructor
  PIBRRT(PIBRRT &&src) noexcept;
  PIBRRT &operator=(PIBRRT &&src) noexcept;
  // Destructor
  ~PIBRRT();
  // Public member functions
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

  long iterations{0}; //
  double sigmaF{std::numeric_limits<double>::max()};
  double searchRadius{0.0}; // in mts
  double deltaStep{0.0};    // in meters
  double radius{0.0};       // in meters
  double minCostA{0.0};
  double minCostB{0.0};
  Node *xMinA{nullptr};
  Node *xMinB{nullptr};
  // Concurrency members
  std::unique_ptr<ThreadPool> threadPool;
  std::condition_variable nearestVertexACV;
  std::condition_variable nearestVertexBCV;

  std::condition_variable nearVerticesACV;
  std::condition_variable nearVerticesBCV;

  std::condition_variable getSortedListACV;
  std::condition_variable getSortedListBCV;

  std::condition_variable getBestTreeParentACV;
  std::condition_variable getBestTreeParentBCV;

  std::condition_variable mapCV;

  std::atomic<bool> connection{false};
  std::atomic<bool> insertInA{false};

  std::atomic<bool> nearestVertexADone{false};
  std::atomic<bool> nearestVertexBDone{false};

  std::atomic<bool> nearVerticesADone{false};
  std::atomic<bool> nearVerticesBDone{false};

  std::atomic<bool> getSortedListADone{false};
  std::atomic<bool> getSortedListBDone{false};

  std::atomic<bool> getBestTreeParentADone{false};
  std::atomic<bool> getBestTreeParentBDone{false};

  std::atomic<bool> mapDone{true};
  // Necessary mutexes
  std::mutex treeAMtx;
  std::mutex treeBMtx;
  std::mutex xNearAMtx;
  std::mutex xNearBMtx;
  std::mutex listAMtx;
  std::mutex listBMtx;
  std::mutex mapMtx;
  std::mutex nearestVertexMtx;
  std::mutex nearVerticesMtx;
  std::mutex getSortedListMtx;
  std::mutex getBestTreeParentMtx;

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
   * @param src Source Parallel IB-RRT * instance
   * @param dst Destination Parallel IB-RRT * instance
   */
  friend void swap(PIBRRT &src, PIBRRT &dst) {
    using std::swap;
    swap(src.treeA, dst.treeA);
    swap(src.treeB, dst.treeB);
    swap(src.treeAPtr, dst.treeAPtr);
    swap(src.treeBPtr, dst.treeBPtr);
    swap(src.listA, dst.listA);
    swap(src.listB, dst.listB);
    swap(src.xNearA, dst.xNearA);
    swap(src.xNearB, dst.xNearB);
    swap(src.sigmaF, dst.sigmaF);
    swap(src.iterations, dst.iterations);
    swap(src.map, dst.map);
    swap(src.path, dst.path);
    swap(src.searchRadius, dst.searchRadius);
    swap(src.deltaStep, dst.deltaStep);
    swap(src.radius, dst.radius);
    swap(src.minCostA, dst.minCostA);
    swap(src.minCostB, dst.minCostB);
    swap(src.xMinA, dst.xMinA);
    swap(src.xMinB, dst.xMinB);
    swap(src.mapMinBoundary, dst.mapMinBoundary);
    swap(src.mapMaxBoundary, dst.mapMaxBoundary);
    swap(src.randomEngine, dst.randomEngine);
    swap(src.xrand, dst.xrand);
    swap(src.yrand, dst.yrand);
    swap(src.zrand, dst.zrand);
    swap(src.rnd, dst.rnd);
    dst.threadPool = std::move(src.threadPool);
  }
  // Define map limits if they were not provided
  void getMapLimits();
  /*! RRTStar components */
  // Random point generation
  Coordinates sample();
  // Get the nearest vertex in the tree to a Node based on Euclidean distance
  void nearestVertex(Coordinates *xRand);
  void nearest(Coordinates xRand, bool isTreeA);
  // Get the set of nearby vertices to a point in a given radius.
  void nearVertices(Coordinates *xRand);
  void near(Coordinates xRand, double sigma, bool isTreeA);
  // Sort nearby vertices by cost.
  void getSortedList(Coordinates *xRand);
  void sortList(Coordinates xRand, bool isTreeA);
  // Select the best parent for the xNew node based on cost-distance.
  Node *getBestTreeParent(Coordinates *xRand);
  void getBestParent(Coordinates xRand, bool isTreeA);

  void rewire(std::vector<std::unique_ptr<Node>> *tree);

  void expandTree(Node *xMin, Coordinates *xNew,
                  std::vector<std::unique_ptr<Node>> *tree);

  void connectTrees(Coordinates *xRand);
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
 * @brief Constructor of Parallel IB-RRT * class
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
template <typename Map, typename BoundingVolume, typename Coordinates,
          typename Path>
PIBRRT<Map, BoundingVolume, Coordinates, Path>::PIBRRT() = default;
template <class Map, class BoundingVolume, class Coordinates, class Path>
[[maybe_unused]] PIBRRT<Map, BoundingVolume, Coordinates, Path>::PIBRRT(
    Map *map_, double radius_)
    : map(map_), deltaStep(0.1), iterations(10000), searchRadius(0.6),
      radius(radius_), randomEngine((std::random_device())()),
      threadPool(std::make_unique<ThreadPool>(7)),
      rnd(std::uniform_real_distribution<>(0.0, 1.0)) {
  // Get Map limits
  getMapLimits();
  // Initialize the uniform distributions for random coordinate generation
  xrand = std::uniform_real_distribution<>(-10.0, 10.0);
  yrand = std::uniform_real_distribution<>(-10.0, 10.0);
  zrand =
      std::uniform_real_distribution<>(mapMinBoundary.z(), mapMaxBoundary.z());

  // Initialize path frame_id
  std::cout << "Parallel IB-RRTStar Planner created!" << std::endl;
}
/**
 * @brief Constructor of Parallel IB-RRT * class
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
 * @param workers_ Number of working threads to be generated.
 */
template <class Map, class BoundingVolume, class Coordinates, class Path>
PIBRRT<Map, BoundingVolume, Coordinates, Path>::PIBRRT(
    Map *map_, Coordinates const &mapMinBoundary_,
    Coordinates const &mapMaxBoundary_, double step, double searchRadius_,
    double radius_, long iterations_, int workers_)
    : map(map_), mapMinBoundary(mapMinBoundary_),
      mapMaxBoundary(mapMaxBoundary_), deltaStep(step),
      searchRadius(searchRadius_), iterations(iterations_), radius(radius_),
      threadPool(std::make_unique<ThreadPool>(workers_)),
      randomEngine((std::random_device())()),
      xrand(std::uniform_real_distribution<>(mapMinBoundary.x(),
                                             mapMaxBoundary.x())),
      yrand(std::uniform_real_distribution<>(mapMinBoundary.y(),
                                             mapMaxBoundary.y())),
      zrand(std::uniform_real_distribution<>(mapMinBoundary.z(),
                                             mapMaxBoundary.z())),
      rnd(std::uniform_real_distribution<>(0.0, 1.0)) {
  // Initialize path frame_id
  std::cout << "Parallel IB-RRTStar Planner created!" << std::endl;
}
/**
 * @brief Constructor of ParallelIB-RRT * class.
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
 * @param workers_ Number of working threads to be generated.
 */
template <class Map, class BoundingVolume, class Coordinates, class Path>
[[maybe_unused]] PIBRRT<Map, BoundingVolume, Coordinates, Path>::PIBRRT(
    Map *map_, Coordinates const &mapMinBoundary_,
    Coordinates const &mapMaxBoundary_, double step, double searchRadius_,
    double radius_, int workers_)
    : map(map_), mapMinBoundary(mapMinBoundary_),
      mapMaxBoundary(mapMaxBoundary_), deltaStep(step), iterations(10000),
      searchRadius(searchRadius_), radius(radius_),
      threadPool(std::make_unique<ThreadPool>(workers_)),
      randomEngine((std::random_device())()),
      xrand(std::uniform_real_distribution<>(mapMinBoundary.x(),
                                             mapMaxBoundary.x())),
      yrand(std::uniform_real_distribution<>(mapMinBoundary.y(),
                                             mapMaxBoundary.y())),
      zrand(std::uniform_real_distribution<>(mapMinBoundary.z(),
                                             mapMaxBoundary.z())),
      rnd(std::uniform_real_distribution<>(0.0, 1.0)) {
  std::cout << "Parallel IB-RRTStar Planner created!" << std::endl;
}
// Destructor
/**
 * @brief Default destructor of Parallel IB-RRT * class.
 * @tparam Map Template parameter, specifies the Map type.
 * @tparam BoundingVolume Template parameter, specifies the type of bounding
 * volume to check for collisions in a map.
 * @tparam Coordinates Template parameter, specifies the type of coordinates
 * data structure (2D/3D)
 * @tparam Path Template parameter, specifies path type, default std::deque of
 * ufo::math::Vector3
 */
template <class Map, class BoundingVolume, class Coordinates, class Path>
PIBRRT<Map, BoundingVolume, Coordinates, Path>::~PIBRRT() = default;
// Move constructor and Move assignment
/**
 * @brief Move constructor for Parallel IB-RRT * class
 * @tparam Map Template parameter, specifies the Map type.
 * @tparam BoundingVolume Template parameter, specifies the type of bounding
 * volume to check for collisions in a map.
 * @tparam Coordinates Template parameter, specifies the type of coordinates
 * data structure (2D/3D)
 * @tparam Path Template parameter, specifies path type, default std::deque of
 * ufo::math::Vector3
 * @param src Source R-value reference object of type  Parallel IB-RRT *.
 */
template <class Map, class BoundingVolume, class Coordinates, class Path>
PIBRRT<Map, BoundingVolume, Coordinates, Path>::PIBRRT(PIBRRT &&src) noexcept {
  swap(src, *this);
}
/**
 * @brief Move assignment operator for the  Parallel IB-RRT * class.
 * @tparam Map Template parameter, specifies the Map type.
 * @tparam BoundingVolume Template parameter, specifies the type of bounding
 * volume to check for collisions in a map.
 * @tparam Coordinates Template parameter, specifies the type of coordinates
 * data structure (2D/3D)
 * @tparam Path Template parameter, specifies path type, default std::deque of
 * ufo::math::Vector3
 * @param src Source R-value reference object of type  Parallel IB-RRT *.
 * @return Reference to new object of class Parallel IB-RRT *
 */
template <class Map, class BoundingVolume, class Coordinates, class Path>
PIBRRT<Map, BoundingVolume, Coordinates, Path> &
PIBRRT<Map, BoundingVolume, Coordinates, Path>::operator=(
    PIBRRT &&src) noexcept {
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
void PIBRRT<Map, BoundingVolume, Coordinates, Path>::getClosestVoxel(
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
void PIBRRT<Map, BoundingVolume, Coordinates, Path>::getMapLimits() {
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
bool PIBRRT<Map, BoundingVolume, Coordinates, Path>::isInCollision(
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
bool PIBRRT<Map, BoundingVolume, Coordinates, Path>::isInCollision(
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
  std::lock_guard lck(mapMtx);
  mapDone = false;
  ufo::geometry::OBB obb(center,
                         ufo::math::Vector3(distance / 2.0, radius, radius),
                         ufo::math::Quaternion(roll, pitch, yaw));
  // Iterate through all leaf nodes that intersects the bounding volume
  // Check intersection with       //occupied/free/unknown space
  for (auto it = map->beginLeaves(obb, true, false, true, false, 0),
            it_end = map->endLeaves();
       it != it_end; ++it) {
    // Is in collision since a leaf node intersects the bounding volume.
    mapDone = true;
    mapCV.notify_one();
    return true;
  }
  mapDone = true;
  mapCV.notify_one();
  // No leaf node intersects the bounding volume.
  return false;
}
// Utilities
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
PIBRRT<Map, BoundingVolume, Coordinates, Path>::norm(Coordinates const &x1,
                                                     Coordinates const &x2) {
  return std::sqrt(((x1.x() - x2.x()) * (x1.x() - x2.x())) +
                   ((x1.y() - x2.y()) * (x1.y() - x2.y())) +
                   ((x1.z() - x2.z()) * (x1.z() - x2.z())));
}

/**
 *                  Core Parallel IB-RRT* components
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
Coordinates PIBRRT<Map, BoundingVolume, Coordinates, Path>::sample() {
  return {xrand(randomEngine), yrand(randomEngine), zrand(randomEngine)};
}
/**
 * @brief Helper function of nearestVertex. This function provides the
 * utilities of the regular IB-RRT* nearestVertex function. The main difference
 * is the thread precautions in place for the function to be used concurrently.
 * @tparam Map Template parameter, specifies the Map type.
 * @tparam BoundingVolume Template parameter, specifies the type of bounding
 * volume to check for collisions in a map.
 * @tparam Coordinates Template parameter, specifies the type of coordinates
 * data structure (2D/3D)
 * @tparam Path Template parameter, specifies path type, default std::deque of
 * ufo::math::Vector3
 * @param xRand  Coordinate pointer to the random vertex generated by the sample
 * process of IB-RRT * algorithm.
 * @param isTreeA Boolean value. Set if the function operates on tree A.
 */
template <typename Map, typename BoundingVolume, typename Coordinates,
          typename Path>
void PIBRRT<Map, BoundingVolume, Coordinates, Path>::nearest(Coordinates xRand,
                                                             bool isTreeA) {
  Node *nearest{nullptr};
  {
    std::lock_guard lck(nearestVertexMtx);
    (isTreeA) ? (nearestVertexADone = false) : (nearestVertexBDone = false);
  }
  double previousDistance{std::numeric_limits<double>::max()};
  double currentDistance{0.0};
  {
    std::lock_guard lck((isTreeA) ? treeAMtx : treeBMtx);
    auto tree = (isTreeA) ? treeAPtr : treeBPtr;
    for (auto &vertex : *tree) {
      // Compute the euclidean distance to the random point
      currentDistance = norm(xRand, vertex->point);
      if (currentDistance < previousDistance) {
        previousDistance = currentDistance;
        nearest = vertex.get();
      }
    }
  }
  {
    std::lock_guard lck((isTreeA) ? xNearAMtx : xNearBMtx);
    (isTreeA) ? xNearA.emplace_back(std::move(nearest))
              : xNearB.emplace_back(std::move(nearest));
  }
  std::lock_guard lck(nearestVertexMtx);
  (isTreeA) ? (nearestVertexADone = true) : (nearestVertexBDone = true);
  (isTreeA) ? nearestVertexACV.notify_one() : nearestVertexBCV.notify_one();
}
/**
 * @brief This function returns the nearest vertex in the tree from any
given state x ∈ X. Given the tree T = (V, E), the nearest vertex procedure can
be defined as: Nearest(T, x) := argminv∈V d(x, v) |→ xmin. nearestVertex for
tree A and tree B is computed concurrently.
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
void PIBRRT<Map, BoundingVolume, Coordinates, Path>::nearestVertex(
    Coordinates *xRand) {
  //
  {
    std::lock_guard lckA(xNearAMtx);
    xNearA.clear();
  }
  {
    std::lock_guard lckB(xNearBMtx);
    xNearB.clear();
  }
  threadPool->workOn(
      std::bind(&PIBRRT<Map, BoundingVolume, Coordinates, Path>::nearest, this,
                *xRand, true));
  threadPool->workOn(
      std::bind(&PIBRRT<Map, BoundingVolume, Coordinates, Path>::nearest, this,
                *xRand, false));
}
/**
 * @brief Helper function of nearVertices. This function provides the
 * utilities of the regular IB-RRT* nearVertices function. The main difference
 * is the thread precautions in place for the function to be used concurrently.
 * @tparam Map Template parameter, specifies the Map type.
 * @tparam BoundingVolume Template parameter, specifies the type of bounding
 * volume to check for collisions in a map.
 * @tparam Coordinates Template parameter, specifies the type of coordinates
 * data structure (2D/3D)
 * @tparam Path Template parameter, specifies path type, default std::deque of
 * ufo::math::Vector3
 * @param xRand Random coordinate pointer, coordinate  generated by the sample
 * function.
 * @param sigma Constant value used to scale the search sphere centered at
 * xRand.
 * @param isTreeA Boolean value. Set if the function operates on tree A.
 */
template <typename Map, typename BoundingVolume, typename Coordinates,
          typename Path>
void PIBRRT<Map, BoundingVolume, Coordinates, Path>::near(Coordinates xRand,
                                                          double sigma,
                                                          bool isTreeA) {
  auto limit = [](const auto &size) {
    return std::pow(std::log((double)size) / (double)size, 1.0 / 3.0);
  };
  {
    std::lock_guard lck(nearVerticesMtx);
    (isTreeA) ? (nearVerticesADone = false) : (nearVerticesBDone = false);
  }
  {
    std::lock_guard lckA((isTreeA) ? treeAMtx : treeBMtx);
    std::lock_guard lckB((isTreeA) ? xNearAMtx : xNearBMtx);
    auto tree = (isTreeA) ? treeAPtr : treeBPtr;
    for (auto &vertex : *tree)
      // If the point is within the search sphere centered at xRand
      if (norm(vertex->point, xRand) <=
          sigma * limit(((isTreeA) ? treeAPtr : treeBPtr)->size())) {
        (isTreeA) ? xNearA.emplace_back(vertex.get())
                  : xNearB.emplace_back(vertex.get());
      }
  }
  std::lock_guard lck(nearVerticesMtx);
  (isTreeA) ? (nearVerticesADone = true) : (nearVerticesBDone = true);
  (isTreeA) ? nearVerticesACV.notify_one() : nearVerticesBCV.notify_one();
}
/**
* @brief This function populates the nearby A or nearby B std::vector
* with  the set of near vertices to a given sample coordinate (xRand) within a
* ball of radius R centered at xRand. nearVertices for
tree A and tree B is computed concurrently.
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
void PIBRRT<Map, BoundingVolume, Coordinates, Path>::nearVertices(
    Coordinates *xRand) {
  {
    std::lock_guard lckA(xNearAMtx);
    xNearA.clear();
  }
  {
    std::lock_guard lckB(xNearBMtx);
    xNearB.clear();
  }
  threadPool->workOn(
      std::bind(&PIBRRT<Map, BoundingVolume, Coordinates, Path>::near, this,
                *xRand, searchRadius, true));
  threadPool->workOn(
      std::bind(&PIBRRT<Map, BoundingVolume, Coordinates, Path>::near, this,
                *xRand, searchRadius, false));
}
/**
 * @brief Helper function of getSortedList. This function provides the
 * utilities of the regular IB-RRT* nearVertices function. The main difference
 * is the thread precautions in place for the function to be used concurrently.
 * @tparam Map Template parameter, specifies the Map type.
 * @tparam BoundingVolume Template parameter, specifies the type of bounding
 * volume to check for collisions in a map.
 * @tparam Coordinates Template parameter, specifies the type of coordinates
 * data structure (2D/3D)
 * @tparam Path Template parameter, specifies path type, default std::deque of
 * ufo::math::Vector3
 *  @param xRand  Coordinate pointer to the random vertex generated by the sample
* process of IB-RRT * algorithm.
 * @param isTreeA  Boolean value. Set if the function operates on tree A.
 */
template <typename Map, typename BoundingVolume, typename Coordinates,
          typename Path>
void PIBRRT<Map, BoundingVolume, Coordinates, Path>::sortList(Coordinates xRand,
                                                              bool isTreeA) {
  // Get sorted List
  double cost{0.0};
  std::pair<double, Node *> n;
  {
    std::lock_guard lck(getSortedListMtx);
    (isTreeA) ? (getSortedListADone = false) : (getSortedListBDone = false);
  }
  {
    std::lock_guard lckA((isTreeA) ? treeAMtx : treeBMtx);
    std::lock_guard lckB((isTreeA) ? xNearAMtx : xNearBMtx);
    std::lock_guard lckC((isTreeA) ? listAMtx : listBMtx);
    auto tree = (isTreeA) ? treeAPtr : treeBPtr;
    auto xNear = (isTreeA) ? xNearA : xNearB;
    for (auto &vertex : xNear) {
      cost =
          norm((*tree)[0]->point, vertex->point) + norm(vertex->point, xRand);
      n.first = cost;
      n.second = vertex;
      (isTreeA) ? listA.emplace_back(std::move(n))
                : listB.emplace_back(std::move(n));
    }
  }
  {
    std::lock_guard lckC((isTreeA) ? listAMtx : listBMtx);
    std::sort((isTreeA) ? listA.begin() : listB.begin(),
              (isTreeA) ? listA.end() : listB.end(),
              [](const auto &a, const auto &b) { return a.first < b.first; });
  }
  std::lock_guard lck(getSortedListMtx);
  (isTreeA) ? (getSortedListADone = true) : (getSortedListBDone = true);
  (isTreeA) ? getSortedListACV.notify_one() : getSortedListBCV.notify_one();
}
/**
 * @brief Sorts the list of nearby nodes to xRand in xNearA (Tree A) and
 * xNearB (Tree B), based on ascending cost to xRand. Sorting of xNearA and
 * xNearB is performed concurrently.
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
void PIBRRT<Map, BoundingVolume, Coordinates, Path>::getSortedList(
    Coordinates *xRand) {
  {
    std::lock_guard lckA(listAMtx);
    listA.clear();
  }
  {
    std::lock_guard lckB(listBMtx);
    listB.clear();
  }
  threadPool->workOn(
      std::bind(&PIBRRT<Map, BoundingVolume, Coordinates, Path>::sortList, this,
                *xRand, true));
  threadPool->workOn(
      std::bind(&PIBRRT<Map, BoundingVolume, Coordinates, Path>::sortList, this,
                *xRand, false));
}
/**
 * @brief Helper function of getBestTreeParent. This function provides the
 * utilities of the regular IB-RRT* getBestTreeParent function. The main
 * difference is the thread precautions in place for the function to be used
 * concurrently.
 * @tparam Map Template parameter, specifies the Map type.
 * @tparam BoundingVolume Template parameter, specifies the type of bounding
 * volume to check for collisions in a map.
 * @tparam Coordinates Template parameter, specifies the type of coordinates
 * data structure (2D/3D)
 * @tparam Path Template parameter, specifies path type, default std::deque of
 * ufo::math::Vector3
 * @param xRand  Coordinate pointer to the random vertex generated by the sample
* process of IB-RRT * algorithm.
 * @param isTreeA Boolean value. Set if the function operates on tree A.
 */
template <typename Map, typename BoundingVolume, typename Coordinates,
          typename Path>
void PIBRRT<Map, BoundingVolume, Coordinates, Path>::getBestParent(
    Coordinates xRand, bool isTreeA) {
  {
    std::lock_guard lck(getBestTreeParentMtx);
    (isTreeA) ? (getBestTreeParentADone = false)
              : (getBestTreeParentBDone = false);
  }
  {
    // Choose best parent
    std::lock_guard lckC((isTreeA) ? listAMtx : listBMtx);
    auto list = (isTreeA) ? listA : listB;
    for (auto &vertex : list) {
      {
        std::unique_lock lckMap(mapMtx);
        mapCV.wait(lckMap, [this] { return mapDone.load(); });
      }
      if (!isInCollision(vertex.second->point, xRand)) {
        (isTreeA) ? (minCostA = vertex.first) : (minCostB = vertex.first);
        (isTreeA) ? (xMinA = vertex.second) : (xMinB = vertex.second);
        break;
      }
    }
  }
  {
    std::lock_guard lck(getBestTreeParentMtx);
    (isTreeA) ? (getBestTreeParentADone = true)
              : (getBestTreeParentBDone = true);
    (isTreeA) ? getBestTreeParentACV.notify_one()
              : getBestTreeParentBCV.notify_one();
  }
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
* @param xRand Random coordinate pointer, coordinate  generated by the sample
function.
* @return Node pointer to the best parent vertex in tree A or B for xRand.
 */
template <class Map, class BoundingVolume, class Coordinates, class Path>
typename PIBRRT<Map, BoundingVolume, Coordinates, Path>::Node *
PIBRRT<Map, BoundingVolume, Coordinates, Path>::getBestTreeParent(
    Coordinates *xRand) {
  insertInA = true;
  minCostA = 0.0;
  minCostB = 0.0;
  Node *xMin = nullptr;
  xMinA = nullptr;
  xMinB = nullptr;

  threadPool->workOn(
      std::bind(&PIBRRT<Map, BoundingVolume, Coordinates, Path>::getBestParent,
                this, *xRand, true));
  threadPool->workOn(
      std::bind(&PIBRRT<Map, BoundingVolume, Coordinates, Path>::getBestParent,
                this, *xRand, false));
  {
    std::unique_lock lck(getBestTreeParentMtx);
    getBestTreeParentACV.wait(lck,
                              [this] { return getBestTreeParentADone.load(); });
    getBestTreeParentBCV.wait(lck,
                              [this] { return getBestTreeParentBDone.load(); });
  }

  if (xMinA != nullptr && xMinB != nullptr) {

    if (minCostA <= minCostB) {
      xMin = xMinA;
    } else if (minCostB < minCostA) {
      xMin = xMinB;
      insertInA = false;
    }
    if (connection)
      connectTrees(xRand);
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
void PIBRRT<Map, BoundingVolume, Coordinates, Path>::rewire(
    std::vector<std::unique_ptr<Node>> *tree) {
  double cost{0.0};
  auto xRand = tree->back().get();
  auto xInit = (*tree)[0].get();
  auto list = (insertInA.load()) ? listA : listB;
  for (auto &vertex : list) {
    cost = norm(xInit->point, xRand->point) +
           norm(xRand->point, vertex.second->point);
    {
      std::unique_lock lckMap(mapMtx);
      mapCV.wait(lckMap, [this] { return mapDone.load(); });
    }
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
void PIBRRT<Map, BoundingVolume, Coordinates, Path>::expandTree(
    Node *xMin, Coordinates *xNew, std::vector<std::unique_ptr<Node>> *tree) {

  std::unique_ptr<Node> newVertex =
      std::make_unique<Node>(*xNew); // expand(xNew,&xMin->point));
  newVertex->parent = xMin;
  newVertex->costToParent = norm(xMin->point, newVertex->point); // cost so far.
  std::lock_guard lck((insertInA.load()) ? treeAMtx : treeBMtx);
  tree->emplace_back(std::move(newVertex));
  // Rewire the tree
  rewire(tree);
}
/**
 * @brief Heuristic function employed by Parallel IB-RRT*, main
responsible for generating a path. This procedure updates the end-to-end
collision-free path between tree A and tree B through a common vertex xRand if
and only if there is collision free path between the best parent vertex on tree
A with xRand and the best parent vertex on tree B with xRand. The path is
updated if the cost of concatenated paths, c(σa|σb), is found to be less than
the cost of the existing end-to-end path c(σf). Connection between the trees is
only successful if the boolean variable connection is true.
 * @tparam Map Template parameter, specifies the Map type.
 * @tparam BoundingVolume Template parameter, specifies the type of bounding
 * volume to check for collisions in a map.
 * @tparam Coordinates Template parameter, specifies the type of coordinates
 * data structure (2D/3D)
 * @tparam Path Template parameter, specifies path type, default std::deque of
 * ufo::math::Vector3
 * @param xRand Random coordinate generated by the sample function, added as a
new Node to either tree A or B.
 */
template <typename Map, typename BoundingVolume, typename Coordinates,
          typename Path>
void PIBRRT<Map, BoundingVolume, Coordinates, Path>::connectTrees(
    Coordinates *xRand) {
  double costA{0.0};
  double costB{0.0};
  double sigmaAB{0.0};
  Path pathA;
  Path pathB;
  {
    std::unique_lock lckMap(mapMtx);
    mapCV.wait(lckMap, [this] { return mapDone.load(); });
  }
  if (!isInCollision(xMinA->point, *xRand) &&
      !isInCollision(xMinB->point, *xRand)) {
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
Path PIBRRT<Map, BoundingVolume, Coordinates, Path>::traceBack(Node *node,
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
 * @brief Implementation of the Parallel IB-RRT* algorithm's main body.
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
Path *PIBRRT<Map, BoundingVolume, Coordinates, Path>::computePath(
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
  while (iter--) { //
    connection = true;
    // Sample
    *xRand = sample();
    // Get the nearby vertices for both trees.
    {
      nearVertices(xRand.get());
      std::unique_lock lck(nearVerticesMtx);
      nearVerticesACV.wait(lck, [this] { return nearVerticesADone.load(); });
      nearVerticesBCV.wait(lck, [this] { return nearVerticesBDone.load(); });
    }
    if (xNearA.empty() || xNearB.empty()) {
      nearestVertex(xRand.get());
      {
        std::unique_lock lck(nearestVertexMtx);
        nearestVertexACV.wait(lck,
                              [this] { return nearestVertexADone.load(); });
        nearestVertexBCV.wait(lck,
                              [this] { return nearestVertexBDone.load(); });
      }
      connection = false;
    }
    // Sort group of near vertices.
    getSortedList(xRand.get());
    {
      std::unique_lock lck(getSortedListMtx);
      getSortedListACV.wait(lck, [this] { return getSortedListADone.load(); });
      getSortedListBCV.wait(lck, [this] { return getSortedListBDone.load(); });
    }
    xMin = getBestTreeParent(xRand.get());

    if (xMin != nullptr)
      expandTree(xMin, xRand.get(), (insertInA.load()) ? treeAPtr : treeBPtr);
  }

  return (path.empty()) ? nullptr : &path;
}

} // namespace globalPlanner::RRT

#endif // PARALLEL_IB_RRT_STAR_PLANNER_H