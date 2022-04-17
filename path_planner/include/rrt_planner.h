//
// Created by alsarmi on 04/04/22.
//

#ifndef PATH_PLANNER_RRT_PLANNER_H
#define PATH_PLANNER_RRT_PLANNER_H

#include <exception>
#include <limits>
#include <memory>
#include <random>
#include <ufo/map/occupancy_map_color.h>
#include <vector>
namespace planner::RRT {
template <class Map, class BoundingVolume, class Coordinates, class Path>
class RRT {
public:
  // Constructors
  RRT(Map *map_, double radius_);

  RRT(Map *map_, Coordinates const &mapMinBoundary_,
      Coordinates const &mapMaxBoundary_, double step, double epsilon_,
      double threshold_, double searchRadius_, double radius_);

  RRT(Map *map_, Coordinates const &mapMinBoundary_,
      Coordinates const &mapMaxBoundary_, double step, double epsilon_,
      double threshold_, double searchRadius_, long timeout_, double radius_);

  // Copy constructor and Copy assignment are not allowed
  RRT(RRT const &src) = delete;
  RRT &operator=(RRT const &src) = delete;
  // Move assignment and move constructor
  RRT(RRT &&src) noexcept;
  RRT &operator=(RRT &&src) noexcept;
  // Destructor
  ~RRT();
  // Public member functions
  void setMaxIterations(int iterations_) { timeout = iterations_; }
  void setMapBoundaries(Coordinates const &mapMinBoundary_,
                        Coordinates const &mapMaxBoundary_) {
    mapMinBoundary = mapMinBoundary_;
    mapMaxBoundary = mapMaxBoundary_;
  }
  Path *computePath(Coordinates const &start_, Coordinates const &goal_);

private:
  // Private structure for the tree
  struct Node {
    explicit Node(Coordinates point_) : point(point_){};
    explicit Node() = default;
    Coordinates point;
    Node *parent = nullptr;
    Node *child = nullptr;
    double costToParent{0.0};
  };
  // Private members
  std::vector<std::unique_ptr<Node>> tree;
  std::vector<Node *> nearby;
  long timeout; // 30 seconds default
  Map *map;
  Path path;
  double epsilon;      // 0.1 mts default
  double threshold;    // 0.5 default
  double searchRadius; // 0.3 mts default
  double deltaStep;    // in meters
  double radius;       // in meters
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
  friend void swap(RRT &src, RRT &dst) {
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
  // Core component of RRT algorithm
  void expandTree(Node *qNear, Coordinates qNew);
  // RRT* components
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
} // namespace planner::RRT
#endif // PATH_PLANNER_RRT_PLANNER_H