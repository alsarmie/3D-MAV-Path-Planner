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

using Point = ufo::math::Vector3;
using ColorMap = ufo::map::OccupancyMapColor;
using NavPath = std::vector<Point>;
using Sphere = ufo::geometry::Sphere;

template <class Map, class Coordinates, class Path> class RRT {
public:
  // Constructors
  RRT(Map &map_, Coordinates const &mapMinBoundary,
      Coordinates const &mapMaxBoundary, double step, int iters,
      double radius_);
  RRT(Map &map_, Coordinates const &mapMinBoundary,
      Coordinates const &mapMaxBoundary, double step, double radius_);
  RRT(Map &map_, double radius_);
  // Destructor
  ~RRT();
  // Public member functions
  void setMaxIterations(int iterations_) { iterations = iterations_; }
  void setMapBoundaries(Coordinates const &mapMinBoundary,
                        Coordinates const &mapMaxBoundary) {
    mapLBoundary = mapMinBoundary;
    mapUBoundary = mapMaxBoundary;
  }
  Path *computePath(Coordinates const &start_, Coordinates const &goal_);

private:
  // Private structure for the tree
  struct Node {
    explicit Node(Coordinates _point) : point(_point){};
    explicit Node() = default;
    Coordinates point;
    Node *parent = nullptr;
    Node *child = nullptr;
    double costToParent{0.0};
  };
  // Don't allow the object to be copied or copy assigned.

  // Define map limits if they were not provided
  void getMapLimits();
  // Core component of RRT algorithm
  void expandTree(Node *qNear, Coordinates qNew);
  void rewire();
  void chooseParent(Node *qNear, Node *newVertex);
  // 3D collision checking
  bool isInCollision(Coordinates const &center); // To check if point is in
                                                 // occupied or unknown space.
  bool isInCollision(Coordinates const &goal_,
                     Coordinates const &position_); // To check if path from A
                                                    // to B is collision free.
  void getClosestVoxel(Coordinates const &src, Coordinates &dst);
  // Random point generation with validation check( is within free space).
  Coordinates generateRandomPoint();
  Node *nearestVertex(Node *qrand);
  Coordinates steer(Coordinates const &nearestVertex,
                    Coordinates const &randomVertex);
  void traceBack();


  // Private members
  std::vector<std::unique_ptr<Node>> tree;
  std::vector<Node *> nearby;
  int iterations;
  Map &map;
  Path path;
  double epsilon{0.01}; // 1 cm //TODO: make this an argument of the
                        // constructor
  double threshold{0.5};
  double searchRadius{0.3};
  double deltaStep; // in meters
  double radius;    // in meters
  Coordinates mapLBoundary;
  Coordinates mapUBoundary;
  // Random number generator:
  std::random_device randomDevice;
  std::mt19937 randomEngine;
  std::uniform_real_distribution<> xrand;
  std::uniform_real_distribution<> yrand;
  std::uniform_real_distribution<> zrand;
  std::uniform_real_distribution<> rnd;

};

#endif // PATH_PLANNER_RRT_PLANNER_H