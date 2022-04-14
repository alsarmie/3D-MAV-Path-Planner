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
using BoundingVolume = ufo::geometry::BoundingVar;
using Depth = ufo::map::DepthType;
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
  Path *computePath(Coordinates start_, Coordinates goal_);

private:
  // Private structure for the tree
  struct Node {
    explicit Node(Coordinates _point) : point(_point){};
    explicit Node() = default;
    Coordinates point;
    Node *parent = nullptr;
    Node *child = nullptr;
  };
  // Define map limits if they were not provided
  void getMapLimits();
  // Core component of RRT algorithm
  void expandTree(Node *qNear, Coordinates qNew);
  // 3D collision checking
  bool isInCollision(Coordinates const &center); // To check if point is in
                                                 // occupied or unknown space.
  bool isInCollision(Coordinates const &goal_,
                     Coordinates const &position_); // To check if path from A
                                                    // to B is collision free.
  void getClosestVoxel(Coordinates const &src, Point &dst);
  // Random point generation with validation check( is within free space).
  Coordinates generateRandomPoint();
  Node *nearestVertex(Node *qrand);
  Coordinates steer(Coordinates const &nearestVeretx,
                    Coordinates const &randomVertex);
  void traceBack();
  // Private members
  std::vector<std::unique_ptr<Node>> tree;
  int iterations;
  Map &map;
  Path path;
  double epsilon{0.10}; // 10 cm //TODO: make this an argument of the
                        // constructor
  double deltaStep;     // in meters
  double radius;        // in meters
  Coordinates mapLBoundary;
  Coordinates mapUBoundary;

  // Random number generator:
  std::random_device randomDevice;
  std::mt19937 randomEngine;
  std::uniform_real_distribution<> xrand;
  std::uniform_real_distribution<> yrand;
  std::uniform_real_distribution<> zrand;
};
// Forward declarations of template specialization.
template <>
void RRT<ColorMap, Point, NavPath>::getClosestVoxel(const Point &src,
                                                    Point &dst);
template <> void RRT<ColorMap, Point, NavPath>::getMapLimits(); //
template <>
bool RRT<ColorMap, Point, NavPath>::isInCollision(Point const &center);
template <>
bool RRT<ColorMap, Point, NavPath>::isInCollision(Point const &goal_,
                                                  Point const &position_);
template <>
NavPath *RRT<ColorMap, Point, NavPath>::computePath(Point start_, Point goal_);
template <> Point RRT<ColorMap, Point, NavPath>::generateRandomPoint();
template <>
RRT<ColorMap, Point, NavPath>::Node *
RRT<ColorMap, Point, NavPath>::nearestVertex(Node *qrand);
template <>
Point RRT<ColorMap, Point, NavPath>::steer(Point const &nearestVertex,
                                           Point const &randomVertex);
template <>
void RRT<ColorMap, Point, NavPath>::expandTree(Node *qNear, Point qNew);
template <> void RRT<ColorMap, Point, NavPath>::traceBack();
#endif // PATH_PLANNER_RRT_PLANNER_H