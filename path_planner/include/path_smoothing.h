
#ifndef PATH_SMOOTHING_H
#define PATH_SMOOTHING_H
// Catmull-Rom splines for path smoothing
namespace globalPlanner {
template <typename Point, typename Path> class CatmullRomSpline {
public:
  CatmullRomSpline();
  CatmullRomSpline(double alpha_, double tension_);
  ~CatmullRomSpline() = default;
  Path interpolate(double step, Path *control);

private:
  double tension{0.0};
  double alpha{0.0};
  double t01{0.0};
  double t12{0.0};
  double t23{0.0};
  Point a;
  Point b;
  Point c;
  Point d;
  Point m1;
  Point m2;
  std::vector<double> time;
  Path interpolatedPath;
  double epsilon{0.001}; // To avoid dividing by zero
};
// Constructors
template <typename Point, typename Path>
CatmullRomSpline<Point, Path>::CatmullRomSpline() : tension(0.0), alpha(0.5) {}
template <typename Point, typename Path>
CatmullRomSpline<Point, Path>::CatmullRomSpline(double alpha_, double tension_)
    : alpha(alpha_), tension(tension_) {}
template <typename Point, typename Path>

Path CatmullRomSpline<Point, Path>::interpolate(double step, Path *control) {
  try {
    if (step == 0)
      throw std::runtime_error(" Step size cannot be 0!");
    if (control->size() < 4)
      throw std::runtime_error("Not enough control points! #Current points: " +
                               std::to_string(control->size()));
  } catch (const std::exception &e) {
    std::cout << "Cannot interpolate, reason: " << e.what() << std::endl;
  };

  interpolatedPath.clear();
  // Precompute the t for interpolation
  time.clear();
  for (int t = 0; t < step; t++)
    time.emplace_back(((double)t) / step);

  // For  calculating t01,t12,t23.
  auto tnm = [&](const Point &A, const Point &B) {
    auto norm = std::sqrt(((A.x() - B.x()) * (A.x() - B.x())) +
                          ((A.y() - B.y()) * (A.y() - B.y())) +
                          ((A.z() - B.z()) * (A.z() - B.z())));
    (norm == 0) ? (norm = epsilon) : 0;
    return std::pow(norm, alpha);
  };
  auto tan1 = [&](const Point &p0, const Point &p1, const Point &p2) {
    return ((p2 - p1) + (((p1 - p0) / t01 - (p2 - p0) / (t01 + t12)) * t12)) *
           (1.0 - tension);
  };
  auto tan2 = [&](const Point &p1, const Point &p2, const Point &p3) {
    return (p2 - p1 + ((p3 - p2) / t23 - (p3 - p1) / (t12 + t23)) * t12) *
           (1.0 - tension);
  };
  auto s = [&](const auto &t) { return a * t * t * t + b * t * t + c * t + d; };
  // interpolate for all points.

  for (int i = 0; i < control->size() - 3; i++) {
    t01 = tnm((*control)[i], (*control)[i + 1]);
    t12 = tnm((*control)[i + 1], (*control)[i + 2]);
    t23 = tnm((*control)[i + 2], (*control)[i + 3]);
    m1 = tan1((*control)[i], (*control)[i + 1], (*control)[i + 2]);
    m2 = tan2((*control)[i + 1], (*control)[i + 2], (*control)[i + 3]);
    a = (((*control)[i + 1] - (*control)[i + 2]) * 2.0) + m1 + m2;
    b = ((*control)[i + 1] - (*control)[i + 2]) * (-3.0) - m1 - m1 - m2;
    c = m1;
    d = (*control)[i + 1];
    for (const auto &t : time)
      interpolatedPath.emplace_back(s(t));
  }

  return interpolatedPath;
}
// Methods
} // namespace globalPlanner

#endif