// Chapter 11: Embodied Footprints, IEEE TITS 25(2), 2046-2060, 2024.
// DOI 10.1109/TITS.2023.3316175. Project LICENSE applies to this
// implementation. C++17 + CasADi/IPOPT. No MATLAB, AMPL or Python dependency.
// The search library is protected. Cases contain only boundaries and obstacles.
#include <algorithm>
#include <array>
#include <casadi/casadi.hpp>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
extern "C" int plan_rigid_path(const double *, const double *, const double *,
                               const double *, int, int, double *, int, char *,
                               int);
namespace ef {
using Point = std::array<double, 2>;
using Polygon = std::array<Point, 4>;
using Pose = std::array<double, 3>;
using State = std::array<double, 8>; // x,y,theta,v,a,phi,omega,dt
constexpr double pi = 3.14159265358979323846, Lw = 2.8, Lf = 3.76, Lr = .929,
                 width = 1.942, vmax = 5, amax = .75, phimax = .7, wmax = .5,
                 lambda = .9, max_dt = .25;
struct Case {
  std::array<double, 6> boundary;
  std::vector<Polygon> obstacles;
};
Case load_case(const std::filesystem::path &file) {
  std::ifstream f(file);
  if (!f)
    throw std::runtime_error("Cannot read scene: " + file.string());
  Case c;
  for (auto &v : c.boundary)
    f >> v;
  int count = 0;
  f >> count;
  if (count < 1 || count > 10000)
    throw std::runtime_error("Invalid obstacle count.");
  c.obstacles.resize(count);
  for (auto &p : c.obstacles)
    for (auto &q : p)
      f >> q[0] >> q[1];
  if (!f)
    throw std::runtime_error("Truncated scene file.");
  return c;
}
std::vector<Pose> search(const Case &c) {
  double bounds[] = {0, 30, 0, 30}, vehicle[] = {Lw, Lf, Lr, width, phimax};
  std::vector<double> obstacles;
  for (auto &p : c.obstacles)
    for (auto &q : p) {
      obstacles.push_back(q[0]);
      obstacles.push_back(q[1]);
    }
  std::vector<double> buffer(3 * 100000);
  char error[512]{};
  int count = plan_rigid_path(bounds, vehicle, c.boundary.data(),
                              obstacles.data(), int(c.obstacles.size()), 0,
                              buffer.data(), 100000, error, 512);
  if (count < 2)
    throw std::runtime_error(error);
  std::vector<Pose> path;
  for (int i = 0; i < count; ++i) {
    Pose q{buffer[3 * i], buffer[3 * i + 1], buffer[3 * i + 2]};
    if (path.empty() ||
        std::hypot(q[0] - path.back()[0], q[1] - path.back()[1]) > 1e-8)
      path.push_back(q);
  }
  return path;
}
double interpolate(const std::vector<double> &distance,
                   const std::vector<double> &values, double s) {
  auto it = std::upper_bound(distance.begin(), distance.end(), s);
  int hi = std::clamp(int(it - distance.begin()), 1, int(distance.size()) - 1);
  double r = (s - distance[hi - 1]) / (distance[hi] - distance[hi - 1]);
  return values[hi - 1] + r * (values[hi] - values[hi - 1]);
}
std::vector<State> initial_guess(const std::vector<Pose> &path) {
  size_t count = path.size();
  std::vector<double> distance(count), x(count), y(count), theta(count),
      curvature(count);
  for (size_t i = 0; i < count; ++i) {
    x[i] = path[i][0];
    y[i] = path[i][1];
    theta[i] = path[i][2];
    if (i)
      distance[i] =
          distance[i - 1] + std::hypot(x[i] - x[i - 1], y[i] - y[i - 1]);
  }
  for (size_t i = 0; i < count; ++i) {
    size_t a = i ? i - 1 : i, b = i + 1 < count ? i + 1 : i;
    curvature[i] = (theta[b] - theta[a]) / (distance[b] - distance[a]);
  }
  double length = distance.back(),
         peak = std::min(vmax, std::sqrt(length * amax)), ramp = peak / amax,
         cruise = std::max(0., (length - peak * peak / amax) / peak),
         T = 2 * ramp + cruise;
  int samples = int(std::ceil(T / .001)) + 1;
  std::vector<State> dense(samples);
  std::vector<double> time(samples);
  for (int i = 0; i < samples; ++i) {
    double t = T * i / (samples - 1),
           v = std::min(peak, std::min(amax * t, amax * (T - t)));
    double s = t <= ramp ? .5 * amax * t * t
                         : (t <= ramp + cruise
                                ? .5 * peak * ramp + peak * (t - ramp)
                                : length - .5 * amax * (T - t) * (T - t));
    time[i] = t;
    dense[i] = {interpolate(distance, x, s),
                interpolate(distance, y, s),
                interpolate(distance, theta, s),
                v,
                0,
                std::clamp(std::atan(Lw * interpolate(distance, curvature, s)),
                           -phimax, phimax),
                0,
                0};
  }
  dense.front()[5] = dense.back()[5] = 0;
  std::vector<int> selected{0};
  while (selected.back() < samples - 1) {
    int anchor = selected.back(), candidate = anchor + 1;
    for (; candidate < samples; ++candidate) {
      double dt = time[candidate] - time[anchor], s = dense[anchor][3] * dt,
             k = std::abs(std::tan(dense[anchor][5]) / Lw), a = k * s;
      if (dt > lambda * max_dt || a > lambda * pi / 2 ||
          k * Lf * std::tan(a) > lambda * (1 + width * k / 2) ||
          (1 + width * k / 2) * std::tan(a) > lambda * Lr * k)
        break;
    }
    selected.push_back(
        std::min(samples - 1, std::max(anchor + 1, candidate - 1)));
  }
  std::vector<State> guess;
  for (int id : selected)
    guess.push_back(dense[id]);
  for (size_t i = 0; i + 1 < guess.size(); ++i) {
    double dt = time[selected[i + 1]] - time[selected[i]];
    guess[i][7] = dt;
    guess[i][4] = std::clamp((guess[i + 1][3] - guess[i][3]) / dt, -amax, amax);
    guess[i][6] = std::clamp((guess[i + 1][5] - guess[i][5]) / dt, -wmax, wmax);
  }
  guess.front()[4] = guess.front()[6] = 0;
  return guess;
}
std::vector<State> optimize(const Case &scene, const std::vector<State> &guess,
                            const std::string &linear_solver) {
  using casadi::Dict;
  using casadi::DM;
  using casadi::DMDict;
  using casadi::SX;
  using casadi::SXDict;
  int n = int(guess.size());
  const double inf = std::numeric_limits<double>::infinity();
  SX z = SX::sym("z", 22 * n), cost = 0;
  std::vector<SX> constraints;
  std::vector<double> lower(22 * n, -inf), upper(22 * n, inf), initial(22 * n),
      lbg, ubg;
  auto at = [&](int i, int j) -> SX { return z(j * n + i); };
  auto bound = [&](int i, int j, double l, double u) {
    lower[j * n + i] = l;
    upper[j * n + i] = u;
  };
  auto add = [&](SX expression, double l = 0, double u = 0) {
    constraints.push_back(expression);
    lbg.push_back(l);
    ubg.push_back(u);
  };
  using SPoly = std::array<std::array<SX, 2>, 4>;
  auto triangle_sum = [](const SPoly &p, std::array<SX, 2> q) {
    SX a = 0;
    for (int j = 0; j < 4; ++j) {
      auto b = p[j], c = p[(j + 1) % 4];
      a += .5 *
           fabs((b[0] - q[0]) * (c[1] - q[1]) - (b[1] - q[1]) * (c[0] - q[0]));
    }
    return a;
  };
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < 8; ++j)
      initial[j * n + i] = guess[i][j];
    bound(i, 3, 0, vmax);
    bound(i, 4, -amax, amax);
    bound(i, 5, -phimax, phimax);
    bound(i, 6, -wmax, wmax);
    bound(i, 7, 0, max_dt);
    if (i == n - 1) {
      bound(i, 7, 0, 0);
      for (int j = 8; j < 22; ++j)
        bound(i, j, 0, 0);
      continue;
    }
    SX dt = at(i, 7), v = at(i, 3), phi = at(i, 5), theta = at(i, 2),
       k = at(i, 8), s = at(i, 9), angle = fabs(k) * s;
    add(k - tan(phi) / Lw);
    add(s - v * dt);
    initial[8 * n + i] = std::tan(guess[i][5]) / Lw;
    initial[9 * n + i] = guess[i][3] * guess[i][7];
    cost += dt * dt;
    add(at(i + 1, 0) - at(i, 0) - s * cos(theta));
    add(at(i + 1, 1) - at(i, 1) - s * sin(theta));
    add(at(i + 1, 2) - theta - s * k);
    add(at(i + 1, 3) - v - dt * at(i, 4));
    add(at(i + 1, 5) - phi - dt * at(i, 6));
    add(angle, -inf, pi / 2);
    add(fabs(k) * Lf * tan(angle) - 1 - width * fabs(k) / 2, -inf, 0);
    add((1 + width * fabs(k) / 2) * tan(angle) - Lr * fabs(k), -inf, 0);
    SX front = at(i, 10), left = at(i, 11), right = at(i, 13);
    add(front - s - width * fabs(k) * s / 2);
    add(left - fmax(-Lr * k * s, (Lf + s / 2) * k * s));
    add(at(i, 12));
    add(right - fmax(Lr * k * s, -(Lf + s / 2) * k * s));
    double initial_s = initial[9 * n + i], initial_k = initial[8 * n + i],
           turn = initial_s * initial_k;
    initial[10 * n + i] = initial_s + width * std::abs(turn) / 2;
    initial[11 * n + i] = std::max(-Lr * turn, (Lf + initial_s / 2) * turn);
    initial[13 * n + i] = std::max(Lr * turn, -(Lf + initial_s / 2) * turn);
    std::array<SX, 4> dx{Lf + front, Lf + front, SX(-Lr), SX(-Lr)},
        dy{width / 2 + left, -width / 2 - right, -width / 2 - right,
           width / 2 + left};
    SPoly box;
    for (int j = 0; j < 4; ++j) {
      box[j] = {at(i, 14 + 2 * j), at(i, 15 + 2 * j)};
      add(box[j][0] - at(i, 0) - dx[j] * cos(theta) + dy[j] * sin(theta));
      add(box[j][1] - at(i, 1) - dx[j] * sin(theta) - dy[j] * cos(theta));
      double initial_dx = j < 2 ? Lf + initial[10 * n + i] : -Lr,
             initial_dy = (j == 0 || j == 3) ? width / 2 + initial[11 * n + i]
                                             : -width / 2 - initial[13 * n + i];
      initial[(14 + 2 * j) * n + i] = guess[i][0] +
                                      initial_dx * std::cos(guess[i][2]) -
                                      initial_dy * std::sin(guess[i][2]);
      initial[(15 + 2 * j) * n + i] = guess[i][1] +
                                      initial_dx * std::sin(guess[i][2]) +
                                      initial_dy * std::cos(guess[i][2]);
    }
    if (i == 0)
      continue;
    for (const auto &obstacle : scene.obstacles) {
      SPoly polygon;
      double twice = 0;
      for (int j = 0; j < 4; ++j) {
        polygon[j] = {SX(obstacle[j][0]), SX(obstacle[j][1])};
        twice += obstacle[j][0] * obstacle[(j + 1) % 4][1] -
                 obstacle[j][1] * obstacle[(j + 1) % 4][0];
      }
      double area = .5 * std::abs(twice);
      for (int j = 0; j < 4; ++j) {
        add(triangle_sum(polygon, box[j]), area + std::min(.02, .02 * area),
            inf);
        add(triangle_sum(box, polygon[j]) -
                (Lf + Lr + front) * (width + left + right),
            .1, inf);
      }
    }
  }
  for (int endpoint : {0, n - 1}) {
    bool last = endpoint > 0;
    bound(endpoint, 0, scene.boundary[last ? 3 : 0],
          scene.boundary[last ? 3 : 0]);
    bound(endpoint, 1, scene.boundary[last ? 4 : 1],
          scene.boundary[last ? 4 : 1]);
    double angle = last ? guess.back()[2] : scene.boundary[2];
    bound(endpoint, 2, angle, angle);
    for (int j : {3, 4, 5, 6})
      bound(endpoint, j, 0, 0);
  }
  Dict options{{"print_time", false},
               {"ipopt.print_level", 0},
               {"ipopt.tol", 1e-3},
               {"ipopt.constr_viol_tol", 1e-7},
               {"ipopt.max_iter", 3000},
               {"ipopt.max_cpu_time", 60.},
               {"ipopt.mu_strategy", "adaptive"},
               {"ipopt.linear_solver", linear_solver}};
  if (const char *hsl = std::getenv("HSL_LIBRARY"))
    options["ipopt.hsllib"] = std::string(hsl);
  auto solver = casadi::nlpsol(
      "embodied_footprint", "ipopt",
      SXDict{{"x", z}, {"f", cost}, {"g", SX::vertcat(constraints)}}, options);
  auto result = solver(DMDict{{"x0", DM(initial)},
                              {"lbx", DM(lower)},
                              {"ubx", DM(upper)},
                              {"lbg", DM(lbg)},
                              {"ubg", DM(ubg)}});
  if (!static_cast<bool>(solver.stats().at("success")))
    throw std::runtime_error("IPOPT: " +
                             solver.stats().at("return_status").to_string());
  auto values = result.at("x").get_elements(),
       g = result.at("g").get_elements();
  double violation = 0;
  for (size_t i = 0; i < g.size(); ++i)
    violation = std::max({violation, lbg[i] - g[i], g[i] - ubg[i]});
  for (size_t i = 0; i < values.size(); ++i)
    violation =
        std::max({violation, lower[i] - values[i], values[i] - upper[i]});
  if (violation > 1e-5)
    throw std::runtime_error(
        "Final constraints exceed verification tolerance.");
  std::vector<State> solution(n);
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < 8; ++j)
      solution[i][j] = values[j * n + i];
  std::cout << "constraint_residual=" << violation << '\n';
  return solution;
}
bool overlap(const Polygon &a, const Polygon &b) {
  for (const auto *polygon : {&a, &b})
    for (int e = 0; e < 4; ++e) {
      Point axis{(*polygon)[e][1] - (*polygon)[(e + 1) % 4][1],
                 (*polygon)[(e + 1) % 4][0] - (*polygon)[e][0]};
      double length = std::hypot(axis[0], axis[1]);
      if (length < 1e-12)
        continue;
      axis[0] /= length;
      axis[1] /= length;
      double amin = 1e100, amax = -1e100, bmin = 1e100, bmax = -1e100;
      for (int j = 0; j < 4; ++j) {
        double p = a[j][0] * axis[0] + a[j][1] * axis[1],
               q = b[j][0] * axis[0] + b[j][1] * axis[1];
        amin = std::min(amin, p);
        amax = std::max(amax, p);
        bmin = std::min(bmin, q);
        bmax = std::max(bmax, q);
      }
      if (amax <= bmin + 1e-7 || bmax <= amin + 1e-7)
        return false;
    }
  return true;
}
void validate(const Case &scene, const std::vector<State> &solution) {
  double violation = 0;
  for (size_t i = 0; i < solution.size(); ++i) {
    const auto &q = solution[i];
    for (double value : q)
      if (!std::isfinite(value))
        throw std::runtime_error("Nonfinite state.");
    double k = std::tan(q[5]) / Lw, s = q[3] * q[7], a = std::abs(k) * s;
    double front = s + width * a / 2,
           left = std::max(-Lr * k * s, (Lf + s / 2) * k * s),
           right = std::max(Lr * k * s, -(Lf + s / 2) * k * s);
    violation = std::max(
        {violation, a - pi / 2,
         std::abs(k) * Lf * std::tan(a) - 1 - width * std::abs(k) / 2,
         (1 + width * std::abs(k) / 2) * std::tan(a) - Lr * std::abs(k)});
    Polygon box;
    double dx[] = {Lf + front, Lf + front, -Lr, -Lr},
           dy[] = {width / 2 + left, -width / 2 - right, -width / 2 - right,
                   width / 2 + left};
    for (int j = 0; j < 4; ++j)
      box[j] = {q[0] + dx[j] * cos(q[2]) - dy[j] * sin(q[2]),
                q[1] + dx[j] * sin(q[2]) + dy[j] * cos(q[2])};
    for (const auto &obstacle : scene.obstacles)
      if (overlap(box, obstacle))
        throw std::runtime_error("Embodied footprint intersects an obstacle.");
    if (i + 1 < solution.size()) {
      const auto &next = solution[i + 1];
      violation = std::max({violation, std::abs(next[0] - q[0] - s * cos(q[2])),
                            std::abs(next[1] - q[1] - s * sin(q[2])),
                            std::abs(next[2] - q[2] - s * k),
                            std::abs(next[3] - q[3] - q[4] * q[7]),
                            std::abs(next[5] - q[5] - q[6] * q[7])});
    }
    for (int sample = 0; sample <= 20; ++sample) {
      double theta = k * s * sample / 20, lx, ly;
      if (std::abs(k) < 1e-9) {
        lx = s * sample / 20;
        ly = 0;
      } else {
        lx = sin(theta) / k;
        ly = (1 - cos(theta)) / k;
      }
      double ux[] = {Lf, Lf, -Lr, -Lr},
             uy[] = {width / 2, -width / 2, -width / 2, width / 2};
      for (int j = 0; j < 4; ++j) {
        double x = lx + ux[j] * cos(theta) - uy[j] * sin(theta),
               y = ly + ux[j] * sin(theta) + uy[j] * cos(theta);
        violation = std::max({violation, x - Lf - front, -Lr - x,
                              y - width / 2 - left, -width / 2 - right - y});
      }
    }
  }
  if (violation > 1e-5)
    throw std::runtime_error(
        "Independent dynamics/coverage verification failed.");
  std::cout << "independent_validation=passed residual=" << violation << '\n';
}
void save(const std::vector<State> &solution,
          const std::filesystem::path &directory) {
  std::filesystem::create_directories(directory);
  std::ofstream f(directory / "trajectory.csv");
  f << std::setprecision(17) << "time,x,y,theta,v,a,phi,omega,dt\n";
  double t = 0;
  for (const auto &row : solution) {
    f << t;
    for (auto value : row)
      f << ',' << value;
    f << '\n';
    t += row[7];
  }
  if (!f)
    throw std::runtime_error("Cannot write result.");
  std::cout << "nodes=" << solution.size() << " duration=" << t << '\n';
}
} // namespace ef
int main(int argc, char **argv) {
  try {
    int id = 3;
    std::string data = EF_DATA_DIR, output = "results", solver = "mumps";
    for (int i = 1; i < argc; ++i) {
      std::string key = argv[i];
      if (i + 1 == argc)
        throw std::runtime_error("Missing argument value.");
      std::string value = argv[++i];
      if (key == "--case")
        id = std::stoi(value);
      else if (key == "--data")
        data = value;
      else if (key == "--output")
        output = value;
      else if (key == "--linear-solver")
        solver = value;
      else
        throw std::runtime_error("Unknown option: " + key);
    }
    if (id < 1 || id > 3)
      throw std::runtime_error("case must be 1, 2 or 3.");
    auto start = std::chrono::steady_clock::now();
    auto scene = ef::load_case(std::filesystem::u8path(data) /
                               ("case_" + std::to_string(id) + ".txt"));
    auto path = ef::search(scene);
    auto guess = ef::initial_guess(path);
    std::cout << "search_points=" << path.size()
              << " initial_nodes=" << guess.size() << std::endl;
    auto solution = ef::optimize(scene, guess, solver);
    ef::validate(scene, solution);
    ef::save(solution, std::filesystem::u8path(output));
    std::cout << "planning_seconds="
              << std::chrono::duration<double>(
                     std::chrono::steady_clock::now() - start)
                     .count()
              << '\n';
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
