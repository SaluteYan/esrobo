// Exact URDF chain kinematics and bounded contact solver. No hardware access.
// Python hand_retarget.py remains the reference implementation and state owner.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <stdexcept>
#include <tuple>
#include <vector>
namespace py = pybind11;
using Vec = std::array<double, 3>;
using Mat = std::array<double, 9>;
using Q = std::array<double, 10>;
using J = std::array<Vec, 10>;
using Square = std::array<Q, 10>;
static Vec add(Vec a, Vec b) {
    for (int i = 0; i < 3; i++)
        a[i] += b[i];
    return a;
}
static Vec sub(Vec a, Vec b) {
    for (int i = 0; i < 3; i++)
        a[i] -= b[i];
    return a;
}
static Vec mul(Vec a, double s) {
    for (double &v : a)
        v *= s;
    return a;
}
static double dot(Vec a, Vec b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
static double norm(Vec a) { return std::sqrt(dot(a, a)); }
static Vec cross(Vec a, Vec b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
static Mat eye() { return {1, 0, 0, 0, 1, 0, 0, 0, 1}; }
static Vec mv(const Mat &a, Vec b) {
    Vec c{};
    for (int i = 0; i < 3; i++)
        for (int k = 0; k < 3; k++)
            c[i] += a[3 * i + k] * b[k];
    return c;
}
static Mat mm(const Mat &a, const Mat &b) {
    Mat c{};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 3; k++)
                c[3 * i + j] += a[3 * i + k] * b[3 * k + j];
    return c;
}
static Mat rotation(Vec a, double theta) {
    Mat k = {0, -a[2], a[1], a[2], 0, -a[0], -a[1], a[0], 0};
    Mat k2 = mm(k, k), r = eye();
    double s = std::sin(theta), c = 1 - std::cos(theta);
    for (int i = 0; i < 9; i++)
        r[i] += s * k[i] + c * k2[i];
    return r;
}
static Q linear_solve(Square a, Q b, int n) {
    for (int k = 0; k < n; k++) {
        int p = k;
        for (int i = k + 1; i < n; i++)
            if (std::abs(a[i][k]) > std::abs(a[p][k]))
                p = i;
        if (std::abs(a[p][k]) < 1e-18)
            throw std::runtime_error("singular damped hand system");
        std::swap(a[p], a[k]);
        std::swap(b[p], b[k]);
        for (int i = k + 1; i < n; i++) {
            double f = a[i][k] / a[k][k];
            for (int j = k + 1; j < n; j++)
                a[i][j] -= f * a[k][j];
            b[i] -= f * b[k];
        }
    }
    Q x{};
    for (int i = n - 1; i >= 0; i--) {
        double v = b[i];
        for (int j = i + 1; j < n; j++)
            v -= a[i][j] * x[j];
        x[i] = v / a[i][i];
    }
    return x;
}
struct Joint {
    Vec offset, axis;
    Mat rot;
    int source;
    double ratio, bias;
};
struct Chain {
    std::array<Vec, 4> p{};
    std::array<J, 4> jac{};
};
struct Deadline : std::runtime_error {
    using std::runtime_error::runtime_error;
};
using Clock = std::chrono::steady_clock;
struct Key {
    bool outside = false;
    double violation = 0, pose = 0, regularization = 0;
    bool operator<(const Key &b) const {
        return std::tie(outside, violation, pose, regularization) <
               std::tie(b.outside, b.violation, b.pose, b.regularization);
    }
};
struct Constraint {
    Q normal{};
    double margin = 0;
};
struct Evaluation {
    std::array<double, 64> residual{};
    std::array<Q, 64> jac{};
    int rows = 0, restore_begin = 0, restore_end = 0;
    Key key;
    Chain thumb;
    std::array<Chain, 4> fingers;
    std::array<Vec, 3> directions;
    std::vector<Constraint> constraints;
};
struct Policy {
    double flex, side, tolerance, contact_weight, initial_weight, opening_weight, temporal, finger,
        huber, plane, damping;
    int updates;
};
static std::vector<double> doubles(py::handle obj, size_t size) {
    auto a = py::array_t<double, py::array::c_style | py::array::forcecast>::ensure(obj);
    if (!a || size_t(a.size()) != size)
        throw py::value_error("invalid native hand array shape");
    std::vector<double> v(a.data(), a.data() + size);
    for (double x : v)
        if (!std::isfinite(x))
            throw py::value_error("nonfinite native hand input");
    return v;
}
static Q as_q(py::handle obj) {
    auto v = doubles(obj, 10);
    Q q{};
    std::copy(v.begin(), v.end(), q.begin());
    return q;
}
static std::array<Vec, 3> directions(const Chain &c, std::array<J, 3> *jac = nullptr) {
    std::array<Vec, 3> d{};
    for (int b = 0; b < 3; b++) {
        Vec v = sub(c.p[b + 1], c.p[b]);
        double l = norm(v);
        if (l < 1e-8)
            throw std::runtime_error("degenerate model bone");
        d[b] = mul(v, 1 / l);
        if (jac)
            for (int k = 0; k < 10; k++) {
                Vec dv = sub(c.jac[b + 1][k], c.jac[b][k]);
                (*jac)[b][k] = mul(sub(dv, mul(d[b], dot(d[b], dv))), 1 / l);
            }
    }
    return d;
}
static Vec plane(const std::array<Vec, 3> &d, const std::array<J, 3> *jd, J *jp) {
    Vec c = cross(d[0], d[2]);
    double eps = std::sin(5 * 3.14159265358979323846 / 180), l = std::sqrt(dot(c, c) + eps * eps);
    if (jd && jp)
        for (int k = 0; k < 10; k++) {
            Vec dc = add(cross((*jd)[0][k], d[2]), cross(d[0], (*jd)[2][k]));
            (*jp)[k] = sub(mul(dc, 1 / l), mul(c, dot(c, dc) / (l * l * l)));
        }
    return mul(c, 1 / l);
}
class Model {
    std::array<Joint, 20> joints;
    std::array<Vec, 5> tips;
    std::array<std::array<int, 4>, 4> finger_ids;
    Q lower, upper, neutral;
    std::array<Vec, 26> rest;
    double width;
    Policy policy;
    std::vector<Vec> seed_angles;
    std::vector<std::array<Vec, 3>> seed_directions;

  public:
    Model(py::dict data, py::dict p) {
        auto offsets = doubles(data["offsets"], 60), axes = doubles(data["axes"], 60),
             rot = doubles(data["rotations"], 180);
        auto source = doubles(data["sources"], 20), ratio = doubles(data["ratios"], 20),
             bias = doubles(data["biases"], 20);
        auto tip = doubles(data["tips"], 15), fids = doubles(data["finger_ids"], 16),
             rp = doubles(data["rest"], 78);
        for (int i = 0; i < 20; i++) {
            for (int k = 0; k < 3; k++) {
                joints[i].offset[k] = offsets[i * 3 + k];
                joints[i].axis[k] = axes[i * 3 + k];
            }
            for (int k = 0; k < 9; k++)
                joints[i].rot[k] = rot[i * 9 + k];
            joints[i].source = int(source[i]);
            joints[i].ratio = ratio[i];
            joints[i].bias = bias[i];
            if (joints[i].source < 0 || joints[i].source >= 10)
                throw py::value_error("invalid joint source");
        }
        for (int f = 0; f < 5; f++)
            for (int k = 0; k < 3; k++)
                tips[f][k] = tip[f * 3 + k];
        for (int f = 0; f < 4; f++)
            for (int j = 0; j < 4; j++) {
                finger_ids[f][j] = int(fids[f * 4 + j]);
                if (finger_ids[f][j] < -1 || finger_ids[f][j] >= 20)
                    throw py::value_error("invalid finger chain");
            }
        for (int i = 0; i < 26; i++)
            for (int k = 0; k < 3; k++)
                rest[i][k] = rp[i * 3 + k];
        lower = as_q(data["lower"]);
        upper = as_q(data["upper"]);
        neutral = as_q(data["neutral"]);
        width = py::cast<double>(data["width"]);
        if (!(width > 0))
            throw py::value_error("invalid palm width");
        auto sa =
            py::array_t<double, py::array::c_style | py::array::forcecast>::ensure(data["seeds"]);
        if (!sa || sa.size() % 3)
            throw py::value_error("invalid seed table");
        for (int i = 0; i < sa.size() / 3; i++) {
            Vec v{};
            Q q{};
            for (int k = 0; k < 3; k++)
                v[k] = q[k] = sa.data()[3 * i + k];
            seed_angles.push_back(v);
            seed_directions.push_back(directions(thumb(q)));
        }
        policy = {py::cast<double>(p["flex_correction_deg"]) * 3.14159265358979323846 / 180,
                  py::cast<double>(p["side_correction_deg"]) * 3.14159265358979323846 / 180,
                  py::cast<double>(p["contact_tolerance_m"]),
                  py::cast<double>(p["contact_weight"]),
                  py::cast<double>(p["initial_contact_weight"]),
                  py::cast<double>(p["opening_vector_weight"]),
                  py::cast<double>(p["temporal_weight"]),
                  py::cast<double>(p["finger_regularization"]),
                  py::cast<double>(p["huber_delta"]),
                  py::cast<double>(p["plane_weight"]),
                  py::cast<double>(p["damping"]),
                  py::cast<int>(p["max_updates"])};
    }
    Chain chain(const Q &q, const std::vector<int> &ids, int finger) const {
        std::array<Vec, 5> origins{}, axes{};
        Mat r = eye();
        Vec pos{};
        int count = int(ids.size());
        for (int j = 0; j < count; j++) {
            int i = ids[j];
            if (i < 0) {
                origins[j] = pos;
                continue;
            }
            const Joint &g = joints[i];
            pos = add(pos, mv(r, g.offset));
            origins[j] = pos;
            Mat base = mm(r, g.rot);
            axes[j] = mv(base, g.axis);
            r = mm(base, rotation(g.axis, q[g.source] * g.ratio + g.bias));
        }
        Chain c;
        int first = finger == 0 ? 2 : 1;
        for (int p = 0; p < 3; p++)
            c.p[p] = origins[first + p];
        c.p[3] = add(pos, mv(r, tips[finger]));
        for (int p = 0; p < 4; p++) {
            int stop = p == 3 ? count : first + p;
            for (int j = 0; j < stop; j++) {
                int i = ids[j];
                if (i < 0)
                    continue;
                const Joint &g = joints[i];
                c.jac[p][g.source] =
                    add(c.jac[p][g.source], mul(cross(axes[j], sub(c.p[p], origins[j])), g.ratio));
            }
        }
        return c;
    }
    Chain thumb(const Q &q) const { return chain(q, {0, 1, 2, 3, 4}, 0); }
    Chain finger(const Q &q, int f) const {
        return chain(q, std::vector<int>(finger_ids[f].begin(), finger_ids[f].end()), f + 1);
    }
    std::array<Vec, 26> assemble(const Chain &t, const std::array<Chain, 4> &fs) const {
        auto pts = rest;
        for (int p = 0; p < 4; p++)
            pts[2 + p] = t.p[p];
        for (int f = 0; f < 4; f++)
            for (int p = 0; p < 4; p++)
                pts[7 + 5 * f + p] = fs[f].p[p];
        pts[0] = {};
        for (int f = 0; f < 4; f++)
            pts[0] = add(pts[0], mul(fs[f].p[0], .25));
        return pts;
    }
    py::tuple landmarks(py::handle angles) const {
        Q q = as_q(angles);
        Chain t = thumb(q);
        std::array<Chain, 4> fs;
        for (int f = 0; f < 4; f++)
            fs[f] = finger(q, f);
        auto pts = assemble(t, fs);
        py::array_t<double> out({26, 3}), jac({26, 3, 10});
        auto a = out.mutable_unchecked<2>();
        auto b = jac.mutable_unchecked<3>();
        for (int p = 0; p < 26; p++)
            for (int c = 0; c < 3; c++) {
                a(p, c) = pts[p][c];
                for (int k = 0; k < 10; k++)
                    b(p, c, k) = 0;
            }
        for (int p = 0; p < 4; p++)
            for (int c = 0; c < 3; c++)
                for (int k = 0; k < 10; k++)
                    b(p + 2, c, k) = t.jac[p][k][c];
        for (int f = 0; f < 4; f++)
            for (int p = 0; p < 4; p++)
                for (int c = 0; c < 3; c++)
                    for (int k = 0; k < 10; k++) {
                        b(7 + 5 * f + p, c, k) = fs[f].jac[p][k][c];
                        if (p == 0)
                            b(0, c, k) += fs[f].jac[p][k][c] * .25;
                    }
        return py::make_tuple(out, jac);
    }
    py::dict solve(py::handle bq, py::handle pq, py::handle dd, py::handle pp, py::handle cc,
                   py::handle blend, py::handle gg, py::handle ww, double seconds) const {
        Q base = as_q(bq), previous = as_q(pq);
        auto dv = doubles(dd, 9), pv = doubles(pp, 12), cv = doubles(cc, 4), pr = doubles(blend, 4),
             gates = doubles(gg, 4), weights = doubles(ww, 3);
        std::array<Vec, 3> desired{};
        std::array<Vec, 4> pair{};
        std::array<bool, 4> contact{};
        std::vector<int> active, indices = {0, 1, 2};
        std::array<int, 4> flex = {4, 5, 7, 9}, side = {3, -1, 6, 8};
        for (int k = 0; k < 3; k++)
            for (int c = 0; c < 3; c++)
                desired[k][c] = dv[3 * k + c];
        for (int f = 0; f < 4; f++) {
            contact[f] = cv[f] != 0;
            for (int c = 0; c < 3; c++)
                pair[f][c] = pv[3 * f + c];
            if (contact[f] || pr[f] > 0) {
                active.push_back(f);
                if (side[f] >= 0)
                    indices.push_back(side[f]);
                indices.push_back(flex[f]);
            }
        }
        int n = int(indices.size());
        Q lo{}, hi{}, prior{}, reg{};
        for (int i = 0; i < n; i++) {
            int k = indices[i];
            lo[i] = lower[k];
            hi[i] = upper[k];
            prior[i] = previous[k];
            reg[i] = policy.temporal;
        }
        for (int f : active)
            for (int i = 3; i < n; i++) {
                int k = indices[i];
                if (k != side[f] && k != flex[f])
                    continue;
                double radius =
                    (k == flex[f] ? policy.flex : policy.side) * (contact[f] ? 1. : pr[f]);
                lo[i] = std::max(lo[i], base[k] - radius);
                hi[i] = std::min(hi[i], base[k] + radius);
                prior[i] = base[k];
                reg[i] = policy.finger;
            }
        Vec target_plane = plane(desired, nullptr, nullptr);
        double plane_scale = std::sqrt(policy.plane) * norm(target_plane);
        std::array<Chain, 4> fixed;
        for (int f = 0; f < 4; f++)
            fixed[f] = finger(base, f);
        auto deadline = Clock::now() + std::chrono::duration<double>(seconds);
        auto check = [&]() {
            if (Clock::now() >= deadline)
                throw Deadline("left hand solver exceeded 20 ms");
        };
        int evaluations = 0;
        auto evaluate = [&](const Q &x) {
            check();
            evaluations++;
            Evaluation e;
            e.fingers = fixed;
            Q full = base;
            for (int i = 0; i < n; i++)
                full[indices[i]] = x[i];
            e.thumb = thumb(full);
            for (int f : active)
                e.fingers[f] = finger(full, f);
            std::array<J, 3> jd{};
            e.directions = directions(e.thumb, &jd);
            J jp{};
            Vec pl = plane(e.directions, &jd, &jp);
            auto row = [&](double value, const Q &j) {
                e.residual[e.rows] = value;
                e.jac[e.rows] = j;
                e.rows++;
            };
            auto posture = [&](double value, const Q &j) {
                double a = std::abs(value),
                       robust = std::sqrt(std::min(1., policy.huber / std::max(a, 1e-12)));
                Q rj = j;
                for (int i = 0; i < n; i++)
                    rj[i] *= robust;
                row(value * robust, rj);
                e.key.pose +=
                    (a <= policy.huber ? value * value
                                       : 2 * policy.huber * a - policy.huber * policy.huber);
            };
            for (int b = 0; b < 3; b++)
                for (int c = 0; c < 3; c++) {
                    double w = std::sqrt(weights[b] / 3);
                    Q j{};
                    for (int i = 0; i < n; i++)
                        j[i] = jd[b][indices[i]][c] * w;
                    posture((e.directions[b][c] - desired[b][c]) * w, j);
                }
            for (int c = 0; c < 3; c++) {
                Q j{};
                for (int i = 0; i < n; i++)
                    j[i] = jp[indices[i]][c] * plane_scale;
                posture((pl[c] - target_plane[c]) * plane_scale, j);
            }
            for (int i = 0; i < n; i++) {
                Q j{};
                j[i] = std::sqrt(reg[i]);
                row((x[i] - prior[i]) * j[i], j);
                e.key.regularization += reg[i] * (x[i] - prior[i]) * (x[i] - prior[i]);
            }
            std::array<Vec, 4> delta{};
            std::array<J, 4> pair_jac{};
            for (int f = 0; f < 4; f++) {
                delta[f] = sub(sub(e.fingers[f].p[3], e.thumb.p[3]), pair[f]);
                for (int i = 0; i < n; i++)
                    pair_jac[f][i] =
                        sub(e.fingers[f].jac[3][indices[i]], e.thumb.jac[3][indices[i]]);
            }
            e.restore_begin = e.rows;
            for (int f : active) {
                if (!contact[f])
                    continue;
                double length = norm(delta[f]);
                J dj = pair_jac[f];
                if (length > 1e-8 && length <= policy.tolerance) {
                    Constraint c;
                    for (int i = 0; i < n; i++)
                        c.normal[i] = dot(delta[f], dj[i]) / length;
                    c.margin = std::max(0., policy.tolerance - length - 1e-5);
                    e.constraints.push_back(c);
                }
                e.key.violation =
                    std::max(e.key.violation, std::max(length - policy.tolerance + 2e-6, 0.));
                Vec v{};
                J jv = dj;
                double tol = policy.tolerance - 2e-4;
                if (length > policy.tolerance) {
                    v = mul(delta[f], 1 - tol / length);
                    for (int i = 0; i < n; i++)
                        jv[i] = add(
                            mul(dj[i], 1 - tol / length),
                            mul(delta[f], tol * dot(delta[f], dj[i]) / (length * length * length)));
                }
                double w = std::sqrt(policy.contact_weight * gates[f]) / width;
                for (int c = 0; c < 3; c++) {
                    Q j{};
                    for (int i = 0; i < n; i++)
                        j[i] = jv[i][c] * w;
                    row(v[c] * w, j);
                }
            }
            e.restore_end = e.rows;
            e.key.outside = e.key.violation > 1e-6;
            if (!e.key.outside)
                e.key.violation = 0;
            for (int f = 0; f < 4; f++)
                if (!contact[f] && gates[f] > 1e-8) {
                    double w = std::sqrt(policy.opening_weight) * gates[f] / width;
                    for (int c = 0; c < 3; c++) {
                        Q j{};
                        for (int i = 0; i < n; i++)
                            j[i] = pair_jac[f][i][c] * w;
                        double v = delta[f][c] * w;
                        row(v, j);
                        e.key.pose += v * v;
                    }
                }
            return e;
        };
        Q x{}, candidate{}, best_x{};
        Evaluation current, best;
        int updates = 0, attempts = 0, stalled = 0;
        {
            py::gil_scoped_release release;
            for (int i = 0; i < n; i++)
                x[i] = std::clamp(previous[indices[i]], lo[i], hi[i]);
            current = evaluate(x);
            best = current;
            best_x = x;
            auto merit = [&](const Evaluation &e) {
                return e.key.pose + e.key.regularization +
                       policy.initial_weight * std::pow(e.key.violation / width, 2);
            };
            auto consider = [&](Q v) {
                Evaluation other = evaluate(v);
                if (other.key < best.key) {
                    best = other;
                    best_x = v;
                }
                if (merit(other) < merit(current)) {
                    current = other;
                    x = v;
                }
            };
            for (int i = 0; i < n; i++)
                candidate[i] = std::clamp(i < 3 ? neutral[i] : base[indices[i]], lo[i], hi[i]);
            consider(candidate);
            int seed = 0;
            double seed_cost = 1e100;
            for (size_t i = 0; i < seed_angles.size(); i++) {
                double cost = 0;
                for (int b = 0; b < 3; b++)
                    cost += dot(sub(seed_directions[i][b], desired[b]),
                                sub(seed_directions[i][b], desired[b])) *
                            weights[b];
                if (cost < seed_cost) {
                    seed_cost = cost;
                    seed = int(i);
                }
            }
            for (int i = 0; i < n; i++)
                candidate[i] =
                    std::clamp(i < 3 ? seed_angles[seed][i] : base[indices[i]], lo[i], hi[i]);
            consider(candidate);
            double damping = policy.damping;
            for (int iteration = 0; iteration < policy.updates; iteration++) {
                check();
                Square h{};
                Q gradient{};
                int begin = current.key.outside ? current.restore_begin : 0,
                    end = current.key.outside ? current.restore_end : current.rows;
                for (int r = begin; r < end; r++)
                    for (int i = 0; i < n; i++) {
                        gradient[i] += current.jac[r][i] * current.residual[r];
                        for (int j = 0; j < n; j++)
                            h[i][j] += current.jac[r][i] * current.jac[r][j];
                    }
                for (int i = 0; i < n; i++)
                    h[i][i] += damping;
                std::array<bool, 10> blocked{};
                Q step{};
                for (int pass = 0; pass < n + 1; pass++) {
                    Square ch = h;
                    Q rhs{};
                    for (int i = 0; i < n; i++)
                        rhs[i] = -gradient[i];
                    for (int i = 0; i < n; i++)
                        if (blocked[i]) {
                            for (int j = 0; j < n; j++)
                                ch[i][j] = ch[j][i] = 0;
                            ch[i][i] = 1;
                            rhs[i] = 0;
                        }
                    step = linear_solve(ch, rhs, n);
                    bool added = false;
                    for (int i = 0; i < n; i++)
                        if (!blocked[i] && ((x[i] <= lo[i] + 1e-9 && step[i] < 0) ||
                                            (x[i] >= hi[i] - 1e-9 && step[i] > 0))) {
                            blocked[i] = true;
                            added = true;
                        }
                    if (!added) {
                        h = ch;
                        break;
                    }
                }
                for (int pass = 0; pass < 2; pass++)
                    for (const auto &c : current.constraints) {
                        Q normal = c.normal;
                        double violation = -c.margin;
                        for (int i = 0; i < n; i++) {
                            if (blocked[i])
                                normal[i] = 0;
                            violation += normal[i] * step[i];
                        }
                        if (violation > 0) {
                            Q direction = linear_solve(h, normal, n);
                            double den = 0;
                            for (int i = 0; i < n; i++)
                                den += normal[i] * direction[i];
                            for (int i = 0; i < n; i++)
                                step[i] -= direction[i] * violation / std::max(den, 1e-12);
                        }
                    }
                double largest = 0;
                for (int i = 0; i < n; i++)
                    largest = std::max(largest, std::abs(step[i]));
                double scale = std::min(1., .3 / std::max(largest, 1e-12));
                for (int i = 0; i < n; i++) {
                    if (step[i] > 1e-12)
                        scale = std::min(scale, (hi[i] - x[i]) / step[i]);
                    if (step[i] < -1e-12)
                        scale = std::min(scale, (lo[i] - x[i]) / step[i]);
                }
                largest = 0;
                for (int i = 0; i < n; i++) {
                    step[i] *= std::max(scale, 0.);
                    largest = std::max(largest, std::abs(step[i]));
                }
                if (largest < 1e-7)
                    break;
                bool accepted = false;
                for (double scale : {1., .5}) {
                    attempts++;
                    Q trial_x = x;
                    for (int i = 0; i < n; i++)
                        trial_x[i] = std::clamp(x[i] + scale * step[i], lo[i], hi[i]);
                    auto trial = evaluate(trial_x);
                    if (trial.key < current.key) {
                        x = trial_x;
                        current = trial;
                        damping = std::max(damping * .5, 1e-6);
                        accepted = true;
                        stalled = 0;
                        break;
                    }
                }
                updates++;
                if (!accepted) {
                    stalled++;
                    damping = std::min(damping * 10, 10.);
                    if (stalled >= 2)
                        break;
                }
            }
            check();
            if (best.key < current.key) {
                x = best_x;
                current = best;
            }
        }
        Q q = base;
        for (int i = 0; i < n; i++)
            q[indices[i]] = x[i];
        auto points = assemble(current.thumb, current.fingers);
        py::array_t<double> result_q(10), result_points({26, 3}), result_dirs({3, 3});
        auto rq = result_q.mutable_unchecked<1>();
        auto rp = result_points.mutable_unchecked<2>();
        auto rd = result_dirs.mutable_unchecked<2>();
        for (int i = 0; i < 10; i++)
            rq(i) = q[i];
        for (int p = 0; p < 26; p++)
            for (int c = 0; c < 3; c++)
                rp(p, c) = points[p][c];
        for (int b = 0; b < 3; b++)
            for (int c = 0; c < 3; c++)
                rd(b, c) = current.directions[b][c];
        py::dict out;
        out["joints"] = result_q;
        out["points"] = result_points;
        out["directions"] = result_dirs;
        out["evaluations"] = evaluations;
        out["updates"] = updates;
        out["attempts"] = attempts;
        out["dof"] = n;
        return out;
    }
};
PYBIND11_MODULE(_hand_native_ext, m) {
    py::register_exception<Deadline>(m, "Deadline", PyExc_TimeoutError);
    py::class_<Model>(m, "Model")
        .def(py::init<py::dict, py::dict>())
        .def("solve", &Model::solve)
        .def("landmarks", &Model::landmarks);
}
