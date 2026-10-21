// Exact grid kernels for fine-resolution voxel exploration. No ROS dependency.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <utility>
#include <vector>
#include <array>
#include <functional>

namespace {
constexpr double INF = std::numeric_limits<double>::infinity();
struct Ray {
    int64_t cell[3], end[3], step[3];
    double next[3], increment[3];
    Ray(const double* start, const double* target, const double* origin, double res) {
        for (int a = 0; a < 3; ++a) {
            cell[a] = static_cast<int64_t>(std::floor((start[a] - origin[a]) / res));
            end[a] = static_cast<int64_t>(std::floor((target[a] - origin[a]) / res));
            double delta = target[a] - start[a];
            step[a] = (delta > 0) - (delta < 0);
            increment[a] = delta == 0 ? INF : res / std::abs(delta);
            next[a] = delta == 0 ? INF :
                (origin[a] + (cell[a] + (step[a] > 0)) * res - start[a]) / delta;
        }
    }
    bool finished() const {
        return cell[0] == end[0] && cell[1] == end[1] && cell[2] == end[2];
    }
    int64_t id(const int64_t* shape) const {
        for (int a = 0; a < 3; ++a)
            if (cell[a] < 0 || cell[a] >= shape[a]) return -1;
        return (cell[2] * shape[1] + cell[1]) * shape[0] + cell[0];
    }
    void advance() {
        double t = std::min({next[0], next[1], next[2]});
        for (int a = 0; a < 3; ++a) if (next[a] <= t + 1e-12) {
            cell[a] += step[a];
            next[a] += increment[a];
        }
    }
    int64_t limit() const {
        return std::abs(end[0] - cell[0]) + std::abs(end[1] - cell[1]) +
               std::abs(end[2] - cell[2]) + 1;
    }
};
}

extern "C" int64_t voxel_visible(
    const int8_t* state, const int64_t* shape, const double* origin, double res,
    const double* start, const int64_t* targets, int64_t count,
    double min_range, double max_range, double vfov_min, double vfov_max,
    int optimistic, int64_t* output) {
    int64_t size = shape[0] * shape[1] * shape[2], written = 0;
    for (int64_t i = 0; i < count; ++i) {
        int64_t id = targets[i];
        if (id < 0 || id >= size) continue;
        double target[3] = {
            origin[0] + (id % shape[0] + .5) * res,
            origin[1] + (id / shape[0] % shape[1] + .5) * res,
            origin[2] + (id / (shape[0] * shape[1]) + .5) * res};
        double dx = target[0] - start[0], dy = target[1] - start[1], dz = target[2] - start[2];
        double d2 = dx*dx + dy*dy + dz*dz;
        if (d2 < min_range*min_range || d2 > max_range*max_range) continue;
        double angle = std::atan2(dz, std::hypot(dx, dy));
        if (angle < vfov_min || angle > vfov_max) continue;
        Ray ray(start, target, origin, res);
        int64_t source = ray.id(shape), limit = ray.limit();
        bool visible = true;
        for (int64_t k = 0; k < limit && !ray.finished(); ++k, ray.advance()) {
            int64_t cell = ray.id(shape);
            if (cell == source) continue;
            if (cell < 0 || state[cell] == 1 || (!optimistic && state[cell] == -1)) {
                visible = false;
                break;
            }
        }
        if (visible) output[written++] = id;
    }
    return written;
}

extern "C" void voxel_integrate(
    int8_t* state, const int64_t* shape, const double* origin, double res,
    const double* start, const double* endpoints, const uint8_t* hits, int64_t count) {
    std::vector<int64_t> occupied;
    occupied.reserve(count);
    for (int64_t i = 0; i < count; ++i) {
        Ray ray(start, endpoints + 3*i, origin, res);
        int64_t limit = ray.limit();
        for (int64_t k = 0; k < limit; ++k) {
            int64_t id = ray.id(shape);
            bool done = ray.finished();
            if (id >= 0) {
                if (done && hits[i]) occupied.push_back(id);
                else state[id] = 0;
            }
            if (done) break;
            ray.advance();
        }
    }
    for (auto id : occupied) state[id] = 1;
}

extern "C" void voxel_raycast(
    const int8_t* state, const int64_t* shape, const double* origin, double res,
    const double* start, const double* endpoints, int64_t count, int64_t* hits) {
    for (int64_t i = 0; i < count; ++i) {
        hits[i] = -1;
        Ray ray(start, endpoints + 3*i, origin, res);
        int64_t limit = ray.limit();
        for (int64_t k = 0; k < limit; ++k) {
            int64_t id = ray.id(shape);
            if (id >= 0 && state[id] == 1) { hits[i] = id; break; }
            if (ray.finished()) break;
            ray.advance();
        }
    }
}

extern "C" void voxel_first_unknown(
    const int8_t* state, const int64_t* shape, const double* origin, double res,
    const double* start, const double* endpoints, int64_t count, double min_range, int64_t* hits) {
    for (int64_t i = 0; i < count; ++i) {
        hits[i] = -1;
        const double* target = endpoints + 3*i;
        double dx = target[0]-start[0], dy = target[1]-start[1], dz = target[2]-start[2];
        double length = std::sqrt(dx*dx+dy*dy+dz*dz), entry = 0.;
        Ray ray(start, target, origin, res);
        int64_t limit = ray.limit();
        for (int64_t k = 0; k < limit && entry < 1.-1e-9; ++k) {
            int64_t id = ray.id(shape);
            double leaving = std::min({ray.next[0], ray.next[1], ray.next[2], 1.});
            if (id >= 0) {
                if (state[id] == 1) break;
                if (state[id] == -1) {
                    if (leaving*length > min_range+1e-9) hits[i] = id;
                    break;
                }
            }
            if (ray.finished()) break;
            entry = leaving;
            ray.advance();
        }
    }
}

// A full revolution has the same voxel traversal at every grid-centred view.
// Cache geometry only; occupancy and ownership are re-read on every planning
// snapshot. An already claimed unknown STILL occludes subsequent ray cells.
static int64_t collect_views(
    const int8_t* state, const int64_t* shape, const double* origin, double res,
    const double* starts, int64_t views, const double* directions, int64_t rays,
    double min_range, const uint8_t* allowed, int32_t* owners,
    int64_t* hits, int64_t capacity, int64_t* offsets) {
    if (!views) return 0;
    int64_t written = 0;
    // Preorder ray-prefix trie. A blocked node skips its entire subtree.
    // Shared prefixes do not synthesize rays or join paths after divergence.
    struct Cell { int16_t x, y, z; uint8_t measurable; int32_t end; };
    struct Node { Cell cell; int32_t first = -1, next = -1; };
    struct Pattern {
        double res = 0., blind = 0.;
        std::array<double, 3> fraction{};
        std::vector<double> directions;
        std::vector<Cell> cells;
    };
    static thread_local Pattern pattern;
    std::array<double, 3> fraction;
    for (int a = 0; a < 3; ++a) {
        double value = (starts[a]-origin[a])/res;
        fraction[a] = value-std::floor(value);
    }
    bool same = pattern.res == res && pattern.blind == min_range &&
        pattern.directions.size() == size_t(3*rays) &&
        std::equal(pattern.directions.begin(), pattern.directions.end(), directions);
    for (int a = 0; a < 3; ++a)
        same = same && std::abs(pattern.fraction[a]-fraction[a]) < 1e-9;
    for (int64_t v = 0; v < views; ++v) for (int a = 0; a < 3; ++a) {
        double value = (starts[3*v+a]-origin[a])/res;
        if (std::abs(value-std::floor(value)-fraction[a]) > 1e-9) return -1;
    }
    if (!same) {
        pattern = Pattern{};
        pattern.fraction = fraction;
        const double zero[3] = {0., 0., 0.};
        double source[3];
        for (int a = 0; a < 3; ++a) source[a] = fraction[a]*res;
        std::vector<Node> nodes(1, Node{{0, 0, 0, 0, 0}});
        for (int64_t i = 0; i < rays; ++i) {
            double target[3], length2 = 0.;
            for (int a = 0; a < 3; ++a) {
                target[a] = source[a]+directions[3*i+a];
                length2 += directions[3*i+a]*directions[3*i+a];
            }
            const double length = std::sqrt(length2);
            Ray ray(source, target, zero, res);
            const int64_t limit = ray.limit();
            double entry = 0.;
            int32_t parent = 0;
            for (int64_t k = 0; k < limit && entry < 1.-1e-9; ++k) {
                for (int a = 0; a < 3; ++a)
                    if (std::abs(ray.cell[a]) > 32767) { pattern = Pattern{}; return -1; }
                double leaving = std::min({ray.next[0], ray.next[1], ray.next[2], 1.});
                int32_t node = 0;
                if (k) {
                    node = nodes[parent].first;
                    while (node >= 0) {
                        const auto& c = nodes[node].cell;
                        if (c.x == ray.cell[0] && c.y == ray.cell[1] && c.z == ray.cell[2]) break;
                        node = nodes[node].next;
                    }
                    if (node < 0) {
                        node = nodes.size();
                        nodes.push_back(Node{{int16_t(ray.cell[0]), int16_t(ray.cell[1]),
                                              int16_t(ray.cell[2]), 0, 0},
                                             -1, nodes[parent].first});
                        nodes[parent].first = node;
                    }
                }
                // At an unknown node, any beam that reaches beyond the blind
                // radius is sufficient evidence of potential observability.
                nodes[node].cell.measurable |= uint8_t(leaving*length > min_range+1e-9);
                parent = node;
                if (ray.finished()) break;
                entry = leaving;
                ray.advance();
            }
        }
        pattern.cells.reserve(nodes.size());
        std::function<void(int32_t)> flatten = [&](int32_t node) {
            int32_t index = pattern.cells.size();
            pattern.cells.push_back(nodes[node].cell);
            for (int32_t child = nodes[node].first; child >= 0; child = nodes[child].next)
                flatten(child);
            pattern.cells[index].end = pattern.cells.size();
        };
        flatten(0);
        pattern.res = res;
        pattern.blind = min_range;
        pattern.directions.assign(directions, directions+3*rays);
    }
    for (int64_t v = 0; v < views; ++v) {
        if (offsets) offsets[v] = written;
        int64_t base[3];
        for (int a = 0; a < 3; ++a) base[a] = std::floor((starts[3*v+a]-origin[a])/res);
        bool inside = true;
        double farthest2 = 0.;
        for (int a = 0; a < 3; ++a) {
            inside &= base[a] >= 0 && base[a] < shape[a];
            double edge = std::max(fraction[a], 1.-fraction[a])*res;
            farthest2 += edge*edge;
        }
        if (!inside) return -1;
        int64_t source = (base[2]*shape[1]+base[1])*shape[0]+base[0];
        if (state[source] == 1 || (state[source] == -1 && farthest2 <= min_range*min_range)) continue;
        for (int32_t k = 0; k < int32_t(pattern.cells.size());) {
            const auto& c = pattern.cells[k];
            int64_t x = base[0]+c.x, y = base[1]+c.y, z = base[2]+c.z;
            if (x < 0 || x >= shape[0] || y < 0 || y >= shape[1] || z < 0 || z >= shape[2]) {
                k = c.end;
                continue;
            }
            int64_t id = (z*shape[1]+y)*shape[0]+x;
            if (state[id] == 0) { ++k; continue; }
            if (state[id] == -1 && c.measurable && allowed[id] &&
                    (offsets ? owners[id] != v : owners[id] < 0)) {
                owners[id] = v;
                if (offsets) {
                    if (written < capacity) hits[written] = id;
                    ++written;
                }
            }
            k = c.end;  // Claimed unknowns still occlude all descendant rays.
        }
    }
    if (offsets) offsets[views] = written;
    return written;
}

extern "C" int voxel_claim_views(
    const int8_t* state, const int64_t* shape, const double* origin, double res,
    const double* starts, int64_t views, const double* directions, int64_t rays,
    double min_range, const uint8_t* allowed, int32_t* owners) {
    return collect_views(state, shape, origin, res, starts, views, directions, rays,
                         min_range, allowed, owners, nullptr, 0, nullptr) >= 0;
}

// Independent visible sets (CSR), before nearest ownership. The return value
// is the required capacity; a small caller buffer can be safely retried.
extern "C" int64_t voxel_collect_views(
    const int8_t* state, const int64_t* shape, const double* origin, double res,
    const double* starts, int64_t views, const double* directions, int64_t rays,
    double min_range, const uint8_t* allowed, int32_t* owners,
    int64_t* hits, int64_t capacity, int64_t* offsets) {
    return collect_views(state, shape, origin, res, starts, views, directions, rays,
                         min_range, allowed, owners, hits, capacity, offsets);
}

// Label 26-connected unknown regions without changing occupancy evidence.
extern "C" int voxel_unknown_components(const int8_t* state, const int64_t* shape,
                                        int32_t* labels) {
    const int nx = shape[0], ny = shape[1], nz = shape[2];
    const int64_t size = int64_t(nx)*ny*nz;
    std::fill(labels, labels+size, 0);
    int component = 0;
    std::vector<int64_t> queue;
    for (int64_t seed = 0; seed < size; ++seed) {
        if (state[seed] != -1 || labels[seed]) continue;
        ++component;
        queue.clear();
        queue.push_back(seed);
        labels[seed] = component;
        for (size_t head = 0; head < queue.size(); ++head) {
            const int64_t id = queue[head];
            const int x = id%nx, y = id/nx%ny, z = id/(int64_t(nx)*ny);
            for (int dz = -1; dz <= 1; ++dz)
                for (int dy = -1; dy <= 1; ++dy)
                    for (int dx = -1; dx <= 1; ++dx) {
                        const int xx = x+dx, yy = y+dy, zz = z+dz;
                        if (xx < 0 || xx >= nx || yy < 0 || yy >= ny || zz < 0 || zz >= nz) continue;
                        const int64_t next = (int64_t(zz)*ny+yy)*nx+xx;
                        if (state[next] == -1 && !labels[next]) {
                            labels[next] = component;
                            queue.push_back(next);
                        }
                    }
        }
    }
    return component;
}

// Exact incremental small-island classification for UNKNOWN deletions only.
// Every newly split fragment touches a deleted voxel. Search those neighbours
// only, stopping once K connected cells prove that a fragment is not small.
// Proof marks are local to this update: old large components may have split.
extern "C" void voxel_update_islands(const int8_t* state, const int64_t* shape,
                                     const int64_t* removed, int64_t count,
                                     int64_t threshold, uint8_t* ignored) {
    const int nx = shape[0], ny = shape[1], nz = shape[2];
    const int64_t size = int64_t(nx)*ny*nz;
    std::vector<int32_t> seen(size, 0);
    std::vector<int64_t> queue;
    int32_t token = 0;
    auto neighbours = [&](int64_t id, auto visit) {
        const int x = id%nx, y = id/nx%ny, z = id/(int64_t(nx)*ny);
        for (int dz = -1; dz <= 1; ++dz)
            for (int dy = -1; dy <= 1; ++dy)
                for (int dx = -1; dx <= 1; ++dx) {
                    const int xx=x+dx, yy=y+dy, zz=z+dz;
                    if (xx>=0 && xx<nx && yy>=0 && yy<ny && zz>=0 && zz<nz)
                        visit((int64_t(zz)*ny+yy)*nx+xx);
                }
    };
    for (int64_t i=0; i<count; ++i) ignored[removed[i]]=0;
    for (int64_t i=0; i<count; ++i) neighbours(removed[i], [&](int64_t seed) {
        if (state[seed]!=-1 || ignored[seed] || seen[seed]==-1) return;
        ++token;
        queue.clear(); queue.push_back(seed); seen[seed]=token;
        bool large = threshold<=1;
        for (size_t head=0; head<queue.size() && !large; ++head) {
            neighbours(queue[head], [&](int64_t next) {
                if (large || state[next]!=-1 || seen[next]==token) return;
                if (seen[next]==-1) { large=true; return; }
                seen[next]=token; queue.push_back(next);
                if (int64_t(queue.size())>=threshold) large=true;
            });
        }
        for (auto id: queue) {
            if (large) seen[id]=-1;
            else ignored[id]=1;
        }
    });
}

extern "C" void grid_distances(
    const uint8_t* safe, int height, int width, int row, int col,
    double resolution, double offset, double* distance) {
    std::fill(distance, distance + int64_t(height)*width, INF);
    if (row < 0 || col < 0 || row >= height || col >= width || !safe[row*width+col]) return;
    using Entry = std::pair<double, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
    int source = row*width+col;
    distance[source] = offset;
    queue.push({offset, source});
    while (!queue.empty()) {
        auto [cost, id] = queue.top(); queue.pop();
        if (cost > distance[id] + 1e-10) continue;
        int r = id/width, c = id%width;
        for (int dr = -1; dr <= 1; ++dr) for (int dc = -1; dc <= 1; ++dc) {
            if (!dr && !dc) continue;
            int nr = r+dr, nc = c+dc;
            if (nr < 0 || nc < 0 || nr >= height || nc >= width || !safe[nr*width+nc]) continue;
            if (dr && dc && (!safe[(r+dr)*width+c] || !safe[r*width+c+dc])) continue;
            double next = cost + resolution * (dr && dc ? std::sqrt(2.) : 1.);
            int target = nr*width+nc;
            if (next < distance[target] - 1e-10) {
                distance[target] = next;
                queue.push({next, target});
            }
        }
    }
}
