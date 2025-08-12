// ===================== src/map/grid_map.cpp =====================
#include "path_planning/grid_map.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <fstream>
#include <queue>

namespace path_planning {

static inline size_t idx(int x, int y, int w) { return static_cast<size_t>(y) * w + x; }

GridMap::GridMap(int w, int h, uint8_t fill) {
    size_ = {w, h};
    data_.assign(std::max(0, w*h), fill);
}

uint8_t GridMap::at(int x, int y) const {
    if (x < 0 || y < 0 || x >= width() || y >= height()) return 1; // treat OOB as blocked
    return data_[idx(x,y,width())];
}

void GridMap::set(int x, int y, uint8_t value) {
    if (x < 0 || y < 0 || x >= width() || y >= height()) return;
    data_[idx(x,y,width())] = value ? 1 : 0;
}

bool GridMap::isFree(int x, int y) const { return (x>=0 && y>=0 && x<width() && y<height()) && at(x,y)==0; }

void GridMap::clear(uint8_t value) { std::fill(data_.begin(), data_.end(), value ? 1 : 0); }

void GridMap::rectangle(int x0, int y0, int w, int h, uint8_t value, bool filled) {
    int x1 = x0 + w - 1, y1 = y0 + h - 1;
    if (filled) {
        for (int y = y0; y <= y1; ++y) for (int x = x0; x <= x1; ++x) set(x,y,value);
    } else {
        for (int x = x0; x <= x1; ++x) set(x,y0,value), set(x,y1,value);
        for (int y = y0; y <= y1; ++y) set(x0,y,value), set(x1,y,value);
    }
}

// Bresenham line
void GridMap::line(int x0, int y0, int x1, int y1, uint8_t value) {
    int dx = std::abs(x1-x0), sx = x0<x1?1:-1;
    int dy = -std::abs(y1-y0), sy = y0<y1?1:-1; 
    int err = dx + dy, e2;
    while (true) {
        set(x0,y0,value);
        if (x0==x1 && y0==y1) break;
        e2 = 2*err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void GridMap::circle(int cx, int cy, int r, uint8_t value, bool filled) {
    int x = r, y = 0; int err = 0;
    auto plot8 = [&](int px, int py){
        if (filled) {
            for (int ix = cx - px; ix <= cx + px; ++ix) set(ix, cy + py, value), set(ix, cy - py, value);
        } else {
            set(cx + px, cy + py, value); set(cx - px, cy + py, value);
            set(cx + px, cy - py, value); set(cx - px, cy - py, value);
            set(cx + py, cy + px, value); set(cx - py, cy + px, value);
            set(cx + py, cy - px, value); set(cx - py, cy - px, value);
        }
    };
    while (x >= y) { plot8(x,y); y++; if (err <= 0) err += 2*y + 1; if (err > 0) { x--; err -= 2*x + 1; } }
}

// ------------------ I/O ------------------
bool GridMap::saveASCII(const std::string& path) const {
    std::ofstream ofs(path);
    if (!ofs) return false;
    ofs << width() << ' ' << height() << '\n';
    for (int y=0; y<height(); ++y) {
        for (int x=0; x<width(); ++x) ofs << int(at(x,y)) << (x+1==width()? '\n' : ' ');
    }
    return true;
}

GridMap GridMap::loadASCII(const std::string& path) {
    std::ifstream ifs(path);
    int w=0,h=0; ifs >> w >> h; GridMap m(w,h,0);
    for (int y=0; y<h; ++y) for (int x=0; x<w; ++x) { int v; ifs >> v; m.set(x,y, v?1:0); }
    return m;
}

bool GridMap::savePGM(const std::string& path, bool binary) const {
    std::ofstream ofs(path, binary ? std::ios::binary : std::ios::out);
    if (!ofs) return false;
    if (binary) {
        ofs << "P5\n" << width() << ' ' << height() << "\n255\n";
        for (int y=0; y<height(); ++y) for (int x=0; x<width(); ++x) {
            uint8_t pix = at(x,y) ? 0 : 255; // obstacle black, free white
            ofs.write(reinterpret_cast<const char*>(&pix), 1);
        }
    } else {
        ofs << "P2\n" << width() << ' ' << height() << "\n255\n";
        for (int y=0; y<height(); ++y) {
            for (int x=0; x<width(); ++x) {
                int pix = at(x,y) ? 0 : 255; ofs << pix << (x+1==width()? '\n' : ' ');
            }
        }
    }
    return true;
}

GridMap GridMap::loadPGM(const std::string& path) {
    std::ifstream ifs(path, std::ios::binary);
    std::string magic; ifs >> magic; if (magic != "P2" && magic != "P5") return GridMap();
    int w=0,h=0,maxv=255; ifs >> w >> h >> maxv; ifs.get(); // consume one whitespace
    GridMap m(w,h,0);
    if (magic == "P5") {
        for (int y=0; y<h; ++y) for (int x=0; x<w; ++x) {
            unsigned char pix; ifs.read(reinterpret_cast<char*>(&pix),1);
            m.set(x,y, (pix < 128) ? 1 : 0);
        }
    } else { // P2
        for (int y=0; y<h; ++y) for (int x=0; x<w; ++x) {
            int pix; ifs >> pix; m.set(x,y, (pix < 128) ? 1 : 0);
        }
    }
    return m;
}

// ------------------ Procedural generation ------------------
GridMap GridMap::genUniform(const RandomParams& p) {
    GridMap m(p.width, p.height, 0);
    std::mt19937 rng(p.seed);
    std::bernoulli_distribution bern(std::clamp(p.obstacle_density, 0.0, 1.0));
    for (int y=0; y<p.height; ++y) for (int x=0; x<p.width; ++x) if (bern(rng)) m.set(x,y,1);
    if (p.add_border_walls) {
        for (int x=0; x<p.width; ++x) m.set(x,0,1), m.set(x,p.height-1,1);
        for (int y=0; y<p.height; ++y) m.set(0,y,1), m.set(p.width-1,y,1);
    }
    return m;
}

void GridMap::growClusters(GridMap& m, int iterations, Connectivity conn) {
    if (iterations <= 0) return;
    GridMap out(m.width(), m.height(), 0);
    const int dirs8[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
    const int dirs4[4][2] = {{1,0},{-1,0},{0,1},{0,-1}};
    const int (*dirs)[2] = (conn==Connectivity::Eight) ? dirs8 : dirs4;
    const int nd = (conn==Connectivity::Eight) ? 8 : 4;
    for (int it=0; it<iterations; ++it) {
        for (int y=0; y<m.height(); ++y) {
            for (int x=0; x<m.width(); ++x) {
                int cnt = 0;
                for (int k=0; k<nd; ++k) { int nx=x+dirs[k][0], ny=y+dirs[k][1]; if (!m.isFree(nx,ny)) cnt++; }
                // simple cellular automata rule to make blobs
                if (!m.isFree(x,y)) out.set(x,y, cnt >= 3 ? 1 : 0);
                else out.set(x,y, cnt >= 5 ? 1 : 0);
            }
        }
        m = out; // copy
    }
}

GridMap GridMap::genRoomsAndCorridors(const RandomParams& p) {
    GridMap m(p.width, p.height, 1); // start fully blocked, then carve
    std::mt19937 rng(p.seed);
    std::uniform_int_distribution<int> rw(p.room_min_w, p.room_max_w);
    std::uniform_int_distribution<int> rh(p.room_min_h, p.room_max_h);
    std::uniform_int_distribution<int> rx(1, std::max(1,p.width-2));
    std::uniform_int_distribution<int> ry(1, std::max(1,p.height-2));

    struct Rect { int x,y,w,h,cx,cy; };
    std::vector<Rect> rooms;
    rooms.reserve(std::max(0,p.num_rooms));

    auto carve_rect = [&](int x0,int y0,int w,int h){
        for (int y=y0; y<y0+h && y<p.height; ++y)
            for (int x=x0; x<x0+w && x<p.width; ++x)
                m.set(x,y,0);
    };

    // place rooms (naive rejection sampling to avoid heavy overlaps)
    int tries = p.num_rooms * 10;
    while ((int)rooms.size() < p.num_rooms && tries-- > 0) {
        int w = std::min(rw(rng), p.width-2), h = std::min(rh(rng), p.height-2);
        int x = std::min(std::max(1, rx(rng)-w/2), p.width-w-1);
        int y = std::min(std::max(1, ry(rng)-h/2), p.height-h-1);
        Rect r{x,y,w,h, x+w/2, y+h/2};
        bool overlap = false;
        for (const auto& q: rooms) {
            if (r.x < q.x+q.w && r.x+r.w > q.x && r.y < q.y+q.h && r.y+r.h > q.y) { overlap = true; break; }
        }
        if (!overlap) { rooms.push_back(r); carve_rect(r.x,r.y,r.w,r.h); }
    }

    // connect rooms with Manhattan corridors using MST over room centers
    if (rooms.size() >= 2) {
        // Simple Prim MST
        std::vector<int> parent(rooms.size(), -1);
        std::vector<int> key(rooms.size(), INT32_MAX);
        std::vector<bool> inMST(rooms.size(), false);
        key[0]=0; parent[0]=-1;
        for (size_t c=0; c<rooms.size()-1; ++c) {
            int u=-1; int best=INT32_MAX;
            for (size_t i=0;i<rooms.size();++i) if (!inMST[i] && key[i]<best) {best=key[i]; u=(int)i;}
            inMST[u]=true;
            for (size_t v=0; v<rooms.size(); ++v) if (!inMST[v]) {
                int w = std::abs(rooms[u].cx-rooms[v].cx)+std::abs(rooms[u].cy-rooms[v].cy);
                if (w < key[v]) { key[v]=w; parent[v]=u; }
            }
        }
        auto carve_corridor = [&](int x0,int y0,int x1,int y1){
            // carve axis-aligned L-shape with corridor_width
            auto carve_line = [&](int ax,int ay,int bx,int by){
                int dx = (bx>ax)?1:((bx<ax)?-1:0); int dy=(by>ay)?1:((by<ay)?-1:0);
                while (ax!=bx || ay!=by) {
                    for (int ox=-(p.corridor_width/2); ox<=p.corridor_width/2; ++ox)
                        for (int oy=-(p.corridor_width/2); oy<=p.corridor_width/2; ++oy)
                            m.set(ax+ox, ay+oy, 0);
                    ax += dx; ay += dy;
                }
            };
            if (std::bernoulli_distribution(0.5)(rng)) {
                carve_line(x0,y0,x1,y0); carve_line(x1,y0,x1,y1);
            } else {
                carve_line(x0,y0,x0,y1); carve_line(x0,y1,x1,y1);
            }
        };
        for (size_t v=1; v<rooms.size(); ++v) if (parent[v] != -1) {
            carve_corridor(rooms[v].cx, rooms[v].cy, rooms[parent[v]].cx, rooms[parent[v]].cy);
        }
    }

    if (p.add_border_walls) {
        for (int x=0; x<p.width; ++x) m.set(x,0,1), m.set(x,p.height-1,1);
        for (int y=0; y<p.height; ++y) m.set(0,y,1), m.set(p.width-1,y,1);
    }
    return m;
}

GridMap GridMap::random(const RandomParams& p) {
    if (p.num_rooms > 0) {
        auto m = genRoomsAndCorridors(p);
        if (p.cluster_iterations > 0)
            growClusters(m, p.cluster_iterations, p.connectivity);
        return m;
    } else {
        auto m = genUniform(p);
        if (p.cluster_iterations > 0)
            growClusters(m, p.cluster_iterations, p.connectivity);
        return m;
    }
}

} // namespace path_planning