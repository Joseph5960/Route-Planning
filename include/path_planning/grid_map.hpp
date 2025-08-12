// ===================== include/path_planning/grid_map.hpp =====================
#pragma once
#include <cstdint>
#include <random>
#include <string>
#include <vector>

namespace path_planning {

// 8-connected moves or 4-connected moves
enum class Connectivity : uint8_t { Four = 4, Eight = 8 };

// A simple binary occupancy grid: 0 = free, 1 = obstacle
class GridMap {
public:
    struct Size { int width{0}; int height{0}; };

    // Parameters for random map generation
    struct RandomParams {
        int width{64};                 // cells
        int height{64};                // cells
        double obstacle_density{0.20}; // 0.0 ~ 1.0 (used in Uniform mode)
        unsigned int seed{42};
        bool add_border_walls{true};
        // Rooms & corridors mode
        int num_rooms{0};              // if >0, generate rectangular rooms
        int room_min_w{6};
        int room_min_h{6};
        int room_max_w{14};
        int room_max_h{14};
        int corridor_width{2};         // Manhattan corridors between room centers
        // Clustering (optional light-weight clustering over uniform noise)
        int cluster_iterations{0};     // e.g. 2~5 to grow blobs; 0 disables
        Connectivity connectivity{Connectivity::Eight};
    };

    GridMap() = default;
    GridMap(int width, int height, uint8_t fill = 0);

    // Accessors
    inline int width() const { return size_.width; }
    inline int height() const { return size_.height; }
    inline Size size() const { return size_; }

    // Bounds-safe get/set
    uint8_t at(int x, int y) const; // returns 1 for obstacle, 0 for free (out of range -> 1)
    void set(int x, int y, uint8_t value);

    // I/O (portable ASCII PGM P2 and binary PGM P5)
    bool savePGM(const std::string& path, bool binary = true) const;
    static GridMap loadPGM(const std::string& path);

    // I/O plain ASCII: width height then grid rows of 0/1
    bool saveASCII(const std::string& path) const;
    static GridMap loadASCII(const std::string& path);

    // Manual painting helpers
    void clear(uint8_t value = 0);
    void rectangle(int x0, int y0, int w, int h, uint8_t value = 1, bool filled = true);
    void line(int x0, int y0, int x1, int y1, uint8_t value = 1);
    void circle(int cx, int cy, int r, uint8_t value = 1, bool filled = true);

    // Procedural generation entry point
    static GridMap random(const RandomParams& p);

    // Utility: is cell free (inside bounds and value==0)
    bool isFree(int x, int y) const;

private:
    Size size_{};
    std::vector<uint8_t> data_; // row-major, size = width*height

    // internals for generation
    static GridMap genUniform(const RandomParams& p);
    static void growClusters(GridMap& m, int iterations, Connectivity conn);
    static GridMap genRoomsAndCorridors(const RandomParams& p);
};

} // namespace path_planning