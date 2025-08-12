// ===================== tools/generate_map.cpp =====================
#include "path_planning/grid_map.hpp"
#include <iostream>
#include <string>

using namespace path_planning;

static void usage() {
    std::cerr << "Usage: generate_map --w W --h H [--density D] [--seed S]\\n"
                 "                     [--rooms N] [--room-min 6x6] [--room-max 14x14]\\n"
                 "                     [--corridor W] [--cluster I] [--no-border]\\n"
                 "                     [--ascii out.asc | --pgm out.pgm]\\n";
}

int main(int argc, char** argv) {
    GridMap::RandomParams p;
    std::string out_pgm, out_ascii;

    for (int i=1; i<argc; ++i) {
        std::string a = argv[i];
        auto next = [&](int& dst){ if (i+1>=argc) { usage(); return false; } dst = std::stoi(argv[++i]); return true; };
        auto nextd = [&](double& dst){ if (i+1>=argc) { usage(); return false; } dst = std::stod(argv[++i]); return true; };
        if (a=="--w" && !next(p.width)) return 1;
        else if (a=="--h" && !next(p.height)) return 1;
        else if (a=="--density" && !nextd(p.obstacle_density)) return 1;
        else if (a=="--seed" && !next(reinterpret_cast<int&>(p.seed))) return 1;
        else if (a=="--rooms" && !next(p.num_rooms)) return 1;
        else if (a=="--room-min") { if (i+1>=argc) { usage(); return 1; } sscanf(argv[++i], "%dx%d", &p.room_min_w, &p.room_min_h); }
        else if (a=="--room-max") { if (i+1>=argc) { usage(); return 1; } sscanf(argv[++i], "%dx%d", &p.room_max_w, &p.room_max_h); }
        else if (a=="--corridor" && !next(p.corridor_width)) return 1;
        else if (a=="--cluster" && !next(p.cluster_iterations)) return 1;
        else if (a=="--no-border") { p.add_border_walls = false; }
        else if (a=="--ascii") { if (i+1>=argc) { usage(); return 1; } out_ascii = argv[++i]; }
        else if (a=="--pgm") { if (i+1>=argc) { usage(); return 1; } out_pgm = argv[++i]; }
        else { usage(); return 1; }
    }

    if (p.width<=0 || p.height<=0) { usage(); return 1; }

    auto map = GridMap::random(p);

    bool ok = false;
    if (!out_pgm.empty()) ok |= map.savePGM(out_pgm, true);
    if (!out_ascii.empty()) ok |= map.saveASCII(out_ascii);

    if (!ok) {
        std::cerr << "No output specified or failed to save. Use --pgm or --ascii." << std::endl;
        return 2;
    }

    std::cout << "Generated map " << p.width << "x" << p.height << " with settings.\n";
    return 0;
}

