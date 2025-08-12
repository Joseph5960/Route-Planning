# 路径规划项目 / Route Planning

一个使用 **C++** 和 **Python** 完成的完整路径规划工程项目，包含地图构建、多种路径搜索算法（Dijkstra、A*、Fuzzy A* 等），以及性能对比和可视化动画。

A complete path planning project implemented in **C++** and **Python**, featuring map construction, multiple search algorithms (Dijkstra, A*, Fuzzy A*, etc.), and performance visualization.

---

## 功能特点 / Features

- **地图构建 / Map Construction**  
  支持基于栅格和加权图的地图表示，并提供文件读写（PGM/PNG/ASCII）。

- **路径规划算法 / Path Planning Algorithms**  
  - Dijkstra
  - A*
  - Fuzzy A*
  - （可选）Jump Point Search、Theta*、D* Lite、RRT*

- **性能基准测试 / Performance Benchmark**  
  对比算法的运行时间、节点扩展数、路径长度等指标。

- **可视化 / Visualization**  
  使用 Python（`matplotlib`、`pygame`）动态展示搜索过程。

---

## 技术栈 / Tech Stack
- **核心 / Core**: C++17 / CMake
- **可视化 / Visualization**: Python 3.x
- **测试 / Testing**: GoogleTest 或 Catch2
- **基准测试 / Benchmark**: Google Benchmark
- **语言绑定 / Bindings**: pybind11（C++ ↔ Python）

---

## 项目结构 / Project Structure
```
项目结构
path-planning/
├─ CMakeLists.txt
├─ cmake/                         # CMake 模块（FetchContent等）
├─ third_party/                   # 外部依赖（pybind11 / fmt / gtest / benchmark）
├─ include/                       # 公共头文件（对外API）
│  ├─ path_planning/
│  │  ├─ types.hpp               # Node/Coord/Cost/PathResult等基础类型
│  │  ├─ grid_map.hpp            # GridMap接口与数据结构（仅声明）
│  │  ├─ graph_map.hpp           # 道路图/一般图接口
│  │  ├─ heuristic.hpp           # 启发式接口与常用实现（Manhattan/Euclidean/Diagonal）
│  │  ├─ cost_model.hpp          # 代价模型（含模糊/风险代价接口）
│  │  ├─ frontier.hpp            # Open/Closed集合抽象（优先队列等）
│  │  ├─ search_base.hpp         # ISearch基类 & 参数结构
│  │  ├─ factory.hpp             # 算法工厂（字符串/枚举→实例）
│  │  └─ version.hpp             # 版本信息
│  └─ path_planning.hpp          # “单头入口”，汇总对外可见API
│
├─ src/
│  ├─ map/
│  │  ├─ grid_map.cpp            # GridMap实现（读写PGM/PNG/ASCII）
│  │  └─ graph_map.cpp           # 一般图/道路图实现（OSM预处理后的图）
│  ├─ heuristics/
│  │  └─ heuristic.cpp           # 常用启发式实现
│  ├─ cost/
│  │  └─ cost_model.cpp          # 代价/风险/模糊数去模糊化策略
│  ├─ core/
│  │  ├─ frontier_binary_heap.cpp# 二叉堆OpenList实现
│  │  └─ utils.cpp               # 通用工具（计时、范围检查、IO）
│  ├─ algos/                     # ★ 每个算法一个{hpp,cpp}（或hpp+inl）
│  │  ├─ dijkstra.hpp
│  │  ├─ dijkstra.cpp
│  │  ├─ astar.hpp
│  │  ├─ astar.cpp
│  │  ├─ fuzzy_astar.hpp
│  │  ├─ fuzzy_astar.cpp
│  │  ├─ jps.hpp                 # 选做
│  │  ├─ jps.cpp
│  │  ├─ theta_star.hpp          # 选做
│  │  └─ theta_star.cpp
│  ├─ bench/
│  │  └─ cpp_benchmarks.cpp      # Google Benchmark基准
│  ├─ bindings/
│  │  └─ pybind_module.cpp       # Python绑定（可选）
│  └─ main/
│     └─ demo_cli.cpp            # 命令行Demo：加载地图→运行算法→输出路径
│
├─ python/                        # 可视化与实验（通过pybind11或文件IO）
│  ├─ experiments/
│  │  ├─ run_demo.py             # 一键跑Demo并可视化
│  │  └─ sweep_benchmarks.py     # 批量实验/导出csv
│  ├─ viz/
│  │  ├─ animate_grid.py         # 栅格搜索动画
│  │  └─ plot_compare.py         # 算法对比柱状图/雷达图
│  └─ io/
│     └─ loaders.py              # 读写中间文件（若不直接调bindings）
│
├─ data/
│  ├─ maps/                      # PGM/PNG/ASCII地图
│  └─ graphs/                    # 预处理后道路图（edge list/adj list）
│
├─ tests/
│  ├─ CMakeLists.txt
│  ├─ test_grid_map.cpp
│  ├─ test_dijkstra.cpp
│  ├─ test_astar.cpp
│  ├─ test_fuzzy_astar.cpp
│  └─ golden/                    # 固定输入/期望输出（回归测试）
│
├─ docs/
│  ├─ design.md                  # 架构设计&扩展指南
│  ├─ algorithms.md              # 各算法说明与复杂度
│  └─ api.md                     # C++/Python API用法
└─ tools/
   ├─ build_all.sh               # 一键构建/测试/benchmark脚本
   └─ format_lint.sh             # clang-format/clang-tidy
```

---

## 许可证 / License
本项目基于 [MIT License](LICENSE) 开源。

---

## 作者 / Author
- GitHub: [Joseph5960](https://github.com/Joseph5960)
