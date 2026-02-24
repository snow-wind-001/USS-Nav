# USS-Nav 复现问题与解决策略（基于论文原文）

## 1. 复现中的关键缺口

1. **官方代码与仿真环境未公开**
   - 论文说明代码将公开，但当前不可得。
   - 仿真依赖 `Marsim + Unity`，无法直接 1:1 复刻。

2. **SCG 细节存在未定义参数**
   - `r_vis` 未给固定值。
   - Boundary surfaces 聚类中的“分裂阈值”和法向偏差阈值未给。
   - Seed validity 的安全边界与 non-enclosure 判据未给具体公式。

3. **Region 增量聚类未给完整工程细节**
   - 只给了“局部子图重聚类 + 合并”思想。
   - Region ID 稳定策略未给具体实现。

4. **对象融合存在实现自由度**
   - 给了公式与阈值（语义 0.75、几何 0.1/0.5），但点云预处理、采样、动态物体处理未固定。

5. **LLM 粗到细决策输入 schema 未完整公开**
   - 仅描述 Current Area / Visit History / Regional Description。
   - Prompt 模板、先验注入方式（邻接先验）未公开。

6. **TSP 代价矩阵未给明确公式**
   - 论文只说明由 path cost 与 info gain 组成。

## 2. 本工程采用的可执行默认解

1. **工程策略**
   - 先保证 `M1->M4` 数据流可运行，再替换真实感知模型和 LLM。
   - 所有参数集中到 `uss_nav_bringup/config/params.yaml`。

2. **参数默认值（严格对齐论文 + 必要补全）**
   - Rolling grid: `8x8x4m, resolution=0.1`
   - SCG: `fibonacci_samples=50, max_ray_radius=1.4, max_seed_extension=2.0`
   - Region: `leiden_resolution=0.02`
   - GCM: `cell_size=1.0, visited_unknown_threshold=50`（补全）
   - SCG loop closure: `r_vis=3.0`（补全，可调）
   - Region forced edge: `epsilon=0.05`（补全）
   - TSP-like local ordering: `alpha=1.0, beta=0.5`（补全）

3. **对象融合实现（遵循论文公式）**
   - 几何相似度 `Omega_geo`：使用点到集合最近邻阈值比例。
   - 语义相似度 `Omega_sem`：cosine similarity。
   - 匹配规则：
     - 语义优先：`Omega_sem > 0.75 && Omega_geo > 0.1`
     - 几何兜底：`Omega_geo(A,B)>0.5 && Omega_geo(B,A)>0.5`

4. **LLM 模块策略**
   - 先实现 rule-based 区域选择闭环，保留 LLM 接入点。
   - 当语义上下文不足时自动回退 coverage-first（符合论文 V-B 描述）。

## 3. 当前代码覆盖范围

- 已实现 ROS2 三包骨架：
  - `uss_nav_interfaces`
  - `uss_nav_stack`
  - `uss_nav_bringup`
- 已实现首版节点：
  - `rolling_grid_node`
  - `gcm_node`
  - `frontier_node`
  - `scg_node`
  - `region_node`
  - `object_graph_node`（合成观测占位）
  - `planner_node`
- 已提供一键启动：
  - `uss_nav_bringup/launch/bringup.launch.py`

## 4. 下一步需要你确认的高影响选项

1. 运行模式：`仿真优先` 还是 `真机传感器流（Fast-LIO2 + D455）`
2. 最小闭环目标：`A 纯探索` / `B 语义找物` / `C 接 LLM API`
3. LLM 方案：`Qwen API` / `本地模型` / `先规则后接入`
