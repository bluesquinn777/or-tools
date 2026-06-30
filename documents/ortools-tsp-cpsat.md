**!!! 本报告基于 [google/or-tools at 79e340fd50e1cae499bd6c9035486cf06c8261a5](https://github.com/google/or-tools/tree/79e340fd50e1cae499bd6c9035486cf06c8261a5)（OR-Tools 9.15）!!!**

# 0. 总览：CP-SAT 路径求解 TSP

> 调研对象：OR-Tools 9.15 源码（`ortools/sat/` 以及桥接文件 `ortools/constraint_solver/routing_sat.cc`）
> 调研范围：仅关注 TSP（单旅行商、单回路）在 **CP-SAT 求解器**上的建模与求解。
> 本报告与 `ortools-tsp-traditional-cp.md` 对齐关注点，但只重复两者不同的建模部分。

CP-SAT 是 OR-Tools 的新一代求解器：**CDCL SAT 内核 + 整数界传播 + 内嵌 LP（单纯形）+ 割平面 + 多 worker 组合搜索**。模型统一表示为 protobuf（`CpModelProto`，cp_model.proto），TSP 通过其中的 **`circuit` 约束**表达。

TSP 进入 CP-SAT 有两个入口，最终汇合到同一个 `CircuitConstraintProto`：

1. **直接建模**：用户用 `CpModelBuilder::AddCircuitConstraint()` 逐弧 `AddArc(tail, head, literal)`，目标为 `Minimize(Σ dist(i,j)·literal(i,j))`（cp_model.h:575-595；语义定义 cp_model.proto:193-205）。
2. **从 Routing 库桥接**：`RoutingModel` 在 `use_cp_sat=true`、`use_generalized_cp_sat=true`，或“实例不超过兜底阈值且传统路径没有收集到解”时调用 `SolveModelWithSat`；默认阈值为 20（routing.cc:3449-3471；routing_parameters.cc:181-187）。单车辆模型由 `PopulateSingleRouteModelFromRoutingModel`（routing_sat.cc:384-415）翻译成 circuit 约束。

与传统路径的根本差异是：**传统路径以构造启发式和局部搜索为主；CP-SAT 以传播、冲突学习和下界推进为主。若搜索未被时限等条件截断，CP-SAT 可以证明最优或不可行。**

# 1. TSP 建模

CP-SAT **不是 successor 整数变量模型**。它为每条候选有向弧建立布尔变量：

$$
x_{ij} =
\begin{cases}
1, & \text{回路选择弧 } i \rightarrow j\\
0, & \text{否则}
\end{cases}
$$

对所有必须访问的城市，不添加自环；模型为：

$$
\begin{align}
\min \quad & \sum_{i \ne j} c_{ij}x_{ij} \\
\text{s.t.}\quad
& \sum_{j \ne i} x_{ij}=1, && \forall i \\
& \sum_{j \ne i} x_{ji}=1, && \forall i \\
& \text{Circuit}(x), &&
\end{align}
$$

`Circuit(x)` 不只是入度、出度约束。它还要求所有被选中的非自环弧形成**一个回路**，从而排除多个互不相交的子环。

若节点允许不访问，则必须为它添加自环 `x[i,i]`；`x[i,i]=1` 表示该节点不进入主回路。纯 TSP 不需要这些自环。

与传统 CP 的 successor 模型相比：

| 维度 | 传统 CP Routing | CP-SAT |
| --- | --- | --- |
| 主变量 | 每节点一个整数 `NextVar(i)` | 每条候选弧一个 BoolVar `x[i,j]` |
| 稠密全图变量数 | `O(n)` 个整数变量 | `n(n-1)` 个布尔变量 |
| 子环处理 | `NoCycle(nexts, active)` | `CircuitPropagator` + 可选 LP 子环割 |
| 目标 | `Element(cost[i], NextVar(i))` 后求和 | 平坦线性目标 `Σ c[i,j]x[i,j]` |

因此本报告保留建模章节；后续“变量与约束编码”只解释该模型进入 CP-SAT 内核后的实际形态。

# 2. 求解器结构

## 2.1 核心头文件

本节按 CP-SAT 求解 TSP 的执行链组织：先是模型表达，然后是 presolve/对称性，之后加载到内部传播器，再进入 SAT+整数搜索、LP 松弛与割平面，最后列出 routing 桥接和调度类旁支文件。

| 文件 | 主要功能 | TSP 相关性 |
| --- | --- | --- |
| `cp_model.h/.cc` | 用户层 C++ 建模接口，负责创建变量、目标和约束并写入 `CpModelProto`。 | 直接 CP-SAT TSP 从 `CpModelBuilder::AddCircuitConstraint()` 开始；`CircuitConstraint::AddArc(tail, head, literal)` 逐条加入候选弧（`cp_model.h:580-595, 882`）。 |
| `cp_model.proto` | CP-SAT 的统一模型 IR，前端建模和 routing 翻译最终都落成这个 protobuf。 | TSP 的核心是 `CircuitConstraintProto`，用 `tails/heads/literals` 三个平行数组保存弧；语义要求相关节点一进一出，非自环选中弧形成单一 circuit（`cp_model.proto:193-205`）。 |
| `cp_model_solver.h/.cc` | `SolveCpModel()` 主入口，负责参数、日志、presolve、postsolve、模型加载、worker 组合搜索和 solution callback。 | TSP circuit 从这里进入完整 CP-SAT 流程：presolve 后加载 circuit/LP/cuts，再由 `no_lp/default_lp/max_lp/lb_tree_search/LNS` 等 worker 共享 incumbent 和界（`cp_model_solver.cc:2407`）。 |
| `cp_model_presolve.h/.cc` | CP-SAT PreSolve 主体，在搜索前反复改写 `CpModelProto`：删除约束、固定变量、传播 domain、提取等价关系、规范化线性表达式等。 | TSP 直接相关算子是 `PresolveCircuit()`：重排弧、检查入/出度、删除假弧、固定唯一弧、发现已闭合回路或不可行节点（`cp_model_presolve.cc:7551, 9852`）。 |
| `cp_model_symmetries.h/.cc` | 模型对称性检测与对称破缺约束生成。 | 对称距离矩阵 TSP 天然有反向 tour 等对称；该文件属于 CP-SAT 通用 presolve 后段，由 `DetectAndAddSymmetryToProto()` 注入对称性信息或约束（`cp_model_symmetries.cc:784`，调用点 `cp_model_solver.cc:2816`、`cp_model_presolve.cc:13996`）。 |
| `cp_model_loader.h/.cc` | 把 `CpModelProto` 加载成内部 SAT literal、整数变量、watcher 传播器、LP 约束等对象。 | `LoadCircuitConstraint()` 将 proto 弧转成内部 literal 并重编号节点，然后调用 `LoadSubcircuitConstraint()`；这是 circuit 从模型层进入传播层的入口（`cp_model_loader.cc:1664-1675`）。 |
| `circuit.h/.cc` | circuit/subcircuit 约束传播器，实现入/出度约束装配、子环排除、路径链维护和 reason 生成。 | TSP 单回路语义的核心传播文件；`LoadSubcircuitConstraint()` 先加每点一进一出，再注册 `CircuitPropagator` 排除多个子环（`circuit.cc:720`）。 |
| `all_different.h/.cc` | `AllDifferentConstraint` 传播器，用于整数变量集合互异。 | 基础 TSP 默认不依赖它；只有 `use_all_different_for_circuit=true` 时，circuit 才会额外把“后继互异”交给 all-different 强化传播（`sat_parameters.proto:1108`）。 |
| `integer.h`、`integer_search.h/.cc` | 整数 trail、整数界传播调度总线 `GenericLiteralWatcher`、分支决策和底层 SAT solver 调用。 | TSP 弧 literal、目标变量、LP 下界和 circuit propagator 都通过 watcher/SAT trail 参与传播与回溯；`IntegerSearchHelper::SolveIntegerProblem()` 是整数搜索进入 SAT 内核的重要路径。 |
| `lb_tree_search.h/.cc` | 类 MIP 的 lower-bound tree search worker，维护搜索树节点、LP basis、reduced-cost 推理和 objective lower bound。 | 对优化 TSP，它不是建模必需项，但在 portfolio 中用于强化最优性证明；`lb_tree_search` worker 通常配合 LP/cuts 改进下界（`lb_tree_search.cc:53, 413, 1148`）。 |
| `cp_model_search.h/.cc` | 生成 CP-SAT worker 参数组合，决定是否启用 no-LP/default-LP/max-LP/core/LNS/lb-tree 等搜索策略。 | TSP 是否实际跑 LP 和 routing cuts，取决于 worker 的 `linearization_level` 和 portfolio 选择；`lns_routing` 会显式使用较高线性化等级（`cp_model_search.cc:505-587, 806-811, 844`）。 |
| `linear_relaxation.h/.cc` | 把 CP-SAT 约束转换成 LP 松弛行，并按约束类型挂接 CutGenerator。 | `AppendCircuitRelaxation()` 给 circuit 加入入/出度 LP 行；`linearization_level > 1` 时 `AddCircuitCutGenerator()` 注册子环割生成器（`linear_relaxation.cc:491, 579, 1496-1499`）。 |
| `linear_programming_constraint.h/.cc` | LP 传播器和 GLOP 单纯形接口，负责求 LP、维护 LP basis、添加 cuts、产生 objective cut、MIR/CG/ZeroHalf cut、reduced-cost branching/fixing。 | 优化 TSP 的下界证明主要靠这里把 LP/cuts 的结果反馈给 SAT/整数搜索；它不负责 circuit 语义本身，而是执行 LP 求解与割管理（`linear_programming_constraint.cc:281, 846, 942, 1607, 2141`）。 |
| `cuts.h/.cc` | 通用 cut 数据结构和部分 MIP cut 算法，如 MIR、CG、ZeroHalf、implied bound 等。 | 基础 TSP 的专用子环割不在这里，而在 `routing_cuts.*`；但 LP 传播器仍可能使用这些通用 MIP cuts 改善线性松弛。 |
| `routing_cuts.h/.cc` | routing/circuit 专用割分离算法，围绕图连通性、子环、路径不等式生成 CutGenerator。 | TSP 最关键的 LP 强化文件；`SeparateSubtourInequalities()` 分离子环割，必要时走 Gomory-Hu 精确分离和 Blossom 类割（`routing_cuts.cc:3097, 3208-3235`）。 |
| `scheduling_cuts.h/.cc` | no-overlap、cumulative、scheduling 约束线性化和 cut generator。 | 基础 circuit TSP 不走这里；若模型混入时间窗、interval、no-overlap 或调度式资源约束，才会通过这些 cut 加强 LP。 |
| `integer_expr.h/.cc` | 非线性整数表达式和派生整数约束的传播器，如 product、division、abs、min/max、weighted sum 等。 | 基础 TSP 的目标是线性弧成本，通常不需要这些传播器；若扩展模型含乘除、max、piecewise 表达式，则由 `cp_model_loader.cc` 加载到这里的传播逻辑。 |
| `disjunctive.h/.cc` | `NoOverlap`/disjunctive 约束传播，包括 overload checking、detectable precedences、not-last、edge-finding 等。 | 基础 TSP 不相关；带时间窗或机器调度式扩展时，NoOverlap 约束才会进入该传播器和对应 scheduling cuts。 |
| `sat_solver.h/.cc`、`sat_base.h`、`sat_decision.h/.cc`、`sat_inprocessing.h/.cc` | CDCL SAT 内核与布尔搜索基础设施，包括 trail、子句传播、冲突分析、学习、重启、phase saving、inprocessing。 | TSP 的弧变量是 Bool literal；degree 约束、circuit reason、LP 传播和 learned clauses 最终都落到 SAT trail/冲突学习机制上。 |
| `cp_model_lns.h/.cc` | CP-SAT LNS 邻域生成器，基于已有 incumbent 固定部分变量并重求解子问题。 | 有可行 tour 后，routing 类 LNS worker 可以继续改进 TSP；它是改进解的搜索机制，不是 circuit 的基本语义实现。 |
| `sat_parameters.proto` | CP-SAT 参数定义和默认值。 | TSP 相关开关包括 `linearization_level`、`num_workers`、`optimize_with_lb_tree_search`、`routing_cut_*`、`use_all_different_for_circuit`、`enumerate_all_solutions` 等。 |
| `ortools/constraint_solver/routing_sat.cc` | 传统 `RoutingModel` 到 CP-SAT 的桥接层，把 routing 模型翻译成 `CpModelProto`，并把 SAT 解转回 routing 解。 | 当传统 routing 走 CP-SAT 分支时，单车 TSP 由 `PopulateSingleRouteModelFromRoutingModel()` 翻译成 circuit，再由 `SolveModelWithSat()` 调用 `SolveCpModel()`。 |

## 2.2 核心类层次

```text
Model (sat/model.h)                          // 依赖注入容器，万物经 GetOrCreate<T>() 获取
│
├── SatSolver (sat_solver.h)                 // CDCL：决策/单元传播/冲突学习/重启
│   ├── Trail / VariablesAssignment          // 布尔赋值 trail（含每个赋值的 reason）
│   ├── LiteralWatchers / ClauseManager      // 二观察字面量子句传播
│   └── BinaryImplicationGraph               // 二元蕴含图（at-most-one 在此高效表示）
│
├── IntegerTrail (integer.h)                 // 整数变量的界 trail（与布尔 trail 同步回溯）
├── IntegerEncoder                           // 字面量 ⇔ 整数视图（lit ⇒ var∈{0,1}）互译
├── GenericLiteralWatcher                    // 传播器总线：字面量/整数界事件 → 传播器ID队列
│
├── PropagatorInterface 实现者（TSP 相关）
│   ├── CircuitPropagator (circuit.h:46)     // ★ 子环消除/强制自环传播，含 ReversibleInterface
│   ├── AllDifferentConstraint (all_different.h:72)  // 可选叠加
│   └── LinearProgrammingConstraint (linear_programming_constraint.h:137)
│        ├── 内嵌 glop::RevisedSimplex
│        ├── LinearConstraintManager         // LP 行池 + 割管理
│        └── CutGenerator 列表               // ← CreateStronglyConnectedGraphCutGenerator
│
├── CpModelMapping (cp_model_mapping.h)      // proto 变量号 → 内部 Literal/IntegerVariable
├── SharedResponseManager                    // worker 间共享最好解/界/统计
├── SubSolver 框架 (subsolver.h)             // 并行 worker（全问题求解器 + LNS 生成器）
│   └── NeighborhoodGenerator (cp_model_lns.h)
│       ├── RoutingRandomNeighborhoodGenerator (:815)
│       ├── RoutingPathNeighborhoodGenerator   (:827)
│       └── RoutingFullPathNeighborhoodGenerator (:844)
│
└── 用户层：CpModelBuilder / CircuitConstraint (cp_model.h:580)
            └─（或）RoutingModel + routing_sat.cc 桥
```

与传统 CP 的对位关系：`Demon`→`GenericLiteralWatcher` 的 watch 机制；`Queue` 三级优先级→watcher 的传播器优先级序列；trail 回溯→`SetLevel()`/`ReversibleInterface`；`NoCycle` 约束→`CircuitPropagator`；`OptimizeVar` 分支限界→objective var 界收紧 + LP 下界 + core/lb_tree_search 等专用 worker。

# 3. 算法流程图

```text
 ┌────────────────────────────────────────────────────────────────────┐
 │ 用户建模（二选一）                                                    │
 │ A. CpModelBuilder:                                                  │
 │    lit[i][j] = NewBoolVar()  ∀候选弧                                 │
 │    circuit = AddCircuitConstraint(); circuit.AddArc(i, j, lit)      │
 │    Minimize(Σ d(i,j)·lit[i][j]);  Solve(model)                      │
 │ B. RoutingModel(use_cp_sat=true，或小实例传统路径无解时兜底)          │
 │    → SolveModelWithSat (routing_sat.cc:1173)                        │
 │    → PopulateSingleRouteModelFromRoutingModel (:384):               │
 │      遍历 NextVar(i) 当前域逐弧建 0/1 变量, End 索引重映射回 Start,    │
 │      目标系数=GetHomogeneousCost, kint64max 弧剔除,                  │
 │      CP 现有解→solution_hint (:1100)                                │
 └──────────────────────────────┬─────────────────────────────────────┘
                                ▼  CpModelProto{variables, circuit, objective}
 ┌────────────────────────────────────────────────────────────────────┐
 │ ① Presolve (cp_model_presolve.cc)                                   │
 │   通用: 域收缩/等价字面量合并/probing/对偶界强化/对称检测              │
 │   PresolveCircuit (:7551):                                          │
 │   · ReindexArcs 致密重编号                                           │
 │   · 入度/出度唯一候选弧 → 字面量固定为真; 与真弧冲突的弧 → 固定为假,    │
 │     迭代到不动点 (:7592-7635)                                        │
 │   · 删除假弧; 任一节点出/入度归零 → UNSAT (:7637-7675)                │
 │   · 真弧已成完整环 → 其余节点强制自环, 约束整体删除 (:7677-7707)       │
 │   · 度=2 的节点 → 两弧字面量互为否定 (:7716-)                         │
 └──────────────────────────────┬─────────────────────────────────────┘
                                ▼  简化后的 CpModelProto
 ┌────────────────────────────────────────────────────────────────────┐
 │ ② 加载 LoadCircuitConstraint (cp_model_loader.cc:1664)              │
 │    → LoadSubcircuitConstraint (circuit.cc:720):                     │
 │    · 每节点出弧集: AddAtMostOne(二元蕴含图) + 子句(≥1) = exactly-one  │
 │    · 每节点入弧集: 同上                                              │
 │    · new CircuitPropagator(...) 注册到 GenericLiteralWatcher        │
 │    · (可选 use_all_different_for_circuit) AllDifferentConstraint    │
 │   目标: Σcost·lit → 整数目标变量 + IntegerEncoder 字面量整数视图      │
 │   LP 构建 (linearization_level≥1, linear_relaxation.cc:1493):       │
 │    · AppendCircuitRelaxation: 出/入 exactly-one 线性行 (:491)        │
 │    · level≥2: AddCircuitCutGenerator → SCC 子环割生成器 (:579)       │
 └──────────────────────────────┬─────────────────────────────────────┘
                                ▼
 ┌────────────────────────────────────────────────────────────────────┐
 │ ③ 并行 worker 组合搜索 (cp_model_solver.cc, cp_model_search.cc:844)  │
 │                                                                     │
 │  全问题 worker（共享最好解/界）:                                       │
 │   default_lp / no_lp / max_lp(线性化2级,含子环割) / quick_restart /   │
 │   core / reduced_costs / pseudo_costs / probing /                   │
 │   lb_tree_search / objective_lb_search / feasibility_jump ...       │
 │                                                                     │
 │  每个 worker 内部主循环（CDCL × CP × LP）:                            │
 │  ┌───────────────────────────────────────────────────────────┐     │
 │  │ 决策: 选未定弧字面量(VSIDS/phase saving/LP值导向极性)        │     │
 │  │ → 单元传播(子句/AMO) ⇄ CircuitPropagator(链维护/堵环)        │     │
 │  │   ⇄ LP传播(单纯形求下界, 约化成本固定变量, 违反割→加割)        │     │
 │  │ → 冲突: 分析→学习子句→非时序回跳→VSIDS加权                    │     │
 │  │ → 找到可行环: 更新上界, objective_var≤best-1 继续             │     │
 │  └───────────────────────────────────────────────────────────┘     │
 │                                                                     │
 │  LNS worker（有首个可行解后激活, cp_model_solver.cc:2027-2045）:      │
 │   单个 circuit: routing_random_lns / routing_path_lns               │
 │   + rins/rens 等通用 LNS: 固定部分弧, 小模型重解                      │
 │                                                                     │
 │  终止: 下界==上界(OPTIMAL) / 超时(FEASIBLE) / 证明无解(INFEASIBLE)    │
 └──────────────────────────────┬─────────────────────────────────────┘
                                ▼  CpSolverResponse{status, solution, bounds}
 ┌────────────────────────────────────────────────────────────────────┐
 │ ④ 解提取                                                            │
 │  A. 直接建模: SolutionBooleanValue(resp, lit[i][j]) 重建回路          │
 │  B. 桥接: ConvertToSolution (routing_sat.cc:455): 真弧→NextVar 赋值, │
 │     OPTIMAL 时目标值同时作为 Routing 侧下界 (:428-452),               │
 │     再经 CP 侧 CheckIfAssignmentIsFeasible 验证入库                  │
 └────────────────────────────────────────────────────────────────────┘
```

# 4. 数据流程图

```text
用户世界                 proto 层                     内核层
────────               ─────────                    ────────
n 城市,
d(i,j) 距离  ──►  每条候选弧一个 BoolVar
                  (变量号 v ∈ CpModelProto.variables,
                   域 [0,1])
                  CircuitConstraintProto:
                    tails=[...], heads=[...],
                    literals=[...]
                  CpObjectiveProto:
                    vars=[lit...], coeffs=[d(i,j)...]
                        │
                        │ Presolve: 弧固定/删除(改写proto),
                        │ 目标常量偏移进 offset
                        ▼
                  简化 CpModelProto
                        │ CpModelMapping::Literals()
                        ▼
                                            sat::Literal per 弧
                                            (cp_model_loader.cc:1673)
                                            ├── 二元蕴含图: AMO(出弧), AMO(入弧)
                                            ├── 子句: (≥1 出弧), (≥1 入弧)
                                            ├── CircuitPropagator: graph_ 哈希
                                            │   {(tail,head)→Literal}, self_arcs_[]
                                            │   (circuit.cc 构造函数)
                                            ├── IntegerEncoder: lit 的 0/1 整数视图
                                            │   (routing_cuts.cc:3175-3189 取视图)
                                            └── LP 列: 每弧一列, 行=exactly-one,
                                                目标行 = Σ d·x
                        │
                        │ 搜索期间的数据变化:
                        │ · 布尔 trail: 弧字面量 真/假 + reason
                        │ · CircuitPropagator.next_/prev_: 部分链
                        │ · IntegerTrail: objective_var 界
                        │ · LP: x*∈[0,1]^m 分数解 → 割池新行（m 为候选弧数）
                        │ · 学习子句库: 冲突的泛化
                        ▼
                  CpSolverResponse.solution
                  (每个变量一个 int64)
                        │
用户世界      ◄─────────┘
回路: 从节点0沿 value(lit)=1 的弧重建
目标值: response.objective_value
状态: OPTIMAL / FEASIBLE / INFEASIBLE
```

桥接入口（Routing→CP-SAT）多一道前置变形，证据在 routing_sat.cc:384-415：
- **End 索引折叠**：Routing 的 start/end 双索引折回单一 depot 节点：`if (model.IsStart(head)) continue; if (model.IsEnd(head)) head = model.Start(0);`（L398-399）——CP-SAT 的 circuit 是真正的"环"，不需要开链技巧；
- **域驱动建弧**：只为 `NextVar(tail)` 当前域中的 head 建弧（L390-393），CP 侧已删除的弧不会进入 SAT 模型；
- **不可行弧剔除**：`cost == kint64max` 的弧直接跳过（L402）；
- **可选节点编码**：自环弧的目标系数 = `UnperformedPenalty`（L400-401；纯 TSP 无可选节点，不生成自环）；
- **热启动**：CP 侧已有解写入 `solution_hint`（L1100-1123）。

---

# 5. 主要函数

## 5.1 用户层：`CpModelBuilder::AddCircuitConstraint` / `CircuitConstraint::AddArc`

cp_model.h:575-595：

```cpp
/**
 * Specialized circuit constraint.
 * This constraint allows adding arcs to the circuit constraint incrementally.
 */
class CircuitConstraint : public Constraint {
 public:
  /**
   * Add an arc to the circuit.
   * @param tail the index of the tail node.
   * @param head the index of the head node.
   * @param literal it will be set to true if the arc is selected.
   */
  void AddArc(int tail, int head, BoolVar literal);
  ...
};
```

proto 语义（cp_model.proto:193-200 注释原文）：*"All the other nodes must have exactly one incoming and one outgoing selected arc... All the selected arcs that are not self-loops must form a single circuit."* —— 即：自环=节点不参与；非自环被选弧必须构成**唯一**回路。一个典型 n 城 TSP 直接建模即 n(n−1) 个 BoolVar + 一个 circuit 约束 + 线性目标。

## 5.2 桥接层：`PopulateSingleRouteModelFromRoutingModel`

routing_sat.cc:384-415（单车 TSP 专用分支，`PopulateModelFromRoutingModel` L420-426 按 `vehicles()==1` 分派）：

```cpp
ArcVarMap PopulateSingleRouteModelFromRoutingModel(const RoutingModel& model,
                                                   CpModelProto* cp_model) {
  ArcVarMap arc_vars;
  const int num_nodes = model.Nexts().size();
  CircuitConstraintProto* circuit =
      cp_model->add_constraints()->mutable_circuit();
  for (int tail = 0; tail < num_nodes; ++tail) {
    std::unique_ptr<IntVarIterator> iter(
        model.NextVar(tail)->MakeDomainIterator(false));
    for (int head : InitAndGetValues(iter.get())) {
      if (model.IsStart(head)) continue;
      if (model.IsEnd(head)) head = model.Start(0);
      const int64_t cost = tail != head ? model.GetHomogeneousCost(tail, head)
                                        : model.UnperformedPenalty(tail);
      if (cost == std::numeric_limits<int64_t>::max()) continue;
      const int index = AddVariable(cp_model, 0, 1);
      circuit->add_literals(index);
      circuit->add_tails(tail);
      circuit->add_heads(head);
      cp_model->mutable_objective()->add_vars(index);
      cp_model->mutable_objective()->add_coeffs(cost);
      gtl::InsertOrDie(&arc_vars, {tail, head}, index);
    }
  }
  ...
  return arc_vars;
}
```

另注意桥接的能力边界：`RoutingModelCanBeSolvedBySat` 要求 `GetVehicleClassesCount()==1`（routing_sat.cc:67-69），即该快速通道只支持同构车队（TSP 天然满足）；更广的模型走 `PopulateGeneralizedRouteModelFromRoutingModel`（用 `RoutesConstraintProto`）。

## 5.3 Presolve：`CpModelPresolver::PresolveCircuit`

cp_model_presolve.cc:7551 起，对 TSP 实际生效的规则（均有 `UpdateRuleStats` 计数）：

1. **唯一候选弧固定**：某节点只剩 1 条候选出（入）弧 ⇒ 该字面量固定为真（L7594-7604，规则名 "circuit: fixed singleton arcs"）；
2. **真弧排他**：节点已有真出（入）弧 ⇒ 同节点其它出（入）弧固定为假（L7606-7631，"circuit: set literal to false"），1+2 迭代至不动点；
3. **假弧删除与致命度检查**：移除假弧后任何节点出/入度为 0 ⇒ 模型 UNSAT（L7637-7675）；
4. **整环识别**：真弧已连成回路 ⇒ 环外节点自环固定为真、其余弧为假，整条约束删除（L7677-7707，"circuit: fully specified"）；
5. **度=2 推理**：某节点恰有两条候选出（入）弧 ⇒ 两字面量互为否定，可合并变量（L7716 起）。

此外通用 presolve（probing、等价字面量、对称性检测 `symmetry_level`、目标域收缩等）一并作用于弧变量。

## 5.4 加载：`LoadSubcircuitConstraint`

circuit.cc:720-777（被 cp_model_loader.cc:1664 调用）：

```cpp
void LoadSubcircuitConstraint(int num_nodes, ..., bool multiple_subcircuit_through_zero) {
  ...
  // 每节点收集出/入弧字面量
  for (int arc = 0; arc < num_arcs; arc++) {
    exactly_one_outgoing[tails[arc]].push_back(literals[arc]);
    exactly_one_incoming[heads[arc]].push_back(literals[arc]);
  }
  // exactly-one = AtMostOne(进二元蕴含图) + 子句(至少一)
  for (...) { AddAtMostOne(...); model->Add(EnforcedClause(..., exactly_one_incoming[i])); }
  for (...) { AddAtMostOne(...); model->Add(EnforcedClause(..., exactly_one_outgoing[i])); }
  // 全局子环传播器
  model->TakeOwnership(new CircuitPropagator(num_nodes, tails, heads, ..., options, model));
  // 可选：AllDifferent 叠加（默认关）
  if (params->use_all_different_for_circuit() && ...) {
    AllDifferentConstraint* constraint = new AllDifferentConstraint(...);
    ...
  }
}
```

circuit.h:43-45 的注释强调了一个正确性前提：**CircuitPropagator 依赖 exactly-one 约束先行传播**（"for correctness, this constraint requires that 'exactly one' constraints have been added for all the incoming (resp. outgoing) arcs"）——度约束交给高效的子句/AMO 机制，传播器专注子环。

## 5.5 核心传播器：`CircuitPropagator`

circuit.cc:145-411。状态：`next_[n]/prev_[n]`（当前已固定的部分链）、`next_literal_[n]`（链上弧的字面量，作解释用）、`must_be_in_cycle_`（自环字面量已为假的节点）、回溯支持 `level_ends_+added_arcs_`（`SetLevel`，L145-163，实现 `ReversibleInterface`，**不用 trail 而是按层重放**）。

**watch 注册的一个细节**（构造函数 L98-99，注释原文 *"Tricky: For self-arc, we watch instead when the arc become false"*）：普通弧 watch 其字面量**变真**；自环弧反向 watch 其否定字面量——即**自环变假**时被唤醒，因为"节点不许留在环外"才是需要传播的事件。完全没有自环弧的节点（纯 TSP 的全部节点）在构造期就被直接放进 `must_be_in_cycle_`（L113-122，`self_arcs_[node]==kFalseLiteralIndex` 分支）。

**`IncrementalPropagate(watch_indices)`**（L204-249）——被 watch 的字面量变真时：
- 自环分支（`arc.tail==arc.head`，对应"自环字面量变假"事件）⇒ 节点加入 `must_be_in_cycle_`（L216-220）；
- 非自环弧变真 ⇒ 度冲突检测：`next_[tail]` 已有值 ⇒ 冲突，reason=两条弧字面量（L222-241）；否则 `AddArc` 并入链（L243-245）。

**`Propagate()`**（L253-411）——对每个节点找其所在链 `start_node→…→end_node`：
- **缺强制节点的环=冲突**：链闭成环但漏掉了 `must_be_in_cycle_` 中的节点 ⇒ 以整条链字面量 + 该节点自环假字面量为 reason 报冲突（L336-344）；
- **堵环传播（子环消除的核心）**：链未闭合且"闭环会漏掉强制节点" ⇒ 把闭环弧 `(end_node→start_node)` 的字面量**传播为假**，reason=链上所有弧（L346-364：`trail_.EnqueueWithStoredReason(kNoClauseId, literal.Negated())`）。这等价于传统 CP 中 NoCycle 的 `RemoveValue`，但**带可解释的 reason，能参与冲突学习**；
- **成环收尾**：链闭成合法环 ⇒ 环外所有节点的自环字面量传播为真（节点必须不参与），且多个传播共享同一 reason 以省内存（L367-408，`EnqueueWithSameReasonAs`）。

纯 TSP（无自环弧）下：所有节点天然 `must_be_in_cycle_`（`self_arcs_[i]=kFalseLiteralIndex`），任何不含全部节点的闭环企图都会被"堵环传播"提前阻断——**这正是惰性子环消除**，与 LP 割形成互补。

## 5.6 LP 与割：`AppendCircuitRelaxation` + `SeparateSubtourInequalities`

**线性松弛**（linear_relaxation.cc:491-527）：每节点出弧集合与入弧集合各产生 `at_most_one` + `Σlit ≥ 1` 两类行——即指派问题（AP）松弛。`linearization_level>1` 时再挂割生成器（L1497-1499）。

**割分离**（routing_cuts.cc:3097-3170，注释直引 Applegate-Bixby-Chvátal-Cook 教科书 §6）每轮 LP 解后执行：

1. `InitializeForNewLpSolution`（L2173 起）：取每条弧的 LP 值（经字面量整数视图，L2186-2191），按 LP 值降序排序；
2. **启发式分离**：`GenerateInterestingSubsets`（L2864-2922）按 LP 值降序做并查集合并，**合并树上每个中间连通块都是候选子集 S**（书中算法 6.3）；对每个 S 调 `TrySubsetCut`→`AddOutgoingCut`（L2357-2458）：检查 `Σ_{(i,j): i∈S, j∉S} x_ij ≥ 1` 的违反量，出割/入割择稀疏者加入割池（subtour elimination cut 的有向版）；
3. **精确分离兜底**：启发式没产出时，对"对称化"弧值（`SymmetrizeArcs`：x̄ᵢⱼ=xᵢⱼ+xⱼᵢ）建 **Gomory-Hu 树**（n−1 次最大流得全点对最小割），对树导出的全部子集再试割（L3119-3142，"CircuitExact"）；
4. **Blossom 割**：再无产出时按 Letchford-Reinelt-Theis 2004 精确分离 2-matching/blossom 不等式（L3144-3169，"CircuitBlossom"）。

割并不会无限增生：`TryAllSubsets`（L3040-3089）利用子集树结构，小子集出割后跳过其超集（L3081），注释解释"加太多不相关割会拖垮通用 MIP 割启发式"。

**LP 传播器**（linear_programming_constraint.h:137 头注）则把这些行喂给 GLOP 修订单纯形：传播目标下界、以 **reduced cost strengthening** 固定字面量（约化成本超过当前 gap 时固定弧变量），并以整数算术复核浮点结果保证可靠性。

## 5.7 求解入口与回写：`SolveModelWithSat` / `ConvertToSolution`

- `SolveModelWithSat`（routing_sat.cc:1173-1240）：剩余时限×0.95、目标 scaling/offset 透传、构建 proto、写 hint、`SolveRoutingModel`（L1127-1152：参数合并 + `RegisterExternalBooleanAsLimit` 中断挂接 + 可选 `NewFeasibleSolutionObserver` 中途解回调）→ `SolveCpModel`。
- `ConvertToSolution`（L455-491）：真弧逐条回填 `NextVar`，depot 出弧按 vehicle 顺序分配，开链补 End；`ConvertObjectiveToSolution`（L428-452）在 OPTIMAL 时以整数算术重算目标（避免缩放舍入），FEASIBLE 时回填 `inner_objective_lower_bound` 作为 Routing 侧合法下界——这就是传统报告中 `objective_lower_bound_` 与 `ROUTING_OPTIMAL` 判定的来源。

---

# 6. 变量与约束编码

针对一个 n 城 TSP 实例（直接建模，全连接，无可选城市）：

## 6.1 变量编码

| 变量 | 数量 | 域 | 含义 |
| --- | --- | --- | --- |
| 弧字面量 `lit(i,j)` | n(n−1) 个 BoolVar（proto 变量，域[0,1]） | {0,1} | 弧 (i→j) 是否在回路中（cp_model.proto:201-205） |
| （可选节点才有）自环 `lit(i,i)` | 0 个（纯 TSP） | — | 节点不参与回路；**多重自环被禁止**（proto 注释 L200） |
| 内部目标变量 | 1 个整数视图 | [LB, UB] | CP-SAT 内部用于维护 `Σ d(i,j)·lit(i,j)` 的上下界；不需要用户显式建模 |
| 内部派生 | — | — | 每个在 LP/割中出现的字面量获得 0/1 整数视图（`IntegerEncoder::GetLiteralView`，routing_cuts.cc:3175-3189） |

**没有 next 整数变量、没有 MTZ 序变量、没有 rank 变量**（多车桥接路径里才有 rank/vehicle 辅助变量，routing_sat.cc:210-260）。TSP 的解就是一组真字面量构成的有向环。

## 6.2 约束清单（加载后内核中的实际形态）

| # | 约束 | 内核形态 | 源码 |
| --- | --- | --- | --- |
| K1 | 每节点出度 ≤1 | `AtMostOne(出弧字面量)` 进二元蕴含图（无枚举二次方子句） | circuit.cc:744-753、702-717 |
| K2 | 每节点出度 ≥1 | 子句 `(lit(i,j₁) ∨ … ∨ lit(i,j_k))` | circuit.cc:748 |
| K3 | 每节点入度 ≤1 / ≥1 | 同 K1/K2 对入弧 | circuit.cc:742-750 |
| K4 | **单回路（子环消除）** | `CircuitPropagator`：链维护 + 堵环字面量传播 + 缺节点环冲突（§5.5） | circuit.cc:763-764, 253-411 |
| K5 | （可选，默认关）后继互异 | `AllDifferentConstraint`（位图/匹配传播），参数 `use_all_different_for_circuit`（默认 false） | circuit.cc:770-776；sat_parameters.proto:1108 |
| K6 | 目标 | 线性目标 `min Σ d·lit`；搜索中上界递减约束 + LP 下界传播 | cp_model.proto objective；linear_programming_constraint.* |
| K7 | LP 行（线性化≥1） | 出/入 exactly-one 的 LP 行（at_most_one 行 + ≥1 行分开放） | linear_relaxation.cc:510-526 |
| K8 | 子环割（线性化≥2） | 动态生成 `Σ_{δ⁺(S)} x ≥ 1`（含可选节点扩展式 `≥ 1 − loop_in − loop_out`，AddOutgoingCut L2398-2455）、Gomory-Hu 精确割、Blossom 割 | routing_cuts.cc:2357-2458, 3097-3170 |

**编码如何展开**：对全连接 n 城实例，K1-K3 共 2n 条 AMO + 2n 条子句；K4 是一个全局传播器，内部用 `graph_` 哈希表查找“链尾到链头”的候选闭环弧，但一次完整传播仍需扫描相关路径和 mandatory 节点；K7 在 LP 中形成 2n 组度约束；K8 按需增长。桥接入口按 `NextVar` 当前域建弧，因此传统 CP 侧已有的域裁剪会直接缩小 SAT 模型（routing_sat.cc:390-393）。

## 6.3 与传统 CP 编码的对照

| 维度 | 传统 CP（Routing） | CP-SAT |
| --- | --- | --- |
| 决策变量 | n 个整数 `nexts_[i]∈[0,n]`（successor 模型） | n(n−1) 个布尔弧字面量（边模型） |
| 度约束 | `ValueAllDifferent`（值传播） | 子句 + 二元蕴含图 AMO（单元传播，可学习） |
| 子环消除 | `NoCycle` 约束（RemoveValue，无解释） | `CircuitPropagator`（带 reason 的字面量传播，参与 CDCL 学习）+ LP 子环割 |
| 回路表示 | depot 拆 start/end，开链到 sink | 真正的环，depot 不特殊（桥接时把 End 折回 Start） |
| 成本 | 每节点 Element 通道 + Sum | 平坦线性目标 + LP 下界 + reduced cost 固定 |
| 下界能力 | 几乎没有（轻传播） | AP 松弛 + 子环割 + blossom 割 ⇒ 强 LP 下界，可证最优 |

# 7. Presolve

对 TSP 实例在建模/求解链路上实际发生的预处理（与传统路径"预处理散在建模期"不同，CP-SAT 有正式的 presolve 阶段）：

| 预处理 | 内容 | 源码 |
| --- | --- | --- |
| 桥接期域过滤 | 仅为 `NextVar` 域中的 head 建弧；`kint64max` 弧剔除 | routing_sat.cc:390-402 |
| `ReindexArcs` | 把任意节点编号压缩为致密 `[0,num_nodes)`（presolve 与加载各做一次） | circuit.h:219-249；cp_model_presolve.cc:7558、cp_model_loader.cc:1674 |
| **PresolveCircuit** | 唯一弧固定→真弧排他→不动点迭代；假弧删除；度归零判 UNSAT；整环识别并删约束；度=2 字面量合并（详见 §5.3） | cp_model_presolve.cc:7551-7750 |
| 通用布尔/整数 presolve | probing（试探赋值固定字面量）、等价字面量合并、支配/对偶界强化、重复约束去重 | cp_model_presolve.cc（通用部分） |
| 对称性 | `symmetry_level>0` 时检测模型自动机同构并加对称破缺；对称距离矩阵的 TSP 两方向环是天然对称，可被识别 | cp_model_symmetries.* |
| 目标处理 | 常量并入 offset；系数 GCD 约简；目标变量域由 LP/传播逐步收紧 | cp_model_presolve.cc 通用 |
| 热启动 hint | 桥接时 CP 解→`solution_hint`，被 hint 修复 worker（`fix_variables_to_their_hinted_value` / hint-based completion）利用 | routing_sat.cc:1100-1123 |

# 8. Callback 与外部控制

CP-SAT 面向建模用户的回调面比传统 CP 窄。TSP 链路涉及：

- **可行解观察者**：`NewFeasibleSolutionObserver(observer)`——每个改进解回调；桥接路径用它实现"中途解回写 Routing 并触发 Routing 侧 at-solution 监视器"（routing_sat.cc:1146-1148、1221-1233，需开 `report_intermediate_cp_sat_solutions`）。
- **外部中断**：`TimeLimit::RegisterExternalBooleanAsLimit(&atomic_bool)`（routing_sat.cc:1144-1145）；Routing 的 `interrupt_cp_sat_` 原子量即挂在此处。
- **时限**：`max_time_in_seconds` 参数（桥接时取剩余时间，L1133-1141）。
- **hint**：桥接层把已有 Routing 解写入 `solution_hint`；hint 只引导搜索，不改变可行域，也不等于固定变量。
- `GenericLiteralWatcher` 和 `CircuitPropagator::RegisterWith()` 是求解器内部传播器接口，不是用户 callback。
- 对比传统路径：CP-SAT 没有向普通建模用户开放 demon/SearchMonitor 级别的细粒度钩子；主要控制点是观察者、中断、hint 和参数。

# 9. 搜索流程

CP-SAT 对 TSP 的搜索是**组合主义**的（cp_model_search.cc、cp_model_solver.cc）：

**(a) 单 worker 内核循环**：CDCL 风格——决策（选一个未赋值字面量/整数分支）→ 传播至不动点（子句单元传播 → AMO/蕴含图 → CircuitPropagator → LP 传播器，按注册优先级）→ 冲突则分析学习（1-UIP 子句、非时序回跳、VSIDS 活动度上调）→ 重启策略周期性重置。决策极性默认 `POLARITY_FALSE`+**phase saving**（sat_parameters.proto:57、70），LP 相关 worker 还会用 LP 分数解引导极性（`exploit_lp_solution` 系列参数）。对优化目标：每得可行环即收紧目标上界继续（行为上等价分支限界），下界由 LP/传播抬升，两界相遇即 OPTIMAL。

**(b) worker 组合**（`GetNamedParameters`，cp_model_search.cc:505 起）：`no_lp`（线性化0）、`default_lp`（线性化1）、`max_lp`（线性化2且 `add_lp_constraints_lazily=false`）、`core`、`lb_tree_search`、`objective_lb_search` 等。**子环割的真实开关是该 worker 的 `linearization_level > 1`，不是 worker 名称本身**；例如 `max_lp`、`core_max_lp`、`lb_tree_search`，以及桥接模式的默认单 worker 都可以启用。并行模式下，各 worker 经 `SharedResponseManager` 共享最好解与界。

**(c) LNS worker**（拿到首个可行解后并行开动，cp_model_solver.cc:2027-2045）：

- `routing_random_lns`：随机松弛回路中一批弧；
- `routing_path_lns`：松弛**连续弧段**（类似传统路径的 Or-opt 段重排，但重排交给子求解器精确做）；
- `routing_full_path_lns` 只在模型含 `routes`，或含多个 `circuit` 时注册。**单个 TSP `circuit` 不注册它**。

此外还有通用 LNS。LNS 子问题固定部分弧变量，再用 CP-SAT 重解缩小后的模型。

**(d) 桥接模式的特殊配置**：Routing 侧默认 `sat_parameters.linearization_level=2`、`num_workers=1`（routing_parameters.cc:184-185）——单线程但开满子环割，符合"兜底精确求解小 TSP"的用途。

# 10. 传播流程

**(a) 传播总线**：`GenericLiteralWatcher`（integer.h）维护 字面量→传播器 的 watch 列表与传播器队列。字面量被赋真（trail 入栈）后，唤醒注册过该字面量的传播器：先跑廉价的（子句、二元蕴含、AMO），`CircuitPropagator` 与 LP 等重型传播器排后（注册时声明优先级；circuit.h:45 注释要求 exactly-one 先行）。所有传播必须给出 **reason**（字面量集合），供冲突分析回溯使用。

**(b) 回溯模型**：布尔 trail + `IntegerTrail` 同步分层回退；`CircuitPropagator` 实现 `ReversibleInterface::SetLevel`（circuit.cc:145-163）：记录每层加入的弧（`level_ends_`/`added_arcs_`），回跳时**逆向撤销 next_/prev_**，比通用 trail 更紧凑。

**(c) circuit 的具体传播强度**（对 TSP）：
- **度推理**：完全由子句/AMO 完成——节点的 k 条出弧中 k−1 条为假 ⇒ 最后一条单元传播为真；任一为真 ⇒ 其余经蕴含图全部为假。O(观察字面量) 均摊；
- **链推理**（CircuitPropagator）：弧变真并链，链尾到链头的"闭环弧"若会形成不完整环则**主动传播其字面量为假**（带整链 reason，circuit.cc:346-364）；环一旦合法闭合，环外节点自环传播为真（L367-408）。每次传播的 reason 都是"链上弧字面量集合"（`FillReasonForPath`，L165-192），**冲突分析能把"这几条弧不能共存"学成子句**，这是传统 NoCycle 完全不具备的能力；
- **LP 传播**：单纯形最优值 → 目标下界提升（`IntegerTrail` 上界/下界 enqueue）；reduced cost strengthening → 直接固定弧字面量；割生成器在 LP 分数解上分离子环/blossom 割，加密松弛（§5.6）。LP 的传播同样带 reason（对偶证书的整数化），可参与学习；
- **可选 AllDifferent**（默认关）：把"后继函数是排列"作为匹配问题再传播一层，对某些 circuit 实例有益（sat_parameters.proto:1108-1110）。

**(d) 三层机制的互补**：单元传播负责**局部度逻辑**；`CircuitPropagator` 负责**整数可行性上的子环**；LP+割负责**分数解上的连通性与目标下界**。前两层保证组合模型正确并提供学习素材；第三层通常决定最优性证明的下界质量。

# 11. 实验设计

本报告不填写未经实测的性能结论。建议与传统 CP 报告使用同一批实例和同一机器，至少记录：

| 组别 | 实例 | 目的 |
| --- | --- | --- |
| 随机欧氏 TSP | `n=20, 40, 60, 80, 100`，每个规模多个 seed | 观察变量平方增长、首解和证明时间 |
| TSPLIB 中小实例 | 已知最优值、100 城以内 | 比较最优性证明和上下界收敛 |
| TSPLIB 较大实例 | 100 城以上 | 确认 CP-SAT 的规模边界及传统 Routing 的优势 |

每次实验至少报告：

- `status`、目标值、`best_objective_bound`、相对 gap；
- wall time、冲突数、分支数；
- 布尔变量数、LP iterations、生成的 routing cuts；
- `num_workers`、`linearization_level`、随机 seed；
- 首个可行解时间与最终证明时间。

建议比较四组配置：

| 配置 | 关键参数 | 要回答的问题 |
| --- | --- | --- |
| SAT only | `num_workers=1, linearization_level=0` | 只靠子句和 circuit 传播能走多远 |
| 默认 LP | `num_workers=1, linearization_level=1` | 指派松弛的增益 |
| 强 LP | `num_workers=1, linearization_level=2` | 子环割对下界和证明的增益 |
| Portfolio | `num_workers>1` | 多 worker 与 LNS 对首解、最终 gap 的增益 |

直接 CP-SAT 与 Routing→CP-SAT 桥接必须分开统计。桥接默认 `num_workers=1`、`linearization_level=2`，且可能带传统 CP hint；它不等同于直接使用默认 `CpSolver` 参数。

# 12. 适用边界与结论

1. **编码**：边模型——每条候选弧一个布尔字面量；circuit 全局约束 = "2n 个 exactly-one（子句+AMO）+ 一个带解释的子环传播器"；目标是平坦线性式。没有 next/序数/流变量。
2. **求解**：presolve → CDCL × `CircuitPropagator` × LP（指派松弛、子环割、Gomory-Hu、Blossom、reduced-cost fixing）→ 可选多 worker 与 LNS。上下界相遇时证明 OPTIMAL。
3. **两条路径的分工**（这也是 OR-Tools 自己的工程选择，见 routing.cc:3449-3471 的兜底逻辑与 routing_parameters.cc:187 的默认阈值）：大规模 TSP/VRP 用传统路径拿高质量启发式解；小规模（或需要最优性证明）用 CP-SAT；Routing 库把两者缝合——CP 解作为 hint 热启动 CP-SAT，CP-SAT 的 OPTIMAL/下界回填 Routing 的 `objective_lower_bound_` 用于断言 `ROUTING_OPTIMAL`。
4. 工程上最值得借鉴的三个 trick：**exactly-one 与子环传播分层**（廉价机制处理度逻辑，全局传播器只管环）、**带 reason 的全局约束传播**（让结构传播参与子句学习）、**割平面按"启发式→精确(Gomory-Hu)→Blossom"逐级兜底且利用子集树去冗**（routing_cuts.cc:3119-3169）。
