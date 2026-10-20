# DfAM 建模技能（给 AI 用的 3D 打印设计工作流）

一个给 AI 编程助手加载的技能包：先用经典力学判断零件会怎么坏，再把几何设计到不坏，最后才是打印规则。打印规则是约束，不是设计目标。

## 这一版改了什么

v1 是一套"打印规则集合"——先定姿态、层间各向异性、45° 自支撑、线宽整数倍。规则本身没错，但写成了绝对禁令，会诱导 AI 为了通过打印规则而牺牲结构性能。

v2 重构成"力学优先，制造约束协同"：

| 维度 | v1 | v2 |
|---|---|---|
| 核心命题 | "90% 在 CAD 里决定" | 删掉无依据的百分比，改为五步设计顺序 |
| 力学 | 只有"主应力放 XY" | 失效模式表、初算公式、Kt、屈曲、疲劳、蠕变、安全系数 |
| 层间折减 | 统一 0.5–0.7 | 分材料/条件，标 D3，要求实测替换，禁止当真实强度 |
| 45° 法则 | 默认最优 | 候选方案之一，先比较姿态/几何/支撑/拆件四条路 |
| 壁厚 | 必须线宽整数倍 | 启发式，不得覆盖强度与功能尺寸 |
| 姿态 | 最省支撑 | 硬约束筛选 + 帕累托/显式权重，输出取舍依据 |
| 预检器 | 单一 PASS/FAIL | 三段状态 + 力学恒为 UNKNOWN + 未实现项不默认通过 |
| 评测 | 无 | 8 个案例 + 7 维评分 + 禁止结论清单 |

## 核心主张

> 打印件的性能由几何、材料本构、载荷与边界、打印方向、层间结合、走线、后处理共同决定。**不要在缺少证据时用固定百分比描述各占多少。**
>
> 任何规则都要带适用条件。经验规则只做筛选，不能替代计算与试验。

冲突时按四层裁决，下层不得覆盖上层：

1. **功能与安全**（强度、刚度、温度、寿命、装配）
2. **制造可行**（设备、材料、支撑、最小特征、精度）
3. **多目标优化**（质量、成本、时间、外观）
4. **验证与交付**（预检、计算、试打、风险）

## 内容

```
SKILL.md                        设计原则、冲突裁决、物理约束、力学判据、证据等级、姿态优化
references/mechanics.md         截面特性、梁/轴/压杆/连接/卡扣公式、K 与 Kt、安全系数、蠕变、4 个算例
references/process-profiles.md  材料 × 工艺 × 设备参数，数据来源分级 D0–D3，UNKNOWN 标记，标定方案
references/geometry-patterns.md 18 张设计卡片（机制/适用性/对 σ k m P 的影响/定尺寸/失效/验证）
scripts/dfam_preflight.py       几何预筛：结构化三段状态、帕累托姿态、UNKNOWN 不隐藏
evals/                          8 个评测案例 + 评分协议，防止越改越差
```

## 安装

```bash
git clone https://github.com/liu-1101B/dfam-print-design-skill.git ~/.dsh/skills/dfam-print-design
```

或只对单个项目生效：克隆到 `<项目>/.dsh/skills/dfam-print-design`。

## 用法

- 说"帮我设计一个夹具/外壳/卡扣"时技能自动触发；
- 也可以直接跑几何预筛：

```bash
python scripts/dfam_preflight.py part.stl
python scripts/dfam_preflight.py part.stl --json
python scripts/dfam_preflight.py part.stl --angle 50 --bed 256x256x256
python scripts/dfam_preflight.py part.stl --weights support=.4,height=.2,contact=.2,ceiling=.2
```

退出码 `0` = PASS/WARN，`1` = FAIL，可直接当构建闸门。

## 预检输出长什么样

```
DfAM 几何预筛 · part.stl   [dfam_preflight 2.0]
范围   geometric pre-screening only; not a mechanical validation
网格   三角面 4416   顶点 2210   体积 2.13 cm³   表面积 2267 mm²
包围盒 82.0 × 10.0 × 20.0 mm   首层接触 324 mm²（39.5%）   悬垂需支撑 0.0%
几何   PASS
制造   PASS
力学   UNKNOWN
   ?   S1 材料许用值
   ?   S2 载荷与边界条件
   ...
姿态（四项几何指标；未给硬约束与权重）
   as-is      支撑 0.0%  高 20.0 mm  首层 324 mm²  天花板 0.0 mm²   pareto
   ...
结论   PASS   （力学验证：UNKNOWN）
```

**`PASS` 只代表几何与制造检查通过。** 力学项永远是 `UNKNOWN`，直到你提供载荷、许用值和证据等级。这是设计上的刻意选择，不是偷懒。

## 适用边界

- 默认 FDM / 0.4 喷嘴 / 0.2 层高 / PETG；换工艺查 `process-profiles.md`。
- 承力、高温、人身安全件：技能要求标注证据等级，未达 E3 时必须写"初步设计，待验证"。
- 预检器不做壁厚、自相交、真实支撑体积、时间估算——这些项返回 `UNKNOWN`，不装作通过。

## 依赖

Python 3.8+ 标准库。无需 numpy / trimesh。支持二进制/ASCII STL 与 3MF（含嵌套 component 变换）。
