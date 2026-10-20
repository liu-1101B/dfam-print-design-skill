# DfAM 建模技能（给 AI 用的 3D 打印设计规范）

一个给 AI 编程助手加载的技能包：在**画第一根线之前**就把打印姿态、各向异性、层纹方向、自支撑角、线宽量化、配合公差定下来，而不是事后靠切片参数补救。

## 核心主张

> 打印件的强度、外观、支撑量，90% 在 CAD 里就已经定了；切片参数只能做 ±10% 的补偿。

四条不可谈判的物理事实，各自对应一条建模法则：

| 事实 | 法则 |
|---|---|
| 层间是弱面（各向异性） | 主应力落在 XY 面内并沿走线；Z 向强度按 0.5–0.7 折减 |
| 层厚是量化单位（台阶效应） | Z 向过渡只用 ≥45° 直平面；要光滑的弧面绕 Z 轴放 |
| 悬垂需要支撑（45° 法则） | 悬垂改倒角/斜坡；水平孔改泪滴或菱形 |
| 挤出是定量体积 | 壁厚/孔径 = N × 挤出线宽，不是 N × 喷嘴直径 |

一个常被说错、这里刻意纠偏的点：**45° 倒角不"消除"层纹**，它把不规则宽台阶换成等宽细台阶。真无层纹只有后处理、熨烫或换工艺。

## 内容

```
SKILL.md                       核心思想、8 步流程、数值表、决策规则、10 条交付前自检
references/process-profiles.md FDM 八种材料 + 树脂/SLS/MJF/SLM 的数值、公差、收缩、后处理
references/geometry-patterns.md 13 个几何配方（45° 过渡族、泪滴孔、桥接、拆件、卡扣、点阵…）
scripts/dfam_preflight.py      无依赖预检：水密/法向一致/体积/悬垂率/首层接触/六姿态排序
```

## 安装

放到全局技能目录（所有工作区生效）：

```bash
git clone https://github.com/liu-1101B/dfam-print-design-skill.git ~/.dsh/skills/dfam-print-design
```

或只对单个项目生效：克隆到 `<项目>/.dsh/skills/dfam-print-design`。

## 用法

- 对话里说"帮我设计一个夹具/外壳/卡扣"时技能会自动触发；
- 也可以直接跑预检：

```bash
python scripts/dfam_preflight.py part.stl            # 人读报告
python scripts/dfam_preflight.py part.stl --json     # 机器读
python scripts/dfam_preflight.py part.stl --angle 50 # 换自支撑角
```

退出码 `0` = PASS/WARN，`1` = FAIL，可以直接当构建闸门用。

## 预检示例

```
DfAM 预检 · part.stl
网格   三角面 4416   顶点 2210   水密 是   法向一致 是   体积 2.13 cm³   表面积 2267 mm²
包围盒 82.0 × 10.0 × 20.0 mm   首层接触 324 mm²（占包围盒 39.5%）
悬垂   需支撑 0.0%（0.0 / 328.7 mm²）   水平天花板 0.0 mm²   自支撑面 5 mm²
姿态   当前已是最省支撑的轴向姿态（0.0%）
结论   PASS
```

依赖：Python 3.8+ 标准库，无需 numpy/trimesh。支持二进制/ASCII STL 与 3MF（含嵌套 component 变换）。

## 适用边界

数值默认 FDM / 0.4 喷嘴 / 0.2 层高 / PETG。换工艺按 `process-profiles.md` 换算；承力、高温、人身安全件不能靠 FDM 保证，技能里明确要求写风险声明。
