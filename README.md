# lathe — 车床 XZ 平面连续刀路碰撞验证

纯 Go 标准库实现。验证刀尖（半径 `ToolRadius` 的闭圆盘）沿轨迹段连续运动时，
与机床包络（轴对齐矩形）和卡爪（闭合简单多边形集合，可凹陷、可重叠、无孔）的碰撞关系。

## 接口

```go
type Point    struct{ X, Z float64 }        // mm，X 为半径值
type Envelope struct{ MinX, MinZ, MaxX, MaxZ float64 }
type Polygon  []Point                        // 闭合简单多边形
type Interval struct{ Start, End float64 }   // 参数 u 的闭区间

type Machine struct {
    Envelope   Envelope
    Jaws       []Polygon
    ToolRadius float64
}

type Line struct{ From, To Point }              // 允许零长度
type Arc  struct{ Center, Start Point; Sweep float64 } // 弧度，±一整圈内，非零

func (m *Machine) CheckSegment(seg Segment) []Interval
```

- `CheckSegment` 返回该段全部碰撞参数区间（u ∈ [0,1]）的并集，重叠或接触的区间
  已合并为最大闭区间；无碰撞返回空。孤立相切返回 `Start == End` 的区间；
  碰撞的零长度段返回 `[{0, 1}]`。
- 碰撞判定：圆盘接触或越过包络边界即碰撞（等价于圆盘中心越出四边各收缩
  `ToolRadius` 的矩形）；圆盘与任一卡爪实体（含内部）的最短距离 ≤ 0.5mm 即碰撞
  （等价于中心到卡爪实体的距离 ≤ `ToolRadius + 0.5`）。

## 方法

把碰撞集合表示为 `{u : collides(p(u))}`。收集所有可能改变碰撞状态的候选参数：
包络边界的穿越点，以及轨迹与每条卡爪边的“胶囊”（边膨胀 `ToolRadius+0.5`）边界的
交点——直线轨迹为二次/线性方程，圆弧轨迹化为 `cos(θ-φ)=v` 求解。候选点把 [0,1]
剖分为开子区间；由于阈值严格大于 0，进入/离开卡爪实体必发生在某胶囊区间内部，
故每个开子区间上碰撞状态恒定，用中点判定，候选点单独判定，最后拼接合并。

## 数值误差

- 判定容差 `tol = 1e-9` mm：距边界/阈值 1e-9 mm 以内按碰撞处理（保守方向）。
- 余弦方程的变量在 |v| ≤ 1+1e-12 时钳位到 ±1，保证相切（重根）情形不丢失，
  孤立接触点得以保留为退化区间。
- 候选参数去重间隔 1e-9；区间合并只依赖共享端点，无额外容差。
- 返回区间端点的误差量级为 1e-9（相对 u），测试以 1e-6 容差比对独立推导值。

## 测试

`go test ./...` 覆盖：圆弧两次进入同一凹卡爪且中间保留安全间隔、整圆与卡爪
危险区孤立相切及间隙放大后无碰撞、零长度段两种结果、直线越过包络边界。
期望区间均由余弦定理/线性插值独立推导，不复用库内实现。
