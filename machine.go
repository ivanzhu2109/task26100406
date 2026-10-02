package lathe

import "sort"

// Envelope 是轴对齐矩形机床包络。
type Envelope struct {
	MinX, MinZ, MaxX, MaxZ float64
}

// Interval 是参数 u 的闭区间 [Start, End]，孤立接触点 Start == End。
type Interval struct {
	Start, End float64
}

// Machine 描述机床几何与刀尖半径。
// 刀尖是半径 ToolRadius 的闭圆盘：接触或越过包络边界即碰撞；
// 圆盘与任一卡爪实体的最短距离不大于 jawClearance 也算碰撞。
type Machine struct {
	Envelope   Envelope
	Jaws       []Polygon
	ToolRadius float64
}

// jawClearance 是圆盘与卡爪实体碰撞的最短距离阈值（mm）。
const jawClearance = 0.5

// tol 是数值容差，边界附近按碰撞处理（保守方向）。
const tol = 1e-9

// clearance 返回刀尖中心到卡爪实体的最小允许距离。
func (m *Machine) clearance() float64 { return m.ToolRadius + jawClearance }

// collides 判断刀尖中心位于 p 时是否碰撞。
func (m *Machine) collides(p Point) bool {
	e := m.Envelope
	r := m.ToolRadius
	if p.X <= e.MinX+r+tol || p.X >= e.MaxX-r-tol ||
		p.Z <= e.MinZ+r+tol || p.Z >= e.MaxZ-r-tol {
		return true
	}
	d := m.clearance()
	for _, jaw := range m.Jaws {
		if pointInPolygon(p, jaw) {
			return true
		}
		for i := range jaw {
			a := jaw[i]
			b := jaw[(i+1)%len(jaw)]
			if distPointSegment(p, a, b) <= d+tol {
				return true
			}
		}
	}
	return false
}

// CheckSegment 返回该段全部碰撞参数区间（已合并为最大闭区间），无碰撞返回空。
//
// 方法：收集所有可能改变碰撞状态的候选参数 u（包络边界穿越、
// 到卡爪边距离等于阈值的穿越），把 [0,1] 剖分为子区间；
// 每个开子区间上碰撞状态恒定，用中点判定；候选点单独判定。
// 碰撞的子区间与点拼成闭区间后合并相邻接触者。
func (m *Machine) CheckSegment(seg Segment) []Interval {
	roots := collectRoots(seg, m)
	sort.Float64s(roots)
	roots = dedupe(roots)

	n := len(roots)
	var pieces []Interval
	for i := 0; i+1 < n; i++ {
		mid := (roots[i] + roots[i+1]) / 2
		if m.collides(seg.position(mid)) {
			pieces = append(pieces, Interval{roots[i], roots[i+1]})
		}
	}
	for _, u := range roots {
		if m.collides(seg.position(u)) {
			pieces = append(pieces, Interval{u, u})
		}
	}
	return mergeIntervals(pieces)
}

// dedupe 去除排序后相距过近的重复参数。
func dedupe(us []float64) []float64 {
	out := us[:0]
	for _, u := range us {
		if len(out) == 0 || u-out[len(out)-1] > 1e-9 {
			out = append(out, u)
		}
	}
	return out
}

// mergeIntervals 合并重叠或接触的闭区间为最大闭区间。
func mergeIntervals(ivs []Interval) []Interval {
	if len(ivs) == 0 {
		return nil
	}
	sort.Slice(ivs, func(i, j int) bool {
		if ivs[i].Start != ivs[j].Start {
			return ivs[i].Start < ivs[j].Start
		}
		return ivs[i].End < ivs[j].End
	})
	out := []Interval{ivs[0]}
	for _, iv := range ivs[1:] {
		last := &out[len(out)-1]
		if iv.Start <= last.End {
			if iv.End > last.End {
				last.End = iv.End
			}
		} else {
			out = append(out, iv)
		}
	}
	return out
}
