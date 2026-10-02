package lathe

import (
	"math"
	"testing"
)

const testTol = 1e-6

func checkIntervals(t *testing.T, got, want []Interval) {
	t.Helper()
	if len(got) != len(want) {
		t.Fatalf("区间数量不符: got %v, want %v", got, want)
	}
	for i := range want {
		if math.Abs(got[i].Start-want[i].Start) > testTol ||
			math.Abs(got[i].End-want[i].End) > testTol {
			t.Errorf("区间 %d 不符: got [%v, %v], want [%v, %v]",
				i, got[i].Start, got[i].End, want[i].Start, want[i].End)
		}
	}
}

// 场景一：上半圆轨迹两次进入同一凹卡爪（飞镖形）的危险区，中间有安全间隔。
//
// 独立推导：卡爪两个尖顶分别在半径 9、角度 60° 与 120° 处；
// 凹口取 (0,4)，使各边所在直线到轨迹圆的距离远大于阈值（>= 4.5），
// 多边形整体在半径 9 的圆盘内，轨迹不会进入实体，
// 故危险区仅由尖顶处半径 d=r+0.5=1.5 的圆盘与轨迹圆（R=10）的交点决定：
//
//	cos(Δ) = (R² + ρ² - d²) / (2Rρ),  ρ=9, R=10, d=1.5
//
// θ ∈ [π/3-Δ, π/3+Δ] 与 [2π/3-Δ, 2π/3+Δ]，u = θ/π。
func TestArcEntersConcaveJawTwice(t *testing.T) {
	tip := func(theta float64) Point {
		return Point{9 * math.Cos(theta), 9 * math.Sin(theta)}
	}
	jaw := Polygon{
		tip(math.Pi / 3),     // 尖顶 1：(4.5, 7.794...)
		{0, 4},               // 凹口（自反顶点）
		tip(2 * math.Pi / 3), // 尖顶 2
		{0, 3},
	}
	m := &Machine{
		Envelope:   Envelope{-100, -100, 100, 100},
		Jaws:       []Polygon{jaw},
		ToolRadius: 1,
	}
	arc := Arc{Center: Point{0, 0}, Start: Point{10, 0}, Sweep: math.Pi}

	got := m.CheckSegment(arc)

	const R, rho, d = 10.0, 9.0, 1.5
	delta := math.Acos((R*R + rho*rho - d*d) / (2 * R * rho))
	want := []Interval{
		{(math.Pi/3 - delta) / math.Pi, (math.Pi/3 + delta) / math.Pi},
		{(2*math.Pi/3 - delta) / math.Pi, (2*math.Pi/3 + delta) / math.Pi},
	}
	checkIntervals(t, got, want)

	// 安全间隔必须保留：中点 u=0.5（圆弧正上方 (0,10)，距卡爪大于 4.5mm）无碰撞。
	if m.collides(arc.position(0.5)) {
		t.Errorf("u=0.5 应为安全点")
	}
}

// 场景二：整圆轨迹与卡爪危险区仅相切一次；几何外移使间隙大于阈值后无碰撞。
//
// 独立推导：卡爪顶点 (0, 8.5) 到轨迹圆（R=10）的最短距离恰为 1.5 = r+0.5，
// 唯一接触点在圆弧最高点 θ=π/2，即 u = (π/2)/(2π) = 0.25。
// 顶点移到 (0, 8.4) 后最小间隙 1.6 > 1.5，整圈无碰撞。
func TestFullCircleTangentToJaw(t *testing.T) {
	m := &Machine{
		Envelope:   Envelope{-100, -100, 100, 100},
		Jaws:       []Polygon{{{0, 8.5}, {-1, 7}, {1, 7}}},
		ToolRadius: 1,
	}
	circle := Arc{Center: Point{0, 0}, Start: Point{10, 0}, Sweep: 2 * math.Pi}

	got := m.CheckSegment(circle)
	want := []Interval{{0.25, 0.25}} // 孤立相切点
	checkIntervals(t, got, want)

	// 最小间隙 1.6 > 1.5：无碰撞。
	m.Jaws = []Polygon{{{0, 8.4}, {-1, 6.9}, {1, 6.9}}}
	if got := m.CheckSegment(circle); len(got) != 0 {
		t.Errorf("间隙大于阈值应无碰撞, got %v", got)
	}
}

// 零长度直线段：安全返回空，碰撞返回完整 [0,1]。
func TestZeroLengthLine(t *testing.T) {
	m := &Machine{
		Envelope:   Envelope{-100, -100, 100, 100},
		Jaws:       []Polygon{{{0, 8.5}, {-1, 7}, {1, 7}}},
		ToolRadius: 1,
	}
	if got := m.CheckSegment(Line{Point{0, -50}, Point{0, -50}}); len(got) != 0 {
		t.Errorf("安全零长度段应返回空, got %v", got)
	}
	// 点 (0,9) 距顶点 (0,8.5) 为 0.5 <= 1.5，碰撞。
	got := m.CheckSegment(Line{Point{0, 9}, Point{0, 9}})
	checkIntervals(t, got, []Interval{{0, 1}})
}

// 直线轨迹越过收缩后的包络边界：u = 9/20 = 0.45 起碰撞。
func TestLineCrossesEnvelope(t *testing.T) {
	m := &Machine{
		Envelope:   Envelope{-10, -10, 10, 10},
		ToolRadius: 1,
	}
	got := m.CheckSegment(Line{Point{0, 0}, Point{20, 0}})
	checkIntervals(t, got, []Interval{{0.45, 1}})
}
