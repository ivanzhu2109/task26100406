package lathe

import "math"

// Point 是 XZ 平面上的点，单位 mm，X 为半径值。
type Point struct {
	X, Z float64
}

func (p Point) sub(q Point) Point { return Point{p.X - q.X, p.Z - q.Z} }
func (p Point) add(q Point) Point { return Point{p.X + q.X, p.Z + q.Z} }
func (p Point) scale(s float64) Point {
	return Point{p.X * s, p.Z * s}
}
func (p Point) dot(q Point) float64 { return p.X*q.X + p.Z*q.Z }
func (p Point) norm() float64       { return math.Hypot(p.X, p.Z) }

// Polygon 是闭合简单多边形（卡爪截面），顶点按顺序排列，首尾自动闭合。
type Polygon []Point

// distPointSegment 返回点到闭线段 ab 的最短距离。
func distPointSegment(p, a, b Point) float64 {
	ab := b.sub(a)
	len2 := ab.dot(ab)
	if len2 == 0 {
		return p.sub(a).norm()
	}
	t := p.sub(a).dot(ab) / len2
	if t < 0 {
		t = 0
	} else if t > 1 {
		t = 1
	}
	return p.sub(a.add(ab.scale(t))).norm()
}

// pointOnSegment 判断点是否在闭线段 ab 上（带容差）。
func pointOnSegment(p, a, b Point) bool {
	const eps = 1e-9
	ab := b.sub(a)
	ap := p.sub(a)
	cross := ab.X*ap.Z - ab.Z*ap.X
	if math.Abs(cross) > eps*(ab.norm()+1) {
		return false
	}
	return ap.dot(ab) >= -eps && ap.dot(ab) <= ab.dot(ab)+eps
}

// pointInPolygon 判断点是否在闭合简单多边形内（边界视为内部）。
// 射线法：向右发射水平射线，统计与边的交点数，奇数为内部。
func pointInPolygon(p Point, poly Polygon) bool {
	n := len(poly)
	if n < 3 {
		return false
	}
	inside := false
	j := n - 1
	for i := 0; i < n; i++ {
		a, b := poly[j], poly[i]
		if pointOnSegment(p, a, b) {
			return true
		}
		if (a.Z > p.Z) != (b.Z > p.Z) {
			xInt := a.X + (p.Z-a.Z)*(b.X-a.X)/(b.Z-a.Z)
			if p.X < xInt {
				inside = !inside
			}
		}
		j = i
	}
	return inside
}
