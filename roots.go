package lathe

import "math"

// collectRoots 返回 [0,1] 内所有可能改变碰撞状态的候选参数（含端点 0、1）。
// 允许包含多余候选：多余的候选只会把子区间切得更细，不影响正确性。
func collectRoots(seg Segment, m *Machine) []float64 {
	roots := []float64{0, 1}
	d := m.clearance()
	switch s := seg.(type) {
	case Line:
		roots = append(roots, lineEnvelopeRoots(s, m)...)
		for _, jaw := range m.Jaws {
			for i := range jaw {
				a, b := jaw[i], jaw[(i+1)%len(jaw)]
				roots = append(roots, lineCapsuleRoots(s, a, b, d)...)
			}
		}
	case Arc:
		roots = append(roots, arcEnvelopeRoots(s, m)...)
		for _, jaw := range m.Jaws {
			for i := range jaw {
				a, b := jaw[i], jaw[(i+1)%len(jaw)]
				roots = append(roots, arcCapsuleRoots(s, a, b, d)...)
			}
		}
	}
	return roots
}

// addIfIn 把落在 [0,1]（带容差）的参数裁剪后加入。
func addIfIn(roots []float64, u float64) []float64 {
	const eps = 1e-12
	if u < -eps || u > 1+eps {
		return roots
	}
	if u < 0 {
		u = 0
	} else if u > 1 {
		u = 1
	}
	return append(roots, u)
}

// lineEnvelopeRoots 直线轨迹穿越收缩后包络边界的参数。
func lineEnvelopeRoots(l Line, m *Machine) []float64 {
	e := m.Envelope
	r := m.ToolRadius
	var roots []float64
	dx, dz := l.To.X-l.From.X, l.To.Z-l.From.Z
	if dx != 0 {
		roots = addIfIn(roots, (e.MinX+r-l.From.X)/dx)
		roots = addIfIn(roots, (e.MaxX-r-l.From.X)/dx)
	}
	if dz != 0 {
		roots = addIfIn(roots, (e.MinZ+r-l.From.Z)/dz)
		roots = addIfIn(roots, (e.MaxZ-r-l.From.Z)/dz)
	}
	return roots
}

// solveQuad 求 a2 u^2 + a1 u + a0 = 0 的实根。
func solveQuad(a2, a1, a0 float64) []float64 {
	const eps = 1e-12
	if math.Abs(a2) < eps {
		if math.Abs(a1) < eps {
			return nil
		}
		return []float64{-a0 / a1}
	}
	disc := a1*a1 - 4*a2*a0
	if disc < 0 {
		if disc > -eps*(a1*a1+1) {
			disc = 0 // 相切
		} else {
			return nil
		}
	}
	sq := math.Sqrt(disc)
	return []float64{(-a1 - sq) / (2 * a2), (-a1 + sq) / (2 * a2)}
}

// lineCapsuleRoots 直线轨迹与边 ab 的“胶囊”（边膨胀 d）边界的交点参数。
// 边界由三部分构成：端点 a、b 处半径 d 的圆，以及两侧距离 d 的平行线。
func lineCapsuleRoots(l Line, a, b Point, d float64) []float64 {
	var roots []float64
	v := l.To.sub(l.From)
	for _, q := range []Point{a, b} {
		w := l.From.sub(q)
		for _, u := range solveQuad(v.dot(v), 2*v.dot(w), w.dot(w)-d*d) {
			roots = addIfIn(roots, u)
		}
	}
	edge := b.sub(a)
	if L := edge.norm(); L > 0 {
		n := Point{-edge.Z / L, edge.X / L} // 单位法向
		c := n.dot(v)
		if c != 0 {
			roots = addIfIn(roots, (d-n.dot(l.From.sub(a)))/c)
			roots = addIfIn(roots, (-d-n.dot(l.From.sub(a)))/c)
		}
	}
	return roots
}
