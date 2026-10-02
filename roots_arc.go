package lathe

import "math"

// arcThetaToU 把圆弧上的角度 theta 换算为参数 u，不在扫角范围内时返回 false。
func arcThetaToU(a Arc, theta float64) (float64, bool) {
	d := a.Start.sub(a.Center)
	theta0 := math.Atan2(d.Z, d.X)
	u := (theta - theta0) / a.Sweep
	const eps = 1e-9
	if u < -eps || u > 1+eps {
		return 0, false
	}
	if u < 0 {
		u = 0
	} else if u > 1 {
		u = 1
	}
	return u, true
}

// solveCos 求 cos(theta - phi) = v 在圆弧扫角范围内的全部参数 u。
// |v| 略超 1 时按相切处理（钳位），明显超出则无解。
func solveCos(a Arc, phi, v float64) []float64 {
	const eps = 1e-12
	if v > 1+eps || v < -1-eps {
		return nil
	}
	if v > 1 {
		v = 1
	} else if v < -1 {
		v = -1
	}
	delta := math.Acos(v)
	const tau = 2 * math.Pi
	var roots []float64
	for _, base := range []float64{phi + delta, phi - delta} {
		// 扫角范围不超过一整圈，k 取 -1..1 足够覆盖。
		for k := -1; k <= 1; k++ {
			if u, ok := arcThetaToU(a, base+float64(k)*tau); ok {
				roots = addIfIn(roots, u)
			}
		}
	}
	return roots
}

// arcEnvelopeRoots 圆弧轨迹穿越收缩后包络边界的参数。
// C + R*(cos,sin) = bound 化为 cos/sin 方程求解。
func arcEnvelopeRoots(a Arc, m *Machine) []float64 {
	e := m.Envelope
	r := m.ToolRadius
	d := a.Start.sub(a.Center)
	R := d.norm()
	if R == 0 {
		return nil
	}
	var roots []float64
	// X 方向：Cx + R cos(theta) = bound
	for _, bound := range []float64{e.MinX + r, e.MaxX - r} {
		roots = append(roots, solveCos(a, 0, (bound-a.Center.X)/R)...)
	}
	// Z 方向：Cz + R sin(theta) = bound，即 cos(theta - pi/2) = (bound-Cz)/R
	for _, bound := range []float64{e.MinZ + r, e.MaxZ - r} {
		roots = append(roots, solveCos(a, math.Pi/2, (bound-a.Center.Z)/R)...)
	}
	return roots
}

// arcCapsuleRoots 圆弧轨迹与边 ab 的“胶囊”（边膨胀 d）边界的交点参数。
// 边界由端点 a、b 处半径 d 的圆，以及两侧距离 d 的平行线构成。
func arcCapsuleRoots(a Arc, p, q Point, d float64) []float64 {
	c := a.Start.sub(a.Center)
	R := c.norm()
	if R == 0 {
		return nil
	}
	var roots []float64
	// 端点圆：|C + R e(theta) - P|^2 = d^2
	// 即 cos(theta - phi) = (R^2 + rho^2 - d^2) / (2 R rho)，rho = |P - C|
	for _, pt := range []Point{p, q} {
		diff := pt.sub(a.Center)
		rho := diff.norm()
		if rho == 0 {
			continue // 端点与圆心重合：距离恒定，无穿越
		}
		v := (R*R + rho*rho - d*d) / (2 * R * rho)
		roots = append(roots, solveCos(a, math.Atan2(diff.Z, diff.X), v)...)
	}
	// 偏移线：n . (C + R e(theta)) = n . P +/- d
	edge := q.sub(p)
	if L := edge.norm(); L > 0 {
		n := Point{-edge.Z / L, edge.X / L}
		phi := math.Atan2(n.Z, n.X)
		base := n.dot(p.sub(a.Center))
		roots = append(roots, solveCos(a, phi, (base+d)/R)...)
		roots = append(roots, solveCos(a, phi, (base-d)/R)...)
	}
	return roots
}
