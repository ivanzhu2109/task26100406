package lathe

import (
	"math"
	"sort"
)

// jawIntervals returns the u-intervals on which the moving disk comes
// within distance d of the filled jaw polygon (including its interior).
func jawIntervals(seg Segment, verts []Point, d float64) []Interval {
	var out []Interval
	out = append(out, insideIntervals(seg, verts)...)
	n := len(verts)
	for i := 0; i < n; i++ {
		out = append(out, capsuleIntervals(seg, verts[i], verts[(i+1)%n], d)...)
	}
	return mergeIntervals(out)
}

// insideIntervals returns the u-intervals on which the path point is
// inside (or on the boundary of) the polygon.
func insideIntervals(seg Segment, verts []Point) []Interval {
	events := []float64{0, 1}
	n := len(verts)
	for i := 0; i < n; i++ {
		a := verts[i]
		b := verts[(i+1)%n]
		switch s := seg.(type) {
		case Line:
			events = append(events, lineEdgeEvents(s, a, b)...)
		case Arc:
			events = append(events, arcEdgeEvents(s, a, b)...)
		}
	}
	sort.Float64s(events)
	uniq := events[:0]
	for _, u := range events {
		if len(uniq) == 0 || u-uniq[len(uniq)-1] > mergeTol {
			uniq = append(uniq, u)
		}
	}
	var out []Interval
	for i := 0; i+1 < len(uniq); i++ {
		u0, u1 := uniq[i], uniq[i+1]
		if u1-u0 < mergeTol {
			continue
		}
		mid := seg.position(0.5 * (u0 + u1))
		if pointInPolygon(mid, verts) {
			out = append(out, Interval{u0, u1})
		}
	}
	return out
}

// lineEdgeEvents returns the u parameters where the line path crosses
// edge a-b.
func lineEdgeEvents(l Line, a, b Point) []float64 {
	v := l.End.Sub(l.Start)
	w := b.Sub(a)
	den := v.Cross(w)
	if math.Abs(den) < eps {
		if math.Abs(v.Cross(a.Sub(l.Start))) > eps*(v.Norm()+1) {
			return nil
		}
		vv := v.Dot(v)
		if vv < eps*eps {
			return nil
		}
		var out []float64
		for _, q := range []Point{a, b} {
			u := q.Sub(l.Start).Dot(v) / vv
			if u >= -eps && u <= 1+eps {
				out = append(out, clamp01(u))
			}
		}
		return out
	}
	rel := a.Sub(l.Start)
	u := rel.Cross(w) / den
	t := rel.Cross(v) / den
	if u >= -eps && u <= 1+eps && t >= -eps && t <= 1+eps {
		return []float64{clamp01(u)}
	}
	return nil
}

// arcEdgeEvents returns the u parameters where the arc path crosses
// edge a-b.
func arcEdgeEvents(arc Arc, a, b Point) []float64 {
	w := b.Sub(a)
	A := w.Dot(w)
	if A < eps*eps {
		return nil
	}
	c := arc.Center
	rad := arc.radius()
	rel := a.Sub(c)
	B := 2 * rel.Dot(w)
	C := rel.Dot(rel) - rad*rad
	disc := B*B - 4*A*C
	if disc < -eps {
		return nil
	}
	if disc < 0 {
		disc = 0
	}
	sq := math.Sqrt(disc)
	t0 := arc.theta0()
	var out []float64
	for _, t := range []float64{(-B - sq) / (2 * A), (-B + sq) / (2 * A)} {
		if t < -eps || t > 1+eps {
			continue
		}
		q := a.Add(w.Mul(t))
		theta := math.Atan2(q.Y-c.Y, q.X-c.X)
		if u, ok := thetaToU(theta, t0, arc.Sweep); ok {
			out = append(out, u)
		}
	}
	return out
}

// thetaToU maps an absolute angle to the arc parameter u in [0, 1].
func thetaToU(theta, theta0, sweep float64) (float64, bool) {
	rel := math.Mod(theta-theta0, 2*math.Pi)
	if sweep > 0 {
		if rel < 0 {
			rel += 2 * math.Pi
		}
	} else {
		if rel > 0 {
			rel -= 2 * math.Pi
		}
	}
	u := rel / sweep
	if u < -eps || u > 1+eps {
		return 0, false
	}
	return clamp01(u), true
}

// capsuleIntervals returns the u-intervals on which the path point is
// within distance d of segment a-b.
func capsuleIntervals(seg Segment, a, b Point, d float64) []Interval {
	var out []Interval
	out = append(out, diskIntervals(seg, a, d)...)
	out = append(out, diskIntervals(seg, b, d)...)
	out = append(out, slabIntervals(seg, a, b, d)...)
	return mergeIntervals(out)
}

// diskIntervals returns the u-intervals on which the path point is
// within distance d of point q.
func diskIntervals(seg Segment, q Point, d float64) []Interval {
	switch s := seg.(type) {
	case Line:
		v := s.End.Sub(s.Start)
		A := v.Dot(v)
		if A < eps*eps {
			return nil
		}
		rel := s.Start.Sub(q)
		B := 2 * rel.Dot(v)
		C := rel.Dot(rel) - d*d
		disc := B*B - 4*A*C
		if disc < -eps {
			return nil
		}
		if disc < 0 {
			disc = 0
		}
		sq := math.Sqrt(disc)
		return clipInterval((-B-sq)/(2*A), (-B+sq)/(2*A))
	case Arc:
		rad := s.radius()
		w := s.Center.Sub(q)
		D := w.Norm()
		if D < eps {
			if rad <= d+eps {
				return []Interval{{0, 1}}
			}
			return nil
		}
		k := (d*d - rad*rad - D*D) / (2 * rad * D)
		psi := math.Atan2(w.Y, w.X)
		return cosBandIntervals(s.theta0(), s.Sweep, psi, -2, k)
	}
	return nil
}

// slabIntervals returns the u-intervals on which the path point is
// within perpendicular distance d of the line through a-b while its
// projection falls between a and b.
func slabIntervals(seg Segment, a, b Point, d float64) []Interval {
	ab := b.Sub(a)
	length := ab.Norm()
	if length < eps {
		return nil
	}
	t := ab.Mul(1 / length)
	n := Point{-t.Y, t.X}
	switch s := seg.(type) {
	case Line:
		v := s.End.Sub(s.Start)
		rel := s.Start.Sub(a)
		i1 := linBand(n.Dot(rel), n.Dot(v), -d, d)
		i2 := linBand(t.Dot(rel), t.Dot(v), 0, length)
		return intersectIntervals(i1, i2)
	case Arc:
		rad := s.radius()
		rel := s.Center.Sub(a)
		psiN := math.Atan2(n.Y, n.X)
		i1 := cosBandIntervals(s.theta0(), s.Sweep, psiN,
			(-d-n.Dot(rel))/rad, (d-n.Dot(rel))/rad)
		psiT := math.Atan2(t.Y, t.X)
		i2 := cosBandIntervals(s.theta0(), s.Sweep, psiT,
			(0-t.Dot(rel))/rad, (length-t.Dot(rel))/rad)
		return intersectIntervals(i1, i2)
	}
	return nil
}

// linBand returns the u-intervals where lo <= v0+u*dv <= hi.
func linBand(v0, dv, lo, hi float64) []Interval {
	if math.Abs(dv) < eps {
		if v0 >= lo-eps && v0 <= hi+eps {
			return []Interval{{0, 1}}
		}
		return nil
	}
	u0 := (lo - v0) / dv
	u1 := (hi - v0) / dv
	if u0 > u1 {
		u0, u1 = u1, u0
	}
	return clipInterval(u0, u1)
}
