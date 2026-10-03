package lathe

import "math"

// Segment is a toolpath segment parameterized by u in [0, 1].
type Segment interface {
	// position returns the disk center at parameter u.
	position(u float64) Point
	// stationary reports whether the segment never moves; if so it
	// returns the fixed position.
	stationary() (Point, bool)
}

// Line is a straight segment from Start to End (zero length allowed).
type Line struct {
	Start, End Point
}

func (l Line) position(u float64) Point {
	return l.Start.Add(l.End.Sub(l.Start).Mul(u))
}

func (l Line) stationary() (Point, bool) {
	if l.Start.Dist(l.End) < eps {
		return l.Start, true
	}
	return Point{}, false
}

// Arc is a circular arc around Center, starting at Start, sweeping
// Sweep radians (positive: CCW, negative: CW, 0 < |Sweep| <= 2π).
type Arc struct {
	Center Point
	Start  Point
	Sweep  float64
}

func (a Arc) radius() float64 { return a.Start.Dist(a.Center) }

func (a Arc) theta0() float64 {
	r := a.Start.Sub(a.Center)
	return math.Atan2(r.Y, r.X)
}

func (a Arc) position(u float64) Point {
	r := a.Start.Sub(a.Center)
	t := math.Atan2(r.Y, r.X) + u*a.Sweep
	rad := r.Norm()
	return a.Center.Add(Point{rad * math.Cos(t), rad * math.Sin(t)})
}

func (a Arc) stationary() (Point, bool) {
	if a.radius() < eps {
		return a.Center, true
	}
	return Point{}, false
}
