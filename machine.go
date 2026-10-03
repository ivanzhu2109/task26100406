package lathe

import "math"

// Envelope is the axis-aligned machine envelope (mm). The tool disk
// must stay strictly inside it; touching the boundary is a collision.
type Envelope struct {
	Min, Max Point
}

// Polygon is a closed simple polygon (a jaw cross-section). It may be
// concave and may overlap other polygons; its interior is solid.
type Polygon struct {
	Vertices []Point
}

// Machine groups the envelope and the jaws.
type Machine struct {
	Envelope Envelope
	Jaws     []Polygon
}

// pointInPolygon reports whether p is inside or on the boundary.
func pointInPolygon(p Point, poly []Point) bool {
	inside := false
	n := len(poly)
	for i := 0; i < n; i++ {
		a := poly[i]
		b := poly[(i+1)%n]
		if onSegment(p, a, b) {
			return true
		}
		if (a.Y > p.Y) != (b.Y > p.Y) {
			x := a.X + (p.Y-a.Y)*(b.X-a.X)/(b.Y-a.Y)
			if x > p.X {
				inside = !inside
			}
		}
	}
	return inside
}

// onSegment reports whether p lies on segment a-b (within tolerance).
func onSegment(p, a, b Point) bool {
	ab := b.Sub(a)
	ap := p.Sub(a)
	if math.Abs(ab.Cross(ap)) > eps*(ab.Norm()+1) {
		return false
	}
	d := ap.Dot(ab)
	return d >= -eps && d <= ab.Dot(ab)+eps
}

// distPointSegment returns the distance from p to segment a-b.
func distPointSegment(p, a, b Point) float64 {
	ab := b.Sub(a)
	l2 := ab.Dot(ab)
	if l2 < eps*eps {
		return p.Dist(a)
	}
	t := p.Sub(a).Dot(ab) / l2
	if t < 0 {
		t = 0
	}
	if t > 1 {
		t = 1
	}
	return p.Dist(a.Add(ab.Mul(t)))
}

// distPointPolygon returns the distance from p to the filled polygon
// (0 when p is inside).
func distPointPolygon(p Point, poly []Point) float64 {
	if pointInPolygon(p, poly) {
		return 0
	}
	best := math.Inf(1)
	n := len(poly)
	for i := 0; i < n; i++ {
		d := distPointSegment(p, poly[i], poly[(i+1)%n])
		if d < best {
			best = d
		}
	}
	return best
}
