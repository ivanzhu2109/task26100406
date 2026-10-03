package lathe

import (
	"math"
	"testing"
)

const tol = 1e-6

func polar(r, deg float64) Point {
	rad := deg * math.Pi / 180
	return Point{r * math.Cos(rad), r * math.Sin(rad)}
}

func assertIntervals(t *testing.T, got, want []Interval) {
	t.Helper()
	if len(got) != len(want) {
		t.Fatalf("interval count: got %v, want %v", got, want)
	}
	for i := range want {
		if math.Abs(got[i].Start-want[i].Start) > tol ||
			math.Abs(got[i].End-want[i].End) > tol {
			t.Fatalf("interval %d: got %v, want %v (all: %v)", i, got[i], want[i], got)
		}
	}
}

// concaveJaw is a simple concave polygon. Its two spike tips V1, V2 sit
// at radius 4.5 from the origin at 120° and 60°; every other feature is
// either strictly closer to the origin (radial spike edges) or far away
// from the test arc, so near the arc the polygon behaves like the two
// tip vertices alone.
func concaveJaw() Polygon {
	v1 := polar(4.5, 120)
	v2 := polar(4.5, 60)
	c1 := v1.Add(polar(3, 290))
	c2 := v2.Add(polar(3, 250))
	return Polygon{Vertices: []Point{
		v1, c1,
		polar(3.5, 60), // B2
		v2, c2,
		{3, -5}, {-3, -5},
		polar(3.5, 120), // B1
	}}
}

// An arc of radius 5 around the origin sweeps from 40° to 140°. The jaw
// danger zone (tool radius 0.5 + clearance 0.5 = 1.0) around each tip
// vertex at radius 4.5 is entered twice with a safe gap in between:
// for a tip V at angle phi, |p(theta)-V|^2 = 4.5^2+5^2-2*4.5*5*cos(theta-phi)
// <= 1  <=>  cos(theta-phi) >= 44.25/45.
func TestArcEntersConcaveJawTwice(t *testing.T) {
	const toolR = 0.5
	m := Machine{
		Envelope: Envelope{Min: Point{-20, -20}, Max: Point{20, 20}},
		Jaws:     []Polygon{concaveJaw()},
	}
	theta0 := 40 * math.Pi / 180
	sweep := 100 * math.Pi / 180
	arc := Arc{
		Center: Point{0, 0},
		Start:  Point{5 * math.Cos(theta0), 5 * math.Sin(theta0)},
		Sweep:  sweep,
	}
	got := CheckSegment(arc, m, toolR)

	beta := math.Acos(44.25 / 45.0)
	u := func(theta float64) float64 { return (theta - theta0) / sweep }
	want := []Interval{
		{u(math.Pi/3 - beta), u(math.Pi/3 + beta)},
		{u(2*math.Pi/3 - beta), u(2*math.Pi/3 + beta)},
	}
	assertIntervals(t, got, want)

	// Both ends are safe and a strictly safe gap separates the two
	// collision intervals.
	if got[0].Start <= 0 || got[1].End >= 1 {
		t.Fatalf("arc ends must be safe: %v", got)
	}
	if gap := got[1].Start - got[0].End; gap <= 0.3 {
		t.Fatalf("expected a wide safe gap, got %v in %v", gap, got)
	}
}

// A full-circle path of radius 4 around the origin is tangent to the
// jaw danger zone (distance 1.0) exactly once: the jaw's farthest
// vertex from the origin is (0,3) at radius 3, all other features are
// strictly closer, so the minimum distance 4-3 = 1 equals the
// clearance threshold only at angle 90°, i.e. u = 0.25.
func TestFullCircleTangentToJaw(t *testing.T) {
	const toolR = 0.5
	jaw := Polygon{Vertices: []Point{
		{0, 3}, {2, 0}, {2, -2}, {-2, -2}, {-2, 0},
	}}
	m := Machine{
		Envelope: Envelope{Min: Point{-10, -10}, Max: Point{10, 10}},
		Jaws:     []Polygon{jaw},
	}
	circle := Arc{Center: Point{0, 0}, Start: Point{4, 0}, Sweep: 2 * math.Pi}

	got := CheckSegment(circle, m, toolR)
	assertIntervals(t, got, []Interval{{0.25, 0.25}})

	// Shrinking the jaw by 0.9 lifts the minimum gap to 4-2.7 = 1.3,
	// above the 1.0 threshold: no collision at all.
	scaled := Polygon{Vertices: make([]Point, len(jaw.Vertices))}
	for i, v := range jaw.Vertices {
		scaled.Vertices[i] = v.Mul(0.9)
	}
	m.Jaws = []Polygon{scaled}
	if got := CheckSegment(circle, m, toolR); len(got) != 0 {
		t.Fatalf("expected no collision, got %v", got)
	}
}

// A straight cut from (5,5) to (15,5) inside envelope [0,10]^2 with
// tool radius 1 stays safe while x < 9, i.e. u < 0.4, and collides
// from the touch point u = 0.4 onward.
func TestLineExitsEnvelope(t *testing.T) {
	m := Machine{Envelope: Envelope{Min: Point{0, 0}, Max: Point{10, 10}}}
	got := CheckSegment(Line{Start: Point{5, 5}, End: Point{15, 5}}, m, 1)
	assertIntervals(t, got, []Interval{{0.4, 1}})
}

// Zero-length segments: safe point -> empty, colliding point -> [0,1].
func TestZeroLengthLine(t *testing.T) {
	jaw := Polygon{Vertices: []Point{{0, 0}, {4, 0}, {4, 4}, {0, 4}}}
	m := Machine{
		Envelope: Envelope{Min: Point{-10, -10}, Max: Point{10, 10}},
		Jaws:     []Polygon{jaw},
	}
	safe := Line{Start: Point{8, 8}, End: Point{8, 8}}
	if got := CheckSegment(safe, m, 0.5); len(got) != 0 {
		t.Fatalf("expected no collision, got %v", got)
	}
	inside := Line{Start: Point{2, 2}, End: Point{2, 2}}
	assertIntervals(t, CheckSegment(inside, m, 0.5), []Interval{{0, 1}})
}
