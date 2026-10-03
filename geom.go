package lathe

import (
	"math"
	"sort"
)

// eps is the general geometric tolerance (mm or radians, as used).
const eps = 1e-9

// mergeTol is the parameter tolerance below which adjacent intervals
// are considered touching and merged.
const mergeTol = 1e-9

// Point is a 2D point in the XZ plane (mm, X is a radius value).
type Point struct {
	X, Y float64
}

// Add returns p+q.
func (p Point) Add(q Point) Point { return Point{p.X + q.X, p.Y + q.Y} }

// Sub returns p-q.
func (p Point) Sub(q Point) Point { return Point{p.X - q.X, p.Y - q.Y} }

// Mul returns k*p.
func (p Point) Mul(k float64) Point { return Point{p.X * k, p.Y * k} }

// Dot returns the dot product p·q.
func (p Point) Dot(q Point) float64 { return p.X*q.X + p.Y*q.Y }

// Cross returns the z component of the 2D cross product p×q.
func (p Point) Cross(q Point) float64 { return p.X*q.Y - p.Y*q.X }

// Norm returns the Euclidean length of p.
func (p Point) Norm() float64 { return math.Hypot(p.X, p.Y) }

// Dist returns the Euclidean distance between p and q.
func (p Point) Dist(q Point) float64 { return p.Sub(q).Norm() }

// Interval is a closed parameter interval [Start, End] within [0, 1].
// A degenerate interval with Start == End represents an isolated touch.
type Interval struct {
	Start, End float64
}

// mergeIntervals sorts and merges overlapping or touching intervals.
func mergeIntervals(ivs []Interval) []Interval {
	sort.Slice(ivs, func(i, j int) bool {
		if ivs[i].Start != ivs[j].Start {
			return ivs[i].Start < ivs[j].Start
		}
		return ivs[i].End < ivs[j].End
	})
	var out []Interval
	for _, iv := range ivs {
		if iv.End < iv.Start-eps {
			continue
		}
		if iv.End < iv.Start {
			iv.End = iv.Start
		}
		if n := len(out); n > 0 && iv.Start <= out[n-1].End+mergeTol {
			if iv.End > out[n-1].End {
				out[n-1].End = iv.End
			}
			continue
		}
		out = append(out, iv)
	}
	return out
}

// intersectIntervals returns the intersection of two interval lists.
func intersectIntervals(a, b []Interval) []Interval {
	sortIntervals(a)
	sortIntervals(b)
	var out []Interval
	i, j := 0, 0
	for i < len(a) && j < len(b) {
		lo := math.Max(a[i].Start, b[j].Start)
		hi := math.Min(a[i].End, b[j].End)
		if hi >= lo-eps {
			if hi < lo {
				hi = lo
			}
			out = append(out, Interval{lo, hi})
		}
		if a[i].End < b[j].End {
			i++
		} else {
			j++
		}
	}
	return out
}

func sortIntervals(ivs []Interval) {
	sort.Slice(ivs, func(i, j int) bool {
		if ivs[i].Start != ivs[j].Start {
			return ivs[i].Start < ivs[j].Start
		}
		return ivs[i].End < ivs[j].End
	})
}

// clipInterval intersects [u0, u1] with [0, 1].
func clipInterval(u0, u1 float64) []Interval {
	if u1 < -eps || u0 > 1+eps {
		return nil
	}
	u0 = clamp01(u0)
	u1 = clamp01(u1)
	if u1 < u0 {
		u1 = u0
	}
	return []Interval{{u0, u1}}
}

func clamp01(v float64) float64 {
	if v < 0 {
		return 0
	}
	if v > 1 {
		return 1
	}
	return v
}
