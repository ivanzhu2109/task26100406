package lathe

// jawClearance is the minimum allowed distance (mm) between the tool
// disk and any jaw solid.
const jawClearance = 0.5

// CheckSegment returns the union of all parameter intervals of seg on
// which the tool disk of radius toolRadius collides with the machine
// envelope or any jaw. Intervals are merged into maximal closed
// intervals; an isolated touch is a degenerate [u, u] interval.
// A safe zero-length segment yields nil, a colliding one yields
// [{0, 1}].
func CheckSegment(seg Segment, m Machine, toolRadius float64) []Interval {
	if p, ok := seg.stationary(); ok {
		if pointCollision(m, p, toolRadius) {
			return []Interval{{0, 1}}
		}
		return nil
	}
	var out []Interval
	out = append(out, envelopeIntervals(seg, m.Envelope, toolRadius)...)
	d := toolRadius + jawClearance
	for _, jaw := range m.Jaws {
		out = append(out, jawIntervals(seg, jaw.Vertices, d)...)
	}
	out = mergeIntervals(out)
	var res []Interval
	for _, iv := range out {
		if iv.End < -eps || iv.Start > 1+eps {
			continue
		}
		iv.Start = clamp01(iv.Start)
		iv.End = clamp01(iv.End)
		if iv.End < iv.Start {
			iv.End = iv.Start
		}
		res = append(res, iv)
	}
	return res
}

// pointCollision reports whether a disk of radius r centered at p
// collides with the machine.
func pointCollision(m Machine, p Point, r float64) bool {
	e := m.Envelope
	if p.X <= e.Min.X+r || p.X >= e.Max.X-r ||
		p.Y <= e.Min.Y+r || p.Y >= e.Max.Y-r {
		return true
	}
	d := r + jawClearance
	for _, jaw := range m.Jaws {
		if distPointPolygon(p, jaw.Vertices) <= d+eps {
			return true
		}
	}
	return false
}
