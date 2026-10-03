package lathe

import (
	"math"
	"sort"
)

// envelopeIntervals returns the u-intervals on which the moving disk
// of radius r touches or crosses the envelope boundary.
func envelopeIntervals(seg Segment, env Envelope, r float64) []Interval {
	loX := env.Min.X + r
	hiX := env.Max.X - r
	loY := env.Min.Y + r
	hiY := env.Max.Y - r
	switch s := seg.(type) {
	case Line:
		var out []Interval
		out = append(out, axisOutside(s.Start.X, s.End.X-s.Start.X, loX, hiX)...)
		out = append(out, axisOutside(s.Start.Y, s.End.Y-s.Start.Y, loY, hiY)...)
		return mergeIntervals(out)
	case Arc:
		rad := s.radius()
		t0 := s.theta0()
		var out []Interval
		out = append(out, cosBandIntervals(t0, s.Sweep, 0, -2, (loX-s.Center.X)/rad)...)
		out = append(out, cosBandIntervals(t0, s.Sweep, 0, (hiX-s.Center.X)/rad, 2)...)
		out = append(out, cosBandIntervals(t0, s.Sweep, math.Pi/2, -2, (loY-s.Center.Y)/rad)...)
		out = append(out, cosBandIntervals(t0, s.Sweep, math.Pi/2, (hiY-s.Center.Y)/rad, 2)...)
		return mergeIntervals(out)
	}
	return nil
}

// axisOutside returns the u-intervals where v0+u*dv <= lo or >= hi.
func axisOutside(v0, dv, lo, hi float64) []Interval {
	if math.Abs(dv) < eps {
		if v0 <= lo || v0 >= hi {
			return []Interval{{0, 1}}
		}
		return nil
	}
	uLo := (lo - v0) / dv
	uHi := (hi - v0) / dv
	var out []Interval
	if dv > 0 {
		out = append(out, clipInterval(math.Inf(-1), uLo)...)
		out = append(out, clipInterval(uHi, math.Inf(1))...)
	} else {
		out = append(out, clipInterval(uLo, math.Inf(1))...)
		out = append(out, clipInterval(math.Inf(-1), uHi)...)
	}
	return out
}

// cosBandIntervals returns the u-intervals where
// lo <= cos(theta0 + u*sweep - psi) <= hi, with u in [0, 1].
func cosBandIntervals(theta0, sweep, psi, lo, hi float64) []Interval {
	start := theta0 - psi
	a, b := start, start+sweep
	if a > b {
		a, b = b, a
	}
	bounds := []float64{a, b}
	addRoots := func(k float64) {
		if k < -1 || k > 1 {
			return
		}
		alpha := math.Acos(k)
		for _, base := range []float64{alpha, -alpha} {
			n0 := int(math.Floor((a-base)/(2*math.Pi))) - 1
			n1 := int(math.Ceil((b-base)/(2*math.Pi))) + 1
			for n := n0; n <= n1; n++ {
				t := base + 2*math.Pi*float64(n)
				if t > a+eps && t < b-eps {
					bounds = append(bounds, t)
				}
			}
		}
	}
	addRoots(lo)
	addRoots(hi)
	sort.Float64s(bounds)
	uniq := bounds[:0]
	for _, t := range bounds {
		if len(uniq) == 0 || t-uniq[len(uniq)-1] > eps {
			uniq = append(uniq, t)
		}
	}
	inBand := func(t float64) bool {
		c := math.Cos(t)
		return c >= lo-eps && c <= hi+eps
	}
	toU := func(t float64) float64 { return (t - start) / sweep }
	var out []Interval
	emit := func(t0, t1 float64) {
		u0, u1 := toU(t0), toU(t1)
		if u0 > u1 {
			u0, u1 = u1, u0
		}
		out = append(out, clipInterval(u0, u1)...)
	}
	for i := 0; i+1 < len(uniq); i++ {
		t0, t1 := uniq[i], uniq[i+1]
		if inBand(0.5 * (t0 + t1)) {
			emit(t0, t1)
		}
	}
	// Isolated tangencies: single angles inside the band.
	for _, t := range uniq {
		if inBand(t) {
			emit(t, t)
		}
	}
	return mergeIntervals(out)
}
