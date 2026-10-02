package lathe

import "math"

// Segment 是一段刀尖中心轨迹，参数 u 从 0 到 1。
type Segment interface {
	position(u float64) Point
}

// Line 是直线段，允许零长度。u 按位移线性插值。
type Line struct {
	From, To Point
}

func (l Line) position(u float64) Point {
	return Point{
		X: l.From.X + u*(l.To.X-l.From.X),
		Z: l.From.Z + u*(l.To.Z-l.From.Z),
	}
}

// Arc 是圆弧段：圆心 Center、起点 Start、有符号扫角 Sweep（弧度），
// 正值逆时针，负值顺时针，绝对值不超过一整圈且非零。u 按扫角线性插值。
type Arc struct {
	Center, Start Point
	Sweep         float64
}

func (a Arc) position(u float64) Point {
	d := a.Start.sub(a.Center)
	theta := math.Atan2(d.Z, d.X) + a.Sweep*u
	r := d.norm()
	return Point{
		X: a.Center.X + r*math.Cos(theta),
		Z: a.Center.Z + r*math.Sin(theta),
	}
}
