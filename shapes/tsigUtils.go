package shapes

import "math"

// Init ensures all the shapes init functions are called,
// it does nothing else
func Init() {

}

// PolarToCartesian takes polar coordinates of R, theta (inclination angle) and
// phi (Azimuth angle) and converts them to cartesian XYZ
func PolarToCartesian(r, theta, phi float64) (X, Y, Z float64) {
	X = r * math.Sin(theta) * math.Cos(phi)
	Y = r * math.Sin(theta) * math.Sin(phi)
	Z = r * math.Cos(theta)
	return
}

// PolarToCylindrical takes polar coordinates of R, theta (inclination angle) and
// phi (Azimuth angle) and converts them to cylindrical  coordinates
// of R, Z, Phi
func PolarToCylindrical(r, theta, phi float64) (R, Z, Phi float64) {
	R = r * math.Sin(theta)
	Z = r * math.Cos(theta)
	Phi = phi

	return
}

// CylindricalToCartesian takes polar coordinates of R, theta (inclination angle) and
// phi (Azimuth angle) and converts them to cylindrical  coordinates
// of R, Z, Phi
func CylindricalToCartesian(r, z, azimuth float64) (X, Y, Z float64) {
	X = r * math.Cos(azimuth)
	Y = r * math.Sin(azimuth)
	Z = z
	return
}

// ThreeDistance calculates the distance between 2 3d points
func ThreeDistance(x1, x2, y1, y2, z1, z2 float64) float64 {
	return math.Sqrt(math.Pow((x1)-x2, 2) + math.Pow(y1-y2, 2) + math.Pow(z1-z2, 2))
}
