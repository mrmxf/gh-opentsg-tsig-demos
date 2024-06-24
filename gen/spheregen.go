package main

import (
	"fmt"
	"io"
	"math"
	"os"

	"encoding/json"
)

func main() {
	f, _ := os.Create("out/realTiles.obj")
	maxAngle := 30.0
	maxTheta := (math.Pi / 180) * maxAngle

	GenSphereOBJ(f, 0.5, 0.5, 5, maxTheta, maxTheta, 500, 500)

	fs, _ := os.Create("out/realTilesSmall.obj")
	fst, _ := os.Create("out/realTilesSmall.json")

	GenSphereOBJSquare(fs, fst, 0.5, 0.5, 5, maxTheta, maxTheta, 500, 500)
	//ObjToTsig("out/realTiles.obj", 100, 100) 6000,6000
	//sphere()

	fc, _ := os.Create("out/realTilesCurve.obj")
	fct, _ := os.Create("out/realTilesCurve.json")
	GenCurveOBJ(fc, fct, 0.5, 0.5, 5, 5, maxTheta, 500, 500)

	fcu, _ := os.Create("out/realTilesCube.obj")
	fcut, _ := os.Create("out/realTilesCube.json")
	fmt.Println(GenHalfCubeOBJ(fcu, fcut, 0.5, 0.5, 5, 5, 2.5, 500, 500))
}

// make a distance calculator
// math.Sqrt(math.Pow((z+xinc)-prevX, 2) + math.Pow((z+yinc)-prevY, 2)

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

/*
width is x plane
depth is y plane
height is z plane
*/
func GenHalfCubeOBJ(wObj io.Writer, wTsig io.Writer, tileHeight, tileWidth float64, CubeWidth, CubeHeight, CubeDepth float64, dx, dy float64) error {

	// check the dimensions
	if int(math.Ceil(CubeWidth/tileWidth)) != int(CubeWidth/tileWidth) {
		return fmt.Errorf("cubewidth of %v does", CubeWidth)
	}

	if int(math.Ceil(CubeHeight/tileHeight)) != int(CubeHeight/tileHeight) {
		return fmt.Errorf("tile height of %v is not an integer multiple of a cube height of %v", tileHeight, CubeHeight)
	}

	// start at 0,0
	pixelWidth := (CubeWidth + CubeDepth*2) * dx
	pixelHeight := (CubeDepth*2 + CubeHeight) * dy

	// count of tiles in each segment of cube
	leftRight := int((CubeDepth * 2 / tileWidth) * (CubeHeight / tileHeight))
	boots := int((CubeDepth * 2 / tileWidth) * (CubeWidth / tileHeight))
	back := int((CubeHeight / tileHeight) * (CubeWidth / tileWidth))

	tiles := make([]Tilelayout, leftRight+boots+back)

	// calculate the uv map steps in each direction

	uStep := tileWidth / (CubeWidth + CubeDepth*2)
	vStep := tileHeight / (CubeDepth*2 + CubeHeight)

	// plane keeps the information for
	// each plane of the cube that is created
	type plane struct {
		// Tile step values
		iEnd, jEnd     float64
		iStep, jStep   float64
		iStart, jStart float64
		// UV map values
		uStart, vStart float64
		// the plane value that isn't moved
		planeConst float64
		// one of "x", "y" or "z"
		plane string

		// is it facing the expected direction
		inverse bool
	}

	planes := []plane{
		// left wall
		{iEnd: CubeDepth, jEnd: CubeHeight, iStep: tileWidth, jStep: tileHeight, planeConst: 0, plane: "y", vStart: (CubeDepth / tileHeight) * vStep, inverse: true, uStart: ((CubeDepth + CubeWidth) / tileWidth) * uStep},
		// right wall
		{iEnd: CubeDepth, jEnd: CubeHeight, iStep: tileWidth, jStep: tileHeight, planeConst: CubeWidth, plane: "y", vStart: (CubeDepth / tileHeight) * vStep},

		// back wall
		{iEnd: CubeWidth, jEnd: CubeHeight, iStep: tileWidth, jStep: tileHeight, planeConst: CubeDepth, plane: "x", vStart: (CubeDepth / tileHeight) * vStep, uStart: ((CubeDepth) / tileWidth) * uStep},

		// Top
		{iEnd: CubeDepth, jEnd: CubeWidth, iStep: tileWidth, jStep: tileHeight, planeConst: CubeHeight, plane: "z", vStart: ((CubeDepth + CubeHeight) / tileHeight) * vStep, uStart: ((CubeDepth) / tileWidth) * uStep},

		// Bottom
		{iEnd: CubeDepth, jEnd: CubeWidth, iStep: tileWidth, jStep: tileHeight, planeConst: 0, plane: "z", uStart: ((CubeDepth) / tileWidth) * uStep},
	}

	// count the vertexes per face
	count := 1
	tCount := 0

	for _, p := range planes {

		uTotal := (p.iEnd - p.iStart) / p.iStep
		width := uTotal * uStep

		ujTotal := (p.jEnd - p.jStart) / p.iStep
		ujwidth := ujTotal * uStep
		iCount := 0
		for i := p.iStart; i < p.iEnd; i += p.iStep {

			jCount := 0
			for j := p.jStart; j < p.jEnd; j += p.jStep {

				switch p.plane {
				case "x":

					wObj.Write([]byte(fmt.Sprintf("v %v %v %v \n", p.planeConst, i, j)))
					wObj.Write([]byte(fmt.Sprintf("v %v %v %v \n", p.planeConst, i+p.iStep, j)))
					wObj.Write([]byte(fmt.Sprintf("v %v %v %v \n", p.planeConst, i+p.iStep, j+p.jStep)))
					wObj.Write([]byte(fmt.Sprintf("v %v %v %v \n", p.planeConst, i, j+p.jStep)))

					wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", p.uStart+width-float64(iCount)*uStep, p.vStart+float64(jCount)*vStep)))
					wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", p.uStart+width-float64(iCount+1)*uStep, p.vStart+float64(jCount)*vStep)))
					wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", p.uStart+width-float64(iCount+1)*uStep, p.vStart+float64(jCount+1)*vStep)))
					wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", p.uStart+width-float64(iCount)*uStep, p.vStart+float64(jCount+1)*vStep)))

					tiles[tCount] = Tilelayout{Layout: Positions{Flat: XY{X: int((p.uStart + width - float64(iCount)*uStep) * pixelWidth), Y: int((1 - (p.vStart + float64(jCount+1)*vStep)) * pixelHeight)}, Size: XY{X: int(dx), Y: int(dy)}}}

				case "y":
					wObj.Write([]byte(fmt.Sprintf("v %v %v %v \n", i, p.planeConst, j)))
					wObj.Write([]byte(fmt.Sprintf("v %v %v %v \n", i+p.iStep, p.planeConst, j)))
					wObj.Write([]byte(fmt.Sprintf("v %v %v %v \n", i+p.iStep, p.planeConst, j+p.jStep)))
					wObj.Write([]byte(fmt.Sprintf("v %v %v %v \n", i, p.planeConst, j+p.jStep)))

					// if inversed change the direction of the uv map
					if p.inverse {
						wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", p.uStart+width-float64(iCount)*uStep, p.vStart+float64(jCount)*vStep)))
						wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", p.uStart+width-float64(iCount+1)*uStep, p.vStart+float64(jCount)*vStep)))
						wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", p.uStart+width-float64(iCount+1)*uStep, p.vStart+float64(jCount+1)*vStep)))
						wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", p.uStart+width-float64(iCount)*uStep, p.vStart+float64(jCount+1)*vStep)))

					} else {

						wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", p.uStart+float64(iCount)*uStep, p.vStart+float64(jCount)*vStep)))
						wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", p.uStart+float64(iCount+1)*uStep, p.vStart+float64(jCount)*vStep)))
						wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", p.uStart+float64(iCount+1)*uStep, p.vStart+float64(jCount+1)*vStep)))
						wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", p.uStart+float64(iCount)*uStep, p.vStart+float64(jCount+1)*vStep)))
					}

					tiles[tCount] = Tilelayout{Layout: Positions{Flat: XY{X: int((p.uStart + width - float64(iCount)*uStep) * pixelWidth), Y: int((1 - (p.vStart + float64(jCount+1)*vStep)) * pixelHeight)}, Size: XY{X: int(dx), Y: int(dy)}}}

				case "z":
					wObj.Write([]byte(fmt.Sprintf("v %v %v %v \n", i, j+p.jStep, p.planeConst)))
					wObj.Write([]byte(fmt.Sprintf("v %v %v %v \n", i, j, p.planeConst)))
					wObj.Write([]byte(fmt.Sprintf("v %v %v %v \n", i+p.iStep, j, p.planeConst)))
					wObj.Write([]byte(fmt.Sprintf("v %v %v %v \n", i+p.iStep, j+p.jStep, p.planeConst)))

					wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", p.uStart+ujwidth-float64(jCount+1)*uStep, p.vStart+float64(iCount)*vStep)))
					wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", p.uStart+ujwidth-float64(jCount)*uStep, p.vStart+float64(iCount)*vStep)))
					wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", p.uStart+ujwidth-float64(jCount)*uStep, p.vStart+float64(iCount+1)*vStep)))
					wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", p.uStart+ujwidth-float64(jCount+1)*uStep, p.vStart+float64(iCount+1)*vStep)))

					tiles[tCount] = Tilelayout{Layout: Positions{Flat: XY{X: int((p.uStart + ujwidth - float64(jCount)*uStep) * pixelWidth), Y: int((1 - (p.vStart + float64(iCount+1)*vStep)) * pixelHeight)}, Size: XY{X: int(dx), Y: int(dy)}}}

				default:
					continue
				}

				wObj.Write([]byte(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3)))
				count += 4
				jCount++
				tCount++
			}
			iCount++
		}
	}

	tsig := TPIG{Tilelayout: tiles, Dimensions: Dimensions{Flat: XY2D{X0: 0, X1: int(pixelWidth), Y0: 0, Y1: int(pixelHeight)}}}

	enc := json.NewEncoder(wTsig)
	enc.SetIndent("", "    ")

	return enc.Encode(tsig)

}

// Angle in radians
func GenCurveOBJ(wObj io.Writer, wTsig io.Writer, tileHeight, tileWidth float64, cylinderRadius, cylinderHeight, azimuthMaxAngle float64, dx, dy float64) {
	// r phi z

	// get the total angle covered by the cylinder.
	azimuthInc := (2 * math.Asin(tileWidth/(2*cylinderRadius)))

	z := 0.0
	count := 1
	azimuth := -azimuthMaxAngle

	pixelWidth := math.Ceil(2*azimuthMaxAngle/azimuthInc) * dx
	pixelHeight := math.Ceil(cylinderHeight/tileHeight) * dy

	// rows * column
	tiles := make([]Tilelayout, int(math.Ceil(2*azimuthMaxAngle/azimuthInc)*math.Ceil(cylinderHeight/tileHeight)))

	uWidth := 1 / (math.Ceil(2 * azimuthMaxAngle / azimuthInc))
	vheight := 1 / math.Ceil(cylinderHeight/tileHeight)
	v := 0.0

	i := 0
	for z < cylinderHeight {
		u := 1.0

		for azimuth <= azimuthMaxAngle {
			//	tileCount++

			// get angle change

			x1, y1, z1 := CylindricalToCartesian(cylinderRadius, z, azimuth)
			wObj.Write([]byte(fmt.Sprintf("v %v %v %v \n", x1, y1, z1)))
			wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", u, v)))

			x2, y2, z2 := CylindricalToCartesian(cylinderRadius, z, azimuth+azimuthInc) // increase azimuth
			wObj.Write([]byte(fmt.Sprintf("v %v %v %v \n", x2, y2, z2)))
			wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", u-uWidth, v)))

			x3, y3, z3 := CylindricalToCartesian(cylinderRadius, z+tileHeight, azimuth+azimuthInc) // increase azimuth and height
			wObj.Write([]byte(fmt.Sprintf("v %v %v %v \n", x3, y3, z3)))
			wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", u-uWidth, v+vheight)))

			x4, y4, z4 := CylindricalToCartesian(cylinderRadius, z+tileHeight, azimuth) // increase height
			wObj.Write([]byte(fmt.Sprintf("v %v %v %v \n", x4, y4, z4)))
			wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", u, v+vheight)))

			wObj.Write([]byte(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3)))

			azimuth += azimuthInc
			u -= uWidth
			count += 4

			tiles[i] = Tilelayout{Layout: Positions{Flat: XY{X: int(u * pixelWidth), Y: int((1 - (v + vheight)) * pixelHeight)}, Size: XY{X: int(dx), Y: int(dy)}}}

			i++
		}

		// increase the z height
		// as well as the uv map height
		v += vheight
		z += tileHeight
		azimuth = -azimuthMaxAngle
	}

	tsig := TPIG{Tilelayout: tiles, Dimensions: Dimensions{Flat: XY2D{X0: 0, X1: int(pixelWidth), Y0: 0, Y1: int(pixelHeight)}}}

	enc := json.NewEncoder(wTsig)
	enc.SetIndent("", "    ")
	enc.Encode(tsig)

}

// Angle in radians
func GenSphereOBJ(w io.Writer, tileHeight, tileWidth float64, sphereRadius, thetaMaxAngle, azimuthMaxAngle float64, dx, dy float64) {

	azimuth, clockAz := 0.0, 0.0
	// tileCount := 0
	theta := math.Pi / 2
	count := 1

	thetaInc := 2 * (math.Asin(tileHeight / (2 * sphereRadius)))
	azimuthInc := (2 * math.Asin(tileWidth/(2*sphereRadius)))
	theta = (math.Pi / 2)

	// @TODO deal with the slight voerlaps

	uWidth := 1 / (2 * math.Ceil(azimuthMaxAngle/azimuthInc))
	vheight := 1 / (2 * math.Ceil(thetaMaxAngle/thetaInc))

	//@TODO maybe add the ushift to everything
	maxX := 2 * math.Ceil(azimuthMaxAngle/azimuthInc) * dx
	/*
		tileX := tileWidth / 0.001 //consitent dy dx for the moment
		tileY := tileHeight / 0.001
		sizeX := tileX * (2 * math.Ceil(azimuthMaxAngle/azimuthInc))
		sizeY := tileY * (2 * math.Ceil(thetaMaxAngle/thetaInc))
	*/

	//tsig information

	//sizex := 3840
	/*
	   generate the size of the bas eimage then move erveything along
	*/
	// TOP
	v := 0.5
	for theta > (math.Pi/2)-thetaMaxAngle {
		//start Point :=
		topLeftThet := theta - thetaInc
		topLeftAz := azimuth
		// botLeftAz := azimuth

		u := 0.5
		uBot := 0.5
		//		prevUshift := 0.0

		azimuthInc := (2 * math.Asin(tileWidth/(2*sphereRadius))) / math.Sin(theta)
		azimuthIncTop := (2 * math.Asin(tileWidth/(2*sphereRadius))) / math.Sin(topLeftThet)

		futDif := 2 * sphereRadius * (math.Sin((azimuthIncTop-azimuthInc)/2) * math.Sin(theta))

		shift := int((futDif) / (tileWidth / dx))
		ushift := float64(shift/2) * (1.0 / float64(maxX))

		for azimuth < azimuthMaxAngle {
			//	tileCount++

			// get angle change

			x1, y1, z1 := PolarToCartesian(sphereRadius, topLeftThet+thetaInc, topLeftAz)
			w.Write([]byte(fmt.Sprintf("v %v %v %v #bottom left\n", x1, y1, z1)))

			// 	d := 2 * sphereRadius * math.Sin(topLeftAz-botLeftAz)

			//	ushift := float64(shift) * (1.0 / 3840.0)
			// this U needs to shift to the right
			x2, y2, z2 := PolarToCartesian(sphereRadius, topLeftThet+thetaInc, topLeftAz+azimuthInc) // increase azimuth
			w.Write([]byte(fmt.Sprintf("v %v %v %v # bottom right\n", x2, y2, z2)))

			// nlX, nlY, nlZ := PolarToCartesian(sphereRadius, topLeftThet+thetaInc, topLeftAz+azimuthIncTop)

			w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uBot+ushift), v)))        // x1
			w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uBot+uWidth+ushift), v))) //x2
			//w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(u), v)))        // x1
			//	w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(u+uWidth), v))) //x2

			x3, y3, z3 := PolarToCartesian(sphereRadius, topLeftThet, topLeftAz+azimuthIncTop) // increase azimuth and height
			w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x3, y3, z3)))
			w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(u+uWidth), v+vheight)))

			x4, y4, z4 := PolarToCartesian(sphereRadius, topLeftThet, topLeftAz) // increase height
			w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x4, y4, z4)))
			w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(u), v+vheight)))

			w.Write([]byte(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3)))

			//			fmt.Println("4", 1-(u), "3", 1-(u+uWidth))
			//			fmt.Println("shift", ushift, prevUshift)
			//		botLeftAz = topLeftAz + azimuthInc
			uBot += uWidth + (ushift * 2)
			azimuth += azimuthIncTop
			topLeftAz = azimuth
			count += 4
			u += uWidth

			//prevUshift = ushift
		}

		topRightAz := clockAz
		u = 0.5
		uBot = 0.5

		for clockAz > -thetaMaxAngle {
			//	tileCount++

			x1, y1, z1 := PolarToCartesian(sphereRadius, topLeftThet+thetaInc, topRightAz)
			w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x1, y1, z1)))
			w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uBot-ushift), v)))

			x2, y2, z2 := PolarToCartesian(sphereRadius, topLeftThet+thetaInc, topRightAz-azimuthInc) // increase azimuth
			w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x2, y2, z2)))
			w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uBot-uWidth-ushift), v)))

			x3, y3, z3 := PolarToCartesian(sphereRadius, topLeftThet, topRightAz-azimuthIncTop) // increase azimuth and height
			w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x3, y3, z3)))
			w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(u-uWidth), v+vheight)))

			x4, y4, z4 := PolarToCartesian(sphereRadius, topLeftThet, topRightAz) // increase height
			w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x4, y4, z4)))
			w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-u, v+vheight)))

			w.Write([]byte(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3)))

			//	tiles = append(tiles, Tilelayout{Layout: Positions{Flat: XY{X: int((uBot - ushift) * maxX), Y: int((1 - (v + vheight)) * maxY)}, Size: XY{X: int(dx), Y: int(dy)}}})

			//	objbuf.WriteString(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3))
			clockAz -= azimuthIncTop
			topRightAz = clockAz
			count += 4
			u -= uWidth

			uBot -= (uWidth + (ushift * 2))

		}

		theta -= thetaInc
		azimuth = 0
		clockAz = 0
		v += vheight
		//fmt.Println("COINTER", theta, z, zinchold)
		//	z = zinchold

	}

	// Bottom
	v = 0.5
	theta = math.Pi / 2
	for theta < (math.Pi/2)+thetaMaxAngle {
		//start Point :=
		botLeftThet := theta + thetaInc
		botLeftAz := azimuth
		u := 0.5
		uTop := 0.5

		azimuthInc := (2 * math.Asin(tileWidth/(2*sphereRadius))) / math.Sin(theta)
		azimuthIncBot := (2 * math.Asin(tileWidth/(2*sphereRadius))) / math.Sin(botLeftThet)

		futDif := 2 * sphereRadius * (math.Sin((azimuthIncBot-azimuthInc)/2) * math.Sin(botLeftThet-thetaInc))

		shift := int((futDif / 2) / (tileWidth / dx))

		ushift := float64(shift) * (1.0 / float64(maxX))
		for azimuth < thetaMaxAngle {
			// tileCount++

			x1, y1, z1 := PolarToCartesian(sphereRadius, botLeftThet-thetaInc, botLeftAz)
			w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x1, y1, z1)))
			w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uTop+ushift), v)))

			x2, y2, z2 := PolarToCartesian(sphereRadius, botLeftThet-thetaInc, botLeftAz+azimuthInc) // increase azimuth
			w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x2, y2, z2)))
			w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uTop+uWidth+ushift), v)))

			x3, y3, z3 := PolarToCartesian(sphereRadius, botLeftThet, botLeftAz+azimuthIncBot) // increase azimuth and height
			w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x3, y3, z3)))
			w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(u+uWidth), v-vheight)))

			x4, y4, z4 := PolarToCartesian(sphereRadius, botLeftThet, botLeftAz) // increase height
			w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x4, y4, z4)))
			w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(u), v-vheight)))

			//	fmt.Println(math.Sqrt(math.Pow((x2)-x1, 2)+math.Pow((y2)-y1, 2)) + math.Pow((z2)-z1, 2))

			w.Write([]byte(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3)))
			//	tiles = append(tiles, Tilelayout{Layout: Positions{Flat: XY{X: int(u * pixelWidth), Y: int((1 - (v + vheight)) * pixelHeight)}, Size: XY{X: int(dx), Y: int(dy)}}})
			//	objbuf.WriteString(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3))
			azimuth += azimuthIncBot
			botLeftAz = azimuth
			count += 4
			u += uWidth
			// uTop += uWidth + (ushift * 2)'
			uTop += uWidth + ushift
		}

		botRightAz := clockAz
		u = 0.5
		uTop = 0.5
		for clockAz > -thetaMaxAngle {
			// tileCount++

			azimuthInc := (2 * math.Asin(tileWidth/(2*sphereRadius))) / math.Sin(theta)
			azimuthIncTop := (2 * math.Asin(tileWidth/(2*sphereRadius))) / math.Sin(botLeftThet)
			x1, y1, z1 := PolarToCartesian(sphereRadius, botLeftThet-thetaInc, botRightAz)
			w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x1, y1, z1)))
			w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uTop-ushift), v)))

			x2, y2, z2 := PolarToCartesian(sphereRadius, botLeftThet-thetaInc, botRightAz-azimuthInc) // increase azimuth
			w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x2, y2, z2)))
			w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uTop-uWidth-ushift), v)))

			x3, y3, z3 := PolarToCartesian(sphereRadius, botLeftThet, botRightAz-azimuthIncTop) // increase azimuth and height
			w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x3, y3, z3)))
			w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(u-uWidth), v-vheight)))

			x4, y4, z4 := PolarToCartesian(sphereRadius, botLeftThet, botRightAz) // increase height
			w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x4, y4, z4)))
			w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-u, v-vheight)))

			w.Write([]byte(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3)))

			//	objbuf.WriteString(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3))
			clockAz -= azimuthIncTop
			botRightAz = clockAz
			count += 4
			u -= uWidth
			uTop -= (uWidth + (ushift * 2))
		}

		v -= vheight
		theta += thetaInc
		azimuth = 0
		clockAz = 0
		//fmt.Println("COINTER", theta, z, zinchold)
		//	z = zinchold

	}
}

// Angle in radians
func GenSphereOBJSquare(w, wTsig io.Writer, tileHeight, tileWidth float64, sphereRadius, thetaMaxAngle, azimuthMaxAngle float64, dx, dy float64) {

	azimuth, clockAz := 0.0, 0.0
	// tileCount := 0
	theta := math.Pi / 2
	count := 1

	thetaInc := 2 * (math.Asin(tileHeight / (2 * sphereRadius)))
	azimuthInc := (2 * math.Asin(tileWidth/(2*sphereRadius)))
	theta = (math.Pi / 2)

	// @TODO deal with the slight voerlaps

	uWidth := 1 / (2 * math.Ceil(azimuthMaxAngle/azimuthInc))
	vheight := 1 / (2 * math.Ceil(thetaMaxAngle/thetaInc))

	//@TODO maybe add the ushift to everything
	maxX := 2 * math.Ceil(azimuthMaxAngle/azimuthInc) * dx
	maxY := 2 * math.Ceil(thetaMaxAngle/thetaInc) * dy
	/*
		tileX := tileWidth / 0.001 //consitent dy dx for the moment
		tileY := tileHeight / 0.001
		sizeX := tileX * (2 * math.Ceil(azimuthMaxAngle/azimuthInc))
		sizeY := tileY * (2 * math.Ceil(thetaMaxAngle/thetaInc))
	*/
	tiles := []Tilelayout{}
	//sizex := 3840
	/*
	   generate the size of the bas eimage then move erveything along
	*/
	// TOP
	v := 0.5

	for theta > (math.Pi/2)-thetaMaxAngle {
		//start Point :=
		topLeftThet := theta - thetaInc
		topLeftAz := azimuth
		// botLeftAz := azimuth

		u := 0.5
		uBot := 0.5
		//		prevUshift := 0.0

		azimuthInc := (2 * math.Asin(tileWidth/(2*sphereRadius))) / math.Sin(theta)
		azimuthIncTop := (2 * math.Asin(tileWidth/(2*sphereRadius))) / math.Sin(topLeftThet)

		// futDif is the length chordal length difference of the azimuth change on the bottom row.
		// which is the closest current approximation
		futDif := 2 * sphereRadius * (math.Sin((azimuthIncTop-azimuthInc)/2) * math.Sin(theta))

		// find the difference in pixels
		shift := int((futDif)/(tileWidth/dx)) / 2
		ushift := (float64(shift)) * (1.0 / float64(maxX))

		//fmt.Println(futDif, 2*sphereRadius*(math.Sin((azimuthIncTop-azimuthInc)/2)*math.Sin(theta)), shift)
		dropCount := 0

		for azimuth < azimuthMaxAngle {
			//	tileCount++

			/*
				each shift is increased by the count of shift
				so second row goes 1 + 1 + 1
				row below is 2 + 2 + 2 etc
				row below is 3 + 3 + 3
			*/

			x1, y1, z1 := PolarToCartesian(sphereRadius, topLeftThet+thetaInc, topLeftAz)
			x2, y2, z2 := PolarToCartesian(sphereRadius, topLeftThet+thetaInc, topLeftAz+azimuthInc) // increase azimuth
			x3, y3, z3 := PolarToCartesian(sphereRadius, topLeftThet, topLeftAz+azimuthIncTop)       // increase azimuth and height
			x4, y4, z4 := PolarToCartesian(sphereRadius, topLeftThet, topLeftAz)                     // increase height to the bottom

			// for each drop of a pixel shift that row along one
			// to that the uv map that is created is square and can be made a tsig.
			// @TODO update so each drop is two pixels and is a pixel eitherway
			if shift > 0 {
				step := int(dy / float64((shift)+1))
				botX, botY, botZ := x1, y1, z1
				botRX, botRY, botRZ := x2, y2, z2

				leftVectX, leftVectY, leftVectZ := (float64(step)*(x4-x1))/dy, (float64(step)*(y4-y1))/dy, (float64(step)*(z4-z1))/dy
				rightVectX, rightVectY, rightVectZ := (float64(step)*(x3-x2))/dy, (float64(step)*(y3-y2))/dy, (float64(step)*(z3-z2))/dy

				//	vstep := float64(step) * (1.0 / float64(maxY))
				vstep := float64(step) / maxY //(vheight / float64(shift+1))
				ustep := (1.0 / float64(maxX))

				for i := 0; i < shift; i++ {

					//	fmt.Println(x1, y1, x2, z2)

					topX, topY, topZ := botX+leftVectX, botY+leftVectY, botZ+leftVectZ
					topRX, topRY, topRZ := botRX+rightVectX, botRY+rightVectY, botRZ+rightVectZ
					pos := shift - i

					w.Write([]byte(fmt.Sprintf("v %v %v %v\n", botX, botY, botZ)))
					w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uBot+float64((pos))*ustep+float64(dropCount*pos)*ustep), v+(float64(i)*vstep))))

					w.Write([]byte(fmt.Sprintf("v %v %v %v \n", botRX, botRY, botRZ)))
					w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uWidth+uBot+float64((pos))*ustep+float64(dropCount*pos)*ustep), v+(float64(i)*vstep))))

					w.Write([]byte(fmt.Sprintf("v %v %v %v \n", topRX, topRY, topRZ)))
					w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uWidth+uBot+float64((pos))*ustep+float64(dropCount*pos)*ustep), v+(float64(i+1)*vstep))))

					w.Write([]byte(fmt.Sprintf("v %v %v %v \n", topX, topY, topZ)))
					w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uBot+float64((pos))*ustep+float64(dropCount*pos)*ustep), v+(float64(i+1)*vstep))))
					w.Write([]byte(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3)))

					tiles = append(tiles, Tilelayout{Layout: Positions{
						Flat: XY{X: int((1 - (uBot + uWidth + float64((pos))*ustep + float64(dropCount*pos)*ustep)) * maxX), Y: int(math.Round((1 - (v + (float64(i+1) * vstep))) * maxY))},
						Size: XY{X: int(dx), Y: int(maxY * vstep)}}})

					botX, botY, botZ = topX, topY, topZ
					botRX, botRY, botRZ = topRX, topRY, topRZ
					count += 4
				}

				// the max v picks off from the last one to accoount for rounding errors
				w.Write([]byte(fmt.Sprintf("v %v %v %v \n", botX, botY, botZ)))
				w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uBot), v+(vstep*(float64(shift))))))

				w.Write([]byte(fmt.Sprintf("v %v %v %v \n", botRX, botRY, botRZ)))
				w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uWidth+uBot), v+(vstep*(float64(shift))))))

				w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x3, y3, z3)))
				w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uWidth+uBot), v+vheight)))

				w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x4, y4, z4)))
				w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uBot), v+vheight)))

				tiles = append(tiles, Tilelayout{Layout: Positions{
					Flat: XY{X: int((1 - (uBot + uWidth)) * maxX), Y: int(math.Round((1 - (v + vheight)) * maxY))},
					Size: XY{X: int(dx), Y: int(math.Round(maxY * (vheight - vstep*(float64(shift)))))}}})

				// dropCount++
			} else {

				// get angle change

				w.Write([]byte(fmt.Sprintf("v %v %v %v #bottom left\n", x1, y1, z1)))
				// increase azimuth
				w.Write([]byte(fmt.Sprintf("v %v %v %v # bottom right\n", x2, y2, z2)))
				// nlX, nlY, nlZ := PolarToCartesian(sphereRadius, topLeftThet+thetaInc, topLeftAz+azimuthIncTop)
				w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uBot+ushift), v)))        // x1
				w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uBot+uWidth+ushift), v))) //x2
				//w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(u), v)))        // x1
				//	w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(u+uWidth), v))) //x2

				// increase azimuth and height
				w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x3, y3, z3)))
				w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(u+uWidth), v+vheight)))

				// increase height
				w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x4, y4, z4)))
				w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(u), v+vheight)))

				tiles = append(tiles, Tilelayout{Layout: Positions{
					Flat: XY{X: int((1 - (uBot + ushift)) * maxX), Y: int((1 - (v + vheight)) * maxY)},
					Size: XY{X: int(dx), Y: int(dy)}}})

			}
			w.Write([]byte(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3)))

			// nlX, nlY, nlZ := PolarToCartesian(sphereRadius, topLeftThet+thetaInc, topLeftAz+azimuthIncTop)

			//			fmt.Println("4", 1-(u), "3", 1-(u+uWidth))
			//			fmt.Println("shift", ushift, prevUshift)
			//		botLeftAz = topLeftAz + azimuthInc
			uBot += uWidth
			azimuth += azimuthIncTop
			topLeftAz = azimuth
			count += 4
			u += uWidth
			dropCount += 2

		}

		topRightAz := clockAz
		u = 0.5
		uBot = 0.5

		dropCount = 0
		for clockAz > -thetaMaxAngle {

			x1, y1, z1 := PolarToCartesian(sphereRadius, topLeftThet+thetaInc, topRightAz)
			x2, y2, z2 := PolarToCartesian(sphereRadius, topLeftThet+thetaInc, topRightAz-azimuthInc)
			x3, y3, z3 := PolarToCartesian(sphereRadius, topLeftThet, topRightAz-azimuthIncTop)
			x4, y4, z4 := PolarToCartesian(sphereRadius, topLeftThet, topRightAz)

			if shift > 0 {
				step := int(dy / float64(shift+1))
				botX, botY, botZ := x1, y1, z1
				botRX, botRY, botRZ := x2, y2, z2

				leftVectX, leftVectY, leftVectZ := (float64(step)*(x4-x1))/dy, (float64(step)*(y4-y1))/dy, (float64(step)*(z4-z1))/dy
				rightVectX, rightVectY, rightVectZ := (float64(step)*(x3-x2))/dy, (float64(step)*(y3-y2))/dy, (float64(step)*(z3-z2))/dy

				//	vstep := float64(step) * (1.0 / float64(maxY))
				vstep := float64(step) / maxY //(vheight / float64(shift+1))
				ustep := (1.0 / float64(maxX))

				for i := 0; i < shift; i++ {

					//	fmt.Println(x1, y1, x2, z2)
					//////////////TARGET//////////////

					topX, topY, topZ := botX+leftVectX, botY+leftVectY, botZ+leftVectZ
					topRX, topRY, topRZ := botRX+rightVectX, botRY+rightVectY, botRZ+rightVectZ
					pos := shift - i
					stepOffset := -float64((pos))*ustep - float64(dropCount*pos)*ustep

					w.Write([]byte(fmt.Sprintf("v %v %v %v \n", botX, botY, botZ)))
					w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uBot+stepOffset), v+(float64(i)*vstep))))

					w.Write([]byte(fmt.Sprintf("v %v %v %v \n", botRX, botRY, botRZ)))
					w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(-uWidth+uBot+stepOffset), v+(float64(i)*vstep))))

					w.Write([]byte(fmt.Sprintf("v %v %v %v \n", topRX, topRY, topRZ)))
					w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(-uWidth+uBot+stepOffset), v+(float64(i+1)*vstep))))

					w.Write([]byte(fmt.Sprintf("v %v %v %v \n", topX, topY, topZ)))
					w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uBot+stepOffset), v+(float64(i+1)*vstep))))
					w.Write([]byte(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3)))

					tiles = append(tiles, Tilelayout{Layout: Positions{
						Flat: XY{X: int((1 - (uBot + stepOffset)) * maxX), Y: int(math.Round((1 - (v + (float64(i+1) * vstep))) * maxY))},
						Size: XY{X: int(dx), Y: int(maxY * vstep)}}})

					botX, botY, botZ = topX, topY, topZ
					botRX, botRY, botRZ = topRX, topRY, topRZ
					count += 4
				}

				w.Write([]byte(fmt.Sprintf("v %v %v %v \n", botX, botY, botZ)))
				w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uBot), v+(vstep*(float64(shift))))))

				w.Write([]byte(fmt.Sprintf("v %v %v %v \n", botRX, botRY, botRZ)))
				w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(-uWidth+uBot), v+(vstep*(float64(shift))))))

				w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x3, y3, z3)))
				w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(-uWidth+uBot), v+vheight)))

				w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x4, y4, z4)))
				w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uBot), v+vheight)))

				tiles = append(tiles, Tilelayout{Layout: Positions{
					Flat: XY{X: int((1 - uBot) * maxX), Y: int(math.Round((1 - (v + vheight)) * maxY))},
					Size: XY{X: int(dx), Y: int(math.Round(maxY * (vheight - vstep*(float64(shift)))))}}})

				// dropCount++
			} else {
				//	tileCount++

				w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x1, y1, z1)))
				w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uBot-ushift), v)))

				// increase azimuth
				w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x2, y2, z2)))
				w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uBot-uWidth-ushift), v)))

				// increase azimuth and height
				w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x3, y3, z3)))
				w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(u-uWidth), v+vheight)))

				// increase height
				w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x4, y4, z4)))
				w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-u, v+vheight)))
			}

			w.Write([]byte(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3)))

			//	objbuf.WriteString(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3))
			clockAz -= azimuthIncTop
			topRightAz = clockAz
			count += 4
			u -= uWidth
			dropCount += 2
			uBot -= (uWidth)

		}

		theta -= thetaInc
		azimuth = 0
		clockAz = 0
		v += vheight
		//fmt.Println("COINTER", theta, z, zinchold)
		//	z = zinchold

	}

	// Bottom
	v = 0.5
	theta = math.Pi / 2
	for theta < (math.Pi/2)+thetaMaxAngle {
		//start Point :=
		botLeftThet := theta + thetaInc
		botLeftAz := azimuth
		u := 0.5
		uTop := 0.5

		azimuthInc := (2 * math.Asin(tileWidth/(2*sphereRadius))) / math.Sin(theta)
		azimuthIncBot := (2 * math.Asin(tileWidth/(2*sphereRadius))) / math.Sin(botLeftThet)

		futDif := 2 * sphereRadius * (math.Sin((azimuthIncBot-azimuthInc)/2) * math.Sin(botLeftThet-thetaInc))

		shift := int((futDif / 2) / (tileWidth / dx))

		ushift := float64(shift) * (1.0 / float64(maxX))
		dropCount := 0

		for azimuth < thetaMaxAngle {

			// tileCount++
			x1, y1, z1 := PolarToCartesian(sphereRadius, botLeftThet-thetaInc, botLeftAz)
			x2, y2, z2 := PolarToCartesian(sphereRadius, botLeftThet-thetaInc, botLeftAz+azimuthInc) // increase azimuth
			x3, y3, z3 := PolarToCartesian(sphereRadius, botLeftThet, botLeftAz+azimuthIncBot)       // increase azimuth and height
			x4, y4, z4 := PolarToCartesian(sphereRadius, botLeftThet, botLeftAz)                     // increase height
			if shift > 0 {
				step := int(dy / float64(shift+1))
				topX, topY, topZ := x1, y1, z1
				topRX, topRY, topRZ := x2, y2, z2

				leftVectX, leftVectY, leftVectZ := (float64(step)*(x4-x1))/dy, (float64(step)*(y4-y1))/dy, (float64(step)*(z4-z1))/dy
				rightVectX, rightVectY, rightVectZ := (float64(step)*(x3-x2))/dy, (float64(step)*(y3-y2))/dy, (float64(step)*(z3-z2))/dy

				//	vstep := float64(step) * (1.0 / float64(maxY))
				vstep := float64(step) / maxY // (vheight / float64(shift+1))
				ustep := (1.0 / float64(maxX))

				for i := 0; i < shift; i++ {

					botX, botY, botZ := topX+leftVectX, topY+leftVectY, topZ+leftVectZ
					botRX, botRY, botRZ := topRX+rightVectX, topRY+rightVectY, topRZ+rightVectZ
					pos := shift - i

					w.Write([]byte(fmt.Sprintf("v %v %v %v \n", topX, topY, topZ)))
					w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uTop+float64((pos))*ustep+float64(dropCount*pos)*ustep), v-float64(i)*vstep)))

					w.Write([]byte(fmt.Sprintf("v %v %v %v \n", topRX, topRY, topRZ)))
					w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uTop+uWidth+float64((pos))*ustep+float64(dropCount*pos)*ustep), v-float64(i)*vstep)))

					w.Write([]byte(fmt.Sprintf("v %v %v %v \n", botRX, botRY, botRZ)))
					w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uTop+uWidth+float64((pos))*ustep+float64(dropCount*pos)*ustep), v-float64(i+1)*vstep)))

					w.Write([]byte(fmt.Sprintf("v %v %v %v \n", botX, botY, botZ)))
					w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uTop+float64((pos))*ustep+float64(dropCount*pos)*ustep), v-float64(i+1)*vstep)))
					w.Write([]byte(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3)))

					tiles = append(tiles, Tilelayout{Layout: Positions{
						Flat: XY{X: int((1 - (uTop + uWidth + float64((pos))*ustep + float64(dropCount*pos)*ustep)) * maxX), Y: int(math.Round((1 - (v - (float64(i) * vstep))) * maxY))},
						Size: XY{X: int(dx), Y: int(maxY * vstep)}}})

					topX, topY, topZ = botX, botY, botZ
					topRX, topRY, topRZ = botRX, botRY, botRZ
					count += 4
				}

				w.Write([]byte(fmt.Sprintf("v %v %v %v \n", topX, topY, topZ)))
				w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uTop), v-float64(shift)*vstep)))

				w.Write([]byte(fmt.Sprintf("v %v %v %v \n", topRX, topRY, topRZ)))
				w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uTop+uWidth), v-float64(shift)*vstep)))

				w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x3, y3, z3)))
				w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uTop+uWidth), v-vheight)))

				w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x4, y4, z4)))
				w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uTop), v-vheight)))

				tiles = append(tiles, Tilelayout{Layout: Positions{
					Flat: XY{X: int((1 - (uTop + uWidth)) * maxX), Y: int(math.Round((1 - (v - float64(shift)*vstep)) * maxY))},
					Size: XY{X: int(dx), Y: int(math.Round(maxY * (vheight - vstep*(float64(shift)))))}}})

				//	leftVectX, leftVectY, leftVectZ := (x4-x1)/dy, (y4-y1)/dy, (z4-z1)/dy
				//	rightVectX, rightVectY, rightVectZ := (x3-x2)/dy, (y3-y2)/dy, (z3-z2)/dy
				/*

						handle the u differently

						numberOfShifs := shift

					// +1 to rember the 0th line and get the correct amount of increments
					fmt.Println("step", shift+1, thetaInc, int(dy/float64(shift+1)))
				*/

				// print the length difference along the
			} else {

				w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x1, y1, z1)))
				w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uTop+ushift), v)))

				w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x2, y2, z2)))
				w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uTop+uWidth+ushift), v)))

				w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x3, y3, z3)))
				w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(u+uWidth), v-vheight)))

				w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x4, y4, z4)))
				w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(u), v-vheight)))

			}

			//	fmt.Println(math.Sqrt(math.Pow((x2)-x1, 2)+math.Pow((y2)-y1, 2)) + math.Pow((z2)-z1, 2))

			w.Write([]byte(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3)))

			//	objbuf.WriteString(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3))
			azimuth += azimuthIncBot
			botLeftAz = azimuth
			count += 4
			u += uWidth
			// uTop += uWidth + (ushift * 2)'
			uTop += uWidth //+ ushift
			dropCount += 2
		}

		botRightAz := clockAz
		u = 0.5
		uTop = 0.5
		dropCount = 0
		for clockAz > -thetaMaxAngle {

			azimuthInc := (2 * math.Asin(tileWidth/(2*sphereRadius))) / math.Sin(theta)
			azimuthIncTop := (2 * math.Asin(tileWidth/(2*sphereRadius))) / math.Sin(botLeftThet)
			x1, y1, z1 := PolarToCartesian(sphereRadius, botLeftThet-thetaInc, botRightAz)
			x2, y2, z2 := PolarToCartesian(sphereRadius, botLeftThet-thetaInc, botRightAz-azimuthInc) // increase azimuth
			x3, y3, z3 := PolarToCartesian(sphereRadius, botLeftThet, botRightAz-azimuthIncTop)       // increase azimuth and height
			x4, y4, z4 := PolarToCartesian(sphereRadius, botLeftThet, botRightAz)                     // increase height to the bottom

			if shift > 0 {
				step := int(dy / float64(shift+1))
				topX, topY, topZ := x1, y1, z1
				topRX, topRY, topRZ := x2, y2, z2

				leftVectX, leftVectY, leftVectZ := (float64(step)*(x4-x1))/dy, (float64(step)*(y4-y1))/dy, (float64(step)*(z4-z1))/dy
				rightVectX, rightVectY, rightVectZ := (float64(step)*(x3-x2))/dy, (float64(step)*(y3-y2))/dy, (float64(step)*(z3-z2))/dy

				//	vstep := float64(step) * (1.0 / float64(maxY))
				vstep := float64(step) / maxY // (vheight / float64(shift+1))
				ustep := (-1.0 / float64(maxX))

				//fmt.Println("start")
				for i := 0; i < shift; i++ {

					//	fmt.Println(x1, y1, x2, z2)

					botX, botY, botZ := topX+leftVectX, topY+leftVectY, topZ+leftVectZ
					botRX, botRY, botRZ := topRX+rightVectX, topRY+rightVectY, topRZ+rightVectZ
					pos := shift - i

					w.Write([]byte(fmt.Sprintf("v %v %v %v \n", topX, topY, topZ)))
					w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uTop+float64((pos))*ustep+float64(dropCount*pos)*ustep), v-float64(i)*vstep)))

					w.Write([]byte(fmt.Sprintf("v %v %v %v \n", topRX, topRY, topRZ)))
					w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uTop-uWidth+float64((pos))*ustep+float64(dropCount*pos)*ustep), v-float64(i)*vstep)))

					w.Write([]byte(fmt.Sprintf("v %v %v %v \n", botRX, botRY, botRZ)))
					w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uTop-uWidth+float64((pos))*ustep+float64(dropCount*pos)*ustep), v-float64(i+1)*vstep)))

					w.Write([]byte(fmt.Sprintf("v %v %v %v \n", botX, botY, botZ)))
					w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uTop+float64((pos))*ustep+float64(dropCount*pos)*ustep), v-float64(i+1)*vstep)))
					w.Write([]byte(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3)))

					tiles = append(tiles, Tilelayout{Layout: Positions{
						Flat: XY{X: int((1 - (uTop + float64((pos))*ustep + float64(dropCount*pos)*ustep)) * maxX), Y: int(math.Round((1 - (v - (float64(i) * vstep))) * maxY))},
						Size: XY{X: int(dx), Y: int(maxY * vstep)}}})

					topX, topY, topZ = botX, botY, botZ
					topRX, topRY, topRZ = botRX, botRY, botRZ
					count += 4
				}

				w.Write([]byte(fmt.Sprintf("v %v %v %v \n", topX, topY, topZ)))
				w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uTop), v-float64(shift)*vstep)))

				w.Write([]byte(fmt.Sprintf("v %v %v %v \n", topRX, topRY, topRZ)))
				w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uTop-uWidth), v-float64(shift)*vstep)))

				w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x3, y3, z3)))
				w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uTop-uWidth), v-vheight)))

				w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x4, y4, z4)))
				w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uTop), v-vheight)))

				tiles = append(tiles, Tilelayout{Layout: Positions{
					Flat: XY{X: int((1 - uTop) * maxX), Y: int(math.Round((1 - (v - float64(shift)*vstep)) * maxY))},
					Size: XY{X: int(dx), Y: int(math.Round(maxY * (vheight - vstep*(float64(shift)))))}}})
				//	leftVectX, leftVectY, leftVectZ := (x4-x1)/dy, (y4-y1)/dy, (z4-z1)/dy
				//	rightVectX, rightVectY, rightVectZ := (x3-x2)/dy, (y3-y2)/dy, (z3-z2)/dy
				/*

					handle the u differently

					numberOfShifs := shift
				*/
				// +1 to rember the 0th line and get the correct amount of increments

			} else {
				// tileCount++

				w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x1, y1, z1)))
				w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uTop-ushift), v)))

				w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x2, y2, z2)))
				w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uTop-uWidth-ushift), v)))

				// increase azimuth and height
				w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x3, y3, z3)))
				w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(uTop-uWidth), v-vheight)))

				w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x4, y4, z4)))
				w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-uTop, v-vheight)))

			}
			w.Write([]byte(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3)))

			//	objbuf.WriteString(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3))
			clockAz -= azimuthIncTop
			botRightAz = clockAz
			count += 4
			dropCount += 2
			u -= uWidth
			uTop -= (uWidth) // + (ushift * 2))
		}

		v -= vheight
		theta += thetaInc
		azimuth = 0
		clockAz = 0
		//fmt.Println("COINTER", theta, z, zinchold)
		//	z = zinchold

	}

	tsig := TPIG{Tilelayout: tiles, Dimensions: Dimensions{Flat: XY2D{X0: 0, X1: int(maxX), Y0: 0, Y1: int(maxY)}}}

	enc := json.NewEncoder(wTsig)
	enc.SetIndent("", "    ")

	enc.Encode(tsig)
}
