package main

import (
	"fmt"
	"io"
	"math"

	"encoding/json"
)

func main() {

	/*
		maxAngle := 30.0
		maxTheta := (math.Pi / 180) * maxAngle
		fcu, _ := os.Create("./examples/Sphere.yaml")
		o, _ := yaml.Marshal(Sphere{0.5, 0.5, 5, maxTheta, maxTheta, 500, 500})
		fcu.Write(o)*/

	err := cmdObj.Execute()

	if err != nil {
		fmt.Println(err)
		return
	}

	/*
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

		fcu, _ := os.Create("./examples/cube.yaml")
		o, _ := yaml.Marshal(Cube{0.5, 0.5, 5, 5, 2.5, 500, 500})
		fcu.Write(o)
		/*
			fcu, _ := os.Create("out/realTilesCube.obj")
			fcut, _ := os.Create("out/realTilesCube.json")
			fmt.Println(GenHalfCubeOBJ(fcu, fcut, 0.5, 0.5, 5, 5, 2.5, 500, 500))*/
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
GenHalfCubeOBJ generates a TSIG and OBJ for a cube with no front panel.
The dimensions are as so:

  - Width is the x plane

  - Depth is the y plane

  - Height is the z plane

    Errors will be returned if the tiles do not fit exactly into the dimensions. E.g. a tile width of 1 is given and the cube has a width of 3.5
*/
func (c Cube) generate(wObj, wTsig io.Writer) error {

	// check the dimensions
	err := halfCubeFence(c.TileHeight, c.TileWidth, c.CubeWidth, c.CubeHeight, c.CubeDepth)

	if err != nil {
		return err
	}

	// get the dimensions of the flat display.
	pixelWidth := (c.CubeWidth + c.CubeDepth*2) * c.Dx
	pixelHeight := (c.CubeDepth*2 + c.CubeHeight) * c.Dy

	// count of tiles in each segment of cube
	leftRight := int((c.CubeDepth * 2 / c.TileWidth) * (c.CubeHeight / c.TileHeight))
	topbot := int((c.CubeDepth * 2 / c.TileWidth) * (c.CubeWidth / c.TileHeight))
	back := int((c.CubeHeight / c.TileHeight) * (c.CubeWidth / c.TileWidth))

	tiles := make([]Tilelayout, leftRight+topbot+back)

	// calculate the uv map steps in each direction

	uStep := c.TileWidth / (c.CubeWidth + c.CubeDepth*2)
	vStep := c.TileHeight / (c.CubeDepth*2 + c.CubeHeight)

	// plane keeps the information for
	// each plane of the cube that is created.
	// This is to try to create a more efficient loop
	type plane struct {
		// Tile step values
		iEnd, jEnd     float64 // i and j are substitutes for the 2 dimensions
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

	// set all the planes of the cube
	planes := []plane{
		// left wall
		{iEnd: c.CubeDepth, jEnd: c.CubeHeight, iStep: c.TileWidth, jStep: c.TileHeight, planeConst: 0, plane: "y", vStart: (c.CubeDepth / c.TileHeight) * vStep, inverse: true, uStart: ((c.CubeDepth + c.CubeWidth) / c.TileWidth) * uStep},
		// right wall
		{iEnd: c.CubeDepth, jEnd: c.CubeHeight, iStep: c.TileWidth, jStep: c.TileHeight, planeConst: c.CubeWidth, plane: "y", vStart: (c.CubeDepth / c.TileHeight) * vStep},

		// back wall
		{iEnd: c.CubeWidth, jEnd: c.CubeHeight, iStep: c.TileWidth, jStep: c.TileHeight, planeConst: c.CubeDepth, plane: "x", vStart: (c.CubeDepth / c.TileHeight) * vStep, uStart: ((c.CubeDepth) / c.TileWidth) * uStep},

		// Top
		{iEnd: c.CubeDepth, inverse: true, jEnd: c.CubeWidth, iStep: c.TileWidth, jStep: c.TileHeight, planeConst: c.CubeHeight, plane: "z", vStart: ((c.CubeDepth + c.CubeHeight) / c.TileHeight) * vStep, uStart: ((c.CubeDepth) / c.TileWidth) * uStep},

		// Bottom
		{iEnd: c.CubeDepth, jEnd: c.CubeWidth, iStep: c.TileWidth, jStep: c.TileHeight, planeConst: 0, plane: "z", uStart: ((c.CubeDepth) / c.TileWidth) * uStep},
	}

	// vertexCount the vertexes per face
	vertexCount := 1
	tileCount := 0

	for _, p := range planes {

		// get the end points of the u and v traversing
		uTotal := (p.iEnd - p.iStart) / p.iStep
		width := uTotal * uStep

		ujTotal := (p.jEnd - p.jStart) / p.jStep
		ujwidth := ujTotal * uStep

		iCount := 0
		tileFace := ""

		for i := p.iStart; i < p.iEnd; i += p.iStep {

			jCount := 0
			for j := p.jStart; j < p.jEnd; j += p.jStep {

				switch p.plane {
				case "x":

					// do vertex coordinates
					tileFace += fmt.Sprintf("v %v %v %v \n", p.planeConst, i, j)
					tileFace += fmt.Sprintf("v %v %v %v \n", p.planeConst, i+p.iStep, j)
					tileFace += fmt.Sprintf("v %v %v %v \n", p.planeConst, i+p.iStep, j+p.jStep)
					tileFace += fmt.Sprintf("v %v %v %v \n", p.planeConst, i, j+p.jStep)

					// do texture coordinates
					tileFace += fmt.Sprintf("vt %v %v \n", p.uStart+width-float64(iCount)*uStep, p.vStart+float64(jCount)*vStep)
					tileFace += fmt.Sprintf("vt %v %v \n", p.uStart+width-float64(iCount+1)*uStep, p.vStart+float64(jCount)*vStep)
					tileFace += fmt.Sprintf("vt %v %v \n", p.uStart+width-float64(iCount+1)*uStep, p.vStart+float64(jCount+1)*vStep)
					tileFace += fmt.Sprintf("vt %v %v \n", p.uStart+width-float64(iCount)*uStep, p.vStart+float64(jCount+1)*vStep)

					tiles[tileCount] = Tilelayout{Layout: Positions{Flat: XY{X: int(math.Round((p.uStart + width - float64(iCount)*uStep) * pixelWidth)), Y: int(math.Round((1 - (p.vStart + float64(jCount+1)*vStep)) * pixelHeight))}, Size: XY{X: int(c.Dx), Y: int(c.Dy)}}}

				case "y":

					// do vertex coordinates
					tileFace += fmt.Sprintf("v %v %v %v \n", i, p.planeConst, j)
					tileFace += fmt.Sprintf("v %v %v %v \n", i+p.iStep, p.planeConst, j)
					tileFace += fmt.Sprintf("v %v %v %v \n", i+p.iStep, p.planeConst, j+p.jStep)
					tileFace += fmt.Sprintf("v %v %v %v \n", i, p.planeConst, j+p.jStep)

					// if inversed change the direction of the uv map
					if p.inverse {
						tileFace += fmt.Sprintf("vt %v %v \n", p.uStart+width-float64(iCount)*uStep, p.vStart+float64(jCount)*vStep)
						tileFace += fmt.Sprintf("vt %v %v \n", p.uStart+width-float64(iCount+1)*uStep, p.vStart+float64(jCount)*vStep)
						tileFace += fmt.Sprintf("vt %v %v \n", p.uStart+width-float64(iCount+1)*uStep, p.vStart+float64(jCount+1)*vStep)
						tileFace += fmt.Sprintf("vt %v %v \n", p.uStart+width-float64(iCount)*uStep, p.vStart+float64(jCount+1)*vStep)

					} else {

						tileFace += fmt.Sprintf("vt %v %v \n", p.uStart+float64(iCount)*uStep, p.vStart+float64(jCount)*vStep)
						tileFace += fmt.Sprintf("vt %v %v \n", p.uStart+float64(iCount+1)*uStep, p.vStart+float64(jCount)*vStep)
						tileFace += fmt.Sprintf("vt %v %v \n", p.uStart+float64(iCount+1)*uStep, p.vStart+float64(jCount+1)*vStep)
						tileFace += fmt.Sprintf("vt %v %v \n", p.uStart+float64(iCount)*uStep, p.vStart+float64(jCount+1)*vStep)
					}

					tiles[tileCount] = Tilelayout{Layout: Positions{Flat: XY{X: int(math.Round((p.uStart + width - float64(iCount)*uStep) * pixelWidth)), Y: int(math.Round((1 - (p.vStart + float64(jCount+1)*vStep)) * pixelHeight))}, Size: XY{X: int(c.Dx), Y: int(c.Dy)}}}

				case "z":
					tileFace += fmt.Sprintf("v %v %v %v \n", i, j+p.jStep, p.planeConst)
					tileFace += fmt.Sprintf("v %v %v %v \n", i, j, p.planeConst)
					tileFace += fmt.Sprintf("v %v %v %v \n", i+p.iStep, j, p.planeConst)
					tileFace += fmt.Sprintf("v %v %v %v \n", i+p.iStep, j+p.jStep, p.planeConst)

					if p.inverse {
						// write the uv map from the top down instead of the bottom up
						tileFace += fmt.Sprintf("vt %v %v \n", p.uStart+ujwidth-float64(jCount+1)*uStep, p.vStart+(uTotal-float64(iCount))*vStep)
						tileFace += fmt.Sprintf("vt %v %v \n", p.uStart+ujwidth-float64(jCount)*uStep, p.vStart+(uTotal-float64(iCount))*vStep)
						tileFace += fmt.Sprintf("vt %v %v \n", p.uStart+ujwidth-float64(jCount)*uStep, p.vStart+(uTotal-float64(iCount+1))*vStep)
						tileFace += fmt.Sprintf("vt %v %v \n", p.uStart+ujwidth-float64(jCount+1)*uStep, p.vStart+(uTotal-float64(iCount+1))*vStep)

						tiles[tileCount] = Tilelayout{Layout: Positions{Flat: XY{X: int(math.Round((p.uStart + ujwidth - float64(jCount)*uStep) * pixelWidth)), Y: int(math.Round((1 - (p.vStart + (uTotal-float64(iCount))*vStep)) * pixelHeight))}, Size: XY{X: int(c.Dx), Y: int(c.Dy)}}}
					} else {
						tileFace += fmt.Sprintf("vt %v %v \n", p.uStart+ujwidth-float64(jCount+1)*uStep, p.vStart+float64(iCount)*vStep)
						tileFace += fmt.Sprintf("vt %v %v \n", p.uStart+ujwidth-float64(jCount)*uStep, p.vStart+float64(iCount)*vStep)
						tileFace += fmt.Sprintf("vt %v %v \n", p.uStart+ujwidth-float64(jCount)*uStep, p.vStart+float64(iCount+1)*vStep)
						tileFace += fmt.Sprintf("vt %v %v \n", p.uStart+ujwidth-float64(jCount+1)*uStep, p.vStart+float64(iCount+1)*vStep)

						tiles[tileCount] = Tilelayout{Layout: Positions{Flat: XY{X: int(math.Round((p.uStart + ujwidth - float64(jCount)*uStep) * pixelWidth)), Y: int(math.Round((1 - (p.vStart + float64(iCount+1)*vStep)) * pixelHeight))}, Size: XY{X: int(c.Dx), Y: int(c.Dy)}}}
					}

				default:
					// continue without writing for default plans
					continue
				}

				// write the face after each tile
				tileFace += fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", vertexCount, vertexCount, vertexCount+1, vertexCount+1, vertexCount+2, vertexCount+2, vertexCount+3, vertexCount+3)
				vertexCount += 4
				jCount++
				tileCount++
			}
			iCount++
		}

		_, err := wObj.Write([]byte(tileFace))
		if err != nil {
			return fmt.Errorf("error writing to obj %v", err)
		}
	}

	tsig := TPIG{Tilelayout: tiles, Dimensions: Dimensions{Flat: XY2D{X0: 0, X1: int(pixelWidth), Y0: 0, Y1: int(pixelHeight)}}}

	enc := json.NewEncoder(wTsig)
	enc.SetIndent("", "    ")

	return enc.Encode(tsig)

}

func halfCubeFence(tileHeight, tileWidth float64, CubeWidth, CubeHeight, CubeDepth float64) error {

	// check the dimensions
	if int(math.Ceil(CubeWidth/tileWidth)) != int(CubeWidth/tileWidth) {
		return fmt.Errorf("tile width of %v is not an integer multiple of a cube width of %v", tileWidth, CubeWidth)
	}

	if int(math.Ceil(CubeHeight/tileHeight)) != int(CubeHeight/tileHeight) {
		return fmt.Errorf("tile height of %v is not an integer multiple of a cube height of %v", tileHeight, CubeHeight)
	}

	if int(math.Ceil(CubeDepth/tileWidth)) != int(CubeDepth/tileWidth) {
		return fmt.Errorf("tile width of %v is not an integer multiple of a cube depth of %v", tileWidth, CubeDepth)
	}

	if int(math.Ceil(CubeDepth/tileHeight)) != int(CubeDepth/tileHeight) {
		return fmt.Errorf("tile height of %v is not an integer multiple of a cube depth of %v", tileHeight, CubeHeight)
	}

	return nil
}

/*
GenCurveOBJ generates a TSIG and OBJ for a curved cylindrical wall.
The wall is centred around 0,0,0

Angles are in **Radians**
*/
func (c Curve) generate(wObj, wTsig io.Writer) error {

	// get the total angle covered by the cylinder.
	azimuthInc := (2 * math.Asin(c.TileWidth/(2*c.CurveRadius)))

	z := 0.0
	vertexCount := 1
	azimuth := -c.AzimuthMaxAngle

	pixelWidth := math.Ceil(2*c.AzimuthMaxAngle/azimuthInc) * c.Dx
	pixelHeight := math.Ceil(c.CurveHeight/c.TileHeight) * c.Dy

	// rows * column for the total expected tile count
	tiles := make([]Tilelayout, int(math.Ceil(2*c.AzimuthMaxAngle/azimuthInc)*math.Ceil(c.CurveHeight/c.TileHeight)))

	uWidth := 1 / (math.Ceil(2 * c.AzimuthMaxAngle / azimuthInc))
	vheight := 1 / math.Ceil(c.CurveHeight/c.TileHeight)
	v := 0.0

	tileCount := 0
	for z < c.CurveHeight {
		u := 1.0

		tileFaces := ""
		for azimuth <= c.AzimuthMaxAngle {
			//	tileCount++

			// get angle change

			x1, y1, z1 := CylindricalToCartesian(c.CurveRadius, z, azimuth)
			tileFaces += fmt.Sprintf("v %v %v %v \n", x1, y1, z1)
			tileFaces += fmt.Sprintf("vt %v %v \n", u, v)

			x2, y2, z2 := CylindricalToCartesian(c.CurveRadius, z, azimuth+azimuthInc) // increase azimuth
			tileFaces += fmt.Sprintf("v %v %v %v \n", x2, y2, z2)
			tileFaces += fmt.Sprintf("vt %v %v \n", u-uWidth, v)

			x3, y3, z3 := CylindricalToCartesian(c.CurveRadius, z+c.TileHeight, azimuth+azimuthInc) // increase azimuth and height
			tileFaces += fmt.Sprintf("v %v %v %v \n", x3, y3, z3)
			tileFaces += fmt.Sprintf("vt %v %v \n", u-uWidth, v+vheight)

			x4, y4, z4 := CylindricalToCartesian(c.CurveRadius, z+c.TileHeight, azimuth) // increase height
			tileFaces += fmt.Sprintf("v %v %v %v \n", x4, y4, z4)
			tileFaces += fmt.Sprintf("vt %v %v \n", u, v+vheight)

			tileFaces += fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", vertexCount, vertexCount, vertexCount+1, vertexCount+1, vertexCount+2, vertexCount+2, vertexCount+3, vertexCount+3)

			azimuth += azimuthInc
			u -= uWidth
			vertexCount += 4

			tiles[tileCount] = Tilelayout{Layout: Positions{Flat: XY{X: int(math.Round(u * pixelWidth)), Y: int(math.Round((1 - (v + vheight)) * pixelHeight))}, Size: XY{X: int(c.Dx), Y: int(c.Dy)}}}

			tileCount++
		}

		_, err := wObj.Write([]byte(tileFaces))
		if err != nil {
			return fmt.Errorf("error writing to obj %v", err)
		}

		// increase the z height
		// as well as the uv map height
		v += vheight
		z += c.TileHeight
		azimuth = -c.AzimuthMaxAngle
	}

	tsig := TPIG{Tilelayout: tiles, Dimensions: Dimensions{Flat: XY2D{X0: 0, X1: int(pixelWidth), Y0: 0, Y1: int(pixelHeight)}}}

	enc := json.NewEncoder(wTsig)
	enc.SetIndent("", "    ")
	return enc.Encode(tsig)

}

/*
GenSphereOBJSquare generates a sphere made of tiles of size height and width.

This works by splitting each row of pixels into their own tile, so the uv map matches exactly.

All angles are in radians
*/
func (s Sphere) generate(wObj, wTsig io.Writer) error {

	// get the start point
	azimuth, clockAz := 0.0, 0.0
	// tileCount := 0
	theta := math.Pi / 2
	vertexCount := 1

	thetaInc := 2 * (math.Asin(s.TileHeight / (2 * s.Radius)))
	azimuthInc := (2 * math.Asin(s.TileWidth/(2*s.Radius)))
	theta = (math.Pi / 2)

	//
	uTileWidth := 1 / (2 * math.Ceil(s.AzimuthMaxAngle/azimuthInc))
	vTileHeight := 1 / (2 * math.Ceil(s.ThetaMaxAngle/thetaInc))

	maxX := 2 * math.Ceil(s.AzimuthMaxAngle/azimuthInc) * s.Dx
	maxY := 2 * math.Ceil(s.ThetaMaxAngle/thetaInc) * s.Dy

	//
	overrun := 1.0
	for theta > (math.Pi/2)-s.ThetaMaxAngle {
		topLeftTheta := theta - thetaInc
		azimuth := 0.0

		azimuthInc := (2 * math.Asin(s.TileWidth/(2*s.Radius))) / math.Sin(theta)
		azimuthIncTop := (2 * math.Asin(s.TileWidth/(2*s.Radius))) / math.Sin(topLeftTheta)

		// futDif is the length chordal length difference of the azimuth change on the bottom row.
		// which is the closest current approximation
		futDif := 2 * s.Radius * (math.Sin((azimuthIncTop-azimuthInc)/2) * math.Sin(theta))

		// find the difference in pixels at the bottom and at the top
		shift := int((futDif)/(s.TileWidth/s.Dx)) / 2
		ushift := (float64(shift)) * (1.0 / float64(maxX))

		//fmt.Println(futDif, 2*sphereRadius*(math.Sin((azimuthIncTop-azimuthInc)/2)*math.Sin(theta)), shift)
		uTop := 0.5
		for azimuth < s.AzimuthMaxAngle {
			azimuth += azimuthIncTop

			uTop += uTileWidth + (ushift * 2)

		}

		if uTop > overrun {
			overrun = uTop
		}

		theta -= thetaInc
	}

	theta = math.Pi / 2

	xInc := 0.0
	if overrun > 1.0 {
		xInc = math.Round((overrun - 1) * maxX)
	}

	// update the parameters to account for the overrun
	// of tiles
	maxX = maxX + xInc*2
	uTileWidth = s.Dx / maxX

	tiles := []Tilelayout{}

	// -33 is the lowest
	//sizex := 3840
	/*
	   generate the size of the bas eimage then move erveything along
	*/
	// TOP
	v := 0.5

	for theta > (math.Pi/2)-s.ThetaMaxAngle {
		//start Point :=
		topLeftThet := theta - thetaInc
		topLeftAz := azimuth
		// botLeftAz := azimuth

		u := 0.5
		uBot := 0.5
		//		prevUshift := 0.0

		azimuthInc := (2 * math.Asin(s.TileWidth/(2*s.Radius))) / math.Sin(theta)
		azimuthIncTop := (2 * math.Asin(s.TileWidth/(2*s.Radius))) / math.Sin(topLeftThet)

		// futDif is the length chordal length difference of the azimuth change on the bottom row.
		// which is the closest current approximation
		futDif := 2 * s.Radius * (math.Sin((azimuthIncTop-azimuthInc)/2) * math.Sin(theta))

		// find the difference in pixels
		shift := int((futDif)/(s.TileWidth/s.Dx)) / 2

		//fmt.Println(futDif, 2*sphereRadius*(math.Sin((azimuthIncTop-azimuthInc)/2)*math.Sin(theta)), shift)
		radialInc := 0

		for azimuth < s.AzimuthMaxAngle {
			//	tileCount++

			/*
				each shift is increased by the count of shift
				so second row goes 1 + 1 + 1
				row below is 2 + 2 + 2 etc
				row below is 3 + 3 + 3
			*/

			x1, y1, z1 := PolarToCartesian(s.Radius, topLeftThet+thetaInc, topLeftAz)
			x2, y2, z2 := PolarToCartesian(s.Radius, topLeftThet+thetaInc, topLeftAz+azimuthInc) // increase azimuth
			x3, y3, z3 := PolarToCartesian(s.Radius, topLeftThet, topLeftAz+azimuthIncTop)       // increase azimuth and height
			x4, y4, z4 := PolarToCartesian(s.Radius, topLeftThet, topLeftAz)                     // increase height to the bottom

			// for each drop of a pixel shift that row along one
			// to that the uv map that is created is square and can be made a tsig.
			// @TODO update so each drop is two pixels and is a pixel eitherway

			step := int(s.Dy / float64((shift)+1))
			botX, botY, botZ := x1, y1, z1
			botRX, botRY, botRZ := x2, y2, z2

			leftVectX, leftVectY, leftVectZ := (float64(step)*(x4-x1))/s.Dy, (float64(step)*(y4-y1))/s.Dy, (float64(step)*(z4-z1))/s.Dy
			rightVectX, rightVectY, rightVectZ := (float64(step)*(x3-x2))/s.Dy, (float64(step)*(y3-y2))/s.Dy, (float64(step)*(z3-z2))/s.Dy

			//	vstep := float64(step) * (1.0 / float64(maxY))
			vstep := float64(step) / maxY //(vheight / float64(shift+1))
			ustep := (1.0 / float64(maxX))

			tileFaces := ""
			for i := 0; i < shift; i++ {

				//	fmt.Println(x1, y1, x2, z2)

				topX, topY, topZ := botX+leftVectX, botY+leftVectY, botZ+leftVectZ
				topRX, topRY, topRZ := botRX+rightVectX, botRY+rightVectY, botRZ+rightVectZ
				pos := shift - i
				offset := float64((pos))*ustep + float64(radialInc*pos)*ustep

				tileFaces += fmt.Sprintf("v %v %v %v\n", botX, botY, botZ)
				tileFaces += fmt.Sprintf("vt %v %v \n", 1-(uBot+offset), v+(float64(i)*vstep))

				tileFaces += fmt.Sprintf("v %v %v %v \n", botRX, botRY, botRZ)
				tileFaces += fmt.Sprintf("vt %v %v \n", 1-(uTileWidth+uBot+offset), v+(float64(i)*vstep))

				tileFaces += fmt.Sprintf("v %v %v %v \n", topRX, topRY, topRZ)
				tileFaces += fmt.Sprintf("vt %v %v \n", 1-(uTileWidth+uBot+offset), v+(float64(i+1)*vstep))

				tileFaces += fmt.Sprintf("v %v %v %v \n", topX, topY, topZ)
				tileFaces += fmt.Sprintf("vt %v %v \n", 1-(uBot+offset), v+(float64(i+1)*vstep))
				tileFaces += fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", vertexCount, vertexCount, vertexCount+1, vertexCount+1, vertexCount+2, vertexCount+2, vertexCount+3, vertexCount+3)

				tiles = append(tiles, Tilelayout{Layout: Positions{
					Flat: XY{X: int((1 - (uBot + uTileWidth + offset)) * maxX), Y: int(math.Round((1 - (v + (float64(i+1) * vstep))) * maxY))},
					Size: XY{X: int(s.Dx), Y: int(maxY * vstep)}}})

				botX, botY, botZ = topX, topY, topZ
				botRX, botRY, botRZ = topRX, topRY, topRZ
				vertexCount += 4
			}

			// the max v picks off from the last one to accoount for rounding errors
			tileFaces += fmt.Sprintf("v %v %v %v \n", botX, botY, botZ)
			tileFaces += fmt.Sprintf("vt %v %v \n", 1-(uBot), v+(vstep*(float64(shift))))

			tileFaces += fmt.Sprintf("v %v %v %v \n", botRX, botRY, botRZ)
			tileFaces += fmt.Sprintf("vt %v %v \n", 1-(uTileWidth+uBot), v+(vstep*(float64(shift))))

			tileFaces += fmt.Sprintf("v %v %v %v \n", x3, y3, z3)
			tileFaces += fmt.Sprintf("vt %v %v \n", 1-(uTileWidth+uBot), v+vTileHeight)

			tileFaces += fmt.Sprintf("v %v %v %v \n", x4, y4, z4)
			tileFaces += fmt.Sprintf("vt %v %v \n", 1-(uBot), v+vTileHeight)

			tiles = append(tiles, Tilelayout{Layout: Positions{
				Flat: XY{X: int((1 - (uBot + uTileWidth)) * maxX), Y: int(math.Round((1 - (v + vTileHeight)) * maxY))},
				Size: XY{X: int(s.Dx), Y: int(math.Round(maxY * (vTileHeight - vstep*(float64(shift)))))}}})

			// radialInc++

			tileFaces += fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", vertexCount, vertexCount, vertexCount+1, vertexCount+1, vertexCount+2, vertexCount+2, vertexCount+3, vertexCount+3)

			_, err := wObj.Write([]byte(tileFaces))
			if err != nil {
				return fmt.Errorf("error writing to obj %v", err)
			}

			// nlX, nlY, nlZ := PolarToCartesian(sphereRadius, topLeftThet+thetaInc, topLeftAz+azimuthIncTop)

			//			fmt.Println("4", 1-(u), "3", 1-(u+uWidth))
			//			fmt.Println("shift", ushift, prevUshift)
			//		botLeftAz = topLeftAz + azimuthInc
			uBot += uTileWidth
			azimuth += azimuthIncTop
			topLeftAz = azimuth
			vertexCount += 4
			u += uTileWidth
			radialInc += 2

		}

		topRightAz := clockAz
		u = 0.5
		uBot = 0.5

		radialInc = 0
		for clockAz > -s.ThetaMaxAngle {

			x1, y1, z1 := PolarToCartesian(s.Radius, topLeftThet+thetaInc, topRightAz)
			x2, y2, z2 := PolarToCartesian(s.Radius, topLeftThet+thetaInc, topRightAz-azimuthInc)
			x3, y3, z3 := PolarToCartesian(s.Radius, topLeftThet, topRightAz-azimuthIncTop)
			x4, y4, z4 := PolarToCartesian(s.Radius, topLeftThet, topRightAz)

			step := int(s.Dy / float64(shift+1))
			botX, botY, botZ := x1, y1, z1
			botRX, botRY, botRZ := x2, y2, z2

			leftVectX, leftVectY, leftVectZ := (float64(step)*(x4-x1))/s.Dy, (float64(step)*(y4-y1))/s.Dy, (float64(step)*(z4-z1))/s.Dy
			rightVectX, rightVectY, rightVectZ := (float64(step)*(x3-x2))/s.Dy, (float64(step)*(y3-y2))/s.Dy, (float64(step)*(z3-z2))/s.Dy

			//	vstep := float64(step) * (1.0 / float64(maxY))
			vstep := float64(step) / maxY //(vheight / float64(shift+1))
			ustep := (1.0 / float64(maxX))
			tileFaces := ""
			for i := 0; i < shift; i++ {

				//	fmt.Println(x1, y1, x2, z2)
				//////////////TARGET//////////////

				topX, topY, topZ := botX+leftVectX, botY+leftVectY, botZ+leftVectZ
				topRX, topRY, topRZ := botRX+rightVectX, botRY+rightVectY, botRZ+rightVectZ
				pos := shift - i
				stepOffset := -float64((pos))*ustep - float64(radialInc*pos)*ustep

				tileFaces += fmt.Sprintf("v %v %v %v \n", botX, botY, botZ)
				tileFaces += fmt.Sprintf("vt %v %v \n", 1-(uBot+stepOffset), v+(float64(i)*vstep))

				tileFaces += fmt.Sprintf("v %v %v %v \n", botRX, botRY, botRZ)
				tileFaces += fmt.Sprintf("vt %v %v \n", 1-(-uTileWidth+uBot+stepOffset), v+(float64(i)*vstep))

				tileFaces += fmt.Sprintf("v %v %v %v \n", topRX, topRY, topRZ)
				tileFaces += fmt.Sprintf("vt %v %v \n", 1-(-uTileWidth+uBot+stepOffset), v+(float64(i+1)*vstep))

				tileFaces += fmt.Sprintf("v %v %v %v \n", topX, topY, topZ)
				tileFaces += fmt.Sprintf("vt %v %v \n", 1-(uBot+stepOffset), v+(float64(i+1)*vstep))
				tileFaces += fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", vertexCount, vertexCount, vertexCount+1, vertexCount+1, vertexCount+2, vertexCount+2, vertexCount+3, vertexCount+3)

				tiles = append(tiles, Tilelayout{Layout: Positions{
					Flat: XY{X: int((1 - (uBot + stepOffset)) * maxX), Y: int(math.Round((1 - (v + (float64(i+1) * vstep))) * maxY))},
					Size: XY{X: int(s.Dx), Y: int(maxY * vstep)}}})

				botX, botY, botZ = topX, topY, topZ
				botRX, botRY, botRZ = topRX, topRY, topRZ
				vertexCount += 4
			}

			// write the final tile, which may be the only one
			tileFaces += fmt.Sprintf("v %v %v %v \n", botX, botY, botZ)
			tileFaces += fmt.Sprintf("vt %v %v \n", 1-(uBot), v+(vstep*(float64(shift))))

			tileFaces += fmt.Sprintf("v %v %v %v \n", botRX, botRY, botRZ)
			tileFaces += fmt.Sprintf("vt %v %v \n", 1-(-uTileWidth+uBot), v+(vstep*(float64(shift))))

			tileFaces += fmt.Sprintf("v %v %v %v \n", x3, y3, z3)
			tileFaces += fmt.Sprintf("vt %v %v \n", 1-(-uTileWidth+uBot), v+vTileHeight)

			tileFaces += fmt.Sprintf("v %v %v %v \n", x4, y4, z4)
			tileFaces += fmt.Sprintf("vt %v %v \n", 1-(uBot), v+vTileHeight)

			tiles = append(tiles, Tilelayout{Layout: Positions{
				Flat: XY{X: int((1 - uBot) * maxX), Y: int(math.Round((1 - (v + vTileHeight)) * maxY))},
				Size: XY{X: int(s.Dx), Y: int(math.Round(maxY * (vTileHeight - vstep*(float64(shift)))))}}})

			tileFaces += fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", vertexCount, vertexCount, vertexCount+1, vertexCount+1, vertexCount+2, vertexCount+2, vertexCount+3, vertexCount+3)

			_, err := wObj.Write([]byte(tileFaces))
			if err != nil {
				return fmt.Errorf("error writing to obj %v", err)
			}
			//	objbuf.WriteString(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3))
			clockAz -= azimuthIncTop
			topRightAz = clockAz
			vertexCount += 4
			u -= uTileWidth
			radialInc += 2
			uBot -= (uTileWidth)

		}

		theta -= thetaInc
		azimuth = 0
		clockAz = 0
		v += vTileHeight
		//fmt.Println("COINTER", theta, z, zinchold)
		//	z = zinchold

	}

	// Bottom
	v = 0.5
	theta = math.Pi / 2
	for theta < (math.Pi/2)+s.ThetaMaxAngle {
		//start Point :=
		botLeftThet := theta + thetaInc
		botLeftAz := azimuth
		u := 0.5
		uTop := 0.5

		azimuthInc := (2 * math.Asin(s.TileWidth/(2*s.Radius))) / math.Sin(theta)
		azimuthIncBot := (2 * math.Asin(s.TileWidth/(2*s.Radius))) / math.Sin(botLeftThet)

		futDif := 2 * s.Radius * (math.Sin((azimuthIncBot-azimuthInc)/2) * math.Sin(botLeftThet-thetaInc))

		shift := int((futDif / 2) / (s.TileWidth / s.Dx))

		radialInc := 0

		for azimuth < s.ThetaMaxAngle {

			// tileCount++
			x1, y1, z1 := PolarToCartesian(s.Radius, botLeftThet-thetaInc, botLeftAz)
			x2, y2, z2 := PolarToCartesian(s.Radius, botLeftThet-thetaInc, botLeftAz+azimuthInc) // increase azimuth
			x3, y3, z3 := PolarToCartesian(s.Radius, botLeftThet, botLeftAz+azimuthIncBot)       // increase azimuth and height
			x4, y4, z4 := PolarToCartesian(s.Radius, botLeftThet, botLeftAz)                     // increase height

			step := int(s.Dy / float64(shift+1))
			topX, topY, topZ := x1, y1, z1
			topRX, topRY, topRZ := x2, y2, z2

			leftVectX, leftVectY, leftVectZ := (float64(step)*(x4-x1))/s.Dy, (float64(step)*(y4-y1))/s.Dy, (float64(step)*(z4-z1))/s.Dy
			rightVectX, rightVectY, rightVectZ := (float64(step)*(x3-x2))/s.Dy, (float64(step)*(y3-y2))/s.Dy, (float64(step)*(z3-z2))/s.Dy

			//	vstep := float64(step) * (1.0 / float64(maxY))
			vstep := float64(step) / maxY // (vheight / float64(shift+1))
			ustep := (1.0 / float64(maxX))

			tileFaces := ""
			for i := 0; i < shift; i++ {

				botX, botY, botZ := topX+leftVectX, topY+leftVectY, topZ+leftVectZ
				botRX, botRY, botRZ := topRX+rightVectX, topRY+rightVectY, topRZ+rightVectZ
				pos := shift - i

				tileFaces += fmt.Sprintf("v %v %v %v \n", topX, topY, topZ)
				tileFaces += fmt.Sprintf("vt %v %v \n", 1-(uTop+float64((pos))*ustep+float64(radialInc*pos)*ustep), v-float64(i)*vstep)

				tileFaces += fmt.Sprintf("v %v %v %v \n", topRX, topRY, topRZ)
				tileFaces += fmt.Sprintf("vt %v %v \n", 1-(uTop+uTileWidth+float64((pos))*ustep+float64(radialInc*pos)*ustep), v-float64(i)*vstep)

				tileFaces += fmt.Sprintf("v %v %v %v \n", botRX, botRY, botRZ)
				tileFaces += fmt.Sprintf("vt %v %v \n", 1-(uTop+uTileWidth+float64((pos))*ustep+float64(radialInc*pos)*ustep), v-float64(i+1)*vstep)

				tileFaces += fmt.Sprintf("v %v %v %v \n", botX, botY, botZ)
				tileFaces += fmt.Sprintf("vt %v %v \n", 1-(uTop+float64((pos))*ustep+float64(radialInc*pos)*ustep), v-float64(i+1)*vstep)
				tileFaces += fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", vertexCount, vertexCount, vertexCount+1, vertexCount+1, vertexCount+2, vertexCount+2, vertexCount+3, vertexCount+3)

				tiles = append(tiles, Tilelayout{Layout: Positions{
					Flat: XY{X: int((1 - (uTop + uTileWidth + float64((pos))*ustep + float64(radialInc*pos)*ustep)) * maxX), Y: int(math.Round((1 - (v - (float64(i) * vstep))) * maxY))},
					Size: XY{X: int(s.Dx), Y: int(maxY * vstep)}}})

				topX, topY, topZ = botX, botY, botZ
				topRX, topRY, topRZ = botRX, botRY, botRZ
				vertexCount += 4
			}

			tileFaces += fmt.Sprintf("v %v %v %v \n", topX, topY, topZ)
			tileFaces += fmt.Sprintf("vt %v %v \n", 1-(uTop), v-float64(shift)*vstep)

			tileFaces += fmt.Sprintf("v %v %v %v \n", topRX, topRY, topRZ)
			tileFaces += fmt.Sprintf("vt %v %v \n", 1-(uTop+uTileWidth), v-float64(shift)*vstep)

			tileFaces += fmt.Sprintf("v %v %v %v \n", x3, y3, z3)
			tileFaces += fmt.Sprintf("vt %v %v \n", 1-(uTop+uTileWidth), v-vTileHeight)

			tileFaces += fmt.Sprintf("v %v %v %v \n", x4, y4, z4)
			tileFaces += fmt.Sprintf("vt %v %v \n", 1-(uTop), v-vTileHeight)

			tiles = append(tiles, Tilelayout{Layout: Positions{
				Flat: XY{X: int((1 - (uTop + uTileWidth)) * maxX), Y: int(math.Round((1 - (v - float64(shift)*vstep)) * maxY))},
				Size: XY{X: int(s.Dx), Y: int(math.Round(maxY * (vTileHeight - vstep*(float64(shift)))))}}})

			//	fmt.Println(math.Sqrt(math.Pow((x2)-x1, 2)+math.Pow((y2)-y1, 2)) + math.Pow((z2)-z1, 2))

			tileFaces += fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", vertexCount, vertexCount, vertexCount+1, vertexCount+1, vertexCount+2, vertexCount+2, vertexCount+3, vertexCount+3)

			_, err := wObj.Write([]byte(tileFaces))
			if err != nil {
				return fmt.Errorf("error writing to obj %v", err)
			}

			//	objbuf.WriteString(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3))
			azimuth += azimuthIncBot
			botLeftAz = azimuth
			vertexCount += 4
			u += uTileWidth
			// uTop += uWidth + (ushift * 2)'
			uTop += uTileWidth //+ ushift
			radialInc += 2
		}

		botRightAz := clockAz
		u = 0.5
		uTop = 0.5
		radialInc = 0
		for clockAz > -s.ThetaMaxAngle {

			azimuthInc := (2 * math.Asin(s.TileWidth/(2*s.Radius))) / math.Sin(theta)
			azimuthIncTop := (2 * math.Asin(s.TileWidth/(2*s.Radius))) / math.Sin(botLeftThet)
			x1, y1, z1 := PolarToCartesian(s.Radius, botLeftThet-thetaInc, botRightAz)
			x2, y2, z2 := PolarToCartesian(s.Radius, botLeftThet-thetaInc, botRightAz-azimuthInc) // increase azimuth
			x3, y3, z3 := PolarToCartesian(s.Radius, botLeftThet, botRightAz-azimuthIncTop)       // increase azimuth and height
			x4, y4, z4 := PolarToCartesian(s.Radius, botLeftThet, botRightAz)                     // increase height to the bottom

			step := int(s.Dy / float64(shift+1))
			topX, topY, topZ := x1, y1, z1
			topRX, topRY, topRZ := x2, y2, z2

			leftVectX, leftVectY, leftVectZ := (float64(step)*(x4-x1))/s.Dy, (float64(step)*(y4-y1))/s.Dy, (float64(step)*(z4-z1))/s.Dy
			rightVectX, rightVectY, rightVectZ := (float64(step)*(x3-x2))/s.Dy, (float64(step)*(y3-y2))/s.Dy, (float64(step)*(z3-z2))/s.Dy

			//	vstep := float64(step) * (1.0 / float64(maxY))
			vstep := float64(step) / maxY // (vheight / float64(shift+1))
			ustep := (-1.0 / float64(maxX))

			tileFaces := ""
			for i := 0; i < shift; i++ {

				//	fmt.Println(x1, y1, x2, z2)

				botX, botY, botZ := topX+leftVectX, topY+leftVectY, topZ+leftVectZ
				botRX, botRY, botRZ := topRX+rightVectX, topRY+rightVectY, topRZ+rightVectZ
				pos := shift - i

				tileFaces += fmt.Sprintf("v %v %v %v \n", topX, topY, topZ)
				tileFaces += fmt.Sprintf("vt %v %v \n", 1-(uTop+float64((pos))*ustep+float64(radialInc*pos)*ustep), v-float64(i)*vstep)

				tileFaces += fmt.Sprintf("v %v %v %v \n", topRX, topRY, topRZ)
				tileFaces += fmt.Sprintf("vt %v %v \n", 1-(uTop-uTileWidth+float64((pos))*ustep+float64(radialInc*pos)*ustep), v-float64(i)*vstep)

				tileFaces += fmt.Sprintf("v %v %v %v \n", botRX, botRY, botRZ)
				tileFaces += fmt.Sprintf("vt %v %v \n", 1-(uTop-uTileWidth+float64((pos))*ustep+float64(radialInc*pos)*ustep), v-float64(i+1)*vstep)

				tileFaces += fmt.Sprintf("v %v %v %v \n", botX, botY, botZ)
				tileFaces += fmt.Sprintf("vt %v %v \n", 1-(uTop+float64((pos))*ustep+float64(radialInc*pos)*ustep), v-float64(i+1)*vstep)
				tileFaces += fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", vertexCount, vertexCount, vertexCount+1, vertexCount+1, vertexCount+2, vertexCount+2, vertexCount+3, vertexCount+3)

				tiles = append(tiles, Tilelayout{Layout: Positions{
					Flat: XY{X: int((1 - (uTop + float64((pos))*ustep + float64(radialInc*pos)*ustep)) * maxX), Y: int(math.Round((1 - (v - (float64(i) * vstep))) * maxY))},
					Size: XY{X: int(s.Dx), Y: int(maxY * vstep)}}})

				topX, topY, topZ = botX, botY, botZ
				topRX, topRY, topRZ = botRX, botRY, botRZ
				vertexCount += 4
			}

			tileFaces += fmt.Sprintf("v %v %v %v \n", topX, topY, topZ)
			tileFaces += fmt.Sprintf("vt %v %v \n", 1-(uTop), v-float64(shift)*vstep)

			tileFaces += fmt.Sprintf("v %v %v %v \n", topRX, topRY, topRZ)
			tileFaces += fmt.Sprintf("vt %v %v \n", 1-(uTop-uTileWidth), v-float64(shift)*vstep)

			tileFaces += fmt.Sprintf("v %v %v %v \n", x3, y3, z3)
			tileFaces += fmt.Sprintf("vt %v %v \n", 1-(uTop-uTileWidth), v-vTileHeight)

			tileFaces += fmt.Sprintf("v %v %v %v \n", x4, y4, z4)
			tileFaces += fmt.Sprintf("vt %v %v \n", 1-(uTop), v-vTileHeight)

			tiles = append(tiles, Tilelayout{Layout: Positions{
				Flat: XY{X: int((1 - uTop) * maxX), Y: int(math.Round((1 - (v - float64(shift)*vstep)) * maxY))},
				Size: XY{X: int(s.Dx), Y: int(math.Round(maxY * (vTileHeight - vstep*(float64(shift)))))}}})
			//	leftVectX, leftVectY, leftVectZ := (x4-x1)/dy, (y4-y1)/dy, (z4-z1)/dy
			//	rightVectX, rightVectY, rightVectZ := (x3-x2)/dy, (y3-y2)/dy, (z3-z2)/dy
			/*

				handle the u differently

				numberOfShifs := shift
			*/
			// +1 to rember the 0th line and get the correct amount of increments

			tileFaces += fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", vertexCount, vertexCount, vertexCount+1, vertexCount+1, vertexCount+2, vertexCount+2, vertexCount+3, vertexCount+3)

			_, err := wObj.Write([]byte(tileFaces))
			if err != nil {
				return fmt.Errorf("error writing to obj %v", err)
			}
			//	objbuf.WriteString(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3))
			clockAz -= azimuthIncTop
			botRightAz = clockAz
			vertexCount += 4
			radialInc += 2
			u -= uTileWidth
			uTop -= (uTileWidth) // + (ushift * 2))
		}

		v -= vTileHeight
		theta += thetaInc
		azimuth = 0
		clockAz = 0
		//fmt.Println("COINTER", theta, z, zinchold)
		//	z = zinchold

	}

	tsig := TPIG{Tilelayout: tiles, Dimensions: Dimensions{Flat: XY2D{X0: 0, X1: int(maxX), Y0: 0, Y1: int(maxY)}}}

	enc := json.NewEncoder(wTsig)
	enc.SetIndent("", "    ")

	return enc.Encode(tsig)
}

/*
GenSphereOBJ generates a sphere made of tiles of size height and width.
But with a UV map that just shows how a perfect view would loo, not a pixel perfect view.

All angles in radians
*/ /*
func GenSphereOBJ(w io.Writer, tileHeight, tileWidth float64, sphereRadius, thetaMaxAngle, azimuthMaxAngle float64, dx, dy float64) error {

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

	//tsig information

	//sizex := 3840
	/*
	   generate the size of the bas eimage then move erveything along

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
		for azimuth < azimuthMaxAngle {
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

	return nil
} */
