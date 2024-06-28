//  Copyright ©2019-2024  Mr MXF   info@mrmxf.com
//  BSD-3-Clause License           https://opensource.org/license/bsd-3-clause/
//
// Package shapes contains the obj shapes and their configurations

package shapes

import (
	"encoding/json"
	"fmt"
	"io"
	"math"

	"github.com/mrmxf/opentsg-modules/opentsg-core/gridgen"
)

// add the shape to the mian handler here
func init() {
	AddShapeToHandler[SphereCap]("", "")
}

// sphere properties
type SphereCap struct {
	TileHeight      float64 `json:"tileHeight" yaml:"tileHeight"`
	TileWidth       float64 `json:"tileWidth" yaml:"tileWidth"`
	Radius          float64 `json:"radius" yaml:"radius"`
	ThetaMaxAngle   float64 `json:"thetaMaxAngle" yaml:"thetaMaxAngle"`
	AzimuthMaxAngle float64 `json:"azimuthMaxAngle" yaml:"azimuthMaxAngle"`
	Dx              float64 `json:"dx" yaml:"dx"`
	Dy              float64 `json:"dy" yaml:"dy"`
	ShapeName
}

// Returns the name of the object
func (s SphereCap) ObjType() string {
	return "spherecap"
}

/*
GenSphereOBJSquare generates a sphere made of tiles of size height and width.

This works by splitting each row of pixels into their own tile, so the uv map matches exactly.

All angles are in radians
*/
func (s SphereCap) Generate(wObj, wTsig io.Writer) error {

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

	// calculate the overrun by looping through a segment of the
	// sphere cap and seeing if the u value exceeds the regular bounds of
	// 1. Due to the pixel shifting that happens
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

	// reset theta back to what it was
	theta = math.Pi / 2

	xInc := 0.0
	if overrun > 1.0 {
		xInc = math.Round((overrun - 1) * maxX)
	}

	// update the parameters to account for the overrun
	// of tiles, if present
	maxX = maxX + xInc*2
	uTileWidth = s.Dx / maxX

	tiles := []gridgen.Tilelayout{}

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

				tiles = append(tiles, gridgen.Tilelayout{Layout: gridgen.Positions{
					Flat: gridgen.XY{X: int((1 - (uBot + uTileWidth + offset)) * maxX), Y: int(math.Round((1 - (v + (float64(i+1) * vstep))) * maxY))},
					Size: gridgen.XY{X: int(s.Dx), Y: int(maxY * vstep)}}})

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

			tiles = append(tiles, gridgen.Tilelayout{Layout: gridgen.Positions{
				Flat: gridgen.XY{X: int((1 - (uBot + uTileWidth)) * maxX), Y: int(math.Round((1 - (v + vTileHeight)) * maxY))},
				Size: gridgen.XY{X: int(s.Dx), Y: int(math.Round(maxY * (vTileHeight - vstep*(float64(shift)))))}}})

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

				tiles = append(tiles, gridgen.Tilelayout{Layout: gridgen.Positions{
					Flat: gridgen.XY{X: int((1 - (uBot + stepOffset)) * maxX), Y: int(math.Round((1 - (v + (float64(i+1) * vstep))) * maxY))},
					Size: gridgen.XY{X: int(s.Dx), Y: int(maxY * vstep)}}})

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

			tiles = append(tiles, gridgen.Tilelayout{Layout: gridgen.Positions{
				Flat: gridgen.XY{X: int((1 - uBot) * maxX), Y: int(math.Round((1 - (v + vTileHeight)) * maxY))},
				Size: gridgen.XY{X: int(s.Dx), Y: int(math.Round(maxY * (vTileHeight - vstep*(float64(shift)))))}}})

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

				tiles = append(tiles, gridgen.Tilelayout{Layout: gridgen.Positions{
					Flat: gridgen.XY{X: int((1 - (uTop + uTileWidth + float64((pos))*ustep + float64(radialInc*pos)*ustep)) * maxX), Y: int(math.Round((1 - (v - (float64(i) * vstep))) * maxY))},
					Size: gridgen.XY{X: int(s.Dx), Y: int(maxY * vstep)}}})

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

			tiles = append(tiles, gridgen.Tilelayout{Layout: gridgen.Positions{
				Flat: gridgen.XY{X: int((1 - (uTop + uTileWidth)) * maxX), Y: int(math.Round((1 - (v - float64(shift)*vstep)) * maxY))},
				Size: gridgen.XY{X: int(s.Dx), Y: int(math.Round(maxY * (vTileHeight - vstep*(float64(shift)))))}}})

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

				tiles = append(tiles, gridgen.Tilelayout{Layout: gridgen.Positions{
					Flat: gridgen.XY{X: int((1 - (uTop + float64((pos))*ustep + float64(radialInc*pos)*ustep)) * maxX), Y: int(math.Round((1 - (v - (float64(i) * vstep))) * maxY))},
					Size: gridgen.XY{X: int(s.Dx), Y: int(maxY * vstep)}}})

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

			tiles = append(tiles, gridgen.Tilelayout{Layout: gridgen.Positions{
				Flat: gridgen.XY{X: int((1 - uTop) * maxX), Y: int(math.Round((1 - (v - float64(shift)*vstep)) * maxY))},
				Size: gridgen.XY{X: int(s.Dx), Y: int(math.Round(maxY * (vTileHeight - vstep*(float64(shift)))))}}})
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

	tsig := gridgen.TPIG{Tilelayout: tiles, Dimensions: gridgen.Dimensions{Flat: gridgen.XY2D{X0: 0, X1: int(maxX), Y0: 0, Y1: int(maxY)}}}

	enc := json.NewEncoder(wTsig)
	enc.SetIndent("", "    ")

	return enc.Encode(tsig)
}
