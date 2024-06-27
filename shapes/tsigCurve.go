package shapes

import (
	"encoding/json"
	"fmt"
	"io"
	"math"

	"github.com/mrmxf/opentsg-modules/opentsg-core/gridgen"
)

func init() {
	AddShapeToHandler[Curve]("", "")
}

// Curve Properties
type Curve struct {
	TileHeight      float64 `json:"tileHeight" yaml:"tileHeight"`
	TileWidth       float64 `json:"tileWidth" yaml:"tileWidth"`
	CurveRadius     float64 `json:"cylinderRadius" yaml:"cylinderRadius"`
	CurveHeight     float64 `json:"cylinderHeight" yaml:"cylinderHeight"`
	AzimuthMaxAngle float64 `json:"azimuthMaxAngle" yaml:"azimuthMaxAngle"`
	Dx              float64 `json:"dx" yaml:"dx"`
	Dy              float64 `json:"dy" yaml:"dy"`
}

func (c Curve) ObjType() string {
	return "curve"
}

/*
GenCurveOBJ generates a TSIG and OBJ for a curved cylindrical wall.
The wall is centred around 0,0,0

Angles are in **Radians**
*/
func (c Curve) Generate(wObj, wTsig io.Writer) error {

	// get the total angle covered by the cylinder.
	azimuthInc := (2 * math.Asin(c.TileWidth/(2*c.CurveRadius)))

	z := 0.0
	vertexCount := 1
	azimuth := -c.AzimuthMaxAngle

	pixelWidth := math.Ceil(2*c.AzimuthMaxAngle/azimuthInc) * c.Dx
	pixelHeight := math.Ceil(c.CurveHeight/c.TileHeight) * c.Dy

	// rows * column for the total expected tile count
	tiles := make([]gridgen.Tilelayout, int(math.Ceil(2*c.AzimuthMaxAngle/azimuthInc)*math.Ceil(c.CurveHeight/c.TileHeight)))

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

			tiles[tileCount] = gridgen.Tilelayout{Layout: gridgen.Positions{Flat: gridgen.XY{X: int(math.Round(u * pixelWidth)), Y: int(math.Round((1 - (v + vheight)) * pixelHeight))}, Size: gridgen.XY{X: int(c.Dx), Y: int(c.Dy)}}}

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

	tsig := gridgen.TPIG{Tilelayout: tiles, Dimensions: gridgen.Dimensions{Flat: gridgen.XY2D{X0: 0, X1: int(pixelWidth), Y0: 0, Y1: int(pixelHeight)}}}

	enc := json.NewEncoder(wTsig)
	enc.SetIndent("", "    ")
	return enc.Encode(tsig)

}
