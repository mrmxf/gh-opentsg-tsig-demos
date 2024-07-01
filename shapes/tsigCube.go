//  Copyright ©2019-2024  Mr MXF   info@mrmxf.com
//  BSD-3-Clause License           https://opensource.org/license/bsd-3-clause/
//
// Package shapes contains the obj shapes and their configurations

/*
information aobut the rest of it
*/

package shapes

import (
	"encoding/json"
	"fmt"
	"io"
	"math"

	"github.com/mrmxf/opentsg-modules/opentsg-core/gridgen"
)

// add the shape to the main handler here
func init() {

	AddShapeToHandler[Cube]("An open faced cube")
}

// Cube properties
type Cube struct {
	// dimensions of the tiles
	TileHeight float64 `json:"tileHeight" yaml:"tileHeight"`
	TileWidth  float64 `json:"tileWidth" yaml:"tileWidth"`
	// x dimension
	CubeWidth float64 `json:"cubeWidth" yaml:"cubeWidth"`
	// z dimension
	CubeHeight float64 `json:"cubeHeight" yaml:"cubeHeight"`
	// y dimension
	CubeDepth float64 `json:"cubeDepth" yaml:"cubeDepth"`
	// pixels per direction
	Dx float64 `json:"dx" yaml:"dx"`
	Dy float64 `json:"dy" yaml:"dy"`
	// shape name of cube
	ShapeName
}

func (c Cube) ObjType() string {
	return "cube"
}

/*
GenHalfCubeOBJ generates a TSIG and OBJ for a cube with no front panel.
The dimensions are as so:

  - Width is the x plane

  - Depth is the y plane

  - Height is the z plane

    Errors will be returned if the tiles do not fit exactly into the dimensions. E.g. a tile width of 1 is given and the cube has a width of 3.5
*/
func (c Cube) Generate(wObj, wTsig io.Writer) error {

	// check the dimensions
	err := halfCubeFence(c.TileHeight, c.TileWidth, c.CubeWidth, c.CubeHeight, c.CubeDepth)

	if err != nil {
		return err
	}

	// get the dimensions of the flat display.
	pixelWidth := ((c.CubeWidth + c.CubeDepth*2) / c.TileWidth) * c.Dx
	pixelHeight := ((c.CubeDepth*2 + c.CubeHeight) / c.TileHeight) * c.Dy

	// count of tiles in each segment of cube
	leftRight := int((c.CubeDepth * 2 / c.TileWidth) * (c.CubeHeight / c.TileHeight))
	topbot := int((c.CubeDepth * 2 / c.TileWidth) * (c.CubeWidth / c.TileHeight))
	back := int((c.CubeHeight / c.TileHeight) * (c.CubeWidth / c.TileWidth))

	tiles := make([]gridgen.Tilelayout, leftRight+topbot+back)

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
		{iEnd: c.CubeDepth, jEnd: c.CubeHeight, iStep: c.TileWidth, jStep: c.TileHeight, planeConst: 0,
			plane: "y", vStart: (c.CubeDepth / c.TileHeight) * vStep, inverse: true, uStart: ((c.CubeDepth + c.CubeWidth) / c.TileWidth) * uStep},
		// right wall
		{iEnd: c.CubeDepth, jEnd: c.CubeHeight, iStep: c.TileWidth, jStep: c.TileHeight,
			planeConst: c.CubeWidth, plane: "y", vStart: (c.CubeDepth / c.TileHeight) * vStep},

		// back wall
		{iEnd: c.CubeWidth, jEnd: c.CubeHeight, iStep: c.TileWidth, jStep: c.TileHeight,
			planeConst: c.CubeDepth, plane: "x", vStart: (c.CubeDepth / c.TileHeight) * vStep, uStart: ((c.CubeDepth) / c.TileWidth) * uStep},

		// Top
		{iEnd: c.CubeDepth, inverse: true, jEnd: c.CubeWidth, iStep: c.TileWidth, jStep: c.TileHeight,
			planeConst: c.CubeHeight, plane: "z", vStart: ((c.CubeDepth + c.CubeHeight) / c.TileHeight) * vStep, uStart: ((c.CubeDepth) / c.TileWidth) * uStep},

		// Bottom
		{iEnd: c.CubeDepth, jEnd: c.CubeWidth, iStep: c.TileWidth, jStep: c.TileHeight,
			planeConst: 0, plane: "z", uStart: ((c.CubeDepth) / c.TileWidth) * uStep},
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
		// the obj buffer
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

					tiles[tileCount] = gridgen.Tilelayout{Layout: gridgen.Positions{Flat: gridgen.XY{X: int(math.Round((p.uStart + width - float64(iCount+1)*uStep) * pixelWidth)), Y: int(math.Round((1 - (p.vStart + float64(jCount+1)*vStep)) * pixelHeight))}, Size: gridgen.XY{X: int(c.Dx), Y: int(c.Dy)}}}

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
						tiles[tileCount] = gridgen.Tilelayout{Layout: gridgen.Positions{Flat: gridgen.XY{X: int(math.Round((p.uStart + width - float64(iCount+1)*uStep) * pixelWidth)), Y: int(math.Round((1 - (p.vStart + float64(jCount+1)*vStep)) * pixelHeight))}, Size: gridgen.XY{X: int(c.Dx), Y: int(c.Dy)}}}

					} else {

						tileFace += fmt.Sprintf("vt %v %v \n", p.uStart+float64(iCount)*uStep, p.vStart+float64(jCount)*vStep)
						tileFace += fmt.Sprintf("vt %v %v \n", p.uStart+float64(iCount+1)*uStep, p.vStart+float64(jCount)*vStep)
						tileFace += fmt.Sprintf("vt %v %v \n", p.uStart+float64(iCount+1)*uStep, p.vStart+float64(jCount+1)*vStep)
						tileFace += fmt.Sprintf("vt %v %v \n", p.uStart+float64(iCount)*uStep, p.vStart+float64(jCount+1)*vStep)
						tiles[tileCount] = gridgen.Tilelayout{Layout: gridgen.Positions{Flat: gridgen.XY{X: int(math.Round((p.uStart + width - float64(iCount+1)*uStep) * pixelWidth)), Y: int(math.Round((1 - (p.vStart + float64(jCount+1)*vStep)) * pixelHeight))}, Size: gridgen.XY{X: int(c.Dx), Y: int(c.Dy)}}}

					}

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

						tiles[tileCount] = gridgen.Tilelayout{Layout: gridgen.Positions{Flat: gridgen.XY{X: int(math.Round((p.uStart + ujwidth - float64(jCount+1)*uStep) * pixelWidth)), Y: int(math.Round((1 - (p.vStart + (uTotal-float64(iCount))*vStep)) * pixelHeight))}, Size: gridgen.XY{X: int(c.Dx), Y: int(c.Dy)}}}

					} else {
						tileFace += fmt.Sprintf("vt %v %v \n", p.uStart+ujwidth-float64(jCount+1)*uStep, p.vStart+float64(iCount)*vStep)
						tileFace += fmt.Sprintf("vt %v %v \n", p.uStart+ujwidth-float64(jCount)*uStep, p.vStart+float64(iCount)*vStep)
						tileFace += fmt.Sprintf("vt %v %v \n", p.uStart+ujwidth-float64(jCount)*uStep, p.vStart+float64(iCount+1)*vStep)
						tileFace += fmt.Sprintf("vt %v %v \n", p.uStart+ujwidth-float64(jCount+1)*uStep, p.vStart+float64(iCount+1)*vStep)

						tiles[tileCount] = gridgen.Tilelayout{Layout: gridgen.Positions{Flat: gridgen.XY{X: int(math.Round((p.uStart + ujwidth - float64(jCount+1)*uStep) * pixelWidth)), Y: int(math.Round((1 - (p.vStart + float64(iCount+1)*vStep)) * pixelHeight))}, Size: gridgen.XY{X: int(c.Dx), Y: int(c.Dy)}}}
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

	tsig := gridgen.TPIG{Tilelayout: tiles, Dimensions: gridgen.Dimensions{Flat: gridgen.XY2D{X0: 0, X1: int(pixelWidth), Y0: 0, Y1: int(pixelHeight)}}}

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
