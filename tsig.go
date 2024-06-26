package main

type TPIGCompile struct {
	TSIGS      []Imports  `json:"tpig include"`
	Dimensions Dimensions `json:"Dimensions"`
}

type Imports struct {
	Include string `json:"uri"`
	Alias   string `json:"alias"`
}

type TSIG struct {
	Tilelayout []Tilelayout    `json:"Tile layout"`
	Dimensions Dimensions      `json:"Dimensions"`
	Carve      map[string]XY2D `json:"Carve"`
	// NAme?
}

type Tilelayout struct {
	Name       string    `json:"Name,omitempty"`
	Tags       []string  `json:"Tags"`
	Neighbours []string  `json:"Neighbours"`
	Layout     Positions `json:"Layout"`
}

type Positions struct {
	Carve XY `json:"Carve,omitempty"`
	Flat  XY `json:"Flat"`
	Size  XY `json:"XY"`
}

type Dimensions struct {
	Carve XY2D `json:"Carve"`
	Flat  XY2D `json:"Flat"`
}

type XY struct {
	Destination string `json:"Destination,omitempty"`
	X           int    `json:"X"`
	Y           int    `json:"Y"`
}

type XY2D struct {
	X0 int `json:"X0"`
	Y0 int `json:"Y0"`
	X1 int `json:"X1"`
	Y1 int `json:"Y1"`
}

/*
func ObjToTsig(file string, xCount, yCount float32) error {
	o, e := gwob.NewObjFromFile(file, &gwob.ObjParserOptions{})

	fmt.Println(o, e)
	fmt.Println(o.NumberOfElements())
	fmt.Println(o.Coord[0])
	fmt.Println(len(o.Groups))
	fmt.Println(o.StrideSize, o.StrideOffsetPosition, o.StrideOffsetTexture)

	fmt.Println(o.Coord[0:20], len(o.Coord))

	tiles := make([]Tilelayout, o.NumberOfElements())

	tileI := 0
	for i := 0; i < len(o.Coord); i += o.StrideSize {

		uvs := o.Coord[i+o.StrideOffsetTexture : i+o.StrideSize : i+o.StrideSize]
		xPos, yPos := float32(1.0), float32(0.0)

		fmt.Println(o.Indices[i : i+o.StrideSize])
		for j := 0; j < o.StrideSize-o.StrideOffsetTexture; j += 2 {
			if uvs[j] < xPos {
				xPos = uvs[j]
			}

			if uvs[j+1] > yPos {
				yPos = uvs[j+1]
			}
		}
		fmt.Println(xPos, yPos)

		tiles[tileI] = Tilelayout{Layout: Positions{Flat: XY{X: int(xPos * xCount), Y: 0}}}
		// x is kept 0
		// y is inverted

		tileI++
	}

	tsig := TPIG{Tilelayout: tiles}

	fmt.Println(tsig)
	return nil
}
*/
