package main

import (
	"fmt"
	"io"
	"os"

	"github.com/spf13/cobra"
	"gopkg.in/yaml.v3"
)

var cmdObj = &cobra.Command{
	Use:   "",
	Short: "TSIG and OBJ builder",
	Long: `
	TSIG and OBJ builder

	Please choose which object you'd like to build etc
	`,
	/*RunE: func(cmd *cobra.Command, args []string) error {
		fmt.Println(`
		The MRX reg server

		Running on whatever port you choose insert the paths available
		`)
		return nil
	},*/
}

var (
	configFile = ""
	outFile    = ""
)

func init() {
	// add the same flags to each shape
	// as these only handle the input or output
	cmdCube.Flags().StringVar(&configFile, "conf", "", "The configuration file")
	cmdCube.Flags().StringVar(&outFile, "outputFile", "./output", "The name of the output file")

	cmdCurve.Flags().StringVar(&configFile, "conf", "", "The configuration file")
	cmdCurve.Flags().StringVar(&outFile, "outputFile", "./output", "The name of the output file")

	cmdSphere.Flags().StringVar(&configFile, "conf", "", "The configuration file")
	cmdSphere.Flags().StringVar(&outFile, "outputFile", "./output", "The name of the output file")

	cmdObj.AddCommand(cmdCube, cmdCurve, cmdSphere)

}

// Generator is for writing to objs and tsigs
type Generator interface {
	generate(wObj io.Writer, wTsig io.Writer) error
	objType() string
}

// Gen shape is a generic function creator for handling generator objects.
// it can unmarshal the input file and create the io.Write destinations.
func GenShape[gen Generator]() func(cmd *cobra.Command, args []string) error {
	return func(cmd *cobra.Command, args []string) error {

		confBytes, err := os.ReadFile(configFile)

		if err != nil {
			return err
		}
		var genny gen
		yaml.Unmarshal(confBytes, &genny)

		fObj, err := os.Create(outFile + ".obj")
		if err != nil {
			return err
		}

		fTSIG, err := os.Create(outFile + ".json")
		if err != nil {
			return err
		}

		err = genny.generate(fObj, fTSIG)
		if err != nil {
			return err
		}

		fmt.Printf("Generated %v object\n", genny.objType())
		return nil
	}
}

// sphere properties
type Sphere struct {
	TileHeight      float64 `json:"tileHeight" yaml:"tileHeight"`
	TileWidth       float64 `json:"tileWidth" yaml:"tileWidth"`
	Radius          float64 `json:"radius" yaml:"radius"`
	ThetaMaxAngle   float64 `json:"thetaMaxAngle" yaml:"thetaMaxAngle"`
	AzimuthMaxAngle float64 `json:"azimuthMaxAngle" yaml:"azimuthMaxAngle"`
	Dx              float64 `json:"dx" yaml:"dx"`
	Dy              float64 `json:"dy" yaml:"dy"`
}

func (s Sphere) objType() string {
	return "sphere"
}

var cmdSphere = &cobra.Command{
	Use:   "sphere",
	Short: "A 3d sphere",
	Long: `
	A 3d sphere, but longer
	`,
	RunE: GenShape[Sphere](),
}

// Cube properties
type Cube struct {
	TileHeight float64 `json:"tileHeight" yaml:"tileHeight"`
	TileWidth  float64 `json:"tileWidth" yaml:"tileWidth"`
	CubeWidth  float64 `json:"cubeWidth" yaml:"cubeWidth"`
	CubeHeight float64 `json:"cubeHeight" yaml:"cubeHeight"`
	CubeDepth  float64 `json:"cubeDepth" yaml:"cubeDepth"`
	Dx         float64 `json:"dx" yaml:"dx"`
	Dy         float64 `json:"dy" yaml:"dy"`
}

func (c Cube) objType() string {
	return "cube"
}

var cmdCube = &cobra.Command{
	Use:   "cube",
	Short: "A 3d cube",
	Long: `
	A 3d cube, but longer
	`,
	RunE: GenShape[Cube](),
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

func (c Curve) objType() string {
	return "curve"
}

var cmdCurve = &cobra.Command{
	Use:   "curve",
	Short: "A curve wall",
	Long: `
	A 3d curve, but longer
	`,
	RunE: GenShape[Curve](),
}

/*

install viper and get that all going

command flags

- input file (required)
- output file(s) name no extensions
- TSIG only (optional)
- OBJ only (optional)

*/
