//	Copyright ©2019-2024  Mr MXF   info@mrmxf.com
//	BSD-3-Clause License           https://opensource.org/license/bsd-3-clause/
//
// Package shapes contains the obj shapes and their configurations

package shapes

import (
	"fmt"
	"io"
	"os"

	"github.com/spf13/cobra"
	"gopkg.in/yaml.v3"
)

func init() {
	// assign all the cmd functions
	cmdBoth.Flags().StringVar(&configFile, "conf", "", "The configuration file")
	cmdBoth.Flags().StringVar(&outFile, "outputFile", "./output", "The name of the output file")

	cmdObj.Flags().StringVar(&configFile, "conf", "", "The configuration file")
	cmdObj.Flags().StringVar(&outFile, "outputFile", "./output", "The name of the output file")

	cmdTSIG.Flags().StringVar(&configFile, "conf", "", "The configuration file")
	cmdTSIG.Flags().StringVar(&outFile, "outputFile", "./output", "The name of the output file")

	cmdBoth.AddCommand(cmdObj, cmdTSIG, cmdList)
}

// shapes is a map of shapeName - yaml decoder to that type
var shapes = map[string]func([]byte) (Generator, error){}

/*
AddShapeToHandler adds a shape of type generator to be handled by the program.

Call this function during the init stage, before the main program runs.
*/
func AddShapeToHandler[gen Generator](ShortDesc, LongDesc string) {

	var shp gen
	shpName := shp.ObjType()
	if _, ok := shapes[shpName]; ok {
		panic(fmt.Sprintf("Shape type %s has been overwritten. Please ensure each name is unique", shpName))
	}

	shapes[shpName] = unmarshalGenerator[gen]

}

// ShapeName is struct designed to be
// embedded to keep the field name detection constant
/*
e.g.

type example struct {
	field1 string
	field1 string
	ShapeName
}

*/
type ShapeName struct {
	Shape string `json:"shape" yaml:"shape"`
}

// RunHandler runs the CLI functionality
func RunHandler() error {
	err := cmdBoth.Execute()

	if err != nil {

		return err
	}

	return nil
}

// @TODO update to three modes one is default so should run wihtout the key
var cmdBoth = &cobra.Command{
	Use:   "gen",
	Short: "TSIG and OBJ builder",
	Long: `
	TSIG and OBJ builder

	Please choose which object you'd like to build etc
	`,
	RunE: genShapeNew(true, true),
}

// Only build an OBJ
var cmdObj = &cobra.Command{
	Use:   "obj",
	Short: "OBJ builder",
	Long: `
	TSIG and OBJ builder

	Please choose which object you'd like to build etc
	`,
	RunE: genShapeNew(false, true),
}

// only build a TSIG
var cmdTSIG = &cobra.Command{
	Use:   "tsig",
	Short: "TSIG builder",
	Long: `
	TSIG builder

	Please choose which object you'd like to build etc
	`,
	RunE: genShapeNew(true, false),
}

var cmdList = &cobra.Command{
	Use:   "list",
	Short: "list all the available shape names that can be generated",
	Long: `
	List all the available shape names that can be generated
	`,
	RunE: func(cmd *cobra.Command, args []string) error {

		fmt.Println("Available shapes are:")
		for s := range shapes {
			fmt.Printf(" - %v \n", s)
		}

		return nil
	},
}

var (
	configFile = ""
	outFile    = ""
)

// Generator is for writing shapes
// to objs and tsigs
type Generator interface {
	// Generate the shape as an obj and TSIG
	Generate(wObj, wTsig io.Writer) error
	// single word for the object type, e.g. square
	ObjType() string
}

func genShapeNew(tsig, obj bool) func(cmd *cobra.Command, args []string) error {
	return func(cmd *cobra.Command, args []string) error {
		confBytes, err := os.ReadFile(configFile)

		if err != nil {
			return err
		}
		var name ShapeName
		err = yaml.Unmarshal(confBytes, &name)
		if err != nil {
			return err
		}

		if name.Shape == "" {
			return fmt.Errorf("no shape name found, the name field must be named \"shape\" in both json and yaml ")
		}

		shpUnmarshal, ok := shapes[name.Shape]

		if !ok {
			return fmt.Errorf("no shape with the name %v found", name.Shape)
		}

		shp, err := shpUnmarshal(confBytes)
		if err != nil {
			return err
		}

		var fObj io.Writer
		if obj {
			fObj, err = os.Create(outFile + ".obj")
			if err != nil {
				return err
			}
		} else {
			fObj = io.Discard
		}

		var fTSIG io.Writer

		if tsig {
			fTSIG, err = os.Create(outFile + ".json")
			if err != nil {
				return err
			}
		} else {
			fTSIG = io.Discard
		}

		err = shp.Generate(fObj, fTSIG)
		if err != nil {
			return err
		}

		fmt.Printf("Generated %v object\n", shp.ObjType())
		return nil
	}
}

// unmarshalGenerator allows methods to be used for unmarshaling the shapes.
// This is because when the Generator method is wrapped a couple of times
// the output of the unmarshal can not be assigned to the method generator
func unmarshalGenerator[gen Generator](bytes []byte) (Generator, error) {

	out := new(gen)

	if err := yaml.Unmarshal(bytes, out); err != nil {
		return nil, err
	}
	return *out, nil
}
