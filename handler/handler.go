package handler

import (
	"fmt"
	"io"
	"os"

	"github.com/spf13/cobra"
	"gopkg.in/yaml.v3"
)

/*
Add shape adds a shape of type generator to be handled by the programm.
To be called during the init stage of something
*/
func AddShape[gen Generator](ShortDesc, LongDesc string) {

	var g gen
	var cmdShape = &cobra.Command{
		Use:   g.ObjType(),
		Short: ShortDesc,
		Long:  LongDesc,
		RunE:  genShape[gen](),
	}

	cmdShape.Flags().StringVar(&configFile, "conf", "", "The configuration file")
	cmdShape.Flags().StringVar(&outFile, "outputFile", "./output", "The name of the output file")

	cmdObj.AddCommand(cmdShape)

}

// Run runs the CLI functionality
func Run() error {
	err := cmdObj.Execute()

	if err != nil {

		return err
	}

	return nil
}

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

// Generator is for writing to objs and tsigs
type Generator interface {
	Generate(wObj io.Writer, wTsig io.Writer) error
	ObjType() string
}

// Gen shape is a generic function creator for handling generator objects.
// it can unmarshal the input file and create the io.Write destinations.
func genShape[gen Generator]() func(cmd *cobra.Command, args []string) error {
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

		err = genny.Generate(fObj, fTSIG)
		if err != nil {
			return err
		}

		fmt.Printf("Generated %v object\n", genny.ObjType())
		return nil
	}
}
