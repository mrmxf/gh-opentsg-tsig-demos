package main

import (
	"fmt"
	"generator/handler"
	"generator/shapes"
)

func init() {
	// init the shapes to be used by the program
	shapes.Init()
}

func main() {

	err := handler.Run()

	if err != nil {
		fmt.Println(err)
		return
	}
}
