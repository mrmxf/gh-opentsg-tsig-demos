package main

import (
	"fmt"
	"generator/shapes"
)

func main() {

	err := shapes.RunHandler()

	if err != nil {
		fmt.Println(err)
		return
	}
}
