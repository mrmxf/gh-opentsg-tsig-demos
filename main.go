//  Copyright ©2019-2024  Mr MXF   info@mrmxf.com
//  BSD-3-Clause License           https://opensource.org/license/bsd-3-clause/
//
// Package main runs the shape handlers

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
