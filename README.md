# tsig demos

A commad line tool for generating 3d displays in the
[obj][o1] and [TSIG][t2] formats.

Available display shapes are

- An open cube (No front wall panel)
- A curved cylindrical wall
- A Spherical display

## Getting started

Make sure you are familiar with [OpenTSG][t1] and what
[TSIGs][t2] are and how they are used.

In short TSIGs allow you to generate test patterns for 3d displays,
with pixel accurate mapping. These TSIgs are used as
an input for OpenTSG, which generates the test signal.

## Installation

Make sure you have the latest version of Go from the
[official golang source][g1] installed.

Then run the go build command to compile the code.

```cmd
go build
```

## flags

The `--conf` flag tells the cli which input yaml (or json) to parse.

The `--outputFile` tells the name of the file(s) to be saved,
the input string is without the inclusion of the file extension.
e.g.  `--outputFile ./examples/example` will produce two files,
`./examples/example.obj` and `./examples/example.json`

## Demos

Ensure the command line is installed with
`./generator --help`

Please note every obj produced has tha z axis
as the vertical axis, so when importing the obj
into your software of choice you can get the
object the right way up.

The following list of demos will take you through
generating an obj and tsig for that shape.

- [Cube][cbd]
- [Curve][cvd]
- [Sphere][spd]

Once a demo has been run, the TSIG output can be
plugged into openTSG.

### Cube Demo

This demo will walk you through generating a cube shape display

The cube demo is run with an input file of `./examples/cube.yaml`
which looks like

```yaml
# tile dimensions
tileHeight: 0.5
tileWidth: 0.5
# cube dimensions
cubeWidth: 5
cubeHeight: 5
cubeDepth: 2.5
# Pixels per tile
dx: 500
dy: 500
```

```cmd
./generator cube --conf ./examples/cube.yaml --outputFile ./examples/cube
```

Check ./examples/cube for the the output files of obj and json
congratulations you've now made your first obj and TSIG.

### Curve Demo

```cmd
./generator curve --conf ./examples/curve.yaml --outputFile ./examples/curve
```

### Sphere Demo

```cmd
./generator sphere--conf ./examples/sphere.yaml --outputFile ./examples/sphere
```

## Golden ratios

Any numbers that seem to work really well.
e.g. Dx* tile width / radius must = 5

## Technical stuff

Cube uv map design.

Design thoughts
formulas used.
The sphere uv map

[g1]:   https://go.dev/doc/install                "Golang Installation"

[t1]:   https://opentsg.studio/                    "openTSG Website"
[t2]:   https://github.com/mrmxf/opentsg-node/blob/main/READMETPIG.md            "TSIG information"

[o1]:   https://en.wikipedia.org/wiki/Wavefront_.obj_file    "OBJ wikipedia"

[cbd]: #cube-demo
[cvd]: #curve-demo
[spd]: #sphere-demo
