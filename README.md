# tsig demos

A demo repo for making 3d displays in the
[obj](https://en.wikipedia.org/wiki/Wavefront_.obj_file) and
[TSIG](https://link.to.tsig) formats.

Available display shapes are

- An open cube (No front wall panel)
- A curved cylindrical wall
- Spherical  display

## Getting started

Make sure you are familiar with OpenTSG and TSIGs.

In short TSIGs allow you to generate test patterns for 3d objects,
with pixel accurate mapping.

This repo is designed to produce an obj and a
TSIG at the same time, with the intention of
allowing you to view the test pattern on the obj,
that was produced by the TPIG.

## Installation

Make sure you have the latest version of [Go](https://go.dev/doc/install) installed.

Then run the go build command to compile the code.

```cmd
go build
```

## Running the program

Here's a list of
the demos for the
different shaped objects.

Ensure it is installed an running

- [Cube](#cube-demo)
- [Curve](#curve-demo)
- [Sphere](#sphere-demo)

Once a demo has been run plug it into openTSG.

### Cube Demo

```cmd
./generator cube --conf ./examples/cube.yaml --outputFile ./examples/cube
```

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

## Installation manual

yamls with comments!
json agnostic

## Technical stuff

Design thoughts
formulas used.
The sphere uv map
