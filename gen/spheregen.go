package main

import (
	"bytes"
	"fmt"
	"io"
	"math"
	"os"
	"slices"

	"encoding/json"
)

func main() {
	f, _ := os.Create("out/realTiles.obj")
	maxAngle := 30.0
	maxTheta := (math.Pi / 180) * maxAngle
	GenSphereOBJ(f, 0.5, 0.5, 5, maxTheta, maxTheta)
	//ObjToTsig("out/realTiles.obj", 100, 100)
	//sphere()

	fc, _ := os.Create("out/realTilesCurve.obj")
	fct, _ := os.Create("out/realTilesCurve.json")
	GenCurveOBJ(fc, fct, 0.5, 0.5, 5, 5, maxTheta, 500, 500)

	fcu, _ := os.Create("out/realTilesCube.obj")
	fcut, _ := os.Create("out/realTilesCube.json")
	fmt.Println(GenHalfCubeOBJ(fcu, fcut, 0.5, 0.5, 5, 5, 2.5, 500, 500))
}

// generate some xy coordinates

func sphere() {

	/*
		start at 0,0

		A sphere of radius xm made of squares

	*/

	// origin at 0,0
	// wall starts at 0,5

	/*
		get angle limits with this way
		can be applied in both planes
			x = x0 + r * cos(theta)
			y = y0 + r * sin(theta)


	*/

	/*
		calculate change in x and y

		we have x and y size in cm


		circumference = pi D

		chord length formula

		0.1 := 2 × r × sin(theya/2)
		0.1/(2 * r) = sin(theta/2)

		use theta as a pola coordinate then translate to xyz
	*/
	maxAngle := 30.0
	maxTheta := (math.Pi / 180) * maxAngle
	//azimuth := 0.0
	theta := (math.Pi / 2) + maxTheta
	radius := 5.0
	tileSize := 0.8

	// chord length formula
	thetaInc := 2 * math.Asin(tileSize/(2*radius))
	//thetaInc := (2 * math.Asin(tileSize/(2*radius))) / math.Sin(theta)

	/*
			v 5 0 0
		v 5 5 0
		v 5 5 5
		v 5 0 5

		f 1/1 2/2 3/3 4/4*/
	// keep it left

	//z0 := 0
	count := 1
	objbuf := bytes.NewBuffer([]byte{})
	fmt.Println(maxTheta, thetaInc)
	facebuf := bytes.NewBuffer([]byte{})
	//	z := 0.0

	// update this to be in polar space
	// start as a row then update etc
	tiles := []int{}
	for theta > (math.Pi/2)-maxTheta {
		g := (2 * math.Asin(tileSize/(2*radius))) / math.Sin(theta)
		fmt.Println((2 * maxTheta) / g)
		azimuth := 0.0
		tileCount := 0
		for azimuth < maxTheta {
			tileCount++

			azimuthInc := (2 * math.Asin(tileSize/(2*radius))) / math.Sin(theta)
			azimuthIncTop := (2 * math.Asin(tileSize/(2*radius))) / math.Sin(theta-thetaInc)
			PolarStepperForward(objbuf, radius, theta, azimuth, azimuthInc, azimuthIncTop, thetaInc)

			facebuf.WriteString(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3))

			//	objbuf.WriteString(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3))
			azimuth += azimuthInc
			count += 4
		}
		azimuth = 0.0 - (2*math.Asin(tileSize/(2*radius)))/math.Sin(theta)
		for azimuth > -maxTheta {
			tileCount++
			azimuthInc := (2 * math.Asin(tileSize/(2*radius))) / math.Sin(theta)
			azimuthIncTop := (2 * math.Asin(tileSize/(2*radius))) / math.Sin(theta-thetaInc)
			PolarStepperForward(objbuf, radius, theta, azimuth, azimuthInc, azimuthIncTop, thetaInc)

			facebuf.WriteString(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3))

			//	objbuf.WriteString(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3))
			azimuth -= azimuthInc
			count += 4
		}
		fmt.Println("deg", theta*(180/math.Pi))
		tiles = append(tiles, tileCount)
		//thetaInc := 2 * math.Asin(tileSize/(2*radius))
		theta -= thetaInc
		azimuth = 0
		//fmt.Println("COINTER", theta, z, zinchold)
		//	z = zinchold

	}

	height := 1 / float64(len(tiles))
	widthstep := 1 / (float64((slices.Max(tiles)) + 1))
	fmt.Println(tiles)
	fmt.Println(height, widthstep)
	// width = 5400 height = 5300
	/*
				   vt 0.5 0
				   vt 0.5185185185185185 0
				   vt 0.5185185185185185 0.018867924528301886
				   vt 0.5 0

				  vt 0.5185185185185185 0.018867924528301886
				vt 0.537037037037037 0.018867924528301886
				vt 0.537037037037037 0.03773584905660377
				vt 0.5185185185185185 0.03773584905660377

				0.5185185185185185 0
		vt 0.537037037037037 0
		vt 0.537037037037037 0.018867924528301886
		vt 0.5185185185185185 0.018867924528301886

				   vt 0.4814814814814815 0
		vt 0.5 0
		vt 0.5 0.018867924528301886
		vt 0.4814814814814815 0.018867924528301886
				objbuf.WriteString(fmt.Sprintf("vt 0 0 \n"))
					objbuf.WriteString(fmt.Sprintf("vt 1 0 \n"))
					objbuf.WriteString(fmt.Sprintf("vt 1 1 \n"))
					objbuf.WriteString(fmt.Sprintf("vt 0 1\n"))
	*/
	vt := 0
	for v, t := range tiles {
		step := t / 2
		ymin, ymax := float64(v)*height, float64(v+1)*height
		for i := 0; i <= step; i++ {
			xmin, xmax := 0.5+float64(i)*widthstep, 0.5+float64(i+1)*(widthstep)

			objbuf.WriteString(fmt.Sprintf("vt %v %v \n", xmin, ymin))
			objbuf.WriteString(fmt.Sprintf("vt %v %v \n", xmax, ymin))
			objbuf.WriteString(fmt.Sprintf("vt %v %v \n", xmax, ymax))
			objbuf.WriteString(fmt.Sprintf("vt %v %v \n", xmin, ymax))
			vt++
		}

		for i := 0; i < step; i++ {
			xmin, xmax := 0.5-float64(i+1)*widthstep, 0.5-float64(i)*(widthstep)
			objbuf.WriteString(fmt.Sprintf("vt %v %v \n", xmin, ymin))
			objbuf.WriteString(fmt.Sprintf("vt %v %v \n", xmax, ymin))
			objbuf.WriteString(fmt.Sprintf("vt %v %v \n", xmax, ymax))
			objbuf.WriteString(fmt.Sprintf("vt %v %v \n", xmin, ymax))
			vt++
		}
	}

	objbuf.Write(facebuf.Bytes())
	fmt.Println("count", count, ",", count/4, "VTcount", vt)
	f, _ := os.Create("./out/line.obj")
	f.Write(objbuf.Bytes())

	/*

				perfect square is now mad eup of immutable 2d sqaures


				square has a shape calculate the angle needed and go to there.

				Design is Top left top tight and right top left join.count
				Get the size of the object and get some vectors

				z angle will always be the same

				shape = (0,0,0)(0,1,0)(0,1,1)(0,0,1)

				couple of philoshipes are overlapping bottoms / tops
				back.count
				calculate teh z angle

				For z-Axis Rotation-
		This rotation is achieved by using the following rotation equations-


		Xnew = Yold x sinθ + Xold x cosθ
		Znew = Zold
		Ynew = Zold x cosθ – Xold x sinθ

		start with top rght and calculate everything else

		vertical shift.

	*/

	//reverse := math.Asin(tileSize/2* radius)

	//

	// angle := (math.Pi / 2) -  math.Atan(tileSize / radius)
	tileSize = 0.8
	reverse := 2 * (math.Asin(tileSize / (2 * radius)))
	/*	x1, y1, z1 := PolarToCartesian(radius, math.Pi/2, 0)
		x2, y2, z2 := PolarToCartesian(radius, (math.Pi/2)+reverse, 0)
		//	x3, x4 := 0, 0
		fmt.Println(x1, z1, y1, x2, y2, z2, reverse, math.Cos(reverse))
		fmt.Println(ThreeDistance(x1, x2, y1, y2, z1, z2), tileSize)

		x1 = math.Cos(reverse) * tileSize
		z1 = math.Sin(reverse) * tileSize
		y1 = y1
		fmt.Println(x1, z1)*/

	x1, y1, z1 := 5.0, 0.0, 0.0

	// y axis
	x2 := x1*math.Cos(reverse) + y1*math.Sin(reverse)
	y2 := x1*math.Sin(reverse) + y1*math.Cos(reverse)
	z2 := z1
	// zaxis

	x3 := z1*math.Sin(reverse) + x1*math.Cos(reverse)
	y3 := y1
	z3 := y1*math.Cos(reverse) + x1*math.Sin(reverse)

	/*
		x4 := x3*math.Cos(reverse) + y3*math.Sin(reverse)
		y4 := x3*math.Sin(reverse) + y3*math.Cos(reverse)
		z4 := z3*/

	x4 := z2*math.Sin(reverse) + x2*math.Cos(reverse)
	y4 := y2
	z4 := y2*math.Cos(reverse) + x2*math.Sin(reverse)

	fmt.Println("2", x2, y2, z2)
	fmt.Println("3", x3, y3, z3)
	fmt.Println("4", x4, y4, z4)
	fmt.Println(ThreeDistance(x2, x1, y2, y1, z2, z1))

	fmt.Println(ThreeDistance(x1, 0, 0, 0, z1, 0), tileSize)

	//https://www.gpp7.org.in/wp-content/uploads/sites/22/2020/04/file_5e9df44854704.pdf
	/*

		give this a go.

		Start with 0,0 coordinates that are slowly rotated round.

		Rotatez axis round to x2 etc
		then can rotate x2, x1 up in the y axis

	*/
	xChange := math.Cos(reverse) * tileSize
	x1, x2 = x1-xChange, x2-xChange

	objbuf.WriteString(fmt.Sprintf("v %v %v %v \n", x1, y1, z1))

	objbuf.WriteString(fmt.Sprintf("v %v %v %v \n", x2, y2, z2))

	//objbuf.WriteString(fmt.Sprintf("v %v %v %v \n", x3, y3, z3))

	//objbuf.WriteString(fmt.Sprintf("v %v %v %v \n", x4, y4, z4))
	newBuf := bytes.NewBuffer([]byte{})
	azimuth, clockAz := 0.0, 0.0
	tileCount := 0
	theta = math.Pi / 2
	count = 1
	tileSize = 0.4
	thetaInc = 2 * (math.Asin(tileSize / (2 * radius)))
	theta = (math.Pi / 2)

	// TOP
	for theta > (math.Pi/2)-maxTheta {
		//start Point :=
		topLeftThet := theta - thetaInc
		topLeftAz := azimuth
		for azimuth < maxTheta {
			tileCount++

			azimuthInc := (2 * math.Asin(tileSize/(2*radius))) / math.Sin(theta)
			azimuthIncTop := (2 * math.Asin(tileSize/(2*radius))) / math.Sin(topLeftThet)
			x1, y1, z1 := PolarToCartesian(radius, topLeftThet+thetaInc, topLeftAz)
			newBuf.WriteString(fmt.Sprintf("v %v %v %v \n", x1, y1, z1))
			x2, y2, z2 := PolarToCartesian(radius, topLeftThet+thetaInc, topLeftAz+azimuthInc) // increase azimuth
			newBuf.WriteString(fmt.Sprintf("v %v %v %v \n", x2, y2, z2))
			x3, y3, z3 := PolarToCartesian(radius, topLeftThet, topLeftAz+azimuthIncTop) // increase azimuth and height
			newBuf.WriteString(fmt.Sprintf("v %v %v %v \n", x3, y3, z3))

			x4, y4, z4 := PolarToCartesian(radius, topLeftThet, topLeftAz) // increase height
			newBuf.WriteString(fmt.Sprintf("v %v %v %v \n", x4, y4, z4))
			fmt.Println("up left", ThreeDistance(x1, x4, y1, y4, z1, z4))

			//	fmt.Println(math.Sqrt(math.Pow((x2)-x1, 2)+math.Pow((y2)-y1, 2)) + math.Pow((z2)-z1, 2))
			fmt.Println("bottom", ThreeDistance(x1, x2, y1, y2, z1, z2))
			fmt.Println("up right", ThreeDistance(x2, x3, y2, y3, z2, z3))
			fmt.Println("top", ThreeDistance(x3, x4, y3, y4, z3, z4))

			newBuf.WriteString(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3))

			fmt.Println(x1*x2 + y1*y2 + z2*z3)
			fmt.Println(math.Acos((x1*x2 + y1*y2 + z2*z1) / (math.Sqrt(x1*x1+y1*y1+z1*z1) * math.Sqrt(x2*x2+z2*z2+y2*y2))))
			fmt.Println(math.Acos((x4*x3 + y4*y3 + z4*z3) / (math.Sqrt(x3*x3+y3*y3+z3*z3) * math.Sqrt(x4*x4+z4*z4+y4*y4))))
			//	objbuf.WriteString(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3))
			azimuth += azimuthIncTop
			topLeftAz = azimuth
			count += 4
		}

		topRightAz := clockAz
		for clockAz > -maxTheta {
			tileCount++

			azimuthInc := (2 * math.Asin(tileSize/(2*radius))) / math.Sin(theta)
			azimuthIncTop := (2 * math.Asin(tileSize/(2*radius))) / math.Sin(topLeftThet)
			x1, y1, z1 := PolarToCartesian(radius, topLeftThet+thetaInc, topRightAz)
			newBuf.WriteString(fmt.Sprintf("v %v %v %v \n", x1, y1, z1))
			x2, y2, z2 := PolarToCartesian(radius, topLeftThet+thetaInc, topRightAz-azimuthInc) // increase azimuth
			newBuf.WriteString(fmt.Sprintf("v %v %v %v \n", x2, y2, z2))
			x3, y3, z3 := PolarToCartesian(radius, topLeftThet, topRightAz-azimuthIncTop) // increase azimuth and height
			newBuf.WriteString(fmt.Sprintf("v %v %v %v \n", x3, y3, z3))

			x4, y4, z4 := PolarToCartesian(radius, topLeftThet, topRightAz) // increase height
			newBuf.WriteString(fmt.Sprintf("v %v %v %v \n", x4, y4, z4))

			newBuf.WriteString(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3))

			//	objbuf.WriteString(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3))
			clockAz -= azimuthIncTop
			topRightAz = clockAz
			count += 4
		}

		theta -= thetaInc
		azimuth = 0
		clockAz = 0
		//fmt.Println("COINTER", theta, z, zinchold)
		//	z = zinchold

	}

	// Bottom
	theta = math.Pi / 2
	for theta < (math.Pi/2)+maxTheta {
		//start Point :=
		botLeftThet := theta - thetaInc
		botLeftAz := azimuth
		for azimuth < maxTheta {
			tileCount++

			azimuthInc := (2 * math.Asin(tileSize/(2*radius))) / math.Sin(theta)
			azimuthIncTop := (2 * math.Asin(tileSize/(2*radius))) / math.Sin(botLeftThet)
			x1, y1, z1 := PolarToCartesian(radius, botLeftThet+thetaInc, botLeftAz)
			newBuf.WriteString(fmt.Sprintf("v %v %v %v \n", x1, y1, z1))
			x2, y2, z2 := PolarToCartesian(radius, botLeftThet+thetaInc, botLeftAz+azimuthInc) // increase azimuth
			newBuf.WriteString(fmt.Sprintf("v %v %v %v \n", x2, y2, z2))
			x3, y3, z3 := PolarToCartesian(radius, botLeftThet, botLeftAz+azimuthIncTop) // increase azimuth and height
			newBuf.WriteString(fmt.Sprintf("v %v %v %v \n", x3, y3, z3))

			x4, y4, z4 := PolarToCartesian(radius, botLeftThet, botLeftAz) // increase height
			newBuf.WriteString(fmt.Sprintf("v %v %v %v \n", x4, y4, z4))

			//	fmt.Println(math.Sqrt(math.Pow((x2)-x1, 2)+math.Pow((y2)-y1, 2)) + math.Pow((z2)-z1, 2))

			newBuf.WriteString(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3))

			//	objbuf.WriteString(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3))
			azimuth += azimuthIncTop
			botLeftAz = azimuth
			count += 4
		}

		botRightAz := clockAz
		for clockAz > -maxTheta {
			tileCount++

			azimuthInc := (2 * math.Asin(tileSize/(2*radius))) / math.Sin(theta)
			azimuthIncTop := (2 * math.Asin(tileSize/(2*radius))) / math.Sin(botLeftThet)
			x1, y1, z1 := PolarToCartesian(radius, botLeftThet+thetaInc, botRightAz)
			newBuf.WriteString(fmt.Sprintf("v %v %v %v \n", x1, y1, z1))
			x2, y2, z2 := PolarToCartesian(radius, botLeftThet+thetaInc, botRightAz-azimuthInc) // increase azimuth
			newBuf.WriteString(fmt.Sprintf("v %v %v %v \n", x2, y2, z2))
			x3, y3, z3 := PolarToCartesian(radius, botLeftThet, botRightAz-azimuthIncTop) // increase azimuth and height
			newBuf.WriteString(fmt.Sprintf("v %v %v %v \n", x3, y3, z3))

			x4, y4, z4 := PolarToCartesian(radius, botLeftThet, botRightAz) // increase height
			newBuf.WriteString(fmt.Sprintf("v %v %v %v \n", x4, y4, z4))

			newBuf.WriteString(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3))

			//	objbuf.WriteString(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3))
			clockAz -= azimuthIncTop
			botRightAz = clockAz
			count += 4
		}

		theta += thetaInc
		azimuth = 0
		clockAz = 0
		//fmt.Println("COINTER", theta, z, zinchold)
		//	z = zinchold
		fmt.Println("EUN")
	}

	/*
		start at 0,0


	*/

	f, _ = os.Create("./out/real.obj")
	f.Write(newBuf.Bytes())
}

// make a distance calculator
// math.Sqrt(math.Pow((z+xinc)-prevX, 2) + math.Pow((z+yinc)-prevY, 2)

func PolarToCartesian(r, theta, phi float64) (X, Y, Z float64) {
	X = r * math.Sin(theta) * math.Cos(phi)
	Y = r * math.Sin(theta) * math.Sin(phi)
	Z = r * math.Cos(theta)
	return
}

func CylindricalToCartesian(r, z, azimuth float64) (X, Y, Z float64) {
	X = r * math.Cos(azimuth)
	Y = r * math.Sin(azimuth)
	Z = z
	return
}

func PolarStepperForward(objbuf *bytes.Buffer, radius, theta, azimuth, azimuthInc, azimuthIncBottom, thetaInc float64) {
	x1, y1, z1 := PolarToCartesian(radius, theta, azimuth)
	objbuf.WriteString(fmt.Sprintf("v %v %v %v \n", x1, y1, z1))
	x2, y2, z2 := PolarToCartesian(radius, theta, azimuth+azimuthIncBottom) // increase azimuth
	objbuf.WriteString(fmt.Sprintf("v %v %v %v \n", x2, y2, z2))
	x3, y3, z3 := PolarToCartesian(radius, theta-thetaInc, azimuth+azimuthInc) // increase azimuth and height
	objbuf.WriteString(fmt.Sprintf("v %v %v %v \n", x3, y3, z3))

	x4, y4, z4 := PolarToCartesian(radius, theta-thetaInc, azimuth) // increase height
	objbuf.WriteString(fmt.Sprintf("v %v %v %v \n", x4, y4, z4))
	fmt.Println(ThreeDistance(x1, x4, y1, y4, z1, z4))

	//	fmt.Println(math.Sqrt(math.Pow((x2)-x1, 2)+math.Pow((y2)-y1, 2)) + math.Pow((z2)-z1, 2))
	fmt.Println("up", ThreeDistance(x1, x2, y1, y2, z1, z2))
	fmt.Println("up", ThreeDistance(x2, x3, y2, y3, z2, z3))
	fmt.Println("up", ThreeDistance(x3, x4, y3, y4, z3, z4))

	//vt positoin
	/*
		objbuf.WriteString(fmt.Sprintf("vt 0 0 \n"))
		objbuf.WriteString(fmt.Sprintf("vt 1 0 \n"))
		objbuf.WriteString(fmt.Sprintf("vt 1 1 \n"))
		objbuf.WriteString(fmt.Sprintf("vt 0 1\n"))
	*/

}

func PolarStepperBackward(objbuf *bytes.Buffer, radius, theta, azimuth, azimuthInc, azimuthIncTop, thetaInc float64) {
	x1, y1, z1 := PolarToCartesian(radius, theta, azimuth-azimuthInc)
	objbuf.WriteString(fmt.Sprintf("v %v %v %v \n", x1, y1, z1))
	x2, y2, z2 := PolarToCartesian(radius, theta, azimuth) // increase azimuth
	objbuf.WriteString(fmt.Sprintf("v %v %v %v \n", x2, y2, z2))
	x3, y3, z3 := PolarToCartesian(radius, theta-thetaInc, azimuth) // increase azimuth and height
	objbuf.WriteString(fmt.Sprintf("v %v %v %v \n", x3, y3, z3))

	x4, y4, z4 := PolarToCartesian(radius, theta-thetaInc, azimuth-azimuthIncTop) // increase height
	objbuf.WriteString(fmt.Sprintf("v %v %v %v \n", x4, y4, z4))

	/*
		fmt.Println(math.Sqrt(math.Pow((x2)-x1, 2)+math.Pow((y2)-y1, 2)) + math.Pow((z2)-z1, 2))
		fmt.Println("up", math.Sqrt(math.Pow((x4)-x1, 2)+math.Pow((y4)-y1, 2))+math.Pow((z4)-z1, 2))
		fmt.Println("up", math.Sqrt(math.Pow((x3)-x2, 2)+math.Pow((y3)-y2, 2))+math.Pow((z3)-z2, 2))
	*/

	/*
		objbuf.WriteString(fmt.Sprintf("vt 0 0 \n"))
		objbuf.WriteString(fmt.Sprintf("vt 1 0 \n"))
		objbuf.WriteString(fmt.Sprintf("vt 1 1 \n"))
		objbuf.WriteString(fmt.Sprintf("vt 0 1\n"))
	*/
}

func ThreeDistance(x1, x2, y1, y2, z1, z2 float64) float64 {
	return math.Sqrt(math.Pow((x1)-x2, 2) + math.Pow(y1-y2, 2) + math.Pow(z1-z2, 2))
}

/*
width is x plane
depth is y plane
height is z plane
*/
func GenHalfCubeOBJ(wObj io.Writer, wTsig io.Writer, tileHeight, tileWidth float64, CubeWidth, CubeHeight, CubeDepth float64, dx, dy float64) error {

	// check the dimensions
	if int(math.Ceil(CubeWidth/tileWidth)) != int(CubeWidth/tileWidth) {
		return fmt.Errorf("cubewidth of %v does", CubeWidth)
	}

	if int(math.Ceil(CubeHeight/tileHeight)) != int(CubeHeight/tileHeight) {
		return fmt.Errorf("tile height of %v is not an integer multiple of a cube height of %v", tileHeight, CubeHeight)
	}

	// start at 0,0

	pixelWidth := (CubeWidth + CubeDepth*2) * dx
	pixelHeight := (CubeDepth*2 + CubeHeight) * dy

	// rows * column
	leftRight := int((CubeDepth * 2 / tileWidth) * (CubeHeight / tileHeight))
	boots := int((CubeDepth * 2 / tileWidth) * (CubeWidth / tileHeight))
	back := int((CubeHeight / tileHeight) * (CubeWidth / tileWidth))

	fmt.Println(leftRight, boots, back)

	tiles := make([]Tilelayout, leftRight+boots+back)

	// calculate the uv ,ap

	uStep := tileWidth / (CubeWidth + CubeDepth*2)
	vStep := tileHeight / (CubeDepth*2 + CubeHeight)

	type plane struct {
		iEnd, jEnd     float64
		iStep, jStep   float64
		iStart, jStart float64
		uStart, vStart float64
		// the plane value that isn't moved
		planeConst float64
		plane      string
		inverse    bool
	}

	planes := []plane{
		// left wall
		{iEnd: CubeDepth, jEnd: CubeHeight, iStep: tileWidth, jStep: tileHeight, planeConst: 0, plane: "y", vStart: (CubeDepth / tileHeight) * vStep, inverse: true, uStart: ((CubeDepth + CubeWidth) / tileWidth) * uStep},
		// right wall
		{iEnd: CubeDepth, jEnd: CubeHeight, iStep: tileWidth, jStep: tileHeight, planeConst: CubeWidth, plane: "y", vStart: (CubeDepth / tileHeight) * vStep},

		// back wall
		{iEnd: CubeWidth, jEnd: CubeHeight, iStep: tileWidth, jStep: tileHeight, planeConst: CubeDepth, plane: "x", vStart: (CubeDepth / tileHeight) * vStep, uStart: ((CubeDepth) / tileWidth) * uStep},

		// Top
		{iEnd: CubeDepth, jEnd: CubeWidth, iStep: tileWidth, jStep: tileHeight, planeConst: CubeHeight, plane: "z", vStart: ((CubeDepth + CubeHeight) / tileHeight) * vStep, uStart: ((CubeDepth) / tileWidth) * uStep},

		// Bottom
		{iEnd: CubeDepth, jEnd: CubeWidth, iStep: tileWidth, jStep: tileHeight, planeConst: 0, plane: "z", uStart: ((CubeDepth) / tileWidth) * uStep},
	}
	count := 1
	tCount := 0

	for _, p := range planes {

		uTotal := (p.iEnd - p.iStart) / p.iStep
		width := uTotal * uStep

		ujTotal := (p.jEnd - p.jStart) / p.iStep
		ujwidth := ujTotal * uStep
		iCount := 0
		for i := p.iStart; i < p.iEnd; i += p.iStep {

			jCount := 0
			for j := p.jStart; j < p.jEnd; j += p.jStep {

				switch p.plane {
				case "x":

					wObj.Write([]byte(fmt.Sprintf("v %v %v %v \n", p.planeConst, i, j)))
					wObj.Write([]byte(fmt.Sprintf("v %v %v %v \n", p.planeConst, i+p.iStep, j)))
					wObj.Write([]byte(fmt.Sprintf("v %v %v %v \n", p.planeConst, i+p.iStep, j+p.jStep)))
					wObj.Write([]byte(fmt.Sprintf("v %v %v %v \n", p.planeConst, i, j+p.jStep)))

					wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", p.uStart+width-float64(iCount)*uStep, p.vStart+float64(jCount)*vStep)))
					wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", p.uStart+width-float64(iCount+1)*uStep, p.vStart+float64(jCount)*vStep)))
					wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", p.uStart+width-float64(iCount+1)*uStep, p.vStart+float64(jCount+1)*vStep)))
					wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", p.uStart+width-float64(iCount)*uStep, p.vStart+float64(jCount+1)*vStep)))

				case "y":
					wObj.Write([]byte(fmt.Sprintf("v %v %v %v \n", i, p.planeConst, j)))
					wObj.Write([]byte(fmt.Sprintf("v %v %v %v \n", i+p.iStep, p.planeConst, j)))
					wObj.Write([]byte(fmt.Sprintf("v %v %v %v \n", i+p.iStep, p.planeConst, j+p.jStep)))
					wObj.Write([]byte(fmt.Sprintf("v %v %v %v \n", i, p.planeConst, j+p.jStep)))

					// if inversed change the direction of the uv map
					if p.inverse {
						wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", p.uStart+width-float64(iCount)*uStep, p.vStart+float64(jCount)*vStep)))
						wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", p.uStart+width-float64(iCount+1)*uStep, p.vStart+float64(jCount)*vStep)))
						wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", p.uStart+width-float64(iCount+1)*uStep, p.vStart+float64(jCount+1)*vStep)))
						wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", p.uStart+width-float64(iCount)*uStep, p.vStart+float64(jCount+1)*vStep)))

					} else {

						wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", p.uStart+float64(iCount)*uStep, p.vStart+float64(jCount)*vStep)))
						wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", p.uStart+float64(iCount+1)*uStep, p.vStart+float64(jCount)*vStep)))
						wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", p.uStart+float64(iCount+1)*uStep, p.vStart+float64(jCount+1)*vStep)))
						wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", p.uStart+float64(iCount)*uStep, p.vStart+float64(jCount+1)*vStep)))
					}

				case "z":
					wObj.Write([]byte(fmt.Sprintf("v %v %v %v \n", i, j+p.jStep, p.planeConst)))
					wObj.Write([]byte(fmt.Sprintf("v %v %v %v \n", i, j, p.planeConst)))
					wObj.Write([]byte(fmt.Sprintf("v %v %v %v \n", i+p.iStep, j, p.planeConst)))
					wObj.Write([]byte(fmt.Sprintf("v %v %v %v \n", i+p.iStep, j+p.jStep, p.planeConst)))

					wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", p.uStart+ujwidth-float64(jCount+1)*uStep, p.vStart+float64(iCount)*vStep)))
					wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", p.uStart+ujwidth-float64(jCount)*uStep, p.vStart+float64(iCount)*vStep)))
					wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", p.uStart+ujwidth-float64(jCount)*uStep, p.vStart+float64(iCount+1)*vStep)))
					wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", p.uStart+ujwidth-float64(jCount+1)*uStep, p.vStart+float64(iCount+1)*vStep)))

				default:
					continue
				}

				tiles[tCount] = Tilelayout{Layout: Positions{Flat: XY{X: int((p.uStart + width - float64(iCount)*uStep) * pixelWidth), Y: int((1 - (p.vStart + float64(jCount+1)*vStep)) * pixelHeight)}, Size: XY{X: int(dx), Y: int(dy)}}}

				wObj.Write([]byte(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3)))
				count += 4
				jCount++
				tCount++
			}
			iCount++
		}
	}
	tsig := TPIG{Tilelayout: tiles, Dimensions: Dimensions{Flat: XY2D{X0: 0, X1: int(pixelWidth), Y0: 0, Y1: int(pixelHeight)}}}

	enc := json.NewEncoder(wTsig)
	enc.SetIndent("", "    ")

	return enc.Encode(tsig)

}

// Angle in radians
func GenCurveOBJ(wObj io.Writer, wTsig io.Writer, tileHeight, tileWidth float64, cylinderRadius, cylinderHeight, azimuthMaxAngle float64, dx, dy float64) {
	// r phi z
	azimuthInc := (2 * math.Asin(tileWidth/(2*cylinderRadius)))

	z := 0.0
	count := 1
	azimuth := -azimuthMaxAngle

	pixelWidth := math.Ceil(2*azimuthMaxAngle/azimuthInc) * dx
	pixelHeight := math.Ceil(cylinderHeight/tileHeight) * dy

	// rows * column
	tiles := make([]Tilelayout, int(math.Ceil(2*azimuthMaxAngle/azimuthInc)*math.Ceil(cylinderHeight/tileHeight)))

	uWidth := 1 / (math.Ceil(2 * azimuthMaxAngle / azimuthInc))
	vheight := 1 / math.Ceil(cylinderHeight/tileHeight)
	v := 0.0

	i := 0
	for z < cylinderHeight {
		u := 1.0

		for azimuth <= azimuthMaxAngle {
			//	tileCount++

			// get angle change

			x1, y1, z1 := CylindricalToCartesian(cylinderRadius, z, azimuth)
			wObj.Write([]byte(fmt.Sprintf("v %v %v %v \n", x1, y1, z1)))
			wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", u, v)))

			x2, y2, z2 := CylindricalToCartesian(cylinderRadius, z, azimuth+azimuthInc) // increase azimuth
			wObj.Write([]byte(fmt.Sprintf("v %v %v %v \n", x2, y2, z2)))
			wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", u-uWidth, v)))

			x3, y3, z3 := CylindricalToCartesian(cylinderRadius, z+tileHeight, azimuth+azimuthInc) // increase azimuth and height
			wObj.Write([]byte(fmt.Sprintf("v %v %v %v \n", x3, y3, z3)))
			wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", u-uWidth, v+vheight)))

			x4, y4, z4 := CylindricalToCartesian(cylinderRadius, z+tileHeight, azimuth) // increase height
			wObj.Write([]byte(fmt.Sprintf("v %v %v %v \n", x4, y4, z4)))
			wObj.Write([]byte(fmt.Sprintf("vt %v %v \n", u, v+vheight)))

			wObj.Write([]byte(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3)))

			azimuth += azimuthInc
			u -= uWidth
			count += 4

			tiles[i] = Tilelayout{Layout: Positions{Flat: XY{X: int(u * pixelWidth), Y: int((1 - (v + vheight)) * pixelHeight)}, Size: XY{X: int(dx), Y: int(dy)}}}

			i++
		}

		// increase the z height
		// as well as the uv map height
		v += vheight
		z += tileHeight
		azimuth = -azimuthMaxAngle
	}
	tsig := TPIG{Tilelayout: tiles, Dimensions: Dimensions{Flat: XY2D{X0: 0, X1: int(pixelWidth), Y0: 0, Y1: int(pixelHeight)}}}

	enc := json.NewEncoder(wTsig)
	enc.SetIndent("", "    ")
	enc.Encode(tsig)

}

// Angle in radians
func GenSphereOBJ(w io.Writer, tileHeight, tileWidth float64, sphereRadius, thetaMaxAngle, azimuthMaxAngle float64) {

	azimuth, clockAz := 0.0, 0.0
	// tileCount := 0
	theta := math.Pi / 2
	count := 1

	thetaInc := 2 * (math.Asin(tileHeight / (2 * sphereRadius)))
	azimuthInc := (2 * math.Asin(tileWidth/(2*sphereRadius)))
	theta = (math.Pi / 2)

	uWidth := 1 / (2 * math.Ceil(azimuthMaxAngle/azimuthInc))
	vheight := 1 / (2 * math.Ceil(thetaMaxAngle/thetaInc))

	/*
		tileX := tileWidth / 0.001 //consitent dy dx for the moment
		tileY := tileHeight / 0.001
		sizeX := tileX * (2 * math.Ceil(azimuthMaxAngle/azimuthInc))
		sizeY := tileY * (2 * math.Ceil(thetaMaxAngle/thetaInc))
	*/

	//sizex := 3840
	/*
	   generate the size of the bas eimage then move erveything along
	*/
	// TOP
	v := 0.5
	for theta > (math.Pi/2)-thetaMaxAngle {
		//start Point :=
		topLeftThet := theta - thetaInc
		topLeftAz := azimuth
		botLeftAz := azimuth
		u := 0.5
		prevUshift := 0.0

		for azimuth < azimuthMaxAngle {
			//	tileCount++

			// get angle change
			azimuthInc := (2 * math.Asin(tileWidth/(2*sphereRadius))) / math.Sin(theta)
			azimuthIncTop := (2 * math.Asin(tileWidth/(2*sphereRadius))) / math.Sin(topLeftThet)

			x1, y1, z1 := PolarToCartesian(sphereRadius, topLeftThet+thetaInc, topLeftAz)
			w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x1, y1, z1)))

			d := 2 * sphereRadius * math.Sin(topLeftAz-botLeftAz)

			shift := int(d / (0.0015625))
			ushift := float64(shift) * (1.0 / 3840.0)
			w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(u+ushift+prevUshift), v)))                  // this U needs to shift to the right
			x2, y2, z2 := PolarToCartesian(sphereRadius, topLeftThet+thetaInc, topLeftAz+azimuthInc) // increase azimuth
			w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x2, y2, z2)))
			w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(u+uWidth+ushift+prevUshift), v)))

			x3, y3, z3 := PolarToCartesian(sphereRadius, topLeftThet, topLeftAz+azimuthIncTop) // increase azimuth and height
			w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x3, y3, z3)))
			w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(u+uWidth), v+vheight)))

			x4, y4, z4 := PolarToCartesian(sphereRadius, topLeftThet, topLeftAz) // increase height
			w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x4, y4, z4)))
			w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(u), v+vheight)))

			w.Write([]byte(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3)))

			//			fmt.Println("4", 1-(u), "3", 1-(u+uWidth))
			//			fmt.Println("shift", ushift, prevUshift)
			botLeftAz = topLeftAz + azimuthInc
			azimuth += azimuthIncTop
			topLeftAz = azimuth
			count += 4
			u += uWidth
			prevUshift = ushift
		}

		topRightAz := clockAz
		u = 0.5
		for clockAz > -thetaMaxAngle {
			//	tileCount++

			azimuthInc := (2 * math.Asin(tileWidth/(2*sphereRadius))) / math.Sin(theta)
			azimuthIncTop := (2 * math.Asin(tileWidth/(2*sphereRadius))) / math.Sin(topLeftThet)
			x1, y1, z1 := PolarToCartesian(sphereRadius, topLeftThet+thetaInc, topRightAz)
			w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x1, y1, z1)))
			w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-u, v)))

			x2, y2, z2 := PolarToCartesian(sphereRadius, topLeftThet+thetaInc, topRightAz-azimuthInc) // increase azimuth
			w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x2, y2, z2)))
			w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(u-uWidth), v)))

			x3, y3, z3 := PolarToCartesian(sphereRadius, topLeftThet, topRightAz-azimuthIncTop) // increase azimuth and height
			w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x3, y3, z3)))
			w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(u-uWidth), v+vheight)))

			x4, y4, z4 := PolarToCartesian(sphereRadius, topLeftThet, topRightAz) // increase height
			w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x4, y4, z4)))
			w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-u, v+vheight)))

			w.Write([]byte(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3)))

			//	objbuf.WriteString(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3))
			clockAz -= azimuthIncTop
			topRightAz = clockAz
			count += 4
			u -= uWidth
		}

		theta -= thetaInc
		azimuth = 0
		clockAz = 0
		v += vheight
		//fmt.Println("COINTER", theta, z, zinchold)
		//	z = zinchold

	}

	// Bottom
	v = 0.5
	theta = math.Pi / 2
	for theta < (math.Pi/2)+thetaMaxAngle {
		//start Point :=
		botLeftThet := theta + thetaInc
		botLeftAz := azimuth
		u := 0.5
		for azimuth < thetaMaxAngle {
			// tileCount++

			azimuthInc := (2 * math.Asin(tileWidth/(2*sphereRadius))) / math.Sin(theta)
			azimuthIncBot := (2 * math.Asin(tileWidth/(2*sphereRadius))) / math.Sin(botLeftThet)
			x1, y1, z1 := PolarToCartesian(sphereRadius, botLeftThet-thetaInc, botLeftAz)
			w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x1, y1, z1)))
			w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-u, v)))

			x2, y2, z2 := PolarToCartesian(sphereRadius, botLeftThet-thetaInc, botLeftAz+azimuthInc) // increase azimuth
			w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x2, y2, z2)))
			w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(u+uWidth), v)))

			x3, y3, z3 := PolarToCartesian(sphereRadius, botLeftThet, botLeftAz+azimuthIncBot) // increase azimuth and height
			w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x3, y3, z3)))
			w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(u+uWidth), v-vheight)))

			x4, y4, z4 := PolarToCartesian(sphereRadius, botLeftThet, botLeftAz) // increase height
			w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x4, y4, z4)))
			w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-u, v-vheight)))

			//	fmt.Println(math.Sqrt(math.Pow((x2)-x1, 2)+math.Pow((y2)-y1, 2)) + math.Pow((z2)-z1, 2))

			w.Write([]byte(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3)))

			//	objbuf.WriteString(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3))
			azimuth += azimuthIncBot
			botLeftAz = azimuth
			count += 4
			u += uWidth
		}

		botRightAz := clockAz
		u = 0.5
		for clockAz > -thetaMaxAngle {
			// tileCount++

			azimuthInc := (2 * math.Asin(tileWidth/(2*sphereRadius))) / math.Sin(theta)
			azimuthIncTop := (2 * math.Asin(tileWidth/(2*sphereRadius))) / math.Sin(botLeftThet)
			x1, y1, z1 := PolarToCartesian(sphereRadius, botLeftThet-thetaInc, botRightAz)
			w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x1, y1, z1)))
			w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-u, v)))

			x2, y2, z2 := PolarToCartesian(sphereRadius, botLeftThet-thetaInc, botRightAz-azimuthInc) // increase azimuth
			w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x2, y2, z2)))
			w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(u-uWidth), v)))

			x3, y3, z3 := PolarToCartesian(sphereRadius, botLeftThet, botRightAz-azimuthIncTop) // increase azimuth and height
			w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x3, y3, z3)))
			w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-(u-uWidth), v-vheight)))

			x4, y4, z4 := PolarToCartesian(sphereRadius, botLeftThet, botRightAz) // increase height
			w.Write([]byte(fmt.Sprintf("v %v %v %v \n", x4, y4, z4)))
			w.Write([]byte(fmt.Sprintf("vt %v %v \n", 1-u, v-vheight)))

			w.Write([]byte(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3)))

			//	objbuf.WriteString(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3))
			clockAz -= azimuthIncTop
			botRightAz = clockAz
			count += 4
			u -= uWidth
		}

		v -= vheight
		theta += thetaInc
		azimuth = 0
		clockAz = 0
		//fmt.Println("COINTER", theta, z, zinchold)
		//	z = zinchold

	}
}
