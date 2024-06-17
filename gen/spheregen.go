package main

import (
	"bytes"
	"fmt"
	"math"
	"os"
	"slices"
)

func main() {

	sphere()
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
	tileSize := 0.1

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
			PolarStepperForward(objbuf, radius, theta, azimuth, azimuthInc, thetaInc)

			facebuf.WriteString(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3))

			//	objbuf.WriteString(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3))
			azimuth += azimuthInc
			count += 4
		}
		azimuth = 0.0 - (2*math.Asin(tileSize/(2*radius)))/math.Sin(theta)
		for azimuth > -maxTheta {
			tileCount++
			azimuthInc := (2 * math.Asin(tileSize/(2*radius))) / math.Sin(theta)
			PolarStepperForward(objbuf, radius, theta, azimuth, azimuthInc, thetaInc)

			facebuf.WriteString(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3))

			//	objbuf.WriteString(fmt.Sprintf("f %v/%v %v/%v %v/%v %v/%v\n", count, count, count+1, count+1, count+2, count+2, count+3, count+3))
			azimuth -= azimuthInc
			count += 4
		}

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
			objbuf.WriteString(fmt.Sprintf("vt %v %v\n", xmin, ymax))
			vt++
		}

		for i := 0; i < step; i++ {
			xmin, xmax := 0.5-float64(i+1)*widthstep, 0.5-float64(i)*(widthstep)
			objbuf.WriteString(fmt.Sprintf("vt %v %v \n", xmin, ymin))
			objbuf.WriteString(fmt.Sprintf("vt %v %v \n", xmax, ymin))
			objbuf.WriteString(fmt.Sprintf("vt %v %v \n", xmax, ymax))
			objbuf.WriteString(fmt.Sprintf("vt %v %v\n", xmin, ymax))
			vt++
		}
	}

	objbuf.Write(facebuf.Bytes())
	fmt.Println("count", count, ",", count/4, "VTcount", vt)
	f, _ := os.Create("./out/line.obj")
	f.Write(objbuf.Bytes())
}

// make a distance calculator
// math.Sqrt(math.Pow((z+xinc)-prevX, 2) + math.Pow((z+yinc)-prevY, 2)

func PolarToCartesian(r, theta, phi float64) (X, Y, Z float64) {
	X = r * math.Sin(theta) * math.Cos(phi)
	Y = r * math.Sin(theta) * math.Sin(phi)
	Z = r * math.Cos(theta)
	return
}

func PolarStepperForward(objbuf *bytes.Buffer, radius, theta, azimuth, azimuthInc, thetaInc float64) {
	x1, y1, z1 := PolarToCartesian(radius, theta, azimuth)
	objbuf.WriteString(fmt.Sprintf("v %v %v %v \n", x1, y1, z1))
	x2, y2, z2 := PolarToCartesian(radius, theta, azimuth+azimuthInc) // increase azimuth
	objbuf.WriteString(fmt.Sprintf("v %v %v %v \n", x2, y2, z2))
	x3, y3, z3 := PolarToCartesian(radius, theta-thetaInc, azimuth+azimuthInc) // increase azimuth and height
	objbuf.WriteString(fmt.Sprintf("v %v %v %v \n", x3, y3, z3))

	x4, y4, z4 := PolarToCartesian(radius, theta-thetaInc, azimuth) // increase height
	objbuf.WriteString(fmt.Sprintf("v %v %v %v \n", x4, y4, z4))

	/*
		fmt.Println(math.Sqrt(math.Pow((x2)-x1, 2)+math.Pow((y2)-y1, 2)) + math.Pow((z2)-z1, 2))
		fmt.Println("up", math.Sqrt(math.Pow((x4)-x1, 2)+math.Pow((y4)-y1, 2))+math.Pow((z4)-z1, 2))
		fmt.Println("up", math.Sqrt(math.Pow((x3)-x2, 2)+math.Pow((y3)-y2, 2))+math.Pow((z3)-z2, 2))
	*/

	//vt positoin
	/*
		objbuf.WriteString(fmt.Sprintf("vt 0 0 \n"))
		objbuf.WriteString(fmt.Sprintf("vt 1 0 \n"))
		objbuf.WriteString(fmt.Sprintf("vt 1 1 \n"))
		objbuf.WriteString(fmt.Sprintf("vt 0 1\n"))
	*/

}

func PolarStepperBackward(objbuf *bytes.Buffer, radius, theta, azimuth, azimuthInc, thetaInc float64) {
	x1, y1, z1 := PolarToCartesian(radius, theta, azimuth-azimuthInc)
	objbuf.WriteString(fmt.Sprintf("v %v %v %v \n", x1, y1, z1))
	x2, y2, z2 := PolarToCartesian(radius, theta, azimuth) // increase azimuth
	objbuf.WriteString(fmt.Sprintf("v %v %v %v \n", x2, y2, z2))
	x3, y3, z3 := PolarToCartesian(radius, theta-thetaInc, azimuth) // increase azimuth and height
	objbuf.WriteString(fmt.Sprintf("v %v %v %v \n", x3, y3, z3))

	x4, y4, z4 := PolarToCartesian(radius, theta-thetaInc, azimuth-azimuthInc) // increase height
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
