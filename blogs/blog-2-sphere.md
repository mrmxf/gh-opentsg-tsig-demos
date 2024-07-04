# Spheres squares and sadness

Insert hook, say if you missed the first check it out here.

After doing some easier shapes in blog 1
I went back to the task at hand, dealing with a sphere.

A new shape meant another new coordinate system, this time we're using
[spherical coordinates][sc]. This means we now have two angles to contend
with, as we move our square tiles around to construct a shape.

We go back to using the chordal length formula again,
this time with a little tweak to accommodate moving in both angles.
To calculate the angle for change in inclination we can use the same
chordal length formula as last time.

*piccy*

Now for the azimuth, because our cross sectioninal circle that we calculate the
chordal length shrinks as the inlcination nears closer to the poles we need a new formual to 
accommodate this.

*piccy2*

Now we have our inclination steps we can start putting the tiles down,
Each new inclination we calculate the azimuth step. Increasing the top and the bottom of the tile 
by this step. This was my first mistake as the tiles overlapped each othe in the corners.
We need to calculate teh azimuth change at the top and bottom of the tile differently
as the inlclination angle changes the result slightly.
After doing this we ended up with wonky tiles, why? it turns out the angles
race away from each other so every new tile increases the wonkyness.

Wehad to fix this by starting the bottom where the top stopped, giving us a gap
of tiles where we miss the pixels, but this is as close we'll get with squares.

Now onto UV maps, how on earth are we going to get this  square on a sqaure shape.
The first step was just a uv map the resembles the rough idea.







Now to make the uv map, I started with a simpler this is the rough shape of the image thats on the tile.
Made it look like so, to account for the drop in the pixels as you go down the tile and the pixel gap
between the tiles. Looks something like *insert image*

Now to get it pixel perfect, after wrestling with finding ways to flatten it, even delving back
into cylindrical coordinates in my desperation. Eventually realised the first uv map can be approximated
as lines of pixels that start in the same place. So split the tiles into rows where the pixel
start point is approximatley the same. Split into rows so each row had a section on the uv map.

This is where I really learned the importance of a uv map checker such as link and copyrights etc.
Helped get everyhting lined up and inverted it all. - GO back and checked all the results from blog 1 and
had to make fixes to some inverted uv maps that were not caught by gradients of the initial
ebu3373 pattern i used as a test.

Thank you for reading put it all into practive here link to repo


[sc]: https://en.wikipedia.org/wiki/Spherical_coordinate_system