# Spheres squares and sadness

After doing some easier shapes I went back to the task at hand, dealing with a sphere.

To start we used spherical coordinates *insert*, this means we now have two angles to contend
with. Using the chordal length formula again, we can calculate the inclination angle increase
from the tile height and the azimuth angle increase from the tile width.

Start by doing this incrementing along the angles and we got pallelogram shaped tiles, something was going on, 
as the square tiles could not contort like that. Furthermore the distance along the angle lines
did not math the length of the tile. This was because the angular increase was not constant the
further from inclination = pi/2. So the chordal length had to be updated to account for the 3d sphere
to this. Also start from the centre and move out to keep a uniform design across the sphere. More unifrom uv map

Now we try again uh oh this doesn't quite work the tiles are still not quite square.
We have to account for how the tiles would behave with an increasing gap between them.
Join them in the corner increase the radius - it now look like whats expected.

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