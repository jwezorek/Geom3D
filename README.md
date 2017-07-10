# Geom3D

A little repo in response to the fact that there is no good C# library for doing even basic computational geometry library and/or 3D math. 

There is one called Math.Net Numerics along with Math.Net Spatial that will get you vectors and matrices but not with all 
the convenience routines you’d expect to treat vectors like 3D points and so forth. 

To this end this rwpo is merely what you get by extracting the vectors and matrices out of monogame and search-and-replacing 
“float” to “double” to get double precision. 

I also included in here 
  * 3D line segment/line segment intersection code and 
  * 3D triangle/triangle intersection code 
which I transliterated to C#. 

The line segment intersection code came from [Paul Bourke’s web site](http://paulbourke.net/geometry/pointlineplane/#i2l). The triangle intersection code came from running [T
omas Moller’s C code](http://fileadmin.cs.lth.se/cs/personal/tomas_akenine-moller/code/) through just a C preprocessor to resolve all the macros and then transliterating the result to C#.

