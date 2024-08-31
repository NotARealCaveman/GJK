# GJK

This is a 3D implementation of GJK in c++ using barycentric coordinates and voronoi inspection to determine if two objects are colliding, as well as their closest features and the distance between them.

The GJK Function will return true if it detects two objects are NOT colliding. This function is meant to be used in a broader collision detection system. The GJK Function should first be checked with the point(s) representing geometry as well as the convex hull to be tested against. A result of true indicates that the center of the object is not inside of the convex hull and indicates the test for a shallow contact must be performed. The distance computed by the algorithm is sufficient for the shallow contact test. If this test succeed, the simplex configured by the algorithm contains the nessecary information to compute the closest points on each object.

Example Usage:

SphereVHull(Sphere sphere, Hull hull)\
&emsp;Simplex_T<Support> simplex;\
&emsp;Distance d;\
&emsp;if(GJK(sphere,hull,simplex,d))\
&emsp;&emsp;if(d <= sphere.radius^2)\
&emsp;&emsp;&emsp;//shallow contact stuff\
&emsp;else\
&emsp;&emsp;//deep contact stuff\
