# An Introduction to the Examples

The examples are presented in no particular order and only briefly highlight
things of interest, while it should not be considered a guide to programming
with Jipmunk it should give a useful insight.


### The example menu

By executing:-
<pre>
ant example.ExampleMenu
</pre>
within the root of the jipmunk project you will be presented with a
convienient menu with which you can select which demo to run. Just select
the name of the demo from the required and click go!


### BouncyHexagons

This shows that by using a large collection of small segment shapes it is
possible to create complex and even concave collision shapes.


### Convex

In addition to the normal ability to drag object with the mouse button
you can also add aditional vertexes to the polygon.  All the really clever
stuff happens inside the utility class ConvexHullUtil provided by jipmunk.


### LogoSmash

The first time I saw Chipmunk this really impressed me, no interaction here
just enjoy.


### Planet

This is a really neat example, it's only the boxes initial velocity and their
attraction to the planet object that has any influence here so they are
genuinely orbiting!  The example demonstrates using a BodyVelocityFunc class
which is used to provide custom gravity vector (always in the direction
of the planet and not just in a single direction)

This to try, if you're very careful you can throw a box back into orbit.
Speeding up the planet (try 2!) boxes stacking above a certain height on
the planet with be thrown back into orbit...

### Plink

No real interaction required.  Shows creation of PolyShape shapes, also
shows use for the SpaceBodyIteratorFunc class to iterate over all the
bodies in a space.  Unless a small subset of bodies in your application
have a different and specific behavior there is often no need to keep
your own lists of bodies.


### Pump

This show the use of a number of different joint (constraint) types, also
by using the layers bit pattern it is possible to prevent certain parts
of the pump from colliding with itself.  Use the arrow keys to control the
speed of the pump.  This should inspire any budding Rube Goldberg's out
there!


### PyramidStack

This is just a simple stack of boxes that would be stable if it wasn't
for that ball...
Because of its simplicity its probably a good point to start experimenting
from.


### PyramidTopple

Jipmunk is capable of stable stacking even of precarious structures.
Something to try is running the simulation in slow motion (change the
step delta time) and nudge a central box, it can be interesting to see
how a collapse propagates.


### Tumble

Like the Planet example this uses a rogue body to create a rotating shape
that is not effected by the rest of the simulation.  The boxes within it
are effected by this shape and so are tumbled about.
