jipmunk
=======

Java port of the Chipmunk physics engine

Features
========

* Designed specifically for 2D video games.
* Circle, convex polygon, and beveled line segment collision primitives.
* Multiple collision primitives can be attached to a single rigid body.
* Fast broad phase collision detection by using a bounding box tree with great temporal coherence or a spatial hash.
* Extremely fast impulse solving by utilizing Erin Catto’s contact persistence algorithm.
* Supports sleeping objects that have come to rest to reduce the CPU load.
* Support for collision event callbacks based on user definable object types types.
* Flexible collision filtering system with layers, exclusion groups and callbacks.
* Can be used to create all sorts of effects like one way platforms or buoyancy areas. (Examples included)
* Supports point, segment (raycasting), shape and bounding box queries to the collision detection system.
* Impulses amounts can be read out for gameplay effects.
* Large variety of joints – easily make vehicles, ragdolls, and more.
* Joint callbacks.
* Can be used to easily implement breakable or animated joints. (Examples included)
* Maintains a contact graph of all colliding objects.
* Lightweight Java implementation with no external dependencies.
* Simple, read the documentation and see!
* Unrestrictive MIT license

Building
========

Ant is used as the build system. The buildfile is *build.xml*.