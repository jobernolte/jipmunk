/**
 * <h1>Intro</h1> First of all, Chipmunk2D is a 2D rigid body physics library distributed under the MIT license. It is
 * intended to be blazingly fast, portable, numerically stable, and easy to use. For this reason it's been used in
 * hundreds of games across just about every system you can name. This includes top quality titles such as Night Sky for
 * the Wii and many #1 sellers on the iPhone App Store! I've put thousands of hours of work over many years to make
 * Chipmunk2D what it is today. If you find Chipmunk2D has saved you a lot of time, please consider
 * "donating":https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=6666552. You'll make an indie game
 * developer very happy!
 *
 * First of all, I would like to give a Erin Catto a big thank you, as Chipmunk2D's impulse solver was directly inspired
 * by his example code way back in 2006. (Now a full fledged physics engine all it's own:
 * <a href="www.box2d.org">Box2D.org</a>). His contact persistence idea allows for stable stacks of objects with very few
 * iterations of the solver. My previous solver produced mushy piles of objects or required a large amount of CPU to
 * operate stably.
 *
 * <h1>jipmunk Basics</h1>
 *
 * <h2>Overview</h2>
 * There are 4 basic object types you will use in Chipmunk.
 * <ul>
 * <li>*Rigid Bodies:* A rigid body holds the physical properties of an object. (mass, position, rotation, velocity, etc.) It does not have a shape until you attach one or more collision shapes to it. If you’ve done physics with particles before, rigid bodies differ in that they are able to rotate. Rigid bodies generally tend to have a 1:1 correlation to sprites in a game. You should structure your game so that you use the position and rotation of the rigid body for drawing your sprite.</li>
 * <li>*Collision Shapes:* By attaching shapes to bodies, you can define the a body’s shape. You can attach as many shapes to a single body as you need to in order to define a complex shape. Shapes contain the surface properties of an object such as how much friction or elasticity it has.</li>
 * <li>*Constraints/Joints:* Constraints and joints describe how bodies are attached to each other.</li>
 * <li>*Spaces:* Spaces are containers for simulating objects in Chipmunk. You add bodies, shapes and joints to a space and then update the space as a whole. They control how all the rigid bodies, shapes, and constraints interact together.</li>
 * </ul>
 * There is often confusion between rigid bodies and their collision shapes in Chipmunk and how they relate to sprites.
 * A sprite would be a visual representation of an object, while a collision shape is an invisible property that defines
 * how objects should collide. Both the sprite's and the collision shape's position and rotation are controlled by the
 * motion of a rigid body. Generally you want to create a game object type that ties these things all together.
 *
 * <h2>Basic Types</h2>
 * <ul>
 * <li>{@link org.physics.jipmunk.CollisionType}: Unique identifier for collision shape types.</li>
 * <li>{@link org.physics.jipmunk.Group}: Unique identifier for collision groups. A {@link org.physics.jipmunk.Group#NO_GROUP} value is defined that can be used when you don't want to specify a group.</li>
 * <li>{@link org.physics.jipmunk.Bitmask}: Type used as the layers bitmask. A {@link org.physics.jipmunk.Constants#ALL_LAYERS} value is defined that has all layer bits set.</li>
 * </ul> 
 * If you are writting a game engine or language binding on top of Chipmunk, you might want to choose to use object 
 * references instead of integers for collision type and group. I often use class pointers for collision types and game 
 * object pointers for groups. It's much simpler than keeping a table of enumerations around.
 *
 * <h1>Chipmunk Axis Aligned Bounding Boxes: {@link org.physics.jipmunk.BB}</h1>
 * <h2>Struct Definition and Constructors:</h2>
 * {@link org.physics.jipmunk.BB} - Simple bounding box struct. Stored as left, bottom, right, top values.
 * <p>{@link org.physics.jipmunk.BB#BB(float, float, float, float)} - Convenience constructor for {@link org.physics.jipmunk.BB} structs.
 * <p>{@link org.physics.jipmunk.BB#forCircle(Vector2f, float)} - Convenience constructor for making a {@link org.physics.jipmunk.BB} fitting a circle at a given position and radius.
 *
 * <h2>Operations:</h2>
 * <ul>
 *     <li>{@link org.physics.jipmunk.BB#intersects(BB)} - Returns true if the bounding boxes intersect.</li>
 *     <li>{@link org.physics.jipmunk.BB#contains(BB)} - Returns true if the bounding box completely contains another bounding box.</li>
 *     <li>{@link org.physics.jipmunk.BB#contains(Vector2f)} - Returns true if the bounding box contains a vector.</li>
 *     <li>{@link org.physics.jipmunk.BB#merge(BB)} - Return the minimal bounding box that contains both bounding boxes.</li>
 *     <li>{@link org.physics.jipmunk.BB#expand(Vector2f)} - Return the minimal bounding box that contains both the bounding box and the vector.</li>
 *     <li>{@link org.physics.jipmunk.BB#getCenter()} - Return the center of the bounding box.</li>
 *     <li>{@link org.physics.jipmunk.BB#area()} - Return the area of the bounding box.</li>
 *     <li>{@link org.physics.jipmunk.BB#mergedArea(BB)} - Merges the bounding boxes and returns the area of the merged bounding box.</li>
 *     <li>{@link org.physics.jipmunk.BB#segmentQuery(Vector2f, Vector2f)} - Returns the fraction along the segment query the bounding box is hit. Returns {@link java.lang.Float#POSITIVE_INFINITY} if it doesn't hit.</li>
 *     <li>{@link org.physics.jipmunk.BB#intersectsSegment(Vector2f, Vector2f)} - Returns true if the segment defined by the endpoints intersect the bounding box.</li>
 *     <li>{@link org.physics.jipmunk.BB#clampVect(Vector2f)} - Returns a copy of the vector clamped to the bounding box.</li>
 *     <li>{@link org.physics.jipmunk.BB#wrapVect(Vector2f)} - Returns a copy of the vector wrapped to the bounding box.</li>
 * </ul>
 *
 *
 * <h1>Chipmunk Rigid Bodies: {@link org.physics.jipmunk.Body}</h1>
 * <h2>Rogue and Static Bodies:</h2>
 * Normally when you create a rigid body, you add it to a space so the space will start simulating it. This means it
 * will update it's position and velocity, apply forces to it, be affected by gravity, etc. A body that isn't added to a
 * space (and not simulated) is called a <i>rogue body</i>. The most important use for rogue bodies are as static bodies, but
 * you can also use them to implement directly controlled objects such as moving platforms.
 * <p/>
 * Static bodies are rogue bodies, but with a special flag set on them to let Chipmunk know that they never move unless
 * you tell it. Static bodies have two purposes. Originally they were added for the sleeping feature. Because static
 * bodies don't move, Chipmunk knows that it's safe to let objects that are touching or jointed to them fall asleep.
 * Objects touching or jointed to regular rogue bodies are never allowed to sleep. The second purpose for static bodies
 * is that Chipmunk knows shapes attached to them never need to have their collision detection data updated. Chipmunk
 * also doesn't need to bother checking for collisions between static objects. Generally all of your level geometry will
 * be attached to a static body except for things like moving platforms or doors.
 * <p/>
 * In previous versions of Chipmunk before 5.3 you would create an infinite mass rogue body to attach static shapes to
 * using {@link org.physics.jipmunk.Space#addShape(Shape)}. You don't need to do any of that anymore, and shouldn't if
 * you want to use the sleeping feature. Each space has a dedicated static body that you can use to attach your static
 * shapes to. Chipmunk also automatically adds shapes attached to static bodies as static shapes.
 */
package org.physics.jipmunk;