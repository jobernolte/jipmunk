package org.physics.jipmunk;

import java.util.ArrayList;
import java.util.List;

import static org.physics.jipmunk.Arbiter.cpArbiterApplyCachedImpulse;
import static org.physics.jipmunk.Arbiter.cpArbiterApplyImpulse;
import static org.physics.jipmunk.Arbiter.cpArbiterCallSeparate;
import static org.physics.jipmunk.Arbiter.cpArbiterPreStep;
import static org.physics.jipmunk.Array.cpArrayDeleteObj;
import static org.physics.jipmunk.Array.cpArrayPush;
import static org.physics.jipmunk.Assert.cpAssertSoft;
import static org.physics.jipmunk.Assert.cpAssertWarn;
import static org.physics.jipmunk.BBTree.cpBBTreeSetVelocityFunc;
import static org.physics.jipmunk.Body.cpBodyIsSleeping;
import static org.physics.jipmunk.Body.cpBodyIsStatic;
import static org.physics.jipmunk.Body.cpBodyRemoveConstraint;
import static org.physics.jipmunk.Collision.cpCollideShapes;
import static org.physics.jipmunk.HashSet.cpHashSetFilter;
import static org.physics.jipmunk.SpaceComponent.cpBodyActivate;
import static org.physics.jipmunk.SpaceComponent.cpBodyActivateStatic;
import static org.physics.jipmunk.SpaceComponent.cpSpaceProcessComponents;
import static org.physics.jipmunk.SpaceQuery.cpSpacePointQuery;
import static org.physics.jipmunk.SpaceQuery.cpSpacePointQueryFirst;
import static org.physics.jipmunk.SpaceQuery.cpSpaceSegmentQuery;
import static org.physics.jipmunk.SpaceQuery.cpSpaceSegmentQueryFirst;
import static org.physics.jipmunk.SpatialIndex.cpSpatialIndexEach;
import static org.physics.jipmunk.SpatialIndex.cpSpatialIndexInsert;
import static org.physics.jipmunk.SpatialIndex.cpSpatialIndexRemove;
import static org.physics.jipmunk.Util.cpBBIntersects;
import static org.physics.jipmunk.Util.cpfpow;

/** @author jobernolte */
public class Space {
	/// Number of iterations to use in the impulse solver to solve contacts.
	int iterations = 10;

	/** Gravity to pass to rigid bodies when integrating velocity. */
	Vector2f gravity = Util.cpvzero();

	/// Damping rate expressed as the fraction of velocity bodies retain each second.
	/// A value of 0.9 would mean that each body's velocity will drop 10% per second.
	/// The default value is 1.0, meaning no damping is applied.
	/// @note This damping value is different than those of cpDampedSpring and cpDampedRotarySpring.
	private float damping = 1;

	/**
	 * Speed threshold for a body to be considered idle. The default value of 0 means to let the space guess a good
	 * threshold based on gravity.
	 */
	float idleSpeedThreshold = 0;

	/**
	 * Time a group of bodies must remain idle in order to fall asleep. Enabling sleeping also implicitly enables the the
	 * contact graph. The default value of INFINITY disables the sleeping algorithm.
	 */
	float sleepTimeThreshold = Float.POSITIVE_INFINITY;

	/// Amount of encouraged penetration between colliding shapes..
	/// Used to reduce oscillating contacts and keep the collision cache warm.
	/// Defaults to 0.1. If you have poor simulation quality,
	/// increase this number as much as possible without allowing visible amounts of overlap.
	private float collisionSlop = 0.1f;

	/// Determines how fast overlapping shapes are pushed apart.
	/// Expressed as a fraction of the error remaining after each second.
	/// Defaults to pow(1.0 - 0.1, 60.0) meaning that Chipmunk fixes 10% of overlap each frame at 60Hz.
	private float collisionBias = (float) Math.pow(1.0 - 0.1, 60);

	/// Number of frames that contact information should persist.
	/// Defaults to 3. There is probably never a reason to change this value.
	private int collisionPersistence = 3;

	/// Rebuild the contact graph during each step. Must be enabled to use the cpBodyEachArbiter() function.
	/// Disabled by default for a small performance boost. Enabled implicitly when the sleeping feature is enabled.
	private boolean enableContactGraph = false;

	private int stamp;
	List<Body> bodies = new ArrayList<Body>();
	private List<Body> rousedBodies = new ArrayList<Body>();
	List<Body> sleepingComponents = new ArrayList<Body>();
	int locked = 0;
	private LongHashMap<CollisionHandlerEntry> collisionHandlers = new LongHashMap<CollisionHandlerEntry>();
	private final static SpatialIndexBBFunc<Shape> shapeGetBBFunc = new SpatialIndexBBFunc<Shape>() {
		@Override
		public BB apply(Shape obj) {
			return obj.getBb();
		}
	};
	SpatialIndex<Shape> staticShapes = new BBTree<Shape>(shapeGetBBFunc, null);
	SpatialIndex<Shape> activeShapes = new BBTree<Shape>(shapeGetBBFunc, staticShapes);
	List<Arbiter> arbiters = new ArrayList<Arbiter>();
	private LongHashMap<Arbiter> cachedArbiters = new LongHashMap<Arbiter>();
	private Pool<Arbiter> pooledArbiters = new Pool<Arbiter>() {
		@Override
		protected Arbiter create() {
			return new Arbiter();
		}
	};
	private float prev_dt;
	private List<PostStepFunc> postStepCallbacks;
	private Body staticBody;
	List<Constraint> constraints = new ArrayList<Constraint>();
	private IntHashMap<Shape> shapeIds = new IntHashMap<Shape>();
	private int lastShapeId = 0;
	private float curr_dt;

	public Space() {
		cpBBTreeSetVelocityFunc(activeShapes, new BBTreeVelocityFunc<Shape>() {
			@Override
			public Vector2f velocity(Shape obj) {
				return obj.body.v;
			}
		});
		this.staticBody = Body.createStatic();
	}

	public void setIterations(int iterations) {
		this.iterations = iterations;
	}

	public int getIterations() {
		return iterations;
	}

	/**
	 * Global gravity applied to the space. Can be overridden on a per body basis by writing custom integration functions.
	 *
	 * @param gravity the gravity applied to the space
	 */
	public void setGravity(final Vector2f gravity) {
		this.gravity.set(gravity);
	}

	/** @return Global gravity applied to the space. Defaults to cpvzero. */
	public Vector2f getGravity() {
		return gravity;
	}

	public Body getStaticBody() {
		return staticBody;
	}

	public void setSleepTimeThreshold(float sleepTimeThreshold) {
		this.sleepTimeThreshold = sleepTimeThreshold;
	}

	public float getSleepTimeThreshold() {
		return sleepTimeThreshold;
	}

	public float getDamping() {
		return damping;
	}

	public void setDamping(float damping) {
		this.damping = damping;
	}

	public float getCollisionSlop() {
		return collisionSlop;
	}

	public void setCollisionSlop(float collisionSlop) {
		this.collisionSlop = collisionSlop;
	}

	public void addCollisionHandler(int a, int b, CollisionHandler handler) {
		assertSpaceUnlocked();

		long key = (((long) a) << 32) | b;

		// Remove any old function so the new one will get added.
		removeCollisionHandler(a, b);
		if (handler != null) {
			final CollisionHandlerEntry handlerEntry = new CollisionHandlerEntry(handler, a, b);
			collisionHandlers.put(key, handlerEntry);
			collisionHandlers.put((((long) b) << 32) | a, handlerEntry);
		}
	}

	public void removeCollisionHandler(int a, int b) {
		long key = (((long) a) << 32) | b;
		collisionHandlers.remove(key);
		collisionHandlers.remove((((long) b) << 32) | a);
	}

	private void assertSpaceUnlocked() {
		assert locked == 0;
	}

	private void assignShapeId(Shape shape) {

		while (shapeIds.get(lastShapeId) != null) {
			lastShapeId++;
		}
		shape.hashid = lastShapeId++;
		shapeIds.put(shape.hashid, shape);

	}

	private void revokeShapeId(Shape shape) {
		shapeIds.remove(shape.hashid);
		shape.hashid = -1;
	}

	public Shape addShape(Shape shape) {
		Body body = shape.body;
		if (body.isStatic()) {
			return addStaticShape(shape);
		}

		// TODO change these to check if it was added to a space at all.
		assert shape.space == null : "This shape is already added to a space and cannot be added to another.";
		assertSpaceUnlocked();

		assignShapeId(shape);

		body.activate();
		body.addShape(shape);

		shape.update(body.p, body.rot);
		activeShapes.insert(shape, shape.hashid);
		shape.space = this;

		return shape;
	}

	public Shape addStaticShape(Shape shape) {
		assert shape.space == null : "This shape is already added to a space and cannot be added to another.";
		assertSpaceUnlocked();

		assignShapeId(shape);

		Body body = shape.body;
		body.addShape(shape);
		shape.update(body.p, body.rot);
		staticShapes.insert(shape, shape.hashid);
		shape.space = this;

		return shape;
	}

	public Body addBody(Body body) {
		assert !body.isStatic() : "Static bodies cannot be added to a space as they are not meant to be simulated.";
		assert body.space == null : "This body is already added to a space and cannot be added to another.";
		assertSpaceUnlocked();

		bodies.add(body);
		body.space = this;

		return body;
	}

	public Constraint addConstraint(Constraint constraint) {
		cpAssertSoft(constraint.space == null, "This shape is already added to a space and cannot be added to " +
				"another.");
		assertSpaceUnlocked();

		cpBodyActivate(constraint.a);
		cpBodyActivate(constraint.b);
		cpArrayPush(this.constraints, constraint);

		// Push onto the heads of the bodies' constraint lists
		Body a = constraint.a, b = constraint.b;
		constraint.next_a = a.constraintList;
		a.constraintList = constraint;
		constraint.next_b = b.constraintList;
		b.constraintList = constraint;
		constraint.space = this;

		return constraint;
	}

	public void removeShape(Shape shape) {
		if (shape.space != this) {
			throw new IllegalArgumentException("Cannot remove a shape that was not added to the space. (Removed " +
					"twice maybe?)");
		}
		Body body = shape.body;
		if (cpBodyIsStatic(body)) {
			removeStaticShape(shape);
		} else {
			assert containsShape(shape) : "Cannot remove a shape that was not added to the space. (Removed twice " +
					"maybe?)";
			assertSpaceUnlocked();

			cpBodyActivate(body);
			body.removeShape(shape);
			filterArbiters(body, shape);
			cpSpatialIndexRemove(this.activeShapes, shape, shape.hashid);
			shape.space = null;
			revokeShapeId(shape);
		}
	}

	public void removeStaticShape(Shape shape) {
		cpAssertSoft(containsShape(shape),
				"Cannot remove a static or sleeping shape that was not added to the space. (Removed twice maybe?)");
		assertSpaceUnlocked();
		if (shape.space != this) {
			throw new IllegalArgumentException("Cannot remove a shape that was not added to the space. (Removed " +
					"twice maybe?)");
		}

		Body body = shape.body;
		if (body.isStatic()) {
			cpBodyActivateStatic(body, shape);
		}
		body.removeShape(shape);
		filterArbiters(body, shape);
		cpSpatialIndexRemove(this.staticShapes, shape, shape.hashid);
		shape.space = null;
		revokeShapeId(shape);
	}

	public void removeBody(Body body) {
		cpAssertWarn(containsBody(body),
				"Cannot remove a body that was not added to the space. (Removed twice maybe?)");
		assertSpaceUnlocked();
		if (body.space != this) {
			throw new IllegalArgumentException("Cannot remove a body that was not added to the space. " +
					"(Removed twice maybe?)");
		}

		cpBodyActivate(body);
		filterArbiters(body, null);
		cpArrayDeleteObj(this.bodies, body);
		body.space = null;
	}

	public void removeConstraint(Constraint constraint) {
		cpAssertWarn(containsConstraint(constraint),
				"Cannot remove a constraint that was not added to the space. (Removed twice maybe?)");
		assertSpaceUnlocked();

		cpBodyActivate(constraint.a);
		cpBodyActivate(constraint.b);
		cpArrayDeleteObj(this.constraints, constraint);

		cpBodyRemoveConstraint(constraint.a, constraint);
		cpBodyRemoveConstraint(constraint.b, constraint);
		constraint.space = null;
	}

	public boolean containsShape(Shape shape) {
		return (shape.space == this);
	}

	public boolean containsBody(Body body) {
		return (body.space == this);
	}

	public boolean containsConstraint(Constraint constraint) {
		return (constraint.space == this);
	}

	public void reindexStatic() {
		staticShapes.each(new SpatialIndexIteratorFunc<Shape>() {
			@Override
			public void visit(Shape obj) {
				Body body = obj.body;
				obj.update(body.p, body.rot);
			}
		});
		staticShapes.reindex();
	}

	/**
	 * Update the collision detection data for a specific shape in the space.
	 *
	 * @param shape the shape to update
	 */
	public void reindexShape(Shape shape) {
		Body body = shape.body;
		shape.update(body.p, body.rot);

		// attempt to rehash the shape in both hashes
		activeShapes.reindexObject(shape, shape.hashid);
		staticShapes.reindexObject(shape, shape.hashid);
	}

	public void reindexShapesForBody(Body body) {
		for (Shape shape : body.shapes()) {
			reindexShape(shape);
		}
	}

	public void eachBody(final SpaceBodyIteratorFunc func) {
		cpSpaceLock(this);
		{
			for (Body body : bodies) {
				func.visit(body);
			}
			for (Body root : sleepingComponents) {
				for (Body body : root.components()) {
					func.visit(body);
				}
			}
		}
		cpSpaceUnlock(this, true);
	}

	public void eachShape(final SpaceShapeIteratorFunc func) {
		cpSpaceLock(this);
		{
			activeShapes.each(new SpatialIndexIteratorFunc<Shape>() {
				@Override
				public void visit(Shape obj) {
					func.visit(obj);
				}
			});
			staticShapes.each(new SpatialIndexIteratorFunc<Shape>() {
				@Override
				public void visit(Shape obj) {
					func.visit(obj);
				}
			});
		}
		cpSpaceUnlock(this, true);
	}

	public void eachConstraint(final SpaceConstraintIteratorFunc func) {
		cpSpaceLock(this);
		{
			for (Constraint constraint : constraints) {
				func.visit(constraint);
			}
		}
		cpSpaceUnlock(this, true);
	}

	CollisionHandler lookupHandler(int collision_type_a, int collision_type_b) {
		long key = (((long) collision_type_a) << 32) | collision_type_b;
		CollisionHandlerEntry entry = collisionHandlers.get(key);
		return entry != null ? entry.handler : defaultCollisionHandlerEntry.handler;
	}

	CollisionHandlerEntry lookupHandlerEntry(int collision_type_a, int collision_type_b) {
		long key = (((long) collision_type_a) << 32) | collision_type_b;
		CollisionHandlerEntry handler = collisionHandlers.get(key);
		return handler == null ? defaultCollisionHandlerEntry : handler;
	}

	static class ArbiterFilterContext {
		Space space;
		Body body;
		Shape shape;

		ArbiterFilterContext(Space space, Body body, Shape shape) {
			this.space = space;
			this.body = body;
			this.shape = shape;
		}
	}

	static boolean cachedArbitersFilter(Arbiter arb, ArbiterFilterContext context) {
		/*Body body = context.body;
				if (body == arb.body_a || body == arb.body_b) {
					context.space.arbiters.remove(arb);
					context.space.pooledArbiters.free(arb);
					return false;
				}

				return true;*/
		Shape shape = context.shape;
		Body body = context.body;

		// Match on the filter shape, or if it's NULL the filter body
		if (
				(body == arb.body_a && (shape == arb.a || shape == null)) ||
						(body == arb.body_b && (shape == arb.b || shape == null))
				) {
			// Call separate when removing shapes.
			if (shape != null && arb.state != ArbiterState.cpArbiterStateCached)
				cpArbiterCallSeparate(arb, context.space);

			arb.unthread();
			context.space.arbiters.remove(arb);
			context.space.pooledArbiters.free(arb);

			return false;
		}

		return true;
	}

	static long hashPair(int a, int b) {
		return (((long) a) << 32) | b;
	}

	void uncacheArbiter(Arbiter arb) {
		Shape a = arb.a, b = arb.b;
		//Shape[] shape_pair = {a, b};
		long arbHashID = hashPair(a.hashid, b.hashid);
		//cpHashSetRemove(this.cachedArbiters, arbHashID, shape_pair);
		cachedArbiters.remove(arbHashID);
		arbiters.remove(arb);
	}

	void filterArbiters(Body body, Shape filter) {
		final ArbiterFilterContext context = new ArbiterFilterContext(this, body, filter);
		cpHashSetFilter(cachedArbiters, new HashSetFilterFunc<Arbiter>() {
			@Override
			public boolean filter(Arbiter value) {
				return cachedArbitersFilter(value, context);
			}
		});
	}

	void activateBody(Body body) {
		assert !body.isRogue() : "Internal error: Attempting to activate a rogue body.";

		if (locked != 0) {
			// cpSpaceActivateBody() is called again once the space is unlocked
			if (!rousedBodies.contains(body)) {
				cpArrayPush(rousedBodies, body);
			}
		} else {
			cpArrayPush(bodies, body);

			for (Shape shape : body.shapes()) {
				cpSpatialIndexRemove(this.staticShapes, shape, shape.hashid);
				cpSpatialIndexInsert(this.activeShapes, shape, shape.hashid);
			}

			for (Arbiter arb : body.arbiters()) {
				Body bodyA = arb.body_a;
				if (body == bodyA || cpBodyIsStatic(bodyA)) {
					/*TODO int numContacts = arb.numContacts;
										Contact[] contacts = arb.contacts;

										// Restore contact data back to the space's contact buffer memory
										arb.contacts = Arrays.copyOf(contacts, numContacts);*/

					// Reinsert the arbiter into the arbiter cache
					Shape a = arb.a, b = arb.b;
					//Shape[] shape_pair = {a, b};

					long arbHashID = hashPair(a.hashid, b.hashid);
					//cpHashSetInsert(this.cachedArbiters, arbHashID, shape_pair, arb, NULL);
					cachedArbiters.put(arbHashID, arb);

					// Update the arbiter's state
					arb.stamp = this.stamp;
					arb.handler = lookupHandlerEntry(a.collision_type, b.collision_type);
					cpArrayPush(this.arbiters, arb);
				}
			}

			//CP_BODY_FOREACH_CONSTRAINT(body, constraint)
			for (Constraint constraint : body.constraints()) {
				Body bodyA = constraint.a;
				if (body == bodyA || cpBodyIsStatic(bodyA)) cpArrayPush(this.constraints, constraint);
			}
		}
	}

	static boolean queryReject(Shape a, Shape b) {
		return (
				// BBoxes must overlap
				!cpBBIntersects(a.bb, b.bb)
						// Don't collide shapes attached to the same body.
						|| a.body == b.body
						// Don't collide objects in the same non-zero group
						|| (a.group != 0 && a.group == b.group)
						// Don't collide objects that don't share at least on layer.
						|| (a.layers & b.layers) == 0
		);
	}

	CollisionHandler defaultCollisionHandler = new CollisionHandler() {
		@Override
		public boolean begin(Arbiter arb, Space space) {
			return true;
		}

		@Override
		public boolean preSolve(Arbiter arb, Space space) {
			return true;
		}

		@Override
		public void postSolve(Arbiter arb, Space space) {
		}

		@Override
		public void separate(Arbiter arb, Space space) {
		}
	};

	CollisionHandlerEntry defaultCollisionHandlerEntry = new CollisionHandlerEntry(defaultCollisionHandler, 0, 0);

	private ContactList contactList = new ContactList();

	// Callback from the spatial hash.
	void collideShapes(Shape a, Shape b) {
		// Reject any of the simple cases
		if (queryReject(a, b)) {
			return;
		}

		CollisionHandlerEntry handler = lookupHandlerEntry(a.collision_type, b.collision_type);

		boolean sensor = a.sensor || b.sensor;
		if (sensor && handler == defaultCollisionHandler) {
			return;
		}

		// Shape 'a' should have the lower shape type. (required by cpCollideShapes() )
		if (a.getType().ordinal() > b.getType().ordinal()) {
			Shape temp = a;
			a = b;
			b = temp;
		}

		// Narrow-phase collision detection.
		//Contact contacts = cpContactBufferGetArray(space);
		//ContactList contacts = new ContactList();
		ContactList contacts = contactList;
		int numContacts = cpCollideShapes(a, b, contacts);
		assert numContacts == contacts.size();
		if (numContacts == 0) return; // Shapes are not colliding.

		// cpSpacePushContacts(space, numContacts);

		// Get an arbiter from this.arbiterSet for the two shapes.
		// This is where the persistant contact magic comes from.
		//Shape shape_pair[] = {a, b};
		long arbHashID = hashPair(a.hashid, b.hashid);
		Arbiter arb = cachedArbiters.get(arbHashID);
		if (arb == null) {
			arb = pooledArbiters.alloc();
			arb.init(a, b);
			cachedArbiters.put(arbHashID, arb);
		}
		arb.update(contacts, numContacts, handler, a, b);
		contacts.clear();

		// Call the begin function first if it's the first step
		if (arb.state == ArbiterState.cpArbiterStateFirstColl && !handler.begin(arb, this)) {
			arb.ignore(); // permanently ignore the collision until separation
		}

		if (
			// Ignore the arbiter if it has been flagged
				(arb.state != ArbiterState.cpArbiterStateIgnore) &&
						// Call preSolve
						handler.preSolve(arb, this) &&
						// Process, but don't add collisions for sensors.
						!sensor
				) {
			cpArrayPush(arbiters, arb);
		} else {
			//cpSpacePopContacts(space, numContacts);

			arb.contacts = null;
			arb.numContacts = 0;

			// Normally arbiters are set as used after calling the post-solve callback.
			// However, post-solve callbacks are not called for sensors or arbiters rejected from pre-solve.
			if (arb.state != ArbiterState.cpArbiterStateIgnore) arb.state = ArbiterState.cpArbiterStateNormal;
		}

		// Time stamp the arbiter so we know it was used recently.
		arb.stamp = this.stamp;
	}

	// Hashset filter func to throw away old arbiters.
	boolean arbiterSetFilter(Arbiter arb) {
		int ticks = this.stamp - arb.stamp;

		Body a = arb.body_a, b = arb.body_b;

		// TODO should make an arbiter state for this so it doesn't require filtering arbiters for dangling body
		// pointers on body removal.
		// Preserve arbiters on sensors and rejected arbiters for sleeping objects.
		if ((cpBodyIsStatic(a) || cpBodyIsSleeping(a)) && (cpBodyIsStatic(b) || cpBodyIsSleeping(b))) {
			return true;
		}

		// Arbiter was used last frame, but not this one
		if (ticks >= 1 && arb.state != ArbiterState.cpArbiterStateCached) {
			cpArbiterCallSeparate(arb, this);
			arb.state = ArbiterState.cpArbiterStateCached;
		}

		if (ticks >= this.collisionPersistence) {
			arb.contacts = null;
			arb.numContacts = 0;

			//cpArrayPush(pooledArbiters, arb);
			pooledArbiters.free(arb);
			return false;
		}

		return true;
	}

	public void step(float dt) {
		if (dt == 0.0f) return; // don't step if the timestep is 0!

		this.stamp++;

		float prev_dt = this.curr_dt;
		this.curr_dt = dt;

		// Reset and empty the arbiter list.
		//cpArray *arbiters = this.arbiters;
		for (Arbiter arb : arbiters) {
			arb.state = ArbiterState.cpArbiterStateNormal;

			// If both bodies are awake, unthread the arbiter from the contact graph.
			if (!cpBodyIsSleeping(arb.body_a) && !cpBodyIsSleeping(arb.body_b)) {
				arb.unthread();
			}
		}
		arbiters.clear();

		// Integrate positions
		for (Body body : bodies) {
			body.positionFunc.position(body, dt);
		}

		// Find colliding pairs.
		cpSpaceLock(this);
		{
			// cpSpacePushFreshContactBuffer(space);
			cpSpatialIndexEach(this.activeShapes, new SpatialIndexIteratorFunc<Shape>() {
				@Override
				public void visit(Shape shape) {
					Body body = shape.body;
					shape.update(body.p, body.rot);
				}
			});
			this.activeShapes.reindexQuery(new SpatialReIndexQueryFunc<Shape>() {
				@Override
				public void apply(Shape obj1, Shape obj2) {
					collideShapes(obj1, obj2);
				}
			});
		}
		cpSpaceUnlock(this, false);

		// If body sleeping is enabled, do that now.
		if (this.sleepTimeThreshold != Float.POSITIVE_INFINITY || this.enableContactGraph) {
			cpSpaceProcessComponents(this, dt);
		}

		// Clear out old cached arbiters and call separate callbacks
		cpHashSetFilter(this.cachedArbiters, new HashSetFilterFunc<Arbiter>() {
			@Override
			public boolean filter(Arbiter value) {
				return arbiterSetFilter(value);
			}
		});

		// Prestep the arbiters and constraints.
		float slop = this.collisionSlop;
		float biasCoef = 1.0f - cpfpow(this.collisionBias, dt);
		for (int i = 0; i < arbiters.size(); i++) {
			cpArbiterPreStep(arbiters.get(i), dt, slop, biasCoef);
		}

		for (Constraint constraint : constraints) {
			ConstraintPreSolveFunc preSolve = constraint.getPreSolveFunc();
			if (preSolve != null) {
				preSolve.preSolve(constraint, this);
			}
			constraint.preStep(dt);
		}

		// Integrate velocities.
		float damping = cpfpow(this.damping, dt);
		//cpVect gravity = this.gravity;
		for (int i = 0; i < bodies.size(); i++) {
			Body body = bodies.get(i);
			body.velocityFunc.velocity(body, gravity, damping, dt);
		}

		// Apply cached impulses
		float dt_coef = (prev_dt == 0.0f ? 0.0f : dt / prev_dt);
		for (int i = 0; i < arbiters.size(); i++) {
			cpArbiterApplyCachedImpulse(arbiters.get(i), dt_coef);
		}

		for (Constraint constraint : constraints) {
			constraint.applyCachedImpulse(dt_coef);
		}

		// Run the impulse solver.
		for (int i = 0; i < this.iterations; i++) {
			for (int j = 0; j < arbiters.size(); j++) {
				cpArbiterApplyImpulse(arbiters.get(j));
			}
			for (Constraint constraint : constraints) {
				constraint.applyImpulse();
			}
		}

		// Run the constraint post-solve callbacks
		for (Constraint constraint : constraints) {
			ConstraintPostSolveFunc postSolve = constraint.getPostSolveFunc();
			if (postSolve != null) {
				postSolve.postSolve(constraint, this);
			}

		}

		// run the post-solve callbacks
		cpSpaceLock(this);
		for (int i = 0; i < arbiters.size(); i++) {
			Arbiter arb = arbiters.get(i);

			CollisionHandlerEntry handler = arb.handler;
			handler.postSolve(arb, this);
		}
		cpSpaceUnlock(this, false);

		cpSpaceRunPostStepCallbacks(this);
	}

	static void cpSpaceLock(Space space) {
		space.locked++;
	}

	static void cpSpaceUnlock(Space space, boolean runPostStep) {
		space.locked--;
		assert space.locked >= 0 : "Internal Error: Space lock underflow.";

		if (space.locked == 0) {
			List<Body> waking = space.rousedBodies;
			for (int i = 0, count = waking.size(); i < count; i++) {
				space.activateBody(waking.get(i));
			}

			waking.clear();
			if (runPostStep) {
				cpSpaceRunPostStepCallbacks(space);
			}
		}
	}

	static void cpSpaceRunPostStepCallbacks(Space space) {
		// Loop because post step callbacks may add more post step callbacks directly or indirectly.
		while (space.postStepCallbacks != null) {
			List<PostStepFunc> callbacks = space.postStepCallbacks;
			space.postStepCallbacks = null;
			for (PostStepFunc callback : callbacks) {
				callback.call(space);
			}
		}
	}

	static void cpSpaceUncacheArbiter(Space space, Arbiter arb) {
		space.uncacheArbiter(arb);
	}

	/**
	 * Query <code>space</code> at <code>point</code> filtering out matches with the given <code>layers</code> and
	 * <code>group</code>. <code>func</code> is called for each shape found along with the data argument passed to
	 * pointQuery(). Sensor shapes are included.
	 *
	 * @param point  the query point
	 * @param layers the layers to include
	 * @param group  the group to check
	 * @param func   the callback function
	 */
	public void pointQuery(final Vector2f point, int layers, int group, SpacePointQueryFunc func) {
		cpSpacePointQuery(this, point, layers, group, func);
	}

	/**
	 * Query <code>space</code> at <code>point</code> and return the first shape found matching the given
	 * <code>layers</code> and <code>group</code>. Returns <code>null</code> if no shape was found. Sensor shapes are
	 * ignored.
	 *
	 * @param point  the query point
	 * @param layers the layers to include
	 * @param group  the group to check
	 * @return the shape or <code>null</code>
	 */
	public Shape pointQueryFirst(final Vector2f point, int layers, int group) {
		return cpSpacePointQueryFirst(this, point, layers, group);
	}

	/**
	 * Add <code>func</code> to be called before {@link Space#step(float)} returns. You can add post step callbacks from
	 * outside of other callback functions, but there isn?t a good reason to and they won?t be called until the next time
	 * {@link Space#step(float)} is finishing.
	 *
	 * @param func the post step callback to add
	 */
	public void addPostStepCallback(PostStepFunc func) {
		cpAssertWarn(locked != 0,
				"Adding a post-step callback when the space is not locked is unnecessary. " +
						"Post-step callbacks will not called until the end of the next call to cpSpaceStep() or the " +
						"next query.");

		if (postStepCallbacks == null) {
			postStepCallbacks = new ArrayList<PostStepFunc>(1);
		}
		postStepCallbacks.add(func);
	}

	public void setDefaultCollisionHandler(CollisionHandler handler) {
		defaultCollisionHandlerEntry.handler = handler != null ? handler : defaultCollisionHandler;
	}

	/**
	 * Perform a directed line segment query (like a raycast) against the space and return the first shape hit. Returns
	 * <code>null</code> if no shapes were hit.
	 *
	 * @param start  the start point of the segment
	 * @param end    the end point of the segment
	 * @param layers the layers to check
	 * @param group  the group to check
	 * @param out    a {@link SegmentQueryInfo} taking the result
	 * @return the first {@link Shape} hit or <code>null</code> if no shape was hit
	 */
	public Shape segmentQueryFirst(Vector2f start, Vector2f end, int layers, int group, SegmentQueryInfo out) {
		return cpSpaceSegmentQueryFirst(this, start, end, layers, group, out);
	}

	/**
	 * Query this space along the line segment from <code>start</code> to <code>end</code> filtering out matches with the
	 * given <code>layers</code> and <code>group</code>. <code>func</code> is called with the normalized distance along the
	 * line and surface normal for each shape found along. Sensor shapes are included.
	 *
	 * @param start  the start point of the segment
	 * @param end    the end point of the segment
	 * @param layers the layers to filter out
	 * @param group  the group to filter out
	 * @param func   a {@link SpaceSegmentQueryFunc} callback for each found shape
	 */
	public void segmentQuery(Vector2f start, Vector2f end, int layers, int group,
			SpaceSegmentQueryFunc func) {
		cpSpaceSegmentQuery(this, start, end, layers, group, func);
	}
}
