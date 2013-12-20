/*
 * Copyright (c) 2007 Scott Lembcke, (c) 2011 Jürgen Obernolte
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.physics.jipmunk;

import org.physics.jipmunk.impl.Collision;

import java.util.*;

import static org.physics.jipmunk.Array.cpArrayDeleteObj;
import static org.physics.jipmunk.Array.cpArrayPush;
import static org.physics.jipmunk.Assert.*;
import static org.physics.jipmunk.Body.*;
import static org.physics.jipmunk.HashSet.cpHashSetFilter;
import static org.physics.jipmunk.SpaceComponent.*;
import static org.physics.jipmunk.SpaceQuery.*;
import static org.physics.jipmunk.SpatialIndex.*;
import static org.physics.jipmunk.Util.cpfpow;
import static org.physics.jipmunk.Util.cpvzero;

/**
 * Spaces in Chipmunk are the basic unit of simulation. You add rigid dynamicBodies, shapes and constraints to it and
 * then step them all forward through time together. <point/> <b>What Are Iterations, and Why Should I care?</b>
 * <point/> Chipmunk uses an iterative solver to figure out the forces between objects in the space. What this means is
 * that it builds a big list of all of the collisions, joints, and other constraints between the dynamicBodies and makes
 * several passes over the list considering each one individually. The number of passes it makes is the iteration count,
 * and each iteration makes the solution more accurate. If you use too many iterations, the physics should look nice and
 * solid, but may use up too much CPU time. If you use too few iterations, the simulation may seem mushy or bouncy when
 * the objects should be solid. Setting the number of iterations lets you balance between CPU usage and the accuracy of
 * the physics. Chipmunk’s default of 10 iterations is sufficient for most simple games. <point/> <point/>
 * <b>Sleeping</b> <point/> New in Chipmunk is the ability of spaces to disable entire groups of objects that have
 * stopped moving to save CPU time as well as battery life. In order to use this feature you must do 2 things. The first
 * is that you must attach all your static geometry to static dynamicBodies. Objects cannot fall asleep if they are
 * touching a non-static rogue body even if it’s shapes were added as static shapes. The second is that you must enable
 * sleeping explicitly by choosing a time threshold value for cpSpace.sleepTimeThreshold. If you do not set {@link
 * Space#idleSpeedThreshold} explicitly, a value will be chosen automatically based on the current amount of gravity.
 *
 * @author jobernolte
 */
public class Space {

	private static final CollisionHandler DEFAULT_COLLISION_HANDLER = CollisionHandler.createDefaultHandler();
	// / Number of iterations to use in the impulse solver to solve contacts.
	int iterations = 10;
	/** Gravity to pass to rigid dynamicBodies when integrating velocity. */
	Vector2f gravity = Util.cpvzero();
	// / Damping rate expressed as the fraction of velocity dynamicBodies retain each second.
	// / A value of 0.9 would mean that each body's velocity will drop 10% per second.
	// / The default value is 1.0, meaning no damping is applied.
	// / @note This damping value is different than those of cpDampedSpring and cpDampedRotarySpring.
	private float damping = 1;
	/**
	 * Speed threshold for a body to be considered idle. The default value of 0 means to let the space guess a good
	 * threshold based on gravity.
	 */
	private float idleSpeedThreshold = 0;
	/**
	 * Time a group of dynamicBodies must remain idle in order to fall asleep. Enabling sleeping also implicitly enables
	 * the the contact graph. The default value of INFINITY disables the sleeping algorithm.
	 */
	float sleepTimeThreshold = Float.POSITIVE_INFINITY;
	/**
	 * Amount of encouraged penetration between colliding shapes.. Used to reduce oscillating contacts and keep the
	 * collision cache warm. Defaults to 0.1. If you have poor simulation quality, increase this number as much as
	 * possible without allowing visible amounts of overlap.
	 */
	private float collisionSlop = 0.1f;
	/**
	 * Determines how fast overlapping shapes are pushed apart. Expressed as a fraction of the error remaining after
	 * each second. Defaults to pow(1.0 - 0.1, 60.0) meaning that Chipmunk fixes 10% of overlap each frame at 60Hz.
	 */
	private float collisionBias = (float) Math.pow(1.0 - 0.1, 60);
	/**
	 * Number of frames that contact information should persist. Defaults to 3. There is probably never a reason to
	 * change this value.
	 */
	private int collisionPersistence = 3;
	private int stamp;
	private float curr_dt;
	List<Body> dynamicBodies = new LinkedList<>();
	List<Body> otherBodies = new LinkedList<>();
	private List<Body> rousedBodies = new LinkedList<>();
	List<Body> sleepingComponents = new LinkedList<>();
	SpatialIndex<Shape> staticShapes = new BBTree2<>(Shape::getBB, null);
	SpatialIndex<Shape> dynamicShapes = new BBTree2<>(Shape::getBB, staticShapes);
	List<Constraint> constraints = new ArrayList<>();
	List<Arbiter> arbiters = new ArrayList<>();
	// private Map<IdentityMapKey<Shape>, Arbiter> cachedArbiters = new HashMap<>();
	private LongHashMap<Arbiter> cachedArbiters = new LongHashMap<>();
	private Pool<Arbiter> pooledArbiters = new Pool<Arbiter>() {
		@Override
		protected Arbiter create() {
			return new Arbiter();
		}
	};
	int locked = 0;
	private boolean useWildcards;
	private IntHashMap<Shape> shapeIds = new IntHashMap<>();
	private int lastShapeId = 0;

	private static class CollisionHandlerMapKey {
		CollisionType typeA;
		CollisionType typeB;

		private CollisionHandlerMapKey(CollisionType typeA, CollisionType typeB) {
			this.typeA = typeA;
			this.typeB = typeB;
		}

		@Override
		public boolean equals(Object o) {
			if (this == o) {
				return true;
			}
			if (o == null || getClass() != o.getClass()) {
				return false;
			}

			CollisionHandlerMapKey that = (CollisionHandlerMapKey) o;

			return !(typeA != null ? !typeA.equals(that.typeA) : that.typeA != null) && !(typeB != null ?
					!typeB.equals(that.typeB) : that.typeB != null);

		}

		@Override
		public int hashCode() {
			int result = typeA != null ? typeA.hashCode() : 0;
			result = 31 * result + (typeB != null ? typeB.hashCode() : 0);
			return result;
		}
	}

	private Map<CollisionHandlerMapKey, CollisionHandler> collisionHandlers = new HashMap<>();
	private CollisionHandler defaultHandler = CollisionHandler.createDoNothingHandler();
	private boolean skipPostStep;
	private List<PostStepFunc> postStepCallbacks;
	private Body staticBody;

	public Space() {
		BBTree2.cpBBTreeSetVelocityFunc(dynamicShapes, obj -> obj.body.v);
		this.staticBody = Body.createStatic();
	}

	/**
	 * Iterations allow you to control the accuracy of the solver. Defaults to 10.
	 *
	 * @return the number of iterations used by the solver
	 */
	public int getIterations() {
		return iterations;
	}

	/**
	 * Iterations allow you to control the accuracy of the solver.
	 *
	 * @param iterations the number of iterations to be used by the solver
	 */
	public void setIterations(int iterations) {
		this.iterations = iterations;
	}

	/** @return the idle speed threshold being used */
	public float getIdleSpeedThreshold() {
		return idleSpeedThreshold;
	}

	/**
	 * Speed threshold for a body to be considered idle. The default value of 0 means to let the space guess a good
	 * threshold based on gravity.
	 *
	 * @param idleSpeedThreshold the idle speed threshold to be used
	 */
	public void setIdleSpeedThreshold(float idleSpeedThreshold) {
		this.idleSpeedThreshold = idleSpeedThreshold;
	}

	/** @return Global gravity applied to the space. Defaults to cpvzero. */
	public Vector2f getGravity() {
		return gravity;
	}

	/**
	 * Global gravity applied to the space. Can be overridden on a per body basis by writing custom integration
	 * functions.
	 *
	 * @param gravity the gravity applied to the space
	 */
	public void setGravity(final Vector2f gravity) {
		this.gravity.set(gravity);
	}

	/**
	 * A dedicated static body for the space. You don’alpha have to use it, but because it’s memory is managed
	 * automatically with the space it’s very convenient. You can set its user data pointer to something helpful if you
	 * want for callbacks.
	 *
	 * @return the dedicated static body for the space
	 */
	public Body getStaticBody() {
		return staticBody;
	}

	public void setStaticBody(Body body) {
		if (this.staticBody != null) {
			if (this.staticBody.getShapeList() != null) {
				throw new IllegalStateException(
						"Internal Error: Changing the designated static body while the old one still had shapes attached.");
			}
			this.staticBody.space = null;
		}

		this.staticBody = body;
		body.space = this;
	}

	/** @return the current sleep time threshold */
	public float getSleepTimeThreshold() {
		return sleepTimeThreshold;
	}

	/**
	 * Time a group of dynamicBodies must remain idle in order to fall asleep. The default value of {@link
	 * Float#POSITIVE_INFINITY} disables the sleeping feature.
	 *
	 * @param sleepTimeThreshold the sleep time threshold
	 */
	public void setSleepTimeThreshold(float sleepTimeThreshold) {
		this.sleepTimeThreshold = sleepTimeThreshold;
	}

	/** @return the damping being used */
	public float getDamping() {
		return damping;
	}

	/**
	 * Amount of simple damping to apply to the space. A value of 0.9 means that each body will lose 10% of it’s
	 * velocity per second. Defaults to 1. Like gravity can be overridden on a per body basis.
	 *
	 * @param damping the damping to be used
	 */
	public void setDamping(float damping) {
		this.damping = damping;
	}

	/** @return the amount of overlap that is allowed for collisions */
	public float getCollisionSlop() {
		return collisionSlop;
	}

	/**
	 * Amount of overlap between shapes that is allowed. It’s encouraged to set this as high as you can without
	 * noticable overlapping as it improves the stability. It defaults to 0.1.
	 *
	 * @param collisionSlop the amount of overlap that is allowed for collisions
	 */
	public void setCollisionSlop(float collisionSlop) {
		this.collisionSlop = collisionSlop;
	}

	/** @return the current collision bias */
	public float getCollisionBias() {
		return collisionBias;
	}

	/**
	 * Chipmunk allows fast moving objects to overlap, then fixes the overlap over time. Overlapping objects are
	 * unavoidable even if swept collisions are supported, and this is an efficient and stable way to deal with
	 * overlapping objects. The bias value controls what percentage of overlap remains unfixed after a second and
	 * defaults to ~0.2%. Valid values are in the range from 0 to 1, but using 0 is not recommended for stability
	 * reasons. The default value is calculated as cpfpow(1.0f - 0.1f, 60.0f) meaning that Chipmunk attempts to correct
	 * 10% of error ever 1/60th of a second. Note: Very very few games will need to change this value.
	 *
	 * @param collisionBias the collision bias
	 */
	public void setCollisionBias(float collisionBias) {
		this.collisionBias = collisionBias;
	}

	/** @return the current collision persistence (default is 3) */
	public int getCollisionPersistence() {
		return collisionPersistence;
	}

	/**
	 * The number of frames the space keeps collision solutions around for. Helps prevent jittering contacts from
	 * getting worse. This defaults to 3 and very very very few games will need to change this value.
	 *
	 * @param collisionPersistence the new collision persistence
	 */
	public void setCollisionPersistence(int collisionPersistence) {
		this.collisionPersistence = collisionPersistence;
	}

	/**
	 * Retrieves the current (if you are in a callback from {@link Space#step(float)}) or most recent (outside of a
	 * {@link Space#step(float)} call) timestep.
	 *
	 * @return the current timestep
	 */
	public float getCurrentTimeStep() {
		return curr_dt;
	}

	/**
	 * Returns <code>true</code> when in a callback meaning that you cannot add/remove objects from the space. Can be
	 * used to choose to create a post-step callback instead.
	 *
	 * @return <code>true</code> if the space is locked
	 */
	public boolean isLocked() {
		return locked > 0;
	}

	void useWildcardDefaultHandler() {
		// Spaces default to using the slightly faster "do nothing" default handler until wildcards are potentially needed.
		if (!this.useWildcards) {
			this.useWildcards = true;
			this.defaultHandler = DEFAULT_COLLISION_HANDLER;
		}
	}

	public CollisionHandler addDefaultCollisionHandler() {
		useWildcardDefaultHandler();
		return defaultHandler;
	}

	/**
	 * Set a collision handler to handle specific collision types. <point/> The methods are called only when shapes with
	 * the specified collision types collide. <point/> <code>typeA</code> and <code>typeB</code> should be the same
	 * references set to {@link Shape#getCollisionType()}. Add a collision handler for given collision type pair.
	 * <point/> Whenever a shapes with collision type <code>typeA</code> and collision type <code>typeB</code> collide,
	 * these callbacks will be used to process the collision. If you need to fall back on the space’s default callbacks,
	 * you’ll have to provide them individually to each handler definition.
	 *
	 * @param typeA collision type a
	 * @param typeB collision type b
	 * @return {@link CollisionHandler} to be used as callback
	 */

	public CollisionHandler addCollisionHandler(CollisionType typeA, CollisionType typeB) {
		assertSpaceUnlocked();
		CollisionHandlerMapKey key = new CollisionHandlerMapKey(typeA, typeB);
		CollisionHandler handler = collisionHandlers.get(key);
		if (handler == null) {
			handler = new CollisionHandler(typeA, typeB);
			handler.setBeginFunc(CollisionHandler::defaultBegin);
			handler.setPreSolveFunc(CollisionHandler::defaultPreSolve);
			handler.setPostSolveFunc(CollisionHandler::defaultPostSolve);
			handler.setSeparateFunc(CollisionHandler::defaultSeparate);
			collisionHandlers.put(key, handler);
		}
		return handler;
	}

	public CollisionHandler addWildcardHandler(CollisionType type) {
		useWildcardDefaultHandler();

		CollisionHandlerMapKey key = new CollisionHandlerMapKey(type, CollisionType.WILDCARD);
		CollisionHandler handler = collisionHandlers.get(key);
		if (handler == null) {
			handler = new CollisionHandler(type, CollisionType.WILDCARD);
			handler.setBeginFunc(CollisionHandler::alwaysCollide);
			handler.setPreSolveFunc(CollisionHandler::alwaysCollide);
			handler.setPostSolveFunc(CollisionHandler::doNothing);
			handler.setSeparateFunc(CollisionHandler::doNothing);
			collisionHandlers.put(key, handler);
		}
		return handler;
	}

	/**
	 * Removes a {@link CollisionHandler} for a given collision type pair.
	 *
	 * @param typeA collision type a
	 * @param typeB collision type b
	 */
	public void removeCollisionHandler(CollisionType typeA, CollisionType typeB) {
		CollisionHandlerMapKey key = new CollisionHandlerMapKey(typeA, typeB);
		collisionHandlers.remove(key);
	}

	private void assertSpaceUnlocked() {
		assert locked == 0;
	}

	private void assignShapeId(Shape shape) {

		while (shapeIds.get(lastShapeId) != null) {
			lastShapeId++;
		}
		shape.setHashId(lastShapeId++);
		shapeIds.put(shape.getHashId(), shape);

	}

	private void revokeShapeId(Shape shape) {
		shapeIds.remove(shape.getHashId());
		shape.setHashId(-1);
	}

	/**
	 * Adds the given shape to this space. Cannot be called from within a callback other than a {@link PostStepFunc}
	 * callback (which is different than a {@link CollisionHandler#postSolve(Arbiter, Space)} callback!). Attempting to
	 * add or remove objects from the space while {@link Space#step(float)} is still executing will throw an assertion.
	 *
	 * @param shape the {@link Shape} to add to this space
	 * @return the added shape
	 */
	public <T extends Shape> T addShape(T shape) {
		Body body = shape.getBody();

		if (shape.space == this) {
			throw new IllegalArgumentException(
					"You have already added this shape to this space. You must not add it a second time.");
		}
		if (shape.space != null) {
			throw new IllegalArgumentException(
					"You have already added this shape to another space. You cannot add it to a second.");
		}
		cpAssertSpaceUnlocked(this);

		boolean isStatic = body.isStatic();
		if (!isStatic) {
			body.activate();
		}
		body.addShape(shape);

		assignShapeId(shape);
		shape.update(body.transform);
		cpSpatialIndexInsert(isStatic ? this.staticShapes : this.dynamicShapes, shape, shape.getHashId());
		shape.space = this;

		return shape;
	}

	/**
	 * Adds the given body to this space. Cannot be called from within a callback other than a {@link PostStepFunc}
	 * callback (which is different than a {@link CollisionHandler#postSolve(Arbiter, Space)} callback!). Attempting to
	 * add or remove objects from the space while {@link Space#step(float)} is still executing will throw an assertion.
	 *
	 * @param body the {@link Body} to add to this space
	 * @return the added body
	 */
	public Body addBody(Body body) {
		if (body.space == this) {
			throw new IllegalArgumentException(
					"You have already added this body to this space. You must not add it a second time.");
		}
		if (body.space != null) {
			throw new IllegalArgumentException(
					"You have already added this body to another space. You cannot add it to a second.");
		}
		assertSpaceUnlocked();

		if (body.isDynamic()) {
			dynamicBodies.add(body);
		} else {
			otherBodies.add(body);
		}
		body.space = this;

		return body;
	}

	/**
	 * Adds the given constraint to this space. Cannot be called from within a callback other than a {@link
	 * PostStepFunc} callback (which is different than a {@link CollisionHandler#postSolve(Arbiter, Space)} callback!).
	 * Attempting to add or remove objects from the space while {@link Space#step(float)} is still executing will throw
	 * an assertion.
	 *
	 * @param constraint the {@link Constraint} to add to this space
	 * @return the added constraint
	 */
	public <T extends Constraint> T addConstraint(T constraint) {
		cpAssertHard(constraint.space == null,
					 "This shape is already added to a space and cannot be added to " + "another.");
		cpAssertHard(constraint.a != null && constraint.b != null, "Constraint is attached to a NULL body.");

		Body a = constraint.a, b = constraint.b;
		if (a == null || b == null) {
			throw new IllegalArgumentException("Constraint is attached to a null body.");
		}

		assertSpaceUnlocked();
		a.activate();
		b.activate();
		cpArrayPush(this.constraints, constraint);

		// Push onto the heads of the dynamicBodies' constraint lists
		constraint.next_a = a.constraintList;
		a.constraintList = constraint;
		constraint.next_b = b.constraintList;
		b.constraintList = constraint;
		constraint.space = this;

		return constraint;
	}

	/**
	 * Removes the given shape from this space. Cannot be called from within a callback other than a {@link
	 * PostStepFunc} callback (which is different than a {@link CollisionHandler#postSolve(Arbiter, Space)} callback!).
	 * Attempting to add or remove objects from the space while {@link Space#step(float)} is still executing will throw
	 * an assertion.
	 *
	 * @param shape the {@link Shape} to be removed to this space
	 */
	public void removeShape(Shape shape) {
		if (shape.space != this) {
			throw new IllegalArgumentException(
					"Cannot remove a shape that was not added to the space. (Removed " + "twice maybe?)");
		}
		Body body = shape.body;
		cpAssertSpaceUnlocked(this);

		boolean isStatic = cpBodyIsStatic(body);
		if (isStatic) {
			cpBodyActivateStatic(body, shape);
		} else {
			cpBodyActivate(body);
		}

		body.removeShape(shape);
		filterArbiters(body, shape);
		cpSpatialIndexRemove(isStatic ? staticShapes : dynamicShapes, shape, shape.getHashId());
		shape.space = null;
		revokeShapeId(shape);
	}

	/**
	 * Removes the given body from this space. Cannot be called from within a callback other than a {@link PostStepFunc}
	 * callback (which is different than a {@link CollisionHandler#postSolve(Arbiter, Space)} callback!). Attempting to
	 * add or remove objects from the space while {@link Space#step(float)} is still executing will throw an assertion.
	 *
	 * @param body the {@link Body} to be removed to this space
	 */
	public void removeBody(Body body) {
		cpAssertHard(containsBody(body),
					 "Cannot remove a body that was not added to the space. (Removed twice maybe?)");
		assertSpaceUnlocked();
		if (body.space != this) {
			throw new IllegalArgumentException(
					"Cannot remove a body that was not added to the space. " + "(Removed twice maybe?)");
		}

		cpBodyActivate(body);
		// filterArbiters(body, null);
		cpArrayDeleteObj(body.isDynamic() ? this.dynamicBodies : this.otherBodies, body);
		body.space = null;
	}

	/**
	 * Removes the given constraint from this space. Cannot be called from within a callback other than a {@link
	 * PostStepFunc} callback (which is different than a {@link CollisionHandler#postSolve(Arbiter, Space)} callback!).
	 * Attempting to add or remove objects from the space while {@link Space#step(float)} is still executing will throw
	 * an assertion.
	 *
	 * @param constraint the {@link Constraint} to be removed to this space
	 */
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

	/**
	 * Checks if this space contains the given shape.
	 *
	 * @param shape the {@link Shape} to check
	 * @return <code>true</code> if this space contains the shape
	 */
	public boolean containsShape(Shape shape) {
		return (shape.space == this);
	}

	/**
	 * Checks if this space contains the given body.
	 *
	 * @param body the {@link Body} to check
	 * @return <code>true</code> if this space contains the body
	 */
	public boolean containsBody(Body body) {
		return (body.space == this);
	}

	/**
	 * Checks if this space contains the given constraint.
	 *
	 * @param constraint the {@link Constraint} to check
	 * @return <code>true</code> if this space contains the constraint
	 */
	public boolean containsConstraint(Constraint constraint) {
		return (constraint.space == this);
	}

	/** Reindex all static shapes. Generally updating only the shapes that changed is faster. */
	public void reindexStatic() {
		staticShapes.each(Shape::cacheBB);
		staticShapes.reindex();
	}

	/**
	 * Update the collision detection data for a specific shape in the space.
	 *
	 * @param shape the shape to update
	 */
	public void reindexShape(Shape shape) {
		shape.cacheBB();

		// attempt to rehash the shape in both hashes
		dynamicShapes.reindexObject(shape, shape.getHashId());
		staticShapes.reindexObject(shape, shape.getHashId());
	}

	/**
	 * Reindex all the shapes for a certain body.
	 *
	 * @param body the body for which to reindex the shapes
	 */
	public void reindexShapesForBody(Body body) {
		for (Shape shape : body.shapes()) {
			reindexShape(shape);
		}
	}

	/**
	 * Call {@link SpaceBodyIteratorFunc#visit(Body)} for each body in the space. Sleeping dynamicBodies are included,
	 * but static and rogue dynamicBodies are not as they aren’alpha added to the space.
	 *
	 * @param func {@link SpaceBodyIteratorFunc} callback
	 */
	public void eachBody(final SpaceBodyIteratorFunc func) {
		cpSpaceLock(this);
		{
			for (Body body : dynamicBodies) {
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

	/**
	 * Call {@link SpaceShapeIteratorFunc#visit(Shape)} for each shape in the space. Sleeping and static shapes are
	 * included.
	 *
	 * @param func {@link SpaceShapeIteratorFunc} callback
	 */
	public void eachShape(final SpaceShapeIteratorFunc func) {
		cpSpaceLock(this);
		{
			dynamicShapes.each(func::visit);
			staticShapes.each(func::visit);
		}
		cpSpaceUnlock(this, true);
	}

	/**
	 * Call {@link SpaceConstraintIteratorFunc#visit(Constraint)} for each constraint in the space.
	 *
	 * @param func {@link SpaceConstraintIteratorFunc} callback
	 */
	public void eachConstraint(final SpaceConstraintIteratorFunc func) {
		cpSpaceLock(this);
		{
			for (Constraint constraint : constraints) {
				func.visit(constraint);
			}
		}
		cpSpaceUnlock(this, true);
	}

	private final CollisionHandlerMapKey collisionHandlerMapKey = new CollisionHandlerMapKey(null, null);

	CollisionHandler lookupHandler(CollisionType typeA, CollisionType typeB, CollisionHandler defaultHandler) {
		collisionHandlerMapKey.typeA = typeA;
		collisionHandlerMapKey.typeB = typeB;
		CollisionHandler handler = collisionHandlers.get(collisionHandlerMapKey);
		return handler != null ? handler : defaultHandler;
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
		Shape shape = context.shape;
		Body body = context.body;

		// Match on the filter shape, or if it's NULL the filter body
		if ((body == arb.body_a && (shape == arb.a || shape == null)) || (body == arb.body_b && (shape == arb.b
				|| shape == null))) {
			// Call separate when removing shapes.
			if (shape != null && arb.state != ArbiterState.CACHED) {
				arb.state = ArbiterState.INVALIDATED;
				CollisionHandler handler = arb.handler;
				handler.separateFunc.apply(arb, context.space);
			}

			arb.unthread();
			context.space.arbiters.remove(arb);
			context.space.pooledArbiters.free(arb);

			return false;
		}

		return true;
	}

	private static long cachedArbitersHashKey(Shape a, Shape b) {
		return (((long) a.getHashId()) << 32L) | ((long) b.getHashId());
	}

	void uncacheArbiter(Arbiter arb) {
		Shape a = arb.a, b = arb.b;
		cachedArbiters.remove(cachedArbitersHashKey(a, b));
		arbiters.remove(arb);
	}

	void filterArbiters(Body body, Shape filter) {
		final ArbiterFilterContext context = new ArbiterFilterContext(this, body, filter);
		cpHashSetFilter(cachedArbiters, value -> cachedArbitersFilter(value, context));
	}

	void activateBody(Body body) {
		if (!body.isDynamic()) {
			throw new IllegalArgumentException("Internal error: Attempting to activate a non-dynamic body.");
		}

		if (this.locked != 0) {
			// cpSpaceActivateBody() is called again once the space is unlocked
			if (!this.rousedBodies.contains(body)) {
				cpArrayPush(this.rousedBodies, body);
			}
		} else {
			if (body.sleeping.root != null || body.sleeping.next != null) {
				throw new IllegalStateException("Internal error: Activating body non-NULL node pointers.");
			}
			cpArrayPush(this.dynamicBodies, body);

			for (Shape shape : body.shapes()) {
				cpSpatialIndexRemove(this.staticShapes, shape, shape.getHashId());
				cpSpatialIndexInsert(this.dynamicShapes, shape, shape.getHashId());
			}

			for (Arbiter arb : body.arbiters()) {
				Body bodyA = arb.getBodyA();

				// Arbiters are shared between two bodies that are always woken up together.
				// You only want to restore the arbiter once, so bodyA is arbitrarily chosen to own the arbiter.
				// The edge case is when static bodies are involved as the static bodies never actually sleep.
				// If the static body is bodyB then all is good. If the static body is bodyA, that can easily be checked.
				if (body == bodyA || cpBodyIsStatic(bodyA)) {
					/*
					int numContacts = arb.getCount();
					List<Contact> contacts = arb.getContacts();

					// Restore contact values back to the space's contact buffer memory
					arb -> contacts = cpContactBufferGetArray(space);
					memcpy(arb -> contacts, contacts, numContacts * sizeof(struct cpContact));
					cpSpacePushContacts(space, numContacts);
					*/

					// Reinsert the arbiter into the arbiter cache
					Shape a = arb.getShapeA(), b = arb.getShapeB();
					this.cachedArbiters.put(cachedArbitersHashKey(a, b), arb);

					// Update the arbiter's state
					arb.stamp = this.stamp;
					cpArrayPush(this.arbiters, arb);

					// cpfree(contacts);
				}
			}

			for (Constraint constraint : body.constraints()) {
				Body bodyA = constraint.getBodyA();
				if (body == bodyA || cpBodyIsStatic(bodyA)) {
					cpArrayPush(this.constraints, constraint);
				}
			}
		}
	}

	void deactivateBody(Body body) {
		if (body.isRogue()) {
			throw new IllegalStateException("Internal error: Attempting to deactivate a rouge body.");
		}

		cpArrayDeleteObj(this.dynamicBodies, body);

		//CP_BODY_FOREACH_SHAPE(body, shape){
		for (Shape shape : body.shapes()) {
			cpSpatialIndexRemove(this.dynamicShapes, shape, shape.getHashId());
			cpSpatialIndexInsert(this.staticShapes, shape, shape.getHashId());
		}

		//CP_BODY_FOREACH_ARBITER(body, arb){
		for (Arbiter arb : body.arbiters()) {
			Body bodyA = arb.body_a;
			if (body == bodyA || cpBodyIsStatic(bodyA)) {
				uncacheArbiter(arb);

				// Save contact data to a new block of memory so they won'alpha time out
				/*TODO size_t bytes = arb.numContacts*sizeof(cpContact);
								cpContact *contacts = (cpContact *)cpcalloc(1, bytes);
								memcpy(contacts, arb.contacts, bytes);
								arb.contacts = contacts;*/
			}
		}

		for (Constraint constraint : body.constraints()) {
			Body bodyA = constraint.a;
			if (body == bodyA || cpBodyIsStatic(bodyA))
				cpArrayDeleteObj(this.constraints, constraint);
		}
	}

	static boolean queryRejectConstraint(Body a, Body b) {
		for (Constraint constraint : a.constraints()) {
			if (!constraint.collideBodies && ((constraint.a == a && constraint.b == b) || (constraint.a == b
					&& constraint.b == a)))
				return true;
		}

		return false;
	}

	private boolean queryReject(Shape a, Shape b) {
		return (
				// BBoxes must overlap
				!a.bb.intersects(b.bb)
						// Don't collide shapes attached to the same body.
						|| a.body == b.body
						// Don't collide shapes that are filtered.
						|| a.filter.reject(b.filter)
						// Don't collide bodies if they have a constraint with collideBodies == cpFalse.
						|| queryRejectConstraint(a.body, b.body));
	}

	// Callback from the spatial hash.
	CollisionID collideShapes(Shape a, Shape b, CollisionID id) {
		// Reject any of the simple cases
		if (queryReject(a, b)) {
			return id;
		}

		// Narrow-phase collision detection.
		CollisionInfo info = Collision.collide(a, b, id);

		if (info.isEmpty()) {
			return info.getId(); // Shapes are not colliding.
		}
		//cpSpacePushContacts(space, info.count);

		// Get an arbiter from this.arbiterSet for the two shapes.
		// This is where the persistant contact magic comes from.
		long arbHashID = cachedArbitersHashKey(info.getA(), info.getB());
		Arbiter arb = cachedArbiters.get(arbHashID);
		if (arb == null) {
			arb = pooledArbiters.alloc();
			arb.init(a, b);
			cachedArbiters.put(arbHashID, arb);
		}

		// cpArbiterUpdate(arb, & info, space);
		arb.update(info, this);

		CollisionHandler handler = arb.handler;

		// Call the begin function first if it's the first step
		if (arb.state == ArbiterState.FIRST_COLLISION && !handler.begin(arb, this)) {
			arb.ignore(); // permanently ignore the collision until separation
		}

		if (
			// Ignore the arbiter if it has been flagged
				(arb.state != ArbiterState.IGNORE) &&
						// Call preSolve
						handler.preSolve(arb, this) &&
						// Check (again) in case the pre-solve() callback called cpArbiterIgnored().
						arb.state != ArbiterState.IGNORE &&
						// Process, but don't add collisions for sensors.
						!(a.sensor || b.sensor) &&
						// Don't process collisions between two infinite mass bodies.
						!(a.body.m == Float.POSITIVE_INFINITY && b.body.m == Float.POSITIVE_INFINITY)) {
			cpArrayPush(this.arbiters, arb);
		} else {
			// cpSpacePopContacts(space, info.count);

			arb.contacts = null;

			// Normally arbiters are set as used after calling the post-solve callback.
			// However, post-solve() callbacks are not called for sensors or arbiters rejected from pre-solve.
			if (arb.state != ArbiterState.IGNORE) {
				arb.state = ArbiterState.NORMAL;
			}
		}

		// Time stamp the arbiter so we know it was used recently.
		arb.stamp = this.stamp;
		return info.getId();
	}

	// Hashset filter func to throw away old arbiters.

	boolean arbiterSetFilter(Arbiter arb) {
		int ticks = this.stamp - arb.stamp;

		Body a = arb.body_a, b = arb.body_b;

		// TODO should make an arbiter state for this so it doesn'alpha require filtering arbiters for dangling body
		// pointers on body removal.
		// Preserve arbiters on sensors and rejected arbiters for sleeping objects.
		if ((cpBodyIsStatic(a) || cpBodyIsSleeping(a)) && (cpBodyIsStatic(b) || cpBodyIsSleeping(b))) {
			return true;
		}

		// Arbiter was used last frame, but not this one
		if (ticks >= 1 && arb.state != ArbiterState.CACHED) {
			arb.state = ArbiterState.CACHED;
			CollisionHandler handler = arb.handler;
			handler.separateFunc.apply(arb, this);
		}

		if (ticks >= this.collisionPersistence) {
			arb.contacts = null;

			// cpArrayPush(pooledArbiters, arb);
			pooledArbiters.free(arb);
			return false;
		}

		return true;
	}

	static void shapeUpdateFunc(Shape shape) {
		shape.cacheBB();
	}

	/**
	 * Update the space for the given time step. Using a fixed time step is highly recommended. Doing so can greatly
	 * increase the quality of the simulation. The easiest way to do constant timesteps is to simple step forward by
	 * 1/60th of a second (or whatever your target framerate is) for each frame regardless of how long it took to
	 * render. This works fine for many games, but a better way to do it is to separate your physics timestep and
	 * rendering. This is a good article on how to do that.
	 *
	 * @param dt the time step to be used for updating the space
	 */
	public void step(float dt) {
		// don't step if the timestep is 0!
		if (dt == 0.0f) {
			return;
		}

		this.stamp++;

		float prev_dt = this.curr_dt;
		this.curr_dt = dt;

		List<Body> bodies = this.dynamicBodies;
		List<Constraint> constraints = this.constraints;
		List<Arbiter> arbiters = this.arbiters;

		// Reset and empty the arbiter lists.
		for (Arbiter arb : arbiters) {
			arb.state = ArbiterState.NORMAL;

			// If both bodies are awake, unthread the arbiter from the contact graph.
			if (!cpBodyIsSleeping(arb.body_a) && !cpBodyIsSleeping(arb.body_b)) {
				arb.unthread();
			}
		}
		arbiters.clear();

		cpSpaceLock(this);
		{
			// Integrate positions
			for (Body body : bodies) {
				body.positionFunc.apply(body, dt);
			}

			// Find colliding pairs.
			// TODO cpSpacePushFreshContactBuffer(space);
			this.dynamicShapes.each(Space::shapeUpdateFunc);
			this.dynamicShapes.reindexQuery(this::collideShapes);
		}
		cpSpaceUnlock(this, false);

		// Rebuild the contact graph (and detect sleeping components if sleeping is enabled)
		cpSpaceProcessComponents(this, dt);

		cpSpaceLock(this);
		{
			// Clear out old cached arbiters and call separate callbacks
			cpHashSetFilter(this.cachedArbiters, this::arbiterSetFilter);

			// Prestep the arbiters and constraints.
			float slop = this.collisionSlop;
			float biasCoef = 1.0f - cpfpow(this.collisionBias, dt);
			for (Arbiter arb : arbiters) {
				arb.preStep(dt, slop, biasCoef);
			}

			for (Constraint constraint : constraints) {

				ConstraintPreSolveFunc preSolve = constraint.preSolveFunc;
				if (preSolve != null) {
					preSolve.apply(constraint, this);
				}

				constraint.preStep(dt);
			}

			// Integrate velocities.
			float damping = cpfpow(this.damping, dt);
			Vector2f gravity = this.gravity;
			for (Body body : bodies) {
				body.velocityFunc.apply(body, gravity, damping, dt);
			}

			// Apply cached impulses
			float dt_coef = (prev_dt == 0.0f ? 0.0f : dt / prev_dt);
			for (Arbiter arb : arbiters) {
				arb.applyCachedImpulse(dt_coef);
			}

			for (Constraint constraint : constraints) {
				constraint.applyCachedImpulse(dt_coef);
			}

			// Run the impulse solver.
			for (int i = 0; i < this.iterations; i++) {
				for (Arbiter arb : arbiters) {
					arb.applyImpulse();
				}

				for (Constraint constraint : constraints) {
					constraint.applyImpulse(dt);
				}
			}

			// Run the constraint post-solve callbacks
			for (Constraint constraint : constraints) {
				ConstraintPostSolveFunc postSolve = constraint.postSolveFunc;
				if (postSolve != null) {
					postSolve.apply(constraint, this);
				}
			}

			// run the post-solve callbacks
			for (Arbiter arb : arbiters) {
				CollisionHandler handler = arb.handler;
				handler.postSolveFunc.apply(arb, this);
			}
		}
		cpSpaceUnlock(this, true);
	}

	static void cpSpaceLock(Space space) {
		space.locked++;
	}

	static void cpSpaceUnlock(Space space, boolean runPostStep) {
		space.locked--;
		assert space.locked >= 0 : "Internal Error: Space lock underflow.";

		if (space.locked == 0) {
			List<Body> waking = space.rousedBodies;
			for (Body aWaking : waking) {
				space.activateBody(aWaking);
			}
			waking.clear();

			if (space.locked == 0 && runPostStep && !space.skipPostStep) {
				space.skipPostStep = true;

				cpSpaceRunPostStepCallbacks(space);

				space.skipPostStep = false;
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
	 * @param point       the query point
	 * @param maxDistance the max. distance to query
	 * @param filter      the filter to use
	 * @param func        the callback function
	 */
	public void pointQuery(Vector2f point, float maxDistance, ShapeFilter filter, SpacePointQueryFunc func) {
		cpSpacePointQuery(this, point, maxDistance, filter, func);
	}

	/**
	 * Add <code>func</code> to be called before {@link Space#step(float)} returns. You can add post step callbacks from
	 * outside of other callback functions, but there isn?alpha a good reason to and they won?alpha be called until the
	 * next time {@link Space#step(float)} is finishing.
	 *
	 * @param func the post step callback to add
	 */
	public void addPostStepCallback(PostStepFunc func) {
		cpAssertWarn(locked != 0, "Adding a post-step callback when the space is not locked is unnecessary. "
				+ "Post-step callbacks will not called until the end of the next call to cpSpaceStep() or the "
				+ "next query.");

		if (postStepCallbacks == null) {
			postStepCallbacks = new ArrayList<>(1);
		}
		postStepCallbacks.add(func);
	}

	/**
	 * Perform a directed line segment query (like a raycast) against the space and return the first shape hit. Returns
	 * <code>null</code> if no shapes were hit.
	 *
	 * @param start  the start point of the segment.
	 * @param end    the end point of the segment.
	 * @param filter the filter to use for the query.
	 * @param out    a {@link SegmentQueryInfo} taking the result.
	 * @return the first {@link Shape} hit or <code>null</code> if no shape was hit.
	 */
	public SegmentQueryInfo segmentQueryFirst(Vector2f start, Vector2f end, float radius, ShapeFilter filter,
			SegmentQueryInfo out) {
		if (out == null) {
			out = new SegmentQueryInfo();
		}
		return cpSpaceSegmentQueryFirst(this, start, end, radius, filter, out);
	}

	/**
	 * Query this space along the line segment from <code>start</code> to <code>end</code> filtering out matches with
	 * the given <code>layers</code> and <code>group</code>. <code>func</code> is called with the normalized distance
	 * along the line and surface normal for each shape found along. Sensor shapes are included.
	 *
	 * @param start  the start point of the segment.
	 * @param end    the end point of the segment.
	 * @param filter the filter to use for the query.
	 * @param func   a {@link SpaceSegmentQueryFunc} callback for each found shape.
	 */
	public void segmentQuery(Vector2f start, Vector2f end, float radius, ShapeFilter filter,
			SpaceSegmentQueryFunc func) {
		cpSpaceSegmentQuery(this, start, end, radius, filter, func);
	}

	/**
	 * Query this space at <code>point</code> and return the closest shape within maxDistance units of distance. out is
	 * an optional pointer to a cpNearestPointQueryInfo if you want additional information about the match.
	 *
	 * @param point       the query point.
	 * @param maxDistance the max. distance to query.
	 * @param filter      the filter to use for the query.
	 * @param out         if not <code>null</code> use this object as return value, else a new instance will be
	 *                    created.
	 * @return a {@link PointQueryInfo} object with information about the closest shape.
	 */
	public PointQueryInfo pointQueryNearest(Vector2f point, float maxDistance, ShapeFilter filter, PointQueryInfo out) {
		return cpSpacePointQueryNearest(this, point, maxDistance, filter, out);
	}

	public void bbQuery(BB bb, ShapeFilter filter, SpaceBBQueryFunc func) {
		cpSpaceBBQuery(this, bb, filter, func);
	}

	public boolean shapeQuery(Shape shape, SpaceShapeQueryFunc func) {
		return cpSpaceShapeQuery(this, shape, func);
	}

	public List<Shape> getShapes() {
		final List<Shape> shapes = new ArrayList<>();
		dynamicShapes.each(shapes::add);
		return shapes;
	}

	public List<Shape> getStaticShapes() {
		final List<Shape> shapes = new ArrayList<>();
		staticShapes.each(shapes::add);
		return shapes;
	}

	public List<Constraint> getConstraints() {
		return constraints;
	}

	public List<Body> getDynamicBodies() {
		return dynamicBodies;
	}

	public List<Arbiter> getArbiters() {
		return arbiters;
	}

	public void useSpatialHash(float dim, int count) {
		final SpatialIndex<Shape> staticShapes = new SpaceHash<>(dim, count, Shape::getBB, null);
		final SpatialIndex<Shape> activeShapes = new SpaceHash<>(dim, count, Shape::getBB, staticShapes);

		cpSpatialIndexEach(this.staticShapes, obj -> staticShapes.insert(obj, obj.getHashId()));
		cpSpatialIndexEach(this.dynamicShapes, obj -> activeShapes.insert(obj, obj.getHashId()));

		this.staticShapes = staticShapes;
		this.dynamicShapes = activeShapes;
	}

	/**
	 * Convert a dynamic rogue body to a static one. This will convert any shapes attached to the body into static
	 * shapes, but does not handle constraints. If the body is active, you must remove it from the space first.
	 *
	 * @param body the {@link Body} to convert
	 */
	public void convertBodyToStatic(Body body) {
		cpAssertHard(!cpBodyIsStatic(body), "Body is already static.");
		cpAssertHard(cpBodyIsRogue(body), "Remove the body from the space before calling this function.");
		Assert.cpAssertSpaceUnlocked(this);

		body.setMass(Float.POSITIVE_INFINITY);
		body.setMoment(Float.POSITIVE_INFINITY);

		body.setVelocity(cpvzero());
		body.setAngularVelocity(0.0f);

		body.sleeping.idleTime = Float.POSITIVE_INFINITY;
		for (Shape shape : body.shapes()) {
			cpSpatialIndexRemove(this.dynamicShapes, shape, shape.getHashId());
			cpSpatialIndexInsert(this.staticShapes, shape, shape.getHashId());
		}

	}

	/**
	 * Convert a body to a dynamic rogue body. This will convert any static shapes attached to the body into regular
	 * ones. If you want the body to be active after the transition, you must add it to the space also.
	 *
	 * @param body   the {@link Body} to convert
	 * @param mass   the mass to use for the body
	 * @param moment the moment to use for the body
	 */
	public void convertBodyToDynamic(Body body, float mass, float moment) {
		cpAssertHard(cpBodyIsStatic(body), "Body is already dynamic.");
		Assert.cpAssertSpaceUnlocked(this);

		cpBodyActivateStatic(body, null);

		body.setMass(mass);
		body.setMoment(moment);

		body.sleeping.idleTime = 0.0f;
		for (Shape shape : body.shapes()) {
			cpSpatialIndexRemove(this.staticShapes, shape, shape.getHashId());
			cpSpatialIndexInsert(this.dynamicShapes, shape, shape.getHashId());
		}
	}

	public CollisionHandler getDefaultHandler() {
		return defaultHandler;
	}

	public void setDefaultHandler(CollisionHandler defaultHandler) {
		this.defaultHandler = defaultHandler;
	}

	public boolean isUseWildcards() {
		return useWildcards;
	}

	public void setUseWildcards(boolean useWildcards) {
		this.useWildcards = useWildcards;
	}
}
