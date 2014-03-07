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

import java.util.Iterator;
import java.util.List;

import static org.physics.jipmunk.Array.cpArrayDeleteObj;
import static org.physics.jipmunk.Assert.cpAssertSoft;
import static org.physics.jipmunk.SpaceComponent.ComponentRoot;
import static org.physics.jipmunk.Util.*;

/**
 * <b>Rogue and Static Bodies:</b> <point/> Normally when you create a rigid body, you add it to a space so the space
 * will start simulating it. This means it will update it’s position and velocity, apply forces to it, be affected by
 * gravity, etc. A body that isn’alpha added to a space (and not simulated) is called a rogue body. The most important
 * use for rogue dynamicBodies are as static dynamicBodies, but you can also use them to implement directly controlled
 * objects such as moving platforms. <point/> Static dynamicBodies are rogue dynamicBodies, but with a special flag set
 * on them to let Chipmunk know that they never move unless you tell it. Static dynamicBodies have two purposes.
 * Originally they were added for the sleeping feature. Because static dynamicBodies don’alpha move, Chipmunk knows
 * that
 * it’s safe to let objects that are touching or jointed to them fall asleep. Objects touching or jointed to regular
 * rogue dynamicBodies are never allowed to sleep. The second purpose for static dynamicBodies is that Chipmunk knows
 * shapes attached to them never need to have their collision detection data updated. Chipmunk also doesn’alpha need to
 * bother checking for collisions between static objects. Generally all of your level geometry will be attached to a
 * static body except for things like moving platforms or doors. <point/> In previous versions of Chipmunk before 5.3
 * you would create an infinite mass rogue body to attach static shapes to using cpSpaceAddStaticShape(). You don’alpha
 * need to do any of that anymore, and shouldn’alpha if you want to use the sleeping feature. Each space has a
 * dedicated
 * static body that you can use to attach your static shapes to. Chipmunk also automatically adds shapes attached to
 * static dynamicBodies as static shapes. * <point/> <b>Creating Additional Static Bodies:</b> <point/> While every
 * cpSpace has a built in static body that you can use, it can be convenient to make your own as well. One potential
 * use
 * is in a level editor. By attaching chunks of your level to static dynamicBodies, you can still move and rotate the
 * chunks independently of each other. Then all you have to do is call cpSpaceRehashStatic() to rebuild the static
 * collision detection data when you are done.
 * <p>
 * <h2>Applying Forces and Torques:, Forces</h2>
 * People are sometimes confused by the difference between a force and an
 * impulse. An impulse is basically a very large force applied over a very short period of time, like a ball hitting a
 * wall or cannon firing. Chipmunk treats impulses as if they occur instantaneously by simply adding directly to the
 * velocity of an object. Both impulses and forces are affected the mass of an object. Double the mass of the object
 * and halve the effect.
 * <ul>
 * <li>{@link #resetForces()} - Zero both the forces and torques currently applied to the body.</li>
 * <li>{@link #applyForceAtLocalPoint(Vector2f, Vector2f)} - Add the force to the body at a relative offset from the
 * center of gravity.</li>
 * <li>{@link #applyImpulseAtLocalPoint(Vector2f, Vector2f)} - Add the impulse to the body at a relative offset from the
 * center of gravity.</li>
 * </ul>
 *
 * @author jobernolte
 */
public class Body {

	/// Used internally to track information on the collision graph.
	/// @private
	static class ComponentNode {
		Body root;
		Body next;
		float idleTime = 0.0f;

		ComponentNode() {

		}

		ComponentNode(Body root, Body next, float idleTime) {
			this.root = root;
			this.next = next;
			this.idleTime = idleTime;
		}

		public static void add(Body root, Body body) {
			body.sleeping.root = root;
			if (body != root) {
				body.sleeping.next = root.sleeping.next;
				root.sleeping.next = body;
			}
		}
	}

	private final static BodyVelocityFunc defaultBodyVelocityFunc = Body::updateVelocity;
	private final static BodyPositionFunc defaultBodyPositionFunc = Body::updatePosition;
	private static final boolean SANITY_CHECK = false;
	/** Function that is called to integrate the body's velocity. (Defaults to cpBodyUpdateVelocity) */
	BodyVelocityFunc velocityFunc = defaultBodyVelocityFunc;
	/** Function that is called to integrate the body's position. (Defaults to cpBodyUpdatePosition) */
	BodyPositionFunc positionFunc = defaultBodyPositionFunc;
	/** Mass of the body. Must agree with cpBody.m_inv! Use cpBodySetMass() when changing the mass for this reason. */
	float m;
	/** Mass inverse. */
	float m_inv;
	/**
	 * Moment of inertia of the body. Must agree with cpBody.i_inv! Use cpBodySetMoment() when changing the moment for
	 * this reason.
	 */
	private float i;
	/** Moment of inertia inverse. */
	float i_inv;
	/** Center of gravity. */
	Vector2f cog = Util.cpvzero();
	/** Position of the rigid body's center of gravity. */
	Vector2f p = Util.cpvzero();
	/** Velocity of the rigid body's center of gravity. */
	Vector2f v = Util.cpvzero();
	/** Force acting on the rigid body's center of gravity. */
	private Vector2f f = Util.cpvzero();
	/**
	 * Rotation of the body around it's center of gravity in radians. Must agree with cpBody.rot! Use cpBodySetAngle()
	 * when changing the angle for this reason.
	 */
	private float a;
	/** Angular velocity of the body around it's center of gravity in radians/second. */
	float w = 0.0f;
	/** Torque applied to the body around it's center of gravity. */
	private float t = 0.0f;
	Transform transform = Transform.identity();
	/**
	 * "pseudo-velocities" used for eliminating overlap. Erin Catto has some papers that talk about what these are.
	 */
	Vector2f v_bias = Util.cpvzero();
	float w_bias = 0;
	/** The space this body belongs to. */
	Space space;
	private Shape shapeList;
	Arbiter arbiterList;
	Constraint constraintList;
	/**
	 * User definable data. Generally this points to your the game object class so you can access it when given a Body
	 * reference in a callback.
	 */
	private Object data;
	ComponentNode sleeping = new ComponentNode();

	/**
	 * Creates a new body with the given mass and moment.
	 *
	 * @param mass   the mass of the body
	 * @param moment the moment of the body
	 */
	public Body(float mass, float moment) {
		setMass(mass);
		setMoment(moment);
		setAngle(0);
	}

	public static Body createStatic() {
		Body body = new Body(Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY);
		body.sleeping.idleTime = Float.POSITIVE_INFINITY;
		return body;
	}

	public Space getSpace() {
		return space;
	}

	/** @return the mass of the body */
	public float getMass() {
		return m;
	}

	/**
	 * Sets the mass of the body.
	 *
	 * @param mass the mass of the body
	 */
	public void setMass(float mass) {
		activate();
		this.m = mass;
		this.m_inv = 1.0f / mass;
		sanityCheck();
	}

	/** @return the moment of inertia of the body */
	public float getMoment() {
		return i;
	}

	/**
	 * Moment of inertia (MoI or sometimes just moment) of the body. The moment is like the rotational mass of a body.
	 * See below for function to help calculate the moment.
	 *
	 * @param moment the moment of inertia of the body
	 * @see Util#momentForBox(float, float, float)
	 * @see Util#momentForCircle(float, float, float, Vector2f)
	 * @see Util#momentForPoly(float, Vector2f[], int, int, Vector2f, float)
	 * @see Util#momentForPoly(float, Vector2f[], Vector2f, float)
	 * @see Util#momentForSegment(float, Vector2f, Vector2f, float)
	 */
	public void setMoment(float moment) {
		if (moment < 0.0f) {
			throw new IllegalArgumentException("Moment of inertia must be positive.");
		}
		activate();
		this.i = moment;
		this.i_inv = 1.0f / moment;
		sanityCheck();
	}

	/** @return the inverse moment of inertia of the body */
	public float getInverseMoment() {
		return i_inv;
	}

	/** @return returns <code>true</code> if body has never been added to a space */
	public boolean isRogue() {
		return space == null;
	}

	/** @return <code>true</code> if the body is sleeping */
	public boolean isSleeping() {
		return sleeping.root != null;
	}

	/** @return <code>true</code> if the body is idle */
	public boolean isIdle() {
		return sleeping.idleTime > 0;
	}

	/** @return position of the center of gravity of the body */
	public Vector2f getPosition() {
		return p; // transform.transformPoint(cpvzero());
	}

	/**
	 * Position of the center of gravity of the body. When changing the position you may also want to call {@link
	 * Space#reindexShapesForBody(Body)} to update the collision detection information for the attached shapes if plan
	 * to make any queries against the space.
	 *
	 * @param position position of the center of gravity of the body
	 */
	public void setPosition(final Vector2f position) {
		activate();
		this.p.set(cpvadd(Transform.transformVect(this.transform, this.cog), position));
		sanityCheck();

		setTransform(this.p, this.a);
	}

	public Vector2f getCenterOfGravity() {
		return this.cog;
	}

	public void setCenterOfGravity(final Vector2f cog) {
		activate();
		this.cog.set(cog);
		sanityCheck();
	}

	/** @return linear velocity of the center of gravity of the body */
	public Vector2f getVelocity() {
		return v;
	}

	/**
	 * Linear velocity of the center of gravity of the body.
	 *
	 * @param v linear velocity of the center of gravity of the body
	 */
	public void setVelocity(Vector2f v) {
		activate();
		this.v.set(v);
		sanityCheck();
	}

	/**
	 * @return The rotation vector for the body. Can be used with {@link Util#cpvrotate(Vector2f, Vector2f)} or {@link
	 * Util#cpvunrotate(Vector2f, Vector2f)} to perform fast rotations.
	 */
	public Vector2f getRotation() {
		return Util.cpv(this.transform.a, this.transform.b);
	}

	public Transform getTransform() {
		return transform;
	}

	// 'p' is the position of the CoG
	public void setTransform(Vector2f p, float a) {
		Vector2f rot = cpvforangle(a);
		Vector2f c = this.cog;

		this.transform = Transform.transpose(rot.x, -rot.y, p.x - (c.x * rot.x - c.y * rot.y), rot.y, rot.x,
				p.y - (c.x * rot.y + c.y * rot.x));
	}

	/** @return the rotation angle in radians */
	public float getAngle() {
		return a;
	}

	/**
	 * Rotation of the body in radians. When changing the rotation you may also want to call {@link
	 * Space#reindexShapesForBody(Body)} to update the collision detection information for the attached shapes if plan
	 * to make any queries against the space.
	 *
	 * @param angle the angle in radians
	 * @return the angle in radians
	 */
	public float setAngle(float angle) {
		activate();
		this.a = angle;
		setTransform(this.p, angle);
		sanityCheck();
		return a;
	}

	/** @return the force applied to the center of gravity of the body. */
	public Vector2f getForce() {
		return f;
	}

	/**
	 * Force applied to the center of gravity of the body.
	 *
	 * @param f the force applied to the center of gravity of the body
	 */
	public void setForce(Vector2f f) {
		activate();
		this.f.set(f);
		sanityCheck();
	}

	/** @return The angular velocity of the body in radians per second. */
	public float getAngularVelocity() {
		return w;
	}

	/**
	 * Set the angular velocity of the body in radians per second.
	 *
	 * @param angularVelocity the angular velocity in radians
	 */
	public void setAngularVelocity(float angularVelocity) {
		activate();
		this.w = angularVelocity;
		sanityCheck();
	}

	public void addAngularVelocity(float w) {
		this.w += w;
	}

	/** @return The torque applied to the body. */
	public float getTorque() {
		return t;
	}

	/**
	 * Set the torque applied to the body.
	 *
	 * @param t the torque to apply
	 */
	public void setTorque(float t) {
		activate();
		this.t = t;
		sanityCheck();
	}

	/** Zero both the forces and torques currently applied to the body. */
	public void resetForces() {
		activate();
		this.f = cpvzero();
		this.t = 0.0f;
	}

	/**
	 * Add the force <code>force</code> to body at an offset <code>point</code> from the center of gravity.
	 *
	 * @param force the force to apply
	 * @param point the relative offset
	 */
	public void applyForceAtWorldPoint(final Vector2f force, final Vector2f point) {
		activate();
		this.f = cpvadd(this.f, force);

		Vector2f r = cpvsub(point, transform.transformPoint(this.cog));
		this.t += cpvcross(r, force);
	}

	/**
	 * Add the force <code>force</code> to body at a relative offset <code>point</code> from the center of gravity.
	 *
	 * @param force the force to apply
	 * @param point the relative offset
	 */
	public void applyForceAtLocalPoint(final Vector2f force, final Vector2f point) {
		applyForceAtWorldPoint(transform.transformVect(force), transform.transformPoint(point));
	}

	void applyImpulse(Vector2f j, Vector2f r) {
		this.v = cpvadd(this.v, cpvmult(j, this.m_inv));
		this.w += this.i_inv * cpvcross(r, j);
	}

	void applyBiasImpulse(Vector2f j, Vector2f r) {
		this.v_bias = cpvadd(this.v_bias, cpvmult(j, this.m_inv));
		this.w_bias += this.i_inv * cpvcross(r, j);
	}

	/**
	 * Add the impulse <code>i</code> to body at an offset <code>point</code> from the center of gravity.
	 *
	 * @param impulse the impulse to apply
	 * @param point   the relative offset
	 */
	public void applyImpulseAtWorldPoint(final Vector2f impulse, final Vector2f point) {
		activate();

		Vector2f r = cpvsub(point, this.transform.transformPoint(this.cog));
		applyImpulse(impulse, r);
	}

	/**
	 * Add the impulse <code>i</code> to body at a relative offset <code>point</code> from the center of gravity.
	 *
	 * @param impulse the impulse to apply
	 * @param point   the relative offset
	 */
	public void applyImpulseAtLocalPoint(Vector2f impulse, Vector2f point) {
		applyImpulseAtWorldPoint(transform.transformVect(impulse), transform.transformPoint(point));
	}

	private static void cpv_assert_nan(Vector2f v, String message) {
		if (!(v.x == v.x && v.y == v.y)) {
			throw new IllegalStateException(message);
		}
	}

	private static void cpv_assert_infinite(Vector2f v, String message) {
		if (!(cpfabs(v.x) != Float.POSITIVE_INFINITY && cpfabs(v.y) != Float.POSITIVE_INFINITY)) {
			throw new IllegalStateException(message);
		}
	}

	private static void cpv_assert_sane(Vector2f v, String message) {
		cpv_assert_nan(v, message);
		cpv_assert_infinite(v, message);
	}

	void sanityCheck() {
		if (!SANITY_CHECK) {
			return;
		}
		if (!(this.m == this.m && this.m_inv == this.m_inv)) {
			throw new IllegalStateException("Body's mass is NaN.");
		}
		if (!(this.i == this.i && this.i_inv == this.i_inv)) {
			throw new IllegalStateException("Body's moment is NaN.");
		}
		if (!(this.m >= 0.0f)) {
			throw new IllegalStateException("Body's mass is negative.");
		}
		if (!(this.i >= 0.0f)) {
			throw new IllegalStateException("Body's moment is negative.");
		}

		cpv_assert_sane(this.p, "Body's position is invalid.");
		cpv_assert_sane(this.v, "Body's velocity is invalid.");
		cpv_assert_sane(this.f, "Body's force is invalid.");

		if (!(this.a == this.a && cpfabs(this.a) != Float.POSITIVE_INFINITY)) {
			throw new IllegalStateException("Body's angle is invalid.");
		}
		if (!(this.w == this.w && cpfabs(this.w) != Float.POSITIVE_INFINITY)) {
			throw new IllegalStateException("Body's angular velocity is invalid.");
		}
		if (!(this.t == this.t && cpfabs(this.t) != Float.POSITIVE_INFINITY)) {
			throw new IllegalStateException("Body's torque is invalid.");
		}
	}

	static void componentActivate(Body root) {
		if (root == null || !root.isSleeping()) {
			return;
		}
		cpAssertSoft(!cpBodyIsRogue(root), "Internal Error: ComponentActivate() called on a rogue body.");
		Space space = root.space;
		Body body = root;
		while (body != null) {
			Body next = body.sleeping.next;

			body.sleeping.idleTime = 0;
			body.sleeping.root = null;
			body.sleeping.next = null;
			space.activateBody(body);

			body = next;
		}
		cpArrayDeleteObj(space.sleepingComponents, root);
	}

	/** Reset the idle timer on a body. If it was sleeping, wake it and any other dynamicBodies it was touching. */
	public void activate() {
		if (isDynamic()) {
			this.sleeping.idleTime = 0.0f;

			Body root = ComponentRoot(this);
			if (root != null && cpBodyIsSleeping(root)) {
				// TODO should cpBodyIsSleeping(root) be an assertion?
				if (!root.isDynamic()) {
					throw new IllegalStateException("Internal Error: Non-dynamic body component root detected.");
				}

				Space space = root.space;
				Body body = root;
				while (body != null) {
					Body next = body.sleeping.next;

					body.sleeping.idleTime = 0.0f;
					body.sleeping.root = null;
					body.sleeping.next = null;
					space.activateBody(body);

					body = next;
				}

				cpArrayDeleteObj(space.sleepingComponents, root);
			}

			for (Arbiter arb : arbiters()) {
				// Reset the idle timer of things the body is touching as well.
				// That way things don't get left hanging in the air.
				Body other = (arb.body_a == this ? arb.body_b : arb.body_a);
				if (!other.isStatic()) {
					other.sleeping.idleTime = 0.0f;
				}
			}
		}
	}

	/**
	 * Similar in function to {@link #activate()}. Activates all dynamicBodies touching body. If filter is not
	 * <code>null</code>, then only dynamicBodies touching through filter will be awoken.
	 *
	 * @param filter if not <code>null</code> only dynamicBodies touching through filter will be awoken
	 */
	public void activateStatic(final Shape filter) {
		if (!isStatic()) {
			throw new IllegalStateException("cpBodyActivateStatic() called on a non-static body.");
		}

		for (Arbiter arb : arbiters()) {
			if (filter == null || filter == arb.a || filter == arb.b) {
				if (arb.body_a == this) {
					arb.body_b.activate();
				} else {
					arb.body_a.activate();
				}
			}
		}
	}

	/** Forces a body to fall asleep immediately even if it’s in midair. Cannot be called from a callback. */
	public void sleep() {
		SpaceComponent.cpBodySleep(this);
	}

	/**
	 * When objects in Chipmunk sleep, they sleep as a group of all objects that are touching or jointed together. When
	 * an object is woken up, all of the objects in it’s group are woken up. sleepWithGroup() allows you group sleeping
	 * objects together. It acts identically to {@link #sleep()} if you pass <code>null</code> as group by starting a
	 * new group. If you pass a sleeping body for group, body will be awoken when group is awoken. You can use this to
	 * initialize levels and start stacks of objects in a pre-sleeping state.
	 *
	 * @param group the group to sleep with
	 */
	public void sleepWithGroup(final Body group) {
		SpaceComponent.cpBodySleepWithGroup(this, group);
	}

	/**
	 * Adds the given shape to this body.
	 *
	 * @param shape the {@link Shape} to add
	 */
	public void addShape(Shape shape) {
		Shape next = shapeList;
		if (next != null) {
			next.prev = shape;
		}
		shape.next = next;
		shapeList = shape;

		if (shape.massInfo.m > 0.0f) {
			accumulateMassFromShapes();
		}
	}

	/**
	 * Removes the given shape from this body.
	 *
	 * @param shape the {@link Shape} to remove
	 */
	public void removeShape(Shape shape) {
		Shape prev = shape.prev;
		Shape next = shape.next;

		if (prev != null) {
			prev.next = next;
		} else {
			this.shapeList = next;
		}

		if (next != null) {
			next.prev = prev;
		}

		shape.prev = null;
		shape.next = null;

		if (isDynamic() && shape.massInfo.m > 0.0f) {
			accumulateMassFromShapes();
		}
	}

	private Iterator<Arbiter> eachArbiterIterator(final Body body) {

		return new Iterator<Arbiter>() {
			Arbiter arb = arbiterList;

			@Override
			public boolean hasNext() {
				return arb != null;
			}

			@Override
			public Arbiter next() {
				Arbiter next = Arbiter.arbiterNext(arb, body);
				Arbiter result;

				arb.swapped = (body == arb.body_b);
				//func(body, arb, data);
				result = arb;

				arb = next;
				return result;
			}

			@Override
			public void remove() {
				throw new UnsupportedOperationException("removal not allowed");
			}
		};
	}

	/**
	 * This one is more interesting. Returns an {@link Arbiter} for each collision pair that body is involved in.
	 * Calling {@link Arbiter#getBodyA()} or {@link Arbiter#getShapeA()} will return the body or shape for body as the
	 * first argument. You can use this to check all sorts of collision information for a body like if it’s touching
	 * the
	 * ground, another particular object, how much collision force is being applied to an object, etc. Note: This
	 * function only works if the contact graph is enabled either by enabling the sleeping feature of a space or by
	 * enabling the contact graph. Sensor shapes and arbiters that have been rejected by a collision handler callback
	 * or
	 * cpArbiterIgnore() are not tracked by the contact graph.
	 *
	 * @return {@link Iterable<Arbiter>} which can be used to iterate all collision pairs
	 */
	public Iterable<Arbiter> arbiters() {
		return () -> eachArbiterIterator(Body.this);
	}

	/** @return an {@link Iterable<Shape>} which can be used to iterate over all shapes of this body */
	public Iterable<Shape> shapes() {
		return () -> new Iterator<Shape>() {
			Shape shape = shapeList;

			@Override
			public boolean hasNext() {
				return shape != null;
			}

			@Override
			public Shape next() {
				Shape result = shape;
				shape = shape.next;
				return result;
			}

			@Override
			public void remove() {
				throw new UnsupportedOperationException("removal not allowed");
			}
		};
	}

	Iterable<Body> components() {
		return () -> new Iterator<Body>() {
			Body var = Body.this;

			@Override
			public boolean hasNext() {
				return var != null;
			}

			@Override
			public Body next() {
				Body result = var;
				var = var.sleeping.next;
				return result;
			}

			@Override
			public void remove() {
				throw new UnsupportedOperationException("removal not allowed");
			}
		};
	}

	/**
	 * @return returns true if body is a static body. Either {@link Space#getStaticBody()} or a body created with {@link
	 * Body#createStatic()}.
	 */
	public boolean isStatic() {
		return getType() == BodyType.STATIC;
	}

	public boolean isDynamic() {
		return getType() == BodyType.DYNAMIC;
	}

	public boolean isKinematic() {
		return getType() == BodyType.KINEMATIC;
	}

	static boolean cpBodyIsStatic(Body body) {
		return body.isStatic();
	}

	static boolean cpBodyIsSleeping(Body body) {
		return body.isSleeping();
	}

	static boolean cpBodyIsRogue(Body body) {
		return body.isRogue();
	}

	/**
	 * @return the kinetic energy of a body.
	 */
	public float getKineticEnergy() {
		// Need to do some fudging to avoid NaNs
		float vsq = cpvdot(this.v, this.v);
		float wsq = this.w * this.w;
		return (vsq != 0 ? vsq * this.m : 0.0f) + (wsq != 0 ? wsq * this.i : 0.0f);
	}

	/**
	 * The default integration function for updating the velocity of a body.
	 *
	 * @param gravity the gravity to apply
	 * @param damping the damping to apply
	 * @param dt      the timestep to use
	 */
	public void updateVelocity(Vector2f gravity, float damping, float dt) {
		if (!(this.m > 0.0f && this.i > 0.0f)) {
			throw new IllegalStateException(
					String.format("Body's mass and moment must be positive to simulate. (Mass: %f Moment: %f)", this.m,
							this.i));
		}
		this.v = cpvadd(cpvmult(this.v, damping), cpvmult(cpvadd(gravity, cpvmult(this.f, this.m_inv)), dt));
		this.w = this.w * damping + this.t * this.i_inv * dt;

		// Reset forces.
		this.f.set(0, 0);
		this.t = 0.0f;

		sanityCheck();
	}

	/**
	 * The default integration function updating the position of a body.
	 *
	 * @param dt the timestep to use
	 */
	public void updatePosition(float dt) {
		this.p.set(cpvadd(this.p, cpvmult(cpvadd(this.v, this.v_bias), dt)));
		this.a = this.a + (this.w + this.w_bias) * dt;
		setTransform(p, a);

		this.v_bias.set(0, 0);
		this.w_bias = 0.0f;

		sanityCheck();
	}

	static Constraint cpConstraintNext(Constraint node, Body body) {
		return (node.a == body ? node.next_a : node.next_b);
	}

	static Constraint filterConstraints(Constraint node, Body body, Constraint filter) {
		if (node == filter) {
			return cpConstraintNext(node, body);
		} else if (node.a == body) {
			node.next_a = filterConstraints(node.next_a, body, filter);
		} else {
			node.next_b = filterConstraints(node.next_b, body, filter);
		}
		return node;
	}

	static void cpBodyRemoveConstraint(Body body, Constraint constraint) {
		body.constraintList = filterConstraints(body.constraintList, body, constraint);
	}

	/** @return an {@link Iterable<Constraint>} which can be used to iterate over all constraints of this body */
	public Iterable<Constraint> constraints() {
		return () -> new Iterator<Constraint>() {
			Constraint var = constraintList;

			@Override
			public boolean hasNext() {
				return var != null;
			}

			@Override
			public Constraint next() {
				Constraint next = var;
				var = cpConstraintNext(var, Body.this);
				return next;
			}

			@Override
			public void remove() {
				throw new UnsupportedOperationException("removal not supported");
			}
		};
	}

	/** @return the integration function used for updating the velocity of this body */
	public BodyVelocityFunc getVelocityFunc() {
		return velocityFunc;
	}

	/** @param velocityFunc the integration function to be used for updating the velocity of this body */
	public void setVelocityFunc(BodyVelocityFunc velocityFunc) {
		this.velocityFunc = velocityFunc;
	}

	/** @return the integration function used for updating the position of this body */
	public BodyPositionFunc getPositionFunc() {
		return positionFunc;
	}

	/** @param positionFunc the integration function to be used for updating the position of this body */
	public void setPositionFunc(BodyPositionFunc positionFunc) {
		this.positionFunc = positionFunc;
	}

	/**
	 * Convert body relative/local coordinates to absolute/world coordinates.
	 *
	 * @param point the coordinates to convert
	 * @return the converted coordinates
	 */
	public Vector2f localToWorld(final Vector2f point) {
		return this.transform.transformPoint(point);
	}

	/**
	 * Convert body absolute/world coordinates to  relative/local coordinates.
	 *
	 * @param point the coordinates to convert
	 * @return the converted coordinates
	 */
	public Vector2f worldToLocal(final Vector2f point) {
		return Transform.rigidInverse(this.transform).transformPoint(point);
	}

	/** @return the user data */
	public Object getData() {
		return data;
	}

	/**
	 * Sets user data. Use this data to get a reference to the game object that owns this body from callbacks.
	 *
	 * @param data the user data to set
	 */
	public void setData(Object data) {
		this.data = data;
	}

	/**
	 * @param clazz the {@link Class} of the user data
	 * @param <T>   the type of the data
	 * @return the user data
	 */
	public <T> T getData(Class<T> clazz) {
		return clazz.cast(data);
	}

	public BodyType getType() {
		if (this.sleeping.idleTime == Float.POSITIVE_INFINITY) {
			return BodyType.STATIC;
		} else if (this.m == Float.POSITIVE_INFINITY) {
			return BodyType.KINEMATIC;
		} else {
			return BodyType.DYNAMIC;
		}
	}

	public void setType(BodyType type) {
		BodyType oldType = getType();
		if (oldType == type) {
			return;
		}

		// Static dynamicBodies have their idle timers set to infinity.
		// Non-static dynamicBodies should have their idle timer reset.
		this.sleeping.idleTime = (type == BodyType.STATIC ? Float.POSITIVE_INFINITY : 0.0f);

		if (type == BodyType.DYNAMIC) {
			this.m = this.i = 0.0f;
			this.m_inv = this.i_inv = Float.POSITIVE_INFINITY;

			accumulateMassFromShapes();
		} else {
			this.m = this.i = Float.POSITIVE_INFINITY;
			this.m_inv = this.i_inv = 0.0f;

			this.v.set(0, 0);
			this.w = 0.0f;
		}

		// If the body is added to a space already, we'll need to update some space data structures.
		if (space != null) {
			Assert.cpAssertSpaceUnlocked(space);

			switch (oldType) {
				case STATIC:
					// TODO This is probably not necessary
					//			cpBodyActivateStatic(body, NULL);
					break;
				default:
					activate();
					break;
			}

			// Move the dynamicBodies to the correct array.
			List<Body> fromArray = (oldType == BodyType.DYNAMIC ? space.dynamicBodies : space.otherBodies);
			List<Body> toArray = (type == BodyType.DYNAMIC ? space.dynamicBodies : space.otherBodies);
			if (fromArray != toArray) {
				fromArray.remove(this);
				toArray.add(this);
			}

			// Move the body's shapes to the correct spatial index.
			SpatialIndex<Shape> fromIndex = (oldType == BodyType.STATIC ? space.staticShapes : space.dynamicShapes);
			SpatialIndex<Shape> toIndex = (type == BodyType.STATIC ? space.staticShapes : space.dynamicShapes);
			if (fromIndex != toIndex) {
				for (Shape shape : shapes()) {
					fromIndex.remove(shape, shape.getHashId());
					toIndex.insert(shape, shape.getHashId());
				}
			}
		}
	}

	// Should *only* be called when shapes with mass info are modified, added or removed.
	public void accumulateMassFromShapes() {
		if (!isDynamic()) {
			return;
		}

		// Reset the body's mass data.
		this.m = this.i = 0.0f;
		this.cog.set(0, 0);

		// Cache the position to realign it at the end.
		Vector2f pos = Util.cpv(this.getPosition());

		// Accumulate mass from shapes.
		for (Shape shape : shapes()) {
			MassInfo info = shape.massInfo;
			float m = info.m;

			if (m > 0.0f) {
				float msum = this.m + m;

				this.i += m * info.i + Util.cpvdistsq(this.cog, info.cog) * (m * this.m) / msum;
				this.cog = Util.cpvlerp(this.cog, info.cog, m / msum);
				this.m = msum;
			}
		}

		// Recalculate the inverses.
		this.m_inv = 1.0f / this.m;
		this.i_inv = 1.0f / this.i;

		// Realign the body since the CoG has probably moved.
		setPosition(pos);
		sanityCheck();
	}

	Shape getShapeList() {
		return shapeList;
	}

	@Override
	public String toString() {
		return "Body{" +
				"p=" + p +
				", v=" + v +
				", f=" + f +
				", a=" + a +
				", w=" + w +
				", t=" + t +
				", transform=" + transform +
				'}';
	}

	public static void applyImpulses(Body a, Body b, Vector2f r1, Vector2f r2, Vector2f j) {
		a.applyImpulse(cpvneg(j), r1);
		b.applyImpulse(j, r2);
	}

	public static void applyBiasImpulses(Body a, Body b, Vector2f r1, Vector2f r2, Vector2f j) {
		a.applyBiasImpulse(cpvneg(j), r1);
		b.applyBiasImpulse(j, r2);
	}
}
