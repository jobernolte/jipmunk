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

import static org.physics.jipmunk.Array.cpArrayDeleteObj;
import static org.physics.jipmunk.Assert.cpAssertSoft;
import static org.physics.jipmunk.Util.cpfclamp;
import static org.physics.jipmunk.Util.cpvadd;
import static org.physics.jipmunk.Util.cpvclamp;
import static org.physics.jipmunk.Util.cpvcross;
import static org.physics.jipmunk.Util.cpvdot;
import static org.physics.jipmunk.Util.cpvforangle;
import static org.physics.jipmunk.Util.cpvmult;
import static org.physics.jipmunk.Util.cpvrotate;
import static org.physics.jipmunk.Util.cpvsub;
import static org.physics.jipmunk.Util.cpvunrotate;
import static org.physics.jipmunk.Util.cpvzero;

/**
 * <b>Rogue and Static Bodies:</b>
 * <p/>
 * Normally when you create a rigid body, you add it to a space so the space will start simulating it. This means it
 * will update it’s position and velocity, apply forces to it, be affected by gravity, etc. A body that isn’t added to a
 * space (and not simulated) is called a rogue body. The most important use for rogue bodies are as static bodies, but
 * you can also use them to implement directly controlled objects such as moving platforms.
 * <p/>
 * Static bodies are rogue bodies, but with a special flag set on them to let Chipmunk know that they never move unless
 * you tell it. Static bodies have two purposes. Originally they were added for the sleeping feature. Because static
 * bodies don’t move, Chipmunk knows that it’s safe to let objects that are touching or jointed to them fall asleep.
 * Objects touching or jointed to regular rogue bodies are never allowed to sleep. The second purpose for static bodies
 * is that Chipmunk knows shapes attached to them never need to have their collision detection data updated. Chipmunk
 * also doesn’t need to bother checking for collisions between static objects. Generally all of your level geometry will
 * be attached to a static body except for things like moving platforms or doors.
 * <p/>
 * In previous versions of Chipmunk before 5.3 you would create an infinite mass rogue body to attach static shapes to
 * using cpSpaceAddStaticShape(). You don’t need to do any of that anymore, and shouldn’t if you want to use the
 * sleeping feature. Each space has a dedicated static body that you can use to attach your static shapes to. Chipmunk
 * also automatically adds shapes attached to static bodies as static shapes. *
 * <p/>
 * <b>Creating Additional Static Bodies:</b>
 * <p/>
 * While every cpSpace has a built in static body that you can use, it can be convenient to make your own as well. One
 * potential use is in a level editor. By attaching chunks of your level to static bodies, you can still move and rotate
 * the chunks independently of each other. Then all you have to do is call cpSpaceRehashStatic() to rebuild the static
 * collision detection data when you are done.
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
			body.node.root = root;
			if (body != root) {
				body.node.next = root.node.next;
				root.node.next = body;
			}
		}
	}

	private final static BodyVelocityFunc defaultBodyVelocityFunc = new BodyVelocityFunc() {
		@Override
		public void velocity(Body body, Vector2f gravity, float damping, float dt) {
			updateVelocity(body, gravity, damping, dt);
		}
	};

	private final static BodyPositionFunc defaultBodyPositionFunc = new BodyPositionFunc() {
		@Override
		public void position(Body body, float dt) {
			updatePosition(body, dt);
		}
	};

	/** Function that is called to integrate the body's velocity. (Defaults to cpBodyUpdateVelocity) */
	BodyVelocityFunc velocityFunc = defaultBodyVelocityFunc;
	/** Function that is called to integrate the body's position. (Defaults to cpBodyUpdatePosition) */
	BodyPositionFunc positionFunc = defaultBodyPositionFunc;

	/** Mass of the body. Must agree with cpBody.m_inv! Use cpBodySetMass() when changing the mass for this reason. */
	float m;
	/** Mass inverse. */
	float m_inv;

	/**
	 * Moment of inertia of the body. Must agree with cpBody.i_inv! Use cpBodySetMoment() when changing the moment for this
	 * reason.
	 */
	private float i;
	/// Moment of inertia inverse.
	float i_inv;

	/** Position of the rigid body's center of gravity. */
	Vector2f p = Util.cpvzero();
	/** Velocity of the rigid body's center of gravity. */
	Vector2f v = Util.cpvzero();
	/** Force acting on the rigid body's center of gravity. */
	private Vector2f f = Util.cpvzero();

	/**
	 * Rotation of the body around it's center of gravity in radians. Must agree with cpBody.rot! Use cpBodySetAngle() when
	 * changing the angle for this reason.
	 */
	private float a;
	/** Angular velocity of the body around it's center of gravity in radians/second. */
	float w = 0.0f;
	/** Torque applied to the body around it's center of gravity. */
	private float t = 0.0f;

	/** Cached unit length vector representing the angle of the body. Used for fast rotations using cpvrotate(). */
	Vector2f rot = Util.cpvzero();

	/** Maximum velocity allowed when updating the velocity. */
	private float v_limit = Float.POSITIVE_INFINITY;
	/** Maximum rotational rate (in radians/second) allowed when updating the angular velocity. */
	private float w_limit = Float.POSITIVE_INFINITY;

	Vector2f v_bias = Util.cpvzero();
	float w_bias = 0;
	Space space;
	ComponentNode node = new ComponentNode();
	private Shape shapeList;
	Arbiter arbiterList;
	Constraint constraintList;

	/**
	 * Creates a new body with the given mass and moment.
	 *
	 * @param m the mass of the body
	 * @param i the moment of the body
	 */
	public Body(float m, float i) {
		setMass(m);
		setMoment(i);
		setAngleInRadians(0);
	}

	public static Body createStatic() {
		Body body = new Body(Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY);
		body.node.idleTime = Float.POSITIVE_INFINITY;
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
	}

	/**
	 * Moment of inertia (MoI or sometimes just moment) of the body. The moment is like the rotational mass of a body. See
	 * below for function to help calculate the moment.
	 *
	 * @param moment the moment of inertia of the body
	 * @see Util#momentForBox(float, float, float)
	 * @see Util#momentForCircle(float, float, float, Vector2f)
	 * @see Util#momentForPoly(float, Vector2f[], Vector2f)
	 * @see Util#momentForPoly(float, Vector2f[], int, int, Vector2f)
	 * @see Util#momentForSegment(float, Vector2f, Vector2f)
	 */
	public void setMoment(float moment) {
		activate();
		this.i = moment;
		this.i_inv = 1.0f / moment;
	}

	/** @return the moment of inertia of the body */
	public float getMoment() {
		return i;
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
		return node.root != null;
	}

	/** @return <code>true</code> if the body is idle */
	public boolean isIdle() {
		return node.idleTime > 0;
	}

	/**
	 * Position of the center of gravity of the body. When changing the position you may also want to call {@link
	 * Space#reindexShapesForBody(Body)} to update the collision detection information for the attached shapes if plan to
	 * make any queries against the space.
	 *
	 * @param pos position of the center of gravity of the body
	 */
	public void setPosition(final Vector2f pos) {
		activate();
		this.p.set(pos);
	}

	/** @return position of the center of gravity of the body */
	public Vector2f getPosition() {
		return p;
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
	}

	/**
	 * @return The rotation vector for the body. Can be used with {@link Util#cpvrotate(Vector2f, Vector2f)} or {@link
	 *         Util#cpvunrotate(Vector2f, Vector2f)} to perform fast rotations.
	 */
	public Vector2f getRotation() {
		return rot;
	}

	private void setAngle(float angle) {
		this.a = angle;
		this.rot = cpvforangle(angle);
	}

	/**
	 * Rotation of the body in radians. When changing the rotation you may also want to call {@link
	 * Space#reindexShapesForBody(Body)} to update the collision detection information for the attached shapes if plan to
	 * make any queries against the space.
	 *
	 * @param angle the angle in radians
	 */
	public void setAngleInRadians(float angle) {
		activate();
		setAngle(angle);
	}

	/** @return the rotation angle in radians */
	public float getAngleInRadians() {
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
		this.f.set(f);
	}

	/** @return The angular velocity of the body in radians per second. */
	public float getAngVel() {
		return w;
	}

	/**
	 * Set the angular velocity of the body in radians per second.
	 *
	 * @param w the angular velocity in radians
	 */
	public void setAngVel(float w) {
		this.w = w;
	}

	public void addAngVel(float w) {
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
		this.t = t;
	}

	/** Zero both the forces and torques currently applied to the body. */
	public void resetForces() {
		activate();
		this.f = cpvzero();
		this.t = 0.0f;
	}

	/**
	 * Add the force <code>force</code> to body at a relative offset <code>r</code> from the center of gravity.
	 *
	 * @param force the force to apply
	 * @param r     the relative offset
	 */
	public void applyForce(final Vector2f force, final Vector2f r) {
		activate();
		f = cpvadd(f, force);
		t += cpvcross(r, force);
	}

	/**
	 * Add the impulse <code>i</code> to body at a relative offset <code>r</code> from the center of gravity.
	 *
	 * @param j the impulse to apply
	 * @param r the relative offset
	 */
	public void applyImpulse(final Vector2f j, final Vector2f r) {
		activate();
		v = cpvadd(v, cpvmult(j, m_inv));
		w += i_inv * cpvcross(r, j);
	}

	private void sanityCheck() {

	}

	static void componentActivate(Body root) {
		if (root == null || !root.isSleeping()) {
			return;
		}
		cpAssertSoft(!cpBodyIsRogue(root), "Internal Error: ComponentActivate() called on a rogue body.");
		Space space = root.space;
		Body body = root;
		while (body != null) {
			Body next = body.node.next;

			body.node.idleTime = 0;
			body.node.root = null;
			body.node.next = null;
			space.activateBody(body);

			body = next;
		}
		cpArrayDeleteObj(space.sleepingComponents, root);
	}

	void activate() {
		if (!isRogue()) {
			node.idleTime = 0;
			componentActivate(SpaceComponent.ComponentRoot(this));
		}
	}

	/**
	 * Adds the given shape to this body.
	 *
	 * @param shape the {@link Shape} to add
	 */
	public void addShape(Shape shape) {
		Shape next = shapeList;
		if (next != null) next.prev = shape;
		shape.next = next;
		shapeList = shape;
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

				arb.swappedColl = (body == arb.body_b);
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
	 * This one is more interesting. Returns an {@link Arbiter} for each collision pair that body is involved in. Calling
	 * {@link Arbiter#getBodyA()} or {@link Arbiter#getShapeA()} will return the body or shape for body as the first
	 * argument. You can use this to check all sorts of collision information for a body like if it’s touching the ground,
	 * another particular object, how much collision force is being applied to an object, etc. Note: This function only
	 * works if the contact graph is enabled either by enabling the sleeping feature of a space or by enabling the contact
	 * graph. Sensor shapes and arbiters that have been rejected by a collision handler callback or cpArbiterIgnore() are
	 * not tracked by the contact graph.
	 *
	 * @return {@link Iterable<Arbiter>} which can be used to iterate all collision pairs
	 */
	public Iterable<Arbiter> arbiters() {
		return new Iterable<Arbiter>() {
			@Override
			public Iterator<Arbiter> iterator() {
				return eachArbiterIterator(Body.this);
			}
		};
	}

	/** @return an {@link Iterable<Shape>} which can be used to iterate over all shapes of this body */
	public Iterable<Shape> shapes() {
		return new Iterable<Shape>() {
			@Override
			public Iterator<Shape> iterator() {
				return new Iterator<Shape>() {
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
		};
	}

	Iterable<Body> components() {
		return new Iterable<Body>() {
			@Override
			public Iterator<Body> iterator() {
				return new Iterator<Body>() {
					Body var = Body.this;

					@Override
					public boolean hasNext() {
						return var != null;
					}

					@Override
					public Body next() {
						Body result = var;
						var = var.node.next;
						return result;
					}

					@Override
					public void remove() {
						throw new UnsupportedOperationException("removal not allowed");
					}
				};
			}
		};
	}

	/**
	 * @return returns true if body is a static body. Either {@link Space#getStaticBody()} or a body created with {@link
	 *         Body#createStatic()}.
	 */
	public boolean isStatic() {
		return node.idleTime == Float.POSITIVE_INFINITY;
	}

	static boolean cpBodyIsStatic(Body body) {
		return body.isStatic();
	}

	static boolean cpBodyIsSleeping(Body body) {
		return body.isSleeping();
	}

	static boolean cpBodyIsRogue(Body body) {
		return body.isStatic();
	}

	/// Get the kinetic energy of a body.
	static float cpBodyKineticEnergy(Body body) {
		// Need to do some fudging to avoid NaNs
		float vsq = cpvdot(body.v, body.v);
		float wsq = body.w * body.w;
		return (vsq != 0 ? vsq * body.m : 0.0f) + (wsq != 0 ? wsq * body.i : 0.0f);
	}

	private static void cpBodySanityCheck(Body body) {
		body.sanityCheck();
	}

	/**
	 * The default integration function for updating the velocity of a body.
	 *
	 * @param body    the body for which to update the velocity
	 * @param gravity the gravity to apply
	 * @param damping the damping to apply
	 * @param dt      the timestep to use
	 */
	public static void updateVelocity(Body body, Vector2f gravity, float damping, float dt) {
		body.v = cpvclamp(cpvadd(cpvmult(body.v, damping), cpvmult(cpvadd(gravity, cpvmult(body.f, body.m_inv)),
				dt)), body.v_limit);

		float w_limit = body.w_limit;
		body.w = cpfclamp(body.w * damping + body.t * body.i_inv * dt, -w_limit, w_limit);

		cpBodySanityCheck(body);
	}

	/**
	 * The default integration function updating the position of a body.
	 *
	 * @param body the body for which to update the position
	 * @param dt   the timestep to use
	 */
	public static void updatePosition(Body body, float dt) {
		body.p = cpvadd(body.p, cpvmult(cpvadd(body.v, body.v_bias), dt));
		body.setAngle(body.a + (body.w + body.w_bias) * dt);

		body.v_bias = cpvzero();
		body.w_bias = 0.0f;

		cpBodySanityCheck(body);
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
		return new Iterable<Constraint>() {
			@Override
			public Iterator<Constraint> iterator() {
				return new Iterator<Constraint>() {
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
	 * @param v the coordinates to convert
	 * @return the converted coordinates
	 */
	public Vector2f local2World(final Vector2f v) {
		return cpvadd(this.p, cpvrotate(v, this.rot));
	}

	/**
	 * Convert body absolute/world coordinates to  relative/local coordinates.
	 *
	 * @param v the coordinates to convert
	 * @return the converted coordinates
	 */
	public Vector2f world2Local(final Vector2f v) {
		return cpvunrotate(cpvsub(v, this.p), this.rot);
	}
}
