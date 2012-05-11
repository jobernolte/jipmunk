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

/** @author jobernolte */
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

	/// Mass of the body.
	/// Must agree with cpBody.m_inv! Use cpBodySetMass() when changing the mass for this reason.
	float m;
	/// Mass inverse.
	float m_inv;

	/// Moment of inertia of the body.
	/// Must agree with cpBody.i_inv! Use cpBodySetMoment() when changing the moment for this reason.
	private float i;
	/// Moment of inertia inverse.
	float i_inv;

	/// Position of the rigid body's center of gravity.
	Vector2f p = Util.cpvzero();
	/// Velocity of the rigid body's center of gravity.
	Vector2f v = Util.cpvzero();
	/// Force acting on the rigid body's center of gravity.
	private Vector2f f = Util.cpvzero();

	/// Rotation of the body around it's center of gravity in radians.
	/// Must agree with cpBody.rot! Use cpBodySetAngle() when changing the angle for this reason.
	private float a;
	/// Angular velocity of the body around it's center of gravity in radians/second.
	float w = 0.0f;
	/// Torque applied to the body around it's center of gravity.
	private float t = 0.0f;

	/// Cached unit length vector representing the angle of the body.
	/// Used for fast rotations using cpvrotate().
	Vector2f rot = Util.cpvzero();

	/// Maximum velocity allowed when updating the velocity.
	private float v_limit = Float.POSITIVE_INFINITY;
	/// Maximum rotational rate (in radians/second) allowed when updating the angular velocity.
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

	public void setMass(float mass) {
		activate();
		this.m = mass;
		this.m_inv = 1.0f / mass;
	}

	public void setMoment(float moment) {
		activate();
		this.i = moment;
		this.i_inv = 1.0f / moment;
	}

	public float getMoment() {
		return i;
	}

	public float getInverseMoment() {
		return i_inv;
	}

	public boolean isRogue() {
		return space == null;
	}

	public boolean isSleeping() {
		return node.root != null;
	}

	public boolean isIdle() {
		return node.idleTime > 0;
	}

	public void setPosition(final Vector2f pos) {
		activate();
		this.p.set(pos);
	}

	public Vector2f getPosition() {
		return p;
	}

	public Vector2f getVelocity() {
		return v;
	}

	public void setVelocity(Vector2f v) {
		activate();
		this.v = v;
	}

	public Vector2f getRotation() {
		return rot;
	}

	private void setAngle(float angle) {
		this.a = angle;
		this.rot = cpvforangle(angle);
	}

	public void setAngleInRadians(float angle) {
		activate();
		setAngle(angle);
	}

	public float getAngleInRadians() {
		return a;
	}

	public Vector2f getForce() {
		return f;
	}

	public void setForce(Vector2f f) {
		this.f = f;
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

	public void resetForces() {
		activate();
		this.f = cpvzero();
		this.t = 0.0f;
	}

	public void applyForce(final Vector2f force, final Vector2f r) {
		activate();
		f = cpvadd(f, force);
		t += cpvcross(r, force);
	}

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
			componentActivate(node.root);
		}
	}

	public void addShape(Shape shape) {
		Shape next = shapeList;
		if (next != null) next.prev = shape;
		shape.next = next;
		shapeList = shape;
	}

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

	public Iterable<Arbiter> arbiters() {
		return new Iterable<Arbiter>() {
			@Override
			public Iterator<Arbiter> iterator() {
				return eachArbiterIterator(Body.this);
			}
		};
	}

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

	public static void updateVelocity(Body body, Vector2f gravity, float damping, float dt) {
		body.v = cpvclamp(cpvadd(cpvmult(body.v, damping), cpvmult(cpvadd(gravity, cpvmult(body.f, body.m_inv)),
				dt)), body.v_limit);

		float w_limit = body.w_limit;
		body.w = cpfclamp(body.w * damping + body.t * body.i_inv * dt, -w_limit, w_limit);

		cpBodySanityCheck(body);
	}

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

	public BodyVelocityFunc getVelocityFunc() {
		return velocityFunc;
	}

	public void setVelocityFunc(BodyVelocityFunc velocityFunc) {
		this.velocityFunc = velocityFunc;
	}

	public BodyPositionFunc getPositionFunc() {
		return positionFunc;
	}

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
