package org.physics.jipmunk;

import java.util.Iterator;

import static org.physics.jipmunk.Util.apply_bias_impulses;
import static org.physics.jipmunk.Util.apply_impulses;
import static org.physics.jipmunk.Util.cpfclamp;
import static org.physics.jipmunk.Util.cpfmax;
import static org.physics.jipmunk.Util.cpfmin;
import static org.physics.jipmunk.Util.cpv;
import static org.physics.jipmunk.Util.cpvadd;
import static org.physics.jipmunk.Util.cpvdot;
import static org.physics.jipmunk.Util.cpvmult;
import static org.physics.jipmunk.Util.cpvneg;
import static org.physics.jipmunk.Util.cpvperp;
import static org.physics.jipmunk.Util.cpvrotate;
import static org.physics.jipmunk.Util.cpvsub;
import static org.physics.jipmunk.Util.cpvzero;
import static org.physics.jipmunk.Util.k_scalar;
import static org.physics.jipmunk.Util.normal_relative_velocity;
import static org.physics.jipmunk.Util.relative_velocity;

/** @author jobernolte */
public class Arbiter {
	/// Calculated value to use for the elasticity coefficient.
	/// Override in a pre-solve collision handler for custom behavior.
	float e = 0;
	/// Calculated value to use for the friction coefficient.
	/// Override in a pre-solve collision handler for custom behavior.
	float u = 0;
	/// Calculated value to use for applying surface velocities.
	/// Override in a pre-solve collision handler for custom behavior.
	Vector2f surface_vr = Util.cpvzero();

	Shape a;
	Shape b;
	Body body_a;
	Body body_b;

	ArbiterThread thread_a = new ArbiterThread();
	ArbiterThread thread_b = new ArbiterThread();

	int numContacts = 0;
	Contact[] contacts;

	int stamp = 0;
	CollisionHandlerEntry handler;
	boolean swappedColl = false;
	ArbiterState state = ArbiterState.cpArbiterStateFirstColl;

	/** @return Calculated value to use for the elasticity coefficient. */
	public float getElasticity() {
		return e;
	}

	/**
	 * The calculated elasticity for this collision pair. Setting the value in a {@link CollisionHandler#preSolve (Arbiter,
	 * Space)}  callback will override the value calculated by the space.
	 *
	 * @param e the calculated elasticity
	 */
	public void setElasticity(float e) {
		this.e = e;
	}

	/** @return The calculated friction for this collision pair. */
	public float getFriction() {
		return u;
	}

	/**
	 * The calculated friction for this collision pair. Setting the value in a {@link CollisionHandler#preSolve (Arbiter,
	 * Space)} callback will override the value calculated by the space.
	 *
	 * @param u the calculated friction
	 */
	public void setFriction(float u) {
		this.u = u;
	}

	/** @return The calculated surface velocity for this collision pair. */
	public Vector2f getSurfaceVelocity() {
		return surface_vr;
	}

	/**
	 * The calculated surface velocity for this collision pair. Setting the value in a {@link
	 * CollisionHandler#preSolve(Arbiter, Space)} callback will override the value calculated by the space.
	 *
	 * @param surface_vr the calculated surface velocity
	 */
	public void setSurfaceVelocity(Vector2f surface_vr) {
		this.surface_vr = surface_vr;
	}

	public Shape getShapeA() {
		return a;
	}

	public Shape getShapeB() {
		return b;
	}

	public Body getBodyA() {
		return body_a;
	}

	public Body getBodyB() {
		return body_b;
	}

	public Iterable<ContactPoint> contacts() {
		return new Iterable<ContactPoint>() {
			@Override
			public Iterator<ContactPoint> iterator() {
				return new Iterator<ContactPoint>() {
					ContactPoint contact = new ContactPoint();
					int i = 0;

					@Override
					public boolean hasNext() {
						return i < contacts.length;
					}

					@Override
					public ContactPoint next() {
						contact.point = contacts[i].p;
						contact.normal = contacts[i].n;
						contact.dist = contacts[i].dist;
						i++;
						return contact;
					}

					@Override
					public void remove() {
						throw new UnsupportedOperationException("removal not allowed");
					}
				};
			}
		};
	}

	void init(Shape a, Shape b) {
		this.handler = null;
		this.surface_vr = Util.cpvzero();
		this.a = a;
		this.b = b;
		this.body_a = a.getBody();
		this.body_b = b.getBody();
		this.e = 0;
		this.u = 0;
		this.numContacts = 0;
		this.contacts = null;
		this.swappedColl = false;
		this.thread_a.next = null;
		this.thread_b.next = null;
		this.thread_a.prev = null;
		this.thread_b.prev = null;
		this.stamp = 0;
		this.state = ArbiterState.cpArbiterStateFirstColl;
	}

	void reset(ContactList contactList) {
		this.a = this.b = null;
		this.body_a = this.body_b = null;
		contactList.free(this.contacts);
		this.contacts = null;
		this.numContacts = 0;
		this.handler = null;
		this.thread_a.next = null;
		this.thread_b.next = null;
		this.thread_a.prev = null;
		this.thread_b.prev = null;
	}

	ArbiterThread threadForBody(Body body) {
		return (this.body_a == body ? this.thread_a : this.thread_b);
	}

	void ignore() {
		this.state = ArbiterState.cpArbiterStateIgnore;
	}

	void unthreadHelper(Body body) {
		ArbiterThread thread = threadForBody(body);
		Arbiter prev = thread.prev;
		Arbiter next = thread.next;

		if (prev != null) {
			prev.threadForBody(body).next = next;
		} else {
			body.arbiterList = next;
		}

		if (next != null) {
			next.threadForBody(body).prev = prev;
		}

		thread.prev = null;
		thread.next = null;
	}

	void unthread() {
		unthreadHelper(this.body_a);
		unthreadHelper(this.body_b);
	}

	public Vector2f getNormal(int i) {
		assert 0 <= i && i < this.numContacts : "Index error: The specified contact index is invalid for this arbiter";

		Vector2f n = this.contacts[i].n;
		return this.swappedColl ? cpvneg(n) : n;
	}

	public Vector2f getPoint(int i) {
		assert 0 <= i && i < this.numContacts : "Index error: The specified contact index is invalid for this arbiter";

		return this.contacts[i].p;
	}

	public float getDepth(int i) {
		assert 0 <= i && i < this.numContacts : "Index error: The specified contact index is invalid for this arbiter";

		return this.contacts[i].dist;
	}

	public ContactPoint[] getContactPointSet() {
		ContactPoint[] set = new ContactPoint[numContacts];

		int i;
		for (i = 0; i < numContacts; i++) {
			/*set.points[i].point = arb - > CP_PRIVATE(contacts)[i].CP_PRIVATE(p);
						set.points[i].normal = arb - > CP_PRIVATE(contacts)[i].CP_PRIVATE(n);
						set.points[i].dist = arb - > CP_PRIVATE(contacts)[i].CP_PRIVATE(dist);*/
			// TODO Contact can use ContactPoint
			set[i] = new ContactPoint(contacts[i].p, contacts[i].n, contacts[i].dist);
		}

		return set;
	}

	public Vector2f totalImpulse() {
		Vector2f sum = cpvzero();

		for (int i = 0, count = this.numContacts; i < count; i++) {
			Contact con = contacts[i];
			sum = cpvadd(sum, cpvmult(con.n, con.jnAcc));
		}

		return swappedColl ? sum : Util.cpvneg(sum);
	}

	public Vector2f totalImpulseWithFriction() {
		Vector2f sum = cpvzero();

		for (int i = 0, count = this.numContacts; i < count; i++) {
			Contact con = contacts[i];
			sum = cpvadd(sum, cpvrotate(con.n, cpv(con.jnAcc, con.jtAcc)));
		}

		return swappedColl ? sum : Util.cpvneg(sum);
	}

	public float totalKE() {
		float eCoef = (1 - this.e) / (1 + this.e);
		float sum = 0.0f;

		for (int i = 0, count = this.numContacts; i < count; i++) {
			Contact con = contacts[i];
			float jnAcc = con.jnAcc;
			float jtAcc = con.jtAcc;

			sum += eCoef * jnAcc * jnAcc / con.nMass + jtAcc * jtAcc / con.tMass;
		}

		return sum;
	}

	void update(ContactList contacts, int numContacts, CollisionHandlerEntry handler, Shape a, Shape b) {
		// Arbiters without contact data may exist if a collision function rejected the collision.
		if (this.contacts != null) {
			// Iterate over the possible pairs to look for hash value matches.
			for (int i = 0; i < this.numContacts; i++) {
				Contact old = this.contacts[i];

				for (int j = 0; j < numContacts; j++) {
					Contact new_contact = contacts.get(j);

					// This could trigger false positives, but is fairly unlikely nor serious if it does.
					if (new_contact.hash == old.hash) {
						// Copy the persistant contact information.
						new_contact.jnAcc = old.jnAcc;
						new_contact.jtAcc = old.jtAcc;
					}
				}
				contacts.free(old);
			}
		}

		this.contacts = contacts.toArray(new Contact[contacts.size()]);
		this.numContacts = numContacts;

		this.handler = handler;
		this.swappedColl = (a.collision_type != handler.a);

		this.e = a.e * b.e;
		this.u = a.u * b.u;
		this.surface_vr = cpvsub(a.surface_v, b.surface_v);

		// For collisions between two similar primitive types, the order could have been swapped.
		this.a = a;
		this.body_a = a.body;
		this.b = b;
		this.body_b = b.body;

		// mark it as new if it's been cached
		if (this.state == ArbiterState.cpArbiterStateCached) {
			this.state = ArbiterState.cpArbiterStateFirstColl;
		}
	}

	void preStep(float dt, float slop, float bias) {
		Body a = this.body_a;
		Body b = this.body_b;

		for (int i = 0; i < this.numContacts; i++) {
			Contact con = this.contacts[i];

			// Calculate the offsets.
			con.r1 = cpvsub(con.p, a.p);
			con.r2 = cpvsub(con.p, b.p);

			// Calculate the mass normal and mass tangent.
			con.nMass = 1.0f / k_scalar(a, b, con.r1, con.r2, con.n);
			con.tMass = 1.0f / k_scalar(a, b, con.r1, con.r2, cpvperp(con.n));

			// Calculate the target bias velocity.
			con.bias = -bias * cpfmin(0.0f, con.dist + slop) / dt;
			con.jBias = 0.0f;

			// Calculate the target bounce velocity.
			con.bounce = normal_relative_velocity(a, b, con.r1, con.r2, con.n) * this.e;
		}
	}

	void applyCachedImpulse(float dt_coef) {
		if (isFirstContact()) return;

		Body a = this.body_a;
		Body b = this.body_b;

		for (int i = 0; i < this.numContacts; i++) {
			Contact con = this.contacts[i];
			Vector2f j = cpvrotate(con.n, cpv(con.jnAcc, con.jtAcc));
			apply_impulses(a, b, con.r1, con.r2, cpvmult(j, dt_coef));
		}
	}

	// TODO is it worth splitting velocity/position correction?

	void applyImpulse() {
		Body a = this.body_a;
		Body b = this.body_b;

		for (int i = 0; i < this.numContacts; i++) {
			Contact con = this.contacts[i];
			Vector2f n = con.n;
			Vector2f r1 = con.r1;
			Vector2f r2 = con.r2;

			// Calculate the relative bias velocities.
			Vector2f vb1 = cpvadd(a.v_bias, cpvmult(cpvperp(r1), a.w_bias));
			Vector2f vb2 = cpvadd(b.v_bias, cpvmult(cpvperp(r2), b.w_bias));
			float vbn = cpvdot(cpvsub(vb2, vb1), n);

			// Calculate and clamp the bias impulse.
			float jbn = (con.bias - vbn) * con.nMass;
			float jbnOld = con.jBias;
			con.jBias = cpfmax(jbnOld + jbn, 0.0f);
			jbn = con.jBias - jbnOld;

			// Apply the bias impulse.
			apply_bias_impulses(a, b, r1, r2, cpvmult(n, jbn));

			// Calculate the relative velocity.
			Vector2f vr = relative_velocity(a, b, r1, r2);
			float vrn = cpvdot(vr, n);

			// Calculate and clamp the normal impulse.
			float jn = -(con.bounce + vrn) * con.nMass;
			float jnOld = con.jnAcc;
			con.jnAcc = cpfmax(jnOld + jn, 0.0f);
			jn = con.jnAcc - jnOld;

			// Calculate the relative tangent velocity.
			float vrt = cpvdot(cpvadd(vr, this.surface_vr), cpvperp(n));

			// Calculate and clamp the friction impulse.
			float jtMax = this.u * con.jnAcc;
			float jt = -vrt * con.tMass;
			float jtOld = con.jtAcc;
			con.jtAcc = cpfclamp(jtOld + jt, -jtMax, jtMax);
			jt = con.jtAcc - jtOld;

			// Apply the final impulse.
			apply_impulses(a, b, r1, r2, cpvrotate(n, cpv(jn, jt)));
		}
	}

	public boolean isFirstContact() {
		return state == ArbiterState.cpArbiterStateFirstColl;
	}

	static Arbiter arbiterNext(Arbiter node, Body body) {
		return (node.body_a == body ? node.thread_a.next : node.thread_b.next);
	}

	void callSeparate(Space space) {
		// The handler needs to be looked up again as the handler cached on the arbiter may have been deleted since
		// the last step.
		CollisionHandler handler = space.lookupHandler(this.a.collision_type, this.b.collision_type);
		handler.separate(this, space);
	}

	static void cpArbiterCallSeparate(Arbiter arb, Space space) {
		arb.callSeparate(space);
	}

	static void cpArbiterApplyCachedImpulse(Arbiter arb, float dt_coef) {
		arb.applyCachedImpulse(dt_coef);
	}

	static void cpArbiterApplyImpulse(Arbiter arb) {
		arb.applyImpulse();
	}

	static void cpArbiterPreStep(Arbiter arb, float dt, float slop, float bias) {
		arb.preStep(dt, slop, bias);
	}
}
