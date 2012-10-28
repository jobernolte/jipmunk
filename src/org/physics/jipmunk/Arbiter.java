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

/** @author jobernolte */
public class Arbiter {
	/**
	 * Calculated value to use for the elasticity coefficient. Override in a pre-solve collision handler for custom
	 * behavior.
	 */
	float e = 0;
	/**
	 * Calculated value to use for the friction coefficient. Override in a pre-solve collision handler for custom
	 * behavior.
	 */
	float u = 0;
	/**
	 * Calculated value to use for applying surface velocities. Override in a pre-solve collision handler for custom
	 * behavior.
	 */
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
	/**
	 * User definable data. Generally this points to your the game object class so you can access it when given a Body
	 * reference in a callback.
	 */
	private Object data;

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
		return this.swappedColl ? Util.cpvneg(n) : n;
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

	public Contact[] getContacts() {
		return contacts;
	}

	public int getNumContacts() {
		return numContacts;
	}

	public Vector2f totalImpulse() {
		Vector2f sum = Util.cpvzero();

		for (int i = 0, count = this.numContacts; i < count; i++) {
			Contact con = contacts[i];
			sum = Util.cpvadd(sum, Util.cpvmult(con.n, con.jnAcc));
		}

		return swappedColl ? sum : Util.cpvneg(sum);
	}

	public Vector2f totalImpulseWithFriction() {
		Vector2f sum = Util.cpvzero();

		for (int i = 0, count = this.numContacts; i < count; i++) {
			Contact con = contacts[i];
			sum = Util.cpvadd(sum, Util.cpvrotate(con.n, Util.cpv(con.jnAcc, con.jtAcc)));
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

		if (this.contacts == null) {
			this.contacts = contacts.toArray(new Contact[contacts.size()]);
		} else {
			this.contacts = contacts.toArray(this.contacts);
		}
		this.numContacts = numContacts;

		this.handler = handler;
		this.swappedColl = (a.collision_type != handler.a);

		this.e = a.e * b.e;
		this.u = a.u * b.u;
		this.surface_vr = Util.cpvsub(a.surface_v, b.surface_v);

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
			/*con.r1 = Util.cpvsub(con.p, a.p);
			con.r2 = Util.cpvsub(con.p, b.p);*/
			con.r1.set(con.p.getX() - a.p.getX(), con.p.getY() - a.p.getY());
			con.r2.set(con.p.getX() - b.p.getX(), con.p.getY() - b.p.getY());

			// Calculate the mass normal and mass tangent.
			con.nMass = 1.0f / Util.k_scalar(a, b, con.r1, con.r2, con.n);
			con.tMass = 1.0f / Util.k_scalar(a, b, con.r1, con.r2, Util.cpvperp(con.n));

			// Calculate the target bias velocity.
			con.bias = -bias * Util.cpfmin(0.0f, con.dist + slop) / dt;
			con.jBias = 0.0f;

			// Calculate the target bounce velocity.
			con.bounce = Util.normal_relative_velocity(a, b, con.r1, con.r2, con.n) * this.e;
		}
	}

	void applyCachedImpulse(float dt_coef) {
		if (isFirstContact()) return;

		Body a = this.body_a;
		Body b = this.body_b;

		for (int i = 0; i < this.numContacts; i++) {
			Contact con = this.contacts[i];
			Vector2f j = Util.cpvrotate(con.n, Util.cpv(con.jnAcc, con.jtAcc));
			Util.apply_impulses(a, b, con.r1, con.r2, Util.cpvmult(j, dt_coef));
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
			Vector2f vb1 = Util.cpvadd(a.v_bias, Util.cpvmult(Util.cpvperp(r1), a.w_bias));
			Vector2f vb2 = Util.cpvadd(b.v_bias, Util.cpvmult(Util.cpvperp(r2), b.w_bias));
			Vector2f vr = Util.relative_velocity(a, b, r1, r2);

			float vbn = Util.cpvdot(Util.cpvsub(vb2, vb1), n);
			// Calculate the relative velocity.
			float vrn = Util.cpvdot(vr, n);
			// Calculate the relative tangent velocity.
			float vrt = Util.cpvdot(Util.cpvadd(vr, this.surface_vr), Util.cpvperp(n));

			// Calculate and clamp the bias impulse.
			float jbn = (con.bias - vbn) * con.nMass;
			float jbnOld = con.jBias;
			con.jBias = Util.cpfmax(jbnOld + jbn, 0.0f);

			// Calculate and clamp the normal impulse.
			float jn = -(con.bounce + vrn) * con.nMass;
			float jnOld = con.jnAcc;
			con.jnAcc = Util.cpfmax(jnOld + jn, 0.0f);

			// Calculate and clamp the friction impulse.
			float jtMax = this.u * con.jnAcc;
			float jt = -vrt * con.tMass;
			float jtOld = con.jtAcc;
			con.jtAcc = Util.cpfclamp(jtOld + jt, -jtMax, jtMax);

			// Apply the bias impulse.
			Util.apply_bias_impulses(a, b, r1, r2, Util.cpvmult(n, con.jBias - jbnOld));
			// Apply the final impulse.
			Util.apply_impulses(a, b, r1, r2, Util.cpvrotate(n, Util.cpv(con.jnAcc - jnOld, con.jtAcc - jtOld)));
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

	/** @return the user data */
	public Object getData() {
		return data;
	}

	/**
	 * @param clazz the {@link Class} of the user data
	 * @param <T>   the type of the data
	 * @return the user data
	 */
	public <T> T getData(Class<T> clazz) {
		return clazz.cast(data);
	}

	/**
	 * Sets user data. Use this data to get a reference to the game object that owns this body from callbacks.
	 *
	 * @param data the user data to set
	 */
	public void setData(Object data) {
		this.data = data;
	}
}
