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

import org.physics.jipmunk.impl.Contact;

import java.util.Iterator;
import java.util.List;

import static org.physics.jipmunk.Assert.cpAssertHard;
import static org.physics.jipmunk.Util.*;

/**
 * jipmunk’s Arbiter class encapsulates a pair of colliding shapes and all of the data about their collision.
 * <p>
 * Why are they called arbiters? The short answer is that I kept using the word “arbitrates” to describe the way that
 * collisions were resolved and then I saw that Box2D actually called them arbiters way back in 2006 when I was looking
 * at its solver. An arbiter is like a judge, a person that has authority to settle disputes between two people. It was
 * a fun, fitting name and was shorter to type than CollisionPair which I had been using. It was originally meant to be
 * a private internal structure only, but evolved to be useful from callbacks.
 *
 * @author jobernolte
 */
public class Arbiter {

	private final static CollisionHandler DO_NOTHING =
			new CollisionHandler(CollisionType.WILDCARD, CollisionType.WILDCARD, CollisionHandler::alwaysCollide,
								 CollisionHandler::alwaysCollide, CollisionHandler::doNothing,
								 CollisionHandler::doNothing);
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
	List<Contact> contacts;
	Vector2f normal = Util.cpvzero();
	CollisionHandler handler;
	CollisionHandler handlerA;
	CollisionHandler handlerB;
	boolean swapped = false;
	int stamp = 0;
	ArbiterState state = ArbiterState.FIRST_COLLISION;
	/**
	 * User definable data. Generally this points to your the game object class so you can access it when given a Body
	 * reference in a callback.
	 */
	private Object data;

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

	void init(Shape a, Shape b) {
		this.handler = null;
		this.handlerA = null;
		this.handlerB = null;
		this.swapped = false;
		///
		this.e = 0;
		this.u = 0;
		this.surface_vr.set(0, 0);
		///
		this.contacts = null;
		///
		this.a = a;
		this.b = b;
		this.body_a = a.getBody();
		this.body_b = b.getBody();
		///
		this.thread_a.next = null;
		this.thread_b.next = null;
		this.thread_a.prev = null;
		this.thread_b.prev = null;
		///
		this.stamp = 0;
		this.state = ArbiterState.FIRST_COLLISION;
	}

	void reset() {
		this.handler = this.handlerA = this.handlerB = null;
		this.a = this.b = null;
		this.body_a = this.body_b = null;
		this.contacts = null;
	}

	ArbiterThread threadForBody(Body body) {
		return (this.body_a == body ? this.thread_a : this.thread_b);
	}

	public boolean ignore() {
		this.state = ArbiterState.IGNORE;
		return false;
	}

	void unthreadHelper(Body body) {
		ArbiterThread thread = threadForBody(body);
		Arbiter prev = thread.prev;
		Arbiter next = thread.next;

		if (prev != null) {
			prev.threadForBody(body).next = next;
		} else if (body.arbiterList == this) {
			// IFF prev is NULL and body->arbiterList == arb, is arb at the head of the list.
			// This function may be called for an arbiter that was never in a list.
			// In that case, we need to protect it from wiping out the body.arbiterList pointer.
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

	public boolean isFirstContact() {
		return this.state == ArbiterState.FIRST_COLLISION;
	}

	boolean isRemoval() {
		return this.state == ArbiterState.INVALIDATED;
	}

	public int getCount() {
		return (state.ordinal() < ArbiterState.CACHED.ordinal() && contacts != null) ? contacts.size() : 0;
	}

	public Vector2f getNormal() {
		return cpvmult(this.normal, this.swapped ? -1.0f : 1.0f);
	}

	public Vector2f getPoint1(int i) {
		cpAssertHard(0 <= i && i < getCount(), "Index error: The specified contact index is invalid for this arbiter");
		return cpvadd(this.body_a.p, this.contacts.get(i).getR1());
	}

	public Vector2f getPoint2(int i) {
		cpAssertHard(0 <= i && i < getCount(), "Index error: The specified contact index is invalid for this arbiter");
		return cpvadd(this.body_b.p, this.contacts.get(i).getR2());
	}

	public float getDepth(int i) {
		cpAssertHard(0 <= i && i < getCount(), "Index error: The specified contact index is invalid for this arbiter");

		final Contact con = this.contacts.get(i);
		return cpvdot(cpvadd(cpvsub(con.getR2(), con.getR1()), cpvsub(this.body_b.p, this.body_a.p)), this.normal);
	}

	List<Contact> getContacts() {
		return contacts;
	}

	public ContactPointSet getContactPointSet() {
		boolean swapped = this.swapped;
		Vector2f n = this.normal;
		ContactPoint[] points = new ContactPoint[getCount()];

		for (int i = 0; i < points.length; i++) {
			// Contact points are relative to body CoGs;
			final Contact contact = this.contacts.get(i);
			Vector2f p1 = cpvadd(this.body_a.p, contact.getR1());
			Vector2f p2 = cpvadd(this.body_b.p, contact.getR2());

			points[i].point1 = (swapped ? p2 : p1);
			points[i].point2 = (swapped ? p1 : p2);
			points[i].distance = cpvdot(cpvsub(p2, p1), n);
		}

		return new ContactPointSet(swapped ? cpvneg(n) : new Vector2f(n), points);
	}

	public void setContactPointSet(ContactPointSet set) {
		int count = set.getCount();
		if (count != getCount()) {
			throw new IllegalArgumentException("The number of contact points cannot be changed.");
		}

		boolean swapped = this.swapped;
		this.normal.set(swapped ? cpvneg(set.getNormal()) : set.getNormal());

		ContactPoint[] points = set.getPoints();
		for (int i = 0; i < count; i++) {
			// Convert back to CoG relative offsets.
			Vector2f p1 = points[i].point1;
			Vector2f p2 = points[i].point2;
			Contact contact = this.contacts.get(i);
			contact.getR1().set(cpvsub(swapped ? p2 : p1, this.body_a.getPosition()));
			contact.getR2().set(cpvsub(swapped ? p1 : p2, this.body_b.getPosition()));
		}
	}

	public Iterable<ContactPoint> contactPoints() {
		return () -> new Iterator<ContactPoint>() {
			ContactPoint contact = new ContactPoint();
			int i = 0;

			@Override
			public boolean hasNext() {
				return i < getCount();
			}

			@Override
			public ContactPoint next() {
				final Contact contact1 = contacts.get(i);
				// Contact points are relative to body CoGs;
				Vector2f p1 = cpvadd(Arbiter.this.body_a.p, contact1.getR1());
				Vector2f p2 = cpvadd(Arbiter.this.body_b.p, contact1.getR2());

				contact.point1 = (swapped ? p2 : p1);
				contact.point2 = (swapped ? p1 : p2);
				contact.distance = cpvdot(cpvsub(p2, p1), normal);
				i++;
				return contact;
			}

			@Override
			public void remove() {
				throw new UnsupportedOperationException("removal not allowed");
			}
		};
	}

	public Vector2f totalImpulse() {
		Vector2f n = this.normal;
		Vector2f sum = cpvzero();
		for (int i = 0, count = getCount(); i < count; i++) {
			Contact con = contacts.get(i);
			sum = cpvadd(sum, cpvrotate(n, cpv(con.getJnAcc(), con.getJtAcc())));
		}
		return (this.swapped ? sum : cpvneg(sum));
	}

	public float totalKE() {
		float eCoef = (1 - this.e) / (1 + this.e);
		float sum = 0.0f;

		for (int i = 0, count = getCount(); i < count; i++) {
			Contact con = contacts.get(i);
			float jnAcc = con.getJnAcc();
			float jtAcc = con.getJtAcc();

			sum += eCoef * jnAcc * jnAcc / con.getnMass() + jtAcc * jtAcc / con.gettMass();
		}
		return sum;
	}

	public float getRestitution() {
		return this.e;
	}

	public void setRestitution(float restitution) {
		this.e = restitution;
	}

	/** @return The calculated friction for this collision pair. */
	public float getFriction() {
		return this.u;
	}

	/**
	 * The calculated friction for this collision pair. Setting the value in a {@link CollisionHandler#preSolve
	 * (Arbiter, Space)} callback will override the value calculated by the space.
	 *
	 * @param friction the calculated friction
	 */
	public void setFriction(float friction) {
		this.u = friction;
	}

	/** @return the calculated surface velocity for this collision pair. */
	public Vector2f getSurfaceVelocity() {
		return cpvmult(this.surface_vr, this.swapped ? -1.0f : 1.0f);
	}

	/**
	 * The calculated surface velocity for this collision pair. Setting the value in a {@link
	 * CollisionHandler#preSolve(Arbiter, Space)} callback will override the value calculated by the space.
	 *
	 * @param vr the calculated surface velocity.
	 */
	public void setSurfaceVelocity(Vector2f vr) {
		this.surface_vr = cpvmult(vr, this.swapped ? -1.0f : 1.0f);
	}

	public void update(CollisionInfo info, Space space) {
		Shape a = info.getA(), b = info.getB();

		// For collisions between two similar primitive types, the order could have been swapped since the last frame.
		this.a = a;
		this.body_a = a.body;
		this.b = b;
		this.body_b = b.body;

		if (info.getContacts() != null) {
			// Iterate over the possible pairs to look for hash value matches.
			for (Contact con : info.getContacts()) {
				// r1 and r2 store absolute offsets at init time.
				// Need to convert them to relative offsets.
				con.setR1(cpvsub(con.getR1(), a.body.p));
				con.setR2(cpvsub(con.getR2(), b.body.p));

				// Cached impulses are not zeroed at init time.
				con.setJnAcc(0.0f);
				con.setJtAcc(0.0f);

				if (this.contacts != null) {
					for (int j = 0; j < this.contacts.size(); j++) {
						Contact old = this.contacts.get(j);

						// This could trigger false positives, but is fairly unlikely nor serious if it does.
						if (con.getHash() == old.getHash()) {
							// Copy the persistant contact information.
							con.setJnAcc(old.getJnAcc());
							con.setJtAcc(old.getJtAcc());
						}
					}
				}
			}
		}

		this.contacts = info.getContacts();
		this.normal.set(info.getN());

		this.e = a.e * b.e;
		this.u = a.u * b.u;

		Vector2f surface_vr = cpvsub(b.surfaceV, a.surfaceV);
		this.surface_vr = cpvsub(surface_vr, cpvmult(info.getN(), cpvdot(surface_vr, info.getN())));

		CollisionType typeA = info.getA().getCollisionType(), typeB = info.getB().getCollisionType();
		CollisionHandler defaultHandler = space.getDefaultHandler();
		CollisionHandler handler = this.handler = space.lookupHandler(typeA, typeB, defaultHandler);

		// Check if the types match, but don't swap for a default handler which use the wildcard for type A.
		boolean swapped = this.swapped = (typeA != handler.typeA && handler.typeA != CollisionType.WILDCARD);

		if (handler != defaultHandler || space.isUseWildcards()) {
			// The order of the main handler swaps the wildcard handlers too. Uffda.
			this.handlerA = space.lookupHandler((swapped ? typeB : typeA), CollisionType.WILDCARD, DO_NOTHING);
			this.handlerB = space.lookupHandler((swapped ? typeA : typeB), CollisionType.WILDCARD, DO_NOTHING);
		}

		// mark it as new if it's been cached
		if (this.state == ArbiterState.CACHED) {
			this.state = ArbiterState.FIRST_COLLISION;
		}
	}

	void preStep(float dt, float slop, float bias) {
		if (this.contacts != null) {
			Body a = this.body_a;
			Body b = this.body_b;
			Vector2f n = this.normal;
			Vector2f body_delta = cpvsub(b.p, a.p);

			for (Contact con : this.contacts) {
				// Calculate the mass normal and mass tangent.
				con.setnMass(1.0f / k_scalar(a, b, con.getR1(), con.getR2(), n));
				con.settMass(1.0f / k_scalar(a, b, con.getR1(), con.getR2(), cpvperp(n)));

				// Calculate the target bias velocity.
				float dist = cpvdot(cpvadd(cpvsub(con.getR2(), con.getR1()), body_delta), n);
				con.setBias(-bias * cpfmin(0.0f, dist + slop) / dt);
				con.setjBias(0.0f);

				// Calculate the target bounce velocity.
				con.setBounce(normal_relative_velocity(a, b, con.getR1(), con.getR2(), n) * this.e);
			}
		}
	}

	void applyCachedImpulse(float dt_coef) {
		if (isFirstContact()) {
			return;
		}
		if (this.contacts != null) {
			Body a = this.body_a;
			Body b = this.body_b;
			Vector2f n = this.normal;

			for (Contact con : this.contacts) {
				Vector2f j = cpvrotate(n, cpv(con.getJnAcc(), con.getJtAcc()));
				apply_impulses(a, b, con.getR1(), con.getR2(), cpvmult(j, dt_coef));
			}
		}
	}

	// TODO is it worth splitting velocity/position correction?

	void applyImpulse() {
		if (this.contacts == null) {
			return;
		}
		Body a = this.body_a;
		Body b = this.body_b;
		Vector2f n = this.normal;
		Vector2f surface_vr = this.surface_vr;
		float friction = this.u;

		for (Contact con : this.contacts) {
			float nMass = con.getnMass();
			Vector2f r1 = con.getR1();
			Vector2f r2 = con.getR2();

			Vector2f vb1 = cpvadd(a.v_bias, cpvmult(cpvperp(r1), a.w_bias));
			Vector2f vb2 = cpvadd(b.v_bias, cpvmult(cpvperp(r2), b.w_bias));
			Vector2f vr = cpvadd(relative_velocity(a, b, r1, r2), surface_vr);

			float vbn = cpvdot(cpvsub(vb2, vb1), n);
			float vrn = cpvdot(vr, n);
			float vrt = cpvdot(vr, cpvperp(n));

			float jbn = (con.getBias() - vbn) * nMass;
			float jbnOld = con.getjBias();
			con.setjBias(cpfmax(jbnOld + jbn, 0.0f));

			float jn = -(con.getBounce() + vrn) * nMass;
			float jnOld = con.getJnAcc();
			con.setJnAcc(cpfmax(jnOld + jn, 0.0f));

			float jtMax = friction * con.getJnAcc();
			float jt = -vrt * con.gettMass();
			float jtOld = con.getJtAcc();
			con.setJtAcc(cpfclamp(jtOld + jt, -jtMax, jtMax));

			apply_bias_impulses(a, b, r1, r2, cpvmult(n, con.getjBias() - jbnOld));
			apply_impulses(a, b, r1, r2, cpvrotate(n, cpv(con.getJnAcc() - jnOld, con.getJtAcc() - jtOld)));
		}

	}

	static Arbiter arbiterNext(Arbiter node, Body body) {
		return (node.body_a == body ? node.thread_a.next : node.thread_b.next);
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

	boolean callWildcardBeginA(Space space) {
		CollisionHandler handler = this.handlerA;
		return handler.beginFunc.apply(this, space);
	}

	boolean callWildcardBeginB(Space space) {
		CollisionHandler handler = this.handlerB;
		this.swapped = !this.swapped;
		boolean retval = handler.beginFunc.apply(this, space);
		this.swapped = !this.swapped;
		return retval;
	}

	boolean callWildcardPreSolveA(Space space) {
		CollisionHandler handler = this.handlerA;
		return handler.preSolveFunc.apply(this, space);
	}

	boolean callWildcardPreSolveB(Space space) {
		CollisionHandler handler = this.handlerB;
		this.swapped = !this.swapped;
		boolean retval = handler.preSolveFunc.apply(this, space);
		this.swapped = !this.swapped;
		return retval;
	}

	void callWildcardPostSolveA(Space space) {
		CollisionHandler handler = this.handlerA;
		handler.postSolveFunc.apply(this, space);
	}

	void callWildcardPostSolveB(Space space) {
		CollisionHandler handler = this.handlerB;
		this.swapped = !this.swapped;
		handler.postSolveFunc.apply(this, space);
		this.swapped = !this.swapped;
	}

	void callWildcardSeparateA(Space space) {
		CollisionHandler handler = this.handlerA;
		handler.separateFunc.apply(this, space);
	}

	void callWildcardSeparateB(Space space) {
		CollisionHandler handler = this.handlerB;
		this.swapped = !this.swapped;
		handler.separateFunc.apply(this, space);
		this.swapped = !this.swapped;
	}
}
