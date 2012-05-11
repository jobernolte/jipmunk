package org.physics.jipmunk.constraints;

import org.physics.jipmunk.Body;
import org.physics.jipmunk.Constraint;
import org.physics.jipmunk.Util;
import org.physics.jipmunk.Vector2f;

import static org.physics.jipmunk.Util.apply_impulses;
import static org.physics.jipmunk.Util.bias_coef;
import static org.physics.jipmunk.Util.cpfabs;
import static org.physics.jipmunk.Util.cpfclamp;
import static org.physics.jipmunk.Util.cpvadd;
import static org.physics.jipmunk.Util.cpvlength;
import static org.physics.jipmunk.Util.cpvmult;
import static org.physics.jipmunk.Util.cpvrotate;
import static org.physics.jipmunk.Util.cpvsub;
import static org.physics.jipmunk.Util.k_scalar;
import static org.physics.jipmunk.Util.normal_relative_velocity;

/** @author jobernolte */
public class PinJoint extends Constraint {
	Vector2f anchr1, anchr2;
	float dist;

	Vector2f r1, r2;
	Vector2f n;
	float nMass;

	float jnAcc, jnMax;
	float bias;

	/**
	 * <code>a</code> and <code>b</code> are the two bodies to connect, and <code>anchr1</code> and <code>anchr2</code> are
	 * the anchor points on those bodies. The distance between the two anchor points is measured when the joint is created.
	 * If you want to set a specific distance, use the setter function to override it.
	 *
	 * @param a      the first body
	 * @param b      the second body
	 * @param anchr1 the first anchor point
	 * @param anchr2 the second anchor point
	 */
	public PinJoint(Body a, Body b, Vector2f anchr1, Vector2f anchr2) {
		super(a, b);

		this.anchr1 = Util.cpv(anchr1);
		this.anchr2 = Util.cpv(anchr2);

		// STATIC_BODY_CHECK
		Vector2f p1 = (a != null ? cpvadd(a.getPosition(), cpvrotate(anchr1, a.getRotation())) : anchr1);
		Vector2f p2 = (b != null ? cpvadd(b.getPosition(), cpvrotate(anchr2, b.getRotation())) : anchr2);
		this.dist = cpvlength(cpvsub(p2, p1));

		this.jnAcc = 0.0f;
	}

	public Vector2f getAnchr1() {
		return anchr1;
	}

	public void setAnchr1(Vector2f anchr1) {
		this.anchr1.set(anchr1);
	}

	public Vector2f getAnchr2() {
		return anchr2;
	}

	public void setAnchr2(Vector2f anchr2) {
		this.anchr2.set(anchr2);
	}

	public float getDist() {
		return dist;
	}

	public void setDist(float dist) {
		this.dist = dist;
	}

	@Override
	protected void preStep(float dt) {
		preStep(this, dt);
	}

	@Override
	protected void applyCachedImpulse(float dt_coef) {
		applyCachedImpulse(this, dt_coef);
	}

	@Override
	protected void applyImpulse() {
		applyImpulse(this);
	}

	@Override
	protected float getImpulse() {
		return getImpulse(this);
	}

	/*
		*
		*  cpPinJoint.c
		*
		*/

	static void preStep(PinJoint joint, float dt) {
		Body a = joint.a;
		Body b = joint.b;

		joint.r1 = cpvrotate(joint.anchr1, a.getRotation());
		joint.r2 = cpvrotate(joint.anchr2, b.getRotation());

		Vector2f delta = cpvsub(cpvadd(b.getPosition(), joint.r2), cpvadd(a.getPosition(), joint.r1));
		float dist = cpvlength(delta);
		joint.n = cpvmult(delta, 1.0f / (dist != 0 ? dist : (float) Float.POSITIVE_INFINITY));

		// calculate mass normal
		joint.nMass = 1.0f / k_scalar(a, b, joint.r1, joint.r2, joint.n);

		// calculate bias velocity
		float maxBias = joint.maxBias;
		joint.bias = cpfclamp(-bias_coef(joint.errorBias, dt) * (dist - joint.dist) / dt, -maxBias, maxBias);

		// compute max impulse
		joint.jnMax = J_MAX(joint, dt);
	}

	static void applyCachedImpulse(PinJoint joint, float dt_coef) {
		Body a = joint.a;
		Body b = joint.b;

		Vector2f j = cpvmult(joint.n, joint.jnAcc * dt_coef);
		apply_impulses(a, b, joint.r1, joint.r2, j);
	}

	static void applyImpulse(PinJoint joint) {
		Body a = joint.a;
		Body b = joint.b;
		Vector2f n = joint.n;

		// compute relative velocity
		float vrn = normal_relative_velocity(a, b, joint.r1, joint.r2, n);

		// compute normal impulse
		float jn = (joint.bias - vrn) * joint.nMass;
		float jnOld = joint.jnAcc;
		joint.jnAcc = cpfclamp(jnOld + jn, -joint.jnMax, joint.jnMax);
		jn = joint.jnAcc - jnOld;

		// apply impulse
		apply_impulses(a, b, joint.r1, joint.r2, cpvmult(n, jn));
	}

	static float getImpulse(PinJoint joint) {
		return cpfabs(joint.jnAcc);
	}

}
