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
import static org.physics.jipmunk.Util.cpvdot;
import static org.physics.jipmunk.Util.cpvlength;
import static org.physics.jipmunk.Util.cpvmult;
import static org.physics.jipmunk.Util.cpvrotate;
import static org.physics.jipmunk.Util.cpvsub;
import static org.physics.jipmunk.Util.k_scalar;
import static org.physics.jipmunk.Util.relative_velocity;

/** @author jobernolte */
public class SlideJoint extends Constraint {

	Vector2f anchr1, anchr2;
	float min, max;

	Vector2f r1, r2;
	Vector2f n;
	float nMass;

	float jnAcc, jnMax;
	float bias;

	/**
	 * <code>a</code> and <code>b</code> are the two bodies to connect, <code>anchr1</code> and <code>anchr2</code> are the
	 * anchor points on those bodies, and <code>min</code> and <code>max</code> define the allowed distances of the anchor
	 * points.
	 *
	 * @param a      the first body to connect
	 * @param b      the second body to connect
	 * @param anchr1 the first anchor point
	 * @param anchr2 the second anchor point
	 * @param min    the minimum distance
	 * @param max    the maximum distance
	 */
	public SlideJoint(Body a, Body b, Vector2f anchr1, Vector2f anchr2, float min, float max) {
		super(a, b);

		this.anchr1 = Util.cpv(anchr1);
		this.anchr2 = Util.cpv(anchr2);
		this.min = min;
		this.max = max;

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

	public float getMin() {
		return min;
	}

	public void setMin(float min) {
		this.min = min;
	}

	public float getMax() {
		return max;
	}

	public void setMax(float max) {
		this.max = max;
	}

	@Override
	protected void preStep(float dt) {
		this.r1 = cpvrotate(this.anchr1, a.getRotation());
		this.r2 = cpvrotate(this.anchr2, b.getRotation());

		Vector2f delta = cpvsub(cpvadd(b.getPosition(), this.r2), cpvadd(a.getPosition(), this.r1));
		float dist = cpvlength(delta);
		float pdist = 0.0f;
		if (dist > this.max) {
			pdist = dist - this.max;
		} else if (dist < this.min) {
			pdist = this.min - dist;
			dist = -dist;
		}
		this.n = cpvmult(delta, 1.0f / (dist != 0 ? dist : (float) Float.POSITIVE_INFINITY));

		// calculate mass normal
		this.nMass = 1.0f / k_scalar(a, b, this.r1, this.r2, this.n);

		// calculate bias velocity
		//float maxBias = this.constraint.maxBias;
		this.bias = cpfclamp(-bias_coef(this.errorBias, dt) * pdist / dt, -this.maxBias, this.maxBias);

		// compute max impulse
		this.jnMax = J_MAX(this, dt);

		// if bias is 0, then the joint is not at a limit. Reset cached impulse.
		if (this.bias == 0) {
			this.jnAcc = 0.0f;
		}
	}

	@Override
	protected void applyCachedImpulse(float dt_coef) {
		Vector2f j = cpvmult(this.n, this.jnAcc * dt_coef);
		apply_impulses(a, b, this.r1, this.r2, j);
	}

	@Override
	protected void applyImpulse() {
		if (this.bias == 0) {
			return;  // early exit
		}

		//cpBody *a = this.constraint.a;
		//cpBody *b = this.constraint.b;

		Vector2f n = this.n;
		Vector2f r1 = this.r1;
		Vector2f r2 = this.r2;

		// compute relative velocity
		Vector2f vr = relative_velocity(a, b, r1, r2);
		float vrn = cpvdot(vr, n);

		// compute normal impulse
		float jn = (this.bias - vrn) * this.nMass;
		float jnOld = this.jnAcc;
		this.jnAcc = cpfclamp(jnOld + jn, -this.jnMax, 0.0f);
		jn = this.jnAcc - jnOld;

		// apply impulse
		apply_impulses(a, b, this.r1, this.r2, cpvmult(n, jn));
	}

	@Override
	protected float getImpulse() {
		return cpfabs(this.jnAcc);
	}
}
