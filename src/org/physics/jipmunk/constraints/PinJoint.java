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

package org.physics.jipmunk.constraints;

import org.physics.jipmunk.Body;
import org.physics.jipmunk.Constraint;
import org.physics.jipmunk.Util;
import org.physics.jipmunk.Vector2f;

import static org.physics.jipmunk.Util.*;

/** @author jobernolte */
public class PinJoint extends Constraint {
	Vector2f anchorA, anchorB;
	float dist;
	Vector2f r1, r2;
	Vector2f n;
	float nMass;
	float jnAcc;
	float bias;

	/**
	 * <code>a</code> and <code>b</code> are the two bodies to connect, and <code>anchorA</code> and
	 * <code>anchorB</code> are the anchor points on those bodies. The distance between the two anchor points is
	 * measured when the joint is created. If you want to set a specific distance, use the setter function to override
	 * it.
	 *
	 * @param a       the first body
	 * @param b       the second body
	 * @param anchorA the first anchor point
	 * @param anchorB the second anchor point
	 */
	public PinJoint(Body a, Body b, Vector2f anchorA, Vector2f anchorB) {
		super(a, b);

		this.anchorA = Util.cpv(anchorA);
		this.anchorB = Util.cpv(anchorB);

		// STATIC_BODY_CHECK
		Vector2f p1 = (a != null ? a.getTransform().transformPoint(anchorA) : anchorA);
		Vector2f p2 = (b != null ? b.getTransform().transformPoint(anchorB) : anchorB);
		this.dist = cpvlength(cpvsub(p2, p1));

		this.jnAcc = 0.0f;
	}

	public Vector2f getAnchorA() {
		return anchorA;
	}

	public void setAnchorA(Vector2f anchorA) {
		activateBodies();
		this.anchorA.set(anchorA);
	}

	public Vector2f getAnchorB() {
		return anchorB;
	}

	public void setAnchorB(Vector2f anchorB) {
		activateBodies();
		this.anchorB.set(anchorB);
	}

	public float getDist() {
		return dist;
	}

	public void setDist(float dist) {
		activateBodies();
		this.dist = dist;
	}

	@Override
	protected void preStep(float dt) {
		Body a = this.a;
		Body b = this.b;

		this.r1 = a.getTransform().transformVect(cpvsub(this.anchorA, a.getCenterOfGravity()));
		this.r2 = b.getTransform().transformVect(cpvsub(this.anchorB, b.getCenterOfGravity()));

		Vector2f delta = cpvsub(cpvadd(b.getPosition(), this.r2), cpvadd(a.getPosition(), this.r1));
		float dist = cpvlength(delta);
		this.n = cpvmult(delta, 1.0f / (dist != 0 ? dist : Float.POSITIVE_INFINITY));

		// calculate mass normal
		this.nMass = 1.0f / k_scalar(a, b, this.r1, this.r2, this.n);

		// calculate bias velocity
		float maxBias = this.maxBias;
		this.bias = cpfclamp(-bias_coef(this.errorBias, dt) * (dist - this.dist) / dt, -maxBias, maxBias);
	}

	@Override
	protected void applyCachedImpulse(float dt_coef) {
		Body a = this.a;
		Body b = this.b;

		Vector2f j = cpvmult(this.n, this.jnAcc * dt_coef);
		Body.applyImpulses(a, b, this.r1, this.r2, j);
	}

	@Override
	protected void applyImpulse(float dt) {
		Body a = this.a;
		Body b = this.b;
		Vector2f n = this.n;

		// compute relative velocity
		float vrn = normal_relative_velocity(a, b, this.r1, this.r2, n);

		float jnMax = this.maxForce * dt;

		// compute normal impulse
		float jn = (this.bias - vrn) * this.nMass;
		float jnOld = this.jnAcc;
		this.jnAcc = cpfclamp(jnOld + jn, -jnMax, jnMax);
		jn = this.jnAcc - jnOld;

		// apply impulse
		Body.applyImpulses(a, b, this.r1, this.r2, cpvmult(n, jn));
	}

	@Override
	protected float getImpulse() {
		return cpfabs(this.jnAcc);
	}

}
