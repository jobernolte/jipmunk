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
public class SlideJoint extends Constraint {

	Vector2f anchorA, anchorB;
	float min, max;
	Vector2f r1, r2;
	Vector2f n;
	float nMass;
	float jnAcc;
	float bias;

	/**
	 * <code>a</code> and <code>b</code> are the two bodies to connect, <code>anchorA</code> and <code>anchorB</code>
	 * are the anchor points on those bodies, and <code>min</code> and <code>max</code> define the allowed distances of
	 * the anchor points.
	 *
	 * @param a       the first body to connect
	 * @param b       the second body to connect
	 * @param anchorA the first anchor point
	 * @param anchorB the second anchor point
	 * @param min     the minimum distance
	 * @param max     the maximum distance
	 */
	public SlideJoint(Body a, Body b, Vector2f anchorA, Vector2f anchorB, float min, float max) {
		super(a, b);

		this.anchorA = Util.cpv(anchorA);
		this.anchorB = Util.cpv(anchorB);
		this.min = min;
		this.max = max;

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

	public float getMin() {
		return min;
	}

	public void setMin(float min) {
		activateBodies();
		this.min = min;
	}

	public float getMax() {
		return max;
	}

	public void setMax(float max) {
		activateBodies();
		this.max = max;
	}

	@Override
	protected void preStep(float dt) {
		this.r1 = a.getTransform().transformVect(cpvsub(this.anchorA, a.getCenterOfGravity()));
		this.r2 = b.getTransform().transformVect(cpvsub(this.anchorB, b.getCenterOfGravity()));

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

		// if bias is 0, then the joint is not at a limit. Reset cached impulse.
		if (this.bias == 0) {
			this.jnAcc = 0.0f;
		}
	}

	@Override
	protected void applyCachedImpulse(float dt_coef) {
		Vector2f j = cpvmult(this.n, this.jnAcc * dt_coef);
		Body.applyImpulses(a, b, this.r1, this.r2, j);
	}

	@Override
	protected void applyImpulse(float dt) {
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
		this.jnAcc = cpfclamp(jnOld + jn, -this.maxForce * dt, 0.0f);
		jn = this.jnAcc - jnOld;

		// apply impulse
		Body.applyImpulses(a, b, this.r1, this.r2, cpvmult(n, jn));
	}

	@Override
	protected float getImpulse() {
		return cpfabs(this.jnAcc);
	}
}
