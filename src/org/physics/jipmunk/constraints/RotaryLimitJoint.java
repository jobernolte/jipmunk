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

import static org.physics.jipmunk.Util.bias_coef;
import static org.physics.jipmunk.Util.cpfabs;
import static org.physics.jipmunk.Util.cpfclamp;

/** @author jobernolte */
public class RotaryLimitJoint extends Constraint {
	float min, max;

	float iSum;

	float bias;
	float jAcc;

	public RotaryLimitJoint(Body a, Body b, float min, float max) {
		super(a, b);

		this.min = min;
		this.max = max;

		this.jAcc = 0.0f;
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
		float dist = b.getAngleInRadians() - a.getAngleInRadians();
		float pdist = 0.0f;
		if (dist > this.max) {
			pdist = this.max - dist;
		} else if (dist < this.min) {
			pdist = this.min - dist;
		}

		// calculate moment of inertia coefficient.
		this.iSum = 1.0f / (a.getInverseMoment() + b.getInverseMoment());

		// calculate bias velocity
		float maxBias = this.maxBias;
		this.bias = cpfclamp(-bias_coef(this.errorBias, dt) * pdist / dt, -maxBias, maxBias);

		// If the bias is 0, the joint is not at a limit. Reset the impulse.
		if (this.bias == 0) {
			this.jAcc = 0.0f;
		}
	}

	@Override
	protected void applyCachedImpulse(float dt_coef) {
		float j = this.jAcc * dt_coef;
		a.setAngVel(a.getAngVel() - j * a.getInverseMoment());
		b.setAngVel(b.getAngVel() + j * b.getInverseMoment());
	}

	@Override
	protected void applyImpulse(float dt) {
		if (this.bias == 0) {
			return; // early exit
		}

		// compute relative rotational velocity
		float wr = b.getAngVel() - a.getAngVel();

		float jMax = this.maxForce * dt;

		// compute normal impulse
		float j = -(this.bias + wr) * this.iSum;
		float jOld = this.jAcc;
		if (this.bias < 0.0f) {
			this.jAcc = cpfclamp(jOld + j, 0.0f, jMax);
		} else {
			this.jAcc = cpfclamp(jOld + j, -jMax, 0.0f);
		}
		j = this.jAcc - jOld;

		// apply impulse
		a.setAngVel(a.getAngVel() - j * a.getInverseMoment());
		b.setAngVel(b.getAngVel() + j * b.getInverseMoment());
	}

	@Override
	protected float getImpulse() {
		return cpfabs(this.jAcc);
	}
}
