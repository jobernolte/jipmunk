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

import static org.physics.jipmunk.Util.cpfabs;
import static org.physics.jipmunk.Util.cpfclamp;

/** @author jobernolte */
public class SimpleMotor extends Constraint {
	float rate;
	float iSum;
	float jAcc;

	public SimpleMotor(Body a, Body b, float rate) {
		super(a, b);

		this.rate = rate;

		this.jAcc = 0.0f;
	}

	public float getRate() {
		return rate;
	}

	public void setRate(float rate) {
		activateBodies();
		this.rate = rate;
	}

	@Override
	protected void preStep(float dt) {
		// calculate moment of inertia coefficient.
		this.iSum = 1.0f / (a.getInverseMoment() + b.getInverseMoment());
	}

	@Override
	protected void applyCachedImpulse(float dt_coef) {
		float j = this.jAcc * dt_coef;
		a.addAngularVelocity(-j * a.getInverseMoment());
		b.addAngularVelocity(j * b.getInverseMoment());
	}

	@Override
	protected void applyImpulse(float dt) {
		// compute relative rotational velocity
		float wr = b.getAngularVelocity() - a.getAngularVelocity() + this.rate;

		float jMax = this.maxForce * dt;

		// compute normal impulse
		float j = -wr * this.iSum;
		float jOld = this.jAcc;
		this.jAcc = cpfclamp(jOld + j, -jMax, jMax);
		j = this.jAcc - jOld;

		// apply impulse
		a.addAngularVelocity(-j * a.getInverseMoment());
		b.addAngularVelocity(j * b.getInverseMoment());
	}

	@Override
	protected float getImpulse() {
		return cpfabs(this.jAcc);
	}
}
