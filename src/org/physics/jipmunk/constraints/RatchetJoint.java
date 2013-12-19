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
public class RatchetJoint extends Constraint {
	float angle, phase, ratchet;

	float iSum;

	float bias;
	float jAcc;

	public RatchetJoint(Body a, Body b, float phase, float ratchet) {
		super(a, b);

		this.angle = 0.0f;
		this.phase = phase;
		this.ratchet = ratchet;

		// STATIC_BODY_CHECK
		this.angle = (b != null ? b.getAngleInRadians() : 0.0f) - (a != null ? a.getAngleInRadians() : 0.0f);
	}

	public float getAngle() {
		return angle;
	}

	public void setAngle(float angle) {
		this.angle = angle;
	}

	public float getPhase() {
		return phase;
	}

	public void setPhase(float phase) {
		this.phase = phase;
	}

	public float getRatchet() {
		return ratchet;
	}

	public void setRatchet(float ratchet) {
		this.ratchet = ratchet;
	}

	@Override
	protected void preStep(float dt) {
		float angle = this.angle;
		float phase = this.phase;
		float ratchet = this.ratchet;

		float delta = b.getAngleInRadians() - a.getAngleInRadians();
		float diff = angle - delta;
		float pdist = 0.0f;

		if (diff * ratchet > 0.0f) {
			pdist = diff;
		} else {
			this.angle = (float) (Math.floor((delta - phase) / ratchet) * ratchet + phase);
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
		a.setAngularVelocity(a.getAngularVelocity() - j * a.getInverseMoment());
		b.setAngularVelocity(b.getAngularVelocity() + j * b.getInverseMoment());
	}

	@Override
	protected void applyImpulse(float dt) {
		if (this.bias == 0) {
			return; // early exit
		}

		// compute relative rotational velocity
		float wr = b.getAngularVelocity() - a.getAngularVelocity();
		float ratchet = this.ratchet;

		float jMax = this.maxForce * dt;

		// compute normal impulse
		float j = -(this.bias + wr) * this.iSum;
		float jOld = this.jAcc;
		this.jAcc = cpfclamp((jOld + j) * ratchet, 0.0f, jMax * cpfabs(ratchet)) / ratchet;
		j = this.jAcc - jOld;

		// apply impulse
		a.setAngularVelocity(a.getAngularVelocity() - j * a.getInverseMoment());
		b.setAngularVelocity(b.getAngularVelocity() + j * b.getInverseMoment());
	}

	@Override
	protected float getImpulse() {
		return cpfabs(this.jAcc);
	}
}
