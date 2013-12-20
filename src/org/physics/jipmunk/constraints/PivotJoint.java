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

import org.physics.jipmunk.*;

import static org.physics.jipmunk.Util.*;

/** @author jobernolte */
public class PivotJoint extends Constraint {
	Vector2f anchorA, anchorB;
	Vector2f r1, r2;
	Mat2x2 k;
	Vector2f jAcc;
	Vector2f bias;

	public PivotJoint(Body a, Body b, Vector2f anchorA, Vector2f anchorB) {
		super(a, b);
		init(a, b, anchorA, anchorB);
	}

	/**
	 * <code>a</code> and <code>b</code> are the two bodies to connect, and <code>pivot</code> is the point in world
	 * coordinates of the pivot. Because the pivot location is given in world coordinates, you must have the bodies
	 * moved into the correct positions already.
	 *
	 * @param a     the first body to connect
	 * @param b     the second body to connect
	 * @param pivot the pivot location
	 */
	public PivotJoint(Body a, Body b, Vector2f pivot) {
		super(a, b);
		Vector2f anchr1 = (a != null ? a.worldToLocal(pivot) : Util.cpv(pivot));
		Vector2f anchr2 = (b != null ? b.worldToLocal(pivot) : Util.cpv(pivot));
		init(a, b, anchr1, anchr2);
	}

	void init(Body a, Body b, Vector2f anchr1, Vector2f anchr2) {
		this.anchorA = anchr1;
		this.anchorB = anchr2;

		this.jAcc = cpvzero();
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

	@Override
	protected void preStep(float dt) {

		this.r1 = a.getTransform().transformVect(cpvsub(this.anchorA, a.getCenterOfGravity()));
		this.r2 = b.getTransform().transformVect(cpvsub(this.anchorB, b.getCenterOfGravity()));

		// Calculate mass tensor
		this.k = Mat2x2.k_tensor(a, b, this.r1, this.r2);

		// calculate bias velocity
		Vector2f delta = cpvsub(cpvadd(b.getPosition(), this.r2), cpvadd(a.getPosition(), this.r1));
		this.bias = cpvclamp(cpvmult(delta, -bias_coef(this.errorBias, dt) / dt), this.maxBias);
	}

	@Override
	protected void applyCachedImpulse(float dt_coef) {
		Body.applyImpulses(a, b, this.r1, this.r2, cpvmult(this.jAcc, dt_coef));
	}

	@Override
	protected void applyImpulse(float dt) {
		Vector2f r1 = this.r1;
		Vector2f r2 = this.r2;

		// compute relative velocity
		Vector2f vr = relative_velocity(a, b, r1, r2);

		// compute normal impulse
		Vector2f j = k.transform(cpvsub(this.bias, vr));
		Vector2f jOld = this.jAcc;
		this.jAcc = cpvclamp(cpvadd(this.jAcc, j), this.maxForce * dt);
		j = cpvsub(this.jAcc, jOld);

		// apply impulse
		Body.applyImpulses(a, b, this.r1, this.r2, j);
	}

	@Override
	protected float getImpulse() {
		return cpvlength(this.jAcc);
	}
}
