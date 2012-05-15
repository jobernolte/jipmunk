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

import static org.physics.jipmunk.Util.apply_impulses;
import static org.physics.jipmunk.Util.bias_coef;
import static org.physics.jipmunk.Util.cpvadd;
import static org.physics.jipmunk.Util.cpvclamp;
import static org.physics.jipmunk.Util.cpvlength;
import static org.physics.jipmunk.Util.cpvmult;
import static org.physics.jipmunk.Util.cpvrotate;
import static org.physics.jipmunk.Util.cpvsub;
import static org.physics.jipmunk.Util.cpvzero;
import static org.physics.jipmunk.Util.k_tensor;
import static org.physics.jipmunk.Util.mult_k;
import static org.physics.jipmunk.Util.relative_velocity;

/** @author jobernolte */
public class PivotJoint extends Constraint {
	Vector2f anchr1, anchr2;

	Vector2f r1, r2;
	Vector2f k1 = Util.cpvzero();
	Vector2f k2 = Util.cpvzero();

	Vector2f jAcc;
	float jMaxLen;
	Vector2f bias;

	public PivotJoint(Body a, Body b, Vector2f anchr1, Vector2f anchr2) {
		super(a, b);
		init(a, b, anchr1, anchr2);
	}

	/**
	 * <code>a</code> and <code>b</code> are the two bodies to connect, and <code>pivot</code> is the point in world
	 * coordinates of the pivot. Because the pivot location is given in world coordinates, you must have the bodies moved
	 * into the correct positions already.
	 *
	 * @param a     the first body to connect
	 * @param b     the second body to connect
	 * @param pivot the pivot location
	 */
	public PivotJoint(Body a, Body b, Vector2f pivot) {
		super(a, b);
		Vector2f anchr1 = (a != null ? a.world2Local(pivot) : Util.cpv(pivot));
		Vector2f anchr2 = (b != null ? b.world2Local(pivot) : Util.cpv(pivot));
		init(a, b, anchr1, anchr2);
	}

	void init(Body a, Body b, Vector2f anchr1, Vector2f anchr2) {
		this.anchr1 = anchr1;
		this.anchr2 = anchr2;

		this.jAcc = cpvzero();
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

	@Override
	protected void preStep(float dt) {

		this.r1 = cpvrotate(this.anchr1, a.getRotation());
		this.r2 = cpvrotate(this.anchr2, b.getRotation());

		// Calculate mass tensor
		k_tensor(a, b, this.r1, this.r2, this.k1, this.k2);

		// compute max impulse
		this.jMaxLen = J_MAX(this, dt);

		// calculate bias velocity
		Vector2f delta = cpvsub(cpvadd(b.getPosition(), this.r2), cpvadd(a.getPosition(), this.r1));
		this.bias = cpvclamp(cpvmult(delta, -bias_coef(this.errorBias, dt) / dt), this.maxBias);
	}

	@Override
	protected void applyCachedImpulse(float dt_coef) {
		apply_impulses(a, b, this.r1, this.r2, cpvmult(this.jAcc, dt_coef));
	}

	@Override
	protected void applyImpulse() {
		Vector2f r1 = this.r1;
		Vector2f r2 = this.r2;

		// compute relative velocity
		Vector2f vr = relative_velocity(a, b, r1, r2);

		// compute normal impulse
		Vector2f j = mult_k(cpvsub(this.bias, vr), this.k1, this.k2);
		Vector2f jOld = this.jAcc;
		this.jAcc = cpvclamp(cpvadd(this.jAcc, j), this.jMaxLen);
		j = cpvsub(this.jAcc, jOld);

		// apply impulse
		apply_impulses(a, b, this.r1, this.r2, j);
	}

	@Override
	protected float getImpulse() {
		return cpvlength(this.jAcc);
	}
}
