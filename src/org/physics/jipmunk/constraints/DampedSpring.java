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
import static org.physics.jipmunk.Util.cpfexp;
import static org.physics.jipmunk.Util.cpvadd;
import static org.physics.jipmunk.Util.cpvlength;
import static org.physics.jipmunk.Util.cpvmult;
import static org.physics.jipmunk.Util.cpvrotate;
import static org.physics.jipmunk.Util.cpvsub;
import static org.physics.jipmunk.Util.k_scalar;
import static org.physics.jipmunk.Util.normal_relative_velocity;

/**
 * Defined much like a slide joint. {@link org.physics.jipmunk.constraints.DampedSpring#getRestLength()} is the distance
 * the spring wants to be, {@link org.physics.jipmunk.constraints.DampedSpring#getStiffness()} is the spring constant
 * (Young?s modulus), and {@link org.physics.jipmunk.constraints.DampedSpring#getDamping()} is how soft to make the
 * damping of the spring.
 *
 * @author jobernolte
 */
public class DampedSpring extends Constraint {

	private static final DampedSpringForceFunc defaultSpringForce = new DampedSpringForceFunc() {
		@Override
		public float apply(DampedSpring spring, float dist) {
			return (spring.restLength - dist) * spring.stiffness;
		}
	};
	private Vector2f anchorA, anchorB;
	private float restLength;
	private float stiffness;
	private float damping;
	private DampedSpringForceFunc springForceFunc;
	private float target_vrn;
	private float v_coef;
	private Vector2f r1, r2;
	private float nMass;
	private Vector2f n;
	private float jAcc;

	public DampedSpring(Body a, Body b, Vector2f anchorA, Vector2f anchr2, float restLength, float stiffness,
			float damping) {
		super(a, b);

		this.anchorA = Util.cpv(anchorA);
		this.anchorB = Util.cpv(anchr2);

		this.restLength = restLength;
		this.stiffness = stiffness;
		this.damping = damping;
		this.springForceFunc = defaultSpringForce;
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

	public float getRestLength() {
		return restLength;
	}

	public void setRestLength(float restLength) {
		activateBodies();
		this.restLength = restLength;
	}

	public float getStiffness() {
		return stiffness;
	}

	public void setStiffness(float stiffness) {
		activateBodies();
		this.stiffness = stiffness;
	}

	public float getDamping() {
		return damping;
	}

	public void setDamping(float damping) {
		activateBodies();
		this.damping = damping;
	}

	public DampedSpringForceFunc getSpringForceFunc() {
		return springForceFunc;
	}

	public void setSpringForceFunc(DampedSpringForceFunc springForceFunc) {
		this.springForceFunc = springForceFunc;
	}

	@Override
	protected void preStep(float dt) {
		this.r1 = a.getTransform().transformVect(cpvsub(this.anchorA, a.getCenterOfGravity()));
		this.r2 = b.getTransform().transformVect(cpvsub(this.anchorB, b.getCenterOfGravity()));

		Vector2f delta = cpvsub(cpvadd(b.getPosition(), this.r2), cpvadd(a.getPosition(), this.r1));
		float dist = cpvlength(delta);
		this.n = cpvmult(delta, 1.0f / (dist != 0.0f ? dist : Float.POSITIVE_INFINITY));

		float k = k_scalar(a, b, this.r1, this.r2, this.n);
		if (k == 0.0f) {
			throw new IllegalStateException("Unsolvable spring.");
		}
		this.nMass = 1.0f / k;

		this.target_vrn = 0.0f;
		this.v_coef = 1.0f - cpfexp(-this.damping * dt * k);

		// apply spring force
		float f_spring = this.springForceFunc.apply(this, dist);
		float j_spring = this.jAcc = f_spring * dt;
		Body.applyImpulses(a, b, this.r1, this.r2, cpvmult(this.n, j_spring));
	}

	@Override
	protected void applyCachedImpulse(float dt_coef) {
	}

	@Override
	protected void applyImpulse(float dt) {
		Vector2f n = this.n;
		Vector2f r1 = this.r1;
		Vector2f r2 = this.r2;

		// compute relative velocity
		float vrn = normal_relative_velocity(a, b, r1, r2, n);

		// compute velocity loss from drag
		float v_damp = (this.target_vrn - vrn) * this.v_coef;
		this.target_vrn = vrn + v_damp;

		float j_damp = v_damp * this.nMass;
		this.jAcc += j_damp;
		Body.applyImpulses(a, b, this.r1, this.r2, cpvmult(this.n, j_damp));
	}

	@Override
	protected float getImpulse() {
		return jAcc;
	}
}
