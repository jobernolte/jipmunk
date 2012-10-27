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

package org.physics.jipmunk;

import static org.physics.jipmunk.SpaceComponent.cpBodyActivate;

/** @author jobernolte */
public abstract class Constraint {
	/** The first body connected to this constraint. */
	protected final Body a;
	/** The second body connected to this constraint. */
	protected final Body b;

	Space space;

	Constraint next_a;
	Constraint next_b;

	/** The maximum force that this constraint is allowed to use. Defaults to infinity. */
	protected float maxForce;
	/**
	 * The rate at which joint error is corrected. Defaults to pow(1.0 - 0.1, 60.0) meaning that it will correct 10% of the
	 * error every 1/60th of a second.
	 */
	protected float errorBias;
	/** The maximum rate at which joint error is corrected. Defaults to infinity. */
	protected float maxBias;
	/** Function called before the solver runs. Animate your joint anchors, update your motor torque, etc. */
	ConstraintPreSolveFunc preSolveFunc;
	/** Function called after the solver runs. Use the applied impulse to perform effects like breakable joints. */
	ConstraintPostSolveFunc postSolveFunc;

	protected Constraint(Body a, Body b) {
		this.a = a;
		this.b = b;
	}

	public Body getBodyA() {
		return a;
	}

	public Body getBodyB() {
		return b;
	}

	public float getMaxForce() {
		return maxForce;
	}

	public void setMaxForce(float maxForce) {
		this.maxForce = maxForce;
	}

	public float getErrorBias() {
		return errorBias;
	}

	public void setErrorBias(float errorBias) {
		this.errorBias = errorBias;
	}

	public float getMaxBias() {
		return maxBias;
	}

	public void setMaxBias(float maxBias) {
		this.maxBias = maxBias;
	}

	protected abstract void preStep(float dt);

	protected abstract void applyCachedImpulse(float dt_coef);

	protected abstract void applyImpulse(float dt);

	protected abstract float getImpulse();

	protected void activateBodies() {
		if (a != null) cpBodyActivate(a);
		if (b != null) cpBodyActivate(b);
	}

	public ConstraintPreSolveFunc getPreSolveFunc() {
		return preSolveFunc;
	}

	public void setPreSolveFunc(ConstraintPreSolveFunc preSolveFunc) {
		this.preSolveFunc = preSolveFunc;
	}

	public ConstraintPostSolveFunc getPostSolveFunc() {
		return postSolveFunc;
	}

	public void setPostSolveFunc(ConstraintPostSolveFunc postSolveFunc) {
		this.postSolveFunc = postSolveFunc;
	}
}
