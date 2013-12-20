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
	Space space;
	/** The first body connected to this constraint. */
	protected final Body a;
	/** The second body connected to this constraint. */
	protected final Body b;
	Constraint next_a;
	Constraint next_b;
	/** The maximum force that this constraint is allowed to use. Defaults to infinity. */
	protected float maxForce = Float.POSITIVE_INFINITY;
	/**
	 * The rate at which joint error is corrected. Defaults to pow(1.0 - 0.1, 60.0) meaning that it will correct 10% of
	 * the error every 1/60th of a second.
	 */
	protected float errorBias = Util.cpfpow(1.0f - 0.1f, 60.0f);
	/** The maximum rate at which joint error is corrected. Defaults to infinity. */
	protected float maxBias = Float.POSITIVE_INFINITY;
	boolean collideBodies;
	/** Function called before the solver runs. Animate your joint anchors, update your motor torque, etc. */
	ConstraintPreSolveFunc preSolveFunc;
	/** Function called after the solver runs. Use the applied impulse to perform effects like breakable joints. */
	ConstraintPostSolveFunc postSolveFunc;
	/**
	 * User definable data. Generally this points to your the game object class so you can access it when given a Body
	 * reference in a callback.
	 */
	private Object data;

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
		if (maxForce < 0.0f) {
			throw new IllegalArgumentException("max. force must be positive.");
		}
		this.maxForce = maxForce;
		activateBodies();
	}

	public float getErrorBias() {
		return errorBias;
	}

	public void setErrorBias(float errorBias) {
		if (errorBias < 0.0f) {
			throw new IllegalArgumentException("error bias must be positive.");
		}
		this.errorBias = errorBias;
		activateBodies();
	}

	public float getMaxBias() {
		return maxBias;
	}

	public void setMaxBias(float maxBias) {
		if (maxBias < 0.0f) {
			throw new IllegalArgumentException("max. bias must be positive.");
		}
		this.maxBias = maxBias;
		activateBodies();
	}

	public boolean isCollideBodies() {
		return collideBodies;
	}

	public void setCollideBodies(boolean collideBodies) {
		this.collideBodies = collideBodies;
		activateBodies();
	}

	protected abstract void preStep(float dt);

	protected abstract void applyCachedImpulse(float dt_coef);

	protected abstract void applyImpulse(float dt);

	protected abstract float getImpulse();

	protected void activateBodies() {
		if (a != null) {
			a.activate();
		}
		if (b != null) {
			b.activate();
		}
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

	/** @return the user data */
	public Object getData() {
		return data;
	}

	/**
	 * Sets user data. Use this data to get a reference to the game object that owns this body from callbacks.
	 *
	 * @param data the user data to set
	 */
	public void setData(Object data) {
		this.data = data;
	}

	/**
	 * @param clazz the {@link Class} of the user data
	 * @param <T>   the type of the data
	 * @return the user data
	 */
	public <T> T getData(Class<T> clazz) {
		return clazz.cast(data);
	}
}
