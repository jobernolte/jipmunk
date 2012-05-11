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
	float maxForce;
	/**
	 * The rate at which joint error is corrected. Defaults to pow(1.0 - 0.1, 60.0) meaning that it will correct 10% of the
	 * error every 1/60th of a second.
	 */
	protected float errorBias;
	/** The maximum rate at which joint error is corrected. Defaults to infinity. */
	protected float maxBias;
	ConstraintPreSolveFunc preSolveFunc;
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

	protected abstract void applyImpulse();

	protected abstract float getImpulse();

	/// @private
	static void cpConstraintActivateBodies(Constraint constraint) {
		Body a = constraint.a;
		if (a != null) cpBodyActivate(a);
		Body b = constraint.b;
		if (b != null) cpBodyActivate(b);
	}

	protected static float J_MAX(Constraint constraint, float dt) {
		return constraint.maxForce * dt;
	}

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
