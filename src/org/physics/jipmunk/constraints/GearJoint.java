package org.physics.jipmunk.constraints;

import org.physics.jipmunk.Body;
import org.physics.jipmunk.Constraint;

import static org.physics.jipmunk.Util.bias_coef;
import static org.physics.jipmunk.Util.cpfabs;
import static org.physics.jipmunk.Util.cpfclamp;

/** @author jobernolte */
public class GearJoint extends Constraint {
	float phase, ratio;
	float ratio_inv;

	float iSum;

	float bias;
	float jAcc, jMax;

	public GearJoint(Body a, Body b, float phase, float ratio) {
		super(a, b);

		this.phase = phase;
		this.ratio = ratio;
		this.ratio_inv = 1.0f / ratio;

		this.jAcc = 0.0f;
	}

	public float getPhase() {
		return phase;
	}

	public void setPhase(float phase) {
		this.phase = phase;
	}

	public float getRatio() {
		return ratio;
	}

	public void setRatio(float ratio) {
		this.ratio = ratio;
	}

	@Override
	protected void preStep(float dt) {
		// calculate moment of inertia coefficient.
		this.iSum = 1.0f / (a.getInverseMoment() * this.ratio_inv + this.ratio * b.getInverseMoment());

		// calculate bias velocity
		float maxBias = this.maxBias;
		this.bias = cpfclamp(-bias_coef(this.errorBias, dt) * (b.getAngleInRadians() * this.ratio - a
				.getAngleInRadians() - this.phase) / dt, -maxBias, maxBias);

		// compute max impulse
		this.jMax = J_MAX(this, dt);
	}

	@Override
	protected void applyCachedImpulse(float dt_coef) {
		float j = this.jAcc * dt_coef;
		a.addAngVel(-j * a.getInverseMoment() * this.ratio_inv);
		b.addAngVel(j * b.getInverseMoment());
	}

	@Override
	protected void applyImpulse() {
		// compute relative rotational velocity
		float wr = b.getAngVel() * this.ratio - a.getAngVel();

		// compute normal impulse
		float j = (this.bias - wr) * this.iSum;
		float jOld = this.jAcc;
		this.jAcc = cpfclamp(jOld + j, -this.jMax, this.jMax);
		j = this.jAcc - jOld;

		// apply impulse
		a.addAngVel(-j * a.getInverseMoment() * this.ratio_inv);
		b.addAngVel(j * b.getInverseMoment());
	}

	@Override
	protected float getImpulse() {
		return cpfabs(this.jAcc);
	}
}
