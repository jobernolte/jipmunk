package org.physics.jipmunk.constraints;

import org.physics.jipmunk.Body;
import org.physics.jipmunk.Constraint;

import static org.physics.jipmunk.Util.cpfabs;
import static org.physics.jipmunk.Util.cpfclamp;

/** @author jobernolte */
public class SimpleMotor extends Constraint {
	float rate;

	float iSum;

	float jAcc, jMax;

	public SimpleMotor(Body a, Body b, float rate) {
		super(a, b);

		this.rate = rate;

		this.jAcc = 0.0f;
	}

	public float getRate() {
		return rate;
	}

	public void setRate(float rate) {
		this.rate = rate;
	}

	@Override
	protected void preStep(float dt) {
		// calculate moment of inertia coefficient.
		this.iSum = 1.0f / (a.getInverseMoment() + b.getInverseMoment());

		// compute max impulse
		this.jMax = J_MAX(this, dt);
	}

	@Override
	protected void applyCachedImpulse(float dt_coef) {
		float j = this.jAcc * dt_coef;
		a.addAngVel(-j * a.getInverseMoment());
		b.addAngVel(j * b.getInverseMoment());
	}

	@Override
	protected void applyImpulse() {
		// compute relative rotational velocity
		float wr = b.getAngVel() - a.getAngVel() + this.rate;

		// compute normal impulse
		float j = -wr * this.iSum;
		float jOld = this.jAcc;
		this.jAcc = cpfclamp(jOld + j, -this.jMax, this.jMax);
		j = this.jAcc - jOld;

		// apply impulse
		a.addAngVel(-j * a.getInverseMoment());
		b.addAngVel(j * b.getInverseMoment());
	}

	@Override
	protected float getImpulse() {
		return cpfabs(this.jAcc);
	}
}
