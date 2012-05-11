package org.physics.jipmunk.constraints;

import org.physics.jipmunk.Body;
import org.physics.jipmunk.Constraint;

import static org.physics.jipmunk.Util.cpfexp;

/**
 * Like a damped spring, but works in an angular fashion. {@link org.physics.jipmunk.constraints.DampedRotarySpring#getRestAngle()}
 * is the relative angle in radians that the bodies want to have,  {@link DampedRotarySpring#getStiffness()}  and {@link
 * DampedRotarySpring#getDamping()}  work basically the same as on a damped spring.
 *
 * @author jobernolte
 */
public class DampedRotarySpring extends Constraint {

	private final static DampedRotarySpringTorqueFunc defaultSpringTorque = new DampedRotarySpringTorqueFunc() {
		@Override
		public float apply(DampedRotarySpring spring, float relativeAngleInRadians) {
			return (relativeAngleInRadians - spring.restAngle) * spring.stiffness;
		}
	};

	private float restAngle;
	private float stiffness;
	private float damping;
	private DampedRotarySpringTorqueFunc springTorqueFunc;

	private float target_wrn;
	private float w_coef;

	private float iSum;

	public DampedRotarySpring(Body a, Body b, float restAngle, float stiffness, float damping) {
		super(a, b);

		this.restAngle = restAngle;
		this.stiffness = stiffness;
		this.damping = damping;
		this.springTorqueFunc = defaultSpringTorque;
	}

	public float getRestAngle() {
		return restAngle;
	}

	public void setRestAngle(float restAngle) {
		this.restAngle = restAngle;
	}

	public float getStiffness() {
		return stiffness;
	}

	public void setStiffness(float stiffness) {
		this.stiffness = stiffness;
	}

	public float getDamping() {
		return damping;
	}

	public void setDamping(float damping) {
		this.damping = damping;
	}

	public DampedRotarySpringTorqueFunc getSpringTorqueFunc() {
		return springTorqueFunc;
	}

	public void setSpringTorqueFunc(DampedRotarySpringTorqueFunc springTorqueFunc) {
		this.springTorqueFunc = springTorqueFunc;
	}

	@Override
	protected void preStep(float dt) {
		float moment = a.getInverseMoment() + b.getInverseMoment();
		assert moment != 0.0 : "Unsolvable this.";
		this.iSum = 1.0f / moment;

		this.w_coef = 1.0f - cpfexp(-this.damping * dt * moment);
		this.target_wrn = 0.0f;

		// apply spring torque
		float j_spring = this.springTorqueFunc.apply(this, a.getAngleInRadians() - b.getAngleInRadians()) * dt;
		a.setAngVel(a.getAngVel() - j_spring * a.getInverseMoment());
		b.setAngVel(b.getAngVel() + j_spring * b.getInverseMoment());
	}

	@Override
	protected void applyCachedImpulse(float dt_coef) {
	}

	@Override
	protected void applyImpulse() {
		// compute relative velocity
		float wrn = a.getAngVel() - b.getAngVel();//normal_relative_velocity(a, b, r1, r2, n) - this.target_vrn;

		// compute velocity loss from drag
		// not 100% certain this is derived correctly, though it makes sense
		float w_damp = (this.target_wrn - wrn) * this.w_coef;
		this.target_wrn = wrn + w_damp;

		//apply_impulses(a, b, this.r1, this.r2, cpvmult(this.n, v_damp*this.nMass));
		float j_damp = w_damp * this.iSum;
		a.setAngVel(a.getAngVel() + j_damp * a.getInverseMoment());
		b.setAngVel(b.getAngVel() - j_damp * b.getInverseMoment());
	}

	@Override
	protected float getImpulse() {
		return 0;
	}
}
