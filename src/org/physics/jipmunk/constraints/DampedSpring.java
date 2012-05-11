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

	private Vector2f anchr1, anchr2;
	private float restLength;
	private float stiffness;
	private float damping;
	private DampedSpringForceFunc springForceFunc;

	private float target_vrn;
	private float v_coef;

	private Vector2f r1, r2;
	private float nMass;
	private Vector2f n;

	public DampedSpring(Body a, Body b, Vector2f anchr1, Vector2f anchr2, float restLength, float stiffness,
			float damping) {
		super(a, b);

		this.anchr1 = Util.cpv(anchr1);
		this.anchr2 = Util.cpv(anchr2);

		this.restLength = restLength;
		this.stiffness = stiffness;
		this.damping = damping;
		this.springForceFunc = defaultSpringForce;
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

	public float getRestLength() {
		return restLength;
	}

	public void setRestLength(float restLength) {
		this.restLength = restLength;
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

	public DampedSpringForceFunc getSpringForceFunc() {
		return springForceFunc;
	}

	public void setSpringForceFunc(DampedSpringForceFunc springForceFunc) {
		this.springForceFunc = springForceFunc;
	}

	@Override
	protected void preStep(float dt) {
		this.r1 = cpvrotate(this.anchr1, a.getRotation());
		this.r2 = cpvrotate(this.anchr2, b.getRotation());

		Vector2f delta = cpvsub(cpvadd(b.getPosition(), this.r2), cpvadd(a.getPosition(), this.r1));
		float dist = cpvlength(delta);
		this.n = cpvmult(delta, 1.0f / (dist != 0.0f ? dist : Float.POSITIVE_INFINITY));

		float k = k_scalar(a, b, this.r1, this.r2, this.n);
		assert k != 0.0 : "Unsolvable this.";
		this.nMass = 1.0f / k;

		this.target_vrn = 0.0f;
		this.v_coef = 1.0f - cpfexp(-this.damping * dt * k);

		// apply spring force
		float f_spring = this.springForceFunc.apply(this, dist);
		apply_impulses(a, b, this.r1, this.r2, cpvmult(this.n, f_spring * dt));
	}

	@Override
	protected void applyCachedImpulse(float dt_coef) {
	}

	@Override
	protected void applyImpulse() {
		Vector2f n = this.n;
		Vector2f r1 = this.r1;
		Vector2f r2 = this.r2;

		// compute relative velocity
		float vrn = normal_relative_velocity(a, b, r1, r2, n);

		// compute velocity loss from drag
		float v_damp = (this.target_vrn - vrn) * this.v_coef;
		this.target_vrn = vrn + v_damp;

		apply_impulses(a, b, this.r1, this.r2, cpvmult(this.n, v_damp * this.nMass));
	}

	@Override
	protected float getImpulse() {
		return 0;
	}
}
