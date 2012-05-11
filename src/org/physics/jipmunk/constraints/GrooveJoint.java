package org.physics.jipmunk.constraints;

import org.physics.jipmunk.Body;
import org.physics.jipmunk.Constraint;
import org.physics.jipmunk.Util;
import org.physics.jipmunk.Vector2f;

import static org.physics.jipmunk.Util.apply_impulses;
import static org.physics.jipmunk.Util.bias_coef;
import static org.physics.jipmunk.Util.cpvadd;
import static org.physics.jipmunk.Util.cpvclamp;
import static org.physics.jipmunk.Util.cpvcross;
import static org.physics.jipmunk.Util.cpvdot;
import static org.physics.jipmunk.Util.cpvlength;
import static org.physics.jipmunk.Util.cpvmult;
import static org.physics.jipmunk.Util.cpvnormalize;
import static org.physics.jipmunk.Util.cpvperp;
import static org.physics.jipmunk.Util.cpvproject;
import static org.physics.jipmunk.Util.cpvrotate;
import static org.physics.jipmunk.Util.cpvsub;
import static org.physics.jipmunk.Util.cpvzero;
import static org.physics.jipmunk.Util.k_tensor;
import static org.physics.jipmunk.Util.mult_k;
import static org.physics.jipmunk.Util.relative_velocity;

/** @author jobernolte */
public class GrooveJoint extends Constraint {
	Vector2f grv_n, grv_a, grv_b;
	Vector2f anchr2;

	Vector2f grv_tn;
	float clamp;
	Vector2f r1, r2;
	Vector2f k1 = Util.cpvzero(), k2 = Util.cpvzero();

	Vector2f jAcc;
	float jMaxLen;
	Vector2f bias;

	void init(Vector2f groove_a, Vector2f groove_b, Vector2f anchr2) {
		this.grv_a = Util.cpv(groove_a);
		this.grv_b = Util.cpv(groove_b);
		this.grv_n = cpvperp(cpvnormalize(cpvsub(groove_b, groove_a)));
		this.anchr2 = anchr2;

		this.jAcc = cpvzero();
	}

	public GrooveJoint(Body a, Body b, Vector2f groove_a, Vector2f groove_b, Vector2f anchr2) {
		super(a, b);
		init(groove_a, groove_b, anchr2);
	}

	public Vector2f getGrooveA() {
		return grv_a;
	}

	public void setGrooveA(Vector2f value) {
		this.grv_a = value;
		this.grv_n = cpvperp(cpvnormalize(cpvsub(this.grv_b, value)));

		activateBodies();
	}

	public Vector2f getGrooveB() {
		return grv_b;
	}

	public void setGrooveB(Vector2f value) {
		this.grv_b = value;
		this.grv_n = cpvperp(cpvnormalize(cpvsub(value, this.grv_a)));

		activateBodies();
	}

	public Vector2f getAnchr2() {
		return anchr2;
	}

	public void setAnchr2(Vector2f anchr2) {
		this.anchr2 = anchr2;
	}

	@Override
	protected void preStep(float dt) {
		// calculate endpoints in worldspace
		Vector2f ta = a.local2World(this.grv_a);
		Vector2f tb = a.local2World(this.grv_b);

		// calculate axis
		Vector2f n = cpvrotate(this.grv_n, a.getRotation());
		float d = cpvdot(ta, n);

		this.grv_tn = n;
		this.r2 = cpvrotate(this.anchr2, b.getRotation());

		// calculate tangential distance along the axis of r2
		float td = cpvcross(cpvadd(b.getPosition(), this.r2), n);
		// calculate clamping factor and r2
		if (td <= cpvcross(ta, n)) {
			this.clamp = 1.0f;
			this.r1 = cpvsub(ta, a.getPosition());
		} else if (td >= cpvcross(tb, n)) {
			this.clamp = -1.0f;
			this.r1 = cpvsub(tb, a.getPosition());
		} else {
			this.clamp = 0.0f;
			this.r1 = cpvsub(cpvadd(cpvmult(cpvperp(n), -td), cpvmult(n, d)), a.getPosition());
		}

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

	Vector2f grooveConstrain(Vector2f j) {
		Vector2f n = this.grv_tn;
		Vector2f jClamp = (this.clamp * cpvcross(j, n) > 0.0f) ? j : cpvproject(j, n);
		return cpvclamp(jClamp, this.jMaxLen);
	}

	@Override
	protected void applyImpulse() {
		Vector2f r1 = this.r1;
		Vector2f r2 = this.r2;

		// compute impulse
		Vector2f vr = relative_velocity(a, b, r1, r2);

		Vector2f j = mult_k(cpvsub(this.bias, vr), this.k1, this.k2);
		Vector2f jOld = this.jAcc;
		this.jAcc = grooveConstrain(cpvadd(jOld, j));
		j = cpvsub(this.jAcc, jOld);

		// apply impulse
		apply_impulses(a, b, this.r1, this.r2, j);
	}

	@Override
	protected float getImpulse() {
		return cpvlength(this.jAcc);
	}
}
