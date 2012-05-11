package org.physics.jipmunk;

/** @author jobernolte */
public interface BodyVelocityFunc {
	void velocity(Body body, final Vector2f gravity, float damping, float dt);
}
