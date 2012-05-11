package org.physics.jipmunk.constraints;

/** @author jobernolte */
public interface DampedRotarySpringTorqueFunc {
	float apply(DampedRotarySpring spring, float relativeAngleInRadians);
}
