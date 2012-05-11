package org.physics.jipmunk.constraints;

/** @author jobernolte */
public interface DampedSpringForceFunc {
	float apply(DampedSpring spring, float dist);
}
