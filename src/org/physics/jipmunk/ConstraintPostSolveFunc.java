package org.physics.jipmunk;

/** @author jobernolte */
public interface ConstraintPostSolveFunc {
	void postSolve(Constraint constraint, Space space);
}
