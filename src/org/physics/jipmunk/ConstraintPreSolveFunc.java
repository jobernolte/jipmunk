package org.physics.jipmunk;

/** @author jobernolte */
public interface ConstraintPreSolveFunc {
	void preSolve(Constraint constraint, Space space);
}
