package org.physics.jipmunk;

/** @author jobernolte */
public interface CollisionHandler {
	boolean begin(Arbiter arb, Space space);

	boolean preSolve(Arbiter arb, Space space);

	void postSolve(Arbiter arb, Space space);

	void separate(Arbiter arb, Space space);
}
