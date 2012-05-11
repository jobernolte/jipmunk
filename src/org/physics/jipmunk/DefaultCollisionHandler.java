package org.physics.jipmunk;

/** @author jobernolte */
public class DefaultCollisionHandler implements CollisionHandler {
	@Override
	public boolean begin(Arbiter arb, Space space) {
		return true;
	}

	@Override
	public boolean preSolve(Arbiter arb, Space space) {
		return true;
	}

	@Override
	public void postSolve(Arbiter arb, Space space) {
	}

	@Override
	public void separate(Arbiter arb, Space space) {
	}
}
