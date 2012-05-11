package org.physics.jipmunk;

/** @author jobernolte */
class CollisionHandlerEntry {
	CollisionHandler handler;
	int a;
	int b;

	protected CollisionHandlerEntry(CollisionHandler handler, int a, int b) {
		this.handler = handler;
		this.a = a;
		this.b = b;
	}

	boolean begin(Arbiter arb, Space space) {
		return handler.begin(arb, space);
	}

	boolean preSolve(Arbiter arb, Space space) {
		return handler.preSolve(arb, space);
	}

	void postSolve(Arbiter arb, Space space) {
		handler.postSolve(arb, space);
	}

	void separate(Arbiter arb, Space space) {
		handler.separate(arb, space);
	}
}
