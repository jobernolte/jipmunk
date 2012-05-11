package org.physics.jipmunk;

/** @author jobernolte */
enum ArbiterState {
	// Arbiter is active and its the first collision.
	cpArbiterStateFirstColl,
	// Arbiter is active and its not the first collision.
	cpArbiterStateNormal,
	// Collision has been explicitly ignored.
	// Either by returning false from a begin collision handler or calling cpArbiterIgnore().
	cpArbiterStateIgnore,
	// Collison is no longer active. A space will cache an arbiter for up to cpSpace.collisionPersistence more steps.
	cpArbiterStateCached,
}
