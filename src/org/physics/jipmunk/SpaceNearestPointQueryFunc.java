package org.physics.jipmunk;

/** @author jobernolte */
public interface SpaceNearestPointQueryFunc {
	void apply(Shape shape, float distance, Vector2f point);
}
