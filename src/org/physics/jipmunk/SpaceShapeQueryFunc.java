package org.physics.jipmunk;

/** @author jobernolte */
public interface SpaceShapeQueryFunc {
	void apply(Shape shape, ContactPointSet contactPointSet);
}
