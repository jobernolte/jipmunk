package org.physics.jipmunk;

import static org.physics.jipmunk.Util.cpvzero;

/** @author jobernolte */
public class SegmentQueryInfo {
	/** The shape that was hit, NULL if no collision occured. */
	public Shape shape;
	/** The normalized distance along the query segment in the range [0, 1]. */
	public float t = 1.0f;
	/** The normal of the surface hit. */
	public Vector2f n = cpvzero();

	public SegmentQueryInfo() {

	}

	public void set(Shape shape, float t, Vector2f n) {
		this.shape = shape;
		this.t = t;
		this.n.set(n);
	}

	public void reset() {
		shape = null;
		t = 1;
		n.set(0, 0);
	}
}
