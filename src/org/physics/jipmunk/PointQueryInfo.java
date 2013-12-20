package org.physics.jipmunk;

/**
 * Nearest point query info struct.
 * <point/>
 * Point queries are useful for things like mouse picking and simple sensors. They allow you to check if there are
 * shapes within a certain distance of a point, find the closest point on a shape to a given point or find the closest
 * shape to a point.
 *
 * @author jobernolte
 */
public class PointQueryInfo {
	/** The nearest shape, <code>null</code> if no shape was within range. */
	public Shape shape;
	/** The closest point on the shape's surface. (in world space coordinates) */
	public Vector2f point = Util.cpvzero();
	/** The distance to the point. The distance is negative if the point is inside the shape. */
	public float distance;
	/// The gradient of the signed distance function.
	/// The same as info.point/info.distance, but accurate even for very small values of info.distance.
	public Vector2f gradient = Util.cpvzero();

	public PointQueryInfo() {
	}

	public PointQueryInfo(Shape shape, Vector2f point, float distance) {
		this.shape = shape;
		this.point = point;
		this.distance = distance;
	}

	public void set(Shape shape, Vector2f p, float d, Vector2f gradient) {
		this.shape = shape;
		this.point.set(p);
		this.distance = d;
		this.gradient.set(gradient);
	}

	public void set(PointQueryInfo info) {
		this.shape = info.shape;
		this.point.set(info.point);
		this.distance = info.distance;
		this.gradient.set(info.gradient);
	}

	public void reset() {
		this.shape = null;
		this.point.set(0, 0);
		this.distance = 0;
		this.gradient.set(0, 0);
	}
}
