package org.physics.jipmunk;

/** @author jobernolte */
public class ContactPoint {
	/// The position of the contact point.
	public Vector2f point;
	/// The normal of the contact point.
	public Vector2f normal;
	/// The depth of the contact point.
	public float dist;

	public ContactPoint() {
	}

	public ContactPoint(Vector2f point, Vector2f normal, float dist) {
		this.point = point;
		this.normal = normal;
		this.dist = dist;
	}
}
