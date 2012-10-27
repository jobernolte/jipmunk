package org.physics.jipmunk;

/** @author jobernolte */
public class Mat2x2 {
	public float a, b, c, d;

	public Mat2x2(float a, float b, float c, float d) {
		this.a = a;
		this.b = b;
		this.c = c;
		this.d = d;
	}

	public Vector2f transform(final Vector2f v) {
		return Util.cpv(v.getX() * this.a + v.getY() * this.b, v.getX() * this.c + v.getY() * this.d);
	}
}
