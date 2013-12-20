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

	public static Mat2x2 k_tensor(Body a, Body b, Vector2f r1, Vector2f r2) {
		float m_sum = a.m_inv + b.m_inv;

		// start with Identity*m_sum
		float k11 = m_sum, k12 = 0.0f;
		float k21 = 0.0f, k22 = m_sum;

		// add the influence from r1
		float a_i_inv = a.i_inv;
		float r1xsq = r1.x * r1.x * a_i_inv;
		float r1ysq = r1.y * r1.y * a_i_inv;
		float r1nxy = -r1.x * r1.y * a_i_inv;
		k11 += r1ysq;
		k12 += r1nxy;
		k21 += r1nxy;
		k22 += r1xsq;

		// add the influnce from r2
		float b_i_inv = b.i_inv;
		float r2xsq = r2.x * r2.x * b_i_inv;
		float r2ysq = r2.y * r2.y * b_i_inv;
		float r2nxy = -r2.x * r2.y * b_i_inv;
		k11 += r2ysq;
		k12 += r2nxy;
		k21 += r2nxy;
		k22 += r2xsq;

		// invert
		float det = k11 * k22 - k12 * k21;
		if (det == 0.0f) {
			throw new IllegalArgumentException("Unsolvable constraint.");
		}

		float det_inv = 1.0f / det;
		return new Mat2x2(k22 * det_inv, -k12 * det_inv, -k21 * det_inv, k11 * det_inv);
	}

}
