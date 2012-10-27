/*
 * Copyright (c) 2007 Scott Lembcke, (c) 2011 Jürgen Obernolte
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.physics.jipmunk;

import static org.physics.jipmunk.Assert.cpAssertSoft;

/** @author jobernolte */
public class Util {

	/// Clamp @c f to be between @c min and @c max.
	public static float cpfclamp(float f, float min, float max) {
		return Math.min(Math.max(f, min), max);
	}

	/// Linearly interpolate (or extrapolate) between @c f1 and @c f2 by @c t percent.
	public static float cpflerp(float f1, float f2, float t) {
		return f1 * (1.0f - t) + f2 * t;
	}

	/// Linearly interpolate from @c f1 to @c f2 by no more than @c d.
	public static float cpflerpconst(float f1, float f2, float d) {
		return f1 + cpfclamp(f2 - f1, -d, d);
	}

	public static float cpfexp(float v) {
		return (float) Math.exp(v);
	}

	public static Vector2f cpvforangle(float a) {
		return new DefaultVector2f((float) Math.cos(a), (float) Math.sin(a));
	}

	public static Vector2f cpv(float x, float y) {
		return new DefaultVector2f(x, y);
	}

	/// Add two vectors
	public static Vector2f cpvadd(final Vector2f v1, final Vector2f v2) {
		return new DefaultVector2f(v1.getX() + v2.getX(), v1.getY() + v2.getY());
	}

	/// Subtract two vectors.
	public static Vector2f cpvsub(final Vector2f v1, final Vector2f v2) {
		return new DefaultVector2f(v1.getX() - v2.getX(), v1.getY() - v2.getY());
	}

	/// Negate a vector.
	public static Vector2f cpvneg(final Vector2f v) {
		return new DefaultVector2f(-v.getX(), -v.getY());
	}

	/// Scalar multiplication.
	public static Vector2f cpvmult(final Vector2f v, final float s) {
		return new DefaultVector2f(v.getX() * s, v.getY() * s);
	}

	/// Vector dot product.
	public static float cpvdot(final Vector2f v1, final Vector2f v2) {
		return v1.getX() * v2.getX() + v1.getY() * v2.getY();
	}

	/// 2D vector cross product analog.
/// The cross product of 2D vectors results in a 3D vector with only a z component.
/// This function returns the magnitude of the z value.
	public static float cpvcross(final Vector2f v1, final Vector2f v2) {
		return v1.getX() * v2.getY() - v1.getY() * v2.getX();
	}

	/// Returns a perpendicular vector. (90 degree rotation)
	public static Vector2f cpvperp(final Vector2f v) {
		return new DefaultVector2f(-v.getY(), v.getX());
	}

	/// Returns a perpendicular vector. (-90 degree rotation)
	public static Vector2f cpvrperp(final Vector2f v) {
		return new DefaultVector2f(v.getY(), -v.getX());
	}

	/// Returns the vector projection of v1 onto v2.
	public static Vector2f cpvproject(final Vector2f v1, final Vector2f v2) {
		return cpvmult(v2, cpvdot(v1, v2) / cpvdot(v2, v2));
	}

	/// Uses complex number multiplication to rotate v1 by v2. Scaling will occur if v1 is not a unit vector.
	public static Vector2f cpvrotate(final Vector2f v1, final Vector2f v2) {
		return new DefaultVector2f(v1.getX() * v2.getX() - v1.getY() * v2.getY(), v1.getX() * v2.getY() + v1.getY() * v2.getX());
	}

	/// Inverse of cpvrotate().
	public static Vector2f cpvunrotate(final Vector2f v1, final Vector2f v2) {
		return new DefaultVector2f(v1.getX() * v2.getX() + v1.getY() * v2.getY(), v1.getY() * v2.getX() - v1.getX() * v2.getY());
	}

	/// Returns the squared length of v. Faster than cpvlength() when you only need to compare lengths.
	public static float cpvlengthsq(final Vector2f v) {
		return cpvdot(v, v);
	}

	/// Linearly interpolate between v1 and v2.
	public static Vector2f cpvlerp(final Vector2f v1, final Vector2f v2, final float t) {
		return cpvadd(cpvmult(v1, 1.0f - t), cpvmult(v2, t));
	}

	/// Returns a normalized copy of v.
	public static Vector2f cpvnormalize(final Vector2f v) {
		return cpvmult(v, 1.0f / cpvlength(v));
	}

	/// Returns a normalized copy of v or cpvzero if v was already cpvzero. Protects against divide by zero errors.
	public static Vector2f cpvnormalize_safe(final Vector2f v) {
		return (v.getX() == 0.0f && v.getY() == 0.0f ? new DefaultVector2f(0, 0) : cpvnormalize(v));
	}

	/// Clamp v to length len.
	public static Vector2f cpvclamp(final Vector2f v, final float len) {
		return (cpvdot(v, v) > len * len) ? cpvmult(cpvnormalize(v), len) : v;
	}

	/// Linearly interpolate between v1 towards v2 by distance d.
	public static Vector2f cpvlerpfinal(Vector2f v1, Vector2f v2, float d) {
		return cpvadd(v1, cpvclamp(cpvsub(v2, v1), d));
	}

	/// Returns the distance between v1 and v2.
	public static float cpvdist(final Vector2f v1, final Vector2f v2) {
		return cpvlength(cpvsub(v1, v2));
	}

	/// Returns the squared distance between v1 and v2. Faster than cpvdist() when you only need to compare distances.
	public static float cpvdistsq(final Vector2f v1, final Vector2f v2) {
		return cpvlengthsq(cpvsub(v1, v2));
	}

	/// Returns true if the distance between v1 and v2 is less than dist.
	public static boolean cpvnear(final Vector2f v1, final Vector2f v2, final float dist) {
		return cpvdistsq(v1, v2) < dist * dist;
	}

	public static float cpvlength(final Vector2f v) {
		return (float) Math.sqrt(cpvdot(v, v));
	}

	public static Vector2f cpvslerp(final Vector2f v1, final Vector2f v2, final float t) {
		float omega = (float) Math.acos(cpvdot(v1, v2));

		if (omega != 0) {
			float denom = (float) (1.0f / Math.sin(omega));
			return cpvadd(cpvmult(v1, (float) (Math.sin((1.0f - t) * omega) * denom)), cpvmult(v2, (float) (Math.sin(
					t * omega) * denom)));
		} else {
			return v1;
		}
	}

	public static Vector2f cpvslerpfinal(final Vector2f v1, final Vector2f v2, final float a) {
		float angle = (float) Math.acos(cpvdot(v1, v2));
		return cpvslerp(v1, v2, Math.min(a, angle) / angle);
	}

	public static float cpvtoangle(final Vector2f v) {
		return (float) Math.atan2(v.getY(), v.getX());
	}

	public static Vector2f cpvzero() {
		return new DefaultVector2f(0, 0);
	}

	public static float cpfsqrt(float value) {
		return (float) Math.sqrt(value);
	}

	public static float cpfabs(float value) {
		return Math.abs(value);
	}

	public static float cpfmin(float a, float b) {
		return Math.min(a, b);
	}

	public static float cpfmax(float a, float b) {
		return Math.max(a, b);
	}

	public static float cpfpow(float a, float b) {
		return (float) Math.pow(a, b);
	}

	public static BB cpBBNew(float l, float b, float r, float t) {
		return new BB(l, b, r, t);
	}

	/// Returns the area of the bounding box.
	static float cpBBArea(BB bb) {
		return (bb.r - bb.l) * (bb.t - bb.b);
	}

	/// Merges @c a and @c b and returns the area of the merged bounding box.
	static float cpBBMergedArea(BB a, BB b) {
		return (cpfmax(a.r, b.r) - cpfmin(a.l, b.l)) * (cpfmax(a.t, b.t) - cpfmin(a.b, b.b));
	}

	/// Return true if the bounding box intersects the line segment with ends @c a and @c b.
	static boolean cpBBIntersectsSegment(BB bb, Vector2f a, Vector2f b) {
		BB seg_bb = new BB(cpfmin(a.getX(), b.getX()), cpfmin(a.getY(), b.getY()), cpfmax(a.getX(), b.getX()), cpfmax(
				a.getY(), b.getY()));
		if (bb.intersects(seg_bb)) {
			Vector2f axis = cpv(b.getY() - a.getY(), a.getX() - b.getX());
			Vector2f offset = cpv((a.getX() + b.getX() - bb.r - bb.l), (a.getY() + b.getY() - bb.t - bb.b));
			Vector2f extents = cpv(bb.r - bb.l, bb.t - bb.b);

			return (cpfabs(cpvdot(axis, offset)) < cpfabs(axis.getX() * extents.getX()) + cpfabs(
					axis.getY() * extents.getY()));
		}

		return false;
	}

	public static float k_scalar(Body a, Body b, Vector2f r1, Vector2f r2, Vector2f n) {
		float mass_sum = a.m_inv + b.m_inv;
		float r1cn = cpvcross(r1, n);
		float r2cn = cpvcross(r2, n);

		float value = mass_sum + a.i_inv * r1cn * r1cn + b.i_inv * r2cn * r2cn;
		assert value != 0.0 : "Unsolvable collision or constraint.";

		return value;
	}

	public static void k_tensor(Body a, Body b, Vector2f r1, Vector2f r2, Vector2f k1, Vector2f k2) {
		// calculate mass matrix
		// If I wasn't lazy and wrote a proper matrix class, this wouldn't be so gross...
		float k11, k12, k21, k22;
		float m_sum = a.m_inv + b.m_inv;

		// start with I*m_sum
		k11 = m_sum;
		k12 = 0.0f;
		k21 = 0.0f;
		k22 = m_sum;

		// add the influence from r1
		float a_i_inv = a.i_inv;
		float r1xsq = r1.getX() * r1.getX() * a_i_inv;
		float r1ysq = r1.getY() * r1.getY() * a_i_inv;
		float r1nxy = -r1.getX() * r1.getY() * a_i_inv;
		k11 += r1ysq;
		k12 += r1nxy;
		k21 += r1nxy;
		k22 += r1xsq;

		// add the influnce from r2
		float b_i_inv = b.i_inv;
		float r2xsq = r2.getX() * r2.getX() * b_i_inv;
		float r2ysq = r2.getY() * r2.getY() * b_i_inv;
		float r2nxy = -r2.getX() * r2.getY() * b_i_inv;
		k11 += r2ysq;
		k12 += r2nxy;
		k21 += r2nxy;
		k22 += r2xsq;

		// invert
		float determinant = k11 * k22 - k12 * k21;
		cpAssertSoft(determinant != 0.0, "Unsolvable constraint.");

		float det_inv = 1.0f / determinant;
		k1.set(cpv(k22 * det_inv, -k12 * det_inv));
		k2.set(cpv(-k21 * det_inv, k11 * det_inv));
	}

	public static Mat2x2 k_tensor(Body a, Body b, Vector2f r1, Vector2f r2) {
		// calculate mass matrix
		// If I wasn't lazy and wrote a proper matrix class, this wouldn't be so gross...
		float k11, k12, k21, k22;
		float m_sum = a.m_inv + b.m_inv;

		// start with I*m_sum
		k11 = m_sum;
		k12 = 0.0f;
		k21 = 0.0f;
		k22 = m_sum;

		// add the influence from r1
		float a_i_inv = a.i_inv;
		float r1xsq = r1.getX() * r1.getX() * a_i_inv;
		float r1ysq = r1.getY() * r1.getY() * a_i_inv;
		float r1nxy = -r1.getX() * r1.getY() * a_i_inv;
		k11 += r1ysq;
		k12 += r1nxy;
		k21 += r1nxy;
		k22 += r1xsq;

		// add the influnce from r2
		float b_i_inv = b.i_inv;
		float r2xsq = r2.getX() * r2.getX() * b_i_inv;
		float r2ysq = r2.getY() * r2.getY() * b_i_inv;
		float r2nxy = -r2.getX() * r2.getY() * b_i_inv;
		k11 += r2ysq;
		k12 += r2nxy;
		k21 += r2nxy;
		k22 += r2xsq;

		// invert
		float determinant = k11 * k22 - k12 * k21;
		cpAssertSoft(determinant != 0.0, "Unsolvable constraint.");

		float det_inv = 1.0f / determinant;
		return new Mat2x2(
			 k22*det_inv, -k12*det_inv,
			-k21*det_inv,  k11*det_inv
	 	);
	}

	public static Vector2f mult_k(Vector2f vr, Vector2f k1, Vector2f k2) {
		return new DefaultVector2f(cpvdot(vr, k1), cpvdot(vr, k2));
	}

	public static float bias_coef(float errorBias, float dt) {
		return 1.0f - cpfpow(errorBias, dt);
	}

	public static Vector2f relative_velocity(Body a, Body b, Vector2f r1, Vector2f r2) {
		Vector2f v1_sum = cpvadd(a.v, cpvmult(cpvperp(r1), a.w));
		Vector2f v2_sum = cpvadd(b.v, cpvmult(cpvperp(r2), b.w));

		return cpvsub(v2_sum, v1_sum);
	}

	public static float normal_relative_velocity(Body a, Body b, Vector2f r1, Vector2f r2, Vector2f n) {
		return cpvdot(relative_velocity(a, b, r1, r2), n);
	}

	public static void apply_impulse(Body body, Vector2f j, Vector2f r) {
		body.v = cpvadd(body.v, cpvmult(j, body.m_inv));
		body.w += body.i_inv * cpvcross(r, j);
	}

	public static void apply_impulses(Body a, Body b, Vector2f r1, Vector2f r2, Vector2f j) {
		apply_impulse(a, cpvneg(j), r1);
		apply_impulse(b, j, r2);
	}

	public static void apply_bias_impulse(Body body, Vector2f j, Vector2f r) {
		body.v_bias = cpvadd(body.v_bias, cpvmult(j, body.m_inv));
		body.w_bias += body.i_inv * cpvcross(r, j);
	}

	public static void apply_bias_impulses(Body a, Body b, Vector2f r1, Vector2f r2, Vector2f j) {
		apply_bias_impulse(a, cpvneg(j), r1);
		apply_bias_impulse(b, j, r2);
	}

	public final static long CP_HASH_COEF = 3344921057L;

	public static long CP_HASH_PAIR(int A, int B) {
		//return (int) ((A) * CP_HASH_COEF ^ (B) * CP_HASH_COEF);
		return (((long) A) << 32) | B;
	}

	public static float momentForCircle(float m, float r1, float r2, Vector2f offset) {
		return m * (0.5f * (r1 * r1 + r2 * r2) + cpvlengthsq(offset));
	}

	public static float areaForCircle(float r1, float r2) {
		return 2.0f * (float) Math.PI * cpfabs(r1 * r1 - r2 * r2);
	}

	public static float momentForSegment(float m, Vector2f a, Vector2f b) {
		float length = cpvlength(cpvsub(b, a));
		Vector2f offset = cpvmult(cpvadd(a, b), 1.0f / 2.0f);

		return m * (length * length / 12.0f + cpvlengthsq(offset));
	}

	public static float areaForSegment(Vector2f a, Vector2f b, float r) {
		return 2.0f * r * ((float) Math.PI * r + cpvdist(a, b));
	}

	public static float momentForPoly(float m, Vector2f[] verts, Vector2f offset) {
		return momentForPoly(m, verts, 0, verts.length, offset);
	}

	public static float momentForPoly(float m, Vector2f[] verts, int voffset, int length, Vector2f offset) {
		float sum1 = 0.0f;
		float sum2 = 0.0f;
		for (int i = 0; i < length; i++) {
			Vector2f v1 = cpvadd(verts[voffset + i], offset);
			Vector2f v2 = cpvadd(verts[voffset + ((i + 1) % length)], offset);

			float a = cpvcross(v2, v1);
			float b = cpvdot(v1, v1) + cpvdot(v1, v2) + cpvdot(v2, v2);

			sum1 += a * b;
			sum2 += a;
		}

		return (m * sum1) / (6.0f * sum2);
	}

	public static float areaForPoly(Vector2f[] verts) {
		float area = 0.0f;
		for (int i = 0; i < verts.length; i++) {
			area += cpvcross(verts[i], verts[(i + 1) % verts.length]);
		}

		return -area / 2.0f;
	}

	public static Vector2f centroidForPoly(Vector2f[] verts) {
		float sum = 0.0f;
		Vector2f vsum = cpvzero();

		for (int i = 0; i < verts.length; i++) {
			Vector2f v1 = verts[i];
			Vector2f v2 = verts[(i + 1) % verts.length];
			float cross = cpvcross(v1, v2);

			sum += cross;
			vsum = cpvadd(vsum, cpvmult(cpvadd(v1, v2), cross));
		}

		return cpvmult(vsum, 1.0f / (3.0f * sum));
	}

	public static void recenterPoly(Vector2f[] verts) {
		Vector2f centroid = centroidForPoly(verts);

		for (int i = 0; i < verts.length; i++) {
			verts[i] = cpvsub(verts[i], centroid);
		}
	}

	public static float momentForBox(float m, float width, float height) {
		return m * (width * width + height * height) / 12.0f;
	}

	public static Vector2f cpv(Vector2f a) {
		return new DefaultVector2f(a.getX(), a.getY());
	}

	public static float cpffloor(float v) {
		return (float) Math.floor(v);
	}
}
