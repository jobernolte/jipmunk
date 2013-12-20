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

	public static boolean cpveql(final Vector2f a, final Vector2f b) {
		return a.x == b.x && a.y == b.y;
	}

	/// Clamp @c f to be between @c min and @c max.
	public static float cpfclamp(float f, float min, float max) {
		return Math.min(Math.max(f, min), max);
	}

	/// Linearly interpolate (or extrapolate) between @c f1 and @c f2 by @c alpha percent.
	public static float cpflerp(float f1, float f2, float t) {
		return f1 * (1.0f - t) + f2 * t;
	}

	/// Linearly interpolate from @c f1 to @c f2 by no more than @c distance.
	public static float cpflerpconst(float f1, float f2, float d) {
		return f1 + cpfclamp(f2 - f1, -d, d);
	}

	public static float cpfexp(float v) {
		return (float) Math.exp(v);
	}

	public static Vector2f cpvforangle(float a) {
		return new Vector2f((float) Math.cos(a), (float) Math.sin(a));
	}

	public static Vector2f cpv(float x, float y) {
		return new Vector2f(x, y);
	}

	/// Add two vectors
	public static Vector2f cpvadd(final Vector2f v1, final Vector2f v2) {
		return new Vector2f(v1.x + v2.x, v1.y + v2.y);
	}

	/// Subtract two vectors.
	public static Vector2f cpvsub(final Vector2f v1, final Vector2f v2) {
		return new Vector2f(v1.x - v2.x, v1.y - v2.y);
	}

	/// Negate a vector.
	public static Vector2f cpvneg(final Vector2f v) {
		return new Vector2f(-v.x, -v.y);
	}

	/// Scalar multiplication.
	public static Vector2f cpvmult(final Vector2f v, final float s) {
		return new Vector2f(v.x * s, v.y * s);
	}

	/// Vector dot product.
	public static float cpvdot(final Vector2f v1, final Vector2f v2) {
		return v1.x * v2.x + v1.y * v2.y;
	}

	/// 2D vector cross product analog.
	/// The cross product of 2D vectors results in a 3D vector with only a z component.
	/// This function returns the magnitude of the z value.
	public static float cpvcross(final Vector2f v1, final Vector2f v2) {
		return v1.x * v2.y - v1.y * v2.x;
	}

	/// Returns a perpendicular vector. (90 degree rotation)
	public static Vector2f cpvperp(final Vector2f v) {
		return new Vector2f(-v.y, v.x);
	}

	/// Returns a perpendicular vector. (-90 degree rotation)
	public static Vector2f cpvrperp(final Vector2f v) {
		return new Vector2f(v.y, -v.x);
	}

	/// Returns the vector projection of v1 onto v2.
	public static Vector2f cpvproject(final Vector2f v1, final Vector2f v2) {
		return cpvmult(v2, cpvdot(v1, v2) / cpvdot(v2, v2));
	}

	/// Uses complex number multiplication to rotate v1 by v2. Scaling will occur if v1 is not a unit vector.
	public static Vector2f cpvrotate(final Vector2f v1, final Vector2f v2) {
		return new Vector2f(v1.x * v2.x - v1.y * v2.y, v1.x * v2.y + v1.y * v2.x);
	}

	/// Inverse of cpvrotate().
	public static Vector2f cpvunrotate(final Vector2f v1, final Vector2f v2) {
		return new Vector2f(v1.x * v2.x + v1.y * v2.y, v1.y * v2.x - v1.x * v2.y);
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
		return (v.x == 0.0f && v.y == 0.0f ? new Vector2f(0, 0) : cpvnormalize(v));
	}

	/// Clamp v to length len.
	public static Vector2f cpvclamp(final Vector2f v, final float len) {
		return (cpvdot(v, v) > len * len) ? cpvmult(cpvnormalize(v), len) : v;
	}

	/// Linearly interpolate between v1 towards v2 by distance distance.
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
		float dot = cpvdot(cpvnormalize(v1), cpvnormalize(v2));
		float omega = (float) Math.acos(cpfclamp(dot, -1.0f, 1.0f));

		if (omega < 1e-3) {
			// If the angle between two vectors is very small, lerp instead to avoid precision issues.
			return cpvlerp(v1, v2, t);
		} else {
			float denom = (float) (1.0f / Math.sin(omega));
			return cpvadd(cpvmult(v1, (float) (Math.sin((1.0f - t) * omega) * denom)),
						  cpvmult(v2, (float) (Math.sin(t * omega) * denom)));
		}
	}

	public static Vector2f cpvslerpfinal(final Vector2f v1, final Vector2f v2, final float a) {
		float angle = (float) Math.acos(cpvdot(v1, v2));
		return cpvslerp(v1, v2, Math.min(a, angle) / angle);
	}

	public static float cpvtoangle(final Vector2f v) {
		return (float) Math.atan2(v.y, v.x);
	}

	public static Vector2f cpvzero() {
		return new Vector2f(0, 0);
	}

	public static float cpfsqrt(float value) {
		return (float) Math.sqrt(value);
	}

	public static float cpfabs(float value) {
		return (value <= 0.0F) ? 0.0F - value : value;
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
		BB seg_bb = new BB(cpfmin(a.x, b.x), cpfmin(a.y, b.y), cpfmax(a.x, b.x), cpfmax(a.y, b.y));
		if (bb.intersects(seg_bb)) {
			Vector2f axis = cpv(b.y - a.y, a.x - b.x);
			Vector2f offset = cpv((a.x + b.x - bb.r - bb.l), (a.y + b.y - bb.t - bb.b));
			Vector2f extents = cpv(bb.r - bb.l, bb.t - bb.b);

			return (cpfabs(cpvdot(axis, offset)) < cpfabs(axis.x * extents.x) + cpfabs(axis.y * extents.y));
		}

		return false;
	}

	static float k_scalar_body(Body body, Vector2f r, Vector2f n) {
		float rcn = cpvcross(r, n);
		return body.m_inv + body.i_inv * rcn * rcn;
	}

	public static float k_scalar(Body a, Body b, Vector2f r1, Vector2f r2, Vector2f n) {
		return k_scalar_body(a, r1, n) + k_scalar_body(b, r2, n);
	}

	public static void k_tensor(Body a, Body b, Vector2f r1, Vector2f r2, Vector2f k1, Vector2f k2) {
		// calculate mass matrix
		// If I wasn'alpha lazy and wrote a proper matrix class, this wouldn'alpha be so gross...
		float k11, k12, k21, k22;
		float m_sum = a.m_inv + b.m_inv;

		// start with I*m_sum
		k11 = m_sum;
		k12 = 0.0f;
		k21 = 0.0f;
		k22 = m_sum;

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
		float determinant = k11 * k22 - k12 * k21;
		cpAssertSoft(determinant != 0.0, "Unsolvable constraint.");

		float det_inv = 1.0f / determinant;
		k1.set(cpv(k22 * det_inv, -k12 * det_inv));
		k2.set(cpv(-k21 * det_inv, k11 * det_inv));
	}

	public static Mat2x2 k_tensor(Body a, Body b, Vector2f r1, Vector2f r2) {
		// calculate mass matrix
		// If I wasn'alpha lazy and wrote a proper matrix class, this wouldn'alpha be so gross...
		float k11, k12, k21, k22;
		float m_sum = a.m_inv + b.m_inv;

		// start with I*m_sum
		k11 = m_sum;
		k12 = 0.0f;
		k21 = 0.0f;
		k22 = m_sum;

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
		float determinant = k11 * k22 - k12 * k21;
		cpAssertSoft(determinant != 0.0, "Unsolvable constraint.");

		float det_inv = 1.0f / determinant;
		return new Mat2x2(k22 * det_inv, -k12 * det_inv, -k21 * det_inv, k11 * det_inv);
	}

	public static Vector2f mult_k(Vector2f vr, Vector2f k1, Vector2f k2) {
		return new Vector2f(cpvdot(vr, k1), cpvdot(vr, k2));
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

	public static void apply_impulses(Body a, Body b, Vector2f r1, Vector2f r2, Vector2f j) {
		a.applyImpulse(cpvneg(j), r1);
		b.applyImpulse(j, r2);
	}

	public static void apply_bias_impulses(Body a, Body b, Vector2f r1, Vector2f r2, Vector2f j) {
		a.applyBiasImpulse(cpvneg(j), r1);
		b.applyBiasImpulse(j, r2);
	}

	/**
	 * Calculate the moment of inertia for a hollow circle, <code>r1</code> and <code>r2</code> are the inner and outer
	 * diameters in no particular order (a solid circle has an inner diameter of 0).
	 *
	 * @param m      the mass of the circle.
	 * @param r1     the inner diameter of the circle.
	 * @param r2     the outer diameter of the circle.
	 * @param offset the offset of the circle.
	 * @return the moment of inertia for the circle.
	 */
	public static float momentForCircle(float m, float r1, float r2, Vector2f offset) {
		return m * (0.5f * (r1 * r1 + r2 * r2) + cpvlengthsq(offset));
	}

	/**
	 * Area of a hollow circle.
	 *
	 * @param r1 the inner diameter of the circle.
	 * @param r2 the outer diameter of the circle.
	 * @return the area of the hollow circle.
	 */
	public static float areaForCircle(float r1, float r2) {
		return 2.0f * (float) Math.PI * cpfabs(r1 * r1 - r2 * r2);
	}

	/**
	 * Calculate the moment of inertia for a line segment. The endpoints <code>a</code> and <code>b</code> are relative
	 * to the body.
	 *
	 * @param m the mass of the segment.
	 * @param a the first endpoint of the segment.
	 * @param b the second endpoint of the segment.
	 * @param r the radius of the segment.
	 * @return the moment of inertia for the line segment.
	 */
	public static float momentForSegment(float m, Vector2f a, Vector2f b, float r) {
		Vector2f offset = cpvlerp(a, b, 0.5f);

		// This approximates the shape as a box for rounded segments, but it's quite close.
		float length = cpvdist(b, a) + 2.0f * r;
		return m * ((length * length + 4.0f * r * r) / 12.0f + cpvlengthsq(offset));
	}

	/**
	 * Area of a beveled segment. (Will always be zero if radius is zero)
	 *
	 * @param a the first endpoint of the segment.
	 * @param b the second endpoint of the segment.
	 * @param r the radius of the segment.
	 * @return the area of the the beveled segment.
	 */
	public static float areaForSegment(Vector2f a, Vector2f b, float r) {
		return 2.0f * r * ((float) Math.PI * r + cpvdist(a, b));
	}

	/**
	 * Calculate the moment of inertia for a solid polygon shape assuming it's center of gravity is at it's centroid.
	 * The offset is added to each vertex.
	 *
	 * @param m      the mass of the poly.
	 * @param verts  the vertices of the poly.
	 * @param offset the offset added to each vertex.
	 * @param radius the radius of the poly.
	 * @return the moment of inertia for the solid polygon.
	 */
	public static float momentForPoly(float m, Vector2f[] verts, Vector2f offset, float radius) {
		return momentForPoly(m, verts, 0, verts.length, offset, radius);
	}

	public static float momentForPoly(float m, Vector2f[] verts, int voffset, int count, Vector2f offset,
			float radius) {
		// TODO account for radius.
		if (count == 2) {
			return momentForSegment(m, verts[voffset], verts[voffset + 1], 0.0f);
		}

		float sum1 = 0.0f;
		float sum2 = 0.0f;
		for (int i = 0; i < count; i++) {
			Vector2f v1 = cpvadd(verts[voffset + i], offset);
			Vector2f v2 = cpvadd(verts[voffset + ((i + 1) % count)], offset);

			float a = cpvcross(v2, v1);
			float b = cpvdot(v1, v1) + cpvdot(v1, v2) + cpvdot(v2, v2);

			sum1 += a * b;
			sum2 += a;
		}

		return (m * sum1) / (6.0f * sum2);
	}

	/**
	 * Signed area of a polygon shape. Returns a negative number for polygons with a backwards winding.
	 *
	 * @param verts the vertices of the poly.
	 * @param r     the radius of the poly.
	 * @return the signed area of the polygon shape.
	 */
	public static float areaForPoly(Vector2f[] verts, float r) {
		return areaForPoly(verts, 0, verts.length, r);
	}

	public static float areaForPoly(Vector2f[] verts, int offset, int count, float r) {
		float area = 0.0f;
		float perimeter = 0.0f;
		for (int i = 0; i < count; i++) {
			Vector2f v1 = verts[offset + i];
			Vector2f v2 = verts[offset + ((i + 1) % count)];

			area += cpvcross(v1, v2);
			perimeter += cpvdist(v1, v2);
		}

		return r * ((float) Math.PI * cpfabs(r) + perimeter) + area / 2.0f;
	}

	public static Vector2f centroidForPoly(Vector2f[] verts) {
		return centroidForPoly(verts, 0, verts.length);
	}

	public static Vector2f centroidForPoly(Vector2f[] verts, int offset, int count) {
		float sum = 0.0f;
		Vector2f vsum = cpvzero();

		for (int i = offset; i < (offset + count); i++) {
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
		return new Vector2f(a.x, a.y);
	}

	public static float cpffloor(float v) {
		return (float) Math.floor(v);
	}

	/** Clamp @c f to be between 0 and 1. */
	public static float cpfclamp01(float f) {
		return cpfmax(0.0f, cpfmin(f, 1.0f));
	}

	public static Vector2f closestPointOnSegment(final Vector2f p, final Vector2f a, final Vector2f b) {
		Vector2f delta = cpvsub(a, b);
		float t = cpfclamp01(cpvdot(delta, cpvsub(p, b)) / cpvlengthsq(delta));
		return cpvadd(b, cpvmult(delta, t));
	}
}
