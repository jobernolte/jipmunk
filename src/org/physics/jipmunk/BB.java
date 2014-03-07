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

import static org.physics.jipmunk.Util.cpv;
import static org.physics.jipmunk.Util.cpvlerp;

/**
 * Simple bounding box struct. Stored as left, bottom, right, top values.
 *
 * @author jobernolte
 */
public class BB {
	public float l;
	public float b;
	public float r;
	public float t;

	public BB() {

	}

	/**
	 * Convenience constructor.
	 *
	 * @param l the left edge.
	 * @param b the bottom edge.
	 * @param r the right edge.
	 * @param t the top edge.
	 */
	public BB(float l, float b, float r, float t) {
		this.l = l;
		this.b = b;
		this.r = r;
		this.t = t;
	}

	/// Returns true if @c a and @c b intersect.
	public static boolean intersects(final BB a, final BB b) {
		return (a.l <= b.r && b.l <= a.r && a.b <= b.t && b.b <= a.t);
	}

	/**
	 * Returns true if <code>this</code> and <code>bb</code> intersect.
	 *
	 * @param bb the other BB to intersect with
	 * @return <code>true</code> if <code>this</code> intersects <code>bb</code>
	 */
	public boolean intersects(final BB bb) {
		return (this.l <= bb.r && bb.l <= this.r && this.b <= bb.t && bb.b <= this.t);
	}

	/// Returns true if @c other lies completely within @c bb.
	public static boolean contains(final BB bb, final BB other) {
		return (bb.l <= other.l && bb.r >= other.r && bb.b <= other.b && bb.t >= other.t);
	}

	/// Returns true if @c bb contains @c v.
	public static boolean contains(final BB bb, final Vector2f v) {
		return (bb.l <= v.x && bb.r >= v.x && bb.b <= v.y && bb.t >= v.y);
	}

	/**
	 * Returns true if this BB completely contains <code>other</code>.
	 *
	 * @param other the other BB to test against.
	 * @return <code>true</code> if <code>other</code> lies completely within this BB.
	 */
	public boolean contains(final BB other) {
		return (this.l <= other.l && this.r >= other.r && this.b <= other.b && this.t >= other.t);
	}

	public boolean contains(final Vector2f v) {
		return (this.l <= v.x && this.r >= v.x && this.b <= v.y && this.t >= v.y);
	}

	/// Returns a bounding box that holds both bounding boxes.
	public static BB merge(final BB a, final BB b) {
		return new BB(Math.min(a.l, b.l), Math.min(a.b, b.b), Math.max(a.r, b.r), Math.max(a.t, b.t));
	}

	/// Returns a bounding box that holds both bounding boxes.
	public BB merge(final BB b) {
		return new BB(Math.min(this.l, b.l), Math.min(this.b, b.b), Math.max(this.r, b.r), Math.max(this.t, b.t));
	}

	/// Returns a bounding box that holds both @c bb and @c v.
	public static BB expand(final BB bb, final Vector2f v) {
		return new BB(Math.min(bb.l, v.x), Math.min(bb.b, v.y), Math.max(bb.r, v.x), Math.max(bb.t, v.y));
	}

	/// Returns a bounding box that holds both @c bb and @c v.
	public BB expand(final Vector2f v) {
		return new BB(Math.min(this.l, v.x), Math.min(this.b, v.y), Math.max(this.r, v.x), Math.max(this.t, v.y));
	}

	/// Returns the area of the bounding box.
	public static float area(BB bb) {
		return (bb.r - bb.l) * (bb.t - bb.b);
	}

	/**
	 * @return the area of this bounding box.
	 */
	public float area() {
		return (this.r - this.l) * (this.t - this.b);
	}

	/// Merges @c a and @c b and returns the area of the merged bounding box.
	public static float mergedArea(final BB a, final BB b) {
		return (Math.max(a.r, b.r) - Math.min(a.l, b.l)) * (Math.max(a.t, b.t) - Math.min(a.b, b.b));
	}

	/**
	 * Merges this bounding box and {@code b} and returns the area of the merged bounding box.
	 *
	 * @param b the other bounding box to merge.
	 * @return the area of the merged bounding box.
	 */
	public float mergedArea(final BB b) {
		return (Math.max(this.r, b.r) - Math.min(this.l, b.l)) * (Math.max(this.t, b.t) - Math.min(this.b, b.b));
	}

	/**
	 * Returns the fraction along the segment query the BB is hit. Returns {@link Float#POSITIVE_INFINITY} if it doesn'alpha hit.
	 * Returns the fraction along the segment query the BB is hit. Returns {@link Float#POSITIVE_INFINITY} if it
	 * doesn'alpha hit.
	 *
	 * @param bb the bounding box to test.
	 * @param a  the start of the segment.
	 * @param b  the end of the segment.
	 * @return the fraction along the segment query if <code>bb</code> is hit, {@link Float#POSITIVE_INFINITY}
	 * otherwise.
	 */
	public static float segmentQuery(BB bb, Vector2f a, Vector2f b) {
		return bb.segmentQuery(a, b);
	}

	/**
	 * Returns the fraction along the segment query the BB is hit. Returns {@link Float#POSITIVE_INFINITY} if it doesn'alpha hit.
	 *
	 * @param a the start of the segment.
	 * @param b the end of the segment.
	 * @return the fraction along the segment query if <code>bb</code> is hit, {@link Float#POSITIVE_INFINITY} otherwise.
	 */
	public float segmentQuery(Vector2f a, Vector2f b) {
		float idx = 1.0f / (b.x - a.x);
		float tx1 = (this.l == a.x ? Float.NEGATIVE_INFINITY : (this.l - a.x) * idx);
		float tx2 = (this.r == a.x ? Float.POSITIVE_INFINITY : (this.r - a.x) * idx);
		float txmin = Math.min(tx1, tx2);
		float txmax = Math.max(tx1, tx2);

		float idy = 1.0f / (b.y - a.y);
		float ty1 = (this.b == a.y ? Float.NEGATIVE_INFINITY : (this.b - a.y) * idy);
		float ty2 = (this.t == a.y ? Float.POSITIVE_INFINITY : (this.t - a.y) * idy);
		float tymin = Math.min(ty1, ty2);
		float tymax = Math.max(ty1, ty2);

		if (tymin <= txmax && txmin <= tymax) {
			float min = Math.min(txmin, tymin);
			float max = Math.max(txmax, tymax);

			if (0.0f <= max && min <= 1.0f) {
				return Math.max(min, 0.0f);
			}
		}

		return Float.POSITIVE_INFINITY;
	}

	/// Return true if the bounding box intersects the line segment with ends @c a and @c b.
	public static boolean intersectsSegment(final BB bb, final Vector2f a, final Vector2f b) {
		return (segmentQuery(bb, a, b) != Float.POSITIVE_INFINITY);
	}

	/// Return true if the bounding box intersects the line segment with ends @c a and @c b.
	public boolean intersectsSegment(final Vector2f a, final Vector2f b) {
		return (segmentQuery(a, b) != Float.POSITIVE_INFINITY);
	}

	/**
	 * Clamp a vector to this bounding box.
	 *
	 * @param vector the vector to clamp.
	 * @return a new vector instance clamped to this bounding box.
	 */
	public Vector2f clampVect(final Vector2f vector) {
		float x = Math.min(Math.max(this.l, vector.x), this.r);
		float y = Math.min(Math.max(this.b, vector.y), this.t);
		return cpv(x, y);

	}

	/**
	 * wrap a vector to this bounding box.
	 *
	 * @param vector the vector to wrap.
	 * @return a new vector instance wrapped to this bounding box.
	 */
	public Vector2f wrapVect(final Vector2f vector) {
		float ix = Math.abs(this.r - this.l);
		float modx = (vector.x - this.l) % ix;
		float x = (modx > 0.0f) ? modx : modx + ix;

		float iy = Math.abs(this.t - this.b);
		float mody = (vector.y - this.b) % iy;
		float y = (mody > 0.0f) ? mody : mody + iy;

		return cpv(x + this.l, y + this.b);
	}

	/// Constructs a cpBB for a circle with the given position and radius.

	/**
	 * Convenience constructor for making a {@link BB} fitting a circle at position <code>p</code> with radius
	 * <code>r</code>.
	 *
	 * @param p the position of the circle.
	 * @param r the radius of the circle.
	 * @return a {@link BB} fitting the circle.
	 */
	public static BB forCircle(final Vector2f p, float r) {
		return new BB(p.x - r, p.y - r, p.x + r, p.y + r);
	}

	/// Constructs a cpBB centered on a point with the given extents (half sizes).
	public static BB forExtents(final Vector2f c, float hw, float hh) {
		return new BB(c.x - hw, c.y - hh, c.x + hw, c.y + hh);
	}

	/**
	 * Returns the center of a bounding box.
	 *
	 * @return the center of a bounding box.
	 */
	public Vector2f getCenter() {
		return cpvlerp(cpv(this.l, this.b), cpv(this.r, this.t), 0.5f);
	}

	@Override
	public boolean equals(Object o) {
		if (this == o) {
			return true;
		}
		if (o == null || getClass() != o.getClass()) {
			return false;
		}

		BB bb = (BB) o;

		return Float.compare(bb.b, b) == 0 && Float.compare(bb.l, l) == 0 && Float.compare(bb.r, r) == 0
				&& Float.compare(bb.t, t) == 0;

	}

	@Override
	public int hashCode() {
		int result = (l != +0.0f ? Float.floatToIntBits(l) : 0);
		result = 31 * result + (b != +0.0f ? Float.floatToIntBits(b) : 0);
		result = 31 * result + (r != +0.0f ? Float.floatToIntBits(r) : 0);
		result = 31 * result + (t != +0.0f ? Float.floatToIntBits(t) : 0);
		return result;
	}

	@Override
	public String toString() {
		return "BB{" + "l=" + l + ", b=" + b + ", r=" + r + ", t=" + t + '}';
	}
}
