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

/** @author jobernolte */
public class BB {
	public float l;
	public float b;
	public float r;
	public float t;

	public BB() {

	}

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

	/// Returns true if @c other lies completely within @c bb.
	public static boolean contains(final BB bb, final BB other) {
		return (bb.l <= other.l && bb.r >= other.r && bb.b <= other.b && bb.t >= other.t);
	}

	/// Returns true if @c bb contains @c v.
	public static boolean contains(final BB bb, final Vector2f v) {
		return (bb.l <= v.getX() && bb.r >= v.getX() && bb.b <= v.getY() && bb.t >= v.getY());
	}

	/// Returns a bounding box that holds both bounding boxes.
	public static BB merge(final BB a, final BB b) {
		return new BB(
				Math.min(a.l, b.l),
				Math.min(a.b, b.b),
				Math.max(a.r, b.r),
				Math.max(a.t, b.t)
		);
	}

	/// Returns a bounding box that holds both @c bb and @c v.
	public static BB expand(final BB bb, final Vector2f v) {
		return new BB(
				Math.min(bb.l, v.getX()),
				Math.min(bb.b, v.getY()),
				Math.max(bb.r, v.getX()),
				Math.max(bb.t, v.getY())
		);
	}

	/// Returns the area of the bounding box.
	public static float area(BB bb) {
		return (bb.r - bb.l) * (bb.t - bb.b);
	}

	/// Merges @c a and @c b and returns the area of the merged bounding box.
	public static float mergedArea(final BB a, final BB b) {
		return (Math.max(a.r, b.r) - Math.min(a.l, b.l)) * (Math.max(a.t, b.t) - Math.min(a.b, b.b));
	}

	/// Return true if the bounding box intersects the line segment with ends @c a and @c b.
	public static boolean intersectsSegment(final BB bb, final Vector2f a, final Vector2f b) {
		BB seg_bb = new BB(Math.min(a.getX(), b.getX()), Math.min(a.getY(), b.getY()), Math.max(a.getX(), b.getX()),
				Math.max(a.getY(), b.getY()));
		if (intersects(bb, seg_bb)) {
			Vector2f axis = Util.cpv(b.getY() - a.getY(), a.getX() - b.getX());
			Vector2f offset = Util.cpv((a.getX() + b.getX() - bb.r - bb.l), (a.getY() + b.getY() - bb.t - bb.b));
			Vector2f extents = Util.cpv(bb.r - bb.l, bb.t - bb.b);

			return (Math.abs(Util.cpvdot(axis, offset)) < Math.abs(axis.getX() * extents.getX()) + Math.abs(
					axis.getY() * extents
							.getY()));
		}

		return false;
	}

	/// Clamp a vector to a bounding box.
	public static Vector2f clampVect(final BB bb, final Vector2f v) {
		float x = Math.min(Math.max(bb.l, v.getX()), bb.r);
		float y = Math.min(Math.max(bb.b, v.getY()), bb.t);
		return Util.cpv(x, y);

	}

	/// Wrap a vector to a bounding box.
	public static Vector2f wrapVect(final BB bb, final Vector2f v) {
		float ix = Math.abs(bb.r - bb.l);
		float modx = (v.getX() - bb.l) % ix;
		float x = (modx > 0.0f) ? modx : modx + ix;

		float iy = Math.abs(bb.t - bb.b);
		float mody = (v.getY() - bb.b) % iy;
		float y = (mody > 0.0f) ? mody : mody + iy;

		return Util.cpv(x + bb.l, y + bb.b);
	}

	/// Constructs a cpBB for a circle with the given position and radius.
	public static BB forCircle(final Vector2f p, float r) {
		return new BB(p.getX() - r, p.getY() - r, p.getX() + r, p.getY() + r);
	}
}
