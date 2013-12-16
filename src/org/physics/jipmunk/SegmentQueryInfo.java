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

import static org.physics.jipmunk.Util.cpvzero;

/**
 * Segment queries return more information than just a simple yes or no, they also return where a shape was hit and it’s
 * surface normal at the hit point. {@link SegmentQueryInfo#alpha} is the percentage between the query start and end points.
 * If you need the hit point in world space or the absolute distance from start, see the segment query helper functions
 * farther down.
 *
 * @author jobernolte
 */
public class SegmentQueryInfo {
	/** The shape that was hit, <code>null</code> if no collision occured. */
	public Shape shape;
	/** The point of impact. */
	public Vector2f point = Util.cpvzero();
	/** The normal of the surface hit. */
	public Vector2f normal = cpvzero();
	/** The normalized distance along the query segment in the range [0, 1]. */
	public float alpha = 1.0f;

	public SegmentQueryInfo() {

	}

	public SegmentQueryInfo(Shape shape, Vector2f point, Vector2f normal, float alpha) {
		this.shape = shape;
		this.point.set(point);
		this.normal.set(normal);
		this.alpha = alpha;
	}

	public void set(Shape shape, Vector2f point, Vector2f normal, float alpha) {
		this.shape = shape;
		this.point.set(point);
		this.normal.set(normal);
		this.alpha = alpha;
	}

	public void set(SegmentQueryInfo info) {
		this.shape = info.shape;
		this.point.set(info.point);
		this.normal.set(info.normal);
		this.alpha = info.alpha;
	}

	public void reset() {
		shape = null;
		point.set(0, 0);
		alpha = 1;
		normal.set(0, 0);
	}

	/**
	 * Get the hit point for a segment query.
	 *
	 * @param start
	 * @param end
	 * @return
	 */
	public Vector2f getHitPoint(final Vector2f start, final Vector2f end) {
		return Util.cpvlerp(start, end, this.alpha);
	}

	/**
	 * Get the hit distance for a segment query.
	 *
	 * @param start
	 * @param end
	 * @return
	 */
	public float getHitDist(final Vector2f start, final Vector2f end) {
		return Util.cpvdist(start, end) * this.alpha;
	}
}
