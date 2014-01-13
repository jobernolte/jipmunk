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

package org.physics.jipmunk.impl;

import org.physics.jipmunk.*;

import static org.physics.jipmunk.Util.cpvdot;
import static org.physics.jipmunk.Util.cpvzero;

/**
 * Support points are the maximal points on a shape's perimeter along a certain axis.
 * The GJK and EPA algorithms use support points to iteratively sample the surface of the two shapes' minkowski difference.
 *
 * @author jobernolte
 */
public class SupportPoint {
	private final Vector2f p;
	/** Save an index of the point so it can be cheaply looked up as a starting point for the next frame. */
	private final CollisionID index;

	public SupportPoint(Vector2f p, CollisionID index) {
		this.p = new Vector2f(p);
		this.index = index;
	}

	public Vector2f getP() {
		return p;
	}

	public CollisionID getIndex() {
		return index;
	}

	public static SupportPoint circleSupportPoint(Shape shape, Vector2f n) {
		return new SupportPoint(((CircleShape) shape).getTransformedCenter(), new CollisionID(0));
	}

	public static SupportPoint segmentSupportPoint(Shape shape, Vector2f n) {
		SegmentShape seg = (SegmentShape) shape;
		if (cpvdot(seg.getTa(), n) > cpvdot(seg.getTb(), n)) {
			return new SupportPoint(seg.getTa(), new CollisionID(0));
		} else {
			return new SupportPoint(seg.getTb(), new CollisionID(1));
		}
	}

	public static int polySupportPointIndex(int count, SplittingPlane[] planes, Vector2f n) {
		float max = Float.NEGATIVE_INFINITY;
		int index = 0;

		for (int i = 0; i < count; i++) {
			Vector2f v = planes[i].v0;
			float d = cpvdot(v, n);
			if (d > max) {
				max = d;
				index = i;
			}
		}

		return index;
	}

	public static SupportPoint polySupportPoint(Shape shape, Vector2f n) {
		PolyShape poly = (PolyShape) shape;
		SplittingPlane[] planes = poly.getPlanes();
		int i = polySupportPointIndex(planes.length, planes, n);
		return new SupportPoint(planes[i].v0, new CollisionID(i));
	}

	/**
	 * Get a SupportPoint from a cached shape and index.
	 *
	 * @param shape the cached shape.
	 * @param i     the index.
	 * @return the support point.
	 */
	public static SupportPoint shapePoint(Shape shape, int i) {
		switch (shape.getType()) {
			case CIRCLE_SHAPE: {
				return new SupportPoint(((CircleShape) shape).getTransformedCenter(), new CollisionID(0));
			}
			case SEGMENT_SHAPE: {
				SegmentShape seg = (SegmentShape) shape;
				return new SupportPoint(i == 0 ? seg.getTa() : seg.getTb(), new CollisionID(i));
			}
			case POLY_SHAPE: {
				PolyShape poly = (PolyShape) shape;
				final SplittingPlane[] planes = poly.getPlanes();
				// Poly shapes may change vertex count.
				int index = (i < planes.length ? i : 0);
				return new SupportPoint(planes[index].v0, new CollisionID(index));
			}
			default: {
				return new SupportPoint(cpvzero(), new CollisionID(0));
			}
		}
	}
}
