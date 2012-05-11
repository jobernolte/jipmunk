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
 * surface normal at the hit point. {@link SegmentQueryInfo#t} is the percentage between the query start and end points.
 * If you need the hit point in world space or the absolute distance from start, see the segment query helper functions
 * farther down.
 *
 * @author jobernolte
 */
public class SegmentQueryInfo {
	/** The shape that was hit, <code>null</code> if no collision occured. */
	public Shape shape;
	/** The normalized distance along the query segment in the range [0, 1]. */
	public float t = 1.0f;
	/** The normal of the surface hit. */
	public Vector2f n = cpvzero();

	public SegmentQueryInfo() {

	}

	public void set(Shape shape, float t, Vector2f n) {
		this.shape = shape;
		this.t = t;
		this.n.set(n);
	}

	public void reset() {
		shape = null;
		t = 1;
		n.set(0, 0);
	}
}
