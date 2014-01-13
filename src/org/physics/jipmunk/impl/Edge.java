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
import static org.physics.jipmunk.Util.cpvneg;

/**
 * Support edges are the edges of a polygon or segment shape that are in contact.
 *
 * @author jobernolte
 */
public class Edge {
	private final EdgePoint a, b;
	private final float r;
	private final Vector2f n;

	public Edge(EdgePoint a, EdgePoint b, float r, Vector2f n) {
		this.a = a;
		this.b = b;
		this.r = r;
		this.n = new Vector2f(n);
	}

	public EdgePoint getA() {
		return a;
	}

	public EdgePoint getB() {
		return b;
	}

	public float getR() {
		return r;
	}

	public Vector2f getN() {
		return n;
	}

	@Override
	public String toString() {
		return "Edge{" +
				"a=" + a +
				", b=" + b +
				", r=" + r +
				", n=" + n +
				'}';
	}

	public static Edge edgeForPoly(final PolyShape poly, final Vector2f n) {
		final SplittingPlane[] planes = poly.getPlanes();
		int count = planes.length;
		int i1 = SupportPoint.polySupportPointIndex(planes.length, planes, n);

		// TODO: get rid of mod eventually, very expensive on ARM
		int i0 = (i1 - 1 + count) % count;
		int i2 = (i1 + 1) % count;

		int hashid = poly.getHashId();
		if (cpvdot(n, planes[i1].n) > cpvdot(n, planes[i2].n)) {
			return new Edge(new EdgePoint(planes[i0].v0, HashValue.hashPair(hashid, i0)),
							new EdgePoint(planes[i1].v0, HashValue.hashPair(hashid, i1)), poly.getRadius(),
							planes[i1].n);
		} else {
			return new Edge(new EdgePoint(planes[i1].v0, HashValue.hashPair(hashid, i1)),
							new EdgePoint(planes[i2].v0, HashValue.hashPair(hashid, i2)), poly.getRadius(),
							planes[i2].n);
		}
	}

	public static Edge edgeForSegment(final SegmentShape seg, final Vector2f n) {
		int hashid = seg.getHashId();
		if (cpvdot(seg.getTn(), n) > 0.0f) {
			return new Edge(new EdgePoint(seg.getTa(), HashValue.hashPair(hashid, 0)),
							new EdgePoint(seg.getTb(), HashValue.hashPair(hashid, 1)), seg.getRadius(), seg.getTn());
		} else {
			return new Edge(new EdgePoint(seg.getTb(), HashValue.hashPair(hashid, 1)),
							new EdgePoint(seg.getTa(), HashValue.hashPair(hashid, 0)), seg.getRadius(),
							cpvneg(seg.getTn()));
		}
	}
}
