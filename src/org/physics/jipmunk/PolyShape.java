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

import java.util.Iterator;

import static org.physics.jipmunk.Util.*;

/**
 * @author jobernolte
 */
public class PolyShape extends Shape {
	float radius;
	SplittingPlane[] planes;
	SplittingPlane[] origPlanes;

	static MassInfo createMassInfo(float mass, Vector2f[] verts, int offset, int count, float radius) {
		// TODO moment is approximate due to radius.

		Vector2f centroid = Util.centroidForPoly(verts, offset, count);
		return new MassInfo(mass, Util.momentForPoly(1.0f, verts, offset, count, cpvneg(centroid), radius), centroid,
				Util.areaForPoly(verts, offset, count, radius));

	}

	public PolyShape(Body body, float radius, Vector2f... verts) {
		this(body, radius, null, verts, 0, verts.length);
	}

	public PolyShape(Body body, float radius, Transform transform, Vector2f... verts) {
		this(body, radius, transform, verts, 0, verts.length);
	}

	public PolyShape(Body body, float radius, Vector2f[] verts, int offset, int count) {
		this(body, radius, null, verts, offset, count);
	}

	public PolyShape(Body body, float radius, Transform transform, Vector2f[] verts, int offset, int count) {
		super(body, createMassInfo(0.0f, verts, offset, count, radius));
		setVertices(verts, offset, count, transform);
		this.radius = radius;
	}

	public int getNumVertices() {
		return planes.length;
	}

	public Iterable<Vector2f> getVertices() {
		return () -> new Iterator<Vector2f>() {
			int index;

			@Override
			public boolean hasNext() {
				return index < planes.length;
			}

			@Override
			public Vector2f next() {
				return planes[index++].v0;
			}
		};
	}

	public Vector2f getVertexAt(int index) {
		return planes[index].v0;
	}

	public void setVertices(Vector2f[] verts, int offset, int count, Transform transform) {

		Vector2f[] hullVerts = new Vector2f[count];
		if (transform != null) {
			for (int i = 0; i < count; i++) {
				hullVerts[i] = transform.transformPoint(verts[offset + i]);
			}
		} else {
			System.arraycopy(verts, offset, hullVerts, 0, count);
		}

		/*
		 * for (int i = 0; i < count; i++) { System.out.format("SetVerts1: i=%d, v=%s\n", i, hullVerts[offset + i]); }
		 */

		count = ConvexHullUtil.convexHull(hullVerts, hullVerts, count, 0.0f).count;

		this.planes = new SplittingPlane[count];
		this.origPlanes = new SplittingPlane[count];

		for (int i = 0; i < count; i++) {
			Vector2f a = hullVerts[(i - 1 + count) % count];
			Vector2f b = hullVerts[i];
			Vector2f n = cpvnormalize(cpvrperp(cpvsub(b, a)));

			// System.out.format("SetVerts2: i=%d, b=%s, n=%s\n", i, b, n);

			this.planes[i] = new SplittingPlane();
			this.origPlanes[i] = new SplittingPlane();
			this.origPlanes[i].v0 = new Vector2f(b);
			this.origPlanes[i].n = new Vector2f(n);
		}
		float mass = this.massInfo.m;
		this.massInfo = createMassInfo(mass, hullVerts, 0, count, this.radius);
		if (mass > 0.0f) {
			body.accumulateMassFromShapes();
		}
	}

	@Override
	public ShapeType getType() {
		return ShapeType.POLY_SHAPE;
	}

	@Override
	protected BB cacheData(Transform transform) {
		int count = this.planes.length;

		float l = Float.POSITIVE_INFINITY, r = Float.NEGATIVE_INFINITY;
		float b = Float.POSITIVE_INFINITY, t = Float.NEGATIVE_INFINITY;

		for (int i = 0; i < count; i++) {
			Vector2f v = transform.transformPoint(origPlanes[i].v0);
			Vector2f n = transform.transformVect(origPlanes[i].n);

			planes[i].v0 = v;
			planes[i].n = n;

			l = cpfmin(l, v.x);
			r = cpfmax(r, v.x);
			b = cpfmin(b, v.y);
			t = cpfmax(t, v.y);
		}

		float radius = this.radius;
		return (this.bb = new BB(l - radius, b - radius, r + radius, t + radius));
	}

	@Override
	protected void segmentQueryImpl(Vector2f a, Vector2f b, float r2, SegmentQueryInfo info) {
		SplittingPlane[] planes = this.planes;
		int count = this.planes.length;
		float r = this.radius;
		float rsum = r + r2;

		for (int i = 0; i < count; i++) {
			Vector2f n = planes[i].n;
			float an = cpvdot(a, n);
			float d = an - cpvdot(planes[i].v0, n) - rsum;
			if (d < 0.0f) continue;

			float bn = cpvdot(b, n);
			float t = d / (an - bn);
			if (t < 0.0f || 1.0f < t) continue;

			Vector2f point = cpvlerp(a, b, t);
			float dt = cpvcross(n, point);
			float dtMin = cpvcross(n, planes[(i - 1 + count) % count].v0);
			float dtMax = cpvcross(n, planes[i].v0);

			if (dtMin <= dt && dt <= dtMax) {
				info.shape = this;
				info.point.set(cpvsub(cpvlerp(a, b, t), cpvmult(n, r2)));
				info.normal.set(n);
				info.alpha = t;
			}
		}

		// Also check against the beveled vertexes.
		if (rsum > 0.0f) {
			SegmentQueryInfo circleInfo = new SegmentQueryInfo(null, b, cpvzero(), 1.0f);
			for (int i = 0; i < count; i++) {
				circleInfo.alpha = 1.0f;
				CircleShape.circleSegmentQuery(this, planes[i].v0, r, a, b, r2, circleInfo);
				if (circleInfo.alpha < info.alpha) {
					info.set(circleInfo);
				}
			}
		}
	}

	@Override
	public PointQueryInfo pointQuery(Vector2f p, PointQueryInfo info) {
		int count = this.planes.length;
		SplittingPlane[] planes = this.planes;
		float r = this.radius;

		Vector2f v0 = planes[count - 1].v0;
		float minDist = Float.POSITIVE_INFINITY;
		Vector2f closestPoint = cpvzero();
		Vector2f closestNormal = cpvzero();
		boolean outside = false;

		for (int i = 0; i < count; i++) {
			Vector2f v1 = planes[i].v0;
			if (cpvdot(planes[i].n, cpvsub(p, v1)) > 0.0f) outside = true;

			Vector2f closest = closestPointOnSegment(p, v0, v1);

			float dist = cpvdist(p, closest);
			if (dist < minDist) {
				minDist = dist;
				closestPoint = closest;
				closestNormal = planes[i].n;
			}

			v0 = v1;
		}

		float dist = (outside ? minDist : -minDist);
		Vector2f g = cpvmult(cpvsub(p, closestPoint), 1.0f / dist);
		if (info == null) {
			info = new PointQueryInfo();
		}
		info.shape = this;
		info.point.set(cpvadd(closestPoint, cpvmult(g, r)));
		info.distance = dist - r;

		// Use the normal of the closest segment if the distance is small.
		info.gradient.set(minDist > Constants.MAGIC_EPSILON ? g : closestNormal);
		return info;
	}

	static boolean validate(final Vector2f[] verts, int offset, int count) {
		for (int i = 0; i < count; i++) {
			Vector2f a = verts[offset + i];
			Vector2f b = verts[offset + ((i + 1) % count)];
			Vector2f c = verts[offset + ((i + 2) % count)];

			if (cpvcross(cpvsub(b, a), cpvsub(c, b)) < 0.0f) {
				return false;
			}
		}
		return true;
	}

	static public PolyShape createBox(Body body, float width, float height, float radius) {
		float hw = width / 2.0f;
		float hh = height / 2.0f;
		return createBox(body, new BB(-hw, -hh, hw, hh), radius);
	}

	static public PolyShape createBox(Body body, BB box, float radius) {
		Vector2f verts[] = {cpv(box.r, box.b), cpv(box.r, box.t), cpv(box.l, box.t), cpv(box.l, box.b),};
		return new PolyShape(body, radius, verts);
	}

	public static MassInfo massInfo(float mass, Vector2f[] verts, int offset, int count, float radius) {
		// TODO moment is approximate due to radius.

		Vector2f centroid = centroidForPoly(verts, offset, count);
		return new MassInfo(mass, momentForPoly(1.0f, verts, offset, count, cpvneg(centroid), radius), centroid,
				areaForPoly(verts, offset, count, radius));
	}

	public SplittingPlane[] getPlanes() {
		return planes;
	}

	public float getRadius() {
		return radius;
	}
}
