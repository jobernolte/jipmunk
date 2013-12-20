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

import static org.physics.jipmunk.Util.*;

/** @author jobernolte */
public class SegmentShape extends Shape {

	private Vector2f a, b, n;
	Vector2f ta;
	Vector2f tb;
	Vector2f tn;
	float radius;
	Vector2f a_tangent, b_tangent;

	static MassInfo createMassInfo(float mass, Vector2f a, Vector2f b, float r) {
		return new MassInfo(mass, Util.momentForBox(1.0f, cpvdist(a, b) + 2.0f * r, 2.0f * r),
							// TODO is an approximation.
							cpvlerp(a, b, 0.5f), Util.areaForSegment(a, b, r));
	}

	public SegmentShape(Body body, Vector2f a, Vector2f b, float radius) {
		super(body, createMassInfo(0.0f, a, b, radius));
		this.a = Util.cpv(a);
		this.b = Util.cpv(b);
		this.radius = radius;
		this.n = cpvperp(cpvnormalize(cpvsub(b, a)));
		this.a_tangent = Util.cpvzero();
		this.b_tangent = Util.cpvzero();
	}

	public Vector2f getA() {
		return a;
	}

	public Vector2f getB() {
		return b;
	}

	public Vector2f getN() {
		return n;
	}

	public float getRadius() {
		return radius;
	}

	public void setRadius(float radius) {
		this.radius = radius;
		float mass = this.massInfo.m;
		this.massInfo = createMassInfo(mass, this.a, this.b, this.radius);
		if (mass > 0.0f) {
			body.accumulateMassFromShapes();
		}
	}

	public Vector2f getTa() {
		return ta;
	}

	public Vector2f getTb() {
		return tb;
	}

	public Vector2f getTn() {
		return tn;
	}

	public Vector2f getATangent() {
		return a_tangent;
	}

	public Vector2f getBTangent() {
		return b_tangent;
	}

	@Override
	public ShapeType getType() {
		return ShapeType.SEGMENT_SHAPE;
	}

	@Override
	protected BB cacheData(Transform transform) {
		this.ta = transform.transformPoint(this.a);
		this.tb = transform.transformPoint(this.b);
		this.tn = transform.transformVect(this.n);

		float l, r, b, t;

		if (this.ta.x < this.tb.x) {
			l = this.ta.x;
			r = this.tb.x;
		} else {
			l = this.tb.x;
			r = this.ta.x;
		}

		if (this.ta.y < this.tb.y) {
			b = this.ta.y;
			t = this.tb.y;
		} else {
			b = this.tb.y;
			t = this.ta.y;
		}

		float rad = this.radius;
		return new BB(l - rad, b - rad, r + rad, t + rad);
	}

	static boolean inUnitRange(float t) {
		return (0.0f < t && t < 1.0f);
	}

	@Override
	protected void segmentQueryImpl(Vector2f a, Vector2f b, float r2, SegmentQueryInfo info) {
		Vector2f n = this.tn;
		float d = cpvdot(cpvsub(this.ta, a), n);
		float r = this.radius + r2;

		Vector2f flipped_n = (d > 0.0f ? cpvneg(n) : n);
		Vector2f seg_offset = cpvsub(cpvmult(flipped_n, r), a);

		// Make the endpoints relative to 'a' and move them by the thickness of the segment.
		Vector2f seg_a = cpvadd(this.ta, seg_offset);
		Vector2f seg_b = cpvadd(this.tb, seg_offset);
		Vector2f delta = cpvsub(b, a);

		if (cpvcross(delta, seg_a) * cpvcross(delta, seg_b) <= 0.0f) {
			float d_offset = d + (d > 0.0f ? -r : r);
			float ad = -d_offset;
			float bd = cpvdot(delta, n) - d_offset;

			if (ad * bd < 0.0f) {
				float t = ad / (ad - bd);

				info.shape = this;
				info.point.set(cpvsub(cpvlerp(a, b, t), cpvmult(flipped_n, r2)));
				info.normal.set(flipped_n);
				info.alpha = t;
			}
		} else if (r != 0.0f) {
			SegmentQueryInfo info1 = new SegmentQueryInfo(null, b, cpvzero(), 1.0f);
			SegmentQueryInfo info2 = new SegmentQueryInfo(null, b, cpvzero(), 1.0f);
			CircleShape.circleSegmentQuery(this, this.ta, this.radius, a, b, r2, info1);
			CircleShape.circleSegmentQuery(this, this.tb, this.radius, a, b, r2, info2);

			if (info1.alpha < info2.alpha) {
				info.set(info1);
			} else {
				info.set(info2);
			}
		}
	}

	@Override
	public PointQueryInfo pointQuery(Vector2f p, PointQueryInfo out) {
		Vector2f closest = Util.closestPointOnSegment(p, this.ta, this.tb);

		Vector2f delta = cpvsub(p, closest);
		float d = Util.cpvlength(delta);
		float r = this.radius;
		Vector2f g = cpvmult(delta, 1.0f / d);

		if (out == null) {
			out = new PointQueryInfo();
		}
		out.set(this, (d != 0 ? cpvadd(closest, cpvmult(delta, r / d)) : closest), d - r,
				(d > Constants.MAGIC_EPSILON ? g : this.n));
		return out;
	}

	public void setEndpoints(final Vector2f a, final Vector2f b) {
		this.a = Util.cpv(a);
		this.b = Util.cpv(b);
		this.n = cpvperp(cpvnormalize(cpvsub(b, a)));
	}

	public void setNeighbors(final Vector2f prev, final Vector2f next) {
		this.a_tangent = cpvsub(prev, this.a);
		this.b_tangent = cpvsub(next, this.b);
	}

	public static MassInfo massInfo(float mass, Vector2f a, Vector2f b, float r) {
		// TODO is an approximation.
		return new MassInfo(mass, momentForBox(1.0f, cpvdist(a, b) + 2.0f * r, 2.0f * r), cpvlerp(a, b, 0.5f),
							areaForSegment(a, b, r));
	}
}
