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
public class CircleShape extends Shape {
	private Vector2f c = Util.cpvzero();
	Vector2f tc;
	float radius;

	static MassInfo createMassInfo(float mass, float radius, Vector2f center) {
		return new MassInfo(mass, Util.momentForCircle(1.0f, 0.0f, radius, cpvzero()), center,
							Util.areaForCircle(0.0f, radius));
	}

	public CircleShape(Body body, float radius, Vector2f offset) {
		super(body, createMassInfo(0.0f, radius, offset));
		this.c.set(offset);
		this.radius = radius;
	}

	@Override
	public ShapeType getType() {
		return ShapeType.CIRCLE_SHAPE;
	}

	@Override
	protected BB cacheData(Transform transform) {
		Vector2f c = this.tc = transform.transformPoint(this.c);
		return BB.forCircle(c, this.radius);
	}

	protected static void circleSegmentQuery(Shape shape, Vector2f center, float r1, Vector2f a, Vector2f b, float r2,
			SegmentQueryInfo info) {
		Vector2f da = cpvsub(a, center);
		Vector2f db = cpvsub(b, center);
		float rsum = r1 + r2;

		float qa = cpvdot(da, da) - 2.0f * cpvdot(da, db) + cpvdot(db, db);
		float qb = cpvdot(da, db) - cpvdot(da, da);
		float det = qb * qb - qa * (cpvdot(da, da) - rsum * rsum);

		if (det >= 0.0f) {
			float t = (-qb - cpfsqrt(det)) / (qa);
			if (0.0f <= t && t <= 1.0f) {
				Vector2f n = cpvnormalize(cpvlerp(da, db, t));

				info.shape = shape;
				info.point.set(cpvsub(cpvlerp(a, b, t), cpvmult(n, r2)));
				info.normal.set(n);
				info.alpha = t;
			}
		}
	}

	@Override
	protected void segmentQueryImpl(Vector2f a, Vector2f b, float radius, SegmentQueryInfo info) {
		circleSegmentQuery(this, this.tc, this.radius, a, b, radius, info);
	}

	@Override
	public PointQueryInfo pointQuery(Vector2f p, PointQueryInfo out) {
		Vector2f delta = cpvsub(p, this.tc);
		float d = Util.cpvlength(delta);
		float r = this.radius;

		if (out == null) {
			out = new PointQueryInfo();
		}
		out.set(this, cpvadd(this.tc, cpvmult(delta, r / d)), d - r,
				(d > Constants.MAGIC_EPSILON ? cpvmult(delta, 1.0f / d) : cpv(0.0f, 1.0f)));
		return out;
	}

	public float getRadius() {
		return radius;
	}

	public void setRadius(float radius) {
		this.radius = radius;

		float mass = this.massInfo.m;
		this.massInfo = createMassInfo(mass, this.radius, this.c);
		if (mass > 0.0f) {
			body.accumulateMassFromShapes();
		}
	}

	public Vector2f getOffset() {
		return c;
	}

	public void setOffset(final Vector2f offset) {
		this.c.set(offset);

		float mass = this.massInfo.m;
		this.massInfo = createMassInfo(mass, this.radius, this.c);
		if (mass > 0.0f) {
			body.accumulateMassFromShapes();
		}
	}

	public Vector2f getTransformedCenter() {
		return tc;
	}
}
