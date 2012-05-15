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

import static org.physics.jipmunk.Util.cpfsqrt;
import static org.physics.jipmunk.Util.cpvadd;
import static org.physics.jipmunk.Util.cpvdot;
import static org.physics.jipmunk.Util.cpvlerp;
import static org.physics.jipmunk.Util.cpvnear;
import static org.physics.jipmunk.Util.cpvnormalize;
import static org.physics.jipmunk.Util.cpvrotate;
import static org.physics.jipmunk.Util.cpvsub;

/** @author jobernolte */
public class CircleShape extends Shape {
	private Vector2f c = Util.cpvzero();
	Vector2f tc;
	float r;

	public CircleShape(Body body, float radius, Vector2f offset) {
		super(body);
		this.c.set(offset);
		this.r = radius;
	}

	@Override
	public ShapeType getType() {
		return ShapeType.CIRCLE_SHAPE;
	}

	@Override
	protected BB cacheData(Vector2f pos, Vector2f rot) {
		Vector2f c = tc = cpvadd(pos, cpvrotate(this.c, rot));
		return new BB(c.getX() - r, c.getY() - r, c.getX() + r, c.getY() + r);
	}

	@Override
	public boolean pointQuery(Vector2f p) {
		return cpvnear(tc, p, r);
	}

	static void segmentQueryImpl(Shape shape, Vector2f center, float r, Vector2f a, Vector2f b,
			SegmentQueryInfo info) {
		// offset the line to be relative to the circle
		a = cpvsub(a, center);
		b = cpvsub(b, center);

		float qa = cpvdot(a, a) - 2.0f * cpvdot(a, b) + cpvdot(b, b);
		float qb = -2.0f * cpvdot(a, a) + 2.0f * cpvdot(a, b);
		float qc = cpvdot(a, a) - r * r;

		float det = qb * qb - 4.0f * qa * qc;

		if (det >= 0.0f) {
			float t = (-qb - cpfsqrt(det)) / (2.0f * qa);
			if (0.0f <= t && t <= 1.0f) {
				info.set(shape, t, cpvnormalize(cpvlerp(a, b, t)));
			}
		}
	}

	@Override
	protected void segmentQueryImpl(Vector2f a, Vector2f b, SegmentQueryInfo info) {
		segmentQueryImpl(this, tc, r, a, b, info);
	}

	public float getRadius() {
		return r;
	}

	public void setRadius(float r) {
		this.r = r;
	}

	public Vector2f getOffset() {
		return c;
	}

	public void setOffset(final Vector2f offset) {
		this.c.set(offset);
	}

	public Vector2f getTransformedCenter() {
		return tc;
	}
}
