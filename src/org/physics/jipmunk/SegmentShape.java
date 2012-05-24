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

import static org.physics.jipmunk.Util.cpBBNew;
import static org.physics.jipmunk.Util.cpfabs;
import static org.physics.jipmunk.Util.cpvadd;
import static org.physics.jipmunk.Util.cpvcross;
import static org.physics.jipmunk.Util.cpvdot;
import static org.physics.jipmunk.Util.cpvlengthsq;
import static org.physics.jipmunk.Util.cpvlerp;
import static org.physics.jipmunk.Util.cpvneg;
import static org.physics.jipmunk.Util.cpvnormalize;
import static org.physics.jipmunk.Util.cpvperp;
import static org.physics.jipmunk.Util.cpvrotate;
import static org.physics.jipmunk.Util.cpvsub;

/** @author jobernolte */
public class SegmentShape extends Shape {

	private Vector2f a, b, n;
	Vector2f ta;
	Vector2f tb;
	Vector2f tn;
	float r;
	Vector2f a_tangent, b_tangent;

	public SegmentShape(Body body, Vector2f a, Vector2f b, float r) {
		super(body);
		this.a = Util.cpv(a);
		this.b = Util.cpv(b);
		this.r = r;
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
		return r;
	}

	public Vector2f getTa() {
		return ta;
	}

	public Vector2f getTb() {
		return tb;
	}

	@Override
	public ShapeType getType() {
		return ShapeType.SEGMENT_SHAPE;
	}

	@Override
	protected BB cacheData(Vector2f p, Vector2f rot) {
		this.ta = cpvadd(p, cpvrotate(this.a, rot));
		this.tb = cpvadd(p, cpvrotate(this.b, rot));
		this.tn = cpvrotate(this.n, rot);

		float l, r, b, t;

		if (this.ta.getX() < this.tb.getX()) {
			l = this.ta.getX();
			r = this.tb.getX();
		} else {
			l = this.tb.getX();
			r = this.ta.getX();
		}

		if (this.ta.getY() < this.tb.getY()) {
			b = this.ta.getY();
			t = this.tb.getY();
		} else {
			b = this.tb.getY();
			t = this.ta.getY();
		}

		float rad = this.r;
		return cpBBNew(l - rad, b - rad, r + rad, t + rad);
	}

	@Override
	public boolean pointQuery(Vector2f p) {
		if (!this.bb.contains(p)) return false;

		// Calculate normal distance from segment.
		float dn = cpvdot(this.tn, p) - cpvdot(this.ta, this.tn);
		float dist = cpfabs(dn) - this.r;
		if (dist > 0.0f) return false;

		// Calculate tangential distance along segment.
		float dt = -cpvcross(this.tn, p);
		float dtMin = -cpvcross(this.tn, this.ta);
		float dtMax = -cpvcross(this.tn, this.tb);

		// Decision tree to decide which feature of the segment to collide with.
		if (dt <= dtMin) {
			if (dt < (dtMin - this.r)) {
				return false;
			} else {
				return cpvlengthsq(cpvsub(this.ta, p)) < (this.r * this.r);
			}
		} else {
			if (dt < dtMax) {
				return true;
			} else {
				if (dt < (dtMax + this.r)) {
					return cpvlengthsq(cpvsub(this.tb, p)) < (this.r * this.r);
				} else {
					return false;
				}
			}
		}
	}

	static boolean inUnitRange(float t) {
		return (0.0f < t && t < 1.0f);
	}

	@Override
	protected void segmentQueryImpl(Vector2f a, Vector2f b, SegmentQueryInfo info) {
		Vector2f n = this.tn;
		// flip n if a is behind the axis
		if (cpvdot(a, n) < cpvdot(this.ta, n))
			n = cpvneg(n);

		float an = cpvdot(a, n);
		float bn = cpvdot(b, n);

		if (an != bn) {
			float d = cpvdot(this.ta, n) + this.r;
			float t = (d - an) / (bn - an);

			if (0.0f < t && t < 1.0f) {
				Vector2f point = cpvlerp(a, b, t);
				float dt = -cpvcross(this.tn, point);
				float dtMin = -cpvcross(this.tn, this.ta);
				float dtMax = -cpvcross(this.tn, this.tb);

				if (dtMin < dt && dt < dtMax) {
					info.set(this, t, n);

					return; // don't continue on and check endcaps
				}
			}
		}

		if (this.r != 0) {
			SegmentQueryInfo info1 = new SegmentQueryInfo();
			SegmentQueryInfo info2 = new SegmentQueryInfo();
			CircleShape.segmentQueryImpl(this, this.ta, this.r, a, b, info1);
			CircleShape.segmentQueryImpl(this, this.tb, this.r, a, b, info2);

			if (info1.t < info2.t) {
				info.set(info1.shape, info1.t, info1.n);
			} else {
				info.set(info2.shape, info2.t, info2.n);
			}
		}
	}

	public void setEndpoints(final Vector2f a, final Vector2f b) {
		this.a = Util.cpv(a);
		this.b = Util.cpv(b);
		this.n = cpvperp(cpvnormalize(cpvsub(b, a)));
	}

	public void setRadius(float radius) {
		this.r = radius;
	}

	public void setNeighbors(final Vector2f prev, final Vector2f next) {
		this.a_tangent = cpvsub(prev, this.a);
		this.b_tangent = cpvsub(next, this.b);
	}
}
