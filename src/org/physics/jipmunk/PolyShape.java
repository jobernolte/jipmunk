package org.physics.jipmunk;

import static org.physics.jipmunk.Util.cpBBContainsVect;
import static org.physics.jipmunk.Util.cpBBNew;
import static org.physics.jipmunk.Util.cpfmax;
import static org.physics.jipmunk.Util.cpfmin;
import static org.physics.jipmunk.Util.cpv;
import static org.physics.jipmunk.Util.cpvadd;
import static org.physics.jipmunk.Util.cpvcross;
import static org.physics.jipmunk.Util.cpvdot;
import static org.physics.jipmunk.Util.cpvlerp;
import static org.physics.jipmunk.Util.cpvnormalize;
import static org.physics.jipmunk.Util.cpvperp;
import static org.physics.jipmunk.Util.cpvrotate;
import static org.physics.jipmunk.Util.cpvsub;
import static org.physics.jipmunk.Util.cpvzero;

/** @author jobernolte */
public class PolyShape extends Shape {

	Vector2f[] verts;
	Vector2f[] tVerts;
	PolyShapeAxis[] axes;
	PolyShapeAxis[] tAxes;

	public PolyShape(Body body, Vector2f[] verts, Vector2f offset) {
		super(body);
		setUpVerts(verts, 0, verts.length, offset);
	}

	public PolyShape(Body body, Vector2f offset, Vector2f... verts) {
		super(body);
		setUpVerts(verts, 0, verts.length, offset);
	}

	public PolyShape(Body body, Vector2f[] verts, int vertOffset, int vertLength, Vector2f offset) {
		super(body);
		setUpVerts(verts, vertOffset, vertLength, offset);
	}

	void setUpVerts(Vector2f[] verts, int vertOffset, int vertLength, Vector2f offset) {
		if (!validate(verts, vertOffset, vertLength)) {
			throw new IllegalArgumentException("Polygon is concave or has a reversed winding.");
		}
		this.verts = new Vector2f[vertLength];
		this.tVerts = new Vector2f[vertLength];
		this.axes = new PolyShapeAxis[vertLength];
		this.tAxes = new PolyShapeAxis[vertLength];

		for (int i = 0; i < vertLength; i++) {
			Vector2f a = cpvadd(offset, verts[vertOffset + i]);
			Vector2f b = cpvadd(offset, verts[vertOffset + ((i + 1) % vertLength)]);
			Vector2f n = cpvnormalize(cpvperp(cpvsub(b, a)));

			this.verts[i] = a;
			this.axes[i] = new PolyShapeAxis();
			this.axes[i].n = n;
			this.axes[i].d = cpvdot(n, a);
			this.tAxes[i] = new PolyShapeAxis();
		}
	}

	@Override
	public ShapeType getType() {
		return ShapeType.POLY_SHAPE;
	}

	@Override
	protected BB cacheData(Vector2f pos, Vector2f rot) {
		transformAxes(pos, rot);
		return transformVerts(pos, rot);
	}

	@Override
	public boolean pointQuery(Vector2f p) {
		return cpBBContainsVect(this.bb, p) && cpPolyShapeContainsVert(this, p);
	}

	@Override
	protected void segmentQueryImpl(Vector2f a, Vector2f b, SegmentQueryInfo info) {
		PolyShapeAxis[] axes = this.tAxes;
		Vector2f[] verts = this.tVerts;

		for (int i = 0; i < verts.length; i++) {
			Vector2f n = axes[i].n;
			float an = cpvdot(a, n);
			if (axes[i].d > an) continue;

			float bn = cpvdot(b, n);
			float t = (axes[i].d - an) / (bn - an);
			if (t < 0.0f || 1.0f < t) continue;

			Vector2f point = cpvlerp(a, b, t);
			float dt = -cpvcross(n, point);
			float dtMin = -cpvcross(n, verts[i]);
			float dtMax = -cpvcross(n, verts[(i + 1) % verts.length]);

			if (dtMin <= dt && dt <= dtMax) {
				info.set(this, t, n);
			}
		}
	}

	static float cpPolyShapeValueOnAxis(final PolyShape poly, final Vector2f n, final float d) {
		Vector2f[] verts = poly.tVerts;
		float min = cpvdot(n, verts[0]);

		for (int i = 1; i < verts.length; i++) {
			min = cpfmin(min, cpvdot(n, verts[i]));
		}

		return min - d;
	}

	static boolean cpPolyShapeContainsVert(final PolyShape poly, final Vector2f v) {
		PolyShapeAxis[] axes = poly.tAxes;

		for (int i = 0; i < axes.length; i++) {
			float dist = cpvdot(axes[i].n, v) - axes[i].d;
			if (dist > 0.0f) return false;
		}

		return true;
	}

	static boolean cpPolyShapeContainsVertPartial(final PolyShape poly, final Vector2f v, final Vector2f n) {
		PolyShapeAxis[] axes = poly.tAxes;

		for (int i = 0; i < axes.length; i++) {
			if (cpvdot(axes[i].n, n) < 0.0f) continue;
			float dist = cpvdot(axes[i].n, v) - axes[i].d;
			if (dist > 0.0f) return false;
		}

		return true;
	}

	BB transformVerts(Vector2f p, Vector2f rot) {
		Vector2f[] src = this.verts;

		float l = Float.POSITIVE_INFINITY, r = Float.NEGATIVE_INFINITY;
		float b = Float.POSITIVE_INFINITY, t = Float.NEGATIVE_INFINITY;

		for (int i = 0; i < src.length; i++) {
			Vector2f v = cpvadd(p, cpvrotate(src[i], rot));

			tVerts[i] = v;
			l = cpfmin(l, v.getX());
			r = cpfmax(r, v.getX());
			b = cpfmin(b, v.getY());
			t = cpfmax(t, v.getY());
		}

		return cpBBNew(l, b, r, t);
	}

	void transformAxes(Vector2f p, Vector2f rot) {
		PolyShapeAxis[] src = this.axes;
		PolyShapeAxis[] dst = this.tAxes;

		for (int i = 0; i < src.length; i++) {
			Vector2f n = cpvrotate(src[i].n, rot);
			dst[i].n = n;
			dst[i].d = cpvdot(p, n) + src[i].d;
		}
	}

	static boolean validate(final Vector2f[] verts, int offset, int length) {
		for (int i = 0; i < length; i++) {
			Vector2f a = verts[offset + i];
			Vector2f b = verts[offset + ((i + 1) % length)];
			Vector2f c = verts[offset + ((i + 2) % length)];

			if (cpvcross(cpvsub(b, a), cpvsub(c, b)) > 0.0f) {
				return false;
			}
		}
		return true;
	}

	static public PolyShape createBox(Body body, float width, float height) {
		float hw = width / 2.0f;
		float hh = height / 2.0f;

		Vector2f verts[] = {
				cpv(-hw, -hh),
				cpv(-hw, hh),
				cpv(hw, hh),
				cpv(hw, -hh)
		};
		return new PolyShape(body, verts, cpvzero());
	}

}
