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

/**
 * Column major affine transform.
 *
 * @author jobernolte
 */
public class Transform {
	float a, b, c, d, tx, ty;

	public static Transform identity() {
		return new Transform(1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f);
	}

	public Transform() {
	}

	public Transform(float a, float b, float c, float d, float tx, float ty) {
		this.a = a;
		this.b = b;
		this.c = c;
		this.d = d;
		this.tx = tx;
		this.ty = ty;
	}

	/// Construct a new transform matrix in transposed order.
	public static Transform transpose(float a, float c, float tx, float b, float d, float ty) {
		return new Transform(a, b, c, d, tx, ty);
	}

	/// Get the inverse of a transform matrix.
	public static Transform inverse(Transform t) {
		float inv_det = 1.0f / (t.a * t.d - t.c * t.b);
		return transpose(t.d * inv_det, -t.c * inv_det, (t.c * t.ty - t.tx * t.d) * inv_det, -t.b * inv_det,
						 t.a * inv_det, (t.tx * t.b - t.a * t.ty) * inv_det);
	}

	/// Multiply two transformation matrices.
	public static Transform mult(Transform t1, Transform t2) {
		return transpose(t1.a * t2.a + t1.c * t2.b, t1.a * t2.c + t1.c * t2.d, t1.a * t2.tx + t1.c * t2.ty + t1.tx,
						 t1.b * t2.a + t1.d * t2.b, t1.b * t2.c + t1.d * t2.d, t1.b * t2.tx + t1.d * t2.ty + t1.ty);
	}

	/// Transform an absolute point. (i.e. a vertex)
	public static Vector2f transformPoint(Transform t, Vector2f p) {
		return t.transformPoint(p);
	}

	public Vector2f transformPoint(Vector2f p) {
		return new Vector2f(this.a * p.x + this.c * p.y + this.tx, this.b * p.x + this.d * p.y + this.ty);
	}

	/// Transform a vector (i.e. a normal)
	public static Vector2f transformVect(Transform t, Vector2f v) {
		return new Vector2f(t.a * v.x + t.c * v.y, t.b * v.x + t.d * v.y);
	}

	public Vector2f transformVect(Vector2f v) {
		return new Vector2f(this.a * v.x + this.c * v.y, this.b * v.x + this.d * v.y);
	}

	/// Transform a cpBB.
	public static BB transformbBB(Transform t, BB bb) {
		Vector2f center = bb.getCenter();
		float hw = (bb.r - bb.l) * 0.5f;
		float hh = (bb.t - bb.b) * 0.5f;

		float a = t.a * hw, b = t.c * hh, d = t.b * hw, e = t.d * hh;
		float hw_max = cpfmax(cpfabs(a + b), cpfabs(a - b));
		float hh_max = cpfmax(cpfabs(d + e), cpfabs(d - e));
		return BB.forExtents(t.transformPoint(center), hw_max, hh_max);
	}

	/// Create a transation matrix.
	public static Transform translate(Vector2f translate) {
		return transpose(1.0f, 0.0f, translate.x, 0.0f, 1.0f, translate.y);
	}

	/// Create a scale matrix.
	public static Transform scale(float scaleX, float scaleY) {
		return transpose(scaleX, 0.0f, 0.0f, 0.0f, scaleY, 0.0f);
	}

	/// Create a rotation matrix.
	public static Transform rotate(float radians) {
		Vector2f rot = cpvforangle(radians);
		return transpose(rot.x, -rot.y, 0.0f, rot.y, rot.x, 0.0f);
	}

	/// Create a rigid transformation matrix. (transation + rotation)
	public static Transform rigid(Vector2f translate, float radians) {
		Vector2f rot = cpvforangle(radians);
		return transpose(rot.x, -rot.y, translate.x, rot.y, rot.x, translate.y);
	}

	/// Fast inverse of a rigid transformation matrix.
	public static Transform rigidInverse(Transform t) {
		return transpose(t.d, -t.c, (t.c * t.ty - t.tx * t.d), -t.b, t.a, (t.tx * t.b - t.a * t.ty));
	}

	// Miscelaneous (but useful) transformation matrices.
	public static Transform wrap(Transform outer, Transform inner) {
		return mult(inverse(outer), mult(inner, outer));
	}

	public static Transform wrapInverse(Transform outer, Transform inner) {
		return mult(outer, mult(inner, inverse(outer)));
	}

	public static Transform ortho(BB bb) {
		return transpose(2.0f / (bb.r - bb.l), 0.0f, -(bb.r + bb.l) / (bb.r - bb.l), 0.0f, 2.0f / (bb.t - bb.b),
						 -(bb.t + bb.b) / (bb.t - bb.b));
	}

	public static Transform boneScale(Vector2f v0, Vector2f v1) {
		Vector2f d = cpvsub(v1, v0);
		return transpose(d.x, -d.y, v0.x, d.y, d.x, v0.y);
	}

	public static Transform axialScale(Vector2f axis, Vector2f pivot, float scale) {
		float A = axis.x * axis.y * (scale - 1.0f);
		float B = cpvdot(axis, pivot) * (1.0f - scale);

		return transpose(scale * axis.x * axis.x + axis.y * axis.y, A, axis.x * B, A,
						 axis.x * axis.x + scale * axis.y * axis.y, axis.y * B);
	}

	@Override
	public String toString() {
		return "Transform{" +
				"a=" + a +
				", b=" + b +
				", c=" + c +
				", d=" + d +
				", tx=" + tx +
				", ty=" + ty +
				'}';
	}
}
