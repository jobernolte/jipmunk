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

/** @author jobernolte */
class Contact {
	Vector2f p = Util.cpvzero(), n = Util.cpvzero();
	float dist;

	Vector2f r1 = Util.cpvzero(), r2 = Util.cpvzero();
	float nMass, tMass, bounce;

	float jnAcc, jtAcc, jBias;
	float bias;

	long hash;

	Contact init(Vector2f p, Vector2f n, float dist, long hash) {
		this.p.set(p);
		this.n.set(n);
		this.dist = dist;

		this.jnAcc = 0.0f;
		this.jtAcc = 0.0f;
		this.jBias = 0.0f;

		this.hash = hash;

		return this;
	}

	static void cpContactInit(Contact con, Vector2f p, Vector2f n, float dist, long hash) {
		con.init(p, n, dist, hash);
	}

	public Vector2f getPoint() {
		return p;
	}

	public Vector2f getNormal() {
		return n;
	}

	public float getDistance() {
		return dist;
	}
}
