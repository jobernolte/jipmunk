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

import org.physics.jipmunk.Vector2f;

import static org.physics.jipmunk.Util.cpv;

/**
 * @author jobernolte
 */
public class EdgePoint {
	public EdgePoint(Vector2f p, int hash) {
		this.p = new Vector2f(p);
		this.hash = hash;
	}

	private final Vector2f p;
	/** Keep a hash value for Chipmunk's collision hashing mechanism. */
	private final int hash;

	public Vector2f getP() {
		return p;
	}

	public int getHash() {
		return hash;
	}

	@Override
	public String toString() {
		return "EdgePoint{" +
				"p=" + p +
				", hash=" + hash +
				'}';
	}
}
