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

/** @author jobernolte */
public class Contact {
	private Vector2f r1 = new Vector2f(), r2 = new Vector2f();
	private float nMass, tMass;
	private float bounce; // TODO: look for an alternate bounce solution.
	private float jnAcc, jtAcc, jBias;
	private float bias;
	private int hash;

	public Contact() {
	}

	public Contact(Vector2f r1, Vector2f r2, int hash) {
		this.r1.set(r1);
		this.r2.set(r2);
		this.hash = hash;
	}

	public Vector2f getR1() {
		return r1;
	}

	public void setR1(Vector2f r1) {
		this.r1.set(r1);
	}

	public Vector2f getR2() {
		return r2;
	}

	public void setR2(Vector2f r2) {
		this.r2.set(r2);
	}

	public float getnMass() {
		return nMass;
	}

	public void setnMass(float nMass) {
		this.nMass = nMass;
	}

	public float gettMass() {
		return tMass;
	}

	public void settMass(float tMass) {
		this.tMass = tMass;
	}

	public float getBounce() {
		return bounce;
	}

	public void setBounce(float bounce) {
		this.bounce = bounce;
	}

	public float getJnAcc() {
		return jnAcc;
	}

	public void setJnAcc(float jnAcc) {
		this.jnAcc = jnAcc;
	}

	public float getJtAcc() {
		return jtAcc;
	}

	public void setJtAcc(float jtAcc) {
		this.jtAcc = jtAcc;
	}

	public float getjBias() {
		return jBias;
	}

	public void setjBias(float jBias) {
		this.jBias = jBias;
	}

	public float getBias() {
		return bias;
	}

	public void setBias(float bias) {
		this.bias = bias;
	}

	public int getHash() {
		return hash;
	}

	public void setHash(int hash) {
		this.hash = hash;
	}

	@Override
	public String toString() {
		return "Contact{" +
				"r1=" + r1 +
				", r2=" + r2 +
				", hash=" + hash +
				'}';
	}
}
