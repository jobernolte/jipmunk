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

/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.physics.jipmunk;

/** @author jobernolte */
public class DefaultVector2f implements Vector2f {

	private float x;
	private float y;

	public DefaultVector2f() {
	}

	public DefaultVector2f(float x, float y) {
		this.x = x;
		this.y = y;
	}

	public DefaultVector2f(final Vector2f vector2f) {
		this.x = vector2f.getX();
		this.y = vector2f.getY();
	}

	@Override
	public float getX() {
		return x;
	}

	@Override
	public void setX(float x) {
		this.x = x;
	}

	@Override
	public float getY() {
		return y;
	}

	@Override
	public void setY(float y) {
		this.y = y;
	}

	@Override
	public void set(Vector2f vector2f) {
		this.x = vector2f.getX();
		this.y = vector2f.getY();
	}

	@Override
	public void set(float x, float y) {
		this.x = x;
		this.y = y;
	}

	public DefaultVector2f sub(final Vector2f vector2f) {
		this.x -= vector2f.getX();
		this.y -= vector2f.getY();
		return this;
	}

	public DefaultVector2f add(final Vector2f vector2f) {
		this.x += vector2f.getX();
		this.y += vector2f.getY();
		return this;
	}

	public DefaultVector2f mult(float s) {
		this.x *= s;
		this.y *= s;
		return this;
	}

	@Override
	public boolean equals(Object o) {
		if (this == o) return true;
		if (!(o instanceof DefaultVector2f)) return false;

		DefaultVector2f vector2f = (DefaultVector2f) o;

		if (Float.compare(vector2f.x, x) != 0) return false;
		if (Float.compare(vector2f.y, y) != 0) return false;

		return true;
	}

	@Override
	public int hashCode() {
		int result = (x != +0.0f ? Float.floatToIntBits(x) : 0);
		result = 31 * result + (y != +0.0f ? Float.floatToIntBits(y) : 0);
		return result;
	}

	@Override
	public String toString() {
		return "DefaultVector2f{" +
				"x=" + x +
				", y=" + y +
				'}';
	}
}
