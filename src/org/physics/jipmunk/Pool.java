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

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

/** @author jobernolte */
public abstract class Pool<T> {

	protected final LinkedList<T> objects;
	protected int maxCapacity;

	public Pool() {
		this(1000);
	}

	public Pool(int maxCapacity) {
		this.maxCapacity = maxCapacity;
		this.objects = new LinkedList<T>();
	}

	protected abstract T create();

	public T alloc() {
		if (objects.size() == 0) {
			return create();
		} else {
			return objects.removeFirst();
		}
	}

	public void free(T object) {
		if (objects.size() < maxCapacity) {
			objects.add(object);
		}
	}
}
