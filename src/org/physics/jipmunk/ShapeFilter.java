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

/**
 * @author jobernolte
 */
public class ShapeFilter {

	public static final ShapeFilter ALL =
			new ShapeFilter(Constants.NO_GROUP, Constants.ALL_CATEGORIES, Constants.ALL_CATEGORIES);
	public static final ShapeFilter NONE =
			new ShapeFilter(Constants.NO_GROUP, ~Constants.ALL_CATEGORIES, ~Constants.ALL_CATEGORIES);
	private final int group;
	private final int categories;
	private final int mask;

	public ShapeFilter(int group, int categories, int mask) {
		this.group = group;
		this.categories = categories;
		this.mask = mask;
	}

	public int getGroup() {
		return group;
	}

	public int getCategories() {
		return categories;
	}

	public int getMask() {
		return mask;
	}

	public boolean reject(ShapeFilter b) {
		return reject(this, b);
	}

	public static boolean reject(ShapeFilter a, ShapeFilter b) {
		// Reject the collision if:
		return (
				// They are in the same non-zero group.
				(a.group != 0 && a.group == b.group) ||
						// One of the category/mask combinations fails.
						(a.categories & b.mask) == 0 ||
						(b.categories & a.mask) == 0);
	}
}
