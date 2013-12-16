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

import org.physics.jipmunk.impl.Contact;

import java.util.Iterator;
import java.util.List;

import static org.physics.jipmunk.Util.cpvdot;
import static org.physics.jipmunk.Util.cpvneg;
import static org.physics.jipmunk.Util.cpvsub;

/** @author jobernolte */
public class ContactPointSet implements Iterable<ContactPoint> {

	/** The normal of the collision. */
	private final Vector2f normal;
	private final ContactPoint[] points;

	public ContactPointSet(Vector2f normal, ContactPoint[] points) {
		this.normal = normal;
		this.points = points;
	}

	ContactPointSet(CollisionInfo info, boolean swapped) {
		this(info.getN(), info.getContacts(), swapped);
	}

	ContactPointSet(Vector2f normal, List<Contact> contacts, boolean swapped) {
		// cpCollideShapes() may have swapped the contact order. Flip the normal.
		this.normal = (swapped ? cpvneg(normal) : normal);
		this.points = new ContactPoint[contacts != null ? contacts.size() : 0];

		if (contacts != null) {
			int i = 0;
			for (Contact contact : contacts) {
				// cpCollideShapesInfo() returns contacts with absolute positions.
				Vector2f p1 = contact.getR1();
				Vector2f p2 = contact.getR2();

				this.points[i].point1 = (swapped ? p2 : p1);
				this.points[i].point2 = (swapped ? p1 : p2);
				this.points[i].distance = cpvdot(cpvsub(p2, p1), this.normal);
				i++;
			}
		}
	}

	public Vector2f getNormal() {
		return normal;
	}

	public ContactPoint[] getPoints() {
		return points;
	}

	public int getCount() {
		return points != null ? points.length : 0;
	}

	@Override
	public Iterator<ContactPoint> iterator() {
		return new Iterator<ContactPoint>() {
			int index;

			@Override
			public boolean hasNext() {
				return points != null && index < points.length;
			}

			@Override
			public ContactPoint next() {
				return points[index++];
			}
		};
	}
}
