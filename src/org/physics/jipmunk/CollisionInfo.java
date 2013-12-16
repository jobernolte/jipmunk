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

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

/**
 * @author jobernolte
 */
public class CollisionInfo {
	private Shape a, b;
	private CollisionID id;
	private Vector2f n = new Vector2f();
	private List<Contact> contacts = null;

	public CollisionInfo(Shape a, Shape b, CollisionID id, Vector2f n) {
		this.a = a;
		this.b = b;
		this.id = id;
		this.n.set(n);
	}

	public Shape getA() {
		return a;
	}

	public void setA(Shape a) {
		this.a = a;
	}

	public Shape getB() {
		return b;
	}

	public void setB(Shape b) {
		this.b = b;
	}

	public CollisionID getId() {
		return id;
	}

	public void setId(CollisionID id) {
		this.id = id;
	}

	public Vector2f getN() {
		return n;
	}

	public void setN(Vector2f n) {
		this.n.set(n);
	}

	public Contact addContact(Vector2f p1, Vector2f p2, int hash) {
		Contact con = new Contact(p1, p2, hash);
		if (contacts == null) {
			contacts = new LinkedList<>();
		}
		contacts.add(con);
		return con;
	}

	public boolean isEmpty() {
		return contacts == null || contacts.isEmpty();
	}

	public List<Contact> getContacts() {
		return contacts;
	}
}
