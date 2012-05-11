package org.physics.jipmunk;

import java.util.ArrayList;

/** @author jobernolte */
public class ContactPointSet extends ArrayList<ContactPoint> {
	public ContactPointSet() {
	}

	public ContactPointSet(int i) {
		super(i);
	}

	public void add(Vector2f point, Vector2f normal, float dist) {
		add(new ContactPoint(point, normal, dist));
	}
}
