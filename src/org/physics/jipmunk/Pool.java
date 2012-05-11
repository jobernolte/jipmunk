package org.physics.jipmunk;

import java.util.ArrayList;
import java.util.List;

/** @author jobernolte */
public abstract class Pool<T> {

	private final List<T> objects;
	private int maxCapacity;

	public Pool() {
		this(1000);
	}

	public Pool(int maxCapacity) {
		this.maxCapacity = maxCapacity;
		this.objects = new ArrayList<T>(maxCapacity);
	}

	protected abstract T create();

	public T alloc() {
		if (objects.size() == 0) {
			return create();
		} else {
			return objects.remove(objects.size() - 1);
		}
	}

	public void free(T object) {
		if (objects.size() < maxCapacity) {
			objects.add(object);
		}
	}
}
