package org.physics.jipmunk;

/** @author jobernolte */
public interface Vector2f {
	float getX();

	void setX(float x);

	float getY();

	void setY(float y);

	void set(final Vector2f vector2f);

	void set(float x, float y);
}
