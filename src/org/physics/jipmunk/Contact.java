package org.physics.jipmunk;

/** @author jobernolte */
class Contact {
	Vector2f p, n;
	float dist;

	Vector2f r1, r2;
	float nMass, tMass, bounce;

	float jnAcc, jtAcc, jBias;
	float bias;

	long hash;

	Contact init(Vector2f p, Vector2f n, float dist, long hash) {
		this.p = p;
		this.n = n;
		this.dist = dist;

		this.jnAcc = 0.0f;
		this.jtAcc = 0.0f;
		this.jBias = 0.0f;

		this.hash = hash;

		return this;
	}

	static void cpContactInit(Contact con, Vector2f p, Vector2f n, float dist, long hash) {
		con.init(p, n, dist, hash);
	}
}
