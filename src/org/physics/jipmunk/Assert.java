package org.physics.jipmunk;

import java.util.logging.Logger;

/** @author jobernolte */
class Assert {
	final static Logger logger = Logger.getLogger(Assert.class.getName());

	public static void cpAssertSoft(boolean condition, String message) {
		if (!condition) {
			logger.info("failed assertion: " + message);
		}
		assert condition : message;
	}

	public static void cpAssertWarn(boolean condition, String message) {
		assert condition : message;
	}

	public static void cpAssertHard(boolean condition, String message) {
		assert condition : message;
	}

	public static void cpAssertSpaceUnlocked(Space space) {
		assert space.locked == 0;
	}
}
