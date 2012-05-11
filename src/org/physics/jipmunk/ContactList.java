package org.physics.jipmunk;

import java.util.ArrayList;

/** @author jobernolte */
public class ContactList extends ArrayList<Contact> {

	private Pool<Contact> contactPool = new Pool<Contact>() {
		@Override
		protected Contact create() {
			return new Contact();
		}
	};

	public Contact nextContactPoint() {
		Contact con = contactPool.alloc();
		add(con);
		return con;
	}

	public void free(Contact contact) {
		contactPool.free(contact);
	}

	@Override
	public void clear() {
		super.clear();
	}

	public void free(Contact[] contacts) {
		if (contacts != null) {
			for (Contact contact : contacts) {
				contactPool.free(contact);
			}
		}
	}
}
