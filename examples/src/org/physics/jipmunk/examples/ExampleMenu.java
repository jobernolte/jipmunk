package org.physics.jipmunk.examples;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import javax.swing.*;

/** @author chris_c based on work by jobernolte & Lembcke */
public class ExampleMenu extends JFrame implements ActionListener {

	private JComboBox combo;
	private JButton goButton;
	private JPanel jp;

	// list of class names to run...
	static String examples[] = {
			"BouncyHexagons",
			"Convex",
			"LogoSmash",
			"Plink",
			"PyramidStack",
			"Pump"
	};

	public ExampleMenu() {
		setTitle("Examples Menu");
		setSize(300, 200);
		setLocationRelativeTo(null);
		setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);

		combo = new JComboBox(examples);
		goButton = new JButton("Go !");
		goButton.addActionListener(this);
		jp = new JPanel();
		jp.add(combo);
		jp.add(goButton);
		add(jp);
		pack();
	}

	public void actionPerformed(ActionEvent e) {
		String className = "org.physics.jipmunk.examples." + combo.getSelectedItem();
		try {
			ExampleBase example = (ExampleBase) Class.forName(className).newInstance();
			example.start(640, 480);
			hide();
		} catch (Exception ex) {
			ex.printStackTrace();
		}

	}

	public static void main(String[] args) {
		SwingUtilities.invokeLater(new Runnable() {
			public void run() {
				ExampleMenu ex = new ExampleMenu();
				ex.setVisible(true);
			}
		});
	}
}