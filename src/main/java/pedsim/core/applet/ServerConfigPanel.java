package pedsim.core.applet;

import java.awt.Button;
import java.awt.Frame;
import java.awt.Label;
import java.awt.TextField;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;

public class ServerConfigPanel extends Frame {
  private static final long serialVersionUID = 1L;

  private TextField sshPathField;
  private TextField keyPathField;
  private TextField serverField;
  private TextField projectDirField;
  private TextField mainClassField;

  public ServerConfigPanel(ServerLauncherApplet serverLauncher) {
    super("Server Configuration");
    setLayout(null);

    Label sshLabel = new Label("SSH path:");
    sshLabel.setBounds(10, 50, 100, 20);
    add(sshLabel);

    sshPathField = new TextField(serverLauncher.getSshPath());
    sshPathField.setBounds(120, 50, 400, 20);
    add(sshPathField);

    Label keyLabel = new Label("Key path:");
    keyLabel.setBounds(10, 90, 100, 20);
    add(keyLabel);

    keyPathField = new TextField(serverLauncher.getKeyPath());
    keyPathField.setBounds(120, 90, 400, 20);
    add(keyPathField);

    Label serverLabel = new Label("Server:");
    serverLabel.setBounds(10, 130, 100, 20);
    add(serverLabel);

    serverField = new TextField(serverLauncher.getServer());
    serverField.setBounds(120, 130, 400, 20);
    add(serverField);

    Label projectDirLabel = new Label("Project dir:");
    projectDirLabel.setBounds(10, 170, 100, 20);
    add(projectDirLabel);

    projectDirField = new TextField(serverLauncher.getProjectDir());
    projectDirField.setBounds(120, 170, 400, 20);
    add(projectDirField);

    Label mainClassLabel = new Label("Main class:");
    mainClassLabel.setBounds(10, 210, 100, 20);
    add(mainClassLabel);

    mainClassField = new TextField(serverLauncher.getMainClass());
    mainClassField.setBounds(120, 210, 400, 20);
    add(mainClassField);

    Button saveButton = new Button("Save");
    saveButton.setBounds(10, 260, 80, 30);
    saveButton.addActionListener(e -> {
      serverLauncher.setSshPath(sshPathField.getText().trim());
      serverLauncher.setKeyPath(keyPathField.getText().trim());
      serverLauncher.setServer(serverField.getText().trim());
      serverLauncher.setProjectDir(projectDirField.getText().trim());
      serverLauncher.setMainClass(mainClassField.getText().trim());
      closePanel();
    });
    add(saveButton);

    setSize(550, 350);
    setVisible(true);

    addWindowListener(new WindowAdapter() {
      @Override
      public void windowClosing(WindowEvent e) {
        closePanel();
      }
    });
  }

  private void closePanel() {
    setVisible(false);
    dispose();
  }
}

