//java core packages
import java.awt.event.*;
import java.text.DecimalFormat;

//java extension packages
import javax.swing.*;
import javax.swing.border.*;
import javax.media.j3d.*;

public class AppearancePanel extends JPanel
{
  private Shape3D shape3D;
  private JTextField jTextField;
  private JComboBox transparencyModeTypesComboBox;
  private String transparencyModeTypes[] = {"NONE", "BLENDED", "FASTEST", "NICEST", "SCREEN_DOOR"};
  private JLabel jLabel1;
  private JLabel jLabel2;
  private DecimalFormat twoDigits;

  public AppearancePanel(Shape3D shape3DIn)
  {
    shape3D = shape3DIn;

    //set border to the name of the shape
    if (shape3D instanceof MyShape3D)
    {
      MyShape3D myShape3D = (MyShape3D)shape3DIn;
      TitledBorder titledBorder = new TitledBorder(myShape3D.getShapeModel().getLabel());
      titledBorder.setTitleJustification(TitledBorder.CENTER);
      setBorder(titledBorder);
    }

    //add transparency value JLabel1
    twoDigits = new DecimalFormat("0.00");
    float transparencyValue = shape3D.getAppearance().getTransparencyAttributes().getTransparency();
    String transparencyValueString = twoDigits.format(transparencyValue);
    jLabel1 = new JLabel("Transparency Value: " + transparencyValueString);
    add(jLabel1);

    //add transparency value JTextField1
    jTextField = new JTextField(transparencyValueString, 3);
    add(jTextField);

    //add transparency mode type JLabel2
    int transparencyMode = shape3D.getAppearance().getTransparencyAttributes().getTransparencyMode();
    int transparencyModeIndex = getTransparencyModeIndex(transparencyMode);
    String transparencyModeString = transparencyModeTypes[transparencyModeIndex];
    jLabel2 = new JLabel("Transparency Mode: " + transparencyModeString);
    add(jLabel2);

    //add transparency mode type JComboBox
    transparencyModeTypesComboBox = new JComboBox(transparencyModeTypes);
    transparencyModeTypesComboBox.setMaximumRowCount(3);
    add(transparencyModeTypesComboBox);

    //add update JButton
    JButton updateButton = new JButton("Update");
    add(updateButton);

    updateButton.addActionListener(new ActionListener()
    {
      public void actionPerformed(ActionEvent event)
      {
        try
        {
          //set new value for transparency
          String newTransparencyValueString = jTextField.getText();
          float newTransparency = Float.parseFloat(newTransparencyValueString);
          shape3D.getAppearance().getTransparencyAttributes().setTransparency(newTransparency);

          //update jLabel1
          jLabel1.setText("Transparency Value: " + twoDigits.format(newTransparency));

          //set new value for type
          int selectedIndex = transparencyModeTypesComboBox.getSelectedIndex();
          int newTransparencyMode = getTransparencyModeInteger(selectedIndex);
          shape3D.getAppearance().getTransparencyAttributes().setTransparencyMode(newTransparencyMode);

          //update jLabel2
          int transparencyModeIndex = getTransparencyModeIndex(newTransparencyMode);
          String transparencyModeString = transparencyModeTypes[transparencyModeIndex];
          jLabel2.setText("Transparency Mode: " + transparencyModeString);
        }
        catch (NumberFormatException exception)
        {
          JOptionPane.showMessageDialog(AppearancePanel.this,
                                        "Please enter valid data",
                                        "Error", JOptionPane.ERROR_MESSAGE);
        }
      }
    }
    );
  }

  private int getTransparencyModeIndex(int transparencyMode)
  {
    if (transparencyMode == TransparencyAttributes.NONE)
    {
      return 0;
    }
    else if (transparencyMode == TransparencyAttributes.BLENDED)
    {
      return 1;
    }
    else if (transparencyMode == TransparencyAttributes.FASTEST)
    {
      return 2;
    }
    else if (transparencyMode == TransparencyAttributes.NICEST)
    {
      return 3;
    }
    else
    {
      return 4;
    }
  }

  private int getTransparencyModeInteger(int selectedIndex)
  {
    if (transparencyModeTypes[selectedIndex].equals("NONE"))
    {
      return TransparencyAttributes.NONE;
    }
    else if (transparencyModeTypes[selectedIndex].equals("BLENDED"))
    {
      return TransparencyAttributes.BLENDED;
    }
    else if (transparencyModeTypes[selectedIndex].equals("FASTEST"))
    {
      return TransparencyAttributes.FASTEST;
    }
    else if (transparencyModeTypes[selectedIndex].equals("NICEST"))
    {
      return TransparencyAttributes.NICEST;
    }
    else
    {
      return TransparencyAttributes.SCREEN_DOOR;
    }
  }
}