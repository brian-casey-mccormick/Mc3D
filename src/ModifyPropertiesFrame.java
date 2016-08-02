//java core packages
import java.awt.*;

//java extension packages
import javax.swing.*;
import javax.media.j3d.*;

public class ModifyPropertiesFrame extends JFrame
{
  public ModifyPropertiesFrame(Shape3D shape3D)
  {
    //create appearance controller
    super("Appearance");
    AppearancePanel appearancePanel = new AppearancePanel(shape3D);

    //add appearance controller to content pane
    Container contentPane = getContentPane();
    contentPane.add(appearancePanel);

    setSize(350, 200);
    setVisible(true);
  }
}