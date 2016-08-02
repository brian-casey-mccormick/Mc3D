//java core package
import java.awt.*;

//java extension packages
import javax.media.j3d.*;

public class ShapeModel
{
  private Color color;
  private String label;
  private boolean visible;

  public ShapeModel()
  {
    setColor(Color.white);
    setLabel("None");
    setVisibleFlag(true);
  }

  public ShapeModel(Color colorIn, String labelIn, boolean visibleIn)
  {
    setColor(colorIn);
    setLabel(labelIn);
    setVisibleFlag(visibleIn);
  }

  public void setColor(Color colorIn)
  {
    color = colorIn;
  }

  public void setVisibleFlag(boolean visibleIn)
  {
    visible = visibleIn;
  }

  public void setLabel(String labelIn)
  {
    label = labelIn;
  }

  public Color getColor()
  {
    return color;
  }

  public boolean getVisibleFlag()
  {
    return visible;
  }

  public String getLabel()
  {
    return label;
  }
}