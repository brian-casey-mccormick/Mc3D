//java extension packages
import javax.media.j3d.*;

public class MyShape3D extends Shape3D implements Constants
{
  private ShapeModel shapeModel;
  private Appearance appearance;
  private TransparencyAttributes transparencyAttributes;

  public MyShape3D(ShapeModel shapeModelIn)
  {
    //set shape3D attributes
    setCapability(Shape3D.ALLOW_APPEARANCE_READ);
    setCapability(Shape3D.ALLOW_APPEARANCE_WRITE);
    setCapability(Shape3D.ALLOW_GEOMETRY_WRITE);

    //set appearance attributes
    appearance = new Appearance();
    appearance.setCapability(Appearance.ALLOW_TRANSPARENCY_ATTRIBUTES_READ);
    appearance.setCapability(Appearance.ALLOW_TRANSPARENCY_ATTRIBUTES_WRITE);
    setAppearance(appearance);

    //set transparency attributes
    transparencyAttributes = new TransparencyAttributes();
    transparencyAttributes.setCapability(TransparencyAttributes.ALLOW_MODE_READ);
    transparencyAttributes.setCapability(TransparencyAttributes.ALLOW_MODE_WRITE);
    transparencyAttributes.setCapability(TransparencyAttributes.ALLOW_VALUE_READ);
    transparencyAttributes.setCapability(TransparencyAttributes.ALLOW_VALUE_WRITE);
    appearance.setTransparencyAttributes(transparencyAttributes);

    //set shape model
    shapeModel = new ShapeModel();
    shapeModel.setColor(shapeModelIn.getColor());
    shapeModel.setLabel(shapeModelIn.getLabel());
    shapeModel.setVisibleFlag(shapeModelIn.getVisibleFlag());

    //set transparency based on model data
    if (shapeModel.getVisibleFlag() == true)
    {
      transparencyAttributes.setTransparencyMode(TransparencyAttributes.NONE);
      transparencyAttributes.setTransparency(0.0f);
    }
    else
    {
      transparencyAttributes.setTransparencyMode(TransparencyAttributes.BLENDED);
      transparencyAttributes.setTransparency(1.0f);
    }
  }

  public ShapeModel getShapeModel()
  {
    return shapeModel;
  }
}