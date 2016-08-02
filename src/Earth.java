//java extension packages
import javax.vecmath.*;
import javax.media.j3d.*;

//java3D utility packages
import com.sun.j3d.utils.image.*;
import com.sun.j3d.utils.geometry.*;

public class Earth implements Constants
{
  public BranchGroup createScene()
  {
    //load texture for scene object
    TextureLoader textureLoader = new TextureLoader(Earth.class.getResource("images/earth.jpg"), "RGB", null);
    Texture texture = textureLoader.getTexture();

    //create appearance and insert texture into appearance
    Appearance app = new Appearance();
    app.setTexture(texture);

    //create sphere and turn off pickability
    Sphere sphere = new Sphere((float)EARTH_MEAN_RADIUS, Sphere.GENERATE_NORMALS | Sphere.GENERATE_TEXTURE_COORDS, 80, app);
    sphere.setPickable(false);

    //initialize scene and add sphere
    BranchGroup scene = new BranchGroup();
    scene.addChild(sphere);

    //initialize ambient lighting
    BoundingSphere bounds = new BoundingSphere(new Point3d(), Double.MAX_VALUE);
    Light ambientLight = new AmbientLight();
    ambientLight.setInfluencingBounds(bounds);

    //add light node to scene and compile
    scene.addChild(ambientLight);
    scene.compile();

    return scene;
  }
}