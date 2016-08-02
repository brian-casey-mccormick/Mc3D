//java extension packages
import javax.vecmath.*;

public class StateVector
{
  public Vector3d posECEF;
  public Vector3d velECEF;

  public StateVector()
  {

  }

  public StateVector(StateVector stateVectorIn)
  {
    posECEF = stateVectorIn.posECEF;
    velECEF = stateVectorIn.velECEF;
  }

  public StateVector(Vector3d posECEFIn, Vector3d velECEFIn)
  {
    posECEF = posECEFIn;
    velECEF = velECEFIn;
  }

  public void setStateVector(StateVector stateVectorIn)
  {
    posECEF = stateVectorIn.posECEF;
    velECEF = stateVectorIn.velECEF;
  }

  public void setPosECEF(Vector3d posECEFIn)
  {
    posECEF = posECEFIn;
  }

  public void setVelECEF(Vector3d velECEFIn)
  {
    velECEF = velECEFIn;
  }
}