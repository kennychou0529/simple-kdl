
public class PickHelpers
{
    // opengl
    protected GLUquadric       _quadric;
    protected GL               _gl;
    protected GLU              _glu;
    protected PGraphicsOpenGL  _gOpengl = null;

    public void init(PApplet parent)
    {
         _gOpengl = (PGraphicsOpenGL)parent.g;
        _gl  = _gOpengl.gl;
        _glu = _gOpengl.glu;
    }

    ///////////////////////////////////////////////////////////////////////////
    // helper functions
    public void getHitRay(int pick2dX,int pick2dY,
                   PVector r1,PVector r2)
    {
        r1.set(unProject(pick2dX, pick2dY, 0));
        r2.set(unProject(pick2dX, pick2dY, 1));
    }

    public PVector unProject(float winX, float winY, float z)
    {
        if(_gOpengl == null)
            return new PVector();

        _gOpengl.beginGL();
        int viewport[] = new int[4];
        double[] proj=new double[16];
        double[] model=new double[16];

        _gl.glGetIntegerv(GL.GL_VIEWPORT, viewport, 0);
        _gl.glGetDoublev(GL.GL_PROJECTION_MATRIX, proj, 0);
        _gl.glGetDoublev(GL.GL_MODELVIEW_MATRIX, model, 0);

        double[] mousePosArr=new double[4];

        _glu.gluUnProject((double)winX, viewport[3]-(double)winY, (double)z,
                          model, 0, proj, 0, viewport, 0, mousePosArr, 0);

        _gOpengl.endGL();

        return new PVector((float)mousePosArr[0], (float)mousePosArr[1], (float)mousePosArr[2]);
    }
}
