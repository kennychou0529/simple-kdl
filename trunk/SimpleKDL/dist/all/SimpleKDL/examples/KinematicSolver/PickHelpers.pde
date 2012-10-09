
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

		PGL context = _gOpengl.beginPGL();

		_gl  = context.gl;
		_glu = context.glu;

        int viewport[] = new int[4];
        float[] proj=new float[16];
        float[] model=new float[16];

        _gl.glGetIntegerv(GL.GL_VIEWPORT, viewport, 0);
		// glGetFloatv doesn't work with the new processing
		proj = transformP5toGL(((PGraphicsOpenGL)g).projection);
		model = transformP5toGL(((PGraphicsOpenGL)g).modelview);

        float[] mousePosArr=new float[4];

        _glu.gluUnProject((float)winX, viewport[3]-(float)winY, (float)z,
                          model, 0, proj, 0, viewport, 0, mousePosArr, 0);

		_gOpengl.endPGL();

        return new PVector((float)mousePosArr[0], (float)mousePosArr[1], (float)mousePosArr[2]);
    }
}

public static float[] transformP5toGL(PMatrix3D mat)
{
	float[] ret = new float[16];

	ret[0] = mat.m00;
    ret[1] = mat.m10;
    ret[2] = mat.m20;
    ret[3] = mat.m30;

    ret[4] = mat.m01;
    ret[5] = mat.m11;
    ret[6] = mat.m21;
    ret[7] = mat.m31;

    ret[8] = mat.m02;
    ret[9] = mat.m12;
    ret[10] = mat.m22;
    ret[11] = mat.m32;

    ret[12] = mat.m03;
    ret[13] = mat.m13;
    ret[14] = mat.m23;
    ret[15] = mat.m33;

	return ret;
}

