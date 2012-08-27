/* ----------------------------------------------------------------------------
 * SimpleKDL
 * ----------------------------------------------------------------------------
 * Copyright (C) 2012 Max Rheiner / Interaction Design Zhdk
 *
 * This file is part of SimpleOpenNI.
 *
 * SimpleOpenNI is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version (subject to the "Classpath" exception
 * as provided in the LICENSE.txt file that accompanied this code).
 *
 * SimpleKDL is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with SimpleKDL.  If not, see <http://www.gnu.org/licenses/>.
 * ----------------------------------------------------------------------------
 */
 
package SimpleKDL;

import processing.core.*;
import processing.opengl.*;
import javax.media.opengl.*;
import javax.media.opengl.glu.*;

public class Utils
{
    // opengl
    protected GLUquadric       _quadric;
    protected GL               _gl;
    protected GLU              _glu;
    protected PGraphicsOpenGL  _gOpengl = null;

    protected PApplet           _parent;

    protected PVector[]         _sphereVertex;
    protected int               _sphereRes = 10;

    protected static Utils _utils = null;

    public static Utils inst() { return _utils; }
    public static void setInst(Utils utils) { _utils = utils;}

    public void init(PApplet parent)
    {
        _parent = parent;

        _gOpengl = (PGraphicsOpenGL)parent.g;
        _gl  = _gOpengl.gl;
        _glu = _gOpengl.glu;

        /*
        _quadric = _glu.gluNewQuadric();
        _glu.gluQuadricDrawStyle(_quadric, GLU.GLU_FILL);
        _glu.gluQuadricNormals(_quadric, GLU.GLU_SMOOTH);
        */

        // precalc cylinder
        _sphereVertex = new PVector[_sphereRes];
        float q = PApplet.TWO_PI / (float)_sphereVertex.length;
        float angle;
        for(int i=0;i<_sphereVertex.length;i++)
        {
            angle = i * q;
            _sphereVertex[i] = new PVector(_parent.sin(angle), _parent.cos(angle), 0);
            //_sphereVertex[i].normalize();
        }
    }

    PApplet parent() { return _parent; }
    PGraphics graphics() { return _parent.g; }

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

    ///////////////////////////////////////////////////////////////////////////
    // static helper functions

    // convert the frame into a processing matrix
    public static PMatrix3D getMatrix(SimpleKDL.Frame F)
    {
      float[] tf=new float[16];
      int i;
      int j;
      for (i=0;i<3;i++)
      {
        for (j=0;j<3;j++)
          tf[i+j*4]= (float) F.getM().get(i, j);

        tf[i+3*4] = (float) F.getP().get(i);
      }
      for (j=0;j<3;j++)
        tf[3+j*4] = 0.0f;

      tf[15] = 1;

      PMatrix3D mat = new PMatrix3D();
      mat.set(tf);
      // left hand -> right hand
      // https://forum.processing.org/topic/understanding-pmatrix3d#25080000001370087
      mat.transpose();

      return mat;
    }

    public static PMatrix3D getXFormMat(PVector origCenter,
                                        PVector origX,
                                        PVector origY,
                                        PVector origZ,
                                        PVector newCenter,
                                        PVector newX,
                                        PVector newY,
                                        PVector newZ)
    {
        float[] tf=new float[16];

        SimpleKDLMain.getXFormMat(origCenter.array(),
                                  origX.array(),
                                  origY.array(),
                                  origZ.array(),
                                  newCenter.array(),
                                  newX.array(),
                                  newY.array(),
                                  newZ.array(),
                                  tf);

        PMatrix3D xform = new PMatrix3D();
        xform.set(tf);

        return xform;
    }


    public static Frame getFrame(PVector origCenter,
                                 PVector origX,
                                 PVector origY,
                                 PVector origZ,
                                 PVector newCenter,
                                 PVector newX,
                                 PVector newY,
                                 PVector newZ)
    {
        float[] tf=new float[16];

        SimpleKDLMain.getXFormMat(origCenter.array(),
                                  origX.array(),
                                  origY.array(),
                                  origZ.array(),
                                  newCenter.array(),
                                  newX.array(),
                                  newY.array(),
                                  newZ.array(),
                                  tf);

        Rotation rot = new Rotation(tf[0],tf[4],tf[8],
                                    tf[1],tf[5],tf[9],
                                    tf[2],tf[6],tf[10]);

        return new Frame(rot,
                         new Vector(tf[12],tf[13],tf[14]));
    }

    public static PVector getPlaneNormal(PVector p1, PVector p2, PVector p3)
    {
        PVector u = PVector.sub(p2,p1);
        PVector v = PVector.sub(p3,p1);

        PVector ret = u.cross(v);
        ret.normalize();
        return ret;
    }
    ///////////////////////////////////////////////////////////////////////////
    // drawing helpers
    public void drawCylinder(float radius,float h,int slices)
    {
        if(_sphereVertex == null)
            return;

        int i;
        PVector vec;
        // top
        _parent.g.beginShape();
            for(i=0;i<_sphereVertex.length;i++)
            {
                vec = PVector.mult(_sphereVertex[i],radius);
                _parent.g.vertex(vec.x, vec.y, vec.z - (h * .5f));
            }
        _parent.g.endShape(PApplet.CLOSE);

        // bottom
        _parent.g.beginShape();
            for(i=0;i<_sphereVertex.length;i++)
            {
                vec = PVector.mult(_sphereVertex[i],radius);
                _parent.g.vertex(vec.x, vec.y, vec.z + (h * .5f));
            }
        _parent.g.endShape(PApplet.CLOSE);

        // mantel
        _parent.g.beginShape(PApplet.TRIANGLE_STRIP);
            for(i=0;i<_sphereVertex.length;i++)
            {
                vec = PVector.mult(_sphereVertex[i],radius);
                _parent.g.vertex(vec.x, vec.y, vec.z + (h * .5f));
                _parent.g.vertex(vec.x, vec.y, vec.z - (h * .5f));
            }

            vec = PVector.mult(_sphereVertex[0],radius);
            _parent.g.vertex(vec.x, vec.y, vec.z + (h * .5f));
            _parent.g.vertex(vec.x, vec.y, vec.z - (h * .5f));
        _parent.g.endShape();

        /*
        gOpengl.beginGL();
        gl.glPushAttrib(GL.GL_ALL_ATTRIB_BITS);
        gl.glEnable(GL.GL_LIGHTING);

        glu.gluCylinder(quadric, baseRadius, topRadius,h, slices, stacks);

        gl.glPopAttrib();
        gOpengl.endGL();
        */
    }

    public void drawCoordSys(int len)
    {
        // x-axis
        _parent.g.stroke(255, 0, 0);
        _parent.g.line(0, 0, 0, len, 0, 0);

        // y-axis
        _parent.g.stroke(0, 255, 0);
        _parent.g.line(0, 0, 0, 0, len, 0);

        // z-axis
        _parent.g.stroke(0, 0, 255);
        _parent.g.line(0, 0, 0, 0, 0, len);
    }

    public void drawPlane(PVector p1, PVector p2, PVector p3,
                   int len, int repeat)
    {
        repeat--;

        // p1 is the center
        PVector u = PVector.sub(p2, p1);
        u.normalize();
        PVector v = PVector.sub(p3, p1);
        v.normalize();
        PVector dirUp = u.cross(v);
        dirUp.normalize();

        // rectangular
        PVector dirV = u.cross(dirUp);
        dirV.normalize();

        PVector stepsU = PVector.mult(u, (float)len / (float)repeat);
        PVector stepsV = PVector.mult(dirV, (float)len / (float)repeat);

        PVector posU1 = PVector.add(PVector.mult(stepsU, -0.5f * repeat), PVector.mult(stepsV, -0.5f * repeat));
        PVector posU2 = PVector.add(PVector.mult(stepsU, -0.5f * repeat), PVector.mult(stepsV, 0.5f * repeat));

        PVector posV1 = PVector.add(PVector.mult(stepsU, -0.5f * repeat), PVector.mult(stepsV, -0.5f * repeat));
        PVector posV2 = PVector.add(PVector.mult(stepsU, 0.5f * repeat), PVector.mult(stepsV, -0.5f * repeat));

        // horz
        for (int i=0;i<repeat+1;i++)
        {
            _parent.g.line(posU1.x, posU1.y, posU1.z,
                           posU2.x, posU2.y, posU2.z);
            _parent.g.line(posV1.x, posV1.y, posV1.z,
                           posV2.x, posV2.y, posV2.z);

            posU1.add(stepsU);
            posU2.add(stepsU);

            posV1.add(stepsV);
            posV2.add(stepsV);
        }
    }

}
