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

public class Utils
{
    protected PApplet           _parent;

    protected PVector[]         _sphereVertex;
    protected int               _sphereRes = 10;

    protected static Utils _utils = null;

    public static Utils inst() { return _utils; }
    public static void setInst(Utils utils) { _utils = utils;}

    public void init(PApplet parent)
    {
        _parent = parent;

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
    }

    public static void drawCoordSys(PGraphics g,int len)
    {
        // x-axis
        g.stroke(255, 0, 0);
        g.line(0, 0, 0, len, 0, 0);

        // y-axis
        g.stroke(0, 255, 0);
        g.line(0, 0, 0, 0, len, 0);

        // z-axis
        g.stroke(0, 0, 255);
        g.line(0, 0, 0, 0, 0, len);
    }

    public void drawCoordSys(int len)
    {
        drawCoordSys(_parent.g,len);
    }

    public static void drawPlane(PGraphics g,PVector p1, PVector p2, PVector p3,
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
            g.line(posU1.x, posU1.y, posU1.z,
                   posU2.x, posU2.y, posU2.z);
            g.line(posV1.x, posV1.y, posV1.z,
                   posV2.x, posV2.y, posV2.z);

            posU1.add(stepsU);
            posU2.add(stepsU);

            posV1.add(stepsV);
            posV2.add(stepsV);
        }
    }

    public void drawPlane(PVector p1, PVector p2, PVector p3,
                          int len, int repeat)
    {
        drawPlane(_parent.g,p1,p2,p3,len,repeat);
    }

    public static PVector getPlaneNormal(PVector p1, PVector p2, PVector p3)
    {
        PVector u = PVector.sub(p2,p1);
        PVector v = PVector.sub(p3,p1);

        PVector ret = u.cross(v);
        ret.normalize();
        return ret;
    }
}
