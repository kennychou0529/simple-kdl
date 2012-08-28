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


public class KinematicSolver
{
    public final static int     IK_TYPE_DEFAULT     = 0;
    public final static int     IK_TYPE_JL          = 1;


    protected Chain             _chain = null;

    // forward kinematics
    protected int               _maxItr = 100;
    protected double            _eps = 0.01; // all in mm

    protected ChainFkSolverPos_recursive  _fk;

    // inverse kinematics
    protected ChainIkSolverVel_pinv       _vik;
    protected ChainIkSolverPos_NR         _ik;

    protected ChainIkSolverVel_wdls       _ikSolverVel;
    protected ChainIkSolverPos_NR_JL      _ikJL = null;

    protected int               _ikType = IK_TYPE_DEFAULT;

    protected JntArray          _jointAnglesMin;
    protected JntArray          _jointAnglesMax;

    protected JntArray          _jointAnglesDef;
    protected JntArray          _jointAnglesInit;
    protected JntArray          _jointAnglesOut;

    public int getIkType() { return _ikType; }

    public KinematicSolver(Chain chain,float[] jointDegAnglesMin,float[] jointDegAnglesMax)
    {
        _chain = chain;
        _ikType = IK_TYPE_JL;

        // init kinematic solvers

        // set min max angles for the joint limits
        _jointAnglesMin = new JntArray(_chain.getNrOfSegments());
        _jointAnglesMax = new JntArray(_chain.getNrOfSegments());

        _jointAnglesInit = new JntArray(_chain.getNrOfSegments());
        _jointAnglesOut = new JntArray(_chain.getNrOfSegments());
        _jointAnglesDef = new JntArray(_chain.getNrOfSegments());

        for(int i=0; i < chain.getNrOfSegments();i++)
        {
            if(i < jointDegAnglesMin.length && i < jointDegAnglesMax.length)
            {
                _jointAnglesMin.set(i, jointDegAnglesMin[i]);
                _jointAnglesMax.set(i, jointDegAnglesMax[i]);
            }

            _jointAnglesInit.set(i,0);
            _jointAnglesOut.set(i,0);
            _jointAnglesDef.set(i,0);
        }

        // init forward kinematics
        _fk = new ChainFkSolverPos_recursive(_chain);

        // inverse kinematic
        _vik = new ChainIkSolverVel_pinv(_chain);
        _ik = new ChainIkSolverPos_NR(_chain, _fk, _vik,_maxItr,_eps);

        _ikSolverVel = new ChainIkSolverVel_wdls(_chain, _eps, _maxItr );
        _ikJL = new ChainIkSolverPos_NR_JL(_chain,
                                           _jointAnglesMin, _jointAnglesMax,
                                           _fk, _ikSolverVel,
                                           _maxItr, _eps);
    }

    public KinematicSolver(Chain chain)
    {
        _chain = chain;
        _ikType = IK_TYPE_DEFAULT;

        // init kinematic solvers
        _jointAnglesInit = new JntArray(_chain.getNrOfSegments());
        _jointAnglesOut = new JntArray(_chain.getNrOfSegments());
        _jointAnglesDef = new JntArray(_chain.getNrOfSegments());

        for(int i=0; i < chain.getNrOfSegments();i++)
        {
            _jointAnglesInit.set(i,0);
            _jointAnglesOut.set(i,0);
            _jointAnglesDef.set(i,0);
        }

        // init forward kinematics
        _fk = new ChainFkSolverPos_recursive(_chain);

        // inverse kinematic
        _vik = new ChainIkSolverVel_pinv(_chain);
        _ik = new ChainIkSolverPos_NR(_chain, _fk, _vik,_maxItr,_eps);
    }

    public boolean solveIk(SimpleKDL.Frame endFrame)
    {
        JntArray oldOut = new JntArray(_jointAnglesOut);

        int ret;

        switch(_ikType)
        {
        case IK_TYPE_JL:
            ret = _ikJL.CartToJnt(_jointAnglesInit, endFrame, _jointAnglesOut);
            break;
        case IK_TYPE_DEFAULT:
        default:
            ret = _ik.CartToJnt(_jointAnglesInit, endFrame, _jointAnglesOut);
            break;
        }

        if(ret >= 0)
            // use out angles as init angles
            _jointAnglesInit = new JntArray(_jointAnglesOut);
        else
        {   // couldn't solve ik

           _jointAnglesInit = new JntArray(_jointAnglesDef);

           // set last pos
           _jointAnglesOut = new JntArray(oldOut);
        }

        return(ret >= 0);
    }

    public SimpleKDL.Frame solveFk(SimpleKDL.JntArray angles)
    {
        _jointAnglesOut = new JntArray(angles);
        Frame finalFrame = new Frame();
        int ret = _fk.JntToCart(_jointAnglesOut, finalFrame);

        return finalFrame;
    }

    public SimpleKDL.Frame solveFk(float[] angles)
    {
        return solveFk(toJntArray(angles));
    }

    public JntArray getInitAnglesArray() { return _jointAnglesInit; }
    public JntArray getOutAnglesArray() { return _jointAnglesOut; }

    public float[] getInitAngles() { return toArray(_jointAnglesInit); }
    public float[] getOutAngles() { return toArray(_jointAnglesOut); }

    public void setInitAngles(SimpleKDL.JntArray angles)
    {
        _jointAnglesInit = new JntArray(angles);
    }

    public void setOutAngles(SimpleKDL.JntArray angles)
    {
        _jointAnglesOut = new JntArray(angles);
    }

    public int getSegmentCount()
    {
        if(_chain == null)
            return 0;
        return (int)_chain.getNrOfSegments();
    }


    ///////////////////////////////////////////////////////////////////////////
    // helper

    public static JntArray toJntArray(float[] array)
    {
        JntArray jntArray = new JntArray(array.length);

        for(int i=0; i < array.length;i++)
           jntArray.set(i,array[i]);

        return jntArray;
    }

    public static float[] toArray(JntArray jntArray)
    {
        float[] array = new float[(int)jntArray.rows()];

        for(int i=0; i < array.length;i++)
           array[i] = (float)jntArray.get(i);

        return array;
    }

    ///////////////////////////////////////////////////////////////////////////
    // viz part

    public void draw(PGraphics g)
    {
        if(_chain != null)
        {   // draw chain
            switch(_ikType)
            {
            case IK_TYPE_JL:
                drawChain(g,_chain, _jointAnglesOut, _jointAnglesMin, _jointAnglesMax);
                break;
            case IK_TYPE_DEFAULT:
            default:
                drawChain(g,_chain, _jointAnglesOut, null, null);
                break;
            }
        }
    }

    public static void drawChain(PGraphics g,SimpleKDL.Chain chain, SimpleKDL.JntArray angles, SimpleKDL.JntArray anglesMin, SimpleKDL.JntArray anglesMax)
    {
        Segment segment;

        g.pushMatrix();
        if(anglesMin != null && anglesMax != null)
        {
            for (int i=0;i < chain.getNrOfSegments();i++)
                drawSegment(g,chain.getSegment(i), angles.get(i), anglesMin.get(i), anglesMax.get(i));
        }
        else
        {
            for (int i=0;i < chain.getNrOfSegments();i++)
                drawSegment(g,chain.getSegment(i), angles.get(i), 0.0, 0.0);
        }
        g.popMatrix();
    }

    public static void drawSegment(PGraphics g,SimpleKDL.Segment segment, double angle, double angleMin, double angleMax)
    {
        PVector vec = new PVector();

        Joint   curJoint = segment.getJoint();

        PMatrix3D matEnd = getMatrix(segment.getFrameToTip());
        PMatrix3D matRot = getMatrix(curJoint.pose(angle));

        // draw the axis
        Utils.drawCoordSys(g,70);

        // draw angle
        g.pushMatrix();
            Joint.JointType type = curJoint.getType();
            if(type == Joint.JointType.RotX)
            {
                g.rotate(PApplet.radians(90),0.0f,1.0f,0.0f);
                drawJointAngle(g,curJoint,angle);
            }
            else if(type == Joint.JointType.RotY)
            {
                g.rotate(PApplet.radians(90),1.0f,0.0f,0.0f);
                drawJointAngle(g,curJoint,angle);
            }
            else if(type == Joint.JointType.RotZ)
            {
                drawJointAngle(g,curJoint,angle);
            }
            else if(type == Joint.JointType.RotAxis)
            {
                Vector axis = curJoint.JointAxis();
                //rotate((float)angle,(float)axis.x(),(float)axis.y(),(float)axis.z());
                drawJointAngle(g,curJoint,angle);
            }
            else if(type == Joint.JointType.TransX)
            {
            }
            else if(type == Joint.JointType.TransY)
            {
            }
            else if(type == Joint.JointType.TransZ)
            {
            }
            else if(type == Joint.JointType.TransAxis)
            {
            }
        g.popMatrix();

        // set the rotation
        g.applyMatrix(matRot);

        // draw limb
        g.pushStyle();
        matEnd.mult(new PVector(0, 0, 0), vec);

        g.stroke(200, 200, 100);
        g.strokeWeight(2);
        g.line(0, 0, 0,
                              vec.x, vec.y, vec.z);
        g.popStyle();

        // draw the joint
        g.pushMatrix();
            if(type == Joint.JointType.RotX)
            {
              g.rotate(PApplet.radians(90),0.0f,1.0f,0.0f);
              drawJoint(g,curJoint);
            }
            else if(type == Joint.JointType.RotY)
            {
              g.rotate(PApplet.radians(90),1.0f,0.0f,0.0f);
              drawJoint(g,curJoint);
            }
            else if(type == Joint.JointType.RotZ)
            {
              drawJoint(g,curJoint);
            }
            else if(type == Joint.JointType.RotAxis)
            {
              Vector axis = curJoint.JointAxis();
              //rotate((float)angle,(float)axis.x(),(float)axis.y(),(float)axis.z());
              drawJoint(g,curJoint);
            }
            else if(type == Joint.JointType.TransX)
            {
            }
            else if(type == Joint.JointType.TransY)
            {
            }
            else if(type == Joint.JointType.TransZ)
            {
            }
            else if(type == Joint.JointType.TransAxis)
            {
            }
        g.popMatrix();

        g.applyMatrix(matEnd);
    }

    public static void drawJoint(PGraphics g,SimpleKDL.Joint joint)
    {
        float h=40;
        float r=10;

        g.pushMatrix();
        g.pushStyle();
        g.fill(255);
        g.noStroke();
        Utils.inst().drawCylinder(r,h,10);
        g.popStyle();
        g.popMatrix();
    }

    public static void drawJointAngle(PGraphics g,SimpleKDL.Joint joint,double angle)
    {
        float angleStart;
        float angleEnd;

        //angle += PI/2;
        g.pushStyle();
        g.fill(255,255,0,200);
        g.stroke(255,255,255,200);
        g.strokeWeight(1);
        if(angle > 0)
          g.arc(0.0f, 0.0f, 100.0f, 100.0f, 0.0f,(float)angle);
        else
          g.arc(0.0f, 0.0f, 100.0f, 100.0f, (float)angle, 0.0f);
        g.popStyle();
    }

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

    ///////////////////////////////////////////////////////////////////////////
    // static helpers

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


}
