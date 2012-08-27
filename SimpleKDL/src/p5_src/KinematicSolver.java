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

import java.lang.reflect.Method;
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
    protected ChainIkSolverPos_NR_JL      _ikJL;

    protected int               _ikType = IK_TYPE_DEFAULT;

    protected JntArray          _jointAnglesMin;
    protected JntArray          _jointAnglesMax;

    protected JntArray          _jointAnglesDef;
    protected JntArray          _jointAnglesInit;
    protected JntArray          _jointAnglesOut;


    public KinematicSolver()
    {
    }

    public void init(SimpleKDL.Chain chain,float[] jointDegAnglesMin,float[] jointDegAnglesMax)
    {
        _chain = chain;

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
                _jointAnglesMin.set(i, PApplet.radians(jointDegAnglesMin[i]));
                _jointAnglesMax.set(i, PApplet.radians(jointDegAnglesMax[i]));
            }

            _jointAnglesInit.set(i,0);
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

    public boolean solveIk(SimpleKDL.Frame endFrame)
    {
        JntArray oldOut = new JntArray(_jointAnglesOut);

        int ret = _ikJL.CartToJnt(_jointAnglesInit, endFrame, _jointAnglesOut);
        if(ret >= 0)
            _jointAnglesInit = new JntArray(_jointAnglesOut);
        else
        {
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

    public SimpleKDL.JntArray getAngles() { return _jointAnglesOut; }

    ///////////////////////////////////////////////////////////////////////////
    // viz part

    public void draw()
    {
        if(_chain != null)
        {   // draw chain
            switch(_ikType)
            {
            case IK_TYPE_JL:
                drawChain(Utils.inst(),_chain, _jointAnglesOut, _jointAnglesMin, _jointAnglesMax);
                break;
            case IK_TYPE_DEFAULT:
            default:
                 drawChain(Utils.inst(),_chain, _jointAnglesOut, null, null);
                 break;
            }
        }
    }

    public static void drawChain(Utils utils,SimpleKDL.Chain chain, SimpleKDL.JntArray angles, SimpleKDL.JntArray anglesMin, SimpleKDL.JntArray anglesMax)
    {
        Segment segment;

        utils.graphics().pushMatrix();
        if(anglesMin != null && anglesMax != null)
        {
            for (int i=0;i < chain.getNrOfSegments();i++)
                drawSegment(utils,chain.getSegment(i), angles.get(i), anglesMin.get(i), anglesMax.get(i));
        }
        else
        {
            for (int i=0;i < chain.getNrOfSegments();i++)
                drawSegment(utils,chain.getSegment(i), angles.get(i), 0.0, 0.0);
        }
        utils.graphics().popMatrix();
    }

    public static void drawSegment(Utils utils,SimpleKDL.Segment segment, double angle, double angleMin, double angleMax)
    {
        PVector vec = new PVector();

        Joint   curJoint = segment.getJoint();

        PMatrix3D matEnd = Utils.getMatrix(segment.getFrameToTip());
        PMatrix3D matRot = Utils.getMatrix(curJoint.pose(angle));

        // draw the axis
        utils.drawCoordSys(70);

        // draw angle
        utils.graphics().pushMatrix();
        Joint.JointType type = curJoint.getType();
        if(type == Joint.JointType.RotX)
        {
            utils.graphics().rotate(PApplet.radians(90),0.0f,1.0f,0.0f);
            drawJointAngle(utils,curJoint,angle);
        }
        else if(type == Joint.JointType.RotY)
        {
            utils.graphics().rotate(PApplet.radians(90),1.0f,0.0f,0.0f);
            drawJointAngle(utils,curJoint,angle);
        }
        else if(type == Joint.JointType.RotZ)
        {
            drawJointAngle(utils,curJoint,angle);
        }
        else if(type == Joint.JointType.RotAxis)
        {
            Vector axis = curJoint.JointAxis();
            //rotate((float)angle,(float)axis.x(),(float)axis.y(),(float)axis.z());
            drawJointAngle(utils,curJoint,angle);
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
        utils.graphics().popMatrix();

        // set the rotation
        utils.graphics().applyMatrix(matRot);

        // draw limb
        utils.graphics().pushStyle();
        matEnd.mult(new PVector(0, 0, 0), vec);

        utils.graphics().stroke(200, 200, 100);
        utils.graphics().strokeWeight(2);
        utils.graphics().line(0, 0, 0,
                              vec.x, vec.y, vec.z);
        utils.graphics().popStyle();

        // draw the joint
        utils.graphics().pushMatrix();
        if(type == Joint.JointType.RotX)
        {
          utils.graphics().rotate(PApplet.radians(90),0.0f,1.0f,0.0f);
          drawJoint(utils,curJoint);
        }
        else if(type == Joint.JointType.RotY)
        {
          utils.graphics().rotate(PApplet.radians(90),1.0f,0.0f,0.0f);
          drawJoint(utils,curJoint);
        }
        else if(type == Joint.JointType.RotZ)
        {
          drawJoint(utils,curJoint);
        }
        else if(type == Joint.JointType.RotAxis)
        {
          Vector axis = curJoint.JointAxis();
          //rotate((float)angle,(float)axis.x(),(float)axis.y(),(float)axis.z());
          drawJoint(utils,curJoint);
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

        utils.graphics().popMatrix();

        utils.graphics().applyMatrix(matEnd);
    }

    public static void drawJoint(Utils utils,SimpleKDL.Joint joint)
    {
        float h=40;
        float r=10;

        utils.graphics().pushMatrix();
        utils.graphics().pushStyle();
        utils.graphics().fill(255);
        utils.graphics().noStroke();
        utils.drawCylinder(r,h,10);
        utils.graphics().popStyle();
        utils.graphics().popMatrix();
    }

    public static void drawJointAngle(Utils utils,SimpleKDL.Joint joint,double angle)
    {
        float angleStart;
        float angleEnd;

        //angle += PI/2;
        utils.graphics().pushStyle();
        utils.graphics().fill(255,255,0,200);
        utils.graphics().stroke(255,255,255,200);
        utils.graphics().strokeWeight(1);
        if(angle > 0)
          utils.graphics().arc(0.0f, 0.0f, 100.0f, 100.0f, 0.0f,(float)angle);
        else
          utils.graphics().arc(0.0f, 0.0f, 100.0f, 100.0f, (float)angle, 0.0f);
        utils.graphics().popStyle();
    }


}
