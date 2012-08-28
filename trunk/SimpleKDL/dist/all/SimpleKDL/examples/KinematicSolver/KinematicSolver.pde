/* --------------------------------------------------------------------------
 * SimpleKDL Kinematic Solver
 * --------------------------------------------------------------------------
 * Processing Wrapper for the Orocos KDL library
 * http://code.google.com/p/simple-kdl
 * --------------------------------------------------------------------------
 * prog:  Max Rheiner / Interaction Design / Zhdk / http://iad.zhdk.ch/
 * date:  08/26/2012 (m/d/y)
 * ----------------------------------------------------------------------------
 *  This example shows how to use the KinematicSolver
 *  - shift + left mouse button -> sets new position
 * ----------------------------------------------------------------------------
 */

import processing.opengl.*;
import javax.media.opengl.*;
import javax.media.opengl.glu.*;

import peasy.*;
import SimpleKDL.*;

PeasyCam cam;

SimpleKDL.Chain     chain;
SimpleKDL.JntArray  jointAngles;

SimpleKDL.KinematicSolver     kinematicSolver;

PVector endPoint = new PVector();
PVector endPointNormal = new PVector();
PVector endPointRotX = new PVector(1,0,0);
PVector endPointRotY = new PVector(0,1,0);
PVector endPointRotZ = new PVector(0,0,-1);
PMatrix3D endPointMat = new PMatrix3D();

int planeType = 0;
int mousePick = 0;

// scene
PVector sphereCenter = new PVector(-500,400,150);
float   sphereRadius = 150;

PVector boxCenter = new PVector(400,300,75);
PVector boxSize = new PVector(300,200,150);
float   boxRotZ = -15;

// define picking plane x-y
PickHelpers pickHelpers = new PickHelpers();

PVector pickPlaneP1 = new PVector(0, 0, 0);
PVector pickPlaneP2 = new PVector(1, 0, 0);
PVector pickPlaneP3 = new PVector(0, 1, 0);
boolean lastPick = false;

// picking trail
ArrayList  pickTrail = new ArrayList();
int pickTrailMax = 200;

void setup() 
{
  hint(ENABLE_OPENGL_4X_SMOOTH);  // antialiasing is somehow buggy in processing ?!
  smooth();
  size(1280, 720, OPENGL);

  // setup cam
  cam = new peasy.PeasyCam(this, 2000);
  cam.setMinimumDistance(50);
  cam.setMaximumDistance(5000);
   
  ///////////////////////////////////////////////////////////
  // start SimpleKdl
  SimpleKDLContext.init(this);

  ///////////////////////////////////////////////////////////
  // setup the chain
  chain   = new SimpleKDL.Chain();

  SimpleKDL.Joint    joint;
  SimpleKDL.Frame    frame;
  SimpleKDL.Segment  segment;

  joint  = new SimpleKDL.Joint(Joint.JointType.RotZ); 
  frame  = new SimpleKDL.Frame(new SimpleKDL.Vector(0, 0, 100));
  segment = new SimpleKDL.Segment(joint, frame);
  chain.addSegment(segment);

  joint  = new SimpleKDL.Joint(Joint.JointType.RotX); 
  frame  = new SimpleKDL.Frame(new SimpleKDL.Vector(0, 0, 400));
  segment = new SimpleKDL.Segment(joint, frame);
  chain.addSegment(segment);

  joint  = new SimpleKDL.Joint(Joint.JointType.RotX); 
  frame  = new SimpleKDL.Frame(new SimpleKDL.Vector(0, 0.0,400));
  segment = new SimpleKDL.Segment(joint, frame);
  chain.addSegment(segment);
 
  joint  = new SimpleKDL.Joint(Joint.JointType.RotX); 
  frame  = new SimpleKDL.Frame(new SimpleKDL.Vector(0, 0, 100.0));
  segment = new SimpleKDL.Segment(joint, frame);
  chain.addSegment(segment);
 
  ///////////////////////////////////////////////////////////
  // setup the solver
  float[] minAngles = new float[(int)chain.getNrOfSegments()];
  float[] maxAngles = new float[(int)chain.getNrOfSegments()];
  
  minAngles[0] = radians(-990.0);
  maxAngles[0] = radians(961.0);

  minAngles[1] = radians(-20);
  maxAngles[1] = radians(90);

  minAngles[2] = radians(-20);
  maxAngles[2] = radians(170);

  minAngles[3] = radians(-120);
  maxAngles[3] = radians(120);
  
  kinematicSolver = new SimpleKDL.KinematicSolver(chain,minAngles,maxAngles);
  // if you only want ik without joint limits
  //kinematicSolver = new SimpleKDL.KinematicSolver(chain);
    
  ///////////////////////////////////////////////////////////
  // setup for picking
  pickHelpers.init(this);
}

void mousePressed()
{
  if((mouseButton == LEFT) && (keyPressed == true) && (key == CODED) && (keyCode == SHIFT))
  {
    noCursor();
    mousePick = 1;
    
    // switch off the left dragging of camera
    cam.setActive(false);
    
    pickScene();
  }
}

void mouseDragged()
{
  if(mousePick == 0)
    return;  

  pickScene(); 
}

void mouseReleased()
{
  if(mousePick == 1)
  {
    cursor();
    mousePick = 0;
    
    // switch on the left dragging of camera
    cam.setActive(true);
  }    
  
}

void draw() 
{
  // set the lights
  ambientLight(100, 100,100);
  directionalLight(200, 172, 235,
                   1, -1, 0);
                   
  background(0);
  
  //////////////////////////////////////////////
  // draw the chain
  kinematicSolver.draw(g);

  //////////////////////////////////////////////
  // draw picking plane
  pushStyle();
  strokeWeight(1);
  stroke(150, 150, 150, 100);
  Utils.drawPlane(g,pickPlaneP1, pickPlaneP2, pickPlaneP3, 
                  2000, 21);
  popStyle();  

  //////////////////////////////////////////////
  // draw pickTrail
  if(pickTrail.size() > 0)
  {
    PVector vec;
    float q = 1.0f / pickTrail.size();
    noFill();
    beginShape();
    for(int i=0;i < pickTrail.size();i++)
    {
      vec = (PVector) pickTrail.get(i);
      stroke(19, 200, 170, 255 * q * i);
      vertex(vec.x,vec.y,vec.z);
    }
    endShape();
  }
  
  //////////////////////////////////////////////
  // draw picking pos
  pushStyle();
  pushMatrix();
  applyMatrix(endPointMat);
  strokeWeight(5);  
  Utils.drawCoordSys(g,50);
  strokeWeight(1);  
  popMatrix();
  
  pushMatrix();
  translate(endPoint.x, endPoint.y, endPoint.z);
  
  if(lastPick)
  {
    fill(0, 255, 0,200);
    stroke(0, 255, 0,200);
  }    
  else 
  {
    fill(255, 0, 0,200);
    stroke(255, 0, 0,200);
  }  

  line(0,0,0,
       endPointNormal.x * 70,endPointNormal.y  * 70,endPointNormal.z  * 70);
  
  noStroke();
  sphere(15);
  popMatrix();

  //////////////////////////////////////////////
  // draw pickScene  
  pushMatrix();
    fill(185, 255, 69,100);
    stroke(255);
    strokeWeight(1);
    translate(boxCenter.x,boxCenter.y,boxCenter.z);
    rotateZ(radians(boxRotZ));
    box(boxSize.x,boxSize.y,boxSize.z);
  popMatrix();
   
  pushMatrix();
    fill(185, 255, 69,100);
    noStroke();
    translate(sphereCenter.x,sphereCenter.y,sphereCenter.z);
    sphere(sphereRadius);
  popMatrix();
   
  popStyle();
  
}

void keyPressed()
{
  switch(key)
  {
  case ' ':
    println("----------------------");
    println("Test forward kinematic");
  
    // test last position
    SimpleKDL.Frame frame = kinematicSolver.solveFk( kinematicSolver.getOutAngles() );
    
    println("Rotational Matrix of the final Frame: ");
    println(frame.getM());
    println("End-effector position: " + frame.getP());
    break;
    
  case 'p':
    float[] angles = kinematicSolver.getOutAngles();

    println("--------------------------");
    println("Current angles in degrees:");
    for(int i=0; i < angles.length;i++)
      println("joint" + i + " = " +  degrees(angles[i]));
      
    println("Current position:" + endPoint);
          
    break;
  }
}

void pickScene()
{
  PVector r1 = new PVector();
  PVector r2 = new PVector();
  
  // get the current hit ray
  pickHelpers.getHitRay(mouseX,mouseY,
                        r1,r2);
                    
  PVector rDir = PVector.sub(r2,r1);
  PVector pick3d = new PVector();
  PVector pick3d1 = new PVector();
  PVector pickNormal = new PVector();
  PVector pickNormal1 = new PVector();
  
  // check if the hit was on the sphere
  if(RayIntersection.sphere(r1, rDir, 
                            sphereCenter,sphereRadius,
                            pick3d,pick3d1,
                            pickNormal,pickNormal1) >= 1)
  {
    // only use the first picking point
    lastPick = setEndPos(pick3d,pickNormal);   
    return;
  }                             

  // check if the hit was on the box
  PMatrix3D xform = new PMatrix3D();
  xform.translate(boxCenter.x, boxCenter.y, boxCenter.z);

  PMatrix3D rotZ = new PMatrix3D();
  rotZ.rotateZ(radians(boxRotZ));
  xform.apply(rotZ);

  if(RayIntersection.box(xform,
                         r1, rDir, 
                         new PVector(0,0,0),
                         boxSize.x,boxSize.y,boxSize.z,
                         pick3d,pick3d1,
                         pickNormal,pickNormal1) >= 1)
  {
    // only use the first picking point
    lastPick = setEndPos(pick3d,pickNormal);   
    return;
  }   
  
  // check if the hit was on the plane               
  if(RayIntersection.plane(r1, rDir, 
                           pickPlaneP1, pickPlaneP2, pickPlaneP3, 
                           pick3d))
  {
    pickNormal = Utils.getPlaneNormal(pickPlaneP1,pickPlaneP2,pickPlaneP3);
    lastPick = setEndPos(pick3d,pickNormal);
    return;
  }
}
 
void calcEndRotMat(PVector start,PVector end, PVector normalEnd)
{
  PVector v = PVector.sub(end,start);
  v.normalize();
  PVector u = v.cross(normalEnd);

  // the end frame should be aligned with the chain base
  SimpleKDL.Frame testFrame = SimpleKDL.KinematicSolver.getFrame(// orig
                                                     new PVector(0,0,0),  // z
                                                     u,
                                                     v,
                                                     normalEnd,
                                                     // dest
                                                     new PVector(0,0,0),  // x
                                                     new PVector(1,0,0),  // x
                                                     new PVector(0,1,0),  // y
                                                     new PVector(0,0,1)  // z
                                                     );

   SimpleKDL.Rotation rotFlip =  new SimpleKDL.Rotation(1,0,0,
                                                        0,1,0,
                                                        0,0,-1);
                                                       // 0,0,1);
                                                      
   SimpleKDL.Rotation rot =  SimpleKDL.Rotation.RPY(radians(-30),0,0);
 
   SimpleKDL.Rotation rotEnd = SimpleKDL.Rotation.mult(rotFlip,testFrame.getM());
 
 
   endPointRotX = new PVector((float)rotEnd.getData()[0],(float)rotEnd.getData()[1],(float)rotEnd.getData()[2]);
   endPointRotY = new PVector((float)rotEnd.getData()[3],(float)rotEnd.getData()[4],(float)rotEnd.getData()[5]);
   endPointRotZ = new PVector((float)rotEnd.getData()[6],(float)rotEnd.getData()[7],(float)rotEnd.getData()[8]);
}
 
boolean setEndPos(PVector endPos,PVector endPosNormal)
{
  endPoint = endPos.get();
  
  // flip the direction of the normal, because the robot end segment looks down
 // endPointNormal = PVector.mult(endPosNormal,-1);
  endPointNormal = endPosNormal.get();
  
  // calculate rotation frame according to the chain base
  calcEndRotMat(new PVector(0,0,0),
                endPoint,
                endPointNormal);
  
  SimpleKDL.Frame desiredFrame = new SimpleKDL.Frame(new SimpleKDL.Rotation(new SimpleKDL.Vector(endPointRotX.x,endPointRotX.y,endPointRotX.z),
                                                                            new SimpleKDL.Vector(endPointRotY.x,endPointRotY.y,endPointRotY.z),
                                                                            new SimpleKDL.Vector(endPointRotZ.x,endPointRotZ.y,endPointRotZ.z)),
                                                     new SimpleKDL.Vector(endPos.x, endPos.y, endPos.z));
  // calculate matrix for visualization                       
  endPointMat = SimpleKDL.KinematicSolver.getMatrix(desiredFrame);
    
  // set the end position of the chain  
  boolean ret = kinematicSolver.solveIk(desiredFrame);  
  if(ret)
  {  // add to pickTrail
    pickTrail.add(endPoint.get());
    if(pickTrail.size() > pickTrailMax)
      // remove the oldest
      pickTrail.remove(0);
  }
  return ret;  
}


