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

PVector boxCenter = new PVector(500,200,100);
PVector boxSize = new PVector(300,200,150);

  // define picking plane x-y
PVector pickPlaneP1 = new PVector(0, 0, 0);
PVector pickPlaneP2 = new PVector(1, 0, 0);
PVector pickPlaneP3 = new PVector(0, 1, 0);
boolean lastPick = false;


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
  
  minAngles[0] = -990.0;
  maxAngles[0] = 961.0;

  minAngles[1] = -20;
  maxAngles[1] = 90;

  minAngles[2] = -20;
  maxAngles[2] = 170;

  minAngles[3] = -120;
  maxAngles[3] = 120;
  
  kinematicSolver = new SimpleKDL.KinematicSolver();
  
  kinematicSolver.init(chain,minAngles,maxAngles);
  

}

void mousePressed()
{
  if((mouseButton == LEFT) && (keyPressed == true) && (key == CODED) && (keyCode == SHIFT))
  {
    noCursor();
    mousePick = 1;
    // switch off the left dragging
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
    // switch on the left dragging
    cam.setActive(true);
  }    
  
}

void draw() 
{
  // set lights
  ambientLight(100, 100,100);
  directionalLight(200, 172, 235,
                   1, -1, 0);
                   
  background(0);

  // draw chain
  kinematicSolver.draw();

  // draw picking plane
  pushStyle();
  strokeWeight(1);
  stroke(150, 150, 150, 100);
  Utils.inst().drawPlane(pickPlaneP1, pickPlaneP2, pickPlaneP3, 
                         2000, 21);
  popStyle();  

  // draw pick pos
  pushStyle();
  pushMatrix();
  applyMatrix(endPointMat);
  strokeWeight(5);  
  Utils.inst().drawCoordSys(50);
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

  // draw pickScene  
  pushMatrix();
    fill(185, 255, 69,100);
    stroke(255);
    strokeWeight(1);
    translate(boxCenter.x,boxCenter.y,boxCenter.z);
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
/*
  switch(key)
  {
  case ' ':
    pickScene();
    break;
  case 'x':
    // reset
    q_out = new SimpleKDL.JntArray(jointAngles);
    q_init = new SimpleKDL.JntArray(jointAngles);
    break;
  case '1':
    // print the angels
    println("q_init");
    printAngles(q_init);
    println("q_out");
    printAngles(q_out);
    break;
  case 'f':
    forwardKinematicTest();
    break;
  case 'i':
    // change inverse kinematic type
    ikType++;
    if (ikType > 1)
      ikType = 0;

    switch(ikType)
    {
    case 0:
      println("IkType normal");
      break;
    case 1:
      println("IkType jointLimited");
      break;
    }
    break;
  case 'p':
    // change the picking plane
    planeType++;
    if (planeType > 2)
      planeType = 0;

    switch(planeType)
    {
    case 0:
      // define picking plane x-y
      pickPlaneP1 = new PVector(0, 0, 0);
      pickPlaneP2 = new PVector(1, 0, 0);
      pickPlaneP3 = new PVector(0, 1, 0);
      ;      
      break;
    case 1:
      // define picking plane x-z
      pickPlaneP1 = new PVector(0, 0, 0);
      pickPlaneP2 = new PVector(1, 0, 0);
      pickPlaneP3 = new PVector(0, 0, 1);
      ;      
      break;
    case 2:
      // define picking plane y-z
      pickPlaneP1 = new PVector(0, 0, 0);
      pickPlaneP2 = new PVector(0, 1, 0);
      pickPlaneP3 = new PVector(0, 0, 1);
      ;      
      break;
    }
    break;
  }
  */
}

void pickScene()
{
  PVector r1 = new PVector();
  PVector r2 = new PVector();
  
  // get the hit ray
  Utils.inst().getHitRay(mouseX,mouseY,
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
  if(RayIntersection.box(r1, rDir, 
                         boxCenter,boxSize.x,boxSize.y,boxSize.z,
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
  
  println("u" + u);
  println("v" + v);
  println("normalEnd" + normalEnd);

/*  
 SimpleKDL.Frame testFrame = Utils.getFrame(// orig
                                            new PVector(0,0,0),  // x
                                            new PVector(1,0,0),  // x
                                            new PVector(0,1,0),  // y
                                            new PVector(0,0,1),  // z
                                            // dest
                                            new PVector(0,0,0),  // z
                                            u,
                                            v,
                                            normalEnd);
                                            */
  SimpleKDL.Frame testFrame = Utils.getFrame(// orig
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
  println("------------------------------");
  println("Set new inverse kinematics position");
  
  endPoint = endPos.get();
  
  // flip the direction of the normal, because the robot end segment looks down
 // endPointNormal = PVector.mult(endPosNormal,-1);
  endPointNormal = endPosNormal.get();
  println("endPointNormal: "+ endPointNormal);
  
  // calculate rotation frame according to chain base
  //endPointRotX
  calcEndRotMat(new PVector(0,0,0),
                endPoint,
                endPointNormal);
  
  SimpleKDL.Frame desiredFrame = new SimpleKDL.Frame(new SimpleKDL.Rotation(new SimpleKDL.Vector(endPointRotX.x,endPointRotX.y,endPointRotX.z),
                                                                            new SimpleKDL.Vector(endPointRotY.x,endPointRotY.y,endPointRotY.z),
                                                                            new SimpleKDL.Vector(endPointRotZ.x,endPointRotZ.y,endPointRotZ.z)),
                                                     new SimpleKDL.Vector(endPos.x, endPos.y, endPos.z));
 
                          
  endPointMat = Utils.getMatrix(desiredFrame);
  
//  println("Desired Position: " + desiredFrame.getP());
  
  return kinematicSolver.solveIk(desiredFrame);  
}


