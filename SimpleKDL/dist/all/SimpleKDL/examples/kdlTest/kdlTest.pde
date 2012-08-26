import peasy.*;
import SimpleKDL.*;

PeasyCam cam;


SimpleKDL.Chain    chain;

SimpleKDL.Joint    joint0;
SimpleKDL.Frame    frame0;
SimpleKDL.Segment  segment0;

SimpleKDL.Joint    joint1;
SimpleKDL.Frame    frame1;
SimpleKDL.Segment  segment1;

SimpleKDL.Joint    joint2;
SimpleKDL.Frame    frame2;
SimpleKDL.Segment  segment2;

SimpleKDL.JntArray  jointAngles;

SimpleKDL.ChainFkSolverPos_recursive   fk;
SimpleKDL.Frame                        finalFrame;

void setup() 
{
  size(800,600,P3D);
  cam = new peasy.PeasyCam(this, 100);
  cam.setMinimumDistance(50);
  cam.setMaximumDistance(500);

  SimpleKDLContext.start();
  
  // start with the chain
  chain   = new SimpleKDL.Chain();
  
  // set 1. segment of the chain
  joint0  = new SimpleKDL.Joint(Joint.JointType.RotZ); 
  frame0  = new SimpleKDL.Frame(new SimpleKDL.Vector(0.2, 0.3, 0.0));
  segment0 = new SimpleKDL.Segment(joint0,frame0);
  chain.addSegment(segment0);
  
  // set 2. segment of the chain
  joint1  = new SimpleKDL.Joint(Joint.JointType.RotZ); 
  frame1  = new SimpleKDL.Frame(new SimpleKDL.Vector(0.4, 0.0, 0.0));
  segment1 = new SimpleKDL.Segment(joint1,frame1);
  chain.addSegment(segment1);
  
  // set 3. segment of the chain
  joint2  = new SimpleKDL.Joint(Joint.JointType.RotZ); 
  frame2  = new SimpleKDL.Frame(new SimpleKDL.Vector(0.1, 0.1, 0.0));
  segment2 = new SimpleKDL.Segment(joint2,frame2);
  chain.addSegment(segment2);

  //  
  jointAngles = new SimpleKDL.JntArray(3);
  /*
  jointAngles.set(0,0.5236);
  jointAngles.set(1,0.5236);
  jointAngles.set(2,-1.5708);
  */
  
  jointAngles.set(0,radians(30));
  jointAngles.set(1,radians(30));
  jointAngles.set(2,radians(-90));

 
  fk = new SimpleKDL.ChainFkSolverPos_recursive(chain);
  finalFrame = new SimpleKDL.Frame();
  fk.JntToCart(jointAngles,finalFrame);
  
  println("Rotational Matrix of the final Frame: ");
  println(finalFrame.getM());
  println("End-effector position: " + finalFrame.getP());
}


void draw() 
{
  rotateX(-.5);
  rotateY(-.5);
  background(0);
  fill(255,0,0);
  box(30);
  pushMatrix();
  translate(0,0,20);
  fill(0,0,255);
  box(5);
  popMatrix();
}


