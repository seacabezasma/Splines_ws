/**
 * Splines.
 *
 * Here we use the interpolator.keyFrames() nodes
 * as control points to render different splines.
 *
 * Press ' ' to change the spline mode.
 * Press 'g' to toggle grid drawing.
 * Press 'c' to toggle the interpolator path drawing.
 */

import frames.input.*;
import frames.primitives.*;
import frames.core.*;
import frames.processing.*;
import java.util.List;

// global variables
// modes: 0 natural cubic spline; 1 Hermite;
// 2 (degree 7) Bezier; 3 Cubic Bezier
int mode;

Scene scene;
Interpolator interpolator;
OrbitNode eye;
boolean drawGrid = true, drawCtrl = true;
Vector p0, p1, p2;
float a_y, b_y, c_y, d_y, a_x, b_x, c_x, d_x, t, y_t, x_t, z_t, 
  a_z, b_z, c_z, d_z;

//Choose P3D for a 3D scene, or P2D or JAVA2D for a 2D scene
String renderer = P3D;

void setup() {
  size(800, 800, renderer);
  scene = new Scene(this);
  eye = new OrbitNode(scene);
  eye.setDamping(0);
  scene.setEye(eye);
  scene.setFieldOfView(PI / 3);
  //interactivity defaults to the eye
  scene.setDefaultGrabber(eye);
  scene.setRadius(150);
  scene.fitBallInterpolation();
  interpolator = new Interpolator(scene, new Frame());
  // framesjs next version, simply go:
  //interpolator = new Interpolator(scene);

  // Using OrbitNodes makes path editable
  for (int i = 0; i < 8; i++) {
    Node ctrlPoint = new OrbitNode(scene);
    ctrlPoint.randomize();
    interpolator.addKeyFrame(ctrlPoint);
  }
}

void draw() {
  background(175);
  if (drawGrid) {
    stroke(255, 255, 0);
    scene.drawGrid(200, 50);
  }
  if (drawCtrl) {
    fill(255, 0, 0);
    stroke(255, 0, 255);
    for (Frame frame : interpolator.keyFrames())
      scene.drawPickingTarget((Node)frame);
  } else {
    fill(255, 0, 0);
    stroke(255, 0, 255);
    scene.drawPath(interpolator);
  }
  // implement me
  // draw curve according to control polygon an mode
  // To retrieve the positions of the control points do:
  // for(Frame frame : interpolator.keyFrames())
  //   frame.position();
  
  switch(mode)
  {
    case 0:
      drawNatural();
    break;
    
    case 1:
      drawHermiteCatmull(false);
    break;
    
    case 2:
      drawSevenBezier();
    break;
    
    case 3:
      drawCubicBezier();
    break;
    default:
    
  }
  
}

void keyPressed() {
  if (key == ' ')
    mode = mode < 3 ? mode+1 : 0;
  if (key == 'g')
    drawGrid = !drawGrid;
  if (key == 'c')
    drawCtrl = !drawCtrl;
}

void drawNatural()
{
  //points from interpolator
  List<Frame> points = interpolator.keyFrames();
  
  //total points amount
  int dm = points.size();
  //coeficient matrix
  Matrix matrix = new Matrix(dm, dm);
  //x and y axises constants
  Matrix rhs_y = new Matrix(dm, 1);
  Matrix rhs_x = new Matrix(dm, 1);
  Matrix rhs_z = new Matrix(dm, 1);
  
  //fill the matrix
  for(int i=0; i<dm; i++)
  {
    for(int j=0; j<dm; j++)
    {
      if(i==j)
        matrix.data[i][j] = 4;
      else if(j-i == 1 || i-j == 1)
        matrix.data[i][j] = 1;
    }
  }
  
  //fill the constants vectors 
  for(int i=1; i<dm-1; i++)
  {
    p1 = points.get(i-1).position();
    p2 = points.get(i+1).position();
    rhs_y.data[i][0] = 3*(p2.y() - p1.y());
    rhs_x.data[i][0] = 3*(p2.x() - p1.x());
    rhs_z.data[i][0] = 3*(p2.z() - p1.z());
  }
  
  rhs_y.data[0][0] = 3*(points.get(1).position().y() - points.get(0).position().y());
  rhs_y.data[dm-1][0] = 3*(points.get(dm-1).position().y() - points.get(dm-2).position().y());
  
  rhs_x.data[0][0] = 3*(points.get(1).position().x() - points.get(0).position().x());
  rhs_x.data[dm-1][0] = 3*(points.get(dm-1).position().x() - points.get(dm-2).position().x());

  rhs_z.data[0][0] = 3*(points.get(1).position().z() - points.get(0).position().z());
  rhs_z.data[dm-1][0] = 3*(points.get(dm-1).position().z() - points.get(dm-2).position().z());  
  
  matrix.data[0][0] = 2;
  matrix.data[dm-1][dm-1] = 2;
  
  //find the solutions
  Matrix coef_y = matrix.solve(rhs_y);
  Matrix coef_x = matrix.solve(rhs_x);
  Matrix coef_z = matrix.solve(rhs_z);
  
  //find the coeficients per each axis
  for(int i=0; i<dm-1; i++)
  {
    p1 = points.get(i).position();
    p2 = points.get(i+1).position();
    
    a_y = p1.y();
    b_y = coef_y.data[i][0];
    c_y = 3*(p2.y() - p1.y()) - 2*coef_y.data[i][0] - coef_y.data[i+1][0];
    d_y = 2*(p1.y() - p2.y()) + coef_y.data[i][0] + coef_y.data[i+1][0];
    
    a_x = p1.x();
    b_x = coef_x.data[i][0];
    c_x = 3*(p2.x() - p1.x()) - 2*coef_x.data[i][0] - coef_x.data[i+1][0];
    d_x = 2*(p1.x() - p2.x()) + coef_x.data[i][0] + coef_x.data[i+1][0];
    
    a_z = p1.z();
    b_z = coef_z.data[i][0];
    c_z = 3*(p2.z() - p1.z()) - 2*coef_z.data[i][0] - coef_z.data[i+1][0];
    d_z = 2*(p1.z() - p2.z()) + coef_z.data[i][0] + coef_z.data[i+1][0];
    
    //draw the curves
    for(int j=0; j<900; j++)
    {
      t = norm(j, 0, 900);
      y_t = d_y*pow(t, 3) + c_y*sq(t) + b_y*t + a_y;
      x_t = d_x*pow(t, 3) + c_x*sq(t) + b_x*t + a_x;
      z_t = d_z*pow(t, 3) + c_z*sq(t) + b_z*t + a_z;
      stroke(0);
      //strokeWeight(2);
      point(x_t, y_t, z_t);
    }
    
  }
}


void drawHermiteCatmull(boolean catmull)
{
  //points from interpolator
  List<Frame> points = interpolator.keyFrames();
  
  int dm = points.size();
  float tension = 0.5;
  
  //Main coeficient matrix M_h for Cubic Hermit Spline
  Matrix matrix = new Matrix(4, 4);
  
  //filliing row 0
  matrix.data[0][0] = 2;
  matrix.data[0][1] = -2;
  matrix.data[0][2] = 1;
  matrix.data[0][3] = 1;
  
  //filliing row 1
  matrix.data[1][0] = -3;
  matrix.data[1][1] = 3;
  matrix.data[1][2] = -2;
  matrix.data[1][3] = -1;
  
  //filliing row 2
  matrix.data[2][0] = 0;
  matrix.data[2][1] = 0;
  matrix.data[2][2] = 1;
  matrix.data[2][3] = 0;
  
  //filliing row 3
  matrix.data[3][0] = 1;
  matrix.data[3][1] = 0;
  matrix.data[3][2] = 0;
  matrix.data[3][3] = 0;
  
  //Constants Vector
  Matrix rhs_y = new Matrix(4, 1);
  Matrix rhs_x = new Matrix(4, 1);
  Matrix rhs_z = new Matrix(4, 1);
  
  Matrix coef_y, coef_x, coef_z;
  if (catmull) tension = 0.0;
  
  //Using Cardinal aproach
  for(int i=1; i<dm-2; i++)
  {
    p0 = points.get(i-1).position();
    p1 = points.get(i).position();
    p2 = points.get(i+1).position();
    
    rhs_y.data[0][0] = p1.y();
    rhs_x.data[0][0] = p1.x();
    rhs_z.data[0][0] = p1.z();
    
    rhs_y.data[1][0] = p2.y();
    rhs_x.data[1][0] = p2.x();
    rhs_z.data[1][0] = p2.z();
    
    rhs_y.data[2][0] = 0.5*(1-tension)*(p2.y() - p0.y());
    rhs_x.data[2][0] = 0.5*(1-tension)*(p2.x() - p0.x());
    rhs_z.data[2][0] = 0.5*(1-tension)*(p2.z() - p0.z());
    
    p2 = points.get(i+2).position();
    rhs_y.data[3][0] = 0.5*(1-tension)*(p2.y() - p1.y());
    rhs_x.data[3][0] = 0.5*(1-tension)*(p2.x() - p1.x());
    rhs_z.data[3][0] = 0.5*(1-tension)*(p2.z() - p1.z());
    
    //getting coeficients
    coef_y = matrix.times(rhs_y);
    coef_x = matrix.times(rhs_x);
    coef_z = matrix.times(rhs_z);
    
    for(int j=0; j<900; j++)
    {
      t = norm(j, 0, 900);
      y_t = coef_y.data[0][0]*pow(t, 3) + coef_y.data[1][0]*sq(t) + coef_y.data[2][0]*t + coef_y.data[3][0];
      x_t = coef_x.data[0][0]*pow(t, 3) + coef_x.data[1][0]*sq(t) + coef_x.data[2][0]*t + coef_x.data[3][0];
      z_t = coef_z.data[0][0]*pow(t, 3) + coef_z.data[1][0]*sq(t) + coef_z.data[2][0]*t + coef_z.data[3][0];
      stroke(0);
      //strokeWeight(3);
      point(x_t, y_t, z_t);
    }
  }
  
}

void drawSevenBezier()
{
  //points from interpolator
  List<Frame> points = interpolator.keyFrames();
  
  //drawing using the polynomial definition 
  for(int i=0; i<900; i++)
  {
    t = norm(i, 0, 900);
    
    x_t = pow((1-t),7)*points.get(0).position().x() + 
          7*pow(1-t, 6)*t*points.get(1).position().x() + 
          21*pow(1-t, 5)*sq(t) * points.get(2).position().x() + 
          35*pow(1-t, 4)*pow(t, 3)*points.get(3).position().x() + 
          35*pow(1-t, 3)*pow(t, 4)*points.get(4).position().x() + 
          21*pow(1-t, 2)* pow(t, 5)*points.get(5).position().x() + 
          7*pow(1-t, 1)* pow(t, 6)*points.get(6).position().x() + 
          pow(t, 7)*points.get(7).position().x();
          
     y_t = pow((1-t),7)*points.get(0).position().y() + 
          7*pow(1-t, 6)*t*points.get(1).position().y() + 
          21*pow(1-t, 5)*sq(t) * points.get(2).position().y() + 
          35*pow(1-t, 4)*pow(t, 3)*points.get(3).position().y() + 
          35*pow(1-t, 3)*pow(t, 4)*points.get(4).position().y() + 
          21*pow(1-t, 2)* pow(t, 5)*points.get(5).position().y() + 
          7*pow(1-t, 1)* pow(t, 6)*points.get(6).position().y() + 
          pow(t, 7)*points.get(7).position().y();
          
     z_t = pow((1-t),7)*points.get(0).position().z() + 
          7*pow(1-t, 6)*t*points.get(1).position().z() + 
          21*pow(1-t, 5)*sq(t) * points.get(2).position().z() + 
          35*pow(1-t, 4)*pow(t, 3)*points.get(3).position().z() + 
          35*pow(1-t, 3)*pow(t, 4)*points.get(4).position().z() + 
          21*pow(1-t, 2)* pow(t, 5)*points.get(5).position().z() + 
          7*pow(1-t, 1)* pow(t, 6)*points.get(6).position().z() + 
          pow(t, 7)*points.get(7).position().z();
          
     stroke(0);
     //strokeWeight(3);
     point(x_t, y_t, z_t);
  }
}

void drawCubicBezier()
{
  //points from interpolator
  List<Frame> points = interpolator.keyFrames();
  
  //control polygon points vector
  Matrix rhs_y = new Matrix(4, 1);
  Matrix rhs_x = new Matrix(4, 1);
  Matrix rhs_z = new Matrix(4, 1);
  Matrix coef_y, coef_x, coef_z;
  
  Matrix matrix = new Matrix(4, 4);
  
  //filliing row 0
  matrix.data[0][0] = -1;
  matrix.data[0][1] = 3;
  matrix.data[0][2] = -3;
  matrix.data[0][3] = 1;
  
  //filliing row 1
  matrix.data[1][0] = 3;
  matrix.data[1][1] = -6;
  matrix.data[1][2] = 3;
  matrix.data[1][3] = 0;
  
  //filliing row 2
  matrix.data[2][0] = -3;
  matrix.data[2][1] = 3;
  matrix.data[2][2] = 0;
  matrix.data[2][3] = 0;
  
  //filliing row 3
  matrix.data[3][0] = 1;
  matrix.data[3][1] = 0;
  matrix.data[3][2] = 0;
  matrix.data[3][3] = 0;
  
  
  //p1 = poly.points.get(0).position; 
  int dm = points.size();
  
  //parameters for each axis
  coef_x = new Matrix(1,4);
  coef_y = new Matrix(1,4);
  coef_z = new Matrix(1,4);
  
  for(int i=0; i<dm; i++)
  {
    if(i%3 == 0 && dm-i >= 3)
    {
      if(i-1 > 0)
      {
        p1 = points.get(i-1).position();
        p2 = points.get(i+1).position();
        
        //forcing continuity on each point which is a final and starting control point of each bezier curve 
        Frame p = points.get(i);
        p.setPosition((0.5*p1.x()) + (0.5 * p2.x()), (0.5*p1.y()) + (0.5 * p2.y()), (0.5*p1.z()) + (0.5 * p2.z()));
          
        points.set(i, p);
      }
      for(int j=0; j<=3; j++)
      {
        Vector p3 = points.get(i+j).position();
        rhs_x.data[j][0] = p3.x();
        rhs_y.data[j][0] = p3.y();
        rhs_z.data[j][0] = p3.z();
      }
      
      for(int k=0; k<900; k++)
      {
        t = norm(k, 0, 900);
        //x coordinate parameters
        coef_x.data[0][0] = pow(t, 3);
        coef_x.data[0][1] = pow(t, 2);
        coef_x.data[0][2] = t;
        coef_x.data[0][3] = 1;
        
        //y coordinate parameters
        coef_y.data[0][0] = pow(t, 3);
        coef_y.data[0][1] = pow(t, 2);
        coef_y.data[0][2] = t;
        coef_y.data[0][3] = 1;

        //y coordinate parameters
        coef_z.data[0][0] = pow(t, 3);
        coef_z.data[0][1] = pow(t, 2);
        coef_z.data[0][2] = t;
        coef_z.data[0][3] = 1;
        
        Matrix res_x = coef_x.times(matrix.times(rhs_x));
        Matrix res_y = coef_y.times(matrix.times(rhs_y));
        Matrix res_z = coef_z.times(matrix.times(rhs_z));
        
        stroke(0);
        //strokeWeight(3);
        point(res_x.data[0][0], res_y.data[0][0], res_z.data[0][0]);
      }
    }
  }
}
