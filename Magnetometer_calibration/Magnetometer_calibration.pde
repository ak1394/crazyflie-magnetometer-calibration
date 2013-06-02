/******************************************************************************************
* Magnetometer Sampling Sketch for Razor AHRS v1.4.1
* 9 Degree of Measurement Attitude and Heading Reference System
* for Sparkfun "9DOF Razor IMU" and "9DOF Sensor Stick"
*
* Released under GNU GPL (General Public License) v3.0
* Copyright (C) 2011-2012 Quality & Usability Lab, Deutsche Telekom Laboratories, TU Berlin
* Written by Peter Bartz (peter-bartz@gmx.de)
*
* Infos, updates, bug reports and feedback:
*     http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs
******************************************************************************************/

/*
  NOTE: You have to install a library, before this sketch can be run!
  We're using EJML for matrix math, because it's really fast:
  http://code.google.com/p/java-matrix-benchmark/
  Also, it's released under LGPL, which fits well with our GPL.
  Get the library from: http://code.google.com/p/efficient-java-matrix-library/ (you only need
  the .jar file), find your Processing "libraries" folder (normally this is Processing/libraries
  in your user documents folder). Create a folder "EJML" inside "libraries",
  create a folder "library" inside "EJML" and put the .jar inside. Rename to EJML.jar. So you
  should have "libraries/EJML/library/EJML.jar". Restart Processing and you're good.
  More info on installing libraries in Processing: http://wiki.processing.org/w/How_to_Install_a_Contributed_Library
*/
import org.ejml.data.*;
import org.ejml.simple.*;
import org.ejml.ops.*;

import java.io.*;

import processing.opengl.*;

import processing.net.*; 

Client myClient; 
String inString;

final static int NUM_MAGN_SAMPLES = 10000;
float magnetom[][] = new float[NUM_MAGN_SAMPLES][3];
int magnetomIndex = 0;
boolean finished = false;

// Global setup
void setup() {
  // Setup graphics
  size(1400, 800, OPENGL);
  smooth();
  noStroke();
  frameRate(50);
  colorMode(HSB);
  myClient = new Client(this, "192.168.33.200", 1234);
}

void draw() {
  if(finished) {
    return;
  }
  
  // Reset scene
  lights();
   
  // Output "max samples reached" message?
  if (magnetomIndex == NUM_MAGN_SAMPLES - 1) {
    fill(0, 255, 255);
    text("MAX NUMBER OF SAMPLES REACHED!", width/2, 0, -250);
    println("MAX NUMBER OF SAMPLES REACHED!");
  }
 
  pushMatrix(); {
    translate(width/2, height/2, -900);
    
    // Draw sphere and background once
    if (magnetomIndex == 0) {
      background(0);
      noFill();
      stroke(255);
      sphereDetail(10);
      sphere(400);
      fill(200);
    }
  
    // Read and draw new sample
    if (magnetomIndex < NUM_MAGN_SAMPLES) {
      // Read all available magnetometer data from serial port
      while (myClient.available() > 0) {
        inString = myClient.readStringUntil(10).trim();
        String[] list = split(inString, ' ');
        // Read magn data
        magnetom[magnetomIndex][0] = float(list[0]);  // x
        magnetom[magnetomIndex][1] = float(list[1]);  // y
        magnetom[magnetomIndex][2] = float(list[2]);  // z
        println(magnetom[magnetomIndex]);
      }
      
      // Draw new point
      fill((magnetom[magnetomIndex][2]) / 8, 255, 255);
      noStroke();
      translate(magnetom[magnetomIndex][0], magnetom[magnetomIndex][1], magnetom[magnetomIndex][2]);
      sphere(5);
      
      magnetomIndex++;
    }
  } popMatrix();
}

void keyPressed() {
  switch (key) {
    case ' ':  // Calculate and output calibration parameters, output binary magnetometer samples file, quit
    case ENTER:
    case RETURN:
      finished = true;
      outputCalibration();  // Do the magic
      exit();  // We're done
  }
}

void outputCalibration() {
  /* ELLIPSOID FITTING CODE */
  // Adaption of 'ellipsoid_fit' matlab code by Yury Petrov (See 'Matlab' folder
  // that came with the archive containing this file).
  
  // Check if we have at least 9 sample points
  if (magnetomIndex < 9) {
    println("ERROR: not enough magnetometer samples. We need at least 9 points.");
    exit();
  }
  
  // Seperate xyz magnetometer values and make column vectors
  SimpleMatrix x = new SimpleMatrix(magnetomIndex, 1);
  SimpleMatrix y = new SimpleMatrix(magnetomIndex, 1);
  SimpleMatrix z = new SimpleMatrix(magnetomIndex, 1);
  for (int i = 0; i < magnetomIndex; i++) {
    x.set(i, magnetom[i][0]);
    y.set(i, magnetom[i][1]);
    z.set(i, magnetom[i][2]);
  }
  
  
  /*
  % fit ellipsoid in the form Ax^2 + By^2 + Cz^2 + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz = 1
  D = [ x .* x, ...
        y .* y, ...
        z .* z, ...
    2 * x .* y, ...
    2 * x .* z, ...
    2 * y .* z, ...
    2 * x, ...
    2 * y, ... 
     2 * z ];  % ndatapoints x 9 ellipsoid parameters
  */
  SimpleMatrix D = new SimpleMatrix(x.numRows(), 9);
  D.insertIntoThis(0, 0, x.elementMult(x));
  D.insertIntoThis(0, 1, y.elementMult(y));
  D.insertIntoThis(0, 2, z.elementMult(z));
  D.insertIntoThis(0, 3, x.elementMult(y).scale(2));
  D.insertIntoThis(0, 4, x.elementMult(z).scale(2));
  D.insertIntoThis(0, 5, y.elementMult(z).scale(2));
  D.insertIntoThis(0, 6, x.scale(2));
  D.insertIntoThis(0, 7, y.scale(2));
  D.insertIntoThis(0, 8, z.scale(2));
  
  /*
  % solve the normal system of equations
  v = ( D' * D ) \ ( D' * ones( size( x, 1 ), 1 ) );
  */
  SimpleMatrix tempA = D.transpose().mult(D);
  SimpleMatrix ones = x.copy(); ones.set(1);
  SimpleMatrix tempB = D.transpose().mult(ones);
  SimpleMatrix v = tempA.solve(tempB);
  
  /*
  % form the algebraic form of the ellipsoid
  A = [ v(1) v(4) v(5) v(7); ...
        v(4) v(2) v(6) v(8); ...
        v(5) v(6) v(3) v(9); ...
        v(7) v(8) v(9) -1 ];
  */
  SimpleMatrix A = new SimpleMatrix(new double[][]
    {{v.get(0), v.get(3), v.get(4), v.get(6)},
     {v.get(3), v.get(1), v.get(5), v.get(7)},
     {v.get(4), v.get(5), v.get(2), v.get(8)},
     {v.get(6), v.get(7), v.get(8),     -1.0}});
  
  /*
  % find the center of the ellipsoid
  center = -A( 1:3, 1:3 ) \ [ v(7); v(8); v(9) ];
  */
  SimpleMatrix center = A.extractMatrix(0, 3, 0, 3).scale(-1).solve(v.extractMatrix(6, 9, 0, 1));

  /*
  % form the corresponding translation matrix
  T = eye( 4 );
  T( 4, 1:3 ) = center';
  */
  SimpleMatrix T = SimpleMatrix.identity(4);
  T.insertIntoThis(3, 0, center.transpose());
  
  /*
  % translate to the center
  R = T * A * T';
  % solve the eigenproblem
  [ evecs evals ] = eig( R( 1:3, 1:3 ) / -R( 4, 4 ) );
  radii = sqrt( 1 ./ diag( evals ) );
  */
  SimpleMatrix R = T.mult(A).mult(T.transpose());
  SimpleEVD evd = R.extractMatrix(0, 3, 0, 3).divide(-R.get(3, 3)).eig();

  SimpleMatrix evecs = new SimpleMatrix(3, 3);
  evecs.insertIntoThis(0, 0, evd.getEigenVector(0));
  evecs.insertIntoThis(0, 1, evd.getEigenVector(1));
  evecs.insertIntoThis(0, 2, evd.getEigenVector(2));
  
  SimpleMatrix radii = new SimpleMatrix(new double[][]
    {{Math.sqrt(1.0 / evd.getEigenvalue(0).getReal()),
      Math.sqrt(1.0 / evd.getEigenvalue(1).getReal()),
      Math.sqrt(1.0 / evd.getEigenvalue(2).getReal())}});

  //center.print();
  //evecs.print();
  //radii.print();

  
  /* CALCULATE COMPENSATION MATRIX */
  // Adaption of my 'magnetometer_calibration' matlab code. (See 'Matlab' folder
  // that came with the archive containing this file).
  /*
  % compensate distorted magnetometer data
  % e_eigenvecs is an orthogonal matrix, so ' can be used instead of inv()
  scale = inv([e_radii(1) 0 0; 0 e_radii(2) 0; 0 0 e_radii(3)]) * min(e_radii); % scaling matrix
  map = e_eigenvecs'; % transformation matrix to map ellipsoid axes to coordinate system axes
  invmap = e_eigenvecs; % inverse of above
  comp = invmap * scale * map;
  */
  SimpleMatrix scale = new SimpleMatrix(3, 3);
  scale.zero();
  scale.set(0, 0, radii.get(0));
  scale.set(1, 1, radii.get(1));
  scale.set(2, 2, radii.get(2));
  scale = scale.invert().scale(CommonOps.elementMin(radii.getMatrix()));
  
  SimpleMatrix map = evecs.transpose();
  SimpleMatrix invmap = evecs;
  SimpleMatrix comp = invmap.mult(scale).mult(map);
  //comp.print();
  
  /* OUTPUT RESULTS */
  // Output magnetometer samples file
  try {
    println("Trying to write " + magnetomIndex + " sample points to file magnetom.float ...");
    FileOutputStream fos = new FileOutputStream(sketchPath("magnetom.float"));
    DataOutputStream dos = new DataOutputStream(fos);
    for (int i = 0; i < magnetomIndex; i++) {
      dos.writeFloat(magnetom[i][0]);
      dos.writeFloat(magnetom[i][1]);
      dos.writeFloat(magnetom[i][2]);
    }
    fos.close();
    println("Done.");
  } catch(Exception e) {
    println("Exception: " + e.toString());
  }
  println("\n");

  // Output calibration
  System.out.printf("magn_ellipsoid_center = [%.6g, %.6g, %.6g]\n", center.get(0), center.get(1), center.get(2));
  System.out.printf("magn_ellipsoid_transform = [[%.6g, %.6g, %.6g], [%.6g, %.6g, %.6g], [%.6g, %.6g, %.6g]]\n",
    comp.get(0), comp.get(1), comp.get(2), comp.get(3), comp.get(4), comp.get(5), comp.get(6), comp.get(7), comp.get(8));
  println("\n");
}
