package roboticsProject;

import java.util.Arrays;

public class solverTest {
	
	// define lengths of 3 phalanges ("finger bones")
	public static double lP = 39.8;
	public static double lI = 22.4;
	public static double lD = 15.8;

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		
		double[] pF = {70, -18};  // desired final coordinates
		double[] q0 = {0 , -1};  // arbitrary first guess for joint angles
		double[] q1 = new double[2];
		boolean loopContinuation = true;
		double[] error = {100,100};
		int count = 0;
		
		while( loopContinuation == true ) {
			
			q1 = solver(q0,pF);
			
			// if the solver produces an estimated joint angle that is out of bounds, we correct it back back towards the possible bounds
			if ( q1[0] < -Math.PI/3.0 || q1[0] > Math.PI/3.0 ) {
				q1[0] = q1[0] - q1[0] * Math.random();
			}
			if ( q1[1] < -(2.0*Math.PI)/3.0 || q1[1] > 0 ) {
				q1[1] = q1[1] - q1[1] * Math.random();
			}
			
			error = vectorSubtraction(pF, p_i(q1));
			
			// end loop if error is sufficiently small and joint angles are within permitted bounds
			if( Math.abs(error[0]) < 0.0001 && Math.abs(error[1]) < 0.0001 &&  
					q1[0] >= -Math.PI/3.0 && q1[0] <= Math.PI/3.0 &&  
					q1[1] >= -(2.0*Math.PI)/3.0 && q1[1] <= 0 ) {
				loopContinuation = false;
			}
			
			q0 = q1;
			
			count++;
			
		}
	
		System.out.println("The joint angles (parameters) for the metacarpo-phalangeal and proximal inter-phalangeal joints are (respectively): " + Arrays.toString(q1));
		System.out.println("The total number of iterations is: " + count);
		System.out.println("The x and y coordinates of the fingertip are (respectively): " + Arrays.toString(p_i(q1)));
		System.out.println("The errors for the x and y estimates respectively are: " + Arrays.toString(error));

	}

// method defining the solver, where q_i+1 is given by q_i and p_F (the position of the desired coordinates)
public static double[] solver(double[] q, double[] p) {
	
	q = vectorAddition(q, matrixVectorMultiplication(jInv(q), vectorSubtraction(p, p_i(q)))); ;
	
	return q;
	
}

// method defining the inverse of the 2x2 Jacobian Matrix
public static double[][] jInv( double[] q ){
	
	double[][] j = new double[2][2];
	
	j[0][0] = - lP*Math.sin(q[0]) - lI*Math.sin(q[0] + q[1]) - lD*Math.sin(q[0] + (5.0/3.0) * q[1]);
	
	j[0][1] = - lI*Math.sin(q[0] + q[1]) - (5.0/3.0)*lD*Math.sin(q[0] + (5.0/3.0) * q[1]);
	
	j[1][0] = lP*Math.cos(q[0]) + lI*Math.cos(q[0] + q[1]) + lD*Math.cos(q[0] + (5.0/3.0) * q[1]);
	
	j[1][1] = lI*Math.cos(q[0] + q[1]) + (5.0/3.0)*lD*Math.cos(q[0] + (5.0/3.0) * q[1]);
	
	double factor = 1/(j[0][0]*j[1][1] - j[0][1]*j[1][0]) ;
	
	double[][] jInv = new double[2][2];
	
	jInv[0][0] = factor * j[1][1];
	
	jInv[0][1] = -1 * factor * j[0][1];
	
	jInv[1][0] = -1 * factor * j[1][0];
	
	jInv[1][1] = factor * j[0][0];
	
	return jInv;
	
}

// method defining p_i based on some q
public static double[] p_i(double[] q) {
	
	double[] p_i = new double[2];
	
	p_i[0] = lP*Math.cos(q[0]) + lI*Math.cos(q[0] + q[1]) + lD*Math.cos(q[0] + (5.0/3.0) * q[1]);
	
	p_i[1] = lP*Math.sin(q[0]) + lI*Math.sin(q[0] + q[1]) + lD*Math.sin(q[0] + (5.0/3.0) * q[1]);
	
	return p_i;
	
}

// method defining matrix vector addition for two 2x1 vectors
public static double[] vectorAddition(double[] a, double[] b){
	
	double[] c = new double[2];
	
	c[0] = a[0] + b[0];
	
	c[1] = a[1] + b[1];
	
	return c;
	
}

// method defining matrix vector subtraction for two 2x1 vectors
public static double[] vectorSubtraction(double[] a, double[] b){
	
	double[] c = new double[2];
	
	c[0] = a[0] - b[0];
	
	c[1] = a[1] - b[1];
	
	return c;
	
}

// method defining matrix vector multiplication for a 2x1 vector and 2x2 matrix
public static double[] matrixVectorMultiplication(double[][] a, double[] b){
	
	double[] c = new double[2];
	
	c[0] = a[0][0] * b[0] + a[0][1] * b[1];
	
	c[1] = a[1][0] * b[0] + a[1][1] * b[1];
	
	return c;
	
	}	
	
}
