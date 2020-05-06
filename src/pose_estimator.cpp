
/*  @File: pose_estimator.cpp

    @Brief: Implements the perspective-n-point algorithm described 
            in http://rpg.ifi.uzh.ch/docs/CVPR11_kneip.pdf to perform localization for the drone.

    @Author: Matthew Page

    @Date: 1/4/2020

*/

#include<math.h>
#include<iostream>
#include<complex>
#include"Eigen/Dense"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Point.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <zbar.h>


class PoseEstimator{

	public:

	PoseEstimator() 
	    : it(node){

                // Initialize ROS Publishers and Subscribers
  		accuratePos = node.advertise<geometry_msgs::Point>("/accuratePos", 1);

  		cvPub = it.advertise("/cvImage", 1);

  		imageSub = it.subscribe("/ardrone/front/image_raw", 1, &PoseEstimator::image_callback, this);
	}

        /* @Brief: Calculates the coefficient a0 in the quartic equation
           @Param: phi_1 - The calculated parapmeter phi_1
           @Param: phi_2 - The calculated parapmeter phi_2
           @Param: p1 - The calculated parapmeter p1
           @Param: p2 - The calculated parapmeter p2
           @Param: d12 - The distance between points 1 and 2
           @Param: b - The calculated parapmeter b
           @Return: The quartic coefficient a0
        */
	double calc_a0(double phi_1, double phi_2, double p1, double p2, double d12, double b) {
		return -2 * phi_1 * phi_2 * p1 * pow(p2, 2) * d12 * b
			+ pow(phi_2, 2) * pow(p2, 2) * pow(d12, 2) + 2 * pow(p1, 3) * d12
			- pow(p1, 2) * pow(d12, 2) + pow(phi_2, 2) * pow(p1, 2) * pow(p2, 2)
			- pow(p1, 4) - 2 * pow(phi_2, 2) * p1 * pow(p2, 2) * d12
			+ pow(phi_1, 2) * pow(p1, 2) * pow(p2, 2) + pow(phi_2, 2) * pow(p2, 2) * pow(d12, 2) * pow(b, 2);
	}


        /* @Brief: Calculates the coefficient a1 in the quartic equation
           @Param: phi_1 - The calculated parapmeter phi_1
           @Param: phi_2 - The calculated parapmeter phi_2
           @Param: p1 - The calculated parapmeter p1
           @Param: p2 - The calculated parapmeter p2
           @Param: d12 - The distance between points 1 and 2
           @Param: b - The calculated parapmeter b
           @Return: The quartic coefficient a1
        */
	double calc_a1(double phi_1, double phi_2, double p1, double p2, double d12, double b) {
		return 2 * pow(p1, 2) * p2 * d12 * b + 2 * phi_1 * phi_2 * pow(p2, 3) * d12
			- 2 * pow(phi_2, 2) * pow(p2, 3) * d12 * b - 2 * p1 * p2 * pow(d12, 2) * b;
	}


        /* @Brief: Calculates the coefficient a2 in the quartic equation
           @Param: phi_1 - The calculated parapmeter phi_1
           @Param: phi_2 - The calculated parapmeter phi_2
           @Param: p1 - The calculated parapmeter p1
           @Param: p2 - The calculated parapmeter p2
           @Param: d12 - The distance between points 1 and 2
           @Param: b - The calculated parapmeter b
           @Return: The quartic coefficient a2
        */
	double calc_a2(double phi_1, double phi_2, double p1, double p2, double d12, double b) {
		return -pow(phi_2, 2) * pow(p1, 2) * pow(p2, 2) - pow(phi_2, 2) * pow(p2, 2) * pow(d12, 2) * pow(b, 2)
			- pow(phi_2, 2) * pow(p2, 2) * pow(d12, 2) + pow(phi_2, 2) * pow(p2, 4) + pow(phi_1, 2) * pow(p2, 4)
			+ 2 * p1 * pow(p2, 2) * d12 + 2 * phi_1 * phi_2 * p1 * pow(p2, 2) * d12 * b
			- pow(phi_1, 2) * pow(p1, 2) * pow(p2, 2) + 2 * pow(phi_2, 2) * p1 * pow(p2, 2) * d12
			- pow(p2, 2) * pow(d12, 2) * pow(b, 2) - 2 * pow(p1, 2) * pow(p2, 2);
	}


        /* @Brief: Calculates the coefficient a3 in the quartic equation
           @Param: phi_1 - The calculated parapmeter phi_1
           @Param: phi_2 - The calculated parapmeter phi_2
           @Param: p1 - The calculated parapmeter p1
           @Param: p2 - The calculated parapmeter p2
           @Param: d12 - The distance between points 1 and 2
           @Param: b - The calculated parapmeter b
           @Return: The quartic coefficient a3
        */
	double calc_a3(double phi_1, double phi_2, double p1, double p2, double d12, double b) {
		return 2 * pow(p2, 3) * d12 * b + 2 * pow(phi_2, 2) * pow(p2, 3) * d12 * b
			- 2 * phi_1 * phi_2 * pow(p2, 3) * d12;
	}


        /* @Brief: Calculates the coefficient a4 in the quartic equation
           @Param: phi_1 - The calculated parapmeter phi_1
           @Param: phi_2 - The calculated parapmeter phi_2
           @Param: p1 - The calculated parapmeter p1
           @Param: p2 - The calculated parapmeter p2
           @Param: d12 - The distance between points 1 and 2
           @Param: b - The calculated parapmeter b
           @Return: The quartic coefficient a4
        */
	double calc_a4(double phi_1, double phi_2, double p1, double p2, double d12, double b) {
		return -pow(phi_2, 2) * pow(p2, 4) - pow(phi_1, 2) * pow(p2, 4) - pow(p2, 4);
	}

        /* @Brief: Calculates the Euclidean distance between two points
           @Param: p1 - Point 1
           @Param: p2 - Point 2
           @Return: The distance between points 1 and 2
        */
	double calc_distance(Eigen::Vector3d p1, Eigen::Vector3d p2) {
		return sqrt(pow(p2(0) - p1(0), 2) + pow(p2(1) - p1(1), 2) + pow(p2(2) - p1(2), 2));
	}

        /* @Brief: Calculates the parameter b used in calculating the quartic coefficients
           @Param: f1 - the first feature vector
           @Param: f2 - the second feature vector
           @Return: The parameter b
        */
	double calc_b(Eigen::Vector3d f1, Eigen::Vector3d f2) {
		double dot = f1.dot(f2);
		double b = sqrt((1 / (1 - pow(dot, 2))) - 1);
		if (dot < 0)
			return -1 * b;
		else return b;
	}
 
        /* @Brief: calculates the discriminant of fourth degree polynomial of the form ax^4 + bx^3 + cx^2 + dx + e
           @Param: a - The first coefficient
           @Param: b - The second coefficient
           @Param: c - The third coefficient
           @Param: d - The fourth coefficient
           @Param: e - The fifth coefficient
           @Return: The disciminant
        */
	double clac_discriminant(double a, double b, double c, double d, double e) {
		return 256 * pow(a, 3) * pow(e, 3) - 192 * pow(a, 2) * b * d * pow(e, 2) - 128 * pow(a, 2) * pow(c, 2) * pow(e, 2)
			+ 144 * a * a * c * d * d * e - 27 * a * a * d * d * d * d + 144 * a * b * b * c * e * e
			- 6 * a * b * b * d * d * e - 80 * a * b * c * c * d * e + 18 * a * b * c * d * d * d + 16 * a * pow(c, 4) * e
			- 4 * a * c * c * c * d * d - 27 * pow(b, 4) * e * e + 18 * b * b * b * c * d * e - 4 * pow(b, 3) * pow(d, 3)
			- 4 * b * b * pow(c, 3) * e + b * b * c * c * d *d;
	}
 
        /* @Brief: solves a cubic equation of the form ax^3 + bx^2 + cx + d = 0
           @Param: a - The first coefficient
           @Param: b - The second coefficient
           @Param: c - The third coefficient
           @Param: d - The fourth coefficient
           @Return: The three roots of the cubic equation
        */
	Eigen::Vector3cd solveCubic(double a_, double b_, double c_, double d_) {

		std::complex<double> i_const(0, 1.0);

        	std::complex<double> a(a_, 0);
		std::complex<double> b(b_, 0);
		std::complex<double> c(c_, 0);
		std::complex<double> d(d_, 0);

		std::complex<double> f = (((3.0 * c) / a) - (b*b / (a * a))) / 3.0;
		std::complex<double> g = ((2.0 * pow(b, 3) / pow(a, 3)) - (9.0 * b*c / (a * a)) + (27.0 * d / a)) / 27.0;
		std::complex<double> h = (g*g / 4.0) + (pow(f, 3) / 27.0);

		std::complex<double> x1;
		std::complex<double> x2;
		std::complex<double> x3;

                // case 1: All roots Real and Equal
		if ((f.real() >= -0.001 && f.real() <= 0.001) && (g.real() >= -0.001 && g.real() <= 0.001) && (h.real() >= -0.001 && h.real() <= 0.001)) {
			x1 = pow(abs(d / a), 1.0 / 3.0) * (-1.0);
			if ((d.real() / a.real()) < 0)
				x1 *= (-1);
			x2 = x1;
			x3 = x1;
		}

                // case 2: All roots are Real
		else if (h.real() <= 0.0001){
			std::complex<double> i = sqrt((g*g / 4.0) - h);
			std::complex<double> j = pow(i, 1.0 / 3.0);
			std::complex<double> k = acos(-(g / (2.0*i)));
			std::complex<double> L = j * -1.0;
			std::complex<double> M = cos(k / 3.0);
			std::complex<double> N = sqrt(3.0) * sin(k / 3.0);
			std::complex<double> P = (b / (3.0 * a)) * (-1.0);
			x1 = 2.0*j * cos(k / 3.0) - (b / (3.0 * a));
			x2 = L * (M + N) + P;
			x3 = L * (M - N) + P;
		}

                // case 3: One Real, Two Complex roots
		else if (h.real() > 0) {
			std::complex<double> R = -(g / 2.0) + sqrt(h);
			std::complex<double> S = pow(R, 1.0 / 3.0);
			std::complex<double> T = -(g / 2.0) - sqrt(h);
			std::complex<double> U = pow(abs(T), 1.0 / 3.0);
			if (T.real() < 0)
				U *= (-1);
			x1 = (S + U) - (b / (3.0 * a));
			x2 = -(S + U) / 2.0 - (b / (3.0 * a)) + (i_const * (S - U) * sqrt(3.0)) / 2.0;
			x3 = -(S + U) / 2.0 - (b / (3.0 * a)) - (i_const * (S - U) * sqrt(3.0)) / 2.0;
		}
	
		Eigen::Vector3cd roots;

		roots(0) = x1;
		roots(1) = x2;
		roots(2) = x3;

		std::cout << "cubic roots:" << std::endl;
		std::cout << roots << std::endl;

		return roots;
	}

        /* @Brief: solves a fourth degree function of the form ax^4 + bx^3 + cx^2 + dx + e = 0.
           @Algorithm: function reduces the polynomial to cubic and uses solveCubic to find roots
           @Param: a - The first coefficient
           @Param: b - The second coefficient
           @Param: c - The third coefficient
           @Param: d - The fourth coefficient
           @Param: e - The fifth coefficient
           @Return: The four roots of the quartic equation
        */  
	Eigen::Vector4cd solveQuartic(double a, double b, double c, double d, double e) {
		b = b / a;
		c = c / a;
		d = d / a;
		e = e / a;
		a = a / a;

		double p = (8 * c - 3 * b*b) / 8.0;
		double q = (b*b*b - 4 * b*c + 8*d) / 8.0;
		double r = (-3 * pow(b, 4) + 256 * e - 64 * b*d + 16 * b*b*c) / 256.0;

		Eigen::Vector3cd cubicSolutions = solveCubic(1, p / 2.0, ((p*p - 4.0*r) / 16.0), -q*q / 64.0);

        	std::cout << "Cubic Roots Test: " << std::endl << cubicSolutions << std::endl;


		std::complex<double> P = sqrt(cubicSolutions(0));
		std::complex<double> Q = sqrt(cubicSolutions(1));

                // check if first cubic root is zero
		if ((cubicSolutions(0).real() >= -0.000001) && (cubicSolutions(0).real() <= 0.000001) && (cubicSolutions(0).imag() >= -0.000001) && (cubicSolutions(0).imag() <= 0.000001))
			P = sqrt(cubicSolutions(2));

                // check if second cubic root is zero
		if ((cubicSolutions(1).real() >= -0.000001) && (cubicSolutions(1).real() <= 0.000001) && (cubicSolutions(1).imag() >= -0.000001) && (cubicSolutions(1).imag() <= 0.000001))
			Q = sqrt(cubicSolutions(2));

                // for each cubic root
		for(int i = 0; i < 3; i++){
                        // check if root is complex
			if (!((cubicSolutions(i).imag() >= -0.000001) && (cubicSolutions(i).imag() <= 0.000001))) {
				if (!(cubicSolutions((i + 1) % 3).imag() >= -0.000001 && cubicSolutions((i + 1) % 3).imag() <= 0.000001)){
					P = sqrt(cubicSolutions(i));
					Q = sqrt(cubicSolutions((i + 1) % 3));
					break;
				}
				else if(!(cubicSolutions((i + 2) % 3).imag() >= -0.000001 && cubicSolutions((i + 2) % 3).imag() <= 0.000001)){
					P = sqrt(cubicSolutions(i));
					Q = sqrt(cubicSolutions((i + 2) % 3));
					break;
				}
			}
		}

		std::complex<double> R = -q / (8.0 * P * Q);
		std::complex<double> S = b / (4.0 * a);

		Eigen::Vector4cd roots;

                // finish calculating quartic roots
		roots(0) = P + Q + R - S;
		roots(1) = P - Q - R - S;
		roots(2) = -P + Q - R - S;
		roots(3) = -P - Q + R - S;

                // for each root, round to zero if close to zero
        	for (int i = 0; i < 4; i++) {
			if (roots(i).imag() <= 0.000000000001 && roots[i].imag() >= -0.000000000001)
				roots(i).imag(0);
		}

		return roots;
	}

        /* @Brief: Calculates the camera center in the intermediate world frame
           @Param: alpha - the angle alpha
           @Param: theta - the angle theta
           @Param: d12 - the distance between point 1 and point 2
           @Param: b - the paramter b
           @Return: The 3D vector of the camera center in the intermediate frame
        */
	Eigen::Vector3d calc_C_eta(double alpha, double theta, double d12, double b) {

                // calculate needed trigonometric functions
		double cos_alpha = alpha / (sqrt(pow(alpha, 2) + 1));
		double sin_alpha = 1 / sqrt(1 + pow(alpha, 2));          
		double sin_theta = sqrt(1 - pow(theta, 2));				

                // check sign
		if (alpha < 0)
			cos_alpha = -cos_alpha;

                // compute vector
		Eigen::Vector3d C_eta;
		C_eta(0) = d12 * cos_alpha * (sin_alpha * b + cos_alpha);
		C_eta(1) = d12 * sin_alpha * theta * (sin_alpha * b + cos_alpha);
		C_eta(2) = d12 * sin_alpha * sin_theta * (sin_alpha * b + cos_alpha);

		return C_eta;
	}

        /* @Brief: Calculates the transformation matrix Q
           @Param: alpha - the angle alpha
           @Param: theta - the angle theta
           @Return: A 3 x 3 transformation matrix Q
        */
	Eigen::Matrix3d calc_Q(double alpha, double theta) {

                // calculate needed trigonometric functions
		double cos_alpha = alpha / (sqrt(pow(alpha, 2) + 1));
		double sin_alpha = 1 / sqrt(1 + pow(alpha, 2));         
		double sin_theta = sqrt(1 - pow(theta, 2));				

                // check sign
		if (alpha < 0)
			cos_alpha = -cos_alpha;

                // calculate matrix values
		Eigen::Matrix3d Q;
		Q(0, 0) = -cos_alpha;	Q(0, 1) = -sin_alpha * theta;	Q(0, 2) = -sin_alpha * sin_theta;
		Q(1, 0) = sin_alpha;	Q(1, 1) = -cos_alpha * theta;	Q(1, 2) = -cos_alpha * sin_theta;
		Q(2, 0) = 0;			Q(2, 1) = -sin_theta;			Q(2, 2) = theta;

		return Q;
	}

	/* @Brief: Calculates the drone's position based on the position of a global landmark
           @Algorithm: See http://rpg.ifi.uzh.ch/docs/CVPR11_kneip.pdf for details on the algorithm being used
           @Return: void
        */
	void perspective_3_point() {

                //Known world coordinates of landmark
                Eigen::Vector3d P2(0,0,.5);
		Eigen::Vector3d P1(0,0,0);
		Eigen::Vector3d P3(0,.5,0); 

		// Intrinsic Camera parameters
		double focalLength = 569.883158064802;
		double px = 331.403348466206;
		double py = 135.879365106014;
		double skew = 0;

		Eigen::Matrix3d K;
		K << focalLength, skew, px,
			0, focalLength, py,
			0, 0, 1;

		// Get image coordinates of reference points
        	Eigen::Vector3d q2(imCoords[1].x, imCoords[1].y, 1);
        	Eigen::Vector3d q1(imCoords[0].x, imCoords[0].y, 1);
        	Eigen::Vector3d q3(imCoords[3].x, imCoords[3].y, 1);

		// Calculate feature vectors
		Eigen::Vector3d f1 = K.inverse() * q1;
		Eigen::Vector3d f2 = K.inverse() * q2;
		Eigen::Vector3d f3 = K.inverse() * q3;

        	std::cout << f1 << std::endl;
        	std::cout << f2 << std::endl;
        	std::cout << f3 << std::endl;

		f1 /= f1.norm();
		f2 /= f2.norm();
		f3 /= f3.norm();

        	Eigen::Vector3d f_1 = f1;
        	Eigen::Vector3d f_2 = f2;
        	Eigen::Vector3d f_3 = f3;

		Eigen::Vector3d tx = f1;
		Eigen::Vector3d tz = f1.cross(f2) / (f1.cross(f2)).norm();
		Eigen::Vector3d ty = tz.cross(tx);

		// Create transformation matrix
		Eigen::Matrix3d T;
		T << tx, ty, tz;
		T.transposeInPlace();

        	Eigen::Vector3d f3_tau = T * f3; //

		// Check theta 
        	if(f3_tau[2] > 0){
            		f1 = f_2;
            		f2 = f_1;
            		f3 = f_3;

            		tx = f1;
            		tz = f1.cross(f2) / (f1.cross(f2)).norm();
	    		ty = tz.cross(tx);

            		Eigen::Matrix3d T_;
	    		T_ << tx, ty, tz;
	    		T = T_;
	    		T.transposeInPlace();     


            		f3_tau = T * f3;

            		Eigen::Vector3d temp = P2;
            		P2 = P1;
            		P1 = temp;
            		P3 = P3;
        	}

                // calculate new world frame vectors
		Eigen::Vector3d nx = (P2 - P1) / (P2 - P1).norm();
		Eigen::Vector3d nz = nx.cross(P3 - P1) / (nx.cross(P3 - P1)).norm();
		Eigen::Vector3d ny = nz.cross(nx);

                // calculate new world frame matrix
		Eigen::Matrix3d N;
		N << nx, ny, nz;
		N.transposeInPlace();

		Eigen::Vector3d P3_eta = N * (P3 - P1);

                // calculate needed parameters
		double p1 = P3_eta(0);
		double p2 = P3_eta(1);

		double d12 = calc_distance(P1, P2);
		double b = calc_b(f1, f2);

		double phi_1 = f3_tau(0) / f3_tau(2);
		double phi_2 = f3_tau(1) / f3_tau(2);

		// Calculate polynomial coefficients
		double a0 = calc_a0(phi_1, phi_2, p1, p2, d12, b);
		double a1 = calc_a1(phi_1, phi_2, p1, p2, d12, b);
		double a2 = calc_a2(phi_1, phi_2, p1, p2, d12, b);
		double a3 = calc_a3(phi_1, phi_2, p1, p2, d12, b);
		double a4 = calc_a4(phi_1, phi_2, p1, p2, d12, b);

		// discriminant tests
		double delta_ = clac_discriminant(a0, a1, a2, a3, a4);
		double P_ = 8 * a0 * a2 - 3 * a1 * a1;
		double R_ = pow(a1, 3) + 8 * a3 * a0 * a0 - 4 * a0 * a1 * a2;
		double delta_zero_ = a2 * a2 - 3 * a1 * a3 + 12 * a0 * a4;
		double D_ = 64 * pow(a0, 3) * a4 - 16 * a0 * a0 * a2 * a2 + 16 * a0 * a1 * a1 * a2 - 16 * a0 * a0 * a1 * a3 - 3 * pow(a1, 4);

		Eigen::Vector4cd roots = solveQuartic(a4, a3, a2, a1, a0);

		//Roots Test
		Eigen::Vector4cd rootsTest = solveQuartic(3, 6, -123, -126, 1080);
		Eigen::Vector4cd rootsTest2 = solveQuartic(-20, 5, 17, -29, 87);

		std::cout << "Quartic Roots Test: " << std::endl << roots << std::endl;
		std::cout << clac_discriminant(5, 4, 3, 2, 1) << std::endl;

                // for each root, calculate cotan alpha
		Eigen::Vector4d cot_alpha;
		for (int i = 0; i < 4; i++) {
			cot_alpha(i) = (phi_1 / phi_2 * p1 + roots(i).real() * p2 - d12 * b) / (phi_1 / phi_2 * roots(i).real() * p2 - p1 + d12);
		}

                // for each root, calculate the camera center in the intermediate world frame 
		Eigen::MatrixXd C_etas(3, 4);
		for (int i = 0; i < 4; i++) {
			C_etas.col(i) = calc_C_eta(cot_alpha(i), roots(i).real(), d12, b);
		}

                // for each root, calculate the transformation matrix Q
		Eigen::Matrix3d Q[4];
		for (int i = 0; i < 4; i++) {
			Q[i] = calc_Q(cot_alpha(i), roots(i).real());
		}
	
                // for each root, calculate the estimated camera position
		Eigen::MatrixXd C(3, 4);
		Eigen::Matrix3d R[4];
		for (int i = 0; i < 4; i++) {
			C.col(i) = P1 + N.transpose() * C_etas.col(i);
			R[i] = N.transpose() * Q[i].transpose() * T;
		}

                // Output and publish position estimation
		std::cout << C << std::endl;

		pos.x = C(0, 0);
		pos.y = C(1, 0);
		pos.z = C(2, 0);
		accuratePos.publish(pos);
	}

        /* @Brief: Detects a QR code using the ZBar library and extracts 4 feature points from it
           @Param: image - an image frame from the camera feed
           @Return: void
        */
	void find_QR_code(cv::Mat &image){
                // initialize QR code scanner
    		zbar::ImageScanner scanner;

    		scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);

                // prepare image
    		cv::Mat gray;
    		cv::cvtColor(image, gray, CV_BGR2GRAY);

    		zbar::Image zbar_image(image.cols, image.rows, "Y800", (uchar *)gray.data, image.cols * image.rows);

    		int n = scanner.scan(zbar_image);

    		for(zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin(); symbol != zbar_image.symbol_end(); ++symbol){

			//get qr code coordinates
			std::cout << "IMCOORDS:" << std::endl;
        		for(int i = 0; i < symbol->get_location_size(); i++){
	    			imCoords[i] = (cv::Point2f(symbol->get_location_x(i),symbol->get_location_y(i)));
	    			std::cout << imCoords[i] << std::endl;
			}
    		}
	}

        /* @Brief: Detects a QR code using the ZBar library and extracts 4 feature points from it
           @Param: image - an image frame from the camera feed
           @Return: void
        */
	void image_callback(const sensor_msgs::ImageConstPtr& msg){

                //check that pose estimation is enabled
                bool start_pose_estimation;
                node.getParam("/start_pose_estimation", start_pose_estimation);

                if(start_pose_estimation == false){
                    return;
                }

                // convert ROS image message to OpenCV image
  		cv_bridge::CvImagePtr cv_ptr;
  		try
  		{
    			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  		}
  		catch (cv_bridge::Exception& e)
  		{
    			ROS_ERROR("cv_bridge exception: %s", e.what());
    			return;
  		}

                //Identify QR code landmark
  		find_QR_code(cv_ptr->image);

                // draw rectangle around QR code
  		cv::rectangle(cv_ptr->image, imCoords[0], imCoords[2], cv::Scalar( 0, 255, 0 ), 2, 1 );

                // perform pose estimation algorithm
  		perspective_3_point();

                //Publish OpenCV image over ROS
		cvPub.publish(cv_ptr->toImageMsg());
                loop_rate.sleep();

  		cv::waitKey(3);
	}

	private:
	
        //Coordinates of feature points
	cv::Point2f imCoords[4]; 

        // Drone Position
	geometry_msgs::Point pos;

        // ROS node handle and publishers/subscribers
	ros::NodeHandle node;
  	image_transport::ImageTransport it;
  	ros::Publisher accuratePos;
  	image_transport::Publisher cvPub;
  	image_transport::Subscriber imageSub;
        ros::Rate loop_rate = ros::Rate(30);
};

// main driver function
int main(int argc, char** argv) {

  ros::init(argc, argv, "pose_estimator");
  PoseEstimator estimator = PoseEstimator();
  std::cout << "Done" << std::endl;
  ros::spin();

  return 0;
}

