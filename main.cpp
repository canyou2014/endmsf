
#include "SSF_Core.h"
#include "measurement.h"
#include <iostream>
#include <fstream>
#include <math.h>
#include <boost/thread.hpp>
using namespace ssf_core;

int main()

{

    SSF_Core sc;


    std::ofstream scale_in, in;
	std::ifstream imuFile;
	std::ifstream slamFile;
/*
    imuFile.open( "imuData.txt" );
    slamFile.open( "slamPath.txt" );
*/
    imuFile.open( "imu_vm_data.csv" );
    slamFile.open( "ground_vm_data.csv" );
    scale_in.open("predict_scale.txt", std::ios::trunc);
    in.open("predict.txt", std::ios::trunc);

    std::cerr << "Please provide files for testing!" << std::endl
    << "Format is: ekf_test <imuFile> <slamFile>" << std::endl;


	if (!imuFile.is_open())
	{
		std::cerr << "Error opening file: " << std::endl;
		exit(1);
	}
	if (!slamFile.is_open())
	{
		std::cerr << "Error opening file: " << std::endl;
		exit(1);
	}
    Eigen::Matrix<double, 3, 1> b_w;
    Eigen::Matrix<double, 3, 1> b_a;
    Eigen::Matrix<double, 3, 1> inipos;
    Eigen::Matrix<double, 3, 1> inivel;
    Eigen::Matrix<double, 3, 1> w_m;
    Eigen::Matrix<double, 3, 1> a_m;
    Eigen::Quaternion<double> initq;
    Eigen::Quaternion<double> initqwv;
    Eigen::Quaternion<double> initqci;
    Eigen::Matrix<double, 3, 1> initpci;
    initqwv.coeffs()(3) = 1.0;
    initqci.coeffs()(3) = 1.0;
    Eigen::Matrix<double, 25, 25> initP = Eigen::Matrix<double, 25, 25>::Zero();
    double initL = 1.0;
    Eigen::Matrix<double, 3, 1> g(0.0, 0.0, 9.8001);
	std::string lie;
	getline(imuFile, lie);
	getline(slamFile,lie);
	getline(imuFile, lie);
	getline(slamFile,lie);
    std::stringstream inits;
    inits.str(lie);
    char itemp;
    double itime;

    inits >> itime >> itemp >> inipos(0) >> itemp >> inipos(1) >> itemp >> inipos(2)

        >> itemp >> initq.coeffs()(3)>> itemp >> initq.coeffs()(0) >> itemp >> initq.coeffs()(1) >>itemp >> initq.coeffs()(2)
         >> itemp >> inivel(0) >> itemp >> inivel(1) >> itemp >> inivel(2)
        >> itemp >> b_w(0) >> itemp >> b_w(1) >> itemp >> b_w(2)
         >> itemp >> b_a(0) >> itemp >> b_a(1) >> itemp >> b_a(2);


    sc.initialize(inipos, inivel, initq, b_w, b_a, initL, initqwv, initP, w_m, a_m, g, initqci,initpci);
     double r1 = 0.039, r2 = 0.3;  //0.041 0.1  >> and xiaoyu 0.45
			// g
    /*
	Eigen::Matrix<double,28,1> P;
	double d = 0.5;
	P << 0.001, 0.001, 0.001,								// p_i_w
	0.00001, 0.00001, 0.00001,									// v_i_w
	0.0035, 0.0035, 0.0035,								// q_i_w
	0.5, 0.5, 0.5,								// b_omega
	0.1, 0.1, 0.1,									// b_a
	1*log(2),											// lambda
    0.00, 0.00, 0.00,//0.005, 0.005, 0.005,							// p_c_i >>>
	0.00, 0.00, 0.00,//0.05, 0.005, 0.005,								// q_c_i 0.5
	0.0, 0.0, 0.0,									// p_w_v
	0.0, 0.0, 0.0;									// q_w_v
	P = 1*P.cwiseProduct(P);
*/


	unsigned numSlamMeas = 0;
	for( std::string line; getline( slamFile, line );)
	{



        int count = 0;
		// propagate
		for( std::string imuLine; getline( imuFile, imuLine ); )
		{
		    count ++;
			std::stringstream ss;
			ss.str( imuLine );

			Eigen::Vector3d a_m;
			Eigen::Vector3d omega_m;
			double imuTimeNs;

			char tmp;

            ss
		    	>> imuTimeNs >> tmp
				>> omega_m(0) >> tmp >> omega_m(1) >> tmp >> omega_m(2) >> tmp
	    		>> a_m(0) >> tmp >> a_m(1) >> tmp >> a_m(2);

            Eigen::Matrix<double ,6, 1> measurmt;
            measurmt << omega_m(0),omega_m(1),omega_m(2),a_m(0),a_m(1),a_m(2);
            sc.imuCallback(measurmt);

            if(count  == 4)
				break;
		}



		std::stringstream ss;
		ss.str( line );

		Eigen::Vector3d p_c_v;
		Eigen::Quaterniond q_c_v;
		double slamTimeNs;
		int newKf = 0;

		char tmp;
        ss
            >> slamTimeNs >> tmp
			>> p_c_v(0) >> tmp >> p_c_v(1) >> tmp >> p_c_v(2) >> tmp
	    	>> q_c_v.coeffs()(3) >> tmp >> q_c_v.coeffs()(0) >> tmp >> q_c_v.coeffs()(1) >> tmp >> q_c_v.coeffs()(2);


        sc.measurementCallback(p_c_v);

	    numSlamMeas++;
	    q_c_v.normalize();

        getline(slamFile,line);
        getline(slamFile,line);
        getline(slamFile,line);


	}
	scale_in.close();
	return 0;
}
