
#include "SSF_Core.h"
#include "measurement.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
//#include <boost/thread.hpp>
using namespace ssf_core;
typedef struct imuD{

    double timestamp;
    Eigen::Vector3d a_m;
    Eigen::Vector3d omega_m;
    imuD(){}
    imuD(double t, Eigen::Vector3d& a, Eigen::Vector3d& w){
        timestamp = t;
        a_m = a;
        omega_m = w;
    }
} imuData;
struct slamData{
    double timestamp;
    Eigen::Vector3d p_c_v;
    Eigen::Quaterniond q_c_v;
};

int main(int argc, char** argv)

{
    std::vector<imuData> imuV;
    SSF_Core sc;
    int scalecount;
    double scale = 1;
    std::ofstream in;
	std::ifstream imuFile;
	std::ifstream slamFile, configFile;
/*
    imuFile.open( "imuData.txt" );
    slamFile.open( "slamPath.txt" );
*/
if(argc == 4){
    imuFile.open(argv[1]);
    slamFile.open(argv[2]);
    configFile.open(argv[3]);
 }else{
    imuFile.open( "imu_m_data.csv" );
    slamFile.open( "ground_m_data.csv" );
    configFile.open( "configFile.txt" );
 }

    std::string config_line;
    std::string cs;
    double mconfig[10];
    for(size_t i = 0; i< 10; ++ i){
        getline(configFile, config_line);
        std::stringstream config_ss(config_line);
        getline(config_ss, cs, ':');
        getline(config_ss, cs, ':');
        mconfig[i] = std::stof(cs);

    }
    getline(configFile, config_line);
    std::stringstream config_ss(config_line);
    getline(config_ss, cs, ':');
    getline(config_ss, cs, ':');
    double LL = std::stof(cs);

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
    double initL = LL;
    Eigen::Matrix<double, 3, 1> g(0.0, 0.0, 9.8001);
	std::string lie;
	getline(imuFile, lie);
	getline(slamFile,lie);
	getline(imuFile, lie);
	char itemp,tmp;
	double itime;
	std::stringstream initi;
    initi.str(lie);
    initi
		  >> itime >> tmp
          >> w_m(0) >> tmp >> w_m(1) >> tmp >> w_m(2) >> tmp
          >> a_m(0) >> tmp >> a_m(1) >> tmp >> a_m(2);

	getline(slamFile,lie);
    std::stringstream inits;
    inits.str(lie);



    inits >> itime >> itemp >> inipos(0) >> itemp >> inipos(1) >> itemp >> inipos(2)

        >> itemp >> initq.coeffs()(3)>> itemp >> initq.coeffs()(0) >> itemp >> initq.coeffs()(1) >>itemp >> initq.coeffs()(2)
         >> itemp >> inivel(0) >> itemp >> inivel(1) >> itemp >> inivel(2)
        >> itemp >> b_w(0) >> itemp >> b_w(1) >> itemp >> b_w(2)
         >> itemp >> b_a(0) >> itemp >> b_a(1) >> itemp >> b_a(2);


    sc.initialize(inipos, inivel, initq, b_w, b_a, initL, initqwv, initP, w_m, a_m, g, initqci,initpci, mconfig);

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
    std::string sssimu;
    for(int i = 0; i < 100; ++i)
        getline( imuFile, sssimu );

	unsigned numSlamMeas = 0;
	//getline( slamFile, line );
	std::string imuLine;
	double old_timu;
    for( std::string line; getline( slamFile, line );)
	{

        std::stringstream ss;
        ss.str(line);
        Eigen::Vector3d p_c_v;
		Eigen::Quaterniond q_c_v;
		double slamTimeNs;
		int newKf = 0;

		char tmp;
        ss
            >> slamTimeNs >> tmp
			>> p_c_v(0) >> tmp >> p_c_v(1) >> tmp >> p_c_v(2) >> tmp
	    	>> q_c_v.coeffs()(3) >> tmp >> q_c_v.coeffs()(0) >> tmp >> q_c_v.coeffs()(1) >> tmp >> q_c_v.coeffs()(2);
        p_c_v *= scale;
        sc.measurementCallback(p_c_v);
        Eigen::Matrix<double, 4, 1> result;
        sc.get_result(result);


        in << result(0, 0) << "\t"<< result(1, 0) << "\t"<< result(2, 0) << "\t"<< result(3, 0) << std::endl;
        getline(slamFile,line);
        getline(slamFile,line);
        getline(slamFile,line);
        double t_imu;
        do{
            if( !getline( imuFile, imuLine )){
                std::cout << "imu data finished" << std::endl;
            }


            std::stringstream ss;
            ss.str( imuLine );

            Eigen::Vector3d a_m;
            Eigen::Vector3d omega_m;

            char tmp;

            ss
                >> t_imu >> tmp
                >> omega_m(0) >> tmp >> omega_m(1) >> tmp >> omega_m(2) >> tmp
                >> a_m(0) >> tmp >> a_m(1) >> tmp >> a_m(2);


            double delta_t = 0.005;
            if((t_imu - old_timu) < 10000000) delta_t = (t_imu - old_timu)/1000000000.0;
            Eigen::Matrix<double ,6, 1> measurmt;
            measurmt << omega_m(0),omega_m(1),omega_m(2),a_m(0),a_m(1),a_m(2);
            sc.imuCallback(measurmt, delta_t);
            old_timu = t_imu;
            }while(t_imu <= slamTimeNs);

    }



	/*
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
            imuData mimu(imuTimeNs, a_m, omega_m);
            imuV.push_back(mimu);

            omega_m(0) = 100;
            Eigen::Matrix<double, 3, 1> ddd = a_m;
            Eigen::Matrix<double ,6, 1> measurmt;
            measurmt << omega_m(0),omega_m(1),omega_m(2),a_m(0),a_m(1),a_m(2);
            sc.imuCallback(measurmt);

            if(count  == 4)
				break;
		}

        int ad;

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


        //double scale = (1.0 + (rand()%100 - 50)/800.0);
        scalecount ++;
        if(scalecount == 4000)
            //scale += 1;

        p_c_v *= scale;
        //sc.measurementCallback(p_c_v, q_c_v);
        sc.measurementCallback(p_c_v);
        Eigen::Matrix<double, 4, 1> result;
        sc.get_result(result);

	    numSlamMeas++;
	    q_c_v.normalize();
        in << result(0, 0) << "\t"<< result(1, 0) << "\t"<< result(2, 0) << "\t"<< result(3, 0) << std::endl;
        getline(slamFile,line);
        getline(slamFile,line);
        getline(slamFile,line);


	}
	*/
	configFile.close();
	imuFile.close();
	slamFile.close();
	in.close();
	std::cout << "argc == " << argc << std::endl;
	return 0;
}
