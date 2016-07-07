#ifndef SSF_CORE_H_
#define SSF_CORE_H_


#include <Eigen/Eigen>


//#include <dynamic_reconfigure/server.h>

//#include <ssf_core/SSF_CoreConfig.h>

// message includes
/*
#include <sensor_fusion_comm/DoubleArrayStamped.h>
#include <sensor_fusion_comm/ExtState.h>
#include <sensor_fusion_comm/ExtEkf.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
*/
#include <vector>
#include "state.h"

#define N_STATE_BUFFER 256	///< size of unsigned char, do not change!
#define HLI_EKF_STATE_SIZE 16 	///< number of states exchanged with external propagation. Here: p,v,q,bw,bw=16
#define N_MEAS 9
namespace ssf_core{
//typedef dynamic_reconfigure::Server<ssf_core::SSF_CoreConfig> ReconfigureServer;

class SSF_Core
{

public:
  typedef Eigen::Matrix<double, N_STATE, 1> ErrorState;
  typedef Eigen::Matrix<double, N_STATE, N_STATE> ErrorStateCov;

  /// big init routine
  void initialize(const Eigen::Matrix<double, 3, 1> & p, const Eigen::Matrix<double, 3, 1> & v,
                  const Eigen::Quaternion<double> & q, const Eigen::Matrix<double, 3, 1> & b_w,
                  const Eigen::Matrix<double, 3, 1> & b_a, const double & L, const Eigen::Quaternion<double> & q_wv,
                  const Eigen::Matrix<double, N_STATE, N_STATE> & P, const Eigen::Matrix<double, 3, 1> & w_m,
                  const Eigen::Matrix<double, 3, 1> & a_m, const Eigen::Matrix<double, 3, 1> & g,
                  const Eigen::Quaternion<double> & q_ci, const Eigen::Matrix<double, 3, 1> & p_ci, double * mconfig);

  /// retreive all state information at time t. Used to build H, residual and noise matrix by update sensors
  //unsigned char getClosestState(State* timestate, ros::Time tstamp, double delay = 0.00);

  /// get all state information at a given index in the ringbuffer
  bool getStateAtIdx(State* timestate, unsigned char idx);

  SSF_Core();
  ~SSF_Core();

private:
  const static int nFullState_ = 28; ///< complete state
  const static int nBuff_ = 30; ///< buffer size for median q_vw
  const static int nMaxCorr_ = 50; ///< number of IMU measurements buffered for time correction actions
  const static int QualityThres_ = 1e3;

  Eigen::Matrix<double, N_STATE, N_STATE> Fd_; ///< discrete state propagation matrix
  Eigen::Matrix<double, N_STATE, N_STATE> Qd_; ///< discrete propagation noise matrix

  /// state variables
  State StateBuffer_[N_STATE_BUFFER]; ///< EKF ringbuffer containing pretty much all info needed at time t
  unsigned char idx_state_;///< pointer to state buffer at most recent state
  unsigned char idx_P_; ///< pointer to state buffer at P latest propagated
  unsigned char idx_time_; ///< pointer to state buffer at a specific time

  Eigen::Matrix<double, 3, 1> g_; ///< gravity vector

  /// vision-world drift watch dog to determine fuzzy tracking
  int qvw_inittimer_;
  Eigen::Matrix<double, nBuff_, 4> qbuff_;

  /// correction from EKF update
  Eigen::Matrix<double, N_STATE, 1> correction_;

  /// dynamic reconfigure config
  //ssf_core::SSF_CoreConfig config_;

  Eigen::Matrix<double, 3, 3> R_IW_; ///< Rot IMU->World
  Eigen::Matrix<double, 3, 3> R_CI_; ///< Rot Camera->IMU
  Eigen::Matrix<double, 3, 3> R_WV_; ///< Rot World->Vision

  bool initialized_;
  bool predictionMade_;

  /// enables internal state predictions for log replay
  /**
   * used to determine if internal states get overwritten by the external
   * state prediction (online) or internal state prediction is performed
   * for log replay, when the external prediction is not available.
   */
  bool data_playback_;

  enum
  {
    NO_UP, GOOD_UP, FUZZY_UP
  };
    double d = 0.001;
    ///Q
    double noise_a = 0.022563 * d;
    double noise_ba = 0.022563 * d;
    double noise_w = 0.022563 * d;
    double noise_bw = 0.022563 * d;
    double vvc = 0.00001 * d;
    double noise_scale = 0.004 * d;
    double P_scale = 1;
    ///R
    double n_zp_ = 0.1;
  /// propagates the state with given dt


  /// external state propagation
  /**
   * This function gets called when state prediction is performed externally,
   * e.g. by asctec_mav_framework. Msg has to be the latest predicted state.
   * Only use this OR imuCallback by remapping the topics accordingly.
   * \sa{imuCallback}
   */
  //*void stateCallback(const sensor_fusion_comm::ExtEkfConstPtr & msg);

  /// gets called by dynamic reconfigure and calls all registered callbacks in callbacks_
  //void Config(ssf_core::SSF_CoreConfig &config, uint32_t level);

  /// handles the dynamic reconfigure for ssf_core
  //void DynConfig(ssf_core::SSF_CoreConfig &config, uint32_t level);

  /// computes the median of a given vector
  double getMedian(const Eigen::Matrix<double, nBuff_, 1> & data);

public:
  // some header implementations
  /// main update routine called by a given sensor
  void get_result(Eigen::Matrix<double, 4, 1>& results);
  template<class H_type, class Res_type, class R_type>
    bool applyMeasurement(unsigned char idx_delaystate, const Eigen::MatrixBase<H_type>& H_delayed,
                          const Eigen::MatrixBase<Res_type> & res_delayed, const Eigen::MatrixBase<R_type>& R_delayed,
                          double fuzzythres = 0.1)
    {
      EIGEN_STATIC_ASSERT_FIXED_SIZE(H_type);
      EIGEN_STATIC_ASSERT_FIXED_SIZE(R_type);

      // get measurements
      if (!predictionMade_)
        return false;

      // make sure we have correctly propagated cov until idx_delaystate
      //propPToIdx(idx_delaystate);

      R_type S;
      Eigen::Matrix<double, N_STATE, R_type::RowsAtCompileTime> K;
      ErrorStateCov & P = StateBuffer_[idx_delaystate].P_;

      S = H_delayed * StateBuffer_[idx_delaystate].P_ * H_delayed.transpose() + R_delayed;
      K = P * H_delayed.transpose() * S.inverse();

      correction_ = K * res_delayed;
      const ErrorStateCov KH = (ErrorStateCov::Identity() - K * H_delayed);
      P = KH * P * KH.transpose() + K * R_delayed * K.transpose();

      // make sure P stays symmetric
      P = 0.5 * (P + P.transpose());

      return applyCorrection(idx_delaystate, correction_, fuzzythres);
    }
  void propagateState(const double dt);
  /// propagets the error state covariance
  void predictProcessCovariance(const double dt);

  /// applies the correction
  bool applyCorrection(unsigned char idx_delaystate, const ErrorState & res_delayed, double fuzzythres = 0.1);

  /// propagate covariance to a given index in the ringbuffer
  //void propPToIdx(unsigned char idx);

  /// internal state propagation
  /**
   * This function gets called on incoming imu messages an then performs
   * the state prediction internally. Only use this OR stateCallback by
   * remapping the topics accordingly.
   * \sa{stateCallback}
   */
  void measurementCallback(const Eigen::Vector3d& msg);

  void imuCallback(const Eigen::Matrix<double, 6, 1> & msg);
  /// registers dynamic reconfigure callbacks
  /*
  template<class T>
    void registerCallback(void(T::*cb_func)(ssf_core::SSF_CoreConfig& config, uint32_t level), T* p_obj)
    {
      callbacks_.push_back(boost::bind(cb_func, p_obj, _1, _2));
    }
    */
};

};// end namespace

#endif /* SSF_CORE_H_ */
