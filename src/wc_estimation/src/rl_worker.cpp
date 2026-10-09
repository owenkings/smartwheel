// No ROS node, publishers, hardware, wall-clock prediction or guessed TF.
// Numeric stdin/stdout protocol exposes the installed official EKF itself.
#include <robot_localization/ekf.hpp>
#include <robot_localization/filter_common.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

class AuditEkf : public robot_localization::Ekf {
public:
  void initialize(const Eigen::VectorXd &x, const Eigen::MatrixXd &p,
                  const Eigen::MatrixXd &q) {
    setState(x); setEstimateErrorCovariance(p); setProcessNoiseCovariance(q);
    setUseDynamicProcessNoiseCovariance(false);
    setLastMeasurementTime(rclcpp::Time(0, 0, RCL_ROS_TIME));
    setSensorTimeout(rclcpp::Duration::from_seconds(1000000.));
  }
};
struct State { AuditEkf filter; int64_t ns = 0; };
static constexpr int N = 15;
static const std::vector<int> constrained = {2, 3, 4, 8, 9, 10, 14};
static double number(std::istringstream &in) {
  double value;
  if (!(in >> value) || !std::isfinite(value)) throw std::runtime_error("finite numeric argument required");
  return value;
}
static int integer(std::istringstream &in) {
  int value; if (!(in >> value) || value < 0) throw std::runtime_error("nonnegative state id required"); return value;
}
int main() {
  std::map<int, State> states;
  std::cout << std::setprecision(17);
  std::string line;
  while (std::getline(std::cin, line)) {
    try {
      std::istringstream in(line); std::string op; in >> op;
      if (op == "quit") { std::cout << "OK\n" << std::flush; break; }
      if (op == "capabilities") {
        std::string trailing;
        if (in >> trailing) throw std::runtime_error("unexpected capabilities argument");
        std::cout << "CAPABILITIES 2 wheel_cov_v1\n" << std::flush;
        continue;
      }
      int id = integer(in);
      if (op == "new") {
        if (states.count(id)) throw std::runtime_error("state already exists");
        Eigen::VectorXd x(N); Eigen::MatrixXd p = Eigen::MatrixXd::Zero(N,N), q = p;
        for (int i=0;i<N;++i) x[i] = number(in);
        for (int i=0;i<N;++i) { p(i,i)=number(in); if (p(i,i)<=0) throw std::runtime_error("positive initial covariance required"); }
        for (int i=0;i<N;++i) { q(i,i)=number(in); if (q(i,i)<0) throw std::runtime_error("nonnegative process covariance required"); }
        states[id].filter.initialize(x,p,q);
        std::cout << "OK\n";
      } else if (op == "clone") {
        int destination = integer(in);
        if (states.count(destination)) throw std::runtime_error("clone destination exists");
        states.emplace(destination, states.at(id)); std::cout << "OK\n";
      } else if (op == "drop") {
        states.erase(id); std::cout << "OK\n";
      } else if (op == "predict") {
        State &s=states.at(id); double dt=number(in); int observed=integer(in);
        if (dt<0 || dt>1000000 || observed>1) throw std::runtime_error("invalid interval");
        const auto dns=static_cast<int64_t>(std::llround(dt*1e9));
        if (observed && dns && s.filter.getInitializedStatus()) s.filter.predict(rclcpp::Time(s.ns+dns,RCL_ROS_TIME),rclcpp::Duration::from_nanoseconds(dns));
        // Missing observations freeze pose. Inflating covariance is an explicit
        // experiment policy; it is deliberately separate from ekf_node's extrapolation.
        if (!observed && dns) {
          auto p=s.filter.getEstimateErrorCovariance();
          for (int i=0;i<6;++i) for(int j=6;j<N;++j) p(i,j)=p(j,i)=0.;
          p += s.filter.getProcessNoiseCovariance()*dt;
          s.filter.setEstimateErrorCovariance(p);
        }
        s.ns+=dns; s.filter.setLastMeasurementTime(rclcpp::Time(s.ns,RCL_ROS_TIME));
        std::cout << "OK\n";
      } else if (op == "decorrelate") {
        auto &s=states.at(id); auto p=s.filter.getEstimateErrorCovariance();
        for(int i=0;i<6;++i) for(int j=6;j<N;++j) p(i,j)=p(j,i)=0.;
        s.filter.setEstimateErrorCovariance(p); std::cout << "OK\n";
      } else if (op == "wheel" || op == "wheel_cov_v1" || op == "gyro") {
        auto &s=states.at(id);
        const bool wheel = op != "gyro";
        const double sigma=number(in);
        robot_localization::Measurement m;
        m.time_=rclcpp::Time(s.ns,RCL_ROS_TIME); m.topic_name_=wheel ? "wheel" : "gyro";
        m.mahalanobis_thresh_=sigma;
        m.measurement_=Eigen::VectorXd::Zero(N); m.covariance_=Eigen::MatrixXd::Zero(N,N);
        m.update_vector_=std::vector<bool>(N,false);
        m.latest_control_=Eigen::VectorXd::Zero(6);
        m.latest_control_time_=rclcpp::Time(s.ns,RCL_ROS_TIME);
        const std::vector<int> indices=wheel ? std::vector<int>{6,11} : std::vector<int>{11};
        for (int index:indices) {m.measurement_[index]=number(in); m.update_vector_[index]=true;}
        if (op == "wheel_cov_v1") {
          const double vv=number(in), vw=number(in), ww=number(in);
          if (!(vv>0. && ww>0. && std::abs(vw)/std::sqrt(vv)/std::sqrt(ww)<1.))
            throw std::runtime_error("positive definite full wheel covariance required");
          m.covariance_(6,6)=vv; m.covariance_(11,11)=ww;
          m.covariance_(6,11)=m.covariance_(11,6)=vw;
        } else {
          // Legacy protocol preserves the original diagonal-only behavior.
          for (int index:indices) {m.covariance_(index,index)=number(in); if(m.covariance_(index,index)<=0) throw std::runtime_error("positive measurement variance required");}
        }
        // RosFilter::forceTwoD's seven synthetic zero dimensions. This belongs
        // to the ROS wrapper, not to Ekf; keep it visible and regression tested.
        for (int index:constrained) {m.measurement_[index]=0.; m.covariance_(index,index)=1e-6; m.update_vector_[index]=true;}
        std::vector<int> active=indices; active.insert(active.end(),constrained.begin(),constrained.end());
        const auto x=s.filter.getState(); const auto p=s.filter.getEstimateErrorCovariance();
        Eigen::VectorXd innovation(active.size()); Eigen::MatrixXd S(active.size(),active.size());
        for(size_t i=0;i<active.size();++i){innovation[i]=m.measurement_[active[i]]-x[active[i]];
          for(size_t j=0;j<active.size();++j) S(i,j)=p(active[i],active[j])+m.covariance_(active[i],active[j]);}
        const double nis=innovation.dot(S.ldlt().solve(innovation));
        const bool was_initialized=s.filter.getInitializedStatus();
        s.filter.processMeasurement(m); // Official startup and correction semantics.
        std::cout << "UPDATE " << (!was_initialized || (std::isfinite(nis) && nis<sigma*sigma) ? 1:0) << ' ' << nis;
        for(int i=0;i<innovation.size();++i) std::cout << ' ' << innovation[i];
        std::cout << '\n';
      } else if (op == "get") {
        auto &s=states.at(id); auto x=s.filter.getState(); auto p=s.filter.getEstimateErrorCovariance();
        std::cout << "STATE " << s.ns;
        for(int i=0;i<N;++i) std::cout << ' ' << x[i];
        for(int i=0;i<N;++i) for(int j=0;j<N;++j) std::cout << ' ' << p(i,j);
        std::cout << '\n';
      } else throw std::runtime_error("unknown operation");
      std::string trailing; if(in >> trailing) throw std::runtime_error("unexpected trailing argument");
    } catch(const std::exception &e) { std::cout << "ERROR " << e.what() << '\n'; }
    std::cout << std::flush;
  }
}
