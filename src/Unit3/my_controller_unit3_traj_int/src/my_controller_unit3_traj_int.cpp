// Controller Unit 3 Trajectory interpolation

#include <cmath>
#include <cstdlib>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

void multiplyMatrices(double firstMatrix[][20], double secondMatrix[][20],
                      double multResult[][20], int rowFirst, int columnFirst,
                      int rowSecond, int columnSecond);

void sumMatrices(double firstMatrix[][20], double secondMatrix[][20],
                 double sum[][20], int rowFirst, int columnFirst, int rowSecond,
                 int columnSecond);

void subsMatrices(double firstMatrix[][20], double secondMatrix[][20],
                  double subs[][20], int rowFirst, int columnFirst,
                  int rowSecond, int columnSecond);

namespace controller_ns {

class PositionController : public controller_interface::Controller<
                               hardware_interface::EffortJointInterface> {
public:
  bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n) {
    std::string my_joint1;
    std::string my_joint2;
    if (!n.getParam("joint1", my_joint1)) {
      ROS_ERROR("Could not find joint name");
      return false;
    }
    if (!n.getParam("joint2", my_joint2)) {
      ROS_ERROR("Could not find joint name");
      return false;
    }

    joint1_ = hw->getHandle(my_joint1); // throws on failure
    joint2_ = hw->getHandle(my_joint2);

    XmlRpc::XmlRpcValue gains_arm1;
    n.getParam("gains1", gains_arm1);
    KP1_ = gains_arm1[0];
    KD1_ = gains_arm1[1];

    XmlRpc::XmlRpcValue gains_arm2;
    n.getParam("gains2", gains_arm2);
    KP2_ = gains_arm1[0];
    KD2_ = gains_arm1[1];

    XmlRpc::XmlRpcValue coef1_a;
    n.getParam("coef1_a", coef1_a);
    a_link1_[0] = double(coef1_a[0]);
    a_link1_[1] = double(coef1_a[1]);
    a_link1_[2] = double(coef1_a[2]);
    a_link1_[3] = double(coef1_a[3]);

    XmlRpc::XmlRpcValue coef2_a;
    n.getParam("coef2_a", coef2_a);
    a_link2_[0] = double(coef2_a[0]);
    a_link2_[1] = double(coef2_a[1]);
    a_link2_[2] = double(coef2_a[2]);
    a_link2_[3] = double(coef2_a[3]);

    return true;
  }

  void update(const ros::Time &time, const ros::Duration &period) {

    double t;
    t = ros::Time::now().toSec() - tinit_;

    if (t>0 && t < 20) {
      spth1_ = a_link1_[0] + a_link1_[1] * pow(t, 1) +
               a_link1_[2] * pow(t, 2) + a_link1_[3] * pow(t, 3);
      spth2_ = a_link2_[0] + a_link2_[1] * pow(t, 1) +
               a_link2_[2] * pow(t, 2) + a_link2_[3] * pow(t, 3);
    } else {
      spth1_ = 90 * pi_ / 180;
      spth2_ = 20 * pi_ / 180;
    }
    double dt = period.toSec();

    double th1 = joint1_.getPosition();
    double th2 = joint2_.getPosition();

    double dth1;
    double dth2;

    double grav = 9.81;
    double m1 = 1;
    double m2 = 1;

    double l1 = 1;
    double l2 = 1;

    double lc1 = 0.5;
    double lc2 = 0.5;

    double Izz1 = (1.0 / 12.0) * (0.1 * 0.01 + l1 * l1);
    double Izz2 = (1.0 / 12.0) * (0.1 * 0.01 + l2 * l2);

    if (t >= 1) {
      firstflag_ = true;
    }

    if (firstflag_) {
      double input[2][20];

      dth1 = (th1 - th1_last_) / dt;
      dth2 = (th2 - th2_last_) / dt;

      input[0][0] = KP1_ * (spth1_ - th1) + KD1_ * (-dth1);
      input[1][0] = KP2_ * (spth2_ - th2) + KD2_ * (-dth2);

      double effort1 = 0.1 * input[0][0];
      double effort2 = 0.1 * input[1][0];

      joint1_.setCommand(effort1);
      joint2_.setCommand(effort2);
    }
    if (!firstflag_) {
      joint1_.setCommand(0);
      joint2_.setCommand(0);
    }
    th1_last_ = th1;
    th2_last_ = th2;
  }

  void starting(const ros::Time &time) {}
  void stopping(const ros::Time &time) {}

private:
  hardware_interface::JointHandle joint2_;
  hardware_interface::JointHandle joint1_;

  static constexpr double pi_ = 3.14159265359;

  double spth1_ = 0;
  double spth2_ = 0;
  double tinit_ = ros::Time::now().toSec();
  double KP1_ = 0;
  double KP2_ = 0;

  double KD1_ = 0;
  double KD2_ = 0;

  double a_link1_[4];
  double a_link2_[4];

  bool firstflag_ = false;

  double th1_last_ = 0;
  double th2_last_ = 0;
};
PLUGINLIB_EXPORT_CLASS(controller_ns::PositionController,
                       controller_interface::ControllerBase);
} // namespace controller_ns

// Linear Algebra

void multiplyMatrices(double firstMatrix[][20], double secondMatrix[][20],
                      double mult[][20], int rowFirst, int columnFirst,
                      int rowSecond, int columnSecond) {
  int i, j, k;

  // Initializing elements of matrix mult to 0.
  for (i = 0; i < rowFirst; ++i) {
    for (j = 0; j < columnSecond; ++j) {
      mult[i][j] = 0;
    }
  }

  // Multiplying matrix firstMatrix and secondMatrix and storing in array mult.
  for (i = 0; i < rowFirst; ++i) {
    for (j = 0; j < columnSecond; ++j) {
      for (k = 0; k < columnFirst; ++k) {
        mult[i][j] += firstMatrix[i][k] * secondMatrix[k][j];
      }
    }
  }
}

void sumMatrices(double firstMatrix[][20], double secondMatrix[][20],
                 double sum[][20], int rowFirst, int columnFirst, int rowSecond,
                 int columnSecond) {
  int i, j, k;

  // Initializing elements of matrix sum to 0.
  for (i = 0; i < rowFirst; ++i) {
    for (j = 0; j < columnSecond; ++j) {
      sum[i][j] = 0;
    }
  }

  // Adding matrix firstMatrix and secondMatrix and storing in array sum.
  for (i = 0; i < rowFirst; ++i) {
    for (j = 0; j < columnSecond; ++j) {
      sum[i][j] = firstMatrix[i][j] + secondMatrix[i][j];
    }
  }
}

void subsMatrices(double firstMatrix[][20], double secondMatrix[][20],
                  double subs[][20], int rowFirst, int columnFirst,
                  int rowSecond, int columnSecond) {
  int i, j, k;

  // Initializing elements of matrix subs to 0.
  for (i = 0; i < rowFirst; ++i) {
    for (j = 0; j < columnSecond; ++j) {
      subs[i][j] = 0;
    }
  }

  // Adding matrix firstMatrix and secondMatrix and storing in array subs.
  for (i = 0; i < rowFirst; ++i) {
    for (j = 0; j < columnSecond; ++j) {
      subs[i][j] = firstMatrix[i][j] - secondMatrix[i][j];
    }
  }
}