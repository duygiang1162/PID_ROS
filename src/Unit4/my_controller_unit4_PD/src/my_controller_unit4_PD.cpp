// Revisited PD (.yaml working)
// Unit 4
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

    // XmlRpc::XmlRpcValue gains_arm1;
    // n.getParam("gains1", gains_arm1);
    // KP1_ = gains_arm1[0];
    // KD1_ = gains_arm1[1];

    KP1_ = 13.34;
    KD1_ = 6.66;

    // XmlRpc::XmlRpcValue gains_arm2;
    // n.getParam("gains2", gains_arm2);
    // KP2_ = gains_arm2[0];
    // KD2_ = gains_arm2[1];

    KP2_ = 13.34;
    KD2_ = 6.66;

    return true;
  }

  void update(const ros::Time &time, const ros::Duration &period) {

    double dt = period.toSec();

    double th1 = joint1_.getPosition();
    double th2 = joint2_.getPosition();

    double dth1;
    double dth2;

    double g = 9.81;
    double m1 = 1;
    double m2 = 1;

    double l1 = 1;
    double l2 = 1;

    double lc1 = 0.5;
    double lc2 = 0.5;

    if (period == ros::Duration(0.0)) {
      dth1 = 0;
      dth2 = 0;
    }

    if (dt > 0.0) {
      firstflag_ = true;
      dth1 = (th1 - th1_last_) / dt;
      th1_last_ = th1;

      dth2 = (th2 - th2_last_) / dt;
      th2_last_ = th2;
    }

    double error_p[2][20];
    double error_d[2][20];

    double t;
    t = ros::Time::now().toSec();
    double f1 = 1.0;
    double f2 = 0.5;
    double spq[2][20];
    spq[0][0] = 0.5 * sin(f1 * t);
    spq[1][0] = 0.75 * cos(f2 * t);

    error_p[0][0] = spq[0][0] - th1;
    error_p[1][0] = spq[1][0] - th2;

    error_d[0][0] = 0 - dth1;
    error_d[1][0] = 0 - dth2;

    double KP_matrix[2][20];
    double KD_matrix[2][20];

    KP_matrix[0][0] = KP1_;
    KP_matrix[0][1] = 0;
    KP_matrix[1][0] = 0;
    KP_matrix[1][1] = KP2_;

    KD_matrix[0][0] = KD1_;
    KD_matrix[0][1] = 0;
    KD_matrix[1][0] = 0;
    KD_matrix[1][1] = KD2_;

    double aux1[2][20];
    double aux2[2][20];
    double input[2][20];

    multiplyMatrices(KP_matrix, error_p, aux1, 2, 2, 2, 1);
    multiplyMatrices(KD_matrix, error_d, aux2, 2, 2, 2, 1);
    sumMatrices(aux1, aux2, input, 2, 1, 2, 1);

    double effort1 = input[0][0];
    double effort2 = input[1][0];

    if (firstflag_) {
      joint1_.setCommand(effort1);
      joint2_.setCommand(effort2);
    }
    if (!firstflag_) {
      joint1_.setCommand(0);
      joint2_.setCommand(0);
    }
  }

  void starting(const ros::Time &time) {}
  void stopping(const ros::Time &time) {}

private:
  hardware_interface::JointHandle joint2_;
  hardware_interface::JointHandle joint1_;

  static constexpr double pi_ = 3.14159265359;

  double KP1_;
  double KP2_;

  double KD1_;
  double KD2_;

  bool firstflag_ = false;

  double th1_last_ = 0;
  double th2_last_ = 0;
};
PLUGINLIB_EXPORT_CLASS(controller_ns::PositionController,
                       controller_interface::ControllerBase);
} // namespace controller_ns

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