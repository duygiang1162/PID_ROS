// Inverse dynamics (.yaml working)
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

    XmlRpc::XmlRpcValue frequency;
    n.getParam("omega", frequency);
    omega_ = frequency[0];
    return true;
  }

  void update(const ros::Time &time, const ros::Duration &period) {

    double dt = period.toSec();

    double th1 = joint1_.getPosition();
    double th2 = joint2_.getPosition();

    double dth1;
    double dth2;

    double grav = 9.81;
    double m1 = 1;
    double Mb = 0;
    double m2 = 1 + Mb;

    double l1 = 1;
    double l2 = 1;

    double lc1 = 0.5;
    double lc2 = 0.5;

    double Izz1 = (1.0 / 12.0) * (0.1 * 0.01 + l1 * l1);
    double Izz2 = (1.0 / 12.0) * (0.1 * 0.01 + l2 * l2) + pow(l2 / 2, 2) * Mb;

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
    double dq[2][20];
    dq[0][0] = dth1;
    dq[1][0] = dth2;

    double q[2][20];
    q[0][0] = th1;
    q[1][0] = th2;

    double t;
    t = ros::Time::now().toSec();
    double f1 = 1.0;
    double f2 = 0.5;
    double A1 = 0.5;
    double A2 = 0.75;
    double spq[2][20];
    spq[0][0] = A1 * sin(f1 * t);
    spq[1][0] = A2 * cos(f2 * t);

    double d_spq[2][20];
    d_spq[0][0] = A1 * f1 * cos(f1 * t);
    d_spq[1][0] = -A2 * f2 * sin(f2 * t);

    double dd_spq[2][20];
    dd_spq[0][0] = -A1 * f1 * f1 * sin(f1 * t);
    dd_spq[1][0] = -A2 * f2 * f2 * cos(f2 * t);

    double D[2][20];
    D[0][0] = Izz1 + Izz2 + pow(lc1, 2) * m1 + pow(l1, 2) * m2 +
              2 * cos(th2) * l1 * lc2 * m2 + pow(lc2, 2) * m2;
    D[0][1] = Izz2 + cos(th2) * l1 * lc2 * m2 + pow(lc2, 2) * m2;
    D[1][0] = Izz2 + cos(th2) * l1 * lc2 * m2 + pow(lc2, 2) * m2;
    D[1][1] = Izz2 + pow(lc2, 2) * m2;

    double C[2][20];
    C[0][0] = -2 * sin(th2) * l1 * lc2 * m2 * dth2;
    C[0][1] = -sin(th2) * l1 * lc2 * m2 * dth2;
    C[1][0] = sin(th2) * l1 * lc2 * m2 * dth1;
    C[1][1] = 0;

    double g[2][20];
    g[0][0] = 0;
    g[1][0] = 0;

    double BM[2][20];
    BM[0][0] = 0;
    BM[0][1] = 0;
    BM[1][0] = 0;
    BM[1][1] = 0;

    double JM[2][20];
    JM[0][0] = 0;
    JM[0][1] = 0;
    JM[1][0] = 0;
    JM[1][1] = 0;

    double aux1[2][20];
    double aux2[2][20];
    double aux3[2][20];
    double h[2][20];

    multiplyMatrices(C, dq, aux1, 2, 2, 2, 1);
    multiplyMatrices(BM, dq, aux2, 2, 2, 2, 1);
    sumMatrices(aux1, aux2, aux3, 2, 1, 2, 1);
    sumMatrices(aux3, g, h, 2, 1, 2, 1);

    double aux4[2][20];
    double aux5[2][20];

    double K0[2][20];
    K0[0][0] = pow(omega_, 2);
    K0[0][1] = 0;
    K0[1][0] = 0;
    K0[1][1] = pow(omega_, 2);

    double K1[2][20];
    K1[0][0] = 2 * omega_;
    K1[0][1] = 0;
    K1[1][0] = 0;
    K1[1][1] = 2 * omega_;

    subsMatrices(d_spq, dq, aux4, 2, 1, 2, 1);
    multiplyMatrices(K1, aux4, aux5, 2, 2, 2, 1);

    double aux6[2][20];
    double aux7[2][20];

    subsMatrices(spq, q, aux6, 2, 1, 2, 1);
    multiplyMatrices(K0, aux6, aux7, 2, 2, 2, 1);

    double aux8[2][20];
    double v[2][20];

    sumMatrices(aux5, aux7, aux8, 2, 1, 2, 1);
    sumMatrices(dd_spq, aux8, v, 2, 1, 2, 1);

    double M[2][20];
    double aux10[2][20];
    double input[2][20];

    sumMatrices(D, JM, M, 2, 2, 2, 2);
    multiplyMatrices(M, v, aux10, 2, 2, 2, 1);
    sumMatrices(aux10, h, input, 2, 1, 2, 1);

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

  double omega_;
  static constexpr double pi_ = 3.14159265359;

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