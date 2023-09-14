// Inverse dynamics + Inverse Kinematics (.yaml working)
// Unit 6 (Final Project)
#include <cmath>
#include <cstdlib>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <pluginlib/class_list_macros.h>

void multiplyMatrices(double firstMatrix[][20], double secondMatrix[][20],
                      double multResult[][20], int rowFirst, int columnFirst,
                      int rowSecond, int columnSecond);

void sumMatrices(double firstMatrix[][20], double secondMatrix[][20],
                 double sum[][20], int rowFirst, int columnFirst);

void subsMatrices(double firstMatrix[][20], double secondMatrix[][20],
                  double subs[][20], int rowFirst, int columnFirst);

void transMatrix(double Matrix[][20], double trans[][20], int rowFirst,
                 int columnFirst);

void scalarMatrix(double scalar, double Matrix[][20], double result[][20],
                  int rowFirst, int columnFirst);

void invMatrix(double Matrix[][20], double inv[][20], int n);

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

    XmlRpc::XmlRpcValue alpha_inv_kin;
    n.getParam("alpha_inv_kin", alpha_inv_kin);
    alpha_ = alpha_inv_kin[0];

    return true;
  }

  void update(const ros::Time &time, const ros::Duration &period) {

    double dt = period.toSec();
    double t;
    t = ros::Time::now().toSec();

    q_[0][0] = joint1_.getPosition();
    q_[1][0] = joint2_.getPosition();

    double grav = 9.81;
    double m1 = 1;
    double Mb = 0.1;
    double m2 = 1 + Mb;

    double l1 = 1;
    double l2 = 1;

    double lc1 = 0.5;
    double lc2 = 0.5;

    double Izz1 = (1.0 / 12.0) * (0.1 * 0.01 + l1 * l1);
    double Izz2 = (1.0 / 12.0) * (0.1 * 0.01 + l2 * l2) + pow(l2 / 2, 2) * Mb;

    double th1 = q_[0][0];
    double th2 = q_[1][0];
    double dth1 = dq_[0][0];
    double dth2 = dq_[1][0];

    if (t > 5.0) {
      firstflag_ = true;
    }

    if (firstflag_ == false) {
      q_last_[0][0] = joint1_.getPosition();
      q_last_[1][0] = joint2_.getPosition();
      spq_[0][0] = joint1_.getPosition();
      spq_[1][0] = joint2_.getPosition();
      spq_last_[0][0] = joint1_.getPosition();
      spq_last_[1][0] = joint2_.getPosition();
    }
    if (firstflag_ == true) {

      dq_[0][0] = (q_[0][0] - q_last_[0][0]) / dt;
      dq_[1][0] = (q_[1][0] - q_last_[1][0]) / dt;

      // Inverse Kinematics

      double x_sp[2][20];

      if (5 <= t && t < 8) {
        x_sp[0][0] = 0.4;
        x_sp[1][0] = 1.6;
      }
      if (8 <= t && t < 11) {
        x_sp[0][0] = -1.2;
        x_sp[1][0] = -0.6;
      }
      if (11 <= t && t < 14) {
        x_sp[0][0] = -0.6;
        x_sp[1][0] = 1.2;
      }
      if (14 <= t && t < 17) {
        x_sp[0][0] = 1.6;
        x_sp[1][0] = 0.4;
      }
      if (17 <= t && t < 20) {
        x_sp[0][0] = -0.4;
        x_sp[1][0] = -1.6;
      }
      if (t >= 20) {
        x_sp[0][0] = 0.6;
        x_sp[1][0] = 1.2;
      }
      double error_x[2][20];
      double x_forward[2][20];

      x_forward[0][0] = cos(th1) * l1 + cos(th1 + th2) * l2;
      x_forward[1][0] = sin(th1) * l1 + sin(th1 + th2) * l2;

      subsMatrices(x_sp, x_forward, error_x, 2, 1);

      double Jacobian[2][20];
      Jacobian[0][0] = -sin(th1) * l1 - sin(th1 + th2) * l2;
      Jacobian[0][1] = -sin(th1 + th2) * l2;
      Jacobian[1][0] = cos(th1) * l1 + cos(th1 + th2) * l2;
      Jacobian[1][1] = cos(th1 + th2) * l2;

      double JacobianT[2][20];
      JacobianT[0][0] = Jacobian[0][0];
      JacobianT[0][1] = Jacobian[1][0];
      JacobianT[1][0] = Jacobian[0][1];
      JacobianT[1][1] = Jacobian[1][1];

      double diffq[2][20];
      double aux1_inv[2][20];
      double aux2_inv[2][20];
      double aux3_inv[2][20];
      double aux4_inv[2][20];
      double aux_aux_inv[2][20];

      // transMatrix(Jacobian, aux1_inv, 2, 2);
      // multiplyMatrices(aux1_inv, Jacobian, aux2_inv, 2, 2, 2, 2);
      multiplyMatrices(JacobianT, Jacobian, aux2_inv, 2, 2, 2, 2);
      invMatrix(aux2_inv, aux3_inv, 2);

      multiplyMatrices(aux3_inv, JacobianT, aux4_inv, 2, 2, 2, 2);

      multiplyMatrices(aux4_inv, error_x, diffq, 2, 2, 2, 1);

      double aux5_inv[2][20];
      double aux6_inv[2][20];
      double aux7_inv[2][20];

      scalarMatrix(alpha_, diffq, aux5_inv, 2, 1);

      sumMatrices(spq_, aux5_inv, aux7_inv, 2, 1);

      spq_[0][0] = aux7_inv[0][0];
      spq_[1][0] = aux7_inv[1][0];

      d_spq_[0][0] = (spq_[0][0] - spq_last_[0][0]) / dt;
      d_spq_[1][0] = (spq_[1][0] - spq_last_[1][0]) / dt;

      dd_spq_[0][0] = (d_spq_[0][0] - d_spq_last_[0][0]) / dt;
      dd_spq_[1][0] = (d_spq_[1][0] - d_spq_last_[1][0]) / dt;

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

      multiplyMatrices(C, dq_, aux1, 2, 2, 2, 1);
      multiplyMatrices(BM, dq_, aux2, 2, 2, 2, 1);
      sumMatrices(aux1, aux2, aux3, 2, 1);
      sumMatrices(aux3, g, h, 2, 1);

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

      subsMatrices(d_spq_, dq_, aux4, 2, 1);
      multiplyMatrices(K1, aux4, aux5, 2, 2, 2, 1);

      double aux6[2][20];
      double aux7[2][20];

      subsMatrices(spq_, q_, aux6, 2, 1);
      multiplyMatrices(K0, aux6, aux7, 2, 2, 2, 1);

      double aux8[2][20];
      double v[2][20];

      sumMatrices(aux5, aux7, aux8, 2, 1);
      sumMatrices(dd_spq_, aux8, v, 2, 1);

      double M[2][20];
      double aux10[2][20];
      double input[2][20];

      sumMatrices(D, JM, M, 2, 2);
      multiplyMatrices(M, v, aux10, 2, 2, 2, 1);
      sumMatrices(aux10, h, input, 2, 1);

      double effort1 = input[0][0];
      double effort2 = input[1][0];

      joint1_.setCommand(effort1);
      joint2_.setCommand(effort2);

      d_spq_last_[0][0] = d_spq_[0][0];
      d_spq_last_[1][0] = d_spq_[1][0];
      spq_last_[0][0] = spq_[0][0];
      spq_last_[1][0] = spq_[1][0];
      q_last_[0][0] = q_[0][0];
      q_last_[1][0] = q_[1][0];

    }

    else {
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
  double alpha_;
  static constexpr double pi_ = 3.14159265359;

  bool firstflag_ = false;
  double q_[2][20] = {
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
  double q_last_[2][20] = {
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
  double dq_[2][20] = {
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};

  double spq_[2][20] = {
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
  double spq_last_[2][20] = {
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
  double d_spq_last_[2][20] = {
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
  double d_spq_[2][20] = {
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
  double dd_spq_[2][20] = {
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
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

void scalarMatrix(double scalar, double Matrix[][20], double result[][20],
                  int rowFirst, int columnFirst) {
  int i, j;
  // Initializing elements of matrix subs to 0.
  for (i = 0; i < rowFirst; ++i) {
    for (j = 0; j < columnFirst; ++j) {
      result[i][j] = 0;
    }
  }

  for (i = 0; i < rowFirst; ++i) {
    for (j = 0; j < columnFirst; ++j) {
      result[i][j] = scalar * Matrix[i][j];
    }
  }
}

void sumMatrices(double firstMatrix[][20], double secondMatrix[][20],
                 double sum[][20], int rowFirst, int columnFirst) {
  int i, j, k;

  // Initializing elements of matrix sum to 0.
  for (i = 0; i < rowFirst; ++i) {
    for (j = 0; j < columnFirst; ++j) {
      sum[i][j] = 0;
    }
  }

  // Adding matrix firstMatrix and secondMatrix and storing in array sum.
  for (i = 0; i < rowFirst; ++i) {
    for (j = 0; j < columnFirst; ++j) {
      sum[i][j] = firstMatrix[i][j] + secondMatrix[i][j];
    }
  }
}

void subsMatrices(double firstMatrix[][20], double secondMatrix[][20],
                  double subs[][20], int rowFirst, int columnFirst) {
  int i, j, k;

  // Initializing elements of matrix subs to 0.
  for (i = 0; i < rowFirst; ++i) {
    for (j = 0; j < columnFirst; ++j) {
      subs[i][j] = 0;
    }
  }

  // Adding matrix firstMatrix and secondMatrix and storing in array subs.
  for (i = 0; i < rowFirst; ++i) {
    for (j = 0; j < columnFirst; ++j) {
      subs[i][j] = firstMatrix[i][j] - secondMatrix[i][j];
    }
  }
}

void transMatrix(double Matrix[][20], double trans[][20], int rowFirst,
                 int columnFirst) {
  int i, j, k;

  // Initializing elements of matrix subs to 0.
  for (i = 0; i < rowFirst; ++i) {
    for (j = 0; j < columnFirst; ++j) {
      trans[j][i] = 0;
    }
  }

  for (i = 0; i < rowFirst; ++i) {
    for (j = 0; j < columnFirst; ++j) {
      trans[j][i] = Matrix[i][j];
    }
  }
}

void invMatrix(double Matrix[][20], double inv[][20], int n) {
  double aux;
  double input[20][20] = {0};
  int i, j, k;

  for (i = 0; i < n; i++) {
    for (j = 0; j < n; j++) {
      input[i][j] = Matrix[i][j];
      if (i == j) {
        inv[i][j] = 1;
      } else {
        inv[i][j] = 0;
      }
    }
  }

  for (k = 0; k < n; k++) {
    aux = input[k][k];
    for (j = 0; j < n; j++) {
      input[k][j] /= aux;
      inv[k][j] /= aux;
    }
    for (i = 0; i < n; i++) {
      aux = input[i][k];
      for (j = 0; j < n; j++) {
        if (i == k)
          break;
        input[i][j] -= input[k][j] * aux;
        inv[i][j] -= inv[k][j] * aux;
      }
    }
  }
}