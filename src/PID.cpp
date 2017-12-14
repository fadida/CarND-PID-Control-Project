#include "PID.h"
#include <iostream>
#include <cmath>

using namespace std;

constexpr int NUM_OF_PARAMS = 3;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->p_error = 0;
  this->i_error = 0;
  this->d_error = 0;

  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  this->mse       = 0;
  this->n_samples = 0;

  this->twiddleState = TWIDDLE_STATE_STOP;

  this->simulator_idle_speed = false;
}


void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;

  // MSE calculation
  ++n_samples;
  mse += cte * cte;

  // When twiddle is enabled - update state only when vehicle is stuck or we reached max samples.
  if (twiddleState != TWIDDLE_STATE_STOP && (simulator_idle_speed || n_samples == twiddle_max_samples)) {
    twiddleUpdateState(cte);
    clear();
  }

}

double PID::TotalError() {
  return Kp * p_error + Ki * i_error + Kd * d_error;
}

void PID::startTwiddle(double twiddle_tolerance, int max_samples, double delta) {
  this->twiddleState           = TWIDDLE_STATE_START;
  this->twiddle_tolerance      = twiddle_tolerance;
  this->twiddle_max_samples    = max_samples;
  this->twiddle_curr_param_idx = 0;

  this->twiddle_delta_Kp = delta;
  this->twiddle_delta_Ki = delta;
  this->twiddle_delta_Kd = delta;

  this->twiddle_best_mse     = INFINITY;
}

void PID::stopTwiddle() {
  cout << "Twiddle state: TWIDDLE_STATE_STOP" << endl;
  this->twiddleState = TWIDDLE_STATE_STOP;
}


double PID::getMSE() {
  if (n_samples && mse != INFINITY) {
    return mse / n_samples;
  } else {
    return INFINITY;
  }
}

void PID::clear() {
  p_error = 0;
  i_error = 0;
  d_error = 0;

  mse       = 0;
  n_samples = 0;

  // Send reset request to simulator.
  simulator_reset_request = true;

  simulator_idle_speed = false;
}

double& PID::twiddleParam() {
  switch (twiddle_curr_param_idx) {
    case 0:
      return Kp;
    case 1:
      return Ki;
    default:
      return Kd;
  }
}

double& PID::twiddleParamDelta() {
  switch (twiddle_curr_param_idx) {
    case 0:
      return twiddle_delta_Kp;
    case 1:
      return twiddle_delta_Ki;
    default:
      return twiddle_delta_Kd;
  }
}

void PID::twiddleUpdateState(double cte) {
  double mse       = getMSE();
  double sum_delta = twiddle_delta_Kd + twiddle_delta_Ki + twiddle_delta_Kp;

  switch (twiddleState) {
    case TWIDDLE_STATE_START:
      // debug prints
      cout << "Twiddle state: TWIDDLE_STATE_START" << endl;
      cout << "Tweaking param" << twiddle_curr_param_idx << endl;
      cout << "Min MSE=" << twiddle_best_mse << endl;
      cout << "Deltas=[" << twiddle_delta_Kp << ", " <<
               twiddle_delta_Ki << ", " << twiddle_delta_Kd << "]" << endl;

      // Check if reached to twiddle tolerance.
      if (twiddle_curr_param_idx == 0 && sum_delta <= twiddle_tolerance) {
        cout << "Twiddle: done twiddle.";
        cout << "Kp=" << Kp << ", Ki=" << Ki << ", Kd=" << Kd << endl;
        stopTwiddle();
      } else {
        twiddleParam() += twiddleParamDelta();
        twiddleState    = TWIDDLE_STATE_POS_DELTA;
      }

      if (twiddle_best_mse == INFINITY) {
        twiddle_best_mse = mse;
      }
      break;

    case TWIDDLE_STATE_POS_DELTA:
      cout << "Twiddle state: TWIDDLE_POS_DELTA" << endl;
      if (mse < twiddle_best_mse) {
        twiddle_best_mse     = mse;
        twiddleParamDelta() *= 1.1;

        twiddleState = TWIDDLE_STATE_CHANGE_PARAMS;
      } else {
        twiddleParam() -= 2* twiddleParamDelta();
        twiddleState    = TWIDDLE_STATE_NEG_DELTA;
      }
      break;

    case TWIDDLE_STATE_NEG_DELTA:
      cout << "Twiddle state: TWIDDLE_NEG_DELTA" << endl;
      if (mse < twiddle_best_mse) {
        twiddle_best_mse = mse;
        twiddleParamDelta() *= 1.1;
      } else {
        twiddleParam()      += twiddleParamDelta();
        twiddleParamDelta() *= 0.9;
      }

      twiddleState = TWIDDLE_STATE_CHANGE_PARAMS;

      break;

    case TWIDDLE_STATE_CHANGE_PARAMS:
      cout << "Twiddle state: TWIDDLE_STATE_CHANGE_PARAMS" << endl;
      twiddle_curr_param_idx = (twiddle_curr_param_idx + 1) % NUM_OF_PARAMS;
      twiddleState           = TWIDDLE_STATE_START;
      twiddleUpdateState(cte);
      return;

    default:
      cerr << "ERROR: The default state for twiddle shouldn't be accessed." << endl;
      stopTwiddle();
      break;
  }

  cout << "PID coeffs=[" << Kp << ", " << Ki << ", " << Kd << "]" << endl;
}
