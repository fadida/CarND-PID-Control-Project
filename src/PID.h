#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp = 1, double Ki = 1, double Kd = 1);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
   * Starts the twiddle algorithm in order to tweak the filter parameters.
   *
   */
  void startTwiddle(double twiddle_tolerance, int max_samples, double delta);


  /*
   * Stops the twiddle algorithm.
   */
  void stopTwiddle();

  /*
   * Gets the filter mean square error.
   */
  double getMSE();

  /* Flag for reset request. */
  bool simulator_reset_request;

  /* signals PID that vehicle is not moving. */
  bool simulator_idle_speed;

private:
  /* The sum of square errors over n_samples. */
  double mse;

  /* The number of samples the PID processed. */
  long   n_samples;

  /* ============== TWIDDLE Related params - START ======================*/
  enum TwiddleState {
    TWIDDLE_STATE_START,            /* Twiddle initial state. */
    TWIDDLE_STATE_CHANGE_PARAMS,    /* Twiddle state used for changing the parameter to tweak. */
    TWIDDLE_STATE_POS_DELTA,        /* Twiddle state used to check if increasing the parameter will reduce the error. */
    TWIDDLE_STATE_NEG_DELTA,        /* Twiddle state used to check if decreasing the parameter will reduce the error. */
    TWIDDLE_STATE_STOP              /* Twiddle final state or disabled state. */
  };

  /* The current twiddle state. */
  TwiddleState twiddleState;

  /* Twiddle configuration pararms*/
  double twiddle_tolerance;
  int    twiddle_max_samples;
  double twiddle_delta_Kp;
  double twiddle_delta_Ki;
  double twiddle_delta_Kd;

  /* The best error twiddle found. */
  double twiddle_best_mse;

  /* The index for the parameter that twiddle currently tweaks. */
  int twiddle_curr_param_idx;

  /*
   * Updates twiddle state according to state machine.
   */
  void twiddleUpdateState(double cte);

  /*
   * Gets the current parameter that twiddle currently tweaks.
   */
  double& twiddleParam();

  /*
   * Gets the delta that twiddle currently uses.
   */
  double& twiddleParamDelta();


  /* ============== TWIDDLE Related params - END ======================*/

  /*
   * Clears PID errors & sample count.
   */
  void clear();

};

#endif /* PID_H */
