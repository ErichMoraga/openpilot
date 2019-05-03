import numpy as np
from common.numpy_fast import clip, interp
import time
import warnings

try:
    # get monotonic time to ensure that time deltas are always positive
    _current_time = time.monotonic
except AttributeError:
    # time.monotonic() not available (using python < 3.3), fallback to time.time()
    _current_time = time.time
    warnings.warn('time.monotonic() not available in python < 3.3, using time.time() as fallback')

def apply_deadzone(error, deadzone):
  if error > deadzone:
    error -= deadzone
  elif error < - deadzone:
    error += deadzone
  else:
    error = 0.
  return error

class LatPIController(object):
  def __init__(self, k_p, k_i, k_f=1., k_d=0, pos_limit=None, neg_limit=None, rate=100, sat_limit=0.8, convert=None):
    self._k_p = k_p # proportional gain
    self._k_i = k_i # integral gain
    self.k_f = k_f  # feedforward gain
    self.k_d = k_d  # derivative gain

    self.pos_limit = pos_limit
    self.neg_limit = neg_limit

    self.sat_count_rate = 1.0 / rate
    self.i_unwind_rate = 0.3 / rate
    self.i_rate = 1.0 / rate
    self.sat_limit = sat_limit
    self.convert = convert
    self._last_time = _current_time()
    self._last_output = None
    self._last_input = None
    self.sample_time = 0.01 # 100Hz PID
    self.proportional_on_measurement = False # http://brettbeauregard.com/blog/2017/06/introducing-proportional-on-measurement/

    self.reset()

  @property
  def k_p(self):
    return interp(self.speed, self._k_p[0], self._k_p[1])

  @property
  def k_i(self):
    return interp(self.speed, self._k_i[0], self._k_i[1])

  def _check_saturation(self, control, override, error):
    saturated = (control < self.neg_limit) or (control > self.pos_limit)

    if saturated and not override and abs(error) > 0.1:
      self.sat_count += self.sat_count_rate
    else:
      self.sat_count -= self.sat_count_rate

    self.sat_count = clip(self.sat_count, 0.0, 1.0)

    return self.sat_count > self.sat_limit

  def reset(self):
    self.p = 0.0
    self.i = 0.0
    self.f = 0.0
    self.d = 0.0
    self.sat_count = 0.0
    self.saturated = False
    self.control = 0

  def update(self, setpoint, measurement, speed=0.0, check_saturation=True, override=False, feedforward=0., deadzone=0., freeze_integrator=False):
    self.speed = speed

    now = _current_time()
    dt = now - self._last_time if now - self._last_time else 1e-16

    #if self.sample_time is not None and dt < self.sample_time and self._last_output is not None:
    #    # only update every sample_time seconds
    #    return self._last_output

    d_input = measurement - (self._last_input if self._last_input is not None else measurement)

    error = float(apply_deadzone(setpoint - measurement, deadzone))
    if not self.proportional_on_measurement:
        self.p = error * self.k_p
    else:
        self.p -= self.k_p * d_input
        # TODO: Test clipping of this val
        #self.p = clip(self.k_p, self.neg_limit, self.pos_limit)

    self.f = feedforward * self.k_f
    self.d = -self.k_d * d_input / dt #added derivative term

    if override:
      self.i -= self.i_unwind_rate * float(np.sign(self.i))
    else:
      #i = self.i + error * self.k_i * self.i_rate
      i = self.i + error * self.k_i * dt # still 0.01 (100Hz)
      control = self.p + self.f + self.d + i

      if self.convert is not None:
        control = self.convert(control, speed=self.speed)

      # Update when changing i will move the control away from the limits
      # or when i will move towards the sign of the error
      # I don't think this actually does anything after reviewing the code
      if ((error >= 0 and (control <= self.pos_limit or i < 0.0)) or \
          (error <= 0 and (control >= self.neg_limit or i > 0.0))) and \
         not freeze_integrator:
        self.i = i

    control = self.p + self.f + self.i + self.d
    if self.convert is not None:
      control = self.convert(control, speed=self.speed)

    if check_saturation:
      self.saturated = self._check_saturation(control, override, error)
    else:
      self.saturated = False

    self.control = clip(control, self.neg_limit, self.pos_limit)
    # keep track of state
    self._last_output = self.control
    self._last_input = measurement
    self._last_time = now
    #print("p,i,d,f",self.p,self.i,self.d,self.f,"measurement", measurement, "setpoint", setpoint, "control", control, "dt", dt, "d_input", d_input)
    return self.control
