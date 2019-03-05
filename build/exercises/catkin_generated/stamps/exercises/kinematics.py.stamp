from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import collections
import matplotlib
import matplotlib.pylab as plt
import numpy as np
import time
import math


# Constants used for indexing.
X = 0
Y = 1
YAW = 2

# Drawing constants.
REFRESH_RATE = 1. / 15.


def euler(current_pose, t, dt):
  next_pose = current_pose.copy()
  u = 0.25
  #w = np.cos(t)
  w = np.cos(math.floor(t))
  
  # MISSING: Use Euler's integration method to return the next pose of our robot.
  # https://en.wikipedia.org/wiki/Euler_method
  # t is the current time.
  # dt is the time-step duration.
  # current_pose[X] is the current x position.
  # current_pose[Y] is the current y position.
  # current_pose[YAW] is the current orientation of the robot.
  # Update next_pose[X], next_pose[Y], next_pose[YAW].
  next_pose[X] = current_pose[X] + dt * u * np.cos(current_pose[YAW])
  next_pose[Y] = current_pose[Y] + dt * u * np.sin(current_pose[YAW])
  next_pose[YAW] = current_pose[YAW] + dt * w

  return next_pose


def rk4(current_pose, t_input, dt): #changed t to t_input for ex1 (f)
  next_pose = current_pose.copy()
  u = 0.25
  #t = t_input
  t = math.floor(t_input)
  
  # MISSING: Use classical Runge-Kutta to return the next pose of our robot.
  # https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
  # t is the current time.
  # dt is the time-step duration.
  # current_pose[X] is the current x position.
  # current_pose[Y] is the current y position.
  # current_pose[YAW] is the current orientation of the robot.
  # Update next_pose[X], next_pose[Y], next_pose[YAW].
  
  #X, Y, YAW are only dependent on time, so we use only a portion of the full RK4 equations
  #get theta(t+dt) first, since we need it for X and Y
  wk1 = dt * np.cos(t)
  wk2 = dt * np.cos(t+dt/2.0)
  wk3 = wk2
  wk4 = dt * np.cos(t+dt)
  next_pose[YAW] = current_pose[YAW] + (wk1+2*wk2+2*wk3+wk4)/6.0  
  
  #we also need theta(t+dt/2) for X and Y calculations
  wwk1 = dt/2.0 * np.cos(t)
  wwk2 = dt/2.0 * np.cos(t+dt/4.0)
  wwk3 = wk2
  wwk4 = dt/2.0 * np.cos(t+dt/2.0)
  ww = current_pose[YAW] + (wk1+2*wk2+2*wk3+wk4)/6.0

  xk1 = dt * u * np.cos(current_pose[YAW])
  xk2 = dt * u * np.cos(ww)
  xk3 = xk2
  xk4 = dt * u * np.cos(next_pose[YAW])
  next_pose[X] = current_pose[X] + (xk1+2*xk2+2*xk3+xk4)/6.0

  yk1 = dt * u * np.sin(current_pose[YAW])
  yk2 = dt * u * np.sin(ww)
  yk3 = yk2
  yk4 = dt * u * np.sin(next_pose[YAW])
  next_pose[Y] = current_pose[Y] + (yk1+2*yk2+2*yk3+yk4)/6.0
  
  return next_pose


def main(args):
  print('Using method {}'.format(args.method))
  integration_method = globals()[args.method]

  fig = plt.figure()
  ax = fig.add_subplot(111)
  plt.ion()  # Interactive mode.
  plt.grid('on')
  plt.axis('equal')
  plt.xlim([-0.5, 2]) #was ~2
  plt.ylim([-0.75, 1.25])#was ~1.25
  plt.show()
  colors = colors_from('jet', len(args.dt))

  # Show all dt.
  for color, dt in zip(colors, args.dt):
    print('Using dt = {}'.format(dt))

    # Initial robot pose (x, y and theta).
    robot_pose = np.array([0., 0., 0.], dtype=np.float32)
    robot_drawer = RobotDrawer(ax, robot_pose, color=color, label='dt = %.3f [s]' % dt)
    if args.animate:
      fig.canvas.draw()
      fig.canvas.flush_events()

    # Simulate for 10 seconds.
    last_time_drawn = 0.
    last_time_drawn_real = time.time()
    for t in np.arange(0., 10., dt):
      robot_pose = integration_method(robot_pose, t, dt)
      
      plt.title('time = %.3f [s] with dt = %.3f [s]' % (t + dt, dt))
      robot_drawer.update(robot_pose)

      # Do not draw too many frames.
      time_drawn = t
      if args.animate and (time_drawn - last_time_drawn > REFRESH_RATE):
        # Try to draw in real-time.
        time_drawn_real = time.time()
        delta_time_real = time_drawn_real - last_time_drawn_real
        if delta_time_real < REFRESH_RATE:
          time.sleep(REFRESH_RATE - delta_time_real)
        last_time_drawn_real = time_drawn_real
        last_time_drawn = time_drawn
        fig.canvas.draw()
        fig.canvas.flush_events()
    robot_drawer.done()

  plt.ioff()
  plt.title('Trajectories')
  plt.legend(loc='lower right')
  plt.show(block=True)


# Simple class to draw and animate a robot.
class RobotDrawer(object):

  def __init__(self, ax, pose, radius=.05, label=None, color='g'):
    self._pose = pose.copy()
    self._radius = radius
    self._history_x = [pose[X]]
    self._history_y = [pose[Y]]
    self._outside = ax.plot([], [], 'b', lw=2)[0]
    self._front = ax.plot([], [], 'b', lw=2)[0]
    self._path = ax.plot([], [], c=color, lw=2, label=label)[0]
    self.draw()

  def update(self, pose):
    self._pose = pose.copy()
    self._history_x.append(pose[X])
    self._history_y.append(pose[Y])
    self.draw()

  def draw(self):
    a = np.linspace(0., 2 * np.pi, 20)
    x = np.cos(a) * self._radius + self._pose[X]
    y = np.sin(a) * self._radius + self._pose[Y]
    self._outside.set_data(x, y)
    r = np.array([0., self._radius])
    x = np.cos(self._pose[YAW]) * r + self._pose[X]
    y = np.sin(self._pose[YAW]) * r + self._pose[Y]
    self._front.set_data(x, y)
    self._path.set_data(self._history_x, self._history_y)

  def done(self):
    self._outside.set_data([], [])
    self._front.set_data([], [])


def colors_from(cmap_name, ncolors):
    cm = plt.get_cmap(cmap_name)
    cm_norm = matplotlib.colors.Normalize(vmin=0, vmax=ncolors - 1)
    scalar_map = matplotlib.cm.ScalarMappable(norm=cm_norm, cmap=cm)
    return [scalar_map.to_rgba(i) for i in range(ncolors)]


def positive_floats(string):
  values = tuple(float(v) for v in string.split(','))
  for v in values:
    if v <= 0.:
      raise argparse.ArgumentTypeError('{} is not strictly positive.'.format(v))
  return values


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Launches a battery of experiments in parallel')
  parser.add_argument('--method', action='store', default='euler', help='Integration method.', choices=['euler', 'rk4'])
  parser.add_argument('--dt', type=positive_floats, action='store', default=(0.05,), help='Integration step.')
  parser.add_argument('--animate', action='store_true', default=False, help='Whether to animate.')
  args = parser.parse_args()
  main(args)
