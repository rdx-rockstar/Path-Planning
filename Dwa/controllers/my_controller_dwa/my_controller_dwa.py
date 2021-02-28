from enum import Enum
import numpy as np
import math
import time
from controller import Robot,GPS

robot = Robot()
timestep = int(robot.getBasicTimeStep())
left_motor= robot.getDevice('motor_1')
right_motor= robot.getDevice('motor_2')
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_ps=robot.getDevice('ps_1');
left_ps.enable(timestep);
right_ps=robot.getDevice('ps_2');
right_ps.enable(timestep)
gps=robot.getDevice('gps');
gps.enable(timestep)

class RobotType(Enum):
    circle = 0
    rectangle = 1
    
class Config:
    def __init__(self):
        # robot parameter
        self.max_speed = 5.5  
        self.min_speed = -3
        self.dist_btw_wheels=9;
        self.wheel_radius=2.5;
        self.dist_per_radian=self.wheel_radius
        self.max_yaw_rate = 30.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 10
        self.max_delta_yaw_rate = 120.0 * math.pi / 180.0  # [rad/ss]
        self.v_resolution = 0.5
        self.yaw_rate_resolution = 5 * math.pi / 180.0  # [rad/s]
        self.obstacle_radius=10
        self.dt = 0.2  # [s] Time tick for motion prediction
        self.predict_time =1.0 # [s]
        self.to_goal_cost_gain = 1.5
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 1.3
        self.robot_stuck_flag_cons = 0.001  # constant to prevent robot stucked
        self.robot_type = RobotType.circle
        self.board_width=500
        self.board_length=500
        self.robot_radius =10  # [m] for collision check
        self.ob = np.array([[-150,-160],[-125,-50],[0,0],[72,120]])
    @property
    def robot_type(self):
        return self._robot_type

    @robot_type.setter
    def robot_type(self, value):
        if not isinstance(value, RobotType):
            raise TypeError("robot_type must be an instance of RobotType")
        self._robot_type = value

config = Config()    

def dwa_control(x, config, goal, ob):
    dw = calc_dynamic_window(x, config)
    u, trajectory = calc_control_and_trajectory(x, dw, config, goal, ob)
    return u, trajectory
    
def calc_dynamic_window(x, config):
    Vs = [config.min_speed, config.max_speed,
          -config.max_yaw_rate, config.max_yaw_rate]
    # print("max yaw rate",x[4])
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_delta_yaw_rate * config.dt,
          x[4] + config.max_delta_yaw_rate * config.dt]

    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
    return dw
    
def calc_control_and_trajectory(x, dw, config, goal, ob):
    x_init = x[:]
    min_cost = float("inf")
    best_u = [0.0, 0.0]
    best_trajectory = np.array([x])
    bs=0;bg=0;bo=0;by=0;bv=0;
    for v in np.arange(dw[0], dw[1], config.v_resolution):
        print("vel=",v)
        for y in np.arange(dw[2], dw[3], config.yaw_rate_resolution):
            trajectory = predict_trajectory(x_init, v, y, config)
            # calc cost
            to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(trajectory, goal)
            speed_cost = config.speed_cost_gain * (config.max_speed - trajectory[-1, 3])
            ob_cost = config.obstacle_cost_gain * calc_obstacle_cost(trajectory, ob, config)
            print("degree=",math.degrees(y),":","obstacle cost=",ob_cost)
            final_cost = to_goal_cost + speed_cost + ob_cost
            # search minimum trajectory
            if min_cost >= final_cost:
                bo=ob_cost;bg=to_goal_cost;bs=speed_cost;bv=v;by=y;
                min_cost = final_cost
                best_u = [v, y]
                best_trajectory = trajectory
                if abs(best_u[0]) < config.robot_stuck_flag_cons and abs(x[3]) < config.robot_stuck_flag_cons:
                    best_u[1] = -config.max_delta_yaw_rate
    # print("\nall best obstacle cost=",bo," speed cost=",bs," goal cost=",bg,"best vel",bv,"best omega",math.degrees(by),"\n")
    print("total",min_cost,"goal cost=",int(bg),"obstacle cost=",bo,"best vel",int(bv),"best omega",int(math.degrees(by)),"\n")
    return best_u,best_trajectory

def predict_trajectory(x_init, v, y, config):
    x = np.array(x_init)
    trajectory = np.array(x)
    time = 0
    while time < config.predict_time:
        x = motion(x, [v, y], config.dt)
        trajectory = np.vstack((trajectory, x))
        time += config.dt

    return trajectory
    
def motion(x, u, dt):
    x[2] += u[1] * dt
    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[3] = u[0]
    x[4] = u[1]
    return x

def calc_obstacle_cost(trajectory, ob, config):
    ox = ob[:, 0]
    oy = ob[:, 1]
    m=min(config.board_length/2-abs(trajectory[-1,0]),config.board_width/2-abs(trajectory[-1,1]))
    if(m<config.robot_radius):
        return float("inf")
    dx = trajectory[:, 0] - ox[:, None]
    dy = trajectory[:, 1] - oy[:, None]
    r = np.hypot(dx, dy)

    if config.robot_type == RobotType.circle:
        if np.array(r <= config.robot_radius+config.obstacle_radius).any():
            return float("Inf")

    min_r = np.min(r)
    min_r=min(min_r,m)
    return 1000.0 / min_r  # OK

def calc_to_goal_cost(trajectory, goal):
    dx = goal[0] - trajectory[-1, 0]
    dy = goal[1] - trajectory[-1, 1]
    return math.hypot(dx,dy);

def move(x,u,ps):
    # print("in move")
    # print("initial ps",ps)
    d_theta=u[1]*config.predict_time;
    v_left=(2*u[0]-config.dist_btw_wheels*u[1]/2)/2
    v_right=(2*u[0]+config.dist_btw_wheels*u[1]/2)/2
    # print("omega=",math.degrees(u[1]),"lenier vel",u[0],"l vel=",v_left,"r vel=",v_right)
    # print("therotical d_theta=",math.degrees(d_theta))
    start_time=robot.getTime();
    end_time=start_time;
    # print("S",start_time)
    right_motor.setVelocity(v_right)
    left_motor.setVelocity(v_left)
    while robot.step(timestep) != -1:
        right_motor.setVelocity(v_right)
        left_motor.setVelocity(v_left)
        end_time=robot.getTime()
        if(end_time-start_time>=config.predict_time-0.016):
            break
    # print("E",end_time)
    n_ps=[left_ps.getValue(),right_ps.getValue()]
    d_right=(n_ps[1]-ps[1])*config.dist_per_radian
    d_left=(n_ps[0]-ps[0])*config.dist_per_radian
    d_theta=(d_left-d_right)/config.dist_btw_wheels
    # print("end ps",n_ps)
    # print("atual d_theta=",math.degrees(d_theta),"dr=",d_right,"dl=",d_left)
    if(d_theta<0.0001):
        x[0]+=((d_right+d_left)/2)*math.cos(x[2])
        x[1]+=((d_right+d_left)/2)*math.sin(x[2])
    else:
        radius=(d_right/d_theta)+config.dist_btw_wheels/2;
        x[0]+=radius*(math.sin(x[2]-d_theta)-math.sin(x[2]));
        x[1]+=radius*(math.cos(x[2]-d_theta)-math.cos(x[2]));
    x[2]-=d_theta;
    x[3]=u[0]
    x[4]=u[1]
    return x,n_ps

def main(ix=0,iy=0,theta=1.57,gx=2.2, gy=2.2, robot_type=RobotType.circle):
    gps=robot.getDevice('gps');
    gps.enable(timestep)
    max_speed=config.max_speed
    x = np.array([ix,iy,theta, 0.0, 0.0])
    ps=[0.0,0.0]
    goal = np.array([gx, gy])
    config.robot_type = robot_type
    trajectory = np.array(x)
    ob = config.ob
    while robot.step(timestep) != -1:
        u, predicted_trajectory = dwa_control(x, config, goal, ob)
        print("fr",x);
        x,ps= move(x,u,ps)
        # print("predicted",predicted_trajectory[-1])
        # print("actual",x)
        g_val=gps.getValues();
        x[0]=g_val[2]*100
        x[1]=g_val[0]*100
        print("to",x);
        # g_val[0],g_val[2]=g_val[2],g_val[0]
        # print("gps",g_val);
        right_motor.setVelocity(0.0)
        left_motor.setVelocity(0.0)
        trajectory = np.vstack((trajectory, x))  
        dist_to_goal = math.hypot(x[0] - goal[0], x[1] - goal[1])
        print(".\n.")
        if dist_to_goal <= 2*config.robot_radius:
            print("Goal!!")
            break
    print("Done")

main(-210,-210,1.57,230,220,RobotType.circle)