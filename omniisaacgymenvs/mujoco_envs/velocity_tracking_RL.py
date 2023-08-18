import matplotlib.pyplot as plt
import numpy as np
import argparse
import mujoco
import torch 
import os

from omniisaacgymenvs.mujoco_envs.mujoco_base_env import MuJoCoFloatingPlatform
from omniisaacgymenvs.mujoco_envs.RL_games_model_4_mujoco import RLGamesModel

class MuJoCoVelTracking(MuJoCoFloatingPlatform):
    def __init__(self, step_time:float = 0.02, duration:float = 60.0, inv_play_rate:int = 10,
                 mass:float = 5.32, max_thrust:float = 1.0, radius:float = 0.31) -> None:
        super().__init__(step_time, duration, inv_play_rate, mass, max_thrust, radius)

    def initializeLoggers(self) -> None:
        super().initializeLoggers()
        self.velocity_goal = []
        self.position_target = []

    def updateLoggers(self, goal, target) -> None:
        super().updateLoggers()
        self.velocity_goal.append(goal)
        self.position_target.append(target)
    

    def runLoop(self, model, xy: np.ndarray) -> None:
        """
        Runs the simulation loop.
        model: the agent.
        xy: 2D position of the body."""

        self.resetPosition() # Resets the position of the body.
        self.data.qpos[:2] = xy # Sets the position of the body.

        while (self.duration > self.data.time) and (model.isDone() == False):
            state = self.updateState() # Updates the state of the simulation.
            # Get the actions from the controller
            action = model.getAction(state)
            # Plays only once every self.inv_play_rate steps.
            for _ in range(self.inv_play_rate):
                self.applyForces(action)
                mujoco.mj_step(self.model, self.data)
                self.updateLoggers(model.get_goal(), model.get_target_position())
    
    def plotSimulation(self, dpi:int = 90, width:int = 1000, height:int = 1000, save:bool = False) -> None:
        """
        Plots the simulation."""

        figsize = (width / dpi, height / dpi)

        fig, ax = plt.subplots(2, 1, figsize=figsize, dpi=dpi)

        ax[0].plot(self.timevals, self.angular_velocity)
        ax[0].set_title('angular velocity')
        ax[0].set_ylabel('radians / second')

        ax[1].plot(self.timevals, self.linear_velocity, label="system velocities")
        ax[1].plot(self.timevals, self.velocity_goal, label="target velocities")
        ax[1].legend()
        ax[1].set_xlabel('time (seconds)')
        ax[1].set_ylabel('meters / second')
        _ = ax[1].set_title('linear_velocity')
        if save:
            fig.savefig("test_velocities.png")

        fig, ax = plt.subplots(1, 1, figsize=figsize, dpi=dpi)
        ax.plot(np.array(self.position_target)[:,0], np.array(self.position_target)[:,1], label="trajectory")
        ax.plot(np.array(self.position)[:,0], np.array(self.position)[:,1], label="system position")
        ax.legend()
        ax.set_xlabel('meters')
        ax.set_ylabel('meters')
        ax.axis("equal")
        _ = ax.set_title('x y coordinates')
        plt.tight_layout()
        if save:
            fig.savefig("test_positions.png")

class TrajectoryTracker:
    def __init__(self, lookahead=0.25, closed=False):
        self.current_point = -1
        self.lookhead = lookahead
        self.closed = closed
        self.is_done = False

    def generateCircle(self, radius=2, num_points=360*10):
        theta = np.linspace(0, 2*np.pi, num_points, endpoint=(not self.closed))
        self.positions = np.array([np.cos(theta) * radius, np.sin(theta) * radius]).T
        self.angles = np.array([-np.sin(theta), np.cos(theta)]).T

    def generateSquare(self, h=2, num_points=360*10):
        points_per_side = num_points // 4
        s1y = np.linspace(-h/2,h/2, num_points, endpoint=False)
        s1x = np.ones_like(s1y)*h/2
        s2x = np.linspace(h/2,-h/2, num_points, endpoint=False)
        s2y = np.ones_like(s2x) * h/2
        s3y = np.linspace(h/2,-h/2, num_points, endpoint=False)
        s3x = np.ones_like(s3y) * (-h/2)
        s4x = np.linspace(-h/2,h/2, num_points, endpoint=False)
        s4y = np.ones_like(s4x) * (-h/2)
        self.positions = np.vstack([np.hstack([s1x,s2x,s3x,s4x]), np.hstack([s1y,s2y,s3y,s4y])]).T
        self.angles = np.ones_like(self.positions)#np.array([-np.sin(theta), np.cos(theta)]).T

    def generateSpiral(self, start_radius=0.5, end_radius=2, num_loop=5, num_points=360*20):
        radius = np.linspace(start_radius, end_radius, num_points, endpoint=(not self.closed))
        theta = np.linspace(0, 2*np.pi*num_loop, num_points, endpoint=(not self.closed))
        self.positions = np.array([np.cos(theta) * radius, np.sin(theta) * radius]).T
        self.angles = np.array([-np.sin(theta), np.cos(theta)]).T
    
    def getTrackingPointIdx(self, position):
        distances = np.linalg.norm(self.positions - position, axis=1)
        if self.current_point == -1:
            self.current_point = 0
        else:
            indices = np.where(distances < self.lookhead)[0]
            if len(indices) > 0:
                indices = indices[indices < 60]
                if len(indices) > 0:
                    self.current_point = np.max(indices)

    def rollTrajectory(self):
        if self.closed:
            self.positions = np.roll(self.positions, -self.current_point, axis=0)
            self.angles = np.roll(self.angles, -self.current_point, axis=0)
            self.current_point = 0
        else:
            self.positions = self.positions[self.current_point:]
            self.angles = self.angles[self.current_point:]
            self.current_point = 0
        
        if self.positions.shape[0] <= 1:
            self.is_done = True

    def getPointForTracking(self):
        position = self.positions[self.current_point]
        angle = self.angles[self.current_point]
        self.rollTrajectory()
        return position, angle
    
    def get_target_position(self):
        return self.target_position
    
    def computeVelocityVector(self, target_position, position):
        diff = target_position - position
        return diff / np.linalg.norm(diff)
    
    def getVelocityVector(self, position):
        self.getTrackingPointIdx(position)
        self.target_position, target_angle = self.getPointForTracking()
        velocity_vector = self.computeVelocityVector(self.target_position, position)
        return velocity_vector
    
class VelocityTracker:
    def __init__(self, trajectory_tracker: TrajectoryTracker, model: RLGamesModel, target_tracking_velocity:float = 0.25):
        self.trajectory_tracker = trajectory_tracker
        self.model = model
        self.target_tracking_velocity = target_tracking_velocity

        self.obs_state = torch.zeros((1,10), dtype=torch.float32, device="cuda")
    
    def getGoal(self):
        return self.velocity_vector*self.target_tracking_velocity
    
    def getTargetPosition(self):
        return self.trajectory_tracker.get_target_position()
    
    def isDone(self):
        return self.trajectory_tracker.is_done

    def makeObservationBuffer(self, state, velocity_vector):
        self.obs_state[0,:2] = torch.tensor(state["orientation"], dtype=torch.float32, device="cuda")
        self.obs_state[0,2:4] = torch.tensor(state["linear_velocity"], dtype=torch.float32, device="cuda")
        self.obs_state[0,4] = state["angular_velocity"]
        self.obs_state[0,5] = 2
        self.obs_state[0,6:8] = torch.tensor(velocity_vector, dtype=torch.float32, device="cuda")

    def getAction(self, state):
        self.velocity_vector = self.trajectory_tracker.getVelocityVector(state["position"])
        velocity_goal = self.velocity_vector*self.target_tracking_velocity - state["linear_velocity"]
        self.makeObservationBuffer(state, velocity_goal)
        action = self.model.getAction(self.obs_state)
        return action

def parseArgs():
    parser = argparse.ArgumentParser("Generates meshes out of Digital Elevation Models (DEMs) or Heightmaps.")
    parser.add_argument("--model_path", type=str, default=None, help="The path to the model to be loaded. It must be a velocity tracking model.")
    parser.add_argument("--config_path", type=str, default=None, help="The path to the network configuration to be loaded.")
    parser.add_argument("--trajectory_type", type=str, default="Circle", help="The type of trajectory to be generated. Options are: Circle, Square, Spiral.")
    parser.add_argument("--radius", type=float, default=1.5, help="The radius of the circle trajectory. In meters.")
    parser.add_argument("--height", type=float, default=3.0, help="The height of the square trajectory. In meters.")
    parser.add_argument("--start_radius", type=float, default=0.5, help="The starting radius for the spiral for the spiral trajectory. In meters.")
    parser.add_argument("--end_radius", type=float, default=2.0, help="The final radius for the spiral trajectory. In meters.")
    parser.add_argument("--num_loop", type=float, default=5.0, help="The number of loops the spiral trajectory should make. Must be greater than 0.")
    parser.add_argument("--closed", type=bool, default=True, help="Whether the trajectory is closed (it forms a loop) or not.")
    parser.add_argument("--lookahead_dist", type=float, default=0.15, help="How far the velocity tracker looks to generate the velocity vector that will track the trajectory. In meters.")
    parser.add_argument("--sim_duration", type=float, default=240, help="The length of the simulation. In seconds.")
    parser.add_argument("--play_rate", type=float, default=5.0, help="The frequency at which the agent will played. In Hz. Note, that this depends on the sim_rate, the agent my not be able to play at this rate depending on the sim_rate value. To be consise, the agent will play at: sim_rate / int(sim_rate/play_rate)")
    parser.add_argument("--sim_rate", type=float, default=50.0, help="The frequency at which the simulation will run. In Hz.")
    parser.add_argument("--tracking velocity", type=float, default=0.25, help="The tracking velocity. In meters per second.")
    parser.add_argument("--save_dir", type=str, default="velocity_exp", help="The path to the folder in which the results will be stored.")
    parser.add_argument("--platform_mass", type=float, default=5.32, help="The mass of the floating platform. In Kg.")
    parser.add_argument("--platform_radius", type=float, default=0.31, help="The radius of the floating platform. In meters.")
    parser.add_argument("--platform_max_thrust", type=float, default=1.0, help="The maximum thrust of the floating platform. In newtons.")
    args, unknown_args = parser.parse_known_args()
    return args, unknown_args

if __name__ == "__main__":
    # Collects args
    args, _ = parseArgs()
    # Checks args
    assert os.path.exists(args.model_path), "The model file does not exist."
    assert os.path.exists(args.config_path), "The configuration file does not exist."
    assert args.sim_rate > args.play_rate, "The simulation rate must be greater than the play rate."
    assert args.num_loop > 0, "The number of loops must be greater than 0."
    assert args.lookahead_dist > 0, "The lookahead distance must be greater than 0."
    assert args.radius > 0, "The radius must be greater than 0."
    assert args.start_radius > 0, "The start radius must be greater than 0."
    assert args.end_radius > 0, "The end radius must be greater than 0."
    assert args.height > 0, "The height must be greater than 0."
    assert args.sim_duration > 0, "The simulation duration must be greater than 0."
    assert args.play_rate > 0, "The play rate must be greater than 0."
    assert args.sim_rate > 0, "The simulation rate must be greater than 0."
    assert args.tracking_velocity > 0, "The tracking velocity must be greater than 0."
    # Try to create the save directory
    try:
        os.makedirs(args.save_dir, exist_ok=True)
    except:
        raise ValueError("Could not create the save directory.")
    # Creates the trajectory tracker
    tracker = TrajectoryTracker(lookahead=args.lookahead_dist, closed=args.closed)
    if args.trajectory_type.lower() == "square":
        tracker.generateSquare(h=args.height)
    elif args.trajectory_type.lower() == "circle":
        tracker.generateCircle(radius=args.radius)
    elif args.trajectory_type.lower() == "spiral":
        tracker.generateSpiral(start_radius=args.start_radius, end_radius=args.end_radius, num_loop=args.num_loop)
    else:
        raise ValueError("Unknown trajectory type. Must be square, circle or spiral.")
    # Instantiates the RL agent
    model = RLGamesModel(args.config_path, args.model_path)
    #  Creates the velocity tracker
    velocity_tracker = VelocityTracker(tracker, model)
    # Creates the environment
    env = MuJoCoVelTracking(step_time=1.0/args.sim_rate, duration=args.sim_duration, inv_play_rate=int(args.sim_rate/args.play_rate),
                            mass=args.platform_mass, radius=args.platform_radius, max_thrust=args.platform_max_thrust)
    # Runs the simulation
    env.runLoop(velocity_tracker, [0,0])
    # Plots the simulation
    env.plotSimulation(args.save_dir)
    # Saves the simulation data
    env.saveSimulationData(args.save_dir)