from omni.isaac.sensor import IMUSensor, Camera
from omni.isaac.core.utils.rotations import quat_to_rot_matrix

import os
import numpy as np
import torch

class RLCamera:
    def __init__(self, prim_path:str, sensor_param:dict):
        self.camera = Camera(prim_path=prim_path, 
                             frequency=sensor_param["frequency"], 
                             resolution=sensor_param["resolution"])
        # self.camera.add_distance_to_image_plane_to_frame() #<- this does not enable depth image
        self.camera.initialize()
        self.sensor_param = sensor_param
    
    def get_observation(self):
        frame_data = self.camera.get_current_frame()
        rgb = frame_data["rgba"]
        # rgb = torch.from_numpy(frame_data["rgba"][:, :, :3]).permute(2, 1, 0)
        return {"rgb":rgb}


class RLIMU:
    def __init__(self, prim_path:str, sensor_param:dict):
        self.imu = IMUSensor(prim_path=prim_path, name="imu", frequency=sensor_param["frequency"])
        self.imu.initialize()

    def _get_sensor_to_world_transorm(self):
        position, rotation = self.imu.get_world_pose()
        rotation = quat_to_rot_matrix(rotation)
        transform = np.zeros((4, 4))
        transform[:3, :3] = rotation.T
        transform[:3, 3] = -rotation.T @ position.T
        return transform
    
    def get_observation(self):
        """
        get linear accelerationa and angular veocity in imu optical coordinate
        """
        frame_data = self.imu.get_current_frame()
        # imu2global_rt = self._get_sensor_to_world_transorm()
        # lin_acc = imu2global_rt[:3, :3] @ frame_data["lin_acc"].T
        # ang_vel = imu2global_rt[:3, :3] @ frame_data["ang_vel"].T
        lin_acc = frame_data["lin_acc"]
        ang_vel = frame_data["ang_vel"]
        # lin_acc = torch.from_numpy(frame_data["lin_acc"])
        # ang_vel = torch.from_numpy(frame_data["ang_vel"])
        return {"lin_acc":lin_acc, "ang_vel":ang_vel}

class SensorFactory:
    """
    Factory class to create tasks."""

    def __init__(self):
        self.creators = {}

    def register(self, name: str, sensor):
        """
        Registers a new task."""
        self.creators[name] = sensor

    def get(
        self, name: str
    ) -> object:
        """
        Returns a task."""
        assert name in self.creators.keys(), f"{name} not in {self.creators.keys()}"
        return self.creators[name]

sensor_factory = SensorFactory()
sensor_factory.register("RLCamera", RLCamera)
sensor_factory.register("RLIMU", RLIMU)
    
class RLSensors:
    def __init__(self, sensor_cfg:dict):
        self.sensors = []
        for sensor_type, sensor_property in sensor_cfg.items():
            sensor = sensor_factory.get(sensor_type)(sensor_property["prim_path"], sensor_property["params"])
            self.sensors.append(sensor)
            # if sensor_type == "RLIMU":
            #     sensor = sensor_factory.get(sensor_type)(sensor_property["prim_path"], sensor_property["params"])
            #     self.sensors.append(sensor)
    
    def get_observation(self):
        obs = {}
        for sensor in self.sensors:
            sensor_obs = sensor.get_observation()
            obs.update(sensor_obs)
        return obs