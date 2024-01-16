from omni.isaac.sensor import IMUSensor, Camera
from omni.isaac.core.utils.rotations import quat_to_rot_matrix
import omni.replicator.core as rep

import os
import numpy as np
import cv2
import torch

from omniisaacgymenvs.robots.sensors.writer import writer_factory

class RLCamera:
    def __init__(self, prim_path:str, sensor_param:dict):
        self.sensor_param = sensor_param
        self.render_product = rep.create.render_product(
            prim_path, 
            resolution=sensor_param["resolution"])
        self.annotators = {}
        self.writers = {}
        self.enable_rgb()
        self.enable_depth()
    
    def enable_rgb(self):
        rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb_annot.attach([self.render_product])
        self.annotators.update({"rgb":rgb_annot})
        self.writers.update({"rgb":writer_factory.get("RGBWriter")(add_noise=False)})
    
    def enable_depth(self):
        depth_annot = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
        depth_annot.attach([self.render_product])
        self.annotators.update({"depth":depth_annot})
        self.writers.update({"depth":writer_factory.get("DepthWriter")(add_noise=False)})
    
    def get_observation(self):
        obs_buf = {}
        for modality, annotator in self.annotators.items(): 
            writer = self.writers[modality]
            data_pt = writer.get_data(annotator.get_data())
            obs_buf.update({modality:data_pt})
        return obs_buf


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
        # imu2global_rt = self._get_sensor_to_world_transorm()
        # lin_acc = imu2global_rt[:3, :3] @ frame_data["lin_acc"].T
        # ang_vel = imu2global_rt[:3, :3] @ frame_data["ang_vel"].T
        """
        frame_data = self.imu.get_current_frame()
        # -> I dont know why the output is already tensorized though..
        lin_acc = frame_data["lin_acc"]
        ang_vel = frame_data["ang_vel"]
        imu_reading = torch.cat((lin_acc, ang_vel))
        # lin_acc = torch.from_numpy(frame_data["lin_acc"])
        # ang_vel = torch.from_numpy(frame_data["ang_vel"])
        return {"imu":imu_reading}

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
    
    def get_observation(self):
        obs = {}
        for sensor in self.sensors:
            sensor_obs = sensor.get_observation()
            obs.update(sensor_obs)
        return obs