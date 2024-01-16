def generate_cfg(env_path:str):
    cfg = {
        "geom": 
            {
            "articulation_root": [f"{env_path}/D455", [0, 0.3, 0, 0, 0, 0]], 
            "sensor_base": ["camera_link", "/home/lunar5/jnskkmhr/omn_isaacgym_ws/RANS/omniisaacgymenvs/robots/usd/D455_mesh.usd"],
            "links" : ["camera_color_frame", "camera_color_optical_frame", "camera_imu_frame", "camera_imu_optical_frame"], 
            "joints" : [
                ["camera_link", "camera_color_frame", "rigid", [0.011, 0, 0, 0, 0, 0]], 
                ["camera_link", "camera_color_optical_frame", "rigid", [0.011, 0, 0, -90, 0, 0]], 
                ["camera_link", "camera_imu_frame", "rigid", [-0.017, -0.016, 0.007, 0, 0, 0]], 
                ["camera_link", "camera_imu_optical_frame", "rigid", [-0.017, -0.016, 0.007, -90, 0, 0]], 
                        ]
            }, 
        "sensor":
                {
                    "RLCamera":
                    {
                    "prim_path" : f"{env_path}/D455/camera_color_optical_frame/Camera", 
                    "rotation": (180, 0, 0), 
                    "params": {"focalLength":1.93, "focusDistance":50.0, 
                               "clippingRange":(0.1, 50.0), "resolution":(1280, 720), "frequency":20, 
                               "horizontalAperture":3.60, "verticalAperture":2.70
                               }
                    }, 
                    "RLIMU":
                    {
                    "prim_path": f"{env_path}/D455/camera_imu_optical_frame/Imu", 
                    "params": {"frequency":40},
                    }
                }
                }
    return cfg