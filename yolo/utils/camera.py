import pyrealsense2 as rs

def initialize_config(pipeline, device_number):
    config = rs.config()
    config.enable_device(device_number)
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    if device_product_line == 'L500':
        config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    else:
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    return config

def start_camera(device_number):
    pipeline = rs.pipeline()
    config = initialize_config(pipeline, device_number)
    pipeline.start(config)
    return (pipeline, config)

class CameraSwitcher:
    def __init__(self, ids):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        if len(ids) < 1:
            raise Exception("Array \"ids\" must include at least 1 id. 0 were provided.")
        self.ids = ids

    def start(self, camera_id=''):
        if camera_id == '':
            camera_id = self.ids[0]
        if not camera_id in self.ids:
            raise Exception("Camera ID provided does not exist in array \"ids\".")
        self.pipeline, self.config = start_camera(camera_id)
        self.active_id = camera_id

    def switch_camera(self, target_camera=''):
        oconfig = self.config
        self.pipeline.stop()
        try:
            if not target_camera == '':
                if not target_camera in self.ids:
                    raise Exception("Camera ID provided does not exist in array \"ids\".")
                self.config = initialize_config(self.pipeline, target_camera)
                self.pipeline.start(self.config)
                self.active_id = target_camera
            else:
                print("SWITCHED")
                i = self.ids.index(self.active_id)
                if i == (len(self.ids) - 1): 
                    i = 0
                else:
                    i+=1
                self.config = initialize_config(self.pipeline, self.ids[i])
                self.pipeline.start(self.config)
                self.active_id = self.ids[i]
        except Exception as e:
            print(e)
            self.pipeline.start(oconfig)
            return -1

