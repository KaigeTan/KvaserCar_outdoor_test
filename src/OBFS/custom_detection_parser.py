
import cv2
import numpy as np
from ultralytics import YOLO
import pyzed.sl as sl
import json
import os
#from yolo_training.plot_data import plot as plot_graph
from plot_data import plot as plot_graph
from client import OBFS_send

class CustomDetParse:

    SVO_SIZE_720 = 0
    SVO_SIZE_1080 = 1

    def __init__(self, 
                 yolo_model_path,
                 path_to_svo, 
                 plot_file_name,
                 plot_file_path,
                 show_video=True, 
                 save_video=False, 
                 svo_size=SVO_SIZE_720,  
                 svo_fps=30,
                 video_out_path=None, 
                 out_name=None):
        
        # Enable positional tracking module
        self.pos_tracking_param = sl.PositionalTrackingParameters()
        # If the camera is static in space, enabling this setting below provides better depth quality and faster computation
        self.pos_tracking_param.set_as_static = True

        # Set initialization parameters
        self.det_param = sl.ObjectDetectionParameters()
        self.det_param.enable_tracking = True # Objects will keep the same ID between frames
        self.det_param.enable_segmentation = False # Outputs 2D masks over detected objects
        self.det_param.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS 

        # Set runtime parameters
        self.det_param_rt = sl.ObjectDetectionRuntimeParameters()
        self.det_param_rt.detection_confidence_threshold = 60

        # Create a InitParameters object and set configuration parameters
        self.init_params = sl.InitParameters()
        self.init_params.coordinate_units = sl.UNIT.METER
        self.init_params.sdk_verbose = 0
        self.init_params.set_from_svo_file(path_to_svo)
        self.init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # QUALITY
        self.init_params.depth_maximum_distance = 50

        self.yolo_model_path = yolo_model_path
        self.plot_file_name = plot_file_name

        self.data_out = dict()
        self.data_out["t"] = list()
        self.data_out["t_acq"] = list()
        self.data_out["p_x"] = list()
        self.data_out["p_y"] = list()
        self.data_out["p_z"] = list()
        self.data_out["v_x"] = list()
        self.data_out["v_y"] = list()
        self.data_out["v_z"] = list()
        
        self.plot_path = os.path.join(plot_file_path, plot_file_name)     
        self.is_open = False

        if save_video:
            #openCV video
            self.video_path = os.path.join(video_out_path, out_name)
            if svo_size == CustomDetParse.SVO_SIZE_720:
                self.out = cv2.VideoWriter(self.video_path, cv2.VideoWriter_fourcc(*"mjpg"), svo_fps, (1280, 720))
            else:
                self.out = cv2.VideoWriter(self.video_path, cv2.VideoWriter_fourcc(*"mjpg"), svo_fps, (1920, 1080))
        else:
            self.out = None
        
        self.client=OBFS_send("127.0.0.1", 5005)
        

    def open(self):
        self.is_open = True
        self.model = YOLO(self.yolo_model_path)
        self.zed = sl.Camera()

        # Open the ZED
        err = self.zed.open(self.init_params)
        err = self.zed.enable_positional_tracking(self.pos_tracking_param)
        err = self.zed.enable_object_detection(self.det_param)

        return err


    def _log(self, time_step, pos, vel):
        self.data_out["t_acq"].append(time_step)
        self.data_out["p_x"].append(pos[0])
        self.data_out["p_y"].append(pos[1])
        self.data_out["p_z"].append(pos[2])
        self.data_out["v_x"].append(vel[0])
        self.data_out["v_y"].append(vel[1])
        self.data_out["v_z"].append(vel[2])

        msg= str(time_step)+","+str(pos[0])+","+str(pos[1])+","+ str(pos[2])+","+str(vel[0])+","+str(vel[1])+","+str(vel[2])
        return msg


    def _write_log(self):

        with open(self.plot_file_name, 'w') as f:
            json.dump(self.data_out, f)


    def _xywh2abcd(self, xywh):
        """
        Convert YOLO boxes to ZED boxes
        # A ------ B
        # | Object |
        # D ------ C
        """
        output = np.zeros((4, 2))

        # Center / Width / Height -> BBox corners coordinates
        x_min = (xywh[0] - 0.5*xywh[2]) 
        x_max = (xywh[0] + 0.5*xywh[2])
        y_min = (xywh[1] - 0.5*xywh[3])
        y_max = (xywh[1] + 0.5*xywh[3])

        # A ------ B
        # | Object |
        # D ------ C

        output[0][0] = x_min
        output[0][1] = y_min

        output[1][0] = x_max
        output[1][1] = y_min

        output[2][0] = x_max
        output[2][1] = y_max

        output[3][0] = x_min
        output[3][1] = y_max
        return output


    def _detections_to_custom_box(self, results):
        output = []
        # we process one image at a time hence the [0]
        if results[0].boxes is not None:
            plot = results[0].plot()
            clss = results[0].boxes.cls.cpu().tolist()
            conf = results[0].boxes.conf.cpu().tolist()
            boxes = results[0].boxes.xywh
            for box, cls, conf in zip(boxes, clss, conf):
                obj = sl.CustomBoxObjectData()
                obj.bounding_box_2d = self._xywh2abcd(box)
                obj.label = cls
                obj.probability = conf
                obj.is_grounded = True
                output.append(obj)

        return output, plot
    
    def run(self):
        if self.is_open is False:
            self.open()

        svo_image = sl.Mat()
        objects = sl.Objects()

        while True:
            err = self.zed.grab()
            
            if err == sl.ERROR_CODE.SUCCESS:
                # Read side by side frames stored in the SVO
                self.zed.retrieve_image(svo_image, sl.VIEW.LEFT)
                img = svo_image.get_data()
                timestamp = self.zed.get_timestamp(sl.TIME_REFERENCE.IMAGE)
                self.data_out["t"].append(timestamp.get_microseconds())

                img_conver = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
                results = self.model.predict(img_conver)
                detections, img_plot = self._detections_to_custom_box(results)            
                self.zed.ingest_custom_box_objects(detections)
                self.zed.retrieve_objects(objects, self.det_param_rt)


                msg=""
                for ob in objects.object_list:
                    msg=self._log(timestamp.get_milliseconds(), ob.position, ob.velocity)                          
                    self.client.sendMessage(msg)
                # cv2.imshow("instance-segmentation", img_plot)                
                # if self.out is not None:
                #     c = cv2.cvtColor(img_plot, cv2.COLOR_BGRA2BGR)
                #     self.out.write(c)
                # if cv2.waitKey(1) & 0xFF == ord("q"):
                #     break
                
            elif err == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
                print("SVO end has been reached. Looping back to first frame")
                self.zed.set_svo_position(0)
                break

        cv2.destroyAllWindows()
        if self.out is not None:
            self.out.release()

        self.zed.close()
        self._write_log()


    def plot(self):
        plot_graph(self.plot_file_name, title=self.plot_file_name)

  

