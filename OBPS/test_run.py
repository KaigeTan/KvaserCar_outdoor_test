import os
import numpy as np
from custom_detection_parser import CustomDetParse

class VidInfo:
    def __init__(self, path_to_svo, pl_path, video_out_path):
        name = path_to_svo.split(".")[0]
        self.path_to_svo = path_to_svo
        self.pl_file_name = name + ".json"
        self.pl_path = pl_path
        self.video_out_path = video_out_path
        self.video_name = name + ".mp4"

def parse_video(info: VidInfo):
    model_path="/home/tecosa/Documents/zed_scripts/yolo_model/best.pt"
    parser = CustomDetParse(model_path,
                            info.path_to_svo,
                            info.pl_file_name,
                            info.pl_path,
                            save_video=True,
                            video_out_path=video_out_path,
                            out_name=info.video_name)
    
    parser.run()
    #parser.plot()


if __name__ == '__main__':
    svo_path = "/home/tecosa/Documents/zed_scripts/"
    pl_path = "/home/tecosa/Documents/zed_scripts/plots"
    video_out_path = "/home/tecosa/Documents/zed_scripts/videos"

    vinfo1 = VidInfo(os.path.join(svo_path, "HD720_SN37944668_18-13-18.svo2"),
                     pl_path,
                     video_out_path
                     )
    
    vinfo2 = VidInfo(os.path.join(svo_path, "HD720_SN37944668_18-12-34.svo2"),
                    pl_path,
                    video_out_path
                    )

    vinfo3 = VidInfo(os.path.join(svo_path, "HD720_SN37944668_18-02-04.svo2"),
                    pl_path,
                    video_out_path
                    )

    vinfo4 = VidInfo(os.path.join(svo_path, "HD720_SN37944668_18-04-43.svo2"),
                    pl_path,
                    video_out_path
                    )

    vinfo5 = VidInfo(os.path.join(svo_path, "HD720_SN37944668_18-08-47.svo2"),
                    pl_path,
                    video_out_path
                    )


    #parse_video(vinfo1)
    #parse_video(vinfo2)
    parse_video(vinfo3)
    #parse_video(vinfo4)
    #parse_video(vinfo5)