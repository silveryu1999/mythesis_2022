from sanic import Sanic
import asyncio
import time
import json
import base64
import numpy as np
import multiprocessing
import cv2 # OpenCV library
import torch
from yolov4 import *

app = Sanic("server")

@app.listener("before_server_start")
async def worker_start(app, loop):
	app.ctx.use_cuda = True
	use_tiny = False
	app.ctx.darknet, app.ctx.class_names = NewDarknet(use_tiny, app.ctx.use_cuda)

async def handler(request, ws):
    while True:
        str_json = await ws.recv()
        
        total_start_time = time.time()

        data = json.loads(str_json)
        str_encode, frame_id, client_detector_send_time, client_networker_send_time, client_name, network_delay, bandwidth = data['frame'], data['frame_id'], data['client_detector_send_time'], data['client_networker_send_time'], data['client_name'], data['network_delay'], data['bandwidth']
        c_to_s_time = time.time() - client_networker_send_time
        
        # await asyncio.sleep(network_delay)
        
        # process_start_time = time.time()
        
        # json to frame
        data_encode_64 = str_encode.encode('utf-8')
        data_encode = np.frombuffer(base64.b64decode(data_encode_64), np.uint8)
        current_frame = cv2.imdecode(data_encode, cv2.IMREAD_COLOR)

        # detect
        sized = cv2.resize(current_frame, (app.ctx.darknet.width, app.ctx.darknet.height))
        sized = cv2.cvtColor(sized, cv2.COLOR_BGR2RGB)

        boxes = do_detect(app.ctx.darknet, sized, 0.4, 0.6, app.ctx.use_cuda)
        boxes_zero = np.array(boxes[0]).tolist()

        width = current_frame.shape[1]
        height = current_frame.shape[0]

        torch.cuda.empty_cache()

        response_boxes = []
        for i in range(len(boxes_zero)):
            response_boxes.append(
                {
                    'x1': int(boxes_zero[i][0] * width),
                    'y1': int(boxes_zero[i][1] * height),
                    'x2': int(boxes_zero[i][2] * width),
                    'y2': int(boxes_zero[i][3] * height),
                    'conf': boxes_zero[i][5],
                    'name': app.ctx.class_names[int(boxes_zero[i][6])]
                }
            )
        
        # process_time = time.time() - process_start_time
        response = {
            'frame_id': frame_id,
            'boxes': response_boxes,
            'c_to_s_time': c_to_s_time,
            'client_detector_send_time': client_detector_send_time,
            'server_send_time': time.time(),
            'network_delay': network_delay,
            'server_name': "server",
            'bandwidth': bandwidth,
            'process_time': time.time() - total_start_time
        }
        
        print("Frame %d from %s has been processed. C to S time: %fs | Process Time: %fs" % (frame_id, client_name, c_to_s_time, time.time() - total_start_time))

        await ws.send(json.dumps(response))

app.add_websocket_route(handler, "/")

if __name__ == '__main__':
    app.run(host="0.0.0.0", port=12345)
