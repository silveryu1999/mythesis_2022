from sanic import Sanic
import asyncio
import time
import json
import csv
import numpy as np

app = Sanic("server")

async def handler(request, ws):
    while True:
        str_json = await ws.recv()
        
        total_start_time = time.time()

        data = json.loads(str_json)
        str_encode, frame_id, client_detector_send_time, client_networker_send_time, client_name, network_delay, bandwidth = data['frame'], data['frame_id'], data['client_detector_send_time'], data['client_networker_send_time'], data['client_name'], data['network_delay'], data['bandwidth']
        c_to_s_time = time.time() - client_networker_send_time
        
        # update network delay
        network_delay = network_delay + c_to_s_time
        
        response_boxes = []
        
        # adding a process delay
        time.sleep(0.02)
        
        # get result
        with open("/home/silveryu1999/ground_truth/433/" + str(frame_id) + '.txt', encoding="utf-8") as cf:
        	lines = csv.reader(cf, delimiter=",")
        	for line in lines:
        		response_boxes.append(
        			{
        				'x1': int(line[1]),
                    			'y1': int(line[2]),
                    			'x2': int(line[3]),
                    			'y2': int(line[4]),
                    			'conf': float(line[5]),
                    			'name': line[0]
        			}
        		)
        
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
