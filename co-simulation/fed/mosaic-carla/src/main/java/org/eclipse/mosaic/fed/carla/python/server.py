from xmlrpc.server import SimpleXMLRPCServer
from xmlrpc.server import SimpleXMLRPCRequestHandler
import json

# Restrict to a particular path.
class RequestHandler(SimpleXMLRPCRequestHandler):
    rpc_paths = ('/RPC2',)

# Create server
with SimpleXMLRPCServer(('localhost', 8090),
                        requestHandler=RequestHandler) as server:
    server.register_introspection_functions()

    # Register a function under a different name
    def echo(text):
        print(text)       
      
        json_strings = [
        "{\"carla_actor\": null, \"id\": 0, \"object_type\": \"Vehicles\", \"timestamp\": 0, \"bounding_box_in_world_coordinate_frame\": [], \"position\": [20.0, 34.64101791381836, 0.0], \"velocity\": [100.0, 1.0, 0.0], \"rotation\": [[0.9983314336437902, -0.02072298021475338, 0.05389718628791056], [0.02439869428620028, 0.9973554635114954, -0.06846008414323508], [-0.05233595624294384, 0.06966087492121549, 0.9961969233988568]], \"angular_velocity\": [0.0, 0.0, 8.726646064915904e-05], \"position_covariance\": [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]], \"velocity_covariance\": [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]], \"confidence\": 1.0}",
        "{\"carla_actor\": null, \"id\": 1, \"object_type\": \"Pedestrians\", \"timestamp\": 0, \"bounding_box_in_world_coordinate_frame\": [], \"position\": [20.0, 34.64101791381836, 0.0], \"velocity\": [100.0, 1.0, 0.0], \"rotation\": [[0.9983314336437902, -0.02072298021475338, 0.05389718628791056], [0.02439869428620028, 0.9973554635114954, -0.06846008414323508], [-0.05233595624294384, 0.06966087492121549, 0.9961969233988568]], \"angular_velocity\": [0.0, 0.0, 8.726646064915904e-05], \"position_covariance\": [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]], \"velocity_covariance\": [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]], \"confidence\": 1.0}",
        "{\"carla_actor\": null, \"id\": 2, \"object_type\": \"Pedestrians\", \"timestamp\": 0, \"bounding_box_in_world_coordinate_frame\": [], \"position\": [20.0, 34.64101791381836, 0.0], \"velocity\": [100.0, 1.0, 0.0], \"rotation\": [[0.9983314336437902, -0.02072298021475338, 0.05389718628791056], [0.02439869428620028, 0.9973554635114954, -0.06846008414323508], [-0.05233595624294384, 0.06966087492121549, 0.9961969233988568]], \"angular_velocity\": [0.0, 0.0, 8.726646064915904e-05], \"position_covariance\": [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]], \"velocity_covariance\": [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]], \"confidence\": 1.0}",
        "{\"carla_actor\": null, \"id\": 3, \"object_type\": \"Pedestrians\", \"timestamp\": 0, \"bounding_box_in_world_coordinate_frame\": [], \"position\": [20.0, 34.64101791381836, 0.0], \"velocity\": [100.0, 1.0, 0.0], \"rotation\": [[0.9983314336437902, -0.02072298021475338, 0.05389718628791056], [0.02439869428620028, 0.9973554635114954, -0.06846008414323508], [-0.05233595624294384, 0.06966087492121549, 0.9961969233988568]], \"angular_velocity\": [0.0, 0.0, 8.726646064915904e-05], \"position_covariance\": [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]], \"velocity_covariance\": [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]], \"confidence\": 1.0}",
        "{\"carla_actor\": null, \"id\": 4, \"object_type\": \"Vehicles\", \"timestamp\": 0, \"bounding_box_in_world_coordinate_frame\": [], \"position\": [20.0, 34.64101791381836, 0.0], \"velocity\": [100.0, 1.0, 0.0], \"rotation\": [[0.9983314336437902, -0.02072298021475338, 0.05389718628791056], [0.02439869428620028, 0.9973554635114954, -0.06846008414323508], [-0.05233595624294384, 0.06966087492121549, 0.9961969233988568]], \"angular_velocity\": [0.0, 0.0, 8.726646064915904e-05], \"position_covariance\": [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]], \"velocity_covariance\": [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]], \"confidence\": 1.0}",
        "{\"carla_actor\": null, \"id\": 5, \"object_type\": \"Vehicles\", \"timestamp\": 0, \"bounding_box_in_world_coordinate_frame\": [], \"position\": [20.0, 34.64101791381836, 0.0], \"velocity\": [100.0, 1.0, 0.0], \"rotation\": [[0.9983314336437902, -0.02072298021475338, 0.05389718628791056], [0.02439869428620028, 0.9973554635114954, -0.06846008414323508], [-0.05233595624294384, 0.06966087492121549, 0.9961969233988568]], \"angular_velocity\": [0.0, 0.0, 8.726646064915904e-05], \"position_covariance\": [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]], \"velocity_covariance\": [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]], \"confidence\": 1.0}"
        ]

        json_array = [json.loads(json_str) for json_str in json_strings]
        json_array_string = json.dumps(json_array, indent=2)
        

        return json_array_string
   
    server.register_function(echo, 'test.echo')

    # Run the server's main loop
    
    server.serve_forever()