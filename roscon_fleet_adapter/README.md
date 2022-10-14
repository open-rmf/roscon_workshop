# roscon_fleet_adapter

Fleet adapter for the roscon workshop, based on the EasyFullControl API for simplicity of integration

## API Endpoints

This fleet adapter integration relies on a fleet manager and a fleet adapter:
- The **fleet manager** comprises of specific endpoints that help relay commands to the fleet's robots. It communicates with the robots over internal ROS 2 messages, while interfacing with the adapter via an API chosen by the user. For this demo fleet adapter implementation, we are using REST API with FastAPI framework.
- The **fleet adapter** receives commands from RMF and interfaces with the fleet manager to receive robot state information, as well as send task and navigation commands to the robots

To interact with endpoints, launch the demo and then visit http://127.0.0.1:22011/docs in your browser.

### 1. Get Robot Status
The `status` endpoint allows the fleet adapter to access robot state information such as its current position and battery level. This endpoint does not require a Request Body.

There are two ways to request the fleet robot status:

#### a. Get status of all robots in the fleet
Request URL: `http://127.0.0.1:22011/open-rmf/rmf_demos_fm/status/`
##### Response Body:
```json
{
  "data": {
    "all_robots": [
      {
        "robot_name": "tinyRobot1",
        "map_name": "L1",
        "position": {
          "x": 10.0,
          "y": 20.0,
          "yaw": 1.0
        },
        "battery": 100,
        "last_completed_request": 2,
        "destination_arrival": {
          "cmd_id": 3,
          "duration": 14.3
        }
      },
      {
        "robot_name": "tinyRobot2",
        "map_name": "L1",
        "position": {
          "x": 5.0,
          "y": 25.0,
          "yaw": 1.4
        },
        "battery": 100,
        "last_completed_request": 3,
        "destination_arrival": null,
        "replan": true
      }
    ]
  },
  "success": true,
  "msg": ""
}
```

#### b. Get status of specified robot in the fleet
Append a `robot_name` query parameter to the end of the URL.

Request URL: `http://127.0.0.1:22011/open-rmf/rmf_demos_fm/status/?robot_name=tinyRobot1`
##### Response Body:
```json
{
  "data": {
    "robot_name": "tinyRobot1",
    "map_name": "L1",
    "position": {
      "x": 10.0,
      "y": 20.0,
      "yaw": 1.0
    },
    "battery": 100,
    "last_completed_request": 2,
    "destination_arrival": {
      "cmd_id": 3,
      "duration": 14.3
    }
  },
  "success": true,
  "msg": ""
}
```

### 2. Send Navigation Request
The `navigate` endpoint allows the fleet adapter to send navigation waypoints to a specified robot. This endpoint requires a Request Body and a `robot_name` query parameter.

Request URL: `http://127.0.0.1:22011/open-rmf/rmf_demos_fm/navigate/?robot_name=tinyRobot1`
##### Request Body:
```json
{
  "map_name": "L1",
  "destination": {
    "x": 7.0,
    "y": 3.5,
    "yaw": 0.5
  },
  "speed_limit": 0.0
}
```

##### Response Body:
```json
{
  "success": true,
  "msg": ""
}
```

### 3. Stop Robot
The `stop` endpoint allows the fleet adapter to command a specified robot to stop. This endpoint only requires a `robot_name` query parameter.

Request URL: `http://127.0.0.1:22011/open-rmf/rmf_demos_fm/stop/?robot_name=tinyRobot1`
##### Response Body:
```json
{
  "success": true,
  "msg": ""
}
```

### 4. Send Task Request
The `start_task` endpoint allows the fleet adapter to send task requests to a specified robot. This endpoint requires a Request Body and a `robot_name` query parameter.

Request URL: `http://127.0.0.1:22011/open-rmf/rmf_demos_fm/start_task/?robot_name=tinyRobot1`
##### Request Body:
```json
{
  "map_name": "L1",
  "task": "clean_lobby"
}
```

##### Response Body:
```json
{
  "success": true,
  "msg": ""
}
```
