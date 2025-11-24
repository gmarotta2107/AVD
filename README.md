# AVD - Autonomous Vehicle Driving

This project implements an advanced autonomous driving agent for the **CARLA Simulator**. The agent is designed to navigate complex urban scenarios, handling traffic, vulnerable road users, and adverse weather conditions with specific behavioral logic.

## 🚀 Key Features

The core of the project is the `BehaviorAgent`, which extends basic navigation capabilities with intelligent behaviors:

* **🛡️ Vulnerable User Protection:**
    * **Pedestrian Safety:** Detects and performs emergency braking for pedestrians crossing the street.
    * **Cyclist Awareness:** Specific logic to identify cyclists (detecting models like 'gazelle', 'ninja', etc.). It distinguishes between cyclists traveling in the same lane versus those crossing, adjusting lateral offset or braking accordingly.
* **🏎️ Overtaking Logic:**
    * Implements a decision-making algorithm to overtake slow vehicles or static obstacles.
    * Checks for safety by analyzing the speed and position of vehicles in the opposite/overtaking lane before initiating the maneuver.
* **🌧️ Weather Adaptation:**
    * The agent dynamically reduces its maximum speed based on the precipitation levels detected in the simulation environment.
* **🚦 Junction & Traffic Management:**
    * Respects traffic lights and STOP signs.
    * Handles intersections cautiously by monitoring incoming vehicles from different directions.
* **🚧 Static Obstacle Avoidance:**
    * Uses a `PerceptionModule` to detect and navigate around static obstacles on the road.

## 💻 Web Dashboard

The project includes a **Flask-based web server** for real-time debugging and visualization.

* **URL:** `http://localhost:9821`
* **Features:**
    * **Live Streaming:** RGB Camera, Depth Map, and Bird's Eye View (BEV).
    * **Telemetry:** Real-time display of Throttle, Steer, and Brake controls.

## 📂 Project Structure

* `behavior_agent.py`: Main logic for the autonomous agent (state machine, decision making).
* `basic_agent.py`: Base class handling local planning and control.
* `perception_module/`: Handles sensory data processing and obstacle detection.
* `server_http.py`: Web server for visualization.
* `run_test.sh`: Script to launch the simulation.

## 🛠️ Requirements

* **CARLA Simulator** (Compatible with API 0.9.x)
* Python 3.7+
* Dependencies: `carla`, `numpy`, `flask`, `opencv-python`, `shapely`

## ▶️ Usage

1.  **Start CARLA:** Ensure the CARLA simulator server is running.
2.  **Run the Agent:** Execute the main script to start the autonomous agent.
    ```bash
    ./run_test.sh
    ```
3.  **Monitor:** Open your web browser to view the agent's perspective.
    ```
    http://localhost:9821
    ```
4.  **Clean Up:** To kill all related processes after stopping:
    ```bash
    ./elimina_processi.sh
    ```

## 📝 Agent Modes

The agent can be initialized with different behavior profiles:
* **Cautious:** Prioritizes safety, stops more frequently, and maintains larger distances.
* **Normal:** Balanced driving style.
* **Aggressive:** Higher speeds and closer following distances (use with caution).

---
*Developed for the AVD Course - Group 8*
