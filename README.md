# ros-did-fabric-secure-communication
Secure ROS 2 DDS communication using DIDs and Hyperledger Fabric
| Component             | Purpose                                 |
| --------------------- | --------------------------------------- |
| **fabric-gateway**    | Simulates blockchain DID verification   |
| **identity_agent.py** | Device-side identity verifier           |
| **ros_publisher.py**  | Secure ROS2 publisher                   |
| **ros_subscriber.py** | Secure ROS2 subscriber                  |
| **Docker Compose**    | Orchestrates startup order & networking |
-------------------------------------------------------------------------------------------
🔹 Step 1: Docker Compose starts the system

Command you ran:

docker compose up


What happens internally:

Docker creates an isolated network (device_did-net)

Starts fabric-gateway first

ROS nodes wait until gateway becomes healthy

✔️ This ensures identity service is ready before ROS nodes start
---------------------------------------------------------------------------

🔹 Step 2: Fabric DID Gateway starts

From logs:

Fabric DID Gateway running on port 4000


📄 server.js behavior (conceptual):

Maintains a list of valid DIDs

Exposes /verify API

Acts as a mock blockchain ledger

When ROS nodes request verification:

🔍 Verifying DID: did:device:publisher001
🔍 Verifying DID: did:device:subscriber001


✔️ This simulates on-chain DID validation
-------------------------------------------------------------------------------------------------

🔹 Step 3: Identity Agent runs inside each ROS container

Before ROS starts, this happens:

subprocess.call(["python3", "identity_agent.py", DID])


📄 identity_agent.py role:

Sends DID to Fabric Gateway

Receives verification response

Exits non-zero if invalid

✔️ If DID is invalid → ROS node never starts
✔️ This is Zero Trust enforcement
-----------------------------------------------------------------------------------------
🔹 Step 4: ROS nodes start ONLY after verification

From logs:

DEVICE VERIFIED


This proves:

❗ ROS2 nodes are identity-gated, not just network-connected

4️⃣ ROS Publisher execution explained
📄 ros_publisher.py

Key logic:

MAX_MESSAGES = 10


Publisher behavior:

Creates ROS2 node: secure_publisher

Publishes message every 2 seconds

Stops after 10 messages

Gracefully shuts down ROS

Logs:

Hello from VERIFIED publisher #1
...
Hello from VERIFIED publisher #10
Publisher finished. Shutting down.

------------------------------------------------------------
ROS2 + DID + Docker + Fabric Gateway
1. Project Objective
This project demonstrates a secure ROS2 communication system where devices (publisher and subscriber) are authenticated using Decentralized Identifiers (DIDs). Only verified devices are allowed to participate in ROS2 message exchange. A Fabric-like gateway simulates blockchain-based DID verification.
2. System Architecture
The system consists of three Docker containers:
1) Fabric DID Gateway – verifies device DIDs
2) ROS2 Publisher – sends messages after verification
3) ROS2 Subscriber – receives messages after verification

All containers communicate over a Docker bridge network.
3. File-by-File Explanation
Dockerfile:
- Builds a ROS2-ready Ubuntu image
- Installs Python, ROS2, and required dependencies
- Copies application files and sets entrypoint

docker-compose.yml:
- Defines three services (gateway, publisher, subscriber)
- Ensures publisher/subscriber start only after gateway is healthy

server.js (Fabric Gateway):
- Express server running on port 4000
- Verifies DIDs using a mock blockchain registry

identity_agent.py:
- Sends HTTP request to gateway for DID verification
- Exits with success/failure code

ros_publisher.py:
- Verifies DID before starting
- Publishes 10 messages on secure_topic
- Gracefully shuts down

ros_subscriber.py:
- Verifies DID before starting
- Subscribes to secure_topic
- Stops after receiving 10 messages
4. Execution Flow
Step 1: Docker Compose builds images
Step 2: Fabric gateway starts and becomes healthy
Step 3: Publisher & Subscriber verify their DIDs
Step 4: Verified publisher sends messages
Step 5: Subscriber receives messages
Step 6: Both nodes shut down cleanly
5. Commands to Run the Project
Navigate to project directory:
cd ros-did-fabric/fabric-samples/chaincode/device-did/device

Stop existing containers:
docker compose down -v

Build images:
docker compose build --no-cache

Start services:
docker compose up

View logs:
docker compose logs -f ros-publisher
docker compose logs -f ros-subscriber
docker compose logs -f fabric-gateway
6. Final Outcome
You successfully implemented:
- Decentralized identity verification
- Secure ROS2 inter-node communication
- Dockerized microservice architecture
- Controlled node lifecycle and clean shutdown

This project forms a strong foundation for blockchain-backed robotic systems and zero-trust IoT architectures.
