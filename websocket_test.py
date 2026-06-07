import asyncio
import websockets
import json
import random
from datetime import datetime
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("VehicleServer")

class VehicleDataGenerator:
    """Generates realistic vehicle data with some random variation"""
    
    def __init__(self):
        self.speed = 0
        self.rpm = 800  # Idle RPM
        self.in_lane = True
        self.gear = 1
        self.engine_temp = 90
        self.fuel_level = 80
        self.simulation_running = False
        
    def generate_data(self):
        """Generate new vehicle data with realistic patterns"""
        if not self.simulation_running:
            return None
            
        # Simulate acceleration/deceleration patterns
        if random.random() < 0.7:  # 70% chance to maintain or increase speed
            self.speed = min(self.speed + random.uniform(-1, 3), 120)
        else:  # 30% chance to brake
            self.speed = max(self.speed - random.uniform(2, 5), 0)
            
        # RPM calculation based on speed and gear
        self.gear = min(max(int(self.speed / 25) + 1, 1), 6)
        self.rpm = 800 + (self.speed * 30 / self.gear) + random.uniform(-50, 50)
        self.rpm = min(self.rpm, 7000)  # Rev limiter
        
        # Lane keeping - occasionally drift out of lane
        if random.random() < 0.05:  # 5% chance to change lane status
            self.in_lane = not self.in_lane
            
        # Engine temperature fluctuates slightly
        self.engine_temp += random.uniform(-0.5, 0.5)
        self.engine_temp = min(max(self.engine_temp, 85), 110)
        
        # Fuel slowly decreases
        self.fuel_level -= random.uniform(0.01, 0.05)
        self.fuel_level = max(self.fuel_level, 0)
        
        return {
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "speed": round(self.speed, 1),
            "rpm": int(self.rpm),
            "in_lane": self.in_lane,
            "gear": self.gear,
            "engine_temp": round(self.engine_temp, 1),
            "fuel_level": round(self.fuel_level, 1),
            "warning": self.check_warnings()
        }
        
    def check_warnings(self):
        """Generate warning messages based on vehicle state"""
        warnings = []
        if self.rpm > 6500:
            warnings.append("High RPM")
        if not self.in_lane:
            warnings.append("Lane Departure")
        if self.engine_temp > 105:
            warnings.append("Engine Hot")
        if self.fuel_level < 15:
            warnings.append("Low Fuel")
        return warnings if warnings else None
        
    def start_simulation(self):
        self.simulation_running = True
        logger.info("Vehicle simulation started")
        
    def stop_simulation(self):
        self.simulation_running = False
        logger.info("Vehicle simulation stopped")

async def vehicle_data_server(websocket):
    """Handle WebSocket connections and send vehicle data"""
    logger.info(f"New client connected: {websocket.remote_address}")
    vehicle = VehicleDataGenerator()
    vehicle.start_simulation()
    
    try:
        while True:
            data = vehicle.generate_data()
            if data:
                await websocket.send(json.dumps(data))
                logger.debug(f"Sent data: {data}")
            await asyncio.sleep(0.1)  # Send data 10 times per second
            
    except websockets.exceptions.ConnectionClosed:
        logger.info(f"Client disconnected: {websocket.remote_address}")
        vehicle.stop_simulation()
    except Exception as e:
        logger.error(f"Error: {e}")
        vehicle.stop_simulation()

async def main():
    """Start the WebSocket server"""
    port = 8765
    logger.info(f"Starting vehicle data server on ws://0.0.0.0:{port}")
    async with websockets.serve(vehicle_data_server, "0.0.0.0", port):
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    asyncio.run(main())