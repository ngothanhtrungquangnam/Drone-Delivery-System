import asyncio
import math
import numpy as np
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw, VelocityNedYaw)

class DroneDeliveryOptimized:
    def __init__(self):
        self.drone = System()
        
        # Vị trí hiện tại
        self.current_position = np.array([0.0, 0.0, 0.0])
        
        # Cấu hình bay
        self.flight_altitude = -5.0  # Bay cao 5m (cao hơn vật cản cao nhất là 4m)
        self.cruise_speed = 3.0      # Tốc độ bay 3 m/s
        self.safe_distance = 3.5     # Khoảng cách an toàn với vật cản
        
        # --- QUAN TRỌNG: TỌA ĐỘ ĐÍCH ---
        # Phải khớp với model "delivery_zone" trong file obstacle_world.world
        # Tọa độ trong world là: x=20, y=25
        self.delivery_point = np.array([20.0, 25.0, self.flight_altitude])
        
        # --- DANH SÁCH VẬT CẢN (Khớp với file World) ---
        self.obstacles = [
            {"pos": np.array([5.0, 5.0]), "radius": 2.5, "height": 3.0, "name": "wall_1 (Red)"},
            {"pos": np.array([8.0, 10.0]), "radius": 1.5, "height": 4.0, "name": "pole_1 (Green)"},
            {"pos": np.array([12.0, 15.0]), "radius": 2.5, "height": 2.5, "name": "box_1 (Blue)"},
            {"pos": np.array([15.0, 20.0]), "radius": 3.5, "height": 3.0, "name": "wall_2 (Orange)"},
        ]
        
        print(f"🎯 Mục tiêu: ({self.delivery_point[0]}, {self.delivery_point[1]})")

    # --- THUẬT TOÁN TRÁNH VẬT CẢN (POTENTIAL FIELD) ---
    def detect_nearby_obstacles(self):
        nearby = []
        max_range = 10.0  # Tầm quét 10m
        
        for obs in self.obstacles:
            obs_pos_2d = obs["pos"]
            drone_pos_2d = self.current_position[:2]
            
            # Chỉ tính khoảng cách
            delta = obs_pos_2d - drone_pos_2d
            distance = np.linalg.norm(delta)
            
            # Nếu nằm trong bán kính 10m là báo động luôn (Bất kể hướng nào)
            if distance < max_range:
                 nearby.append({"pos": obs_pos_2d, "distance": distance, "radius": obs["radius"], "name": obs["name"]})
        
        return nearby
    def calculate_velocity_command(self, target_position, nearby_obstacles):
        # 1. Lực hút về mục tiêu (Attractive)
        to_target = target_position - self.current_position
        dist_target = np.linalg.norm(to_target[:2])
        
        if dist_target < 0.5: return np.array([0.0, 0.0, 0.0]) # Đã đến nơi
        
        attractive = (to_target / np.linalg.norm(to_target)) * self.cruise_speed
        
        # 2. Lực đẩy từ vật cản (Repulsive)
        repulsive = np.array([0.0, 0.0, 0.0])
        if len(nearby_obstacles) > 0:
            print(f"⚠️ Phát hiện {len(nearby_obstacles)} vật cản! Đang né...")
            for obs in nearby_obstacles:
                vec = self.current_position[:2] - obs["pos"]
                dist = np.linalg.norm(vec)
                safe_dist = self.safe_distance + obs["radius"]
                
                if dist < safe_dist and dist > 0.1:
                    mag = ((safe_dist - dist) / safe_dist) ** 2 * 4.0 # Lực đẩy mạnh
                    repulsive[:2] += (vec / dist) * mag

        # 3. Tổng hợp lực
        total_vel = attractive + repulsive
        
        # Giới hạn tốc độ tối đa
        speed = np.linalg.norm(total_vel[:2])
        if speed > self.cruise_speed * 1.5:
            total_vel[:2] = (total_vel[:2] / speed) * (self.cruise_speed * 1.5)
            
        # Giữ độ cao
        total_vel[2] = np.clip(target_position[2] - self.current_position[2], -1.0, 1.0)
        
        return total_vel

    # --- CÁC HÀM HỆ THỐNG (CONNECT, SETUP, ARM) ---
    async def connect(self):
        print("🔌 Đang kết nối tới Drone (UDP 14540)...")
        await self.drone.connect(system_address="udp://:14540")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("✅ Drone đã kết nối!")
                break

    async def setup_telemetry(self):
        # Cập nhật vị trí liên tục
        async def update_pos():
            async for pos in self.drone.telemetry.position_velocity_ned():
                self.current_position = np.array([pos.position.north_m, pos.position.east_m, pos.position.down_m])
        asyncio.create_task(update_pos())

    async def wait_for_health(self):
        print("🏥 Đang kiểm tra sức khỏe hệ thống (Chờ GPS & Sensors)...")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                # Quan trọng: Chờ cho đến khi Drone cho phép Arm
                if health.is_armable:
                    print("✅ Hệ thống OK - Sẵn sàng cất cánh!")
                    break
            await asyncio.sleep(1)

    async def arm_and_takeoff(self):
        print("🔧 Đang Arming...")
        for i in range(5): # Thử 5 lần
            try:
                await self.drone.action.arm()
                print("✅ Armed thành công!")
                break
            except Exception as e:
                print(f"⚠️ Thử lại arming ({i+1}/5)...")
                await asyncio.sleep(2)
        
        print(f"🚁 Cất cánh lên {abs(self.flight_altitude)}m...")
        await self.drone.action.set_takeoff_altitude(abs(self.flight_altitude))
        await self.drone.action.takeoff()
        await asyncio.sleep(10) # Đợi bay lên ổn định

    # --- NHIỆM VỤ CHÍNH ---
    async def run_mission(self):
        await self.connect()
        await self.setup_telemetry()
        await self.wait_for_health()
        await self.arm_and_takeoff()

        # TASK CHẠY NGẦM: Giữ kết nối Offboard
        async def heartbeat():
            while True:
                # Gửi setpoint rỗng để PX4 biết script còn sống
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0,0,0,0))
                await asyncio.sleep(0.1)
        
        heartbeat_task = asyncio.create_task(heartbeat())
        
        print("🎮 Bắt đầu chế độ Offboard (Né vật cản)...")
        
        # GỬI SETPOINT TRƯỚC (quan trọng!)
        print("   Đang gửi setpoint ban đầu...")
        for _ in range(10):  # Gửi 10 lần để đảm bảo PX4 nhận được
            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))
            await asyncio.sleep(0.1)
        
        # Bây giờ mới bật offboard
        try:
            await self.drone.offboard.start()
            print("✅ Offboard mode đã kích hoạt!")
        except OffboardError as e:
            print(f"❌ Lỗi Offboard: {e}")
            await self.drone.action.land()
            return

        # VÒNG LẶP ĐIỀU KHIỂN
        while True:
            dist = np.linalg.norm(self.current_position[:2] - self.delivery_point[:2])
            
            if dist < 1.0: # Đến đích
                print("✅ ĐÃ ĐẾN VÙNG GIAO HÀNG (DELIVERY ZONE)!")
                break
            
            nearby = self.detect_nearby_obstacles()
            vel = self.calculate_velocity_command(self.delivery_point, nearby)
            
            # Gửi lệnh vận tốc
            await self.drone.offboard.set_velocity_ned(
                VelocityNedYaw(vel[0], vel[1], vel[2], 0.0)
            )
            await asyncio.sleep(0.1) # Tần số cập nhật 10Hz

        # Dừng tại chỗ
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0,0,0,0))
        
        # Giao hàng (Hạ thấp)
        print("📦 Đang hạ thấp để giao hàng...")
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(self.current_position[0], self.current_position[1], -0.5, 0.0)
        )
        await asyncio.sleep(5)
        print("✅ Giao hàng xong!")

        # Quay về nhà
        print("🔙 Đang quay về nhà (0,0)...")
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, -5.0, 0.0)
        )
        # Chờ về đến nơi (đơn giản hóa bằng sleep, thực tế nên check distance)
        await asyncio.sleep(15) 
        
        # Hạ cánh
        print("🛬 Đang hạ cánh...")
        heartbeat_task.cancel() # Dừng tim
        try:
            await self.drone.offboard.stop()
        except: pass
        
        await self.drone.action.land()
        print("🏁 NHIỆM VỤ HOÀN THÀNH!")

if __name__ == "__main__":
    asyncio.run(DroneDeliveryOptimized().run_mission())