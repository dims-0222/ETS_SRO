
import time
import numpy as np
import open3d as o3d

from pynput import keyboard
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


# =========================================================
# 1. KONFIGURASI
# =========================================================

HOST = "localhost"
PORT = 23000

POINT_SIGNAL = "laserPointCloud"
SEQUENCE_SIGNAL = "laserScanSeq"

SPEED = 0.5
STEP_TIME = 0.02
ROUTE_INTERVAL = 0.1
MAP_UPDATE_INTERVAL = 0.15

VOXEL_SIZE = 0.035
MAX_RAW_POINTS = 250_000
MAX_ROUTE_POINTS = 5000


# =========================================================
# 2. KONEKSI COPPELIASIM
# =========================================================

client = RemoteAPIClient(host=HOST, port=PORT)
sim = client.getObject("sim")

drone = sim.getObject("/Quadcopter")
target = sim.getObject("/target")

print("Terhubung ke CoppeliaSim.")
print("Drone:", drone)
print("Target:", target)


# =========================================================
# 3. KEYBOARD
# =========================================================

pressed = set()
running = True


def on_press(key):
    global running

    if key == keyboard.Key.esc:
        running = False
        return False

    try:
        pressed.add(key.char.lower())
    except (AttributeError, TypeError):
        pass


def on_release(key):
    try:
        pressed.discard(key.char.lower())
    except (AttributeError, TypeError):
        pass


listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release
)
listener.start()


# =========================================================
# 4. VISUALISASI OPEN3D
# =========================================================

visualizer = o3d.visualization.Visualizer()

if not visualizer.create_window(
    window_name="Quadcopter - 3D Mapping + Trajectory",
    width=1100,
    height=750
):
    listener.stop()
    raise RuntimeError("Gagal membuat jendela Open3D.")

# Point cloud lingkungan
map_geometry = o3d.geometry.PointCloud()
visualizer.add_geometry(map_geometry)

# Garis rute drone
route_geometry = o3d.geometry.LineSet()
visualizer.add_geometry(route_geometry)

# Titik posisi drone saat ini
drone_marker = o3d.geometry.TriangleMesh.create_sphere(
    radius=0.10
)
drone_marker.paint_uniform_color([1.0, 0.1, 0.1])
visualizer.add_geometry(drone_marker)

# Sumbu koordinat dunia
axes = o3d.geometry.TriangleMesh.create_coordinate_frame(
    size=0.5
)
visualizer.add_geometry(axes)

render = visualizer.get_render_option()
render.background_color = np.array([0.05, 0.05, 0.05])
render.point_size = 2.0

view = visualizer.get_view_control()

all_points = np.empty((0, 3), dtype=np.float64)
route_points = []

last_sequence = None
last_route_time = 0.0
last_map_update = 0.0
camera_initialized = False

print("\nKontrol:")
print("W/S : maju/mundur pada sumbu X dunia")
print("A/D : kiri/kanan pada sumbu Y dunia")
print("R/F : naik/turun")
print("ESC : berhenti")


# =========================================================
# 5. MEMBACA POINT CLOUD DARI LUA
# =========================================================

def read_lidar_points():
    raw = sim.getStringSignal(POINT_SIGNAL)

    if raw is None:
        return None

    if isinstance(raw, bytes):
        raw = raw.decode("utf-8", errors="ignore")

    points = []

    for line in raw.strip().splitlines():
        values = line.strip().split(",")

        if len(values) != 3:
            continue

        try:
            xyz = [float(value) for value in values]

            if np.isfinite(xyz).all():
                points.append(xyz)

        except ValueError:
            continue

    if not points:
        return None

    return np.asarray(points, dtype=np.float64)


# =========================================================
# 6. UPDATE RUTE DRONE
# =========================================================

def update_route():
    global route_points

    position = sim.getObjectPosition(
        drone, sim.handle_world
    )

    current_position = np.asarray(
        position, dtype=np.float64
    )

    # Abaikan posisi berulang yang terlalu berdekatan
    if route_points:
        distance = np.linalg.norm(
            current_position - route_points[-1]
        )

        if distance < 0.02:
            return

    route_points.append(current_position)

    if len(route_points) > MAX_ROUTE_POINTS:
        route_points = route_points[-MAX_ROUTE_POINTS:]

    # Perbarui garis yang menghubungkan posisi aktual drone
    points_array = np.asarray(route_points)

    route_geometry.points = o3d.utility.Vector3dVector(
        points_array
    )

    if len(route_points) >= 2:
        lines = [
            [i, i + 1]
            for i in range(len(route_points) - 1)
        ]

        route_geometry.lines = o3d.utility.Vector2iVector(
            np.asarray(lines, dtype=np.int32)
        )

        # Warna garis rute: hijau
        route_geometry.colors = o3d.utility.Vector3dVector(
            np.tile(
                np.array([[0.1, 1.0, 0.2]]),
                (len(lines), 1)
            )
        )

    visualizer.update_geometry(route_geometry)

    # Pindahkan marker ke posisi drone terbaru
    drone_marker.translate(
        current_position - np.asarray(
            drone_marker.get_center()
        ),
        relative=True
    )

    visualizer.update_geometry(drone_marker)


# =========================================================
# 7. LOOP UTAMA: WASD + MAPPING + RUTE
# =========================================================

try:
    while running:
        loop_start = time.time()

        # -------------------------------------------------
        # A. Kontrol target drone
        # -------------------------------------------------

        position = sim.getObjectPosition(
            target, sim.handle_world
        )

        dx = 0.0
        dy = 0.0
        dz = 0.0

        if "w" in pressed:
            dx += SPEED * STEP_TIME
        if "s" in pressed:
            dx -= SPEED * STEP_TIME
        if "a" in pressed:
            dy += SPEED * STEP_TIME
        if "d" in pressed:
            dy -= SPEED * STEP_TIME
        if "r" in pressed:
            dz += SPEED * STEP_TIME
        if "f" in pressed:
            dz -= SPEED * STEP_TIME

        if dx or dy or dz:
            new_position = [
                position[0] + dx,
                position[1] + dy,
                position[2] + dz
            ]

            sim.setObjectPosition(
                target,
                sim.handle_world,
                new_position
            )

        # -------------------------------------------------
        # B. Ambil scan baru dari Lua
        # -------------------------------------------------

        sequence = sim.getInt32Signal(SEQUENCE_SIGNAL)

        if (
            sequence is not None
            and sequence != last_sequence
        ):
            last_sequence = sequence

            new_points = read_lidar_points()

            if new_points is not None:
                all_points = np.vstack(
                    (all_points, new_points)
                )

                # Batasi penggunaan RAM
                if len(all_points) > MAX_RAW_POINTS:
                    indices = np.linspace(
                        0,
                        len(all_points) - 1,
                        MAX_RAW_POINTS,
                        dtype=int
                    )

                    all_points = all_points[indices]

                print(
                    f"\rScan: {sequence} | "
                    f"Titik baru: {len(new_points)} | "
                    f"Total titik: {len(all_points)}",
                    end="",
                    flush=True
                )

        # -------------------------------------------------
        # C. Rekam posisi aktual drone
        # -------------------------------------------------

        now = time.time()

        if now - last_route_time >= ROUTE_INTERVAL:
            update_route()
            last_route_time = now

        # -------------------------------------------------
        # D. Update visualisasi point cloud
        # -------------------------------------------------

        if now - last_map_update >= MAP_UPDATE_INTERVAL:
            if len(all_points) > 0:
                cloud = o3d.geometry.PointCloud()

                cloud.points = o3d.utility.Vector3dVector(
                    all_points
                )

                # Downsampling untuk rendering
                cloud = cloud.voxel_down_sample(
                    voxel_size=VOXEL_SIZE
                )

                
                # Warna berdasarkan ketinggian (sumbu Z)
                xyz = np.asarray(cloud.points)

                if len(xyz) > 0:
                    z = xyz[:, 2]

                    z_min = np.min(z)
                    z_max = np.max(z)

                    # Normalisasi ketinggian ke rentang 0 sampai 1
                    if z_max > z_min:
                        height = (z - z_min) / (z_max - z_min)
                    else:
                        height = np.zeros_like(z)

                    colors = np.zeros((len(xyz), 3))

                    # Rendah: biru
                    # Sedang: hijau/kuning
                    # Tinggi: merah
                    colors[:, 0] = np.clip(2 * height - 0.5, 0, 1)
                    colors[:, 1] = np.clip(1.5 - 2 * np.abs(height - 0.5), 0, 1)
                    colors[:, 2] = np.clip(1 - 2 * height, 0, 1)

                    cloud.colors = o3d.utility.Vector3dVector(colors)

                map_geometry.points = cloud.points
                map_geometry.colors = cloud.colors

                visualizer.update_geometry(map_geometry)

                # Atur kamera hanya sekali, setelah data cukup
                if (
                    not camera_initialized
                    and len(all_points) >= 1000
                ):
                    visualizer.reset_view_point(True)

                    center = cloud.get_center()
                    view.set_lookat(center.tolist())
                    view.set_front([1, -1, -0.7])
                    view.set_up([0, 0, 1])
                    view.set_zoom(0.5)

                    camera_initialized = True

                    print(
                        "\nKamera Open3D diarahkan ke point cloud."
                    )

            last_map_update = now

        # -------------------------------------------------
        # E. Proses event jendela Open3D
        # -------------------------------------------------

        if not visualizer.poll_events():
            running = False
            break

        visualizer.update_renderer()

        elapsed = time.time() - loop_start

        if elapsed < STEP_TIME:
            time.sleep(STEP_TIME - elapsed)

except KeyboardInterrupt:
    print("\nProgram dihentikan.")

except Exception as error:
    print(f"\nError: {error}")

finally:
    running = False
    listener.stop()
    visualizer.destroy_window()

    print("\nKontrol, mapping, dan perekaman rute dihentikan.")
