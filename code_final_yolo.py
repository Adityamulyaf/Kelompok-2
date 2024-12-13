import collections
try:
    from collections import abc
    print("READY")
    collections.MutableMapping = abc.MutableMapping
except:
    pass

from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil
import time
import cv2
import numpy as np
from ultralytics import YOLO

print('Connecting...')
vehicle = connect('tcp:127.0.0.1:5762')
# vehicle = connect('tcp:192.168.229.171:5762')
# vehicle = connect('/dev/ttyACM0')
# vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True)

def arm_and_takeoff(altitude):
    while not vehicle.is_armable:
        print("waiting to be armable")
        time.sleep(1)
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    print("Taking Off")
    vehicle.simple_takeoff(altitude)

    while True:
        v_alt = vehicle.location.global_relative_frame.alt
        print(">> Altitude = %.1f m" % v_alt)
        if v_alt >= altitude - 1.0:
            print("Target altitude reached")
            break

def add_last_waypoint_to_mission(wp_Last_Latitude, wp_Last_Longitude, wp_Last_Altitude):
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()

    missionlist = [cmd for cmd in cmds]

    wpLastObject = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                           0, 0, 0, 0, 0, 0,
                           wp_Last_Latitude, wp_Last_Longitude, wp_Last_Altitude)
    missionlist.append(wpLastObject)

    cmds.clear()
    for cmd in missionlist:
        cmds.add(cmd)
    cmds.upload()

    return cmds.count


def gerak(vx, vy):
    print(f"Sending movement command: vx={vx}, vy={vy}")
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, 0,
        0, 0, 0,
        0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()

def baca_grid(cap):
    # Memuat model YOLO
    model = YOLO('bola2.pt')  # Ganti dengan path model Anda jika diperlukan

    # Parameter grid
    num_horizontal_lines = 3  # Jumlah kotak vertikal
    num_vertical_lines = 3    # Jumlah kotak horizontal

    # Grid kosong (blank spot)
    blank_grids = {7, 8, 9}  # Penomoran grid dimulai dari kiri bawah

    # Variabel untuk memantau waktu
    no_detection_start_time = None

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("Video selesai atau tidak dapat dibuka.")
            break

        # Melakukan deteksi dengan YOLO
        results = model(frame)  # Deteksi pada frame
        detections = results[0].boxes  # Mendapatkan hasil deteksi

        # Variabel untuk menyimpan grid yang mendeteksi bola merah dan hijau
        detected_red_grids = set()
        detected_green_grids = set()

        # Ukuran frame
        height, width, _ = frame.shape

        # Memproses hasil deteksi
        for box in detections:
            cls = int(box.cls[0])  # Kelas objek (misal, 1 untuk merah, 0 untuk hijau)
            x1, y1, x2, y2 = map(int, box.xyxy[0])  # Koordinat kotak pembatas
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2

            # Tentukan grid tempat bola terdeteksi
            grid_x = center_x // (width // num_vertical_lines)
            grid_y = center_y // (height // num_horizontal_lines)
            grid_number = (num_horizontal_lines - grid_y - 1) * num_vertical_lines + grid_x + 1

            # Abaikan grid kosong
            if grid_number not in blank_grids:
                if cls == 1:  # Bola merah
                    detected_red_grids.add(grid_number)
                elif cls == 0:  # Bola hijau
                    detected_green_grids.add(grid_number)

            # Gambar bounding box pada frame
            color = (0, 255, 0) if cls == 0 else (0, 0, 255)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

        # Logika pengambilan jalur
        if detected_green_grids == {6} and detected_red_grids == {4}:
            gerak(0.8, 0)  # Lurus
            print("Lurus")
        elif detected_green_grids == {6, 3} and detected_red_grids == {4, 1}:
            gerak(0.8, 0)  # Lurus
            print("Lurus")
        elif detected_green_grids == {3} and detected_red_grids == {1}:
            gerak(0.8, 0)  # Lurus
            print("Lurus")
        elif detected_green_grids == {2} and not detected_red_grids:
            gerak(0.2, -0.6)  # Belok kiri 40 derajat
            print("Kiri 40")
        elif detected_red_grids == {2} and not detected_green_grids:
            gerak(0.2, 0.6)  # Belok kanan 40 derajat
            print("Kanan 40")
        elif detected_green_grids == {3, 2} and not detected_red_grids:
            gerak(0.2, -0.6)  # Belok kiri 40 derajat
            print("Kiri 40")
        elif detected_red_grids == {2, 1} and not detected_green_grids:
            gerak(0.2, 0.6)  # Belok kanan 40 derajat
            print("Kanan 40")
        elif detected_green_grids == {6, 5, 4} and not detected_red_grids:
            gerak(0.2, -0.6)  # Belok kiri 40 derajat
            print("Kiri 40")
        elif detected_red_grids == {4, 5, 6} and not detected_green_grids:
            gerak(0.2, 0.6)  # Belok kanan 40 derajat
            print("Kanan 40")
        elif detected_green_grids == {5} and not detected_red_grids:
            gerak(0.5, -0.4)  # Belok kiri 30 derajat
            print("Kiri 30")
        elif detected_red_grids == {5} and not detected_green_grids:
            gerak(0.5, 0.4)  # Belok kanan 30 derajat
            print("Kanan 30")
        elif detected_green_grids == {5} and detected_red_grids == {4}:
            gerak(0.5, -0.4)  # Belok kiri 30 derajat
            print("Kiri 30")
        elif detected_red_grids == {5} and detected_green_grids == {6}:
            gerak(0.5, 0.4)  # Belok kanan 30 derajat
            print("Kanan 30")
        elif detected_green_grids == {5, 6, 3} and not detected_red_grids:
            gerak(0.5, -0.4)  # Belok kiri 30 derajat
            print("Kiri 30")
        elif detected_red_grids == {5, 4, 1} and not detected_green_grids:
            gerak(0.5, 0.4)  # Belok kanan 30 derajat
            print("Kanan 30")
        elif detected_green_grids == {5, 6} and not detected_red_grids:
            gerak(0.7, -0.3)  # Belok kiri 20 derajat
            print("Kiri 20")
        elif detected_red_grids == {5, 4} and not detected_green_grids:
            gerak(0.7, 0.3)  # Belok kanan 20 derajat
            print("Kanan 20")
        elif detected_green_grids == {6} and not detected_red_grids:
            gerak(0.7, -0.3)  # Belok kiri 20 derajat
            print("Kiri 20")
        elif detected_red_grids == {2} and not detected_green_grids:
            gerak(0.7, 0.3)  # Belok kanan 20 derajat
            print("Kanan 20")
        else:
            # Jika tidak ada deteksi bola, cek waktu
            if not detected_red_grids and not detected_green_grids:
                if no_detection_start_time is None:
                    no_detection_start_time = time.time()
                elif time.time() - no_detection_start_time > 3:
                    gerak(0, 0)  # STOP
                    break
            else:
                no_detection_start_time = None  # Reset waktu jika ada deteksi

        print(f"Red grids: {detected_red_grids}, Green grids: {detected_green_grids}")

        # Gambar grid dan angka penomoran grid
        for i in range(1, num_vertical_lines):
            cv2.line(frame, (i * width // num_vertical_lines, 0),
                     (i * width // num_vertical_lines, height), (255, 0, 0), 1)
        for i in range(1, num_horizontal_lines):
            cv2.line(frame, (0, i * height // num_horizontal_lines),
                     (width, i * height // num_horizontal_lines), (255, 0, 0), 1)

        for grid_y in range(num_horizontal_lines):
            for grid_x in range(num_vertical_lines):
                grid_number = (num_horizontal_lines - grid_y - 1) * num_vertical_lines + grid_x + 1
                if grid_number not in blank_grids:  # Abaikan grid kosong
                    text_x = grid_x * (width // num_vertical_lines) + 10
                    text_y = grid_y * (height // num_horizontal_lines) + 30
                    cv2.putText(frame, str(grid_number), (text_x, text_y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Tampilkan hasil akhir
        cv2.imshow("Deteksi Bola", frame)

        # Berhenti dengan tombol 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Menutup video dan jendela
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # cap = cv2.VideoCapture(0)
    cap = cv2.VideoCapture(r'C:\Users\ACER\OneDrive\Dokumen\Bengawan UV\Python\belajarOpenCv\bengawan.mp4')
    baca_grid(cap)
