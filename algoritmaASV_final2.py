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

print('Connecting...')
vehicle = connect('tcp:127.0.0.1:5762')

# Parameter grid
num_horizontal_lines = 3  # Jumlah grid vertikal
num_vertical_lines = 3    # Jumlah grid horizontal

# Variabel global untuk melacak waktu deteksi
no_detection_start_time = None


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


def baca_grid(frame):
    # Baca video dari file
    #cap = cv2.VideoCapture(r'C:\Users\ACER\OneDrive\Dokumen\Bengawan UV\Python\bengawan.mp4')

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

        # Konversi frame ke HSV
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Mask untuk warna merah
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        mask_red1 = cv2.inRange(hsv_frame, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv_frame, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        # Mask untuk warna hijau
        lower_green = np.array([35, 50, 50])
        upper_green = np.array([85, 255, 255])
        mask_green = cv2.inRange(hsv_frame, lower_green, upper_green)

        # Gunakan dilate dan erode untuk menyempurnakan bentuk
        kernel = np.ones((5, 5), np.uint8)
        mask_red = cv2.dilate(mask_red, kernel, iterations=2)
        mask_red = cv2.erode(mask_red, kernel, iterations=1)
        mask_green = cv2.dilate(mask_green, kernel, iterations=2)
        mask_green = cv2.erode(mask_green, kernel, iterations=1)

        # Variabel untuk menyimpan grid yang mendeteksi bola merah dan hijau
        detected_red_grids = set()
        detected_green_grids = set()

        # Ukuran frame
        height, width, _ = frame.shape

        # Deteksi bola merah
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours_red:
            if cv2.contourArea(contour) > 100:  # Abaikan kontur kecil
                (x, y), radius = cv2.minEnclosingCircle(contour)
                center_x, center_y = int(x), int(y)
                radius = int(radius)

                # Gambar bounding circle merah
                cv2.circle(frame, (center_x, center_y), radius, (0, 0, 255), 2)

                # Tentukan grid tempat bola merah terdeteksi
                grid_x = center_x // (width // num_vertical_lines)
                grid_y = center_y // (height // num_horizontal_lines)
                grid_number = (num_horizontal_lines - grid_y - 1) * num_vertical_lines + grid_x + 1

                # Abaikan grid kosong
                if grid_number not in blank_grids:
                    detected_red_grids.add(grid_number)

        # Deteksi bola hijau
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours_green:
            if cv2.contourArea(contour) > 100:  # Abaikan kontur kecil
                (x, y), radius = cv2.minEnclosingCircle(contour)
                center_x, center_y = int(x), int(y)
                radius = int(radius)

                # Gambar bounding circle hijau
                cv2.circle(frame, (center_x, center_y), radius, (0, 255, 0), 2)

                # Tentukan grid tempat bola hijau terdeteksi
                grid_x = center_x // (width // num_vertical_lines)
                grid_y = center_y // (height // num_horizontal_lines)
                grid_number = (num_horizontal_lines - grid_y - 1) * num_vertical_lines + grid_x + 1

                # Abaikan grid kosong
                if grid_number not in blank_grids:
                    detected_green_grids.add(grid_number)

        # Logika pengambilan jalur
        if detected_green_grids == {6} and detected_red_grids == {4}:
            gerak(0.8, 0)  # Lurus
        elif detected_green_grids == {6, 3} and detected_red_grids == {4, 1}:
            gerak(0.8, 0)  # Lurus
        elif detected_green_grids == {3} and detected_red_grids == {1}:
            gerak(0.8, 0)  # Lurus
        elif detected_green_grids == {5, 6} and not detected_red_grids:
            gerak(0.7, -0.3)  # Belok kiri 20 derajat
        elif detected_green_grids == {5, 6, 3} and not detected_red_grids:
            gerak(0.5, -0.4)  # Belok kiri 30 derajat
        elif detected_green_grids == {3, 2} and not detected_red_grids:
            gerak(0.2, -0.6)  # Belok kiri 40 derajat
        elif detected_red_grids == {5, 4} and not detected_green_grids:
            gerak(0.7, 0.3)  # Belok kanan 20 derajat
        elif detected_red_grids == {5, 4, 1} and not detected_green_grids:
            gerak(0.5, 0.4)  # Belok kanan 30 derajat
        elif detected_red_grids == {2, 1} and not detected_green_grids:
            gerak(0.2, 0.6)  # Belok kanan 40 derajat
        else:
            # Jika tidak ada deteksi bola, cek waktu
            if not detected_red_grids and not detected_green_grids:
                if no_detection_start_time is None:
                    no_detection_start_time = time.time()
                elif time.time() - no_detection_start_time > 1:
                    gerak(0, 0)  # STOP
            else:
                no_detection_start_time = None  # Reset waktu jika ada deteksi

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
        cv2.imshow("Video Asli dengan Grid", frame)
        cv2.imshow("Mask Merah", mask_red)
        cv2.imshow("Mask Hijau", mask_green)

        # Berhenti dengan tombol 'q'
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

    # Menutup video dan jendela
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    add_last_waypoint_to_mission(vehicle.location.global_relative_frame.lat,
                                 vehicle.location.global_relative_frame.lon,
                                 vehicle.location.global_relative_frame.alt)
    print("Home waypoint added to the mission")
    arm_and_takeoff(10)
    print("READY")

    cap = cv2.VideoCapture(r'C:\Users\ACER\OneDrive\Dokumen\Bengawan UV\Python\bengawan.mp4')
    
    baca_grid(cap)
