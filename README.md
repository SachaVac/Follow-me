1) Vysoká úroveň řešení (co dělá který blok)
Percepce
Kamera + YOLOv4 → detekce osoby (bounding box + confidence).
RPLIDAR → 2D mapování (SLAM) + detekce překážek v rovině.
2× VL53L0X → krátkodosahová detekce před robotem (safety stop / blind-spot).
(Volitelné) UWB → přesné měření vzdálenosti k tagu (pokud chceš robustní follow-me mimo viditelnost).
Sub-control / low-level
RP2040: čtení enkodérů, PWM řízení motorů, čtení VL53L0X (I²C), signalizace nouze (E-stop).
Jetson Nano: běh výpočetně náročných úloh (YOLO, SLAM, plánování), ROS master.
Řízení / plánování
Lokální plánovač (path follower) + bezpečnostní vrstva (emergency stop z VL53L0X / bump).
Sledovací logika (person tracker → generování cílové pozice relativně k robotu).
Komunikace
ROS topics přes Wi-Fi (nebo lokálně), RP2040 komunikuje s Jetsonem přes UART nebo USB-CDC (ROS node na Jetsonu čte data).
2) Volba SW stacku (doporučení)
OS / základ: Ubuntu (verze kompatibilní s JetPack, který používáš), JetPack pro Jetson Nano + nVidia TensorRT pro rychlé inference.
ROS: použij ROS (pokud jsi na Jetson Nano, většina projektů používá ROS1 — Noetic na Ubuntu 20.04; ROS2 je také možnost, ale ekosystém knihoven je rozsáhlejší v ROS1). (Pozn.: ověř si kompatibilitu s tvým JetPackem.)
YOLO: YOLOv4 nebo YOLOv4-tiny (na Jetsonu doporučená lehčí verze nebo převod do TensorRT).
SLAM / nav: gmapping nebo jiný 2D SLAM pro mapování + move_base (nav stack) nebo Nav2 (ROS2) + lokální planner (TEB nebo DWA).
LIDAR driver: rplidar_ros.
VL53 driver: existují ROS wrappery nebo jednoduché I²C uzly na RP2040, které publikuji sensor_msgs/Range.
RP2040 firmware: jednoduchý firmwarový loop čtoucí enkodéry, čidla, posílá přes UART JSON nebo ROS-serial.
3) Konkrétní nodes / topics (návrh)
Toto je doporučená topologie ROS (pojmenování lze přizpůsobit):
Percepce:
/camera/image_raw — sensor_msgs/Image (kamera)
/yolo/detections — custom msg nebo vision_msgs/Detection2DArray (bounding boxes, confidences)
/person/pose — geometry_msgs/PoseStamped (odhadovaný cíl v robot-frame: x,y,yaw)
LIDAR & SLAM:
/scan — sensor_msgs/LaserScan (RPLIDAR)
/odom — nav_msgs/Odometry (enkodéry + IMU pokud máš)
/map — nav_msgs/OccupancyGrid (gmapping)
Safety sensors:
/range/front_left /range/front_right — sensor_msgs/Range (VL53)
/bump — std_msgs/Bool (bump sensor)
Low level:
/cmd_vel — geometry_msgs/Twist (výstup plánovače / follow logic)
/motor_commands — custom nebo std_msgs/Float32MultiArray (RP2040 subscriber)
/rp2040/status — custom status (baterie, faults, encoder ticks)
Emergency:
/emergency_stop — std_msgs/Bool (publikováno RP2040 nebo safety node)
4) Datové toky a logika follow-me
Detekce osoby (YOLO): z obrazu získáš bounding box. Vypočti centrovanou úhlovou odchylku a odhad vzdálenosti (pomocí velikosti boxu nebo stereo/ToF/UWB pokud k dispozici).
Transformace target → robot_frame: z obrazu + vyskalování (empirické) dostaneš person/pose v souřadnicích (x ≈ vzdálenost, y ≈ boční odchylka).
Behavior:
Pokud person existuje a vzdálenost > target_distance → nastav cmd_vel s lineární rychlostí k udržení vzdálenosti a úhlovou rychlostí pro zarovnání.
Pokud person ztracen → přejdi do state search (otáčení a skenování) nebo stop.
Safety layer:
Při range < stop_threshold z VL53 nebo scan detekuje překážku přímo v trajektorii → publikuj /emergency_stop = true → RP2040 okamžitě vypne PWM / nastaví brzdu a Jetson zastaví plánovač.
5) Hardwarová integrace & wiring (prakticky)
RP2040 ↔ Jetson: UART (USB-CDC) nebo ROS serial. RP2040 posílá enkodéry, VL53 data, stav E-stop; Jetson posílá požadavky na rychlost/motor commands.
VL53L0X: I²C. Pozor — adresa je stejná pro oba moduly, bude třeba adresní multiplexer (TCA9548A) nebo změna adresy (některé breakouty to umožní). Alternativně RP2040 čte a multiplexuje.
Enkodéry: do RP2040 (rychlé čtení z interrupcí), počítej pulzy → publikuj nav_msgs/Odometry.
Motory: přes motor driver (v expansion boardu). Zkontroluj logiku PWM a direction pins, ochranné prvky.
RPLIDAR: USB/serial → připojit přímo k Jetsonu (rplidar driver).
Kamera IMX219: CSI konektor do Jetsonu.
6) Rozvojový plán (fáze + milníky)
Doporučené iterace (rychlé, testovatelné kroky):
Fáze 0 — příprava
Nainstaluj OS, JetPack, TensorRT, ROS, potřebné drivere (rplidar_ros).
Ověř HW: kamera produkuje obrázky, LIDAR scanuje, RP2040 komunikuje.
Fáze 1 — low-level kontrola
Implementuj RP2040 firmware:
čtení enkodérů, jednoduché PID na motory, UART/USB telemetry.
REST/serial rozhraní pro vysílání stavu a přijímání motor_commands.
Na Jetsonu napiš ROS node hw_bridge který komunikuje s RP2040 (publikuje /odom, subscribuje /motor_commands).
Fáze 2 — SLAM + navigace základ
Spusť RPLIDAR + gmapping. Vytvoř mapu místnosti.
Spusť move_base / lokální plánovač, zkušebně posílej cmd_vel lokálně a ověř řízení.
Fáze 3 — person detection
Nasazení YOLOv4-tiny / TensorRT: detekce osob z kamery; publish vision_msgs/Detection2DArray.
Implementuj node person_tracker → consistent ID, smoothing (Kalman filter) a převod na person/pose v robot frame.
Fáze 4 — follow behavior
Node follow_controller čte person/pose + /scan + /range/*. Generuje cmd_vel s omezeními (max speed, max angular), publikuje /cmd_vel.
Integrace s move_base (můžeš použít cmd_vel přímo, nebo nastavit person pose jako lokalní cílovou pozici pro nav stack).
Fáze 5 — safety a ladění
Přidej emergency stop z RP2040 / VL53.
Testy v uzavřeném prostředí: rychlost, reakční doba, chování při ztrátě cíle.
Ladění PID, filtrace dat (median filter pro VL53, outlier removal pro LIDAR).
Fáze 6 — vylepšení
Přidat UWB pro robustní vzdálenost mimo linie viditelnosti (pokud máš tag na osobě).
Přidat IMU pro lepší dead-reckoning.
Přepsat časově kritické části do C++ nebo využít TensorRT FP16 pro rychlost.
7) Důležité parametry & tipy pro ladění
Loop rates:
YOLO inference: cíl ≥ 10 FPS (YOLO-tiny; TensorRT).
LIDAR: 5–10 Hz (A1 typical).
VL53: 20–50 Hz.
RP2040 ↔ Jetson telemetry: 20–50 Hz.
Safety thresholds (příklad):
VL53 stop threshold: 0.25–0.4 m (doladit podle rychlosti).
LIDAR stop sector: přední ±20° vzdálenost < 0.4 m → nouzové zastavení.
Max linear speed: 0.3–0.6 m/s (dle bezpečnosti a řízení).
Smoothing: Kalman filter / exponential moving average pro person/pose → zabrání „třepání“.
Latency: snaž se minimalizovat latenci mezi detekcí a cmd_vel (rychlé inference + lokální safety v RP2040).
8) Testovací checklist
HW: kamera, LIDAR, RP2040, motory fungují nezávisle.
RP2040 → Jetson komunikace funguje, /odom publikováno.
LIDAR mapuje prostředí (gmapping).
YOLO detekuje osobu spolehlivě v různých osvětleních.
follow_controller udržuje vzdálenost a směr při pomalé chůzi.
Nouzové zastavení funguje při ručním vložení překážky.
Test na různých površích / úhlech, s rotující osobou, s částečným zakrytím.
Zálohové bezpečnosti: bump senzory/kill switch fungují.
9) Praktické poznámky k implementaci YOLOv4 na Jetsonu
Použij YOLOv4-tiny nebo převedený model do TensorRT (lepší latence na Nano). Optimalizuj do FP16/INT8 podle potřeby.
Pokud nechceš trénovat: použij pretrained COCO model (třída person).
Pokud chceš lepší přesnost, sbírej vlastní dataset z kamery a dolad si model (transfer learning).
10) Co dělat teď — konkrétní první 7 úkolů (action items)
Zprovoznit Jetson (JetPack, ROS, TensorRT) a ověřit kamera + RPLIDAR → /camera/image_raw, /scan.
Nahrát jednoduchý RP2040 firmware: čtení enkodérů + UART echo; ověřit komunikaci s Jetsonem.
Spustit rplidar_ros + gmapping → vytvořit první mapu.
Nasadit YOLOv4-tiny inference demo (kamera → bounding box) na Jetson (měř FPS).
Implementovat person_tracker node: z bounding box → person/pose.
Implementovat jednoduchý follow_controller (image→cmd_vel) bez plánovače; test v kontrolovaném prostředí.
Přidat VL53L0X + emergency stop (RP2040 čte, publikuje stop) a otestovat zastavení.
11) Bezpečnost & provozní rady
Před testováním vždy odstraň hodnotné/něžné objekty z oblasti.
Měj fyzický kill switch přístupný během testu.
Nastav nízké výchozí rychlosti a postupně zvyšuj.
Udržuj logy (rosbag) pro ladění.
