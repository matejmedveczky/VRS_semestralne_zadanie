# Pásový mobilný robot - Technická špecifikácia

## 1. PREHĽAD PROJEKTU

### 1.1 Cieľ
Vytvorenie funkčného modelu pásového robota s diferenciálnym pohonom, 
ovládaného cez WiFi z PC pomocou gamepadu. Riadenie rýchlosti motorov 
implementované pomocou PID regulátora so spätnou väzbou z IMU.

### 1.2 Vlastnosti
- Diferenciálny pohon: 2× DC775 motor s individuálnym PID riadením
- Distribuovaná architektúra: STM32 (low-level control) + RPi (WiFi gateway)
- Riadenie cez WiFi: ROS2 (PC ↔ RPi) + UART (RPi ↔ STM)

---

## 2. SYSTÉMOVÁ ARCHITEKTÚRA

### 2.1 Blokový diagram
```
┌──────────┐   WiFi/ROS2    ┌──────────┐   UART 115200    ┌──────────┐
│ PC       │ ─────────────> │ RPi 4    │ ─────────────>   │ STM32F4  │
│ Gamepad  │ <───────────── │ Gateway  │ <─────────────   │ MCU      │
└──────────┘                └──────────┘   spätná väzba   └─────┬────┘
                                                                │
                                                                │ PWM/GPIO
                                                                │
                                                     ┌──────────┴──────────┐
                                                     │                     │
                                                 ┌───▼────┐           ┌───▼────┐
                                                 │ Motor  │           │ Motor  │
                                                 │ L      │           │ R      │
                                                 │ DC775  │           │ DC775  │
                                                 └────────┘           └────────┘
                                                     │                     │
                                                 ┌───▼─────────────────────▼───┐
                                                 │     MPU6500 IMU             │
                                                 │  (I2C/SPI to STM)           │
                                                 └─────────────────────────────┘
```

### 2.2 Tok dát

1. PC: Vstup z gamepadu → ROS2 
2. RPi: ROS subscriber → UART 
3. STM: UART RX → PID regulátor
4. STM: Čítanie MPU6500 → Integrovanie rýchlosti → spätná väzba PID
5. STM: Výstup PID → PWM k motorom
6. STM: Telemetria → UART TX → RPi
7. RPi: ROS publishers

---

## 3. HARDVÉROVÁ ŠPECIFIKÁCIA

### 3.1 Komponenty
| Komponent      | Model/Špecifikácia   | Rozhranie    | Účel                       |
|----------------|----------------------|--------------|----------------------------|
| MCU            | STM32                | -            | Riadenie motorov, PID, IMU |
| SBC            | Raspberry Pi 4       | WiFi, UART   | ROS gateway                |
| Motory         | 2× DC775             | PWM          | Diferanciálny pohon        |
| Motor Driver   | TB6612FNG/DRV8833    | GPIO, PWM    | H-bridge                   |
| IMU            | MPU6500              | I2C/SPI      | Odhad rýchlosti            |
| Power Monitor  | Voltage divider      | ADC          | Monitorovanie napätia batérie |

---

## 4. SOFTVÉROVÁ ARCHITEKTÚRA

### 4.1 Raspberry Pi (ROS2 Humble)

#### Node 1: `teleop_node` (Šimon)
**Vstup:** Gamepad cez `joy` package (sensor_msgs/Joy)  
**Výstup:** `/cmd_vel` (geometry_msgs/Twist)  
**Logika:**
- Mapovanie joysticku → linear.x (dopredu/dozadu), angular.z (rotácia)
- Deadzone filtering, obmedzenie akcelerácie
- Bezpečnosť: obmedzenie maximálnej rýchlosti

#### Node 2: `uart_bridge_node` (Šimon)
**Subscribers:**
- `/cmd_vel` (geometry_msgs/Twist) → serializácia do UART packetu

**Publishers:**
- `/battery_state` (sensor_msgs/BatteryState) - napätie z ADC


┌─────┬──────┬─────────┬─────────┬─────────┬──────┐
│Start│ Dĺžka│ VelL    │ VelR    │ Batéria │ CRC8 │
│0xBB │ 1B   │ float32 │ float32 │ uint16  │ 1B   │
└─────┴──────┴─────────┴─────────┴─────────┴──────┘

---

### 4.2 STM32 Firmware

#### Modul 1: UART komunikácia (Miro)
**Zodpovednosti:**
- RX interrupt handler + ring buffer
- Parsovanie frame s CRC validáciou

#### Modul 2: MPU6500 Driver (Miro)
**Zodpovednosti:**
- I2C/SPI inicializácia
- Čítanie: accel XYZ, gyro XYZ
- Kalibrácia

#### Modul 3: Riadenie motorov (Mato)
**Zodpovednosti:**
- PWM generácia (TIM2, TIM3)

#### Modul 4: PID regulátor (Mato)
**Zodpovednosti:**
- Dvojitý PID regulátor riadiaci motory

#### Modul 5: Monitorovanie batérie (Kubo)
**Zodpovednosti:**
- ADC čítanie cez voltage divider
- Alarm pri nízkom napätí: < 10.5V → varovná správa
