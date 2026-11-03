# Citroën GSA Temperature monitoring system

### Simple Temperature Sensor System for Citroën GSA Based on DS18B20

The Citroën GSA is rare nowadays, but enthusiasts like me still cherish it! 🚗  

![1231-24](https://github.com/user-attachments/assets/28c70754-3084-4c28-900b-210f2d806add)  

That's why I developed a small temperature monitoring system to ensure a safe and comfortable driving experience.  

The system includes sensors placed outdoors, indoors, and on the engine to track temperatures while driving.

A must-have if this classic is your daily ride!

---

**BOM:**
   - Arduino Uno or Nano (ATmega328P)
   - DS18B20 temperature sensors (pin D4, 4.7 kΩ pull-up)
   - DS3231 real-time clock module
   - SH1106 OLED display (128x64px)
   - 2 push buttons wired to GND: hours on D2, minutes on D3
   - Optional: battery voltage on A0 (33 kΩ / 10 kΩ divider), oil pressure sender (0.5-4.5 V) on A1

**Libraries:** OneWire, DallasTemperature, U8g2, Time, DS1307RTC (works with the DS3231), Timezone.

**Configuration:**
   - Assign each sensor to its correct location (outdoor, indoor, engine): the sensor addresses are printed on the serial monitor (115200 baud) at startup.
   - Set the time once by sending `T` followed by the current Unix time in UTC (output of `date +%s`) on the serial monitor. Daylight saving time is handled automatically; use the buttons for small adjustments.
   - Enable or disable the optional measurements with `OPTION_TENSION` and `OPTION_PRESSION_HUILE`.

![preview](./apercu_v2.png)

Version 1.0 (DS1307, no buttons) is available in the [`1.0`](https://github.com/stoneset/GSA-Temperature/tree/1.0) branch.

---

<table>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/140ef6bf-781a-4ecd-82fb-425c16a6cc0e" width="400"></td>
    <td><img src="https://github.com/user-attachments/assets/658767af-6571-4778-9c2d-8420577c02db" width="300"></td>
  </tr>
</table>