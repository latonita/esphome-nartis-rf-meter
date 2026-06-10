[СПОДЭС/DLMS/COSEM](https://github.com/latonita/esphome-dlms-cosem) •
[МЭК-61107/IEC-61107](https://github.com/latonita/esphome-iec61107-meter) •
[Энергомера МЭК/IEC](https://github.com/latonita/esphome-energomera-iec) •
[Энергомера CE](https://github.com/latonita/esphome-energomera-ce) •
[СПб ЗИП ЦЭ2727А](https://github.com/latonita/esphome-ce2727a-meter) •
[Ленэлектро ЛЕ-2](https://github.com/latonita/esphome-le2-meter) •
[Пульсар-М](https://github.com/latonita/esphome-pulsar-m) •
[Энергомера BLE](https://github.com/latonita/esphome-energomera-ble) •
[Нартис RF433](https://github.com/latonita/esphome-nartis-rf-meter) •
[Nordic UART (BLE NUS)](https://github.com/latonita/esphome-nordic-uart-ble) •

---

# esphome-nartis-rf-meter

Компонент ESPHome для связи со счётчиками электроэнергии **Нартис И100 / И300** по радиоканалу **433 МГц** с помощью ESP32 и радиомодуля **CMT2300A**.

Компонент заменяет собой выносной цифровой дисплей **Нартис Д101**: он **сопрягается** со счётчиком и далее **регулярно опрашивает** счётчик по протоколу **DLMS/COSEM**, забирая нужные показания по их OBIS-кодам. Полный список кодов можно найти в руководстве по эксплуатации дисплея Нартис Д101, а также приборов учета электроэнергии Нартис И100 и Нартис И300.

# Функции

## Реализованы

- **Активный опрос** счётчика по интервалу (`update_interval`).
- **Сопряжение (pairing)** со счётчиком: эмуляция probe → ACK → SESSION_SETUP → подтверждение, как делает настоящий CIU.
- **Шифрование AES-128-GCM**; сессионный ключ автоматически извлекается из ответа счётчика при сопряжении (на основе серийного номера счётчика).
- **Автоматическое восстановление**: при сбое цикла компонент возвращается в ожидание и повторяет попытку на следующем интервале; после нескольких неудачных чтений подряд — автоматически выполняет повторное сопряжение.
- **Чтение пакетами** (`batch_size`) через DLMS get-request-with-list — несколько OBIS за один запрос.
- **Числовые и текстовые сенсоры** по произвольным OBIS-кодам, классам и атрибутам COSEM.
- **Авто-выбор канала** по RSSI или фиксация канала вручную.
- **Сохранение сессии в NVS**: ключ, счётчики кадров, адрес счётчика сохраняются во flash и восстанавливаются после перезагрузки — сопряжение заново не выполняется.


## Ограничения 

- Автоматическое масштабирование значений не происходит, т.к. счетчик не передает данные scaler/unit. Используйте `filter: - multiply: xxx`
- Опрос счетчика занимает в среднем около 30 секунд - не рекомендуется делать опрос чаще, чем раз 5 минут, дабы не засорять эфир

## Возможные задачи на будущее

- Поддержка дополнительных типов данных DLMS (Clock)

# Аппаратное обеспечение

Понадобится плата на **ESP32** и радиомодуль **CMT2300A** (sub-GHz). Например, **XL2300-SMT**.

Важно: CMT2300A использует нестандартный **3-проводной SPI**, поэтому обмен реализован программным («bit-bang») SPI через обычные GPIO, а не через аппаратный SPI-контроллер. Линия прерывания **GPIO3 (INT2) обязательна** — по ней отслеживается заполнение FIFO при приёме.

## Подключение CMT2300A к ESP32

| CMT2300A         | ESP32   | Описание                          |
|------------------|---------|-----------------------------------|
| SDIO             | GPIO 23 | Данные SPI (двунаправленная)      |
| SCLK             | GPIO 18 | Тактовый сигнал SPI               |
| CSB              | GPIO 27 | Chip Select (банк регистров)      |
| FCSB             | GPIO 26 | FIFO Chip Select                  |
| GPIO3 (INT2)     | GPIO 34 | Прерывание FIFO (обязательно)     |
| VDD              | 3.3V    | Питание                           |
| GND              | GND     | Общий провод                      |

> GPIO выбраны как пример — подставьте пины своей платы. На ESP32 пины 34–39 работают только на вход, что подходит для линии прерывания GPIO3.

# Установка

```yaml
external_components:
  - source: github://latonita/esphome-nartis-rf-meter
    components: [nartis_rf_meter]
    refresh: 1s
```

# Быстрый старт

```yaml
external_components:
  - source: github://latonita/esphome-nartis-rf-meter
    components: [nartis_rf_meter]
    refresh: 1s

# Источник времени обязателен: счётчик проверяет «свежесть» часов в маяке
# и не отдаёт данные при остановившихся часах. Подойдёт время из Home Assistant,
# SNTP, RTC — либо параметр initial_time у хаба (см. ниже).
time:
  - platform: homeassistant
    id: homeassistant_time

nartis_rf_meter:
  pin_sdio: GPIO23
  pin_sclk: GPIO18
  pin_csb: GPIO27
  pin_fcsb: GPIO26
  pin_gpio3: GPIO34

  meter_serial: "012345678901"   # 12 цифр с шильдика счётчика
  
  update_interval: 5min

sensor:
  - platform: nartis_rf_meter
    name: "Активная мощность"
    obis_code: "1.0.1.7.0.255"
    unit_of_measurement: W
    accuracy_decimals: 1
    device_class: power
    state_class: measurement
```

# Как это работает

1. По интервалу `update_interval` запускается цикл опроса.
2. Если сопряжение ещё не выполнено — компонент сканирует каналы по RSSI, выбирает канал и проходит handshake со счётчиком, получая сессионный ключ.
3. Если сопряжение уже есть — ключ и счётчики кадров переиспользуются, и сразу запускается чтение данных.
4. Запрашиваются настроенные сенсоры (по одному или пакетами `batch_size`), значения публикуются в Home Assistant.
5. По завершении цикла радио засыпает до следующего интервала.

# Конфигурация хаба (`nartis_rf_meter`)

| Параметр                      | Тип / по умолчанию | Описание                                                                                       |
|-------------------------------|--------------------|------------------------------------------------------------------------------------------------|
| `pin_sdio`                    | GPIO, обязательно  | Линия данных SPI.                                                                              |
| `pin_sclk`                    | GPIO, обязательно  | Тактовый сигнал SPI.                                                                           |
| `pin_csb`                     | GPIO, обязательно  | Chip Select банка регистров.                                                                   |
| `pin_fcsb`                    | GPIO, обязательно  | FIFO Chip Select.                                                                              |
| `pin_gpio3`                   | GPIO, обязательно  | Линия прерывания INT2 (заполнение FIFO при приёме).                                            |
| `meter_serial`                | строка, обязательно| 12-значный серийный номер счётчика с шильдика (например `"021245003137"`).                     |
| `update_interval`             | время, `5min`      | Период опроса счётчика.                                                                         |
| `fix_channel`                 | 0…3, опц.          | Зафиксировать радиоканал и отключить авто-выбор по RSSI.                                        |
| `use_non_standard_channels`   | bool, `false`      | Использовать альтернативную сетку частот — иногда помогает, если счётчик не отвечает.           |
| `ciu_serial`                  | строка, опц.       | Серийный номер CIU — для замены существующего модуля. По умолчанию выводится из MAC ESP32.      |
| `ciu_address`                 | 16 hex, опц.       | Полный 8-байтный RF-адрес CIU (например `"CD2C0000026B5025"`) — точная эмуляция уже сопряжённого CIU. |
| `batch_size`                  | 1…10, `1`          | Сколько OBIS-атрибутов читать одним запросом get-request-with-list.                            |
| `rx_timeout`                  | время, `3000ms`    | Общий таймаут ожидания ответа (сопряжение и завершение приёма кадра).                          |
| `rx_reply_timeout`            | время, `900ms`     | Короткий таймаут ожидания ответа на GET — до прихода первого байта.                            |

> Счётчику требуются «живые», идущие часы во встроенном маяке, иначе он подтверждает запрос, но не отдаёт данные. Используйте платформу `time:` (Home Assistant / SNTP / RTC) либо параметр `initial_time`.

# Сенсоры

### Числовой сенсор (sensor)

```yaml
sensor:
  - platform: nartis_rf_meter
    name: "Напряжение фаза A"
    obis_code: "1.0.32.7.0.255"
    unit_of_measurement: V
    accuracy_decimals: 1
    device_class: voltage
    state_class: measurement
    filters:
      - multiply: 0.1   # при необходимости масштабируйте значение фильтром
```

Параметры: `obis_code` (обязательно), `obis_class` (по умолчанию `3`), `obis_attr` (по умолчанию `2`). Поддерживаются форматы OBIS `1.0.32.7.0.255`, `1-0:32.7.0.255`, `1-0:32.7.0*255`.

### Текстовый сенсор (text_sensor)

```yaml
text_sensor:
  - platform: nartis_rf_meter
    name: "Серийный номер"
    obis_code: "0.0.96.1.0.255"
```

# Примеры конфигураций

### Трёхфазный счётчик — базовые показания

```yaml
external_components:
  - source: github://latonita/esphome-nartis-rf-meter
    components: [nartis_rf_meter]
    refresh: 1s

time:
  - platform: homeassistant
    id: homeassistant_time

nartis_rf_meter:
  pin_sdio: GPIO23
  pin_sclk: GPIO18
  pin_csb: GPIO27
  pin_fcsb: GPIO26
  pin_gpio3: GPIO34
  meter_serial: "012345678901"
  update_interval: 5min
  # use_non_standard_channels: true   # раскомментируйте, если счётчик не отвечает

sensor:
  - platform: nartis_rf_meter
    name: "Напряжение фаза A"
    obis_code: "1.0.32.7.0.255"
    unit_of_measurement: V
    accuracy_decimals: 1
    device_class: voltage
    state_class: measurement
    filters:
      - multiply: 0.1

  - platform: nartis_rf_meter
    name: "Напряжение фаза B"
    obis_code: "1.0.52.7.0.255"
    unit_of_measurement: V
    accuracy_decimals: 1
    device_class: voltage
    state_class: measurement
    filters:
      - multiply: 0.1

  - platform: nartis_rf_meter
    name: "Напряжение фаза C"
    obis_code: "1.0.72.7.0.255"
    unit_of_measurement: V
    accuracy_decimals: 1
    device_class: voltage
    state_class: measurement
    filters:
      - multiply: 0.1

  - platform: nartis_rf_meter
    name: "Ток фаза A"
    obis_code: "1.0.31.7.0.255"
    unit_of_measurement: A
    accuracy_decimals: 2
    device_class: current
    state_class: measurement
    filters:
      - multiply: 0.1

  - platform: nartis_rf_meter
    name: "Активная мощность (сумма)"
    obis_code: "1.0.1.7.0.255"
    unit_of_measurement: W
    accuracy_decimals: 1
    device_class: power
    state_class: measurement

  - platform: nartis_rf_meter
    name: "Энергия импорт (T0)"
    obis_code: "1.0.1.8.0.255"
    unit_of_measurement: kWh
    accuracy_decimals: 0
    device_class: energy
    state_class: total_increasing

  - platform: nartis_rf_meter
    name: "Частота сети"
    obis_code: "1.0.14.7.0.255"
    unit_of_measurement: Hz
    accuracy_decimals: 2
    device_class: frequency
    state_class: measurement
    filters:
      - multiply: 0.01

```

Полный пример — в файле [`example_esp32.yaml`](example_esp32.yaml).

# Диагностика и советы

- **Счётчик не отвечает / нет сопряжения.** Попробуйте `use_non_standard_channels: true` либо зафиксируйте канал `fix_channel: 0…3`. Проверьте антенну и питание модуля.
- **Проверьте серийный номер.** `meter_serial` — ровно 12 цифр с шильдика; из него выводится стартовый ключ шифрования и адресация.
- **Логи.** Включите подробный вывод, чтобы видеть переходы состояний и дампы кадров:

  ```yaml
  logger:
    level: VERY_VERBOSE
  ```

- **Повторное сопряжение.** Если связь со счётчиком теряется, после нескольких неудачных чтений подряд компонент сам сбрасывает сопряжение и проходит handshake заново на следующем интервале.

# Лицензия

См. файл [LICENSE](LICENSE).
