# ServoTorqueTesting

ESP32-S3 embedded project using ESP-IDF 5.5.3. The firmware initializes the board
and PCA9685 with all channels OFF, then starts one application owner task.
All channels start at 600 us, hold for two seconds, then move to their configured
upper pulse widths along a quintic S curve. They hold for two seconds and return
along the same curve, repeating continuously. Each direction takes two seconds
by default; travel and hold times are configured independently. No position
feedback is available.

## Hardware baseline

- Target: ESP32-S3, dual core, 240 MHz.
- Flash: 16 MB, QIO, 80 MHz. Verify these settings against the actual module.
- FreeRTOS tick: 1000 Hz; development optimization and INFO logs.
- Console: UART0, 115200 baud.
- PCA9685: SDA GPIO39, SCL GPIO38, unshifted 7-bit address `0x40`, I2C at 100 kHz.
- PWM baseline: 321.24 Hz requested, internal oscillator nominally 25 MHz.
- Servo commands use microseconds directly, with channel-specific limits below.
  Physical angular travel depends on the connected servo's pulse response.
- Use suitable external SDA/SCL pull-ups to 3.3 V (check the module's existing
  resistors); internal pull-ups are enabled only as supplementary pull-ups.
  Connect controller/module grounds and supply the servos from a suitable power
  supply. OE must be low for PWM output; this board configuration does not control OE.

## Build and flash

Run these commands from an activated ESP-IDF 5.5.3 terminal:

```powershell
idf.py build
idf.py -p COM9 flash monitor
```

Change COM9 to the board's actual port. Exit the monitor with Ctrl+].

`sdkconfig.defaults` records the project baseline and is versioned; generated
`sdkconfig` and `build/` remain ignored. ESP-IDF loads defaults when generating a
new configuration. Existing values in `sdkconfig` take precedence, so changing
`sdkconfig.defaults` alone does not update an existing configuration. Use
`idf.py menuconfig` to update an existing setup, then review
`idf.py save-defconfig` output before committing configuration changes.

## Flash layout

The custom table is `partitions.csv`, with the partition table at `0x8000`.

| Partition | Offset | Size | Purpose |
| --- | --- | --- | --- |
| nvs | 0x9000 | 24 KiB | Future persistent settings and servo calibration |
| phy_init | 0xF000 | 4 KiB | Reserved PHY initialization data |
| factory | 0x10000 | 4 MiB | Single application image |
| Unallocated | 0x410000 | 11.9375 MiB | Reserved for future requirements |

This layout has no OTA slots or filesystem. NVS initialization will be added
when persistent settings are used. When changing partition offsets or sizes on
a programmed board, plan data migration or erase/reinitialize affected data.

## Source layout

- `main/main.c`: application startup entry point only.
- `components/application/app.c`: hardware/pulse-range composition, request queue,
  one FreeRTOS owner task, and centralized failure reporting.
- `components/application/app_profile.c`: single composition source for allowed
  channel limits and independently configurable motion endpoints/timing.
- `components/application/app_scheduler.c`: exclusive business selection and
  output shutdown before transferring control.
- `components/application/business_servo_cycle.c`: all current business rules,
  injected motion configuration, S-curve timing, and explicit motion states.
- `components/application/private_include/trajectory.h`: pure quintic time scaling.
- `components/application/include/app.h`: the only public application header.
- `components/application/private_include/`: private scheduler/business interfaces;
  businesses receive pulse-output and clock callbacks instead of driver handles.
- `components/board/`: I2C bus ownership and board hardware initialization/cleanup.
- `components/board/include/board_config.h`: wiring, address, speed and startup frequency.
- `components/pca9685/`: reusable chip driver; receives a caller-owned I2C bus.
- `components/pca9685/include/pca9685.h`: public interface and ownership/error contracts.
- `components/servo/`: pulse-range validation and 16-channel updates.
- `components/application/Kconfig.projbuild`: S-curve travel and endpoint hold configuration.
- `sdkconfig.defaults`: reproducible baseline configuration.
- `partitions.csv`: custom flash layout.
- `pytest_hello_world.py`: board startup smoke test (requires pytest-embedded).
- `tests/pca9685/`: host tests with fake I2C registers and mutexes.
- `tests/servo/`: pulse limits, S-curve timing, scheduler ownership, and application
  startup/queue tests with fake PWM, clock and RTOS services.

Hardware initialization follows `main -> application -> board -> pca9685 -> ESP-IDF I2C`.
The application adapts business pulse commands to `servo -> pca9685`. The business
contains no PCA9685, I2C or FreeRTOS dependency and never reads the servo bank's
internal fields. The application owns the hardware and pulse-range servo bank;
the scheduler grants at most one business access. Each channel has its own allowed pulse range.

## Business scheduling and ownership

`app_start()` creates one application task and starts the servo-cycle business by
default. Other tasks request selection through the thread-safe public queue API:

```c
app_select_business(APP_BUSINESS_NONE);        // request stop and all outputs OFF
app_select_business(APP_BUSINESS_SERVO_CYCLE); // request restart from 600 us
```

Check the returned `esp_err_t`: ESP_OK means the request was queued, not completed.
The bounded queue holds four requests and reports ESP_ERR_TIMEOUT when full.
Selection and PWM writes are performed only by the application task. Re-selecting
the active business does not restart its phase or timer. On a stop/switch, the old
business first relinquishes control, then the scheduler disables all outputs.
If shutdown fails, no new business is started until an explicit selection retries
shutdown successfully. The primary operation error and any shutdown error are
logged separately. Processing errors stop the business and attempt shutdown; the
application task remains available for a later explicit stop/restart request.

`app_get_status()` copies a task-context snapshot under a short critical section:
lifecycle, active business, motion phase, last execution error, shutdown error,
and startup cleanup error. It performs no I2C I/O and reports completed owner-task
operations, not pending requests. Routine successful polling preserves the last
execution error; successful explicit selection/start clears it.

If startup cleanup fails, the application enters `APP_LIFECYCLE_CLEANUP_REQUIRED`
and retains hardware resources. `app_retry_cleanup()` retries cleanup, returning
to the uninitialized state only after success; `app_start()` can then be retried.
Serialize lifecycle calls; cleanup retry is not a running application shutdown API.

Add future business rules in their own `business_*.c` file with start/process/stop
operations, register the business ID and dispatch in the scheduler, and select it
through `app_select_business()`. Do not give a second task direct access to the
servo bank. Application startup is single-caller and is not a concurrent shutdown
API; hardware resources remain owned by the application for its lifetime.

## Servo loop

| Channels | Alternating pulse endpoints |
| --- | --- |
| 0, 4, 8, 12 | 600 / 1000 us |
| 1, 5, 9, 13 | 600 / 1500 us |
| 2, 6, 10, 14 | 600 / 2000 us |
| 3, 7, 11, 15 | 600 / 2500 us |

Startup writes 600 us to all channels in one batch. This initial command cannot
be interpolated from an unknown physical position. After the initial hold, all
channels move together using normalized time `t` from 0 to 1 and the quintic
S curve `s(t) = 10t^3 - 15t^4 + 6t^5`. The outward pulse is
`600 + (upper - 600) * s(t)`; the return uses `1 - s(t)`. Command velocity and
acceleration are zero at both endpoints. The different ranges share a travel
time, so channels with larger ranges receive faster pulse changes.

Pulse commands are rounded to whole microseconds and sent as one 16-channel
batch about every 20 ms. The application polls every 5 ms; bus and scheduling
latency can delay a sample. Time is measured with the monotonic clock, missed
samples are skipped, and a late motion update clamps to the endpoint. A late
hold poll starts a fresh motion without replaying earlier cycles. Holds begin
after the endpoint transfer finishes. The shared PWM setting stays at 321.24 Hz requested.
The PCA9685's count resolution further quantizes the electrical pulse widths.

`CONFIG_SERVO_MOTION_TIME_MS` defaults to 2000 ms per direction;
`CONFIG_SERVO_HOLD_TIME_MS` defaults to 2000 ms at each endpoint and may be zero.
Both are available under `Servo motion test` in `idf.py menuconfig`.

`app_profile.c` composes the defaults and passes copied motion configuration to the
scheduler/business. Allowed servo limits and requested motion endpoints are distinct
configuration fields, currently identical to preserve behavior. The business no
longer reads Kconfig or embeds channel grouping. Adjust endpoints/limits in this
composition module; start validates both endpoints before output.

These are commanded trajectories, not measured positions or physical arrival
guarantees. An I2C failure stops the active business and triggers an attempt to
disable all outputs; bus failure cannot guarantee OFF. Frequency and pulse
limits must not change while the business runs.

## PCA9685 interface

| API | Purpose |
| --- | --- |
| `pca9685_create` / `pca9685_delete` | Initialize with all outputs OFF / disable and release device |
| `pca9685_set_frequency` / `pca9685_get_frequency` | Set shared frequency / get realized estimate |
| `pca9685_validate_pulse_widths_us` | Check electrical representability without register I/O, using the write conversion |
| `pca9685_set_pwm` | Set one channel's 12-bit ON/OFF counts, including phase offsets |
| `pca9685_set_channels` | Update 1..16 contiguous channels in one I2C transfer |
| `pca9685_set_pulse_width_us` | Set pulse width in microseconds; zero turns the channel OFF |
| `pca9685_set_pulse_widths_us` | Convert and write a contiguous pulse batch under one driver lock |
| `pca9685_disable_channel` / `pca9685_all_off` | Stop one / all channels |

Channel indices are 0..15; tick values are 0..4095. Equal ON/OFF counts are encoded
as full OFF. Normal counts can wrap across the end of a period. Full-ON output is
not exposed. At nominal 25 MHz, the project's requested 321.24 Hz yields prescaler 18
and an estimated 321.2377 Hz (approximately 3112.96 us per period). Pulse conversion
uses this realized prescaler, not the
requested frequency. `oscillator_hz` can be calibrated from a measured frequency;
the driver does not support an external oscillator or automatic calibration.

Changing frequency stops every channel and clears previous commands. Reapply
pulse widths after a successful change. Each handle serializes task-context calls
with a mutex, including pulse conversion and frequency changes. Contiguous writes
use auto-increment and update-on-STOP; this does not synchronize PWM edges across
different chips. Initialize/delete only after stopping concurrent users, and do
not register asynchronous I2C callbacks on the driver's device.

I2C failures are returned as `esp_err_t`. A failed frequency configuration blocks
subsequent PWM/pulse writes until a successful frequency retry; OFF operations
remain available. Readback verifies MODE1 and the prescaler after initialization.
Invalid arguments and mutex timeouts do not change the prior configuration.
Partial/failed bus transfers cannot guarantee an output state. Hardware OE control
would be a separate board feature. Deletion failures retain the handle for retry;
the caller owns the bus and must delete the device before deleting that bus.

The configured 100 ms timeout applies to each mutex wait/transfer, not the whole
initialization sequence. The driver disables All Call/Sub Call responses and does
not issue a general-call reset, so other devices on the bus are not reset.

References: [NXP PCA9685 datasheet](https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf),
[ESP-IDF 5.5.3 I2C API](https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32s3/api-reference/peripherals/i2c.html).

## Driver verification

In Visual Studio Developer PowerShell with the C/C++ tools installed:

```powershell
./tests/pca9685/run_host_tests.ps1
./tests/servo/run_host_tests.ps1
```

The host tests compile the production board/driver sources against fake I2C and
FreeRTOS interfaces. They cover startup full-OFF state, prescaler sequencing,
microsecond conversion, 16-channel updates, parameter boundaries, mutex timeout,
readback mismatch, transfer failures at every initialization step, retry and cleanup.
The servo tests cover repeated 600..1000/1500/2000/2500 us ranges,
batch validation before I/O, S-curve midpoint and quarter points, monotonic
outward motion, symmetric return, exact hold boundaries, transfer-time
accounting, delayed polls, and failure/stop handling. They also exercise exclusive
ownership, duplicate selection, stop/restart, shutdown failure blocking acquisition,
invalid range rejection before motion, startup cleanup, queue saturation,
and the actual application task consuming queued business selections.
Additional regression checks cover every scheduled default trajectory sample,
injected descending endpoints, zero dwell, configuration copying, status/error
snapshots, repeated cleanup failure/recovery, and electrical validation boundaries.
Tests do not verify electrical
timing, real task concurrency, or physical servo motion. The pytest startup smoke
test requires an ESP32-S3 with PCA9685 connected and starts the motion business.
