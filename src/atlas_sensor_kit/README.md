# atlas_sensor_kit

Atlas sensor configuration and integration.

Jetson-side ROS 2 Humble point-cloud format adapters. Sensor acquisition remains
on the Raspberry Pi. No sensor networking, TF, launch wiring, or deskew is configured here.

## Intended responsibilities

- Sensor-specific configuration and launch files.
- Topic remapping and sensor integration.

## TODO

- Confirmed sensor inventory and interfaces.
- Confirmed topic names, mounting information, and calibration.

## Components

| Plugin | Standalone executable | Conversion |
| --- | --- | --- |
| `atlas_sensor_kit::LivoxToExtendedComponent` | `livox_to_extended_node` | Livox PointCloud2 → XYZIRCAEDT |
| `atlas_sensor_kit::ExtendedToCompactComponent` | `extended_to_compact_node` | XYZIRCAEDT → XYZIRC |

Both use private `~/input/pointcloud` and `~/output/pointcloud` ports with
SensorDataQoS. Malformed clouds are dropped with a throttled ROS error explaining
the cause. Pure functions in `pointcloud_conversion.hpp` throw `ConversionError`
and never modify input. A failed conversion publishes no partial output.

## Contracts and policies

Inputs must be little-endian, unorganized (`height=1`), with exact field order,
names, types, count=1, offsets, stride, row size, and data size. Empty clouds with
height=1 and complete field metadata are supported. Padding, extra fields, big-endian
data, and negative/non-normalized header stamps are rejected. The implementation
uses memcpy for scalar access, including the unaligned Livox double at offset 18.

Livox fields: `x/y/z` FLOAT32 at 0/4/8, `intensity` FLOAT32 at 12,
`tag/line` UINT8 at 16/17, `timestamp` FLOAT64 at 18; stride 26.

Extended fields: `x/y/z` FLOAT32 at 0/4/8, `intensity/return_type` UINT8
at 12/13, `channel` UINT16 at 14, `azimuth/elevation/distance` FLOAT32
at 16/20/24, `time_stamp` UINT32 at 28; stride 32.

Compact fields: the first six extended fields, packed with stride 16.
The compact adapter copies the first 16 bytes of every point without filtering,
reordering, coordinate transformation, or numeric reinterpretation. It preserves
even nonfinite XYZ bit patterns. Both conversions preserve header, point count,
ordering, and the input density flag.

Livox intensity: finite values are clamped to [0,255], then rounded to nearest
integer (halfway values round upward after clamping). NaN and either infinity
become zero. Livox XYZ must be finite and calculated distance must fit FLOAT32.

Metadata is explicitly provisional: `channel=0`, `return_type=0` (Autoware INVALID).
Neither Livox tag nor generated line is interpreted as Autoware metadata.
Tag and line are dropped without tag filtering. This policy is not suitable for
ring-based algorithms without further sensor-specific validation.

Absolute point timestamps are **nanoseconds**, not seconds or offsets. The exact
integer header time is subtracted in long double; finite offsets must lie in
[0, UINT32_MAX] before rounding to nearest nanosecond (halfway upward).
Decreasing point times are rejected; equal times are accepted. There is no
restamping, unit guessing, clock correction, or negative-offset tolerance.
The proximity/range check rejects typical seconds/microseconds/relative-time
inputs at a real epoch, but a numeric field alone cannot prove its units near
epoch zero. Clock synchronization and units remain an acquisition contract.

At 1.7e18 ns, adjacent doubles are 256 ns apart. Lost precision cannot be recovered.
If rounding in the driver represents the first point earlier than the exact
header stamp, the adapter deliberately rejects the cloud rather than clamping
its timestamp. Hardware commissioning must assess how often this occurs; exact
integer timing would require a different input transport or upstream timestamp
representation. Offset range permits approximately 4.295 seconds, not a configured
scan duration. Long double must have at least 64 significand bits (compile-time checked).

## Verified pinned Autoware conventions

Reference root: `~/roam/references/autoware/src`.

- Universe 0.52.0:
  `universe/autoware_universe/sensing/autoware_pointcloud_preprocessor/src/distortion_corrector/distortion_corrector.cpp`
  checks the XYZIRCAEDT indexed layout and computes point time as header seconds
  plus `(header.nanosec + time_stamp) * 1e-9`. Its angle conversion supports Cartesian
  `atan2(y,x)` with sign +1 and zero offset, normalized to [0,2π).
- The same package's `src/outlier_filter/polar_voxel_noise_filter_node.cpp`,
  `cartesian_to_polar`, establishes elevation `atan2(z,sqrt(x²+y²))` and
  Euclidean radius. Atlas uses hypot for numerical stability. Zero origin maps to
  zero angles/distance; signed-zero XYZ is preserved.
- Core 1.9.0:
  `core/autoware_core/common/autoware_point_types/include/autoware/point_types/types.hpp`
  defines the 32-byte extended and 16-byte compact contracts.
- Universe CenterPoint:
  `perception/autoware_lidar_centerpoint/include/autoware/lidar_centerpoint/preprocess/point_type.hpp`
  defines the six-field 16-byte input, and `lib/preprocess/voxel_generator.cpp`
  casts its input buffer to that type.

The deskewer accepts sensor-specific angle conventions; there is no single universal
vendor azimuth convention. Atlas selects the Cartesian convention it explicitly supports.
The deskewer does not recompute elevation; its azimuth/distance updates are separately
controlled upstream. The pinned deskewer adds two UINT32 nanosecond values before
conversion to double. Atlas rejects a rounded offset greater than
`UINT32_MAX - header.nanosec` so that this sum cannot overflow. Normal scan intervals
must be verified independently.

When `autoware_point_types` is discoverable, the library constructs fields and checks
outputs using the actual upstream layout utilities. It is optional because the
Atlas acquisition workspace does not require a complete Autoware installation;
the self-contained fallback constructs the verified wire contract directly.

## Validation

No additional dependencies are installed by this package. Using the existing Humble
environment, build/test in separate directories:

```bash
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/atlas_lidar_log build --packages-select atlas_sensor_kit \
  --build-base /tmp/atlas_lidar_build --install-base /tmp/atlas_lidar_install \
  --cmake-args -DBUILD_TESTING=ON
colcon --log-base /tmp/atlas_lidar_log test --packages-select atlas_sensor_kit \
  --build-base /tmp/atlas_lidar_build --install-base /tmp/atlas_lidar_install
colcon test-result --test-result-base /tmp/atlas_lidar_build/atlas_sensor_kit
```

Tests cover exact upstream field contracts, data bounds, header/metadata/bit-pattern
preservation, polar angles, intensity edge cases, offset validation/rounding,
ordering, absolute-double precision loss, and the pinned deskewer's UINT32 sum bound.
The integration test dynamically loads both actual registered component factories
and publishes a synthetic Livox cloud through their ROS topic interfaces with
intra-process communication disabled. It checks intermediate offsets and the final
compact header, values, metadata and exact layouts. Discovery and delivery each have
a five-second steady-clock deadline; CTest has a 30-second outer timeout. Localhost
domain 177 and test-only remappings isolate the synthetic graph. Identical messages
are retried every 100 ms until both outputs arrive to tolerate best-effort startup
loss. These tests do not validate DDS performance, hardware clocks, extrinsics, or
actual deskew/CenterPoint execution.
