# ICD-SNS-PER: Sensing to Perception

**Status:** Draft · **Level:** L2 · **Owner:** VEH (Vehicle segment)
**Parties:** VEH.SNS (provider: `camera_driver`) → VEH.PER (consumer: `object_detection`)

In simulation, the provider side is implemented by TST.SIM ([ICD-VEH-TST](ICD-VEH-TST)): the Gazebo camera through the ROS-Gazebo bridge.

## 1. Purpose

Camera images, and the calibration needed to turn pixels into directions, from the car's camera to Perception. Sensing publishes images only: no detection, no cropping. It may rectify (undistort) the image, and if it does, `camera_info` must say so.

## 2. Requirements

REQ-PER-001 … REQ-PER-005, REQ-TST-001 (simulator equivalence).

## 3. Interface Items

| ID | Name | Kind | Type | Direction | Frame | Rate | QoS |
| --- | --- | --- | --- | --- | --- | --- | --- |
| SNS-PER-01 | `/camera` | topic | `sensor_msgs/msg/Image` | SNS → PER | `camera_optical_link` | ≥ 10 Hz (sim: 30 Hz) | Sensor data |
| SNS-PER-02 | `/camera_info` | topic | `sensor_msgs/msg/CameraInfo` | SNS → PER | `camera_optical_link` | with every image | Sensor data |
| SNS-PER-03 | camera mount transform | TF (static) | `tf2_msgs/msg/TFMessage` | URDF → PER | `base_link` → `camera_link` → `camera_optical_link` | static | Transient local |

## 4. Item Details

### SNS-PER-01 `/camera`

| Field | Content | Required |
| --- | --- | --- |
| `header.stamp` | Capture time of the frame (exposure middle), not the publish time | Yes |
| `header.frame_id` | `camera_optical_link` (Z forward, X right, Y down; [Conventions](Architecture-Conventions)) | Yes |
| `encoding` | `rgb8` or `bgr8` | Yes |
| `width`, `height` | Must match `camera_info` | Yes |

### SNS-PER-02 `/camera_info`

| Field | Content | Required |
| --- | --- | --- |
| `header.stamp` | Same stamp as the image it describes | Yes |
| `k` | Intrinsics (fx, fy, cx, cy) in pixels | Yes |
| `distortion_model`, `d` | `plumb_bob` or `equidistant` (fisheye) and its coefficients; all zero for a rectified or ideal image | Yes |

The real IMX219 lenses are wide (200° in use, 105° low-distortion in testing; [Rc-Car-Design](Rc-Car-Design)), so the real `camera_info` will carry a fisheye model. The sim camera is an ideal pinhole: 640 × 480, 60° horizontal field of view, no distortion (DEV-08 in [ICD-VEH-TST](ICD-VEH-TST)).

## 5. Timing and Failure

| Item | Stale after | PER behaviour when stale |
| --- | --- | --- |
| `/camera` | 0.5 s | WARN; publishes no detections (an empty list would claim "no cone in view") |
| `/camera_info` | never received | ERROR; cannot turn pixels into directions, publishes nothing |

No startup order is assumed.

## 6. Verification

| Side | Test | Checks |
| --- | --- | --- |
| Provider (sim) | `test_simulation_smoke` | `/camera` publishes with `frame_id` in the TF tree (`test_sensor_frame_ids`) |
| Provider (hardware) | `camera_driver`, Phase 5 | Calibration (`camera_info`) checked against a target |
| Consumer | `object_detection` component tests | Runs from synthetic images and `camera_info` that follow this ICD, including stale and missing `camera_info` |

## 7. Change Log

| Date | Change | Agreed by |
| --- | --- | --- |
| 2026-11-28 | Initial draft (2.2) | |
