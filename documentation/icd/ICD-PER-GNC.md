# ICD-PER-GNC: Perception to GNC

**Status:** Draft · **Level:** L2 · **Owner:** VEH (Vehicle segment)
**Parties:** VEH.PER (provider: `object_detection`) → VEH.GNC (consumers: `localization` for landmark fixes, `planning_control` for the cone approach), and VEH.EXE (`IsConeVisible`, until ICD-PER-EXE is written)

## 1. Purpose

What Perception sees, as positions relative to the car at the time of the camera frame. Perception reports **detections in one frame**: it does not track them over time, match them to course waypoints, or put them on a map. Those are GNC's (LOC, MAP) jobs, because they need the car's pose.

## 2. Requirements

REQ-PER-001 … REQ-PER-005, REQ-GNC-003 (landmark fixes), REQ-SYS-004 (cone approach).

## 3. Interface Items

| ID | Name | Kind | Type | Direction | Frame | Rate | QoS |
| --- | --- | --- | --- | --- | --- | --- | --- |
| PER-GNC-01 | `/perception/cones` | topic | `vision_msgs/msg/Detection3DArray` | PER → GNC, EXE | `base_link`, at the image's stamp | one per processed frame, ≥ TBD-010 Hz | Reliable, volatile, depth 5 |

## 4. Item Details

### PER-GNC-01 `/perception/cones`

One message for **every processed frame**, empty when no cone is seen, so "nothing in view" is distinguishable from "no data".

| Field | Content | Required |
| --- | --- | --- |
| `header.stamp` | The camera frame's stamp (capture time), not the processing time | Yes |
| `header.frame_id` | `base_link` | Yes |
| `detections[].results[0].hypothesis.class_id` | `cone` | Yes |
| `detections[].results[0].hypothesis.score` | Confidence 0–1 | Yes |
| `detections[].results[0].pose.pose.position` | Cone base centre, m, in `base_link`; z is the ground under the cone as seen | Yes |
| `detections[].results[0].pose.covariance` | x/y variance from the range and bearing errors (rotated into `base_link`); z, roll, pitch, yaw unused (≥ 1e6) | Yes |
| `detections[].bbox` | Cone size as seen, m; `bbox.center` = the position above | Yes |
| `detections[].id` | Empty: no tracking across frames | - |

**Partly visible cones** (REQ-PER-002): while the cone's top or bottom is outside the image, the range is uncertain but the bearing is still good. The covariance then grows along the line of sight, and `score` stays what it is: consumers should use the covariance, not drop the detection.

## 5. Timing and Failure

| Situation | PER behaviour | Consumer behaviour |
| --- | --- | --- |
| Camera stale (ICD-SNS-PER) | No messages | Treat as "unknown", not "no cone" |
| Processing slower than the camera | Drops frames, never queues: each message is the newest frame | Uses the stamp, not arrival time |

## 6. Verification

| Side | Test | Checks |
| --- | --- | --- |
| Provider | `object_detection` unit, component and sim tests ([spec §7](object_detection)) | Positions and covariances against known cones (synthetic images; course file + `/sim/truth/poses` in sim) |
| Consumer | `localization`, `planning_control` (Phase 3) | Run from recorded or stub detections that follow this ICD |

## 7. Change Log

| Date | Change | Agreed by |
| --- | --- | --- |
| 2026-11-28 | Initial draft (2.2) | |
