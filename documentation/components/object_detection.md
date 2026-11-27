# object_detection

**Element ID:** VEH.PER.OD · **Package:** `object_detection` · **Status:** Draft

## 1. Purpose

Finds 18" orange traffic cones in each camera frame and reports where they are relative to the car (`base_link`), with the uncertainty of each position.

It does **not**:
- track cones from frame to frame, or decide which cone is which course waypoint (that needs the car's pose: GNC, `localization`);
- put cones or obstacles on a map (`mapping`);
- decide whether or how to approach a cone (PLN, EXE);
- detect obstacles, until Open Item 1 is decided.

## 2. Requirements

| Requirement | How this component meets it |
| --- | --- |
| REQ-PER-001 | Colour segmentation finds orange blobs and keeps the cone-shaped ones (§4); detection rate graded in sim against the cones' true positions |
| REQ-PER-002 | Bearing from the blob's horizontal centre, which stays valid while the top or bottom is cut off; the range falls back to the ground-plane estimate (§4) |
| REQ-PER-003 | Range from the cone's known height (or the ground plane), bearing from the intrinsics, both rotated into `base_link` with their covariance |
| REQ-PER-004 | Shape and size checks reject orange things that aren't cone-shaped; graded in sim with the course's obstacles in view |
| REQ-PER-005 | Pure C++/OpenCV on a downscaled image; always processes the newest frame, never queues; timed on the Jetson Nano (Phase 5) |
| REQ-PER-006 | Not yet: Open Item 1 |

## 3. Interfaces

### 3.1 Inputs

| Name | Type | ICD | Required? | Stale after |
| --- | --- | --- | --- | --- |
| `/camera` | `sensor_msgs/msg/Image` | [ICD-SNS-PER](ICD-SNS-PER) SNS-PER-01 | Yes | 0.5 s |
| `/camera_info` | `sensor_msgs/msg/CameraInfo` | [ICD-SNS-PER](ICD-SNS-PER) SNS-PER-02 | Yes | - (latest kept) |
| Camera mount transform | TF | [ICD-SNS-PER](ICD-SNS-PER) SNS-PER-03 | Yes | - |

### 3.2 Outputs

| Name | Type | ICD | Rate | Notes |
| --- | --- | --- | --- | --- |
| `/perception/cones` | `vision_msgs/msg/Detection3DArray` | [ICD-PER-GNC](ICD-PER-GNC) PER-GNC-01 | per processed frame | `base_link`, at the image's stamp; empty when none seen |
| `~/debug_image` | `sensor_msgs/msg/Image` | debug only | per processed frame | Only when `debug_image: true`: the frame with the mask and detections drawn on |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | [Conventions](Architecture-Conventions) | 1 Hz | §5 |

### 3.3 Actions / Services offered

None. Lifecycle transitions only.

### 3.4 Parameters

All TBD: tuned in sim first, then on real footage (DEV-08).

| Name | Type | Default | Meaning |
| --- | --- | --- | --- |
| `process_width` | int | 640 | The image is scaled to this width before processing (the real camera is 3280 wide) |
| `max_rate` | double | 15.0 | Hz: frames beyond this are skipped |
| `color.hue_min`, `color.hue_max` | int | 0, 22 | Orange hue band (OpenCV 0–179); hue wraps, so a min above max means "wrap through red" |
| `color.saturation_min`, `color.value_min` | int | 120, 80 | Reject washed-out and dark pixels |
| `shape.min_height_px` | int | 8 | Blobs shorter than this are ignored (a cone at ~30 m in the sim camera) |
| `shape.aspect_min`, `shape.aspect_max` | double | 0.9, 2.5 | Height / width of a cone's bounding box |
| `shape.fill_min`, `shape.fill_max` | double | 0.35, 0.8 | Blob area / bounding box area: a cone is a triangle-ish shape, about 0.5 |
| `cone.height` | double | 0.457 | m, the cone's height (18") |
| `cone.base_width` | double | 0.27 | m, the base plate's width |
| `range.pixel_stddev` | double | 1.0 | px, 1σ of the blob's edges, for the covariance |
| `range.max` | double | 30.0 | m: detections beyond this are dropped |
| `debug_image` | bool | false | Publish `~/debug_image` |

## 4. Behaviour

- **Lifecycle:** `configure` validates parameters and builds the logic object. `activate` starts the subscriptions and publishers. `deactivate` stops publishing. `cleanup` drops state. The same pattern as `state_estimation`.
- **Per frame** (recommended method, Open Item 2):
  1. **Scale** to `process_width`, and **undistort** with `camera_info` if it has distortion (none in the sim).
  2. **Colour mask:** HSV threshold for orange, then a morphological open and close to remove specks and fill the cone's stripes.
  3. **Blobs:** connected components of the mask, kept if tall enough and cone-shaped (aspect and fill ratio, §3.4). A blob touching the image's top or bottom edge is kept, but marked **clipped**.
  4. **Bearing** from the blob's horizontal centre through the intrinsics: valid even when clipped.
  5. **Range**, Open Item 3:
     - **from the cone's height** when the whole cone is in view: `r = fy · cone.height / h_px`. It doesn't depend on the ground being flat;
     - **from the ground plane** when the top is clipped: the ray through the blob's bottom edge meets the ground, using the camera's height and pitch from TF. It assumes flat ground, so its covariance is larger;
     - **none** when the bottom is clipped too (closer than about 0.4 m): bearing only, with a large range variance.
  6. **Position and covariance:** range and bearing to x, y in `camera_link`, then into `base_link` by TF. The range variance comes from `range.pixel_stddev` (`σ_r ≈ r² · σ_px / (fy · H)`, so about 4 % at 10 m in the sim camera), the bearing variance from one pixel; both are rotated into x/y.
  7. **Score:** how cone-like the blob is (fill and aspect near a cone's), 0–1.
- **No tracking:** each frame is independent (Open Item 5).
- **What the sim camera can see** (640 × 480, 60° horizontal, at 0.20 m height pitched down 0.1 rad, fy ≈ 554 px): a cone is 25 px tall at 10 m and 13 px at 20 m. The whole cone is in view from about 0.8 m from the camera; closer than that its top is cut off, and closer than about 0.4 m its bottom too. The bumper (0.02 m ahead of the camera) touches the cone's base when its centre is about 0.16 m from the camera, so the last ~0.2 m of the approach is bearing-only (REQ-PER-002).
- **Degraded modes:**

| Situation | Behaviour | Diagnostics |
| --- | --- | --- |
| `/camera` stale | Publishes nothing (not an empty list) | WARN |
| No `camera_info` yet | Publishes nothing | ERROR |
| No camera TF yet | Publishes nothing | ERROR |
| Processing slower than `max_rate` | Skips to the newest frame | WARN when it can't keep TBD-010 |

## 5. Health

`/diagnostics` status `object_detection` (1 Hz, `diagnostic_updater`), ranked ERROR > STALE > WARN > OK like `state_estimation`:

| Level | When |
| --- | --- |
| STALE | Inactive, or no image yet |
| ERROR | No `camera_info` or no camera TF |
| WARN | Camera stale; processing below TBD-010 Hz or above its latency |
| OK | Otherwise |

Key values: `image_age`, `processed_rate`, `processing_time_ms`, `cones_last_frame`, `frames_skipped`.

## 6. Internal Design

Same split as `state_estimation` ([SDP §2](Software-Development-Plan)):
- **`ConeDetector`** (pure C++ and OpenCV, no ROS): image and intrinsics in, a list of `ConeDetection{bearing, range, clipped, covariance, score, box}` in `camera_link`. Each step of §4 is a separate function, unit-tested on its own.
- **`ConeGeometry`**: range from the height, range from the ground plane, and the covariance, from the intrinsics and the mount. Pure maths, separately testable.
- **`object_detection_node`** (lifecycle node): `cv_bridge` in, TF to `base_link`, `vision_msgs` out, diagnostics.

C++ rather than Python, for the Jetson Nano's CPU (REQ-PER-005); OpenCV 4.5 is in the dev image already.

## 7. Verification

| Level | Test | Checks |
| --- | --- | --- |
| L4 unit | `test_cone_geometry` | Range from height and from the ground plane against hand-worked cases; covariance grows as r²; bearing sign and scale |
| L4 unit | `test_cone_detector` | Synthetic images (an orange cone shape drawn at a known size and place on grass/grey): found with the right bearing and range; clipped top / bottom; rejected: brown obstacle, small orange speck, orange slab of the wrong shape; hue wrap |
| L3 component | `test_object_detection_node` | Synthetic `/camera` and `/camera_info` → `/perception/cones` in `base_link` at the image's stamp; empty list when nothing is seen; nothing published when stale or without `camera_info`; diagnostics |
| L2 subsystem | `test_per_cone_detection_sim` (VAL-020..) | Drive the practice course past its cones; compare with the true cone positions (course file and `/sim/truth/poses`, since cones are dynamic): detection rate inside range and view (TBD-007), range and bearing errors and their 3σ consistency (TBD-008), false detections with the obstacles in view (TBD-009), the final approach to a cone (REQ-PER-002) |

## 8. Open Items

| # | Question | Options | Recommendation |
| --- | --- | --- | --- |
| 1 | **Scope:** obstacles in 2.2? (REQ-SYS-007/008 → REQ-PER-006) | (a) cones only; obstacles as a later feature. (b) cones and obstacles now. (c) obstacles from another sensor (ultrasonic, depth camera): a hardware choice | **(a)**: cones are a known colour and shape, obstacles aren't; detecting arbitrary obstacles from one camera is a different and much harder problem, and the sensor choice for it is open |
| 2 | **Detection method** | (a) colour segmentation and shape checks. (b) a learned detector (small YOLO, TensorRT on the Nano). (c) (a) now, (b) later as a verifier or replacement once there is real footage | **(c)**: (a) needs no training data and runs from day one in the sim; (b) needs labelled images, best from the real camera |
| 3 | **Range method** | (a) from the cone's known height. (b) from the ground plane (bottom edge). (c) (a) when whole, (b) when the top is clipped | **(c)**: (a) works on slopes but fails in the final approach when the top is cut off; (b) covers that but assumes flat ground |
| 4 | **Sim camera vs the real lens** (DEV-08) | (a) keep the sim's ideal 60° pinhole for now; model the real lens when it's chosen. (b) change the sim now to the 105° low-distortion lens | **(a)**: the lens is still in testing; the detector undistorts from `camera_info` either way, so the code doesn't change, only the tuning |
| 5 | **Tracking** across frames | (a) none in PER; consumers filter. (b) a simple tracker in PER (ids, smoothing) | **(a)**: LOC and the approach controller need the pose to associate cones anyway |
| 6 | **The architecture's "stereo cameras"** (System Architecture, CAM) | (a) correct it to one camera, the IMX219. (b) keep it, if stereo is planned | Owner: the hardware list has one camera in use and one in testing |
