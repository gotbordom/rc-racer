# Vehicle state estimation in slides: reading list

Background for the `state_estimation` slide work: feature 2.5 `lateral-velocity`, VAL-018 (#11), spec §6.1. It covers the main ways a car's velocity is estimated when the tyres may slide, and where this project's design fits among them.

The summaries come from each paper's abstract and from search results, not from a full read. Check the papers themselves before relying on a detail.

## Read in this order

| # | Read | Why | Time |
|---|---|---|---|
| 1 | **Kong, Pfeiffer, Schildbach, Borrelli (2015)**, *Kinematic and Dynamic Vehicle Models for Autonomous Driving Control Design*, IEEE IV, [doi 10.1109/IVS.2015.7225830](https://doi.org/10.1109/IVS.2015.7225830) ([PDF](https://nuhuo08.github.io/control/IV_KinematicMPC_jason.pdf)) | The two baseline models side by side: the **kinematic bicycle**, which assumes no slip (what `state_estimation` used up to 2.4), and the **dynamic bicycle**, where slip comes from tyre forces. Gives the vocabulary (slip angle, cornering stiffness) and when each model is valid. Short. | 30 min |
| 2 | **A sideslip estimation review**: Lenzo et al., *Vehicle Sideslip Angle Estimation* ([PDF](https://shura.shu.ac.uk/26862/15/Lenzo_VehicleSideslipAngle%28VoR%29.pdf)), or Pieralice et al. (2018) ([PDF](https://shura.shu.ac.uk/24914/1/Pieralice2018sideslip.pdf)) | A map of the field: **kinematic** estimators (integrate the IMU, no vehicle parameters) against **dynamic** ones (tyre model), how they're combined, and the filters used (EKF, UKF, ...). Read the intro and the classification; skim the rest. | 1 h |
| 3 | **Yoon & Peng (2010)**, sideslip estimation with GPS and IMU, ASME DSCC ([PDF](https://huei.engin.umich.edu/wp-content/uploads/sites/186/2015/02/2010-DSCC-4249-Yoon-Peng-final.pdf)) | A **kinematic** estimator close to ours, and why it needs an outside velocity measurement (GPS there) to stop drifting. | 45 min |
| 4 | *An adaptive approach to real-time estimation of vehicle sideslip, road bank angles and sensor bias* ([arXiv 1905.08881](https://arxiv.org/pdf/1905.08881)) | The kinematic approach done more fully: it also estimates IMU bias and road bank. These are the practical error sources our design will meet on hardware. | 45 min |
| 5 | **Non-holonomic constraint (NHC)** in land navigation: *Versatile Land Navigation Using Inertial Sensors and Odometry* ([arXiv 1409.1078](https://arxiv.org/pdf/1409.1078)); also OdoNet ([arXiv 2109.03091](https://arxiv.org/pdf/2109.03091)) | Where our **no-slip pseudo-measurement** comes from: a land vehicle's sideways velocity is taken as almost zero and fed to the filter as a virtual measurement. Also detects slip by comparing inertial velocity with wheel odometry, as our windowed check does. | 1 h |
| 6 | **AMZ Driverless (ETH Zürich, Formula Student)**: *Redundant Perception and State Estimation for Reliable Autonomous Racing* ([arXiv 1809.10099](https://arxiv.org/pdf/1809.10099v1.pdf)); *AMZ Driverless: The Full Autonomous Racing System* ([arXiv 1905.05150](https://arxiv.org/pdf/1905.05150)) | What a winning racing team built: velocity estimated "using slip dynamics" in a 9-state EKF fusing six sensors. These include a direct ground-speed sensor and GNSS, so velocity is *measured* in a slide. Read the state estimation sections. | 1 h |

Optional, for a dynamic-model EKF: *Vehicle Sideslip Angle EKF Estimator based on Nonlinear Vehicle Dynamics Model and Stochastic Tire Forces Modeling*, SAE 2014-01-0144 ([doi](https://doi.org/10.4271%2F2014-01-0144)). See also Chen, *Sideslip angle estimation using extended Kalman filter* ([PDF](https://researchgate.net/profile/Bo-Chiuan-Chen/publication/245309332_Sideslip_angle_estimation_using_extended_Kalman_filter/links/54f6fd040cf28d6dec9c0654/Sideslip-angle-estimation-using-extended-Kalman-filter.pdf)).

## The four approaches, and ours

| Approach | Idea | Needs | Weakness |
|---|---|---|---|
| Kinematic bicycle (no slip) | The rear axle never moves sideways | Wheelbase, steering limit | Can't represent a slide |
| Dynamic bicycle (tyre model) | Slip from tyre forces (slip angle × cornering stiffness, or Pacejka) | Tyre parameters and road friction μ | μ and the tyres are unknown on dirt |
| Kinematic sideslip (IMU integration) | v̇x = ax + ω·vy, v̇y = ay − ω·vx | Just an IMU | **Drifts** without an outside velocity measurement |
| Non-holonomic constraint | "Sideways velocity ≈ 0" as a soft virtual measurement, relaxed when slipping | – | Wrong exactly while sliding |

**This project (2.5):** kinematic sideslip **plus** the non-holonomic constraint, held tight while gripping and loosened in a detected slide. It's a recognised combination. Its weakness is the third row's: in a slide the wheel is distrusted and the constraint is loose, so nothing measures velocity, and the forward-speed error left over from before detection persists (VAL-018 step 2 runs, 2026-11-24).

**Ways to close that gap with what the car has** (one rear-axle encoder, an IMU, no GPS):
- **Speed from the turn.** In a turn, lateral acceleration ≈ forward speed × yaw rate, so forward speed ≈ ay / ω. It's a steady-state assumption used as a weak measurement during a slide. This is our own engineering choice: it isn't presented this way in the papers above, so the scenarios are what prove it (VAL-004/018).
- **A tyre model** (row 2): estimate or calibrate μ and cornering stiffness.
- **A velocity sensor**: optical ground-speed or flow, or GPS. This is what the racing teams do.

## Terms

- **Slip angle (β):** the angle between where the car points and where it moves. In VAL-004/018's slides it's 10–17°.
- **Understeer / oversteer:** the car turns less or more than the steering asks. Ours understeers at μ 0.5.
- **Pseudo-measurement:** a constraint fed to the filter as if it were a sensor reading, with a variance that says how sure you are.
- **ZUPT:** zero-velocity update, "the car is stopped". Our pinned hold (VAL-019) is a cousin of it.
