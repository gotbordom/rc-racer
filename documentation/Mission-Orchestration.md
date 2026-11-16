# Mission Orchestration

**Status:** Draft · **Element:** VEH.EXE (Executive subsystem) · **Owner:** Anthony Tracy

How the vehicle's capabilities are combined into a mission. A mission is written in a small scripting language, a **behavior tree**, instead of being hard-coded, so a new course or strategy means editing a file rather than changing code.

## 1. Requirements

REQ-SYS-002 (mandatory cones in order), REQ-SYS-003 (bonus cones), REQ-SYS-011 (choose bonus cones for score), REQ-SYS-021 (pre-surveyed course map), REQ-SYS-024 (three attempts with reset).

## 2. Why Behavior Trees

| Option | Verdict |
| --- | --- |
| Hard-coded sequence in C++/Python | Every course change is a code change; recovery logic gets tangled |
| State machine (SMACH, FlexBE) | Works, but the number of transitions explodes as recovery cases are added |
| **Behavior tree** (BehaviorTree.CPP) | **Chosen.** Small reusable nodes; sequences, fallbacks and retries are built in; the tree re-checks conditions every tick, so it reacts to change; XML files with a visual editor and live monitor (Groot2); already used by Nav2 |

## 3. Concepts

A tree is **ticked** repeatedly from the root. Each node returns `SUCCESS`, `FAILURE` or `RUNNING`.

| Node | Behaviour | Used for |
| --- | --- | --- |
| `Sequence` | Runs children in order; fails at the first failure | "Do A, then B, then C" |
| `Fallback` | Tries children in order; succeeds at the first success | "Try A; if that fails, do B" |
| `RetryUntilSuccessful` | Re-runs its child up to N times | Retrying a cone approach |
| Condition (leaf) | Checks something, never `RUNNING` | `IsTimeRemaining`, `IsConeVisible` |
| Action (leaf) | Asks a capability to do something; `RUNNING` until it finishes | `NavigateToWaypoint`, `ApproachCone` |
| `SubTree` | A tree used as a node | `VisitCone` reused for every cone |

## 4. Architecture

```
GND.MPL                            VEH.EXE: mission_executive                VEH subsystems
course file (YAML) ──────────┐    ┌──────────────────────────────────┐
mission tree (XML) ──────────┴──> │ BehaviorTree.CPP engine           │
                                  │   leaf nodes = thin clients ──────┼──> actions/services (capabilities)
                                  │   blackboard: course, time, state │<── results, feedback, conditions
                                  └──────────────────────────────────┘
                                         │ mission status ──> TMC
                         FDR ── abort ──>│ (halts the tree; FDR always wins)
```

- **Leaf nodes contain no algorithms.** Each one is a client for one capability, with its ports mapped to the goal and result fields. All the real work stays in the subsystems, which are tested on their own.
- **The capability catalogue (§5) is the contract** between EXE and the rest of the vehicle. It becomes ICD-EXE-GNC and ICD-PER-EXE.
- **Safety is outside the tree.** FDR can stop the car and abort the mission at any time (ICD-FDR-EXE, not started); the tree cannot override it.

## 5. Capability Catalogue

Everything a mission can use. A capability that EXE must sequence ("do it and tell me when done or failed") is an **action** ([Conventions §7](Architecture-Conventions)).

| Capability | BT node | Kind | Provider | ICD | Status |
| --- | --- | --- | --- | --- | --- |
| Load course file into the blackboard | `LoadCourse` | built into EXE | VEH.EXE | - | ⏳ |
| Reset state estimate between attempts | `ResetEstimate` | service `~/reset` | VEH.GNC.SE | ICD-EXE-GNC | ⏳ ([state_estimation](state_estimation)) |
| Wait for the handler's start signal | `WaitForStart` | topic / service | VEH.TMC | ICD-TMC-EXE | ⏳ |
| Drive to a course waypoint (GPS-guided) | `NavigateToWaypoint` | action `NavigateToWaypoint` | VEH.GNC.PLN | ICD-EXE-GNC | ⏳ |
| Final visual approach and touch a cone at low speed (REQ-SYS-004) | `ApproachCone` | action `ApproachCone` | VEH.GNC.PLN using VEH.PER | ICD-EXE-GNC, ICD-PER-EXE | ⏳ |
| Is a cone in view? | `IsConeVisible` | condition on detections topic | VEH.PER.OD | ICD-PER-EXE | ⏳ |
| Is there enough time left for a bonus cone? | `IsTimeRemaining` | built into EXE | VEH.EXE | - | ⏳ |
| Is the vehicle healthy? | `IsHealthy` | condition on aggregated `/diagnostics` | VEH.FDR | ICD-FDR-EXE | ⏳ |
| Report mission progress | (engine) | topic | VEH.EXE → VEH.TMC | ICD-TMC-EXE | ⏳ |

Custom action and service types live in `rc_racer_interfaces`.

## 6. Example: RoboMagellan Mission

The practice course ([SDP §4.4](Software-Development-Plan)): mandatory M1 then M2, bonus B1 if time allows, then the finish.

```xml
<root BTCPP_format="4" main_tree_to_execute="RoboMagellan">

  <BehaviorTree ID="RoboMagellan">
    <Sequence>
      <LoadCourse file="{course_file}" course="{course}"/>
      <ResetEstimate/>
      <WaitForStart/>

      <SubTree ID="VisitCone" waypoint="M1" course="{course}"/>

      <!-- Bonus: only if time allows; giving up is not a mission failure -->
      <Fallback>
        <Sequence>
          <IsTimeRemaining min_seconds="300"/>
          <SubTree ID="VisitCone" waypoint="B1" course="{course}"/>
        </Sequence>
        <AlwaysSuccess/>
      </Fallback>

      <SubTree ID="VisitCone" waypoint="M2" course="{course}"/>
      <NavigateToWaypoint waypoint="finish" course="{course}"/>
    </Sequence>
  </BehaviorTree>

  <!-- Drive near the cone by GPS, then touch it using the camera; retry up to 3 times -->
  <BehaviorTree ID="VisitCone">
    <RetryUntilSuccessful num_attempts="3">
      <Sequence>
        <NavigateToWaypoint waypoint="{waypoint}" course="{course}" tolerance="3.0"/>
        <ApproachCone max_speed="0.3"/>
      </Sequence>
    </RetryUntilSuccessful>
  </BehaviorTree>

</root>
```

Mission trees can be written by hand, or generated by GND.MPL from the course file (mandatory cones in order, bonus cones chosen by expected score, REQ-SYS-011).

## 7. Verification

| Level | Test | Checks |
| --- | --- | --- |
| L4 | BT node unit tests | Each leaf maps ports ↔ goal/result correctly, against mock action servers |
| L3 | `mission_executive` component test | Example tree runs to completion with stub capabilities, including retries, bonus skip, and FDR abort |
| L1 | Mission system test in sim | Practice course mission run end to end; cones touched in order (ground truth) |

## 8. Open Items

| # | Question | Recommendation |
| --- | --- | --- |
| 1 | BehaviorTree.CPP v3 (what Nav2 on Humble uses) or v4 (current, works with Groot2)? | v4 if packaged for Humble; check at implementation time |
| 2 | The course file loader lives in `rc_racer_simulation/scripts/` today, but EXE and GND.MPL need it too | Move the course schema and loader to a shared package (e.g. `rc_racer_course`) before EXE is built |
| 3 | Use Nav2 for `NavigateToWaypoint` (planner, controller, costmaps) or write our own `planning_control`? | Decide in Phase 3; the capability contract stays the same either way |
