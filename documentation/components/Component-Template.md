# Component Template

Copy to `<component>.md` (the package name) before writing the component's code. A component spec says what the component **promises**; the ICDs say what crosses each interface. Write the spec first, then the code, then tests that check the spec.

---

# &lt;component&gt;

**Element ID:** VEH.&lt;SUB&gt;.&lt;CODE&gt; · **Package:** `&lt;component&gt;` · **Status:** Draft | Implemented | Verified

## 1. Purpose

One paragraph. What it does and, just as important, what it does **not** do (see the responsibilities table in [System Architecture](System-Architecture)).

## 2. Requirements

| Requirement | How this component meets it |
| --- | --- |

## 3. Interfaces

### 3.1 Inputs

| Name | Type | ICD | Required? | Stale after |
| --- | --- | --- | --- | --- |

### 3.2 Outputs

| Name | Type | ICD | Rate | Notes |
| --- | --- | --- | --- | --- |

### 3.3 Actions / Services offered

| Name | Type | ICD | Behaviour |
| --- | --- | --- | --- |

### 3.4 Parameters

| Name | Type | Default | Meaning |
| --- | --- | --- | --- |

## 4. Behaviour

- Lifecycle: what happens on configure / activate / deactivate.
- Main algorithm (link to design notes or diagrams).
- Degraded modes: what it does when each input is missing or bad.

## 5. Health

What it reports on `/diagnostics` and when it reports WARN / ERROR.

## 6. Internal Design

Logic class(es) and the node adapter ([SDP §2](Software-Development-Plan)). Diagrams if useful.

## 7. Verification

| Level | Test | Checks |
| --- | --- | --- |
| L4 unit | `test_<component>_logic` | |
| L3 component | `test_<component>_node` | |
| L2 subsystem | | |

## 8. Open Items

| # | Question | Options | Recommendation |
| --- | --- | --- | --- |
