# ICD Template

Copy to `ICD-<A>-<B>.md` (data-flow order) and add it to the ICD register in [System Architecture](System-Architecture). Anything already covered by [Architecture Conventions](Architecture-Conventions) (units, frames, QoS defaults, health) is not repeated - only deviations are listed.

---

# ICD-&lt;A&gt;-&lt;B&gt;: &lt;short title&gt;

**Status:** Draft | Agreed | Superseded · **Level:** L1 / L2 / L3 · **Owner:** &lt;nearest common parent&gt;
**Parties:** &lt;element A ID&gt; (provider) → &lt;element B ID&gt; (consumer)

## 1. Purpose

One paragraph: what flows across this interface and why.

## 2. Requirements

Requirements this interface exists to satisfy (IDs from [System Requirements](System-Requirements)).

## 3. Interface Items

One row per topic / service / action / TF / file.

| ID | Name | Kind | Type | Direction | Frame | Rate | QoS |
| --- | --- | --- | --- | --- | --- | --- | --- |
| &lt;ICD&gt;-01 | `/topic` | topic | `pkg/msg/Type` | A → B | `frame` | Hz | per Conventions §6 |

## 4. Item Details

For each item: field meanings and units that are not obvious, valid ranges, covariance expectations, what "no data" looks like.

## 5. Timing and Failure

- Timeout after which the consumer treats the item as stale, and what it does.
- Startup order assumptions (if any).

## 6. Verification

How each side shows it meets this ICD (test names), and which test exercises both sides together.

## 7. Change Log

| Date | Change | Agreed by |
| --- | --- | --- |
