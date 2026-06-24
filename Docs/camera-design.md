# Dynamic Camera System Design

The first piece of **Milestone 5 — Game Layer**. A runtime camera that frames a
set of targets (the players in a local-multiplayer brawler), pulls back as they
spread apart, and avoids clipping through world geometry. Built for the
Gang Beasts–style target on the roadmap, but it is plain gameplay code — a
script component + system in `Sandbox/src/Scripts/`, nothing engine-side.

---

## The core idea

The game camera is **just an entity** with a `CameraComponent` (fov / near / far)
and a `TransformComponent`; the renderer takes `inverse(worldMatrix)` of the
primary camera as the view (`main.cpp`). So "dynamic camera" means **one system
that writes that entity's transform every frame** — exactly the native-script
model (`OnUpdate(dt)`) every other gameplay system already uses.

The model mirrors the **editor orbit camera** (`Core/Camera.h`): a **pivot** the
camera looks at, a **distance** it sits back, and a framerate-independent
exponential ease (`1 - exp(-smoothing·dt)`) toward target values. The game camera
is that same model where:

- **Pivot = centroid of the targets**, eased.
- **Distance = derived from how spread out the targets are** (frustum-fit), eased.

Multi-target tracking and zoom-out are therefore **the same calculation** — both
fall out of the targets' positions and the camera's frustum.

The orientation is **fixed** (authored pitch/yaw) rather than orbiting. For a
brawler this is the right call and it collapses the hard parts: the framing math
reduces to a spread projected onto two fixed screen axes, and collision becomes a
single fixed-direction sweep.

---

## Goals (v1)

- A `CameraTargetComponent` **tag** — attach to each player; the director frames
  everything wearing it. No hand-authored entity lists.
- A `CameraDirectorComponent` (plain data) on the camera entity holding the fixed
  framing angle, distance clamps, framing margin, smoothing, and collision knobs.
- A `CameraDirectorSystem` (priority **450**, the camera band — after physics 200
  and gameplay 300) that each frame:
  1. Computes the **centroid** of all targets → pivot (+ a height offset).
  2. Derives the **frustum-fit distance** from the targets' spread.
  3. **Eases** pivot + distance toward those targets.
  4. **Spherecasts** pivot → desired camera position and pulls in on a hit.
  5. Writes the camera entity's `TransformComponent` (position + look rotation).

### Out of scope for v1
- Orbiting / player-controlled yaw, and auto-yaw-to-action (see roadmap question —
  deliberately chose **fixed angle**).
- Player-mesh fade when the camera gets close — deferred until play shows it's
  actually needed (a pulled-back brawler camera rarely clips a player).
- Soft "dead zones" / look-ahead / target weighting beyond a simple centroid.
- Multiple cameras / split-screen.

---

## Why these choices

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Orientation | **Fixed** pitch/yaw, only pivot + distance move | Framing collapses to one screen plane; collision is one stable sweep direction; no disorientation in screen-relative local co-op. |
| Target set | **Tag component** (`CameraTargetComponent`), auto-collected | Editor-attachable, serialized for free, no entity-list UI; players just wear the tag. |
| Pivot | **Centroid** of targets, eased | Keeps the group centered; cheap and stable. Weighting is a fast-follow. |
| Distance | **Frustum-fit** from spread, not raw pairwise distance | Guarantees everyone actually fits the view; ties zoom to fov so it's correct at any aspect. |
| Smoothing | Reuse the editor camera's `1 - exp(-s·dt)` ease | Framerate-independent, already proven in the codebase. |
| Distance deadzone | Small hysteresis before re-targeting distance | Stops the camera "breathing" in/out on tiny player jostling. |
| Collision | **SphereCast** from pivot, not a thin ray | A ray slips past wall edges that a near-plane corner still clips; a sphere ~near-plane-sized doesn't. |
| Collision easing | **Snap in, ease out** (asymmetric) | Hide a wall instantly; never pop outward the frame an obstruction clears. |

---

## Framing math (fixed angle)

Given fixed `pitchDeg` / `yawDeg`, build the camera basis:

```
Front = normalize( cos(pitch)·sin(yaw),  -sin(pitch),  -cos(pitch)·cos(yaw) )
Right = normalize( cross(Front, worldUp) )
Up    = normalize( cross(Right, Front) )
```

(yaw 0, pitch 0 → looks down −Z; positive pitch tilts the view downward.)

Pivot = centroid of target positions + `worldUp · pivotHeightOffset`.

Project each target's offset from the pivot onto the screen axes and fit to the
frustum half-angles (`vHalf = fov/2`, `hHalf = atan(aspect · tan(vHalf))`):

```
halfW = max over targets of |dot(pos − pivot, Right)|
halfH = max over targets of |dot(pos − pivot, Up)|
distance = max( halfW / tan(hHalf),  halfH / tan(vHalf) ) · framingMargin
distance = clamp(distance, minDistance, maxDistance)
```

Camera sits at `pivot − Front · distance`, oriented to look along `Front`.
(`aspect` is a component field, default 16:9, so the system needn't be plumbed the
live viewport size.)

---

## Collision avoidance

After easing pivot + distance, sweep from the pivot toward the camera seat:

```
HitResult h = Physics::SphereCast(pivot, collisionRadius, −Front, distance, cameraEntity);
if (h.hit) collidedDist = max(h.distance − collisionSkin, minClearance);
```

Then apply the seat distance with **asymmetric easing**: if the collided distance
is *closer* than the current seat, snap to it immediately; otherwise ease back out
at `recoverSmoothing`. The final position is `pivot − Front · seatDist`.

`SphereCast` takes a single `ignore` entity (the camera). Players between pivot and
seat could still register, but a pitched-back camera casts up-and-behind, so this
is rare in practice; multi-ignore is a fast-follow if it bites.

---

## Editor / usage

1. Attach `CameraDirector` to the primary camera entity.
2. Attach `CameraTarget` to each player entity.
3. Press Play — the camera frames and follows the group.

All tunables (angle, clamps, margin, smoothing, collision) are inspector fields and
serialize with the scene, same as every other script component.
