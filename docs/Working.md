## Step 1: What does the sensor actually give you?

The MPU6050 has a gyroscope + accelerometer. Its onboard DMP chip fuses them and outputs a **quaternion** — basically a compact way to describe "which direction is this sensor currently pointing in 3D space."

Think of it as: the sensor tells you its **absolute orientation** in the world at every moment.

A quaternion is just 4 numbers: `(w, x, y, z)`. Don't stress the math yet — just know it encodes a rotation/orientation.

---

## Step 2: Arduino sends it over WiFi

```
L_FA, 0.707, -0.707, 0.000, 0.000
```

Label + 4 floats. UDP multicast so multiple laptops can receive simultaneously. The receiver thread on your PC just parks these values into `SensorManager`.

---

## Step 3: The Calibration Problem

Here's the first real problem.

The sensor gives you its **absolute** orientation — meaning it's relative to the Earth's magnetic/gravity field, not your body. If you just strap it to your arm, it'll tell you "I'm pointing north-east at 30 degrees" — which is useless. You want "the arm is raised 45 degrees from rest."

**Calibration fixes this.**

When you press `C` (calibrate left forearm), you stand in neutral pose (arms hanging down). The code stores the sensor's current reading as `calibrationReference`.

Now every future reading is interpreted as: **"how much has the sensor moved since I pressed C?"**

That "how much has it moved" is called the **motion delta**.

---

## Step 4: Computing the Motion Delta (The 4 Modes)

You have:
- `q_ref` = what the sensor read when you calibrated (neutral pose)
- `q_current` = what the sensor reads right now

You want: **"how much did it rotate since calibration?"**

In quaternion math, to "undo" a rotation `A` and find what's left from `B`, you compute `B * inverse(A)`.

Think of it like numbers: if reference = 5 and current = 8, the delta = 8 - 5 = 3. Quaternions don't subtract, they **divide** (multiply by inverse). So delta = current × inverse(reference).

That's **Mode 1**.

The other 3 modes are just different orderings/mirrorings of this formula — needed because the sensor might be physically mounted in different orientations on different body parts (flipped, rotated, etc.). Each mode remaps axes differently. You just try them until the limb moves correctly.

---

## Step 5: Neutral Pose + Axis Flip

After getting the delta, two more corrections happen:

**Neutral Pose (180° Y rotation):**

The Mixamo skeleton model's bones, in their default rest state, point in a specific direction. The sensor's "identity" (zero rotation) points in a different direction. You multiply by a fixed 180° rotation to align them. It's a one-time coordinate system alignment.

**Axis flip (negate X and Z):**

OpenGL's coordinate system is right-handed. The MPU's output is in a different handedness. Negating X and Z on the quaternion is like flipping a glove inside-out — you're correcting the handedness mismatch. Without this, movements would be mirrored or inverted.

---

## Step 6: Smoothing

Raw sensor data is noisy. If you directly apply it to the model, the character jitters.

The code uses **SLERP** — spherical interpolation between the previous frame's rotation and the new one.

Instead of snapping to the new value, it slides toward it:

```
smoothed = 35% new + 65% old   (slow movement)
smoothed = 70% new + 30% old   (fast movement)
```

It auto-detects speed by checking how far the new quaternion is from the old one. Fast motion → respond quicker. Slow motion → filter more aggressively.

There's also a **deadband**: if the change is tiny (< 0.003 radians, barely a fraction of a degree), ignore it completely. This kills the micro-jitter when you're holding still.

---

## Step 7: Drift Correction

Gyroscopes drift. Even if you hold perfectly still, after a few minutes the sensor thinks you've rotated slightly. This is a hardware limitation.

The code watches for when you're stationary for 1.5 seconds straight. When that happens, it very slowly nudges the calibration reference toward your current reading (0.2% per frame). 

This silently cancels drift without you noticing or needing to recalibrate.

---

## Step 8: Applying to the 3D Model — The Tricky Part

Now you have a clean, smooth, calibrated quaternion for each body part. How do you actually move the 3D model?

The model has a skeleton — a hierarchy of bones. Hip → Spine → Chest → Shoulder → Upper Arm → Forearm → Hand, etc.

Each bone has a **local rotation** (relative to its parent). To pose the model, you set local rotations, then compute where every bone ends up in world space by multiplying down the chain:

```
world position of hand = hip × spine × chest × shoulder × upper arm × forearm
```

**The problem**: your sensor gives you a world-space direction. The bone needs a local rotation. You have to convert.

Here's how it works for, say, the left upper arm:

1. Take the corrected sensor quaternion for L_UA
2. Rotate the vector `(0, -1, -0.2)` by it — this gives the world-space direction the arm should point (downward with slight forward lean = natural hang)
3. Ask: what local rotation does the upper arm bone need so it points in that direction?
4. To find that: undo the parent bone's rotation (shoulder/chest), then find the minimal rotation that swings the arm from its bind-pose direction to your target direction
5. Set that as the bone's local rotation

Then recompute all global matrices down the chain, and do the same for the forearm, shin, etc.

---

## Step 9: Skinning — Making the Mesh Follow the Bones

The skeleton moves, but what you actually see is the mesh (the skin). Each vertex of the mesh is influenced by 1-4 bones with weights.

For example, a vertex on the elbow might be:
- 60% influenced by the forearm bone
- 40% influenced by the upper arm bone

Every frame, each vertex position is computed as:

```
final position = 0.6 × (forearm bone transform × vertex) 
              + 0.4 × (upper arm bone transform × vertex)
```

This is why the mesh stretches smoothly at joints instead of cracking apart.

---

## The Full Picture in Plain English

```
Sensor on arm moves
    ↓
MPU6050 DMP computes absolute orientation quaternion
    ↓
Sent over WiFi UDP to your PC
    ↓
Normalized + sign-fixed to avoid flipping artifacts
    ↓
Subtract calibration reference → "how much did you move?"
    ↓
Apply coordinate system corrections (neutral pose + axis flip)
    ↓
Smooth with adaptive SLERP (reduce noise, preserve responsiveness)
    ↓
Drift correction if stationary for 1.5s
    ↓
Convert sensor direction → bone local rotation
    ↓
Propagate down skeleton hierarchy (FK)
    ↓
Each mesh vertex weighted-averages bone transforms
    ↓
glVertex3f → you see the character move
```

---

## The process

**1. Figure out where your arm is pointing in world space**

Take the sensor quaternion. Rotate the default arm direction `(0,-1,0)` by it.

```
armDirection = sensorQuat * (0,-1,0) * sensorQuat_inverse
```

Now `armDirection` is a 3D vector like `(-0.7, -0.7, 0)` — this is where your physical arm is pointing right now in the real world.

---

**2. The bone can't use world space — it needs local space**

The bone lives inside the hierarchy. Its rotation is always relative to its parent (the chest).

So you ask: what does `armDirection` look like from the chest's point of view?

```
armDirection_local = chestRotation_inverse * armDirection
```

Now `armDirection_local` is the same direction but expressed relative to the chest.

---

**3. Find what rotation achieves this**

The bone currently points in some direction `currentDir` (from the GLTF bind pose data).

You want it to point in `armDirection_local`.

Find the rotation that goes from `currentDir` to `armDirection_local`:

```
angle = acos( dot(currentDir, armDirection_local) )
axis  = cross(currentDir, armDirection_local)
delta = quaternion built from angle and axis
```

---

**4. Set it on the bone**

```
bone.localRotation = delta * bindRotation
```

---

**5. Recompute the whole hierarchy**

Walk from root to leaves, multiply matrices down the chain. Every bone now knows its world position.

---

**6. Do the same for forearm, thighs, shins**

Same 5 steps repeated per bone. Always parent before child.