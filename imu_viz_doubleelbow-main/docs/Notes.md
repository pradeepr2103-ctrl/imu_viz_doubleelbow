## Quaternion Modes

The sensor delivers a raw orientation quaternion. The program computes a motion delta between the current reading and the calibration reference, then combines it with a neutral pose. The 4 modes change the order of multiplication and whether we use conjugate quaternions – this effectively swaps sensor axes or changes the rotation’s handedness.

- **Mode 1 (default)**: `delta = current * inverse(reference)`  
  Standard relative rotation (sensor‑to‑world). Works when the sensor axes align with the body axes.

- **Mode 2**: `delta = inverse(reference) * current`  
  Reverses multiplication order. Useful if the sensor is mounted “backwards” relative to the body.

- **Mode 3**: `delta = conjugate(current) * inverse(conjugate(reference))`  
  Conjugates (inverts) both quaternions before computing delta. Flips the sign of the vector part, effectively mirroring the rotation.

- **Mode 4**: `delta = inverse(conjugate(reference)) * conjugate(current)`  
  Combination of conjugation and order reversal. Provides the final axis remapping.

If the body part moves in the wrong direction (e.g., arm bends the wrong way), press **M** to cycle modes, then recalibrate. One mode will produce natural motion.

This is actually a really clever and practical solution to a common problem with IMU sensors! Let me break down what's happening and why you need these 4 modes.

---

## What Each Mode Does Mathematically

### Mode 1: `delta = current * inverse(reference)`
This is the **standard relative rotation** formula. 
- Reference = quaternion at calibration time (neutral pose)
- Current = live sensor reading
- The delta represents "how much has the sensor rotated since calibration"

**Works when:** Your sensor axes roughly match the body's natural axes.

### Mode 2: `delta = inverse(reference) * current`
This **reverses multiplication order**.
- Quaternion multiplication is not commutative (A*B ≠ B*A)
- Swapping order effectively rotates in the opposite direction or swaps which coordinate system is "dominant"

**Works when:** The sensor is mounted backwards (e.g., the "front" of the sensor points toward your body instead of away from it)

### Mode 3: `delta = conjugate(current) * inverse(conjugate(reference))`
Conjugate = `(w, -x, -y, -z)` - this **flips the axis of rotation**.
- If the sensor thinks you're rotating around X, conjugation makes it rotate around -X
- This is like looking at the rotation in a mirror

**Works when:** The sensor is mounted upside down or on the opposite limb (e.g., you put the left arm sensor on the right arm)

### Mode 4: `delta = inverse(conjugate(reference)) * conjugate(current)`
**Combines conjugation AND order reversal** - the nuclear option.
- Flips all axes AND reverses the rotation direction

**Works when:** Nothing else works - sensor is mounted in some weird orientation you can't easily describe

---

## Calibration Keys (C, V, B, N, etc.)
What they do: Tell the program "RIGHT NOW is the neutral position for this specific body part"

**The math**:
Store current sensor reading → Use as reference point (zero)
Example: When you press C with your arm down, the program says "Okay, this sensor angle = arm straight down"

**Then when you raise your arm**:
New angle - Stored angle = How much you moved
Apply that movement to the 3D model's arm