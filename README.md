# embedded-quaternion-f32

[![Crates.io](https://img.shields.io/crates/v/embedded-quaternion-f32.svg)](https://crates.io/crates/embedded-quaternion-f32)
[![Downloads](https://img.shields.io/crates/d/embedded-quaternion-f32.svg)](https://crates.io/crates/embedded-quaternion-f32)
[![docs.rs](https://img.shields.io/docsrs/embedded-quaternion-f32)](https://docs.rs/embedded-quaternion-f32)
[![License](https://img.shields.io/crates/l/embedded-quaternion-f32.svg)](LICENSE)

`no_std` `f32` quaternions for embedded 4D rotations: Hamilton product,
rotations in the X-W and Y-Z planes, and 4D → 3D → 2D perspective projection.
Designed for visualizing 4D objects (tesseract, hypersphere, etc.) on
microcontrollers (OLED, TFT), without allocation or `libm`.

## Installation

```toml
[dependencies]
embedded-quaternion-f32 = "0.4"
```
## 📖 Key functions

| Function | Description | Returns |
|---|---|---|
| `Quaternion::new(w, x, y, z)` | Constructs a quaternion | `Quaternion` |
| `Quaternion::ZERO` / `Quaternion::ONE` | Constants: zero / Hamilton's unit | `Quaternion` |
| `from_angle_plane(angle)` | Computes `(sin, cos)` for an angle | `(f32, f32)` |
| `rotate_xw(angle)` | 4D rotation in the X-W plane | `Quaternion` |
| `rotate_yz(angle)` | 4D rotation in the Y-Z plane | `Quaternion` |
| `conjugate()` | Conjugate `q* = w - xi - yj - zk` | `Quaternion` |
| `norm_squared()` | Squared norm (without `sqrt`) | `f32` |
| `project_2d(cx, cy, dist_4d, dist_3d)` | 4D → 3D → 2D perspective projection | `(i16, i16, f32)` |
| `+`, `-`, `*` (between quaternions) | Addition, subtraction, Hamilton product | `Quaternion` |
| `*` (by an `f32`) | Scalar multiplication | `Quaternion` |
| `normalize()` | Normalizes the quaternion (makes it unit length) | `Option<Quaternion>` |

## 💻 Examples

### Construction and constants

```rust
use embedded_quaternion_f32::Quaternion;

let q = Quaternion::new(1.0, 0.0, 0.0, 0.0);
assert_eq!(q, Quaternion::ONE);

```
### Quaternion normalization

```rust
use embedded_quaternion_f32::Quaternion;

let q = Quaternion::new(1.0, 2.0, 3.0, 4.0);

// Normalization (makes the quaternion unit length)
if let Some(normalized) = q.normalize() {
    assert!((normalized.norm_squared() - 1.0).abs() < 1e-4);
}

// The zero quaternion cannot be normalized
assert!(Quaternion::ZERO.normalize().is_none());
```


### 4D rotation (X-W plane) and screen projection

```rust
use embedded_quaternion_f32::Quaternion;
use core::f32::consts::FRAC_PI_2;

// Pure point on the X axis
let p = Quaternion::new(0.0, 1.0, 0.0, 0.0);
let rotated = p.rotate_xw(FRAC_PI_2);

// Projection onto a 320x160 screen, with camera distance 5 in both 4D and 3D
let (screen_x, screen_y, depth) = rotated.project_2d(160.0, 80.0, 5.0, 5.0);
```

### Rotation in the Y-Z plane

```rust
use embedded_quaternion_f32::Quaternion;

let p = Quaternion::new(0.0, 0.0, 1.0, 0.0);
let rotated = p.rotate_yz(core::f32::consts::PI);
```

### Hamilton product and conjugate

```rust
use embedded_quaternion_f32::Quaternion;

let a = Quaternion::new(1.0, 2.0, 3.0, 4.0);
let b = Quaternion::new(5.0, 6.0, 7.0, 8.0);

let product = a * b;          // Hamilton product (noncommutative: a*b != b*a)
let conj = a.conjugate();     // w - xi - yj - zk
let dist2 = a.norm_squared(); // squared norm, without sqrt
```

### Basic arithmetic

```rust
use embedded_quaternion_f32::Quaternion;

let a = Quaternion::new(1.0, 2.0, 3.0, 4.0);
let b = Quaternion::new(0.5, 0.5, 0.5, 0.5);

let sum = a + b;
let diff = a - b;
let scaled = a * 2.0; // multiplication by an f32 scalar
```

## Dependencies

This crate relies on [`embedded-trig-f32`](https://crates.io/crates/embedded-trig-f32)
for `sin`/`cos`/`sincos` and [`embedded-f32-sqrt`](https://crates.io/crates/embedded-f32-sqrt)
for the optimized square root. Adjust the version in `Cargo.toml` if you
use a local dependency (`path = ".."`) during development.

> **Precision**: `embedded-trig-f32` provides fast approximations
> (Bhaskara / McCutcheon), not exact `libm` calculations. The identity
> `sin² + cos² = 1` is guaranteed only within ~2·10⁻³. For rotations
> composed over many iterations, consider periodically renormalizing
> the quaternion if drift becomes noticeable.

## 🧪 Tests

```bash
cargo test
```

## 🛡️ License

Licensed under either of:

    MIT license (http://opensource.org/licenses/MIT)

    Apache License, Version 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

at your option.

## 🦅 About

Developed by Jorge Andre Castro.
