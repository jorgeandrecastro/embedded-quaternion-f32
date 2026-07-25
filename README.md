# embedded-quaternion-f32

[![Crates.io](https://img.shields.io/crates/v/embedded-quaternion-f32.svg)](https://crates.io/crates/embedded-quaternion-f32)
[![Downloads](https://img.shields.io/crates/d/embedded-quaternion-f32.svg)](https://crates.io/crates/embedded-quaternion-f32)
[![docs.rs](https://img.shields.io/docsrs/embedded-quaternion-f32)](https://docs.rs/embedded-quaternion-f32)
[![License](https://img.shields.io/crates/l/embedded-quaternion-f32.svg)](LICENSE)

Quaternions `f32` `no_std` pour rotations 4D en embarqué : produit de Hamilton,
rotations dans les plans X-W et Y-Z, et projection perspective 4D → 3D → 2D.
Pensé pour la visualisation d'objets 4D (tesseract, hypersphère, etc.) sur
microcontrôleur (OLED, TFT), sans allocation ni `libm`.

## Installation

```toml
[dependencies]
embedded-quaternion-f32 = "0.2"
```
## 📖 Fonctions clés

| Fonction | Description | Retour |
|---|---|---|
| `Quaternion::new(w, x, y, z)` | Construit un quaternion | `Quaternion` |
| `Quaternion::ZERO` / `Quaternion::ONE` | Constantes : nul / unité de Hamilton | `Quaternion` |
| `from_angle_plane(angle)` | Calcule `(sin, cos)` pour un angle | `(f32, f32)` |
| `rotate_xw(angle)` | Rotation 4D dans le plan X-W | `Quaternion` |
| `rotate_yz(angle)` | Rotation 4D dans le plan Y-Z | `Quaternion` |
| `conjugate()` | Conjugué `q* = w - xi - yj - zk` | `Quaternion` |
| `norm_squared()` | Norme au carré (sans `sqrt`) | `f32` |
| `project_2d(cx, cy, dist_4d, dist_3d)` | Projection perspective 4D → 3D → 2D | `(i16, i16, f32)` |
| `+`, `-`, `*` (entre quaternions) | Addition, soustraction, produit de Hamilton | `Quaternion` |
| `*` (par un `f32`) | Multiplication par un scalaire | `Quaternion` |
| `normalize()` | Normalise le quaternion (le rend unitaire) | `Option<Quaternion>` |

## 💻 Exemples

### Construction et constantes

```rust
use embedded_quaternion_f32::Quaternion;

let q = Quaternion::new(1.0, 0.0, 0.0, 0.0);
assert_eq!(q, Quaternion::ONE);

```
### Normalisation d'un quaternion

```rust
use embedded_quaternion_f32::Quaternion;

let q = Quaternion::new(1.0, 2.0, 3.0, 4.0);

// Normalisation (rend le quaternion unitaire)
if let Some(normalized) = q.normalize() {
    assert!((normalized.norm_squared() - 1.0).abs() < 1e-4);
}

// Le quaternion nul ne peut pas être normalisé
assert!(Quaternion::ZERO.normalize().is_none());
```


### Rotation 4D (plan X-W) et projection à l'écran

```rust
use embedded_quaternion_f32::Quaternion;
use core::f32::consts::FRAC_PI_2;

// Point pur sur l'axe X
let p = Quaternion::new(0.0, 1.0, 0.0, 0.0);
let rotated = p.rotate_xw(FRAC_PI_2);

// Projection sur un écran 320x160, caméra à distance 5 en 4D et en 3D
let (screen_x, screen_y, depth) = rotated.project_2d(160.0, 80.0, 5.0, 5.0);
```

### Rotation dans le plan Y-Z

```rust
use embedded_quaternion_f32::Quaternion;

let p = Quaternion::new(0.0, 0.0, 1.0, 0.0);
let rotated = p.rotate_yz(core::f32::consts::PI);
```

### Produit de Hamilton et conjugué

```rust
use embedded_quaternion_f32::Quaternion;

let a = Quaternion::new(1.0, 2.0, 3.0, 4.0);
let b = Quaternion::new(5.0, 6.0, 7.0, 8.0);

let product = a * b;          // produit de Hamilton (non commutatif : a*b != b*a)
let conj = a.conjugate();     // w - xi - yj - zk
let dist2 = a.norm_squared(); // norme au carré, sans sqrt
```

### Arithmétique de base

```rust
use embedded_quaternion_f32::Quaternion;

let a = Quaternion::new(1.0, 2.0, 3.0, 4.0);
let b = Quaternion::new(0.5, 0.5, 0.5, 0.5);

let sum = a + b;
let diff = a - b;
let scaled = a * 2.0; // multiplication par un scalaire f32
```

## Dépendances

Cette crate s'appuie sur [`embedded-trig-f32`](https://crates.io/crates/embedded-trig-f32)
pour `sin`/`cos`/`sincos` et [`embedded-f32-sqrt`](https://crates.io/crates/embedded-f32-sqrt)
pour la racine carrée optimisée.. Ajustez la version dans `Cargo.toml` si vous
utilisez une dépendance locale (`path = ".."`) pendant le développement.

> **Précision** : `embedded-trig-f32` fournit des approximations rapides
> (Bhaskara / McCutcheon), pas des calculs `libm` exacts  l'identité
> `sin² + cos² = 1` n'est garantie qu'à ~2·10⁻³ près. Pour des rotations
> composées sur de nombreuses itérations, prévoyez une renormalisation
> périodique du quaternion si la dérive devient sensible.

## 🧪 Tests

```bash
cargo test
```

## Licence

GPL-2.0-or-later

## Copyright (C) 2026 Jorge Andre Castro

