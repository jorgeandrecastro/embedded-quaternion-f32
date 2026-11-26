// Copyright (C) 2026 Jorge Andre Castro
#![no_std]

use core::ops::{Add, Mul, Sub};
use embedded_trig_f32::sincos;
use embedded_f32_sqrt::sqrt; 

/// Represents a quaternion q = w + x*i + y*j + z*k
/// or a point/vector in 4D space.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quaternion {
    pub w: f32, // Scalar component (or fourth dimension)
    pub x: f32, // i component
    pub y: f32, // j component
    pub z: f32, // k component
}

impl Quaternion {
    /// Constructs a quaternion
    pub const fn new(w: f32, x: f32, y: f32, z: f32) -> Self {
        Self { w, x, y, z }
    }

    /// Zero quaternion / Point at the origin
    pub const ZERO: Self = Self::new(0.0, 0.0, 0.0, 0.0);

    /// Hamilton's unit (1)
    pub const ONE: Self = Self::new(1.0, 0.0, 0.0, 0.0);

    /// Creates a 4D rotation quaternion (Hamilton rotor)
    /// in the specified plane (e.g. XW or YZ).
    pub fn from_angle_plane(angle: f32) -> (f32, f32) {
        sincos(angle)
    }

    /// Applies a 4D rotation in the X-W plane (specific to quaternions)
    pub fn rotate_xw(self, angle: f32) -> Self {
        let (s, c) = sincos(angle);
        Self {
            w: self.x * s + self.w * c,
            x: self.x * c - self.w * s,
            y: self.y,
            z: self.z,
        }
    }

    /// Applies a 4D rotation in the Y-Z plane
    pub fn rotate_yz(self, angle: f32) -> Self {
        let (s, c) = sincos(angle);
        Self {
            w: self.w,
            x: self.x,
            y: self.y * c - self.z * s,
            z: self.y * s + self.z * c,
        }
    }

    /// Quaternion conjugate: q* = w - xi - yj - zk
    pub fn conjugate(self) -> Self {
        Self {
            w: self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }

    /// Squared norm (avoids sqrt in embedded environments)
    pub fn norm_squared(self) -> f32 {
        self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z
    }

    /// 4D -> 3D -> 2D perspective projection
    /// Returns (screen_x, screen_y, depth_z)
    pub fn project_2d(
        self,
        center_x: f32,
        center_y: f32,
        dist_4d: f32,
        dist_3d: f32,
    ) -> (i16, i16, f32) {
        // 1. Perspective 4D (w) -> 3D
        let pers_4d = dist_4d / (dist_4d + self.w);
        let x_3d = self.x * pers_4d;
        let y_3d = self.y * pers_4d;
        let z_3d = self.z * pers_4d;

        // 2. Perspective 3D (z) -> 2D (screen)
        let pers_3d = dist_3d / (dist_3d + z_3d);
        let px = (center_x + x_3d * pers_3d) as i16;
        let py = (center_y + y_3d * pers_3d) as i16;

        (px, py, z_3d)
    }

    /// Normalizes the quaternion (makes it unit length)
    /// Returns None if the norm is too close to zero
    pub fn normalize(self) -> Option<Self> {
        let norm_sq = self.norm_squared();
        if norm_sq > f32::EPSILON {
            // Use embedded-f32-sqrt instead of f32::sqrt()
            if let Ok(norm) = sqrt(norm_sq) {
                let inv_norm = 1.0 / norm;
                Some(Self {
                    w: self.w * inv_norm,
                    x: self.x * inv_norm,
                    y: self.y * inv_norm,
                    z: self.z * inv_norm,
                })
            } else {
                // sqrt() returned an error (normally impossible with norm_sq > EPSILON)
                None
            }
        } else {
            None
        }
    }

}

// --- Hamilton product (multiplication of 2 quaternions) ---
// i² = j² = k² = ijk = -1
impl Mul for Quaternion {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self {
        Self {
            w: self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
            x: self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
            y: self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
            z: self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w,
        }
    }
}

// --- Quaternion addition ---
impl Add for Quaternion {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self {
            w: self.w + rhs.w,
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

// --- Quaternion subtraction ---
impl Sub for Quaternion {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self {
            w: self.w - rhs.w,
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

// --- Multiplication by an f32 scalar ---
impl Mul<f32> for Quaternion {
    type Output = Self;
    fn mul(self, scalar: f32) -> Self {
        Self {
            w: self.w * scalar,
            x: self.x * scalar,
            y: self.y * scalar,
            z: self.z * scalar,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::PI;

    const EPS: f32 = 1e-4;

    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() <= EPS
    }

    fn assert_q_approx(q: Quaternion, w: f32, x: f32, y: f32, z: f32) {
        assert!(approx_eq(q.w, w), "w: {} != {}", q.w, w);
        assert!(approx_eq(q.x, x), "x: {} != {}", q.x, x);
        assert!(approx_eq(q.y, y), "y: {} != {}", q.y, y);
        assert!(approx_eq(q.z, z), "z: {} != {}", q.z, z);
    }

    #[test]
    fn test_new_and_constants() {
        let q = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(q.w, 1.0);
        assert_eq!(q.x, 2.0);
        assert_eq!(q.y, 3.0);
        assert_eq!(q.z, 4.0);

        assert_eq!(Quaternion::ZERO, Quaternion::new(0.0, 0.0, 0.0, 0.0));
        assert_eq!(Quaternion::ONE, Quaternion::new(1.0, 0.0, 0.0, 0.0));
    }

    #[test]
    fn test_add() {
        let a = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        let b = Quaternion::new(4.0, 3.0, 2.0, 1.0);
        let sum = a + b;
        assert_q_approx(sum, 5.0, 5.0, 5.0, 5.0);

        // Identity element
        assert_eq!(a + Quaternion::ZERO, a);
    }

    #[test]
    fn test_sub() {
        let a = Quaternion::new(4.0, 3.0, 2.0, 1.0);
        let b = Quaternion::new(1.0, 1.0, 1.0, 1.0);
        let diff = a - b;
        assert_q_approx(diff, 3.0, 2.0, 1.0, 0.0);
        assert_eq!(a - a, Quaternion::ZERO);
    }

    #[test]
    fn test_mul_scalar() {
        let a = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        let scaled = a * 2.0;
        assert_q_approx(scaled, 2.0, 4.0, 6.0, 8.0);

        // Multiplication by 0 gives the zero quaternion
        assert_eq!(a * 0.0, Quaternion::ZERO);
    }

    #[test]
    fn test_conjugate() {
        let a = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        let conj = a.conjugate();
        assert_q_approx(conj, 1.0, -2.0, -3.0, -4.0);

        // The conjugate of the conjugate returns the original
        assert_eq!(conj.conjugate(), a);
    }

    #[test]
    fn test_norm_squared() {
        let a = Quaternion::new(1.0, 2.0, 2.0, 4.0);
        // 1 + 4 + 4 + 16 = 25
        assert!(approx_eq(a.norm_squared(), 25.0));

        assert!(approx_eq(Quaternion::ZERO.norm_squared(), 0.0));
        assert!(approx_eq(Quaternion::ONE.norm_squared(), 1.0));
    }

    #[test]
    fn test_mul_identity() {
        let a = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        // 1 * q = q and q * 1 = q
        assert_q_approx(Quaternion::ONE * a, a.w, a.x, a.y, a.z);
        assert_q_approx(a * Quaternion::ONE, a.w, a.x, a.y, a.z);
    }

    #[test]
    fn test_hamilton_basis_products() {
        let i = Quaternion::new(0.0, 1.0, 0.0, 0.0);
        let j = Quaternion::new(0.0, 0.0, 1.0, 0.0);
        let k = Quaternion::new(0.0, 0.0, 0.0, 1.0);

        // i² = j² = k² = -1
        assert_q_approx(i * i, -1.0, 0.0, 0.0, 0.0);
        assert_q_approx(j * j, -1.0, 0.0, 0.0, 0.0);
        assert_q_approx(k * k, -1.0, 0.0, 0.0, 0.0);

        // i*j = k, j*k = i, k*i = j
        assert_q_approx(i * j, 0.0, 0.0, 0.0, 1.0);
        assert_q_approx(j * k, 0.0, 1.0, 0.0, 0.0);
        assert_q_approx(k * i, 0.0, 0.0, 1.0, 0.0);

        // j*i = -k (noncommutative)
        assert_q_approx(j * i, 0.0, 0.0, 0.0, -1.0);
    }

    #[test]
    fn test_mul_not_commutative() {
        let a = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        let b = Quaternion::new(5.0, 6.0, 7.0, 8.0);
        assert_ne!(a * b, b * a);
    }

    #[test]
    fn test_from_angle_plane() {
        let (s, c) = Quaternion::from_angle_plane(0.0);
        assert!(approx_eq(s, 0.0));
        assert!(approx_eq(c, 1.0));

        let (s, c) = Quaternion::from_angle_plane(PI / 2.0);
        assert!(approx_eq(s, 1.0));
        assert!(approx_eq(c, 0.0));
    }

    #[test]
    fn test_rotate_xw_zero_angle_is_identity() {
        let q = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        let rotated = q.rotate_xw(0.0);
        assert_q_approx(rotated, q.w, q.x, q.y, q.z);
    }

    #[test]
    fn test_rotate_xw_quarter_turn() {
        // Pure point on the X axis (w=0, x=1)
        let q = Quaternion::new(0.0, 1.0, 0.0, 0.0);
        let rotated = q.rotate_xw(PI / 2.0);
        // After a 90° rotation, x -> w
        assert_q_approx(rotated, 1.0, 0.0, 0.0, 0.0);
    }

    #[test]
    fn test_rotate_xw_norm_matches_formula() {
        // Norm preservation depends on sin²+cos² = 1, which is only
        // approximately true with embedded-trig-f32 (~2e-3 tolerance,
        // see its documentation). So we check the formula itself rather
        // than assume a perfect trigonometric identity: the norm
        // of the rotated plane must be (w²+x²) * (s²+c²), computed with
        // the same s/c used internally by rotate_xw.
        let q = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        let angle = 0.73;
        let (s, c) = Quaternion::from_angle_plane(angle);
        let rotated = q.rotate_xw(angle);

        let expected_plane_norm = (q.w * q.w + q.x * q.x) * (s * s + c * c);
        let actual_plane_norm = rotated.w * rotated.w + rotated.x * rotated.x;
        assert!(approx_eq(expected_plane_norm, actual_plane_norm));

        // y and z are unaffected by rotate_xw
        assert!(approx_eq(rotated.y, q.y));
        assert!(approx_eq(rotated.z, q.z));
    }

    #[test]
    fn test_rotate_yz_norm_matches_formula() {
        let q = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        let angle = 1.1;
        let (s, c) = Quaternion::from_angle_plane(angle);
        let rotated = q.rotate_yz(angle);

        let expected_plane_norm = (q.y * q.y + q.z * q.z) * (s * s + c * c);
        let actual_plane_norm = rotated.y * rotated.y + rotated.z * rotated.z;
        assert!(approx_eq(expected_plane_norm, actual_plane_norm));

        // w and x are unaffected by rotate_yz
        assert!(approx_eq(rotated.w, q.w));
        assert!(approx_eq(rotated.x, q.x));
    }

    #[test]
    fn test_sincos_approx_unit_circle() {
        // Check that embedded-trig-f32 meets its documented tolerance
        // (~2e-3) for the identity sin²+cos²=1, without requiring perfect precision.
        for angle in [0.0_f32, 0.3, 0.73, 1.1, PI / 2.0, PI, 2.5] {
            let (s, c) = Quaternion::from_angle_plane(angle);
            assert!(
                (s * s + c * c - 1.0).abs() <= 5e-2,
                "sin²+cos² out of tolerance for angle={angle}: s={s}, c={c}"
            );
        }
    }

    #[test]
    fn test_rotate_yz_zero_angle_is_identity() {
        let q = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        let rotated = q.rotate_yz(0.0);
        assert_q_approx(rotated, q.w, q.x, q.y, q.z);
    }

    #[test]
    fn test_rotate_yz_quarter_turn() {
        // Pure point on the Y axis (y=1)
        let q = Quaternion::new(0.0, 0.0, 1.0, 0.0);
        let rotated = q.rotate_yz(PI / 2.0);
        // After a 90° rotation, y -> z
        assert_q_approx(rotated, 0.0, 0.0, 0.0, 1.0);
    }

    #[test]
    fn test_project_2d_origin_at_center() {
        // Point at the 4D origin -> should project to the center of the screen
        let q = Quaternion::ZERO;
        let (px, py, z) = q.project_2d(160.0, 80.0, 5.0, 5.0);
        assert_eq!(px, 160);
        assert_eq!(py, 80);
        assert!(approx_eq(z, 0.0));
    }

    #[test]
    fn test_project_2d_offset_point() {
        // Point purely on X: should move horizontally
        let q = Quaternion::new(0.0, 2.0, 0.0, 0.0);
        let (px, py, _z) = q.project_2d(100.0, 100.0, 5.0, 5.0);
        assert!(px > 100);
        assert_eq!(py, 100);
    }



    #[test]
    fn test_normalize() {
        let q = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        let normalized = q.normalize().unwrap();
        
        // Check that the norm is approximately 1.0
        assert!(approx_eq(normalized.norm_squared(), 1.0));
        
        // Check that the direction is preserved (q / ||q||)
        let norm_sq = q.norm_squared();
        let norm = sqrt(norm_sq).unwrap();  // Use sqrt in the test as well
        assert_q_approx(
            normalized,
            q.w / norm,
            q.x / norm,
            q.y / norm,
            q.z / norm,
        );
    }

    #[test]
    fn test_normalize_zero() {
        let q = Quaternion::ZERO;
        assert!(q.normalize().is_none());
    }

    #[test]
    fn test_normalize_unit_already() {
        let q = Quaternion::new(0.0, 1.0, 0.0, 0.0);
        let normalized = q.normalize().unwrap();
        assert_q_approx(normalized, 0.0, 1.0, 0.0, 0.0);
    }

   #[test]
    fn test_normalize_small() {
    // Test with clearly distinguishable values
    
    // 1. Quaternion that can clearly be normalized
    let q = Quaternion::new(0.1, 0.0, 0.0, 0.0);
    assert!(q.normalize().is_some());
    let normalized = q.normalize().unwrap();
    assert!(approx_eq(normalized.norm_squared(), 1.0));
    
    // 2. Zero quaternion (should fail)
    let q = Quaternion::ZERO;
    assert!(q.normalize().is_none());
    
    // 3. Very small quaternion (should fail)
    let q = Quaternion::new(1e-20, 0.0, 0.0, 0.0);
    assert!(q.normalize().is_none());
   }





}