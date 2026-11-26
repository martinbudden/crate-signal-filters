#![allow(clippy::cast_precision_loss)]

use core::ops::{Add, Mul, Sub};
use num_traits::Zero;

#[cfg(feature = "nalgebra")]
use nalgebra::{Vector2, Vector3, Vector4};
#[cfg(feature = "vqm")]
use vqm::{Vector2, Vector3, Vector4};

/// `MovingAverageFilter` for `f32`<br>
pub type MovingAverageFilterf32<const N: usize> = MovingAverageFilter<f32, N>;
/// `MovingAverageFilter` for `Vector2f32`<br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type MovingAverageFilterVector2f32<const N: usize> = MovingAverageFilter<Vector2<f32>, N>;
/// `MovingAverageFilter` for `Vector3f32`<br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type MovingAverageFilterVector3f32<const N: usize> = MovingAverageFilter<Vector3<f32>, N>;
/// `MovingAverageFilter` for `Vector4f32`<br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type MovingAverageFilterVector4f32<const N: usize> = MovingAverageFilter<Vector4<f32>, N>;

/// `MovingAverageFilter` for `f64`<br><br>
pub type MovingAverageFilterf64<const N: usize> = MovingAverageFilter<f64, N>;
/// `MovingAverageFilter` for `Vector2f64`<br><br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type MovingAverageFilterVector2f64<const N: usize> = MovingAverageFilter<Vector2<f64>, N>;
/// `MovingAverageFilter` for `Vector3f64`<br><br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type MovingAverageFilterVector3f64<const N: usize> = MovingAverageFilter<Vector3<f64>, N>;
/// `MovingAverageFilter` for `Vector4f64`<br><br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type MovingAverageFilterVector4f64<const N: usize> = MovingAverageFilter<Vector4<f64>, N>;

/// Simple moving average filter.<br><br>
// See [Moving Average Filter - Theory and Software Implementation - Phil's Lab #21](https://www.youtube.com/watch?v=rttn46_Y3c8).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MovingAverageFilter<T, const N: usize> {
    count: usize,
    index: usize,
    sum: T,
    samples: [T; N],
}

impl<T, const N: usize> Default for MovingAverageFilter<T, N>
where
    T: Copy + Zero,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<T, const N: usize> MovingAverageFilter<T, N>
where
    T: Copy + Zero,
{
    /// Constructor.
    #[must_use]
    pub fn new() -> Self {
        Self { count: 0, index: 0, sum: T::zero(), samples: [T::zero(); N] }
    }
}

impl<T, const N: usize> MovingAverageFilter<T, N>
where
    T: Copy + Zero + Add<Output = T> + Sub<Output = T> + Mul<f32, Output = T>,
{
    pub fn reset(&mut self) {
        self.sum = T::zero();
        self.count = 0;
        self.index = 0;
    }

    pub fn update(&mut self, input: T) -> T {
        self.sum = self.sum + input;
        if self.count < N {
            self.samples[self.index] = input;
            self.index += 1;
            self.count += 1;
            return self.sum * (1.0 / self.count as f32);
        }
        if self.index == N {
            self.index = 0;
        }
        self.sum = self.sum - self.samples[self.index];
        self.samples[self.index] = input;
        self.index += 1;

        self.sum * (1.0 / N as f32)
    }
}

#[cfg(test)]
mod test_traits {
    use super::*;

    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_full::<MovingAverageFilter<f32, 2>>();
    }
}

#[cfg(any(debug_assertions, test))]
mod tests {
    #![allow(clippy::float_cmp)]
    #[allow(unused)]
    use super::*;
    #[test]
    fn moving_average_filter_f32() {
        let mut filter = MovingAverageFilter::<f32, 3>::new();
        assert_eq!(1.0, filter.update(1.0));
        assert_eq!(1.5, filter.update(2.0));
        assert_eq!(2.0, filter.update(3.0));
        assert_eq!(3.0, filter.update(4.0));
        assert_eq!(4.0, filter.update(5.0));
        assert_eq!(5.0, filter.update(6.0));
        assert_eq!(7.0, filter.update(10.0));

        filter.reset();
        assert_eq!(4.0, filter.update(4.0));
        assert_eq!(12.0, filter.update(20.0));
        assert_eq!(5.0, filter.update(-9.0));
    }

    #[test]
    fn moving_average_filter_vector3df32() {
        #[cfg(any(feature = "vqm", feature = "nalgebra"))]
        {
            type Vector3f32 = Vector3<f32>;
            let mut filter = MovingAverageFilter::<Vector3f32, 4>::new();
            let mut m = filter.update(Vector3f32::new(1.0, 0.0, -3.0));
            assert_eq!(Vector3f32::new(1.0, 0.0, -3.0), m);

            m = filter.update(Vector3f32::new(2.0, 0.0, -3.0));
            assert_eq!(Vector3f32::new(1.5, 0.0, -3.0), m);

            m = filter.update(Vector3f32::new(3.0, 3.0, 0.0));
            assert_eq!(Vector3f32::new(2.0, 1.0, -2.0), m);

            m = filter.update(Vector3f32::new(4.0, 2.0, -3.0));
            assert_eq!(Vector3f32::new(2.5, 1.25, -2.25), m);

            m = filter.update(Vector3f32::new(5.0, 2.0, -3.0));
            assert_eq!(Vector3f32::new(3.5, 1.75, -2.25), m);

            m = filter.update(Vector3f32::new(6.0, 2.0, -3.0));
            assert_eq!(Vector3f32::new(4.5, 2.25, -2.25), m);

            m = filter.update(Vector3f32::new(10.0, 2.0, -3.0));
            assert_eq!(Vector3f32::new(6.25, 2.0, -3.0), m);

            filter.reset();
            m = filter.update(Vector3f32::new(4.0, 2.0, -3.0));
            assert_eq!(Vector3f32::new(4.0, 2.0, -3.0), m);

            m = filter.update(Vector3f32::new(20.0, 0.0, -3.0));
            assert_eq!(Vector3f32::new(12.0, 1.0, -3.0), m);

            m = filter.update(Vector3f32::new(-9.0, 0.0, -3.0));
            assert_eq!(Vector3f32::new(5.0, 2.0 / 3.0, -3.0), m);
        }
    }
}
