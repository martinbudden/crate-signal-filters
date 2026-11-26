use core::ops::{Div, Mul, Sub};
use num_traits::{One, Zero};

#[cfg(feature = "nalgebra")]
use nalgebra::{Vector2, Vector3, Vector4};
#[cfg(feature = "vqm")]
use vqm::{Vector2, Vector3, Vector4};

use crate::{FilterMulAdd, FilterOne, SignalFilter};

// f64 variants.

/// `Pt1Filter` for `f32`<br>
pub type Pt1Filterf32 = Pt1Filter<f32, f32>;

/// `Pt1Filter` for `Vector2f32`<br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type Pt1FilterVector2f32 = Pt1Filter<Vector2<f32>, f32>;
/// `Pt1Filter` for `Vector3f32`<br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type Pt1FilterVector3f32 = Pt1Filter<Vector3<f32>, f32>;
/// `Pt1Filter` for `Vector4f32`<br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type Pt1FilterVector4f32 = Pt1Filter<Vector4<f32>, f32>;

/// `Pt1Filter` for `Vector2Vector2f32`<br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type Pt1FilterVector2Vector2f32 = Pt1Filter<Vector2<f32>, Vector2<f32>>;
/// `Pt1Filter` for `Vector3Vector3f32`<br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type Pt1FilterVector3Vector3f32 = Pt1Filter<Vector3<f32>, Vector3<f32>>;
/// `Pt1Filter` for `Vector4Vector4f32`<br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type Pt1FilterVector4Vector4f32 = Pt1Filter<Vector4<f32>, Vector4<f32>>;

/// `Pt2Filter` for `f32`<br>
pub type Pt2Filterf32 = Pt2Filter<f32, f32>;

/// `Pt2Filter` for `Vector2f32`<br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type Pt2FilterVector2f32 = Pt2Filter<Vector2<f32>, f32>;
/// `Pt2Filter` for `Vector3f32`<br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type Pt2FilterVector3f32 = Pt2Filter<Vector3<f32>, f32>;
/// `Pt2Filter` for `Vector4f32`<br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type Pt2FilterVector4f32 = Pt2Filter<Vector4<f32>, f32>;

/// `Pt2Filter` for `Vector2Vector2f32`<br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type Pt2FilterVector2Vector2f32 = Pt2Filter<Vector2<f32>, Vector2<f32>>;
/// `Pt2Filter` for `Vector3Vector3f32`<br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type Pt2FilterVector3Vector3f32 = Pt2Filter<Vector3<f32>, Vector3<f32>>;
/// `Pt2Filter` for `Vector4Vector4f32`<br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type Pt2FilterVector4Vector4f32 = Pt2Filter<Vector4<f32>, Vector4<f32>>;

/// `Pt3Filter` for `f32`<br>
pub type Pt3Filterf32 = Pt3Filter<f32, f32>;

/// `Pt3Filter` for `Vector2f32`<br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type Pt3FilterVector2f32 = Pt3Filter<Vector2<f32>, f32>;
/// `Pt3Filter` for `Vector3f32`<br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type Pt3FilterVector3f32 = Pt3Filter<Vector3<f32>, f32>;
/// `Pt3Filter` for `Vector4f32`<br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type Pt3FilterVector4f32 = Pt3Filter<Vector4<f32>, f32>;

/// `Pt3Filter` for `Vector2Vector2f32`<br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type Pt3FilterVector2Vector2f32 = Pt3Filter<Vector2<f32>, Vector2<f32>>;
/// `Pt3Filter` for `Vector3Vector3f32`<br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type Pt3FilterVector3Vector3f32 = Pt3Filter<Vector3<f32>, Vector3<f32>>;
/// `Pt3Filter` for `Vector4Vector4f32`<br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type Pt3FilterVector4Vector4f32 = Pt3Filter<Vector4<f32>, Vector4<f32>>;

// f64 variants.

/// `Pt1Filter` for `f64`<br><br>
pub type Pt1Filterf64 = Pt1Filter<f64, f64>;

/// `Pt1Filter` for `Vector2f64`<br><br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type Pt1FilterVector2f64 = Pt1Filter<Vector2<f64>, f64>;
/// `Pt1Filter` for `Vector3f64`<br><br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type Pt1FilterVector3f64 = Pt1Filter<Vector3<f64>, f64>;
/// `Pt1Filter` for `Vector4f64`<br><br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type Pt1FilterVector4f64 = Pt1Filter<Vector4<f64>, f64>;

/// `Pt1Filter` for `Vector2Vector2f64`<br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type Pt1FilterVector2Vector2f64 = Pt1Filter<Vector2<f64>, Vector2<f64>>;
/// `Pt1Filter` for `Vector3Vector3f64`<br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type Pt1FilterVector3Vector3f64 = Pt1Filter<Vector3<f64>, Vector3<f64>>;
/// `Pt1Filter` for `Vector4Vector4f64`<br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type Pt1FilterVector4Vector4f64 = Pt1Filter<Vector4<f64>, Vector4<f64>>;

/// `Pt2Filter` for `f64`<br><br>
pub type Pt2Filterf64 = Pt2Filter<f64, f64>;

/// `Pt2Filter` for `Vector2f64`<br><br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type Pt2FilterVector2f64 = Pt2Filter<Vector2<f64>, f64>;
/// `Pt2Filter` for `Vector3f64`<br><br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type Pt2FilterVector3f64 = Pt2Filter<Vector3<f64>, f64>;
/// `Pt2Filter` for `Vector4f64`<br><br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type Pt2FilterVector4f64 = Pt2Filter<Vector4<f64>, f64>;

/// `Pt2Filter` for `Vector2Vector2f64`<br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type Pt2FilterVector2Vector2f64 = Pt2Filter<Vector2<f64>, Vector2<f64>>;
/// `Pt2Filter` for `Vector3Vector3f64`<br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type Pt2FilterVector3Vector3f64 = Pt2Filter<Vector3<f64>, Vector3<f64>>;
/// `Pt2Filter` for `Vector4Vector4f64`<br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type Pt2FilterVector4Vector4f64 = Pt2Filter<Vector4<f64>, Vector4<f64>>;

/// `Pt3Filter` for `f64`<br><br>
pub type Pt3Filterf64 = Pt3Filter<f64, f64>;
/// `Pt3Filter` for `Vector2f64`<br><br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type Pt3FilterVector2f64 = Pt3Filter<Vector2<f64>, f64>;
/// `Pt3Filter` for `Vector3f64`<br><br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type Pt3FilterVector3f64 = Pt3Filter<Vector3<f64>, f64>;
/// `Pt3Filter` for `Vector4f64`<br><br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type Pt3FilterVector4f64 = Pt3Filter<Vector4<f64>, f64>;

/// `Pt3Filter` for `Vector2Vector2f64`<br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type Pt3FilterVector2Vector2f64 = Pt3Filter<Vector2<f64>, Vector2<f64>>;
/// `Pt3Filter` for `Vector3Vector3f64`<br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type Pt3FilterVector3Vector3f64 = Pt3Filter<Vector3<f64>, Vector3<f64>>;
/// `Pt3Filter` for `Vector4Vector4f64`<br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type Pt3FilterVector4Vector4f64 = Pt3Filter<Vector4<f64>, Vector4<f64>>;

pub trait PtFilterConstants {
    const PI: Self;
    // FilterPt<n> cutoff correction = 1/sqrt(2^(1/n) - 1)
    const PT2_CUTOFF_CORRECTION: Self;
    const PT3_CUTOFF_CORRECTION: Self;
    const HALF: Self;
}

#[allow(clippy::excessive_precision)]
impl PtFilterConstants for f32 {
    const PI: Self = core::f32::consts::PI;
    const PT2_CUTOFF_CORRECTION: Self = 1.553_773_974;
    const PT3_CUTOFF_CORRECTION: Self = 1.961_459_177;
    const HALF: Self = 0.5;
}

#[allow(clippy::excessive_precision)]
impl PtFilterConstants for f64 {
    const PI: Self = core::f64::consts::PI;
    const PT2_CUTOFF_CORRECTION: Self = 1.553_773_974;
    const PT3_CUTOFF_CORRECTION: Self = 1.961_459_177;
    const HALF: Self = 0.5;
}

#[allow(clippy::doc_paragraphs_missing_punctuation)]
/// Discrete-time, first-order low-pass filter (Proportional Time element).
///
/// It is implemented as a stateful struct that allows for efficient, in-place
/// smoothing of sensor data or motor setpoints.
///
/// The discrete-time transfer function is:
///
/// ```math
/// yₙ = yₙ₋₁ + k·(xₙ - yₙ₋₁)
/// ```
///
/// where `k` is calculated from the time constant `T` and sample time `dt`:
/// ```math
/// k = dt / (T + dt)
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pt1Filter<T, R> {
    state: T,
    k: R,
}

impl<T, R> Default for Pt1Filter<T, R>
where
    T: Copy + Zero,
    R: Copy + FilterOne,
{
    /// Default is k = 1.0, which is passthrough.
    fn default() -> Self {
        Self::new()
    }
}

impl<T, R> Pt1Filter<T, R>
where
    T: Copy + Zero,
    R: Copy + FilterOne,
{
    /// Create a passthrough filter starting at zero.
    #[must_use]
    pub fn new() -> Self
    where
        T: Zero,
        R: FilterOne,
    {
        Self { state: T::zero(), k: R::filter_one() }
    }

    /// Set the `k` of a newly constructed filter.
    #[must_use]
    pub const fn with_k(mut self, k: R) -> Self {
        self.k = k;
        self
    }

    /// Set the signal value (state) of a newly constructed filter.
    #[must_use]
    pub const fn with_state(mut self, state: T) -> Self {
        self.state = state;
        self
    }
}

impl<T, R> SignalFilter<T, R> for Pt1Filter<T, R>
where
    T: Copy + Zero + Sub<Output = T> + FilterMulAdd<R>,
    R: Copy,
{
    /// Reset the historical state to zero.
    fn reset(&mut self) {
        self.state = T::zero();
    }

    /// Reset the historical state to a specific value.
    fn reset_to_value(&mut self, value: T) {
        self.state = value;
    }

    #[inline]
    fn update(&mut self, input: T) -> T {
        // Equation: state = (input - state) * k + state

        self.state = (input - self.state).filter_mul_add(self.k, self.state);
        self.state
    }
}

impl<T, R> Pt1Filter<T, R>
where
    T: Copy,
    R: Copy + Zero + FilterOne + PartialOrd,
{
    /// Set the filter constant so the filter is in passthrough mode (no filtering).
    pub fn set_to_passthrough(&mut self) {
        self.k = R::filter_one();
    }

    /// Set the filter constant `k` directly. This is a low-level operation and should be used with care.
    pub const fn set_k(&mut self, k: R) {
        self.k = k;
    }

    /// Set the filter constant `k`, clamping it to  the range [0, 1] to ensure stability.
    pub fn set_k_safe(&mut self, k: R) {
        self.k = if k < R::zero() {
            R::zero()
        } else if k > R::filter_one() {
            R::filter_one()
        } else {
            k
        };
    }

    pub const fn k(&self) -> R {
        self.k
    }

    pub const fn state(&self) -> T {
        self.state
    }
}

impl<T, R> Pt1Filter<T, R> {
    pub fn set_cutoff_frequency(&mut self, cutoff_frequency_hz: R, delta_t: R)
    where
        R: Copy + Zero + One + PartialOrd + Div<Output = R> + PtFilterConstants,
    {
        self.k = Self::gain_from_frequency(cutoff_frequency_hz, delta_t);
    }

    pub fn set_cutoff_frequency_and_reset(&mut self, cutoff_frequency_hz: R, delta_t: R)
    where
        T: Copy + Zero + Sub<Output = T> + FilterMulAdd<R>,
        R: Copy + Zero + One + PtFilterConstants + PartialOrd + Div<R, Output = R>,
    {
        self.k = Self::gain_from_frequency(cutoff_frequency_hz, delta_t);
        self.reset();
    }

    /// Calculates filter gain based on delay (time constant of filter) - time it takes for filter response to reach 63.2% of a step input.
    pub fn gain_from_delay(delay: R, delta_t: R) -> R
    where
        R: Copy + Zero + One + PartialOrd + Div<Output = R>,
    {
        debug_assert!(delta_t > R::zero());
        if delay <= R::zero() {
            // gain of 1.0 means no filtering
            return R::one();
        }
        let omega = delta_t / delay;
        omega / (omega + R::one())
    }

    pub fn gain_from_frequency(cutoff_frequency_hz: R, delta_t: R) -> R
    where
        R: Copy + Zero + One + PartialOrd + Div<Output = R> + PtFilterConstants,
    {
        debug_assert!(cutoff_frequency_hz > R::zero());
        debug_assert!(delta_t > R::zero());
        let omega = (R::one() + R::one()) * R::PI * cutoff_frequency_hz * delta_t;
        omega / (omega + R::one())
    }
}

impl<T, R> Pt1Filter<T, R> {
    /// Updates the cutoff frequency seamlessly, ie without discontinuity in output.
    pub fn set_cutoff_frequency_seamless(&mut self, cutoff_frequency_hz: R, delta_t: R, last_input: T)
    where
        T: Copy + Zero + Sub<Output = T> + Mul<R, Output = T> + Div<R, Output = T> + FilterMulAdd<R>,
        R: Copy + Zero + One + PtFilterConstants + PartialOrd + Sub<Output = R> + Div<R, Output = R>,
    {
        let new_k = Self::gain_from_frequency(cutoff_frequency_hz, delta_t);

        // Clamp new_k to [0.0, 1.0] to ensure stability
        let safe_new_k = if new_k < R::zero() {
            R::zero()
        } else if new_k > R::one() {
            R::one()
        } else {
            new_k
        };

        // Adjust state to avoid discontinuity if the filter is active and not in passthrough mode.
        if safe_new_k > R::zero() {
            // Reconstruct the last output using old parameters.
            let last_output = (last_input - self.state).filter_mul_add(self.k, self.state);
            // Calculate the target state using the new k value.
            if safe_new_k < R::one() {
                self.state = (last_output - last_input * safe_new_k) / (R::one() - safe_new_k);
            }
        }

        // Apply the new filter constant.
        self.k = safe_new_k;
    }
}

#[allow(clippy::doc_paragraphs_missing_punctuation)]
/// Discrete-time, second-order low-pass filter (Proportional Time element).<br>
/// This is equivalent to two cascaded PT1 filters with the same time constant.
///
/// The discrete-time difference equations are:
///
/// ```math
/// wₙ = wₙ₋₁ + k·(xₙ - wₙ₋₁)
/// yₙ = yₙ₋₁ + k·(wₙ - yₙ₋₁)
/// ```
///
/// where:
/// - `xₙ` is the raw input
/// - `wₙ` is the internal state (output of the first stage)
/// - `yₙ` is the final filtered output
/// - `k` is the filter gain
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pt2Filter<T, R> {
    state: [T; 2],
    k: R,
}

impl<T, R> Default for Pt2Filter<T, R>
where
    T: Copy + Zero,
    R: Copy + FilterOne,
{
    /// Default is k = 1.0, which is passthrough.
    fn default() -> Self {
        Self::new()
    }
}

impl<T, R> Pt2Filter<T, R>
where
    T: Copy + Zero,
    R: Copy + FilterOne,
{
    /// Create a passthrough filter starting at zero.
    #[must_use]
    pub fn new() -> Self
    where
        T: Zero,
        R: FilterOne,
    {
        Self { state: [T::zero(), T::zero()], k: R::filter_one() }
    }

    /// Set the `k` of a newly constructed filter.
    #[must_use]
    pub const fn with_k(mut self, k: R) -> Self {
        self.k = k;
        self
    }

    /// Set the signal value (state) of a newly constructed filter.
    #[must_use]
    pub const fn with_state(mut self, state: [T; 2]) -> Self {
        self.state = state;
        self
    }
}

impl<T, R> SignalFilter<T, R> for Pt2Filter<T, R>
where
    T: Copy + Zero + Sub<Output = T> + FilterMulAdd<R>,
    R: Copy,
{
    /// Reset the historical state to zero.
    fn reset(&mut self) {
        self.state = [T::zero(), T::zero()];
    }

    /// Reset the historical state to a specific value.
    fn reset_to_value(&mut self, value: T) {
        self.state[1] = value;
        self.state[0] = value;
    }

    #[inline]
    fn update(&mut self, input: T) -> T {
        self.state[1] = (input - self.state[1]).filter_mul_add(self.k, self.state[1]);
        self.state[0] = (self.state[1] - self.state[0]).filter_mul_add(self.k, self.state[0]);
        self.state[0]
    }
}

impl<T, R> Pt2Filter<T, R>
where
    T: Copy,
    R: Copy + Zero + FilterOne + PartialOrd,
{
    /// Set the filter constant so the filter is in passthrough mode (no filtering).
    pub fn set_to_passthrough(&mut self) {
        self.k = R::filter_one();
    }

    /// Set the filter constant `k` directly. This is a low-level operation and should be used with care.
    pub fn set_k(&mut self, k: R) {
        self.k = k;
    }

    /// Set the filter constant `k`, clamping it to  the range [0, 1] to ensure stability.
    pub fn set_k_safe(&mut self, k: R) {
        self.k = if k < R::zero() {
            R::zero()
        } else if k > R::filter_one() {
            R::filter_one()
        } else {
            k
        };
    }

    pub fn k(self) -> R {
        self.k
    }

    pub fn state(self) -> [T; 2] {
        self.state
    }
}

impl<T, R> Pt2Filter<T, R> {
    pub fn set_cutoff_frequency(&mut self, cutoff_frequency_hz: R, delta_t: R)
    where
        R: Copy + Zero + One + PartialOrd + Div<Output = R> + PtFilterConstants,
    {
        self.k = Self::gain_from_frequency(cutoff_frequency_hz, delta_t);
    }

    pub fn set_cutoff_frequency_and_reset(&mut self, cutoff_frequency_hz: R, delta_t: R)
    where
        T: Copy + Zero + Sub<Output = T> + FilterMulAdd<R>,
        R: Copy + Zero + One + PtFilterConstants + PartialOrd + Div<R, Output = R>,
    {
        self.k = Self::gain_from_frequency(cutoff_frequency_hz, delta_t);
        self.reset();
    }

    pub fn gain_from_delay(delay: R, delta_t: R) -> R
    where
        R: Copy + Zero + One + PartialOrd + Div<Output = R> + PtFilterConstants,
    {
        Pt1Filter::<T, R>::gain_from_delay(delay * R::PT2_CUTOFF_CORRECTION, delta_t)
    }

    pub fn gain_from_frequency(cutoff_frequency_hz: R, delta_t: R) -> R
    where
        R: Copy + Zero + One + PartialOrd + Div<Output = R> + PtFilterConstants,
    {
        // shift cutoffFrequency to satisfy -3dB cutoff condition
        Pt1Filter::<T, R>::gain_from_frequency(cutoff_frequency_hz * R::PT2_CUTOFF_CORRECTION, delta_t)
    }
}

impl<T, R> Pt2Filter<T, R>
where
    T: Copy + Sub<Output = T> + Div<R, Output = T> + Mul<R, Output = T> + FilterMulAdd<R>,
    R: Copy + Zero + One + PtFilterConstants + PartialOrd + Sub<Output = R> + Div<R, Output = R>,
{
    /// Updates the cutoff frequency seamlessly, ie without discontinuity in output.
    pub fn set_cutoff_frequency_seamless(&mut self, cutoff_frequency_hz: R, delta_t: R, last_input: T) {
        // Calculate the raw gain based on the cutoff frequency
        let omega = (R::one() + R::one()) * R::PI * cutoff_frequency_hz * delta_t;
        let new_k = omega / (omega + R::one());

        // Clamp new_k to [0.0, 1.0] to ensure stability
        let safe_new_k = if new_k < R::zero() {
            R::zero()
        } else if new_k > R::one() {
            R::one()
        } else {
            new_k
        };

        // Adjust state to avoid discontinuity if the filter is active and not in passthrough mode.
        if safe_new_k > R::zero() {
            let one_minus_new_k = R::one() - safe_new_k;

            if one_minus_new_k > R::zero() {
                // Stage 1 output (prior to the coefficient switch)
                let last_output_stage1 = (last_input - self.state[1]).filter_mul_add(self.k, self.state[1]);

                // Stage 2 output (the final filter result right before the switch)
                let last_output_stage2 = (self.state[1] - self.state[0]).filter_mul_add(self.k, self.state[0]);

                // Back-calculate the new internal memory state for Stage 1
                self.state[1] = (last_output_stage1 - last_input * safe_new_k) / one_minus_new_k;

                // Back-calculate the new internal memory state for Stage 2
                // (Stage 2 treats the newly projected Stage 1 state as its incoming sample)
                self.state[0] = (last_output_stage2 - self.state[1] * safe_new_k) / one_minus_new_k;
            }
        }

        // Apply the new filter constant.
        self.k = safe_new_k;
    }
}

#[allow(clippy::doc_paragraphs_missing_punctuation)]
/// Discrete-time, third-order low-pass filter (Proportional Time element).<br>
/// This is equivalent to three cascaded PT1 filters. It provides a very steep
/// 60dB/decade roll-off.<br><br>
///
/// The discrete-time difference equations are:
///
/// ```math
/// uₙ = uₙ₋₁ + k·(xₙ - uₙ₋₁)
/// vₙ = vₙ₋₁ + k·(uₙ - vₙ₋₁)
/// yₙ = yₙ₋₁ + k·(vₙ - yₙ₋₁)
/// ```
///
/// where `uₙ` and `vₙ` are internal intermediate states, and `yₙ` is the final output.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pt3Filter<T, R> {
    state: [T; 3],
    k: R,
}

/// Default is k = 1.0, which is passthrough.
impl<T, R> Default for Pt3Filter<T, R>
where
    T: Copy + Zero,
    R: Copy + FilterOne,
{
    /// Default is k = 1.0, which is passthrough.
    fn default() -> Self {
        Self::new()
    }
}

impl<T, R> Pt3Filter<T, R>
where
    T: Copy + Zero,
    R: Copy + FilterOne,
{
    /// Create a passthrough filter starting at zero.
    #[must_use]
    pub fn new() -> Self {
        Self { state: [T::zero(), T::zero(), T::zero()], k: R::filter_one() }
    }

    /// Set the `k` of a newly constructed filter.
    #[must_use]
    pub const fn with_k(mut self, k: R) -> Self {
        self.k = k;
        self
    }

    /// Set the signal value (state) of a newly constructed filter.
    #[must_use]
    pub const fn with_state(mut self, state: [T; 3]) -> Self {
        self.state = state;
        self
    }
}

impl<T, R> SignalFilter<T, R> for Pt3Filter<T, R>
where
    T: Copy + Zero + Sub<Output = T> + FilterMulAdd<R>,
    R: Copy,
{
    /// Reset the historical state to zero.
    fn reset(&mut self) {
        self.state = [T::zero(), T::zero(), T::zero()];
    }

    /// Reset the historical state to a specific value.
    fn reset_to_value(&mut self, value: T) {
        self.state[1] = value;
        self.state[0] = value;
    }

    #[inline]
    fn update(&mut self, input: T) -> T {
        self.state[2] = (input - self.state[2]).filter_mul_add(self.k, self.state[2]);
        self.state[1] = (self.state[2] - self.state[1]).filter_mul_add(self.k, self.state[1]);
        self.state[0] = (self.state[1] - self.state[0]).filter_mul_add(self.k, self.state[0]);
        self.state[0]
    }
}

impl<T, R> Pt3Filter<T, R>
where
    T: Copy,
    R: Copy + Zero + FilterOne + PartialOrd,
{
    /// Set the filter constant so the filter is in passthrough mode (no filtering).
    pub fn set_to_passthrough(&mut self) {
        self.k = R::filter_one();
    }

    /// Set the filter constant `k` directly. This is a low-level operation and should be used with care.
    pub fn set_k(&mut self, k: R) {
        self.k = k;
    }

    /// Set the filter constant `k`, clamping it to  the range [0, 1] to ensure stability.
    pub fn set_k_safe(&mut self, k: R) {
        self.k = if k < R::zero() {
            R::zero()
        } else if k > R::filter_one() {
            R::filter_one()
        } else {
            k
        };
    }

    pub fn k(self) -> R {
        self.k
    }

    pub fn state(self) -> [T; 3] {
        self.state
    }
}

impl<T, R> Pt3Filter<T, R>
where
    T: Copy + Zero + Sub<Output = T> + FilterMulAdd<R>,
    R: Copy + Zero + One + PtFilterConstants + PartialOrd + Div<R, Output = R>,
{
    pub fn set_cutoff_frequency(&mut self, cutoff_frequency_hz: R, delta_t: R) {
        self.k = Self::gain_from_frequency(cutoff_frequency_hz, delta_t);
    }

    pub fn set_cutoff_frequency_and_reset(&mut self, cutoff_frequency_hz: R, delta_t: R) {
        self.k = Self::gain_from_frequency(cutoff_frequency_hz, delta_t);
        self.reset();
    }

    /// Calculates filter gain based on delay (time constant of filter) - time it takes for filter response to reach 63.2% of a step input.
    pub fn gain_from_delay(delay: R, delta_t: R) -> R {
        Pt1Filter::<T, R>::gain_from_delay(delay * R::PT3_CUTOFF_CORRECTION, delta_t)
    }

    pub fn gain_from_frequency(cutoff_frequency_hz: R, delta_t: R) -> R {
        // shift cutoffFrequency to satisfy -3dB cutoff condition
        Pt1Filter::<T, R>::gain_from_frequency(cutoff_frequency_hz * R::PT3_CUTOFF_CORRECTION, delta_t)
    }
}

#[cfg(test)]
mod test_traits {
    use super::*;

    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_full::<Pt1Filter<f32, f32>>();
        is_full::<Pt1Filterf32>();
        is_full::<Pt2Filter<f32, f32>>();
        is_full::<Pt2Filterf32>();
        is_full::<Pt3Filter<f32, f32>>();
        is_full::<Pt3Filterf32>();
    }
}

#[cfg(any(debug_assertions, test))]
mod tests {
    #![allow(clippy::float_cmp)]
    #![allow(unused_results)]

    #[allow(unused)]
    use super::*;

    #[test]
    fn pt1_filter_f32() {
        let mut filter = Pt1Filterf32::new();

        let mut reading: f32 = 2.7;
        reading = filter.update(reading);
        assert_eq!(2.7, reading);

        // test that filter with default settings performs no filtering
        assert_eq!(1.0, filter.update(1.0));
        assert_eq!(1.0, filter.update(1.0));
        assert_eq!(-1.0, filter.update(-1.0));

        filter.reset();
        assert_eq!(0.0, filter.state());
        assert_eq!(4.0, filter.update(4.0));

        filter.reset();
        filter.set_cutoff_frequency(100.0, 0.001);
        assert_eq!(0.385_869_56, filter.k());
        assert_eq!(0.385_869_56, filter.update(1.0));
        assert_eq!(1.008_713_4, filter.update(2.0));

        filter.set_k(1.0);
        assert_eq!(1.0, filter.update(1.0));
        assert_eq!(2.0, filter.update(2.0));

        filter.reset();
        filter.set_k(0.5);
        assert_eq!(0.5, filter.update(1.0));
        assert_eq!(1.25, filter.update(2.0));

        filter.set_cutoff_frequency_and_reset(100.0, 0.001);
        assert_eq!(0.385_869_56, filter.update(1.0));
        assert_eq!(1.008_713_4, filter.update(2.0));

        filter.set_to_passthrough();
        assert_eq!(1.0, filter.update(1.0));
        assert_eq!(2.0, filter.update(2.0));
    }

    #[test]
    fn pt1_filter_f32_method_call() {
        use crate::UpdateFilter;

        let mut filter = Pt1Filterf32::new().with_k(0.2);
        assert_eq!(0.2, filter.update(1.0));
        assert_eq!(0.2, filter.update(0.2));

        filter.reset();
        let value: f32 = 1.0;
        let value = value.filter_using(&mut filter);
        assert_eq!(0.2, value);
        let value = value.filter_using(&mut filter);
        assert_eq!(0.2, value);
    }

    #[test]
    fn pt1_filter_vector3f32_method_call() {
        let mut filter = Pt1Filterf32::new().with_k(0.25);
        assert_eq!(0.05, filter.update(0.2));
        filter.reset();
        assert_eq!(0.125, filter.update(0.5));
        filter.reset();
        assert_eq!(0.375, filter.update(1.5));

        #[cfg(any(feature = "vqm", feature = "nalgebra"))]
        {
            use crate::UpdateFilter;
            let mut filter = Pt1FilterVector3f32::new().with_k(0.25);
            let value = Vector3::<f32>::new(0.2, 0.5, 1.5);
            let output = filter.update(value);
            assert_eq!(Vector3::<f32>::new(0.05, 0.125, 0.375), output);

            filter.reset();
            let value = Vector3::<f32>::new(0.2, 0.5, 1.5);
            let value = value.filter_using(&mut filter);
            assert_eq!(Vector3::<f32>::new(0.05, 0.125, 0.375), value);
        }
    }

    #[test]
    fn pt2_filter_f32() {
        let mut filter = Pt2Filterf32::new().with_k(1.0);

        // test that filter with default settings performs no filtering
        assert_eq!(1.0, filter.update(1.0));
        assert_eq!(1.0, filter.update(1.0));
        assert_eq!(-1.0, filter.update(-1.0));

        filter.reset();
        assert_eq!(4.0, filter.update(4.0));

        filter.reset();
        filter.set_cutoff_frequency(100.0, 0.001);
        assert_eq!(0.244_031_07, filter.update(1.0));
        assert_eq!(0.735_024_03, filter.update(2.0));

        filter.set_k(1.0);
        assert_eq!(1.0, filter.update(1.0));
        assert_eq!(2.0, filter.update(2.0));

        filter.set_cutoff_frequency_and_reset(100.0, 0.001);
        assert_eq!(0.244_031_07, filter.update(1.0));
        assert_eq!(0.735_024_03, filter.update(2.0));

        filter.set_to_passthrough();
        assert_eq!(1.0, filter.update(1.0));
        assert_eq!(2.0, filter.update(2.0));
    }

    #[test]
    fn pt2_filter_f32_method_call() {
        use crate::UpdateFilter;

        let mut filter = Pt2Filterf32::new().with_k(0.2);
        assert_eq!(0.040_000_003, filter.update(1.0));
        assert_eq!(0.0656, filter.update(0.040_000_003));

        filter.reset();
        let value: f32 = 1.0;
        let value = value.filter_using(&mut filter);
        assert_eq!(0.040_000_003, value);
        let value = value.filter_using(&mut filter);
        assert_eq!(0.0656, value);
    }

    #[test]
    fn pt3_filter_f32() {
        let mut filter = Pt3Filterf32::new().with_k(1.0);

        let mut state = filter.state();
        assert_eq!([0.0, 0.0, 0.0], state);

        // test that filter with default settings performs no filtering
        let mut output = filter.update(1.0);
        assert_eq!(1.0, output);
        state = filter.state();
        assert_eq!([1.0, 1.0, 1.0], state);

        output = filter.update(1.0);
        state = filter.state();
        assert_eq!([1.0, 1.0, 1.0], state);
        assert_eq!(1.0, output);

        assert_eq!(-1.0, filter.update(-1.0));

        filter.reset();
        assert_eq!(4.0, filter.update(4.0));

        filter.reset();
        filter.set_cutoff_frequency(100.0, 0.001);
        assert_eq!(0.168_247_66, filter.update(1.0));
        assert_eq!(0.562_591_97, filter.update(2.0));

        filter.set_k(1.0);
        assert_eq!(1.0, filter.update(1.0));
        assert_eq!(2.0, filter.update(2.0));

        filter.set_cutoff_frequency_and_reset(100.0, 0.001);
        assert_eq!(0.168_247_66, filter.update(1.0));
        assert_eq!(0.562_591_97, filter.update(2.0));

        filter.set_to_passthrough();
        assert_eq!(1.0, filter.update(1.0));
        assert_eq!(2.0, filter.update(2.0));
    }

    #[test]
    fn pt1_filter_vector3f32() {
        #[cfg(any(feature = "vqm", feature = "nalgebra"))]
        {
            type Vector3f32 = Vector3<f32>;

            let mut filter = Pt1Filter::<Vector3f32, f32>::new().with_k(1.0);

            // test that filter with default settings performs no filtering
            let output = filter.update(Vector3f32::new(2.0, 3.0, 5.0));
            assert_eq!(Vector3f32::new(2.0, 3.0, 5.0), output);
            let state = filter.state();
            assert_eq!(Vector3f32::new(2.0, 3.0, 5.0), state);

            filter.reset();
            let state = filter.state();
            assert_eq!(Vector3f32::new(0.0, 0.0, 0.0), state);

            filter.set_cutoff_frequency(100.0, 0.001);
            assert_eq!(0.385_869_56, filter.update(Vector3f32::new(1.0, 0.0, 0.0)).x);
            assert_eq!(1.008_713_4, filter.update(Vector3f32::new(2.0, 0.0, 0.0)).x);

            filter.set_k(1.0);
            assert_eq!(1.0, filter.update(Vector3f32::new(1.0, 0.0, 0.0)).x);
            assert_eq!(2.0, filter.update(Vector3f32::new(2.0, 0.0, 0.0)).x);

            filter.set_cutoff_frequency_and_reset(100.0, 0.001);
            assert_eq!(0.385_869_56, filter.update(Vector3f32::new(1.0, 0.0, 0.0)).x);
            assert_eq!(1.008_713_4, filter.update(Vector3f32::new(2.0, 0.0, 0.0)).x);

            filter.set_to_passthrough();
            assert_eq!(1.0, filter.update(Vector3f32::new(1.0, 0.0, 0.0)).x);
            assert_eq!(2.0, filter.update(Vector3f32::new(2.0, 0.0, 0.0)).x);
        }
    }

    #[test]
    fn pt1_filter_vector3vector3f32() {
        #[cfg(any(feature = "vqm", feature = "nalgebra"))]
        {
            type Vector3f32 = Vector3<f32>;
            pub type Pt1FilterVector3Vector3f32 = Pt1Filter<Vector3f32, Vector3f32>;

            let mut filter = Pt1FilterVector3Vector3f32::new().with_k(Vector3f32::new(0.9, 0.8, 1.0));
            let output = filter.update(Vector3f32::new(2.0, 3.0, 5.0));
            assert_eq!(output.z, 5.0);
        }
    }
}
