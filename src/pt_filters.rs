use core::ops::{Div, Mul, Sub};
use num_traits::{ConstOne, ConstZero, MulAdd, One, Zero};
use vqm::{MathConstants, Vector2d, Vector3d, Vector4d};

use crate::SignalFilter;

/// `Pt1Filter` for `f32`<br>
pub type Pt1Filterf32 = Pt1Filter<f32, f32>;
/// `Pt1Filter` for `Vector2df32`<br>
pub type Pt1FilterVector2df32 = Pt1Filter<Vector2d<f32>, f32>;
/// `Pt1Filter` for `Vector3df32`<br>
pub type Pt1FilterVector3df32 = Pt1Filter<Vector3d<f32>, f32>;
/// `Pt1Filter` for `Vector4df32`<br>
pub type Pt1FilterVector4df32 = Pt1Filter<Vector4d<f32>, f32>;

/// `Pt1Filter` for `f64`<br><br>
pub type Pt1Filterf64 = Pt1Filter<f64, f64>;
/// `Pt1Filter` for `Vector2df64`<br><br>
pub type Pt1FilterVector2df64 = Pt1Filter<Vector2d<f64>, f64>;
/// `Pt1Filter` for `Vector3df64`<br><br>
pub type Pt1FilterVector3df64 = Pt1Filter<Vector3d<f64>, f64>;
/// `Pt1Filter` for `Vector4df64`<br><br>
pub type Pt1FilterVector4df64 = Pt1Filter<Vector4d<f64>, f64>;

/// `Pt2Filter` for `f32`<br>
pub type Pt2Filterf32 = Pt2Filter<f32, f32>;
/// `Pt2Filter` for `Vector2df32`<br>
pub type Pt2FilterVector2df32 = Pt2Filter<Vector2d<f32>, f32>;
/// `Pt2Filter` for `Vector3df32`<br>
pub type Pt2FilterVector3df32 = Pt2Filter<Vector3d<f32>, f32>;
/// `Pt2Filter` for `Vector4df32`<br>
pub type Pt2FilterVector4df32 = Pt2Filter<Vector4d<f32>, f32>;

/// `Pt2Filter` for `f64`<br><br>
pub type Pt2Filterf64 = Pt2Filter<f64, f64>;
/// `Pt2Filter` for `Vector2df64`<br><br>
pub type Pt2FilterVector2df64 = Pt2Filter<Vector2d<f64>, f64>;
/// `Pt2Filter` for `Vector3df64`<br><br>
pub type Pt2FilterVector3df64 = Pt2Filter<Vector3d<f64>, f64>;
/// `Pt2Filter` for `Vector4df64`<br><br>
pub type Pt2FilterVector4df64 = Pt2Filter<Vector4d<f64>, f64>;

/// `Pt3Filter` for `f32`<br>
pub type Pt3Filterf32 = Pt3Filter<f32, f32>;
/// `Pt3Filter` for `Vector2df32`<br>
pub type Pt3FilterVector2df32 = Pt3Filter<Vector2d<f32>, f32>;
/// `Pt3Filter` for `Vector3df32`<br>
pub type Pt3FilterVector3df32 = Pt3Filter<Vector3d<f32>, f32>;
/// `Pt3Filter` for `Vector4df32`<br>
pub type Pt3FilterVector4df32 = Pt3Filter<Vector4d<f32>, f32>;

/// `Pt3Filter` for `f64`<br><br>
pub type Pt3Filterf64 = Pt3Filter<f64, f64>;
/// `Pt3Filter` for `Vector2df64`<br><br>
pub type Pt3FilterVector2df64 = Pt3Filter<Vector2d<f64>, f64>;
/// `Pt3Filter` for `Vector3df64`<br><br>
pub type Pt3FilterVector3df64 = Pt3Filter<Vector3d<f64>, f64>;
/// `Pt3Filter` for `Vector4df64`<br><br>
pub type Pt3FilterVector4df64 = Pt3Filter<Vector4d<f64>, f64>;

#[allow(clippy::doc_paragraphs_missing_punctuation)]
/// Discrete-time, first-order low-pass filter (Proportional Time element).
///
/// It is implemented as a stateful struct that allows for efficient, in-place
/// smoothing of sensor data or motor setpoints.
///
/// The discrete-time transfer function is:
///
/// ```math
/// y{n} = y{n-1} + k * (x{n} - y{n-1})
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

/// Default is k = 1.0, which is passthrough.
impl<T, R> Default for Pt1Filter<T, R>
where
    T: ConstZero,
    R: ConstOne,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<T, R> Pt1Filter<T, R>
where
    T: ConstZero,
    R: ConstOne,
{
    /// Create a filter starting at a specific signal value with a specific k.
    pub const fn with_state_and_k(state: T, k: R) -> Self {
        Self { state, k }
    }

    /// Create a filter starting at zero with a custom k.
    pub const fn with_k(k: R) -> Self
    where
        T: ConstZero,
    {
        Self { state: T::ZERO, k }
    }

    /// Create a passthrough filter starting at zero.
    pub const fn new() -> Self
    where
        T: ConstZero,
        R: ConstOne,
    {
        Self { state: T::ZERO, k: R::ONE }
    }
}

impl<T, R> SignalFilter<T, R> for Pt1Filter<T, R>
where
    T: Copy + Zero + Sub<Output = T> + MulAdd<R, T, Output = T>,
    R: Copy,
{
    fn reset(&mut self) {
        self.state = T::zero();
    }

    /// Reset the historical memory to a specific value instead of hard zero.
    fn reset_to_value(&mut self, value: T) {
        self.state = value;
    }

    fn update(&mut self, input: T) -> T {
        // Equation: state = (input - state) * k + state
 
        self.state = (input - self.state).mul_add(self.k, self.state);
        self.state
    }
}

impl<T, R> Pt1Filter<T, R>
where
    T: Copy + Zero,
    R: Copy + Zero + One + PartialOrd,
{
    pub fn set_to_passthrough(&mut self) {
        self.k = R::one();
    }

    pub fn set_k(&mut self, k: R) {
        self.k = k;
    }

    pub fn set_k_safe(&mut self, k: R) {
        self.k = if k < R::zero() {
            R::zero()
        } else if k > R::one() {
            R::one()
        } else {
            k
        };
    }

    pub fn k(&self) -> R {
        self.k
    }

    pub fn state(&self) -> T {
        self.state
    }
}

impl<T, R> Pt1Filter<T, R>
where
    T: Copy + Sub<Output = T> + MulAdd<R, T, Output = T>,
    R: Copy + Zero + One + MathConstants + PartialOrd + Div<R, Output = R>,
{
    pub fn set_cutoff_frequency(&mut self, cutoff_frequency_hz: R, delta_t: R) {
        self.k = Self::gain_from_frequency(cutoff_frequency_hz, delta_t);
    }

    // Calculates filter gain based on delay (time constant of filter) - time it takes for filter response to reach 63.2% of a step input.
    pub fn gain_from_delay(delay: R, delta_t: R) -> R {
        if delay <= R::zero() {
            // gain of 1.0 means no filtering
            return R::one();
        }
        let omega = delta_t / delay;
        omega / (omega + R::one())
    }

    pub fn gain_from_frequency(cutoff_frequency_hz: R, delta_t: R) -> R {
        let two = R::one() + R::one();
        let omega = two * R::PI * cutoff_frequency_hz * delta_t;
        omega / (omega + R::one())
    }
}

impl<T, R> Pt1Filter<T, R>
where
    T: Copy + Zero + Sub<Output = T> + Mul<R, Output = T> + Div<R, Output = T> + MulAdd<R, T, Output = T>,
    R: Copy + Zero + One + MathConstants + PartialOrd + Sub<Output = R> + Div<R, Output = R>,
{
    /// Updates the cutoff frequency seamlessly, ie without discontinuity in output.
    pub fn set_cutoff_frequency_seamless(&mut self, cutoff_frequency_hz: R, delta_t: R, last_input: T) {
        let new_k = Self::gain_from_frequency(cutoff_frequency_hz, delta_t);

        // Clamp new_k safely
        let safe_new_k = if new_k < R::zero() {
            R::zero()
        } else if new_k > R::one() {
            R::one()
        } else {
            new_k
        };

        // Adjust state if the filter is active and not an exact passthrough bypass
        if safe_new_k > R::zero() {
            // Reconstruct what the last output was using old parameters
            let last_output = (last_input - self.state).mul_add(self.k, self.state);
            // Compute the target state using the new k value
            if safe_new_k < R::one() {
                self.state = (last_output - last_input * safe_new_k) / (R::one() - safe_new_k);
            }
        }

        // Apply the new coefficient
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
/// w{n} = w{n-1} + k * (x{n} - w{n-1})
/// y{n} = y{n-1} + k * (w{n} - y{n-1})
/// ```
///
/// where:
/// - `x{n}` is the raw input
/// - `w{n}` is the internal state (output of the first stage)
/// - `y{n}` is the final filtered output
/// - `k` is the filter gain
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pt2Filter<T, R> {
    state: [T; 2],
    k: R,
}

/// Default is k = 1.0, which is passthrough.
impl<T, R> Default for Pt2Filter<T, R>
where
    T: ConstZero,
    R: ConstOne,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<T, R> Pt2Filter<T, R>
where
    T: ConstZero,
    R: ConstOne,
{
    /// Create a filter starting at a specific signal value with a specific k.
    pub const fn with_state_and_k(state: [T; 2], k: R) -> Self {
        Self { state, k }
    }

    /// Create a filter starting at zero with a custom k.
    pub const fn with_k(k: R) -> Self
    where
        T: ConstZero,
    {
        Self { state: [T::ZERO, T::ZERO], k }
    }

    /// Create a passthrough filter starting at zero.
    pub const fn new() -> Self
    where
        T: ConstZero,
        R: ConstOne,
    {
        Self { state: [T::ZERO, T::ZERO], k: R::ONE }
    }
}

impl<T, R> SignalFilter<T, R> for Pt2Filter<T, R>
where
    T: Copy + Zero + Sub<Output = T> + MulAdd<R, T, Output = T>,
    R: Copy,
{
    fn reset(&mut self) {
        self.state = [T::zero(), T::zero()];
    }

    /// Reset the historical memory to a specific value instead of hard zero.
    fn reset_to_value(&mut self, value: T) {
        self.state[1] = value;
        self.state[0] = value;
    }

    fn update(&mut self, input: T) -> T {
        self.state[1] = (input - self.state[1]).mul_add(self.k, self.state[1]);
        self.state[0] = (self.state[1] - self.state[0]).mul_add(self.k, self.state[0]);
        self.state[0]
    }
}

impl<T, R> Pt2Filter<T, R>
where
    T: Copy + Zero,
    R: Copy + Zero + One + PartialOrd,
{
    pub fn set_to_passthrough(&mut self) {
        self.k = R::one();
    }

    pub fn set_k(&mut self, k: R) {
        self.k = k;
    }

    pub fn set_k_safe(&mut self, k: R) {
        self.k = if k < R::zero() {
            R::zero()
        } else if k > R::one() {
            R::one()
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

impl<T, R> Pt2Filter<T, R>
where
    T: Copy + Sub<Output = T> + MulAdd<R, T, Output = T>,
    R: Copy + Zero + One + MathConstants + PartialOrd + Div<R, Output = R>,
{
    pub fn set_cutoff_frequency(&mut self, cutoff_frequency_hz: R, delta_t: R) {
        self.k = Self::gain_from_frequency(cutoff_frequency_hz, delta_t);
    }

    pub fn gain_from_delay(delay: R, delta_t: R) -> R {
        Pt1Filter::<T, R>::gain_from_delay(delay * R::FILTER_PT2_CUTOFF_CORRECTION, delta_t)
    }
    pub fn gain_from_frequency(cutoff_frequency_hz: R, delta_t: R) -> R {
        // shift cutoffFrequency to satisfy -3dB cutoff condition
        Pt1Filter::<T, R>::gain_from_frequency(cutoff_frequency_hz * R::FILTER_PT2_CUTOFF_CORRECTION, delta_t)
    }
}

impl<T, R> Pt2Filter<T, R>
where
    T: Copy + Zero + Sub<Output = T> + Div<R, Output = T> + Mul<R, Output = T> + MulAdd<R, T, Output = T>,
    R: Copy + Zero + One + MathConstants + PartialOrd + Sub<Output = R> + Div<R, Output = R>,
{
    /// Seamlessly updates the cutoff frequency for the second-order filter
    /// by recalculating historical internal states to eliminate signal "pops".
    pub fn set_cutoff_frequency_seamless(&mut self, cutoff_frequency_hz: R, delta_t: R, last_input: T) {
        // Calculate the raw gain based on the cutoff frequency
        let two = R::one() + R::one();
        let omega = two * R::PI * cutoff_frequency_hz * delta_t;
        let new_k = omega / (omega + R::one());

        // 1. Clamp new_k strictly to ensure stability [0.0, 1.0]
        let safe_new_k = if new_k < R::zero() {
            R::zero()
        } else if new_k > R::one() {
            R::one()
        } else {
            new_k
        };

        // 2. Perform state correction if the filter isn't bypassing completely
        if safe_new_k > R::zero() {
            let one_minus_new_k = R::one() - safe_new_k;

            if one_minus_new_k > R::zero() {
                // Stage 1 output (prior to the coefficient switch)
                let last_output_stage1 = (last_input - self.state[1]).mul_add(self.k, self.state[1]);

                // Stage 2 output (the final filter result right before the switch)
                let last_output_stage2 = (self.state[1] - self.state[0]).mul_add(self.k, self.state[0]);

                // Back-calculate the new internal memory state for Stage 1
                self.state[1] = (last_output_stage1 - last_input * safe_new_k) / one_minus_new_k;

                // Back-calculate the new internal memory state for Stage 2
                // (Stage 2 treats the newly projected Stage 1 state as its incoming sample)
                self.state[0] = (last_output_stage2 - self.state[1] * safe_new_k) / one_minus_new_k;
            }
        }

        // 3. Commit the new coefficient smoothly
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
/// u{n} = u{n-1} + k * (x{n} - u{n-1})
/// v{n} = v{n-1} + k * (u{n} - v{n-1})
/// y{n} = y{n-1} + k * (v{n} - y{n-1})
/// ```
///
/// where `u{n}` and `v{n}` are internal intermediate states, and `y{n}` is the final output.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pt3Filter<T, R> {
    state: [T; 3],
    k: R,
}

/// Default is k = 1.0, which is passthrough.
impl<T, R> Default for Pt3Filter<T, R>
where
    T: ConstZero,
    R: ConstOne,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<T, R> Pt3Filter<T, R>
where
    T: ConstZero,
    R: ConstOne,
{
    /// Create a filter starting at a specific signal value with a specific k.
    pub const fn with_state_and_k(state: [T; 3], k: R) -> Self {
        Self { state, k }
    }
    /// Create a filter starting at zero with a custom k.
    pub const fn with_k(k: R) -> Self {
        Self { state: [T::ZERO, T::ZERO, T::ZERO], k }
    }
    /// Create a passthrough filter starting at zero.
    pub const fn new() -> Self {
        Self::with_k(R::ONE)
    }
}

impl<T, R> SignalFilter<T, R> for Pt3Filter<T, R>
where
    T: Copy + Zero + Sub<Output = T> + MulAdd<R, T, Output = T>,
    R: Copy,
{
    fn reset(&mut self) {
        self.state = [T::zero(), T::zero(), T::zero()];
    }

    /// Reset the historical memory to a specific value instead of hard zero.
    fn reset_to_value(&mut self, value: T) {
        self.state[1] = value;
        self.state[0] = value;
    }

    fn update(&mut self, input: T) -> T {
        self.state[2] = (input - self.state[2]).mul_add(self.k, self.state[2]);
        self.state[1] = (self.state[2] - self.state[1]).mul_add(self.k, self.state[1]);
        self.state[0] = (self.state[1] - self.state[0]).mul_add(self.k, self.state[0]);
        self.state[0]
    }
}

impl<T, R> Pt3Filter<T, R>
where
    T: Copy + Zero,
    R: Copy + Zero + One + PartialOrd,
{
    pub fn set_to_passthrough(&mut self) {
        self.k = R::one();
    }

    pub fn set_k(&mut self, k: R) {
        self.k = k;
    }

    pub fn set_k_safe(&mut self, k: R) {
        self.k = if k < R::zero() {
            R::zero()
        } else if k > R::one() {
            R::one()
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
    T: Copy + Sub<Output = T> + MulAdd<R, T, Output = T>,
    R: Copy + Zero + One + MathConstants + PartialOrd + Div<R, Output = R>,
{
    pub fn set_cutoff_frequency(&mut self, cutoff_frequency_hz: R, delta_t: R) {
        self.k = Self::gain_from_frequency(cutoff_frequency_hz, delta_t);
    }

    pub fn gain_from_delay(delay: R, delta_t: R) -> R {
        Pt1Filter::<T, R>::gain_from_delay(delay * R::FILTER_PT3_CUTOFF_CORRECTION, delta_t)
    }

    pub fn gain_from_frequency(cutoff_frequency_hz: R, delta_t: R) -> R {
        // shift cutoffFrequency to satisfy -3dB cutoff condition
        Pt1Filter::<T, R>::gain_from_frequency(cutoff_frequency_hz * R::FILTER_PT3_CUTOFF_CORRECTION, delta_t)
    }
}

#[cfg(any(debug_assertions, test))]
mod tests {
    #![allow(clippy::float_cmp)]
    #![allow(unused_results)]

    #[allow(unused)]
    use super::*;

    #[allow(unused)]
    fn is_normal<T: Sized + Send + Sync + Unpin>() {}
    #[allow(unused)]
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

        filter.set_cutoff_frequency(100.0, 0.001);
        filter.reset();
        assert_eq!(0.385_869_56, filter.update(1.0));
        assert_eq!(1.008_713_4, filter.update(2.0));

        filter.set_to_passthrough();
        assert_eq!(1.0, filter.update(1.0));
        assert_eq!(2.0, filter.update(2.0));
    }
    #[test]
    fn pt1_filter_f32_method_call() {
        use crate::UpdateFilter;

        let mut filter = Pt1Filterf32::with_k(0.2);
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
    fn pt1_filter_vector3df32_method_call() {
        use crate::UpdateFilter;
        use vqm::Vector3df32;

        let mut filter = Pt1Filterf32::with_k(0.25);
        assert_eq!(0.05, filter.update(0.2));
        filter.reset();
        assert_eq!(0.125, filter.update(0.5));
        filter.reset();
        assert_eq!(0.375, filter.update(1.5));

        let mut filter = Pt1FilterVector3df32::with_k(0.25);
        let value = Vector3df32 { x: 0.2, y: 0.5, z: 1.5 };
        let output = filter.update(value);
        assert_eq!(Vector3df32 { x: 0.05, y: 0.125, z: 0.375 }, output);

        filter.reset();
        let value = Vector3df32 { x: 0.2, y: 0.5, z: 1.5 };
        let value = value.filter_using(&mut filter);
        assert_eq!(Vector3df32 { x: 0.05, y: 0.125, z: 0.375 }, value);
    }
    #[test]
    fn pt2_filter_f32() {
        let mut filter = Pt2Filterf32::with_k(1.0);

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

        filter.set_cutoff_frequency(100.0, 0.001);
        filter.reset();
        assert_eq!(0.244_031_07, filter.update(1.0));
        assert_eq!(0.735_024_03, filter.update(2.0));

        filter.set_to_passthrough();
        assert_eq!(1.0, filter.update(1.0));
        assert_eq!(2.0, filter.update(2.0));
    }
    #[test]
    fn pt2_filter_f32_method_call() {
        use crate::UpdateFilter;

        let mut filter = Pt2Filterf32::with_k(0.2);
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
        let mut filter = Pt3Filterf32::with_k(1.0);

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

        filter.set_cutoff_frequency(100.0, 0.001);
        filter.reset();
        assert_eq!(0.168_247_66, filter.update(1.0));
        assert_eq!(0.562_591_97, filter.update(2.0));

        filter.set_to_passthrough();
        assert_eq!(1.0, filter.update(1.0));
        assert_eq!(2.0, filter.update(2.0));
    }
    #[test]
    fn pt1_filter_vector3df32() {
        use vqm::Vector3df32;

        let mut filter = Pt1Filter::<Vector3df32, f32>::with_k(1.0);

        // test that filter with default settings performs no filtering
        let output = filter.update(Vector3df32 { x: 2.0, y: 3.0, z: 5.0 });
        assert_eq!(Vector3df32 { x: 2.0, y: 3.0, z: 5.0 }, output);
        let state = filter.state();
        assert_eq!(Vector3df32 { x: 2.0, y: 3.0, z: 5.0 }, state);

        filter.reset();
        let state = filter.state();
        assert_eq!(Vector3df32 { x: 0.0, y: 0.0, z: 0.0 }, state);

        filter.set_cutoff_frequency(100.0, 0.001);
        assert_eq!(0.385_869_56, filter.update(Vector3df32 { x: 1.0, y: 0.0, z: 0.0 }).x);
        assert_eq!(1.008_713_4, filter.update(Vector3df32 { x: 2.0, y: 0.0, z: 0.0 }).x);

        filter.set_k(1.0);
        assert_eq!(1.0, filter.update(Vector3df32 { x: 1.0, y: 0.0, z: 0.0 }).x);
        assert_eq!(2.0, filter.update(Vector3df32 { x: 2.0, y: 0.0, z: 0.0 }).x);

        filter.set_cutoff_frequency(100.0, 0.001);
        filter.reset();
        assert_eq!(0.385_869_56, filter.update(Vector3df32 { x: 1.0, y: 0.0, z: 0.0 }).x);
        assert_eq!(1.008_713_4, filter.update(Vector3df32 { x: 2.0, y: 0.0, z: 0.0 }).x);

        filter.set_to_passthrough();
        assert_eq!(1.0, filter.update(Vector3df32 { x: 1.0, y: 0.0, z: 0.0 }).x);
        assert_eq!(2.0, filter.update(Vector3df32 { x: 2.0, y: 0.0, z: 0.0 }).x);
    }
    #[test]
    fn pt1_filter_vector3di16_i32() {
        use vqm::Vector3di16;
        let mut filter = Pt1Filter::<Vector3di16, i32>::new();

        // test that filter with default settings performs no filtering
        let output = filter.update(Vector3di16 { x: 2, y: 3, z: 5 });
        assert_eq!(Vector3di16 { x: 2, y: 3, z: 5 }, output);
        let state = filter.state();
        assert_eq!(Vector3di16 { x: 2, y: 3, z: 5 }, state);
    }
    #[test]
    fn pt1_filter_vector3di16_f32() {
        use vqm::Vector3di16;
        let mut filter = Pt1Filter::<Vector3di16, f32>::new();

        // test that filter with default settings performs no filtering
        let output = filter.update(Vector3di16 { x: 2, y: 3, z: 5 });
        assert_eq!(Vector3di16 { x: 2, y: 3, z: 5 }, output);
        let state = filter.state();
        assert_eq!(Vector3di16 { x: 2, y: 3, z: 5 }, state);
    }
    #[test]
    fn filter_vector3di32_i32() {
        use vqm::Vector3di32;

        let mut filter = Pt1Filter::<Vector3di32, i32>::with_k(1);

        // test that filter with default settings performs no filtering
        let output = filter.update(Vector3di32 { x: 2, y: 3, z: 5 });
        assert_eq!(Vector3di32 { x: 2, y: 3, z: 5 }, output);
    }
}
