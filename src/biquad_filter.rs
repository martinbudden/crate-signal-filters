use crate::{BiquadFilterMethods, FilterMulAdd};
use core::ops::{Add, Div, Mul, Neg, Sub};
use num_traits::{One, Zero};

#[cfg(feature = "nalgebra")]
use nalgebra::{Vector2, Vector3, Vector4};
#[cfg(feature = "vqm")]
use vqm::{Vector2, Vector3, Vector4};

#[cfg(feature = "serde")]
use ::serde::{Deserialize, Serialize};
#[cfg(feature = "storage")]
use {postcard::experimental::max_size::MaxSize, sequential_storage::map::PostcardValue};

use crate::SignalFilter;

/// `BiquadFilter` for `f32`<br>
pub type BiquadFilterf32 = BiquadFilter<f32, f32>;
/// `BiquadFilter` for `Vector2f32`<br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type BiquadFilterVector2f32 = BiquadFilter<Vector2<f32>, f32>;
/// `BiquadFilter` for `Vector3f32`<br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type BiquadFilterVector3f32 = BiquadFilter<Vector3<f32>, f32>;
/// `BiquadFilter` for `Vector4f32`<br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type BiquadFilterVector4f32 = BiquadFilter<Vector4<f32>, f32>;

/// `BiquadFilter` for `f64`<br><br>
pub type BiquadFilterf64 = BiquadFilter<f64, f64>;
/// `BiquadFilter` for `Vector2f64`<br><br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type BiquadFilterVector2f64 = BiquadFilter<Vector2<f64>, f64>;
/// `BiquadFilter` for `Vector3f64`<br><br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type BiquadFilterVector3f64 = BiquadFilter<Vector3<f64>, f64>;
/// `BiquadFilter` for `Vector4f64`<br><br>
#[cfg(any(feature = "vqm", feature = "nalgebra"))]
pub type BiquadFilterVector4f64 = BiquadFilter<Vector4<f64>, f64>;

/// Math constants for use in generic code.<br><br>
pub trait BiquadFilterConstants {
    /// Constant PI for use in generic code.
    const PI: Self;
    /// Constant 0.5 for use in generic code.
    const HALF: Self;
}

impl BiquadFilterConstants for f32 {
    const PI: Self = core::f32::consts::PI;
    const HALF: Self = 0.5;
}

impl BiquadFilterConstants for f64 {
    const PI: Self = core::f64::consts::PI;
    const HALF: Self = 0.5;
}

#[allow(clippy::doc_paragraphs_missing_punctuation)]
/// Second-order biquad IIR filter.<br><br>
///
/// The filter supports weighted notch filtering.
///
/// This implementation uses the Direct Form II Transposed (DF2T) structure.
/// This minimizes floating-point rounding noise and requires only two internal state variables `w1` and `w2`.
/// The difference equation is:
///
/// ```math
/// yₙ  = b0·xₙ + w1ₙ₋₁
/// w1ₙ = b1·xₙ - a1·yₙ + w2ₙ₋₁
/// w2ₙ = b2·xₙ - a2·yₙ
/// ```
///
/// where `x` represents the input signal and `y` represents the filtered output.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BiquadFilter<T, R> {
    state: BiquadFilterState<T>,
    coeffs: BiquadFilterCoefficients<R>,
    weight: R,
    loop_time_seconds: R,
    two_pi_loop_time_seconds: R, // cached value of 2.0 * PI * loop_time_seconds
    q: R,
    one_over_2q: R, // cached value of 1.0 / (2.0 * q)
}

impl<T, R> Default for BiquadFilter<T, R>
where
    T: Copy + Zero,
    R: Copy + Zero + One + BiquadFilterConstants + Div<R, Output = R>,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<T, R> BiquadFilter<T, R> {
    /// Constructor.
    #[must_use]
    pub fn new() -> Self
    where
        T: Copy + Zero,
        R: Copy + Zero + One + BiquadFilterConstants,
    {
        Self {
            state: BiquadFilterState::new(),
            coeffs: BiquadFilterCoefficients::new(),
            weight: R::one(),
            loop_time_seconds: R::zero(),
            two_pi_loop_time_seconds: R::zero(),
            q: R::one(),
            one_over_2q: R::HALF,
        }
    }

    /// Set the coefficients of a newly constructed filter.
    #[must_use]
    pub const fn with_coefficients(mut self, coeffs: BiquadFilterCoefficients<R>) -> Self
    where
        R: Copy,
    {
        self.coeffs = coeffs;
        self
    }

    /// Set the `q` of a newly constructed filter.
    #[must_use]
    pub fn with_q(mut self, q: R) -> Self
    where
        R: Copy + One + Add<Output = R> + Div<Output = R>,
    {
        self.set_q(q);
        self
    }
    /// Set the sample interval of a newly constructed filter.
    #[must_use]
    pub fn with_sample_interval(mut self, loop_time_seconds: R) -> Self
    where
        R: Copy + One + BiquadFilterConstants + Add<Output = R>,
    {
        self.set_sample_interval(loop_time_seconds);
        self
    }
}

impl<T, R> BiquadFilter<T, R> {
    /// Set `Q` the quality factor.
    /// For notch filters `Q` governs the bandwidth of the filter. High `Q` results in a narrow notch, low `Q` a wide notch.
    /// For high and low pass filters `Q` governs the resonance or peaking at the cutoff frequency.
    pub fn set_q(&mut self, q: R)
    where
        R: Copy + One + Add<Output = R> + Div<Output = R>,
    {
        self.q = q;
        self.one_over_2q = R::one() / ((R::one() + R::one()) * q); // cache value for faster setting of frequencies
    }

    pub fn set_sample_interval(&mut self, sample_interval_seconds: R)
    where
        R: Copy + One + BiquadFilterConstants + Add<Output = R> + Mul<Output = R>,
    {
        self.loop_time_seconds = sample_interval_seconds;
        self.two_pi_loop_time_seconds = (R::one() + R::one()) * R::PI * sample_interval_seconds; // cache value for faster setting of frequencies
    }

    pub fn set_sample_rate_hz(&mut self, sample_rate_hz: R)
    where
        R: Copy + One + BiquadFilterConstants + Add<Output = R> + Div<Output = R>,
    {
        self.set_sample_interval(R::one() / sample_rate_hz);
    }
}

/// NOTE: b2 == b0 for lowpass filter.
/// NOTE: b2 == b0 and a1 == b1 for notch filter.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "storage", derive(MaxSize))]
pub struct BiquadFilterCoefficients<R> {
    pub a1: R,
    pub a2: R,
    pub b0: R,
    pub b1: R,
    pub b2: R,
}

#[cfg(feature = "storage")]
impl<T> PostcardValue<'_> for BiquadFilterCoefficients<T> where T: Serialize + MaxSize + for<'de> Deserialize<'de> {}

impl<T> Default for BiquadFilterCoefficients<T>
where
    T: Copy + Zero + One,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<R> BiquadFilterCoefficients<R>
where
    R: Copy + Zero + One,
{
    pub fn new() -> Self {
        Self { a1: R::zero(), a2: R::zero(), b0: R::one(), b1: R::zero(), b2: R::zero() }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BiquadFilterState<T> {
    pub w1: T,
    pub w2: T,
}

impl<T> BiquadFilterState<T>
where
    T: Copy + Zero,
{
    pub fn new() -> Self {
        Self { w1: T::zero(), w2: T::zero() }
    }
}

impl<T> Default for BiquadFilterState<T>
where
    T: Copy + Zero,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<T, R> SignalFilter<T, R> for BiquadFilter<T, R>
where
    T: Copy + Zero + Sub<Output = T> + Mul<R, Output = T> + FilterMulAdd<R>,
    R: Copy + One + Sub<Output = R> + Neg<Output = R>,
{
    fn reset(&mut self) {
        self.state.w1 = T::zero();
        self.state.w2 = T::zero();
    }

    /// Sets the Biquad filter's state cleanly to a specific DC value,
    /// ie setting w1/w2 to a mathematical equilibrium to prevent signal jumps.
    fn reset_to_value(&mut self, value: T) {
        // w1 = value * (1.0 - b0)
        self.state.w1 = value * (R::one() - self.coeffs.b0);
        // w2 = value * (b2 - a2)
        self.state.w2 = value * (self.coeffs.b2 - self.coeffs.a2);
    }

    /// Update filter using Direct Form II.
    #[inline]
    fn update(&mut self, input: T) -> T {
        // Uses Direct Form II (note: Direct Form I uses at least 9 operations: 5 multiplications, 4 additions).

        // 6 operations: 3 mul_adds, 2 multiplications, 1 subtractions
        // Calculate the current output sample: yₙ = b0 * xₙ + w1
        let output = input.filter_mul_add(self.coeffs.b0, self.state.w1);

        // Calculate the next w1 state: w1 = b1 * xₙ - a1 * yₙ + w2
        self.state.w1 = input.filter_mul_add(self.coeffs.b1, self.state.w2) - output * self.coeffs.a1;

        // Calculate the next w2 state: w2 = b2 * xₙ - a2 * yₙ
        self.state.w2 = input.filter_mul_add(self.coeffs.b2, output * -self.coeffs.a2);
        output
    }
}

/// Biquad update notch.
///
/// For a notch filter a1 == b1 and b0 == b2, so this optimized form can be used.
impl<T, R> BiquadFilter<T, R>
where
    T: Copy + Add<Output = T> + Sub<Output = T> + Mul<R, Output = T> + FilterMulAdd<R>,
    R: Copy + Neg<Output = R>,
{
    /// Update filter, optimized for case when filter is configured as a notch filter (ie `a1 = b1` and `b0 = b2`).
    #[inline]
    pub fn update_notch(&mut self, input: T) -> T {
        /*
        // 6 operations: 2 mul_add, 2 multiplications, 2 addition/subtractions
        // Calculate output: yₙ = b0 * xₙ + w1
        let output = input.filter_mul_add(self.coeffs.b0, self.state.w1);

        // Calculate w1 = b1 * xₙ - a1 * yₙ + w2
        // Refactored as w1 = (xₙ - yₙ) * a1 + w2
        self.state.w1 = (input - output).filter_mul_add(self.coeffs.a1, self.state.w2);

        // Calculate w2 = b2 * xₙ - a2 * yₙ
        // Because b2 == b0, this expands to: w2 = b0 * xₙ - a2 * yₙ
        self.state.w2 = input * self.coeffs.b0 - output * self.coeffs.a2;
        */

        // 4 operations: 2 mul_add, 2 addition/subtractions, (negation not used, since compiler will generate a fused multiply-subtract instruction).
        let b0i = input * self.coeffs.b0;
        let output = b0i + self.state.w1;
        self.state.w1 = (input - output).filter_mul_add(self.coeffs.a1, self.state.w2);
        self.state.w2 = output.filter_mul_add(-self.coeffs.a2, b0i); // compiler automatically converts this to fused multiply-subtract

        output
    }
}

impl<T, R> BiquadFilter<T, R>
where
    T: Copy,
    R: Copy + Mul<Output = R>,
{
    /// `weight` of 1.0 gives just output, `weight` of 0.0 gives just input.
    #[inline]
    pub const fn set_weight(&mut self, weight: R) {
        self.weight = weight;
    }

    #[inline]
    pub const fn weight(&self) -> R {
        self.weight
    }

    /// Directly set the filter coefficients.
    #[inline]
    pub const fn set_coefficients(&mut self, coeffs: BiquadFilterCoefficients<R>) {
        self.coeffs = coeffs;
    }

    /// Directly set the filter coefficients and weight.
    #[inline]
    pub const fn set_coefficients_and_weight(&mut self, coeffs: BiquadFilterCoefficients<R>, weight: R) {
        self.weight = weight;
        self.coeffs = coeffs;
    }

    /// Copy parameters from another Biquad filter.
    #[inline]
    pub const fn set_parameters_from(&mut self, other: &BiquadFilter<T, R>) {
        self.weight = other.weight;
        self.coeffs = other.coeffs;
    }

    #[inline]
    pub fn calculate_omega(&self, frequency: R) -> R {
        frequency * self.two_pi_loop_time_seconds
    }

    #[inline]
    pub const fn q(&self) -> R {
        self.q
    }

    #[inline]
    pub const fn loop_time_seconds(&self) -> R {
        self.loop_time_seconds
    }

    #[inline]
    pub const fn state(self) -> BiquadFilterState<T> {
        self.state
    }
}

impl<T, R> BiquadFilter<T, R> {
    /// Set the filter to passthrough mode (ie no filtering).
    pub fn set_to_passthrough(&mut self)
    where
        T: Copy + Zero + Add<Output = T> + Sub<Output = T> + Mul<R, Output = T> + FilterMulAdd<R>,
        R: Copy
            + Zero
            + One
            + PartialOrd
            + Neg<Output = R>
            + BiquadFilterConstants
            + Div<R, Output = R>
            + Sub<R, Output = R>,
    {
        self.coeffs.b0 = R::one();
        self.coeffs.b1 = R::zero();
        self.coeffs.b2 = R::zero();
        self.coeffs.a1 = R::zero();
        self.coeffs.a2 = R::zero();
        self.weight = R::one();
        self.reset();
    }

    #[inline]
    pub fn update_weighted(&mut self, input: T) -> T
    where
        T: Copy + Zero + Add<Output = T> + Sub<Output = T> + Mul<R, Output = T> + FilterMulAdd<R>,
        R: Copy
            + Zero
            + One
            + PartialOrd
            + Neg<Output = R>
            + BiquadFilterConstants
            + Div<R, Output = R>
            + Sub<R, Output = R>,
    {
        let output = self.update(input);
        // weight of 1.0 gives just output, weight of 0.0 gives just input.
        (output - input).filter_mul_add(self.weight, input)
    }

    #[inline]
    pub fn update_notch_weighted(&mut self, input: T) -> T
    where
        T: Copy + Zero + Add<Output = T> + Sub<Output = T> + Mul<R, Output = T> + FilterMulAdd<R>,
        R: Copy
            + Zero
            + One
            + PartialOrd
            + Neg<Output = R>
            + BiquadFilterConstants
            + Div<R, Output = R>
            + Sub<R, Output = R>,
    {
        let output = self.update_notch(input);
        // weight of 1.0 gives just output, weight of 0.0 gives just input
        (output - input).filter_mul_add(self.weight, input)
    }

    /// Initialize the filter as a low pass filter.
    pub fn init_low_pass(&mut self, frequency_hz: R, sample_interval_seconds: R, q: R)
    where
        T: Copy + Zero + Add<Output = T> + Sub<Output = T> + Mul<R, Output = T> + FilterMulAdd<R>,
        R: Copy
            + Zero
            + One
            + PartialOrd
            + Neg<Output = R>
            + BiquadFilterConstants
            + Div<R, Output = R>
            + Sub<R, Output = R>
            + BiquadFilterMethods,
    {
        debug_assert!(q > R::zero());
        self.set_sample_interval(sample_interval_seconds);
        self.set_q(q);
        self.set_low_pass_frequency_assuming_q(frequency_hz);
        self.reset();
    }

    /// Initialize the filter as a notch filter.
    pub fn init_notch(&mut self, frequency_hz: R, sample_interval_seconds: R, q: R)
    where
        T: Copy + Zero + Add<Output = T> + Sub<Output = T> + Mul<R, Output = T> + FilterMulAdd<R>,
        R: Copy
            + Zero
            + One
            + PartialOrd
            + Neg<Output = R>
            + BiquadFilterConstants
            + Div<R, Output = R>
            + Sub<R, Output = R>
            + BiquadFilterMethods,
    {
        debug_assert!(q > R::zero());
        self.set_sample_interval(sample_interval_seconds);
        self.set_q(q);
        self.set_notch_frequency_assuming_q(frequency_hz);
        self.reset();
    }

    /// Pre-charges the filter states based on the initial sample value.
    /// This drastically reduces startup transient ringing and settling time.
    pub fn pre_charge(&mut self, initial_value: T)
    where
        T: Copy + FilterMulAdd<R> + Mul<R, Output = T>,
        R: Copy + One + Sub<Output = R>,
    {
        // For a DC input, steady state output equals input (unity gain).
        // Using: yₙ = b0*x + w1  =>  w1 = x * (1 - b0)
        self.state.w1 = initial_value * (R::one() - self.coeffs.b0);

        // Using: w2 = b2*x - a2*y  =>  Because x == y and b2 == b0:
        // w2 = x * (b0 - a2)
        self.state.w2 = initial_value * (self.coeffs.b0 - self.coeffs.a2);
    }

    //Note: weight must be in range [0, 1].
    pub fn calculate_low_pass_coefficients_assuming_q(&mut self, frequency_hz: R) -> BiquadFilterCoefficients<R>
    where
        R: Copy + Zero + One + PartialOrd + Div<Output = R> + Sub<Output = R> + Neg<Output = R> + BiquadFilterMethods,
    {
        debug_assert!(frequency_hz > R::zero());
        let omega = frequency_hz * self.two_pi_loop_time_seconds;
        let (sin_omega, cos_omega) = omega.sin_cos();
        let alpha = sin_omega * self.one_over_2q;
        let a0_reciprocal = R::one() / (R::one() + alpha);

        let b1 = (R::one() - cos_omega) * a0_reciprocal;
        let b0 = b1 * (R::one() / (R::one() + R::one()));
        BiquadFilterCoefficients {
            b0,
            b1,
            b2: b0,
            a1: -(R::one() + R::one()) * cos_omega * a0_reciprocal,
            a2: (R::one() - alpha) * a0_reciprocal,
        }
    }

    pub fn set_low_pass_frequency_weighted_assuming_q(&mut self, frequency_hz: R, weight: R)
    where
        R: Copy + Zero + One + PartialOrd + Div<Output = R> + Sub<Output = R> + Neg<Output = R> + BiquadFilterMethods,
    {
        self.weight = weight;
        self.coeffs = self.calculate_low_pass_coefficients_assuming_q(frequency_hz);
    }

    pub fn set_low_pass_frequency_assuming_q(&mut self, frequency_hz: R)
    where
        R: Copy + Zero + One + PartialOrd + Div<Output = R> + Sub<Output = R> + Neg<Output = R> + BiquadFilterMethods,
    {
        self.set_low_pass_frequency_weighted_assuming_q(frequency_hz, R::one());
    }

    pub fn calculate_notch_coefficients_from_sin_cos_assuming_q(
        &mut self,
        sin_omega: R,
        cos_omega: R,
    ) -> BiquadFilterCoefficients<R>
    where
        R: Copy + One + Add<Output = R> + Mul<Output = R> + Div<Output = R> + Neg<Output = R> + Sub<Output = R>,
    {
        let alpha = sin_omega * self.one_over_2q;
        let a0reciprocal = R::one() / (R::one() + alpha);
        let b1 = -(R::one() + R::one()) * cos_omega * a0reciprocal;
        BiquadFilterCoefficients {
            b0: a0reciprocal,
            b2: a0reciprocal,
            b1,
            a1: b1,
            a2: (R::one() - alpha) * a0reciprocal,
        }
    }
    #[inline]
    pub fn set_notch_frequency_weighted_from_sin_cos_assuming_q(&mut self, sin_omega: R, cos_omega: R, weight: R)
    where
        R: Copy + One + Add<Output = R> + Mul<Output = R> + Div<Output = R> + Neg<Output = R> + Sub<Output = R>,
    {
        self.weight = weight;
        self.coeffs = self.calculate_notch_coefficients_from_sin_cos_assuming_q(sin_omega, cos_omega);
    }

    pub fn calculate_notch_coefficients_assuming_q(&mut self, frequency_hz: R) -> BiquadFilterCoefficients<R>
    where
        R: Copy
            + One
            + Add<Output = R>
            + Mul<Output = R>
            + Div<Output = R>
            + Neg<Output = R>
            + Sub<Output = R>
            + BiquadFilterMethods,
    {
        let omega = frequency_hz * self.two_pi_loop_time_seconds;
        let (sin_omega, cos_omega) = omega.sin_cos();
        self.calculate_notch_coefficients_from_sin_cos_assuming_q(sin_omega, cos_omega)
    }

    #[inline]
    pub fn set_notch_frequency_weighted_assuming_q(&mut self, frequency_hz: R, weight: R)
    where
        R: Copy
            + One
            + Add<Output = R>
            + Mul<Output = R>
            + Div<Output = R>
            + Neg<Output = R>
            + Sub<Output = R>
            + BiquadFilterMethods,
    {
        self.weight = weight;
        self.coeffs = self.calculate_notch_coefficients_assuming_q(frequency_hz);
    }

    #[inline]
    pub fn set_notch_frequency_assuming_q(&mut self, frequency_hz: R)
    where
        R: Copy
            + One
            + Add<Output = R>
            + Mul<Output = R>
            + Div<Output = R>
            + Neg<Output = R>
            + Sub<Output = R>
            + BiquadFilterMethods,
    {
        self.set_notch_frequency_weighted_assuming_q(frequency_hz, R::one());
    }

    #[inline]
    pub fn set_notch_frequency(&mut self, center_frequency_hz: R, lower_cutoff_frequency_hz: R)
    where
        R: Copy + One + Add<Output = R> + Div<Output = R> + Sub<Output = R> + Neg<Output = R> + BiquadFilterMethods,
    {
        self.set_q(Self::calculate_q(center_frequency_hz, lower_cutoff_frequency_hz));
        self.set_notch_frequency_assuming_q(center_frequency_hz);
    }

    pub fn calculate_q(center_frequency_hz: R, lower_cutoff_frequency_hz: R) -> R
    where
        R: Copy + Mul<Output = R> + Sub<Output = R> + Div<Output = R>,
    {
        center_frequency_hz * lower_cutoff_frequency_hz
            / (center_frequency_hz * center_frequency_hz - lower_cutoff_frequency_hz * lower_cutoff_frequency_hz)
    }

    pub fn set_q_from_frequencies(&mut self, center_frequency_hz: R, lower_cutoff_frequency_hz: R)
    where
        R: Copy + One + Add<Output = R> + Div<Output = R> + Sub<Output = R>,
    {
        self.set_q(Self::calculate_q(center_frequency_hz, lower_cutoff_frequency_hz));
    }
}

#[cfg(test)]
mod test_traits {
    use super::*;

    fn _is_normal<T: Sized + Send + Sync + Unpin>() {}
    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}
    fn _is_full_eq<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + Eq + PartialEq>() {}
    #[cfg(feature = "serde")]
    fn is_serde<T: Serialize + for<'a> Deserialize<'a>>() {}
    #[cfg(feature = "storage")]
    fn is_storage<T: MaxSize + for<'a> PostcardValue<'a>>() {}

    #[test]
    fn normal_types() {
        is_full::<BiquadFilter<f32, f32>>();
        is_full::<BiquadFilterf32>();
        is_full::<BiquadFilterState<f32>>();
        is_full::<BiquadFilterCoefficients<f32>>();
        #[cfg(feature = "serde")]
        is_serde::<BiquadFilterCoefficients<f32>>();
        #[cfg(feature = "storage")]
        is_storage::<BiquadFilterCoefficients<f32>>();
    }
}

#[cfg(any(debug_assertions, test))]
mod tests {
    #![allow(clippy::float_cmp)]
    #![allow(unused)]
    #[allow(clippy::wildcard_imports)]
    use super::*;
    macro_rules! assert_near {
        ($left:expr, $right:expr, $eps:expr) => {
            assert!(
                ($left - $right).abs() < $eps,
                "\n    **** assert_near FAILED. Expected: {}, Found: {}\n",
                $left,
                $right
            );
        };
    }

    #[test]
    fn biquad_filter_f32() {
        let mut filter = BiquadFilterf32::default();

        // test that filter with default settings performs no filtering
        assert_eq!(1.0, filter.update(1.0));
        assert_eq!(1.0, filter.update(1.0));
        assert_eq!(-1.0, filter.update(-1.0));

        filter.reset();
        assert_eq!(4.0, filter.update(4.0));

        filter.set_coefficients_and_weight(
            BiquadFilterCoefficients { a1: 2.0, a2: 3.0, b0: 5.0, b1: 7.0, b2: 11.0 },
            13.0,
        );
        filter.set_to_passthrough();
        assert_eq!(1.0, filter.update(1.0));
        assert_eq!(2.0, filter.update(2.0));
        assert_eq!(1.0, filter.update_weighted(1.0));
        assert_eq!(2.0, filter.update_weighted(2.0));
    }

    #[test]
    fn biquad_filter_vector3df32() {
        #[cfg(any(feature = "vqm", feature = "nalgebra"))]
        {
            type Vector3f32 = Vector3<f32>;
            let mut filter = BiquadFilterVector3f32::default();
            let mut state: BiquadFilterState<Vector3f32>;

            // test that filter with default settings performs no filtering
            let output = filter.update(Vector3f32::new(2.0, 3.0, 5.0));
            assert_eq!(Vector3f32::new(2.0, 3.0, 5.0), output);
            state = filter.state();
            assert_eq!(0.0, state.w1.x);
            assert_eq!(0.0, state.w2.x);

            filter.reset();
            state = filter.state();
            assert_eq!(0.0, state.w1.x);
            assert_eq!(0.0, state.w2.x);
            assert_eq!(4.0, filter.update(Vector3f32::new(4.0, 0.0, 0.0)).x);

            filter.set_coefficients_and_weight(
                BiquadFilterCoefficients { a1: 2.0, a2: 3.0, b0: 5.0, b1: 7.0, b2: 11.0 },
                13.0,
            );
            filter.set_to_passthrough();
            assert_eq!(1.0, filter.update(Vector3f32::new(1.0, 0.0, 0.0)).x);
            assert_eq!(2.0, filter.update(Vector3f32::new(2.0, 0.0, 0.0)).x);
            assert_eq!(1.0, filter.update_weighted(Vector3f32::new(1.0, 0.0, 0.0)).x);
            assert_eq!(2.0, filter.update_weighted(Vector3f32::new(2.0, 0.0, 0.0)).x);
        }
    }

    #[test]
    fn test_notch_filter_attenuation_and_passthrough() {
        let sample_interval_s: f32 = 0.001; // 1 kHz sampling rate
        let notch_frequency: f32 = 50.0; // 50 Hz power line hum filter
        let q_factor: f32 = 10.0; // Narrow notch width

        // Initialize two identical filters to test different signals
        let mut notch_signal_filter = BiquadFilter::new().with_q(q_factor).with_sample_interval(sample_interval_s);
        let mut pass_signal_filter = BiquadFilter::new().with_q(q_factor).with_sample_interval(sample_interval_s);

        // Calculate filter coefficients
        let coeffs = notch_signal_filter.calculate_notch_coefficients_assuming_q(notch_frequency);

        notch_signal_filter.set_coefficients(coeffs);
        pass_signal_filter.set_coefficients(coeffs);

        // Assert that the optimization invariants hold true mathematically
        assert_near!(0.984_784_1, coeffs.b0, 1e-6);
        assert_near!(0.984_784_1, coeffs.b2, 1e-6);
        assert_eq!(coeffs.b1, coeffs.a1);

        // Exact expected values from the Audio EQ Cookbook for 50Hz @ 1kHz, Q=10.0
        // omega = 2 * PI * 50 / 1000 = 0.31415927
        // alpha = sin(omega) / (2 * 10) = 0.01545085
        // cos_omega = 0.9510565
        // a0 = 1.0 + 0.01545085 = 1.0154508
        // expected_a1 = (-2.0 * cos_omega) / a0 = -1.8731707
        // expected_a2 = -((1.0 - alpha) / a0) = -0.9695684
        let expected_a1 = -1.873_171;
        let expected_a2 = 0.969_568_5; // Pre-inverted inside generator
        assert!(
            (coeffs.a1 - expected_a1).abs() < 1e-6,
            "Coefficient a1 mismatch. Found: {}, Expected: {}",
            coeffs.a1,
            expected_a1
        );
        assert!(
            (coeffs.a2 - expected_a2).abs() < 1e-6,
            "Coefficient a2 mismatch. Found: {}, Expected: {}",
            coeffs.a2,
            expected_a2
        );

        // Run a simulation loop (1 second's worth of data)
        let total_samples = 1000;
        let mut max_notch_output: f32 = 0.0;
        let mut max_pass_output: f32 = 0.0;

        for i in 0..total_samples {
            #[allow(clippy::cast_precision_loss)]
            let t = (i as f32) * sample_interval_s;

            // Signal A: Pure 50Hz sine wave (Targeted for suppression)
            let notch_input = (2.0 * f32::PI * notch_frequency * t).sin();
            let notch_output = notch_signal_filter.update_notch(notch_input);

            // Signal B: Pure 5Hz sine wave (Far away from notch, should pass through)
            let pass_input = (2.0 * f32::PI * 5.0 * t).sin();
            let pass_output = pass_signal_filter.update_notch(pass_input);

            if i == 0 {
                // Iteration 1 (t = 0.0): input is 0.0, everything stays at 0.0
                assert_eq!(notch_signal_filter.state.w1, 0.0);
                assert_eq!(notch_signal_filter.state.w2, 0.0);
            } else if i == 1 {
                let expected_w1_iter2 = -0.008_807_512;
                let expected_w2_iter2 = 0.009_260_75;

                assert!(
                    (notch_signal_filter.state.w1 - expected_w1_iter2).abs() < 1e-6,
                    "W1 mismatch. Found: {}, Expected: {}",
                    notch_signal_filter.state.w1,
                    expected_w1_iter2
                );
                assert!(
                    (notch_signal_filter.state.w2 - expected_w2_iter2).abs() < 1e-6,
                    "W2 mismatch. Found: {}, Expected: {}",
                    notch_signal_filter.state.w2,
                    expected_w2_iter2
                );
            }
            // Let the filter stabilize for the first 500 samples, then measure peak output
            if i > 500 {
                if notch_output.abs() > max_notch_output {
                    max_notch_output = notch_output.abs();
                }
                if pass_output.abs() > max_pass_output {
                    max_pass_output = pass_output.abs();
                }
            }
        }

        // --- Verifications ---

        // The 50Hz signal targeted by the notch should be heavily attenuated.
        assert!(
            max_notch_output < 0.05,
            "Notch frequency was not attenuated enough. Peak remaining: {max_notch_output}",
        );

        // The 5Hz signal is far below the notch and should pass through almost entirely unhindered.
        assert!(
            max_pass_output > 0.95,
            "Passthrough frequency was incorrectly dampened. Peak output: {max_pass_output}",
        );
    }

    #[test]
    fn test_notch_filter_reset() {
        #![allow(clippy::excessive_precision)]
        let sample_interval_s: f32 = 0.001; // 1 kHz sampling rate
        let notch_freq: f32 = 60.0; // 50 Hz power line hum filter
        let q_factor: f32 = 1.0; // Narrow notch width

        let mut filter = BiquadFilter::new().with_q(q_factor).with_sample_interval(sample_interval_s);

        // calculate the filter coefficients
        let coeffs = filter.calculate_notch_coefficients_assuming_q(notch_freq);

        // Coefficient Verification Statements ---
        assert_near!(0.844_549_36, coeffs.b0, 1e-6);
        assert_near!(0.844_549_36, coeffs.b2, 1e-6);
        assert_eq!(coeffs.b1, coeffs.a1); // Hard invariant for the optimized loop

        // Exact expected values from the Audio EQ Cookbook for 60Hz @ 1kHz, Q=1.0
        let expected_a1 = -1.570_485_8;
        let expected_a2 = 0.689_098_7; // This is pre-inverted in our generator loop

        assert!(
            (coeffs.a1 - expected_a1).abs() < 1e-5,
            "Coefficient a1 mismatch. Found: {}, Expected: {}",
            coeffs.a1,
            expected_a1
        );
        assert!(
            (coeffs.a2 - expected_a2).abs() < 1e-5,
            "Coefficient a2 mismatch. Found: {}, Expected: {}",
            coeffs.a2,
            expected_a2
        );

        // Commit verified coefficients to filter
        filter.set_coefficients(coeffs);

        // Iteration 1 State Verification ---
        _ = filter.update_notch(1.0);

        // w1 remains zero because (input - output) is exactly 0.0
        assert_near!(-0.244_132_79, filter.state.w1, 1e-5);

        // w2 = output * coeffs.a2 + input -> (1.0 * -0.68910035) + 1.0 = 0.31089965
        let expected_w2_iter1 = 0.262_571_45;
        assert!(
            (filter.state.w2 - expected_w2_iter1).abs() < 1e-5,
            "Iteration 1 w2 mismatch. Found: {}, Expected: {}",
            filter.state.w2,
            expected_w2_iter1
        );

        // Iteration 2 State Verification ---
        _ = filter.update_notch(1.0);

        // w1 shifts to capture the prior step's w2 value
        let expected_w1_iter2 = -0.364_968_1;
        assert!(
            (filter.state.w1 - expected_w1_iter2).abs() < 1e-5,
            "Iteration 2 w1 mismatch. Found: {}, Expected: {}",
            filter.state.w1,
            expected_w1_iter2
        );

        // w2 remains constant because input and output remain stable at 1.0
        let expected_w2_iter2 = 0.430_803_1;
        assert!(
            (filter.state.w2 - expected_w2_iter2).abs() < 1e-5,
            "Iteration 2 w2 mismatch. Found: {}, Expected: {}",
            filter.state.w2,
            expected_w2_iter2
        );

        // --- 4. Deep Clean / Reset Cycle Verification ---
        for _ in 0..8 {
            _ = filter.update_notch(1.0);
        }

        // Verify the states have evolved and are dirty
        assert!(filter.state.w1 != 0.0);
        assert!(filter.state.w2 != 0.0);

        // Execute reset
        filter.reset();

        // Verify states have returned cleanly to pure zero
        assert_eq!(filter.state.w1, 0.0);
        assert_eq!(filter.state.w2, 0.0);
    }
}
