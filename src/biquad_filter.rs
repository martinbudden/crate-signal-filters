use core::ops::{Add, Div, Mul, Neg, Sub};
use num_traits::{ConstOne, ConstZero, MulAdd, One, Zero};
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};
use vqm::{MathConstants, SqrtMethods, TrigonometricMethods, Vector2d, Vector3d, Vector4d};

use crate::SignalFilter;

/// `BiquadFilter` for `f32`<br>
pub type BiquadFilterf32 = BiquadFilter<f32, f32>;
/// `BiquadFilter` for `Vector2df32`<br>
pub type BiquadFilterVector2df32 = BiquadFilter<Vector2d<f32>, f32>;
/// `BiquadFilter` for `Vector3df32`<br>
pub type BiquadFilterVector3df32 = BiquadFilter<Vector3d<f32>, f32>;
/// `BiquadFilter` for `Vector4df32`<br>
pub type BiquadFilterVector4df32 = BiquadFilter<Vector4d<f32>, f32>;

/// `BiquadFilter` for `f64`<br><br>
pub type BiquadFilterf64 = BiquadFilter<f64, f64>;
/// `BiquadFilter` for `Vector2df64`<br><br>
pub type BiquadFilterVector2df64 = BiquadFilter<Vector2d<f64>, f64>;
/// `BiquadFilter` for `Vector3df64`<br><br>
pub type BiquadFilterVector3df64 = BiquadFilter<Vector3d<f64>, f64>;
/// `BiquadFilter` for `Vector4df64`<br><br>
pub type BiquadFilterVector4df64 = BiquadFilter<Vector4d<f64>, f64>;

#[allow(clippy::doc_paragraphs_missing_punctuation)]
/// Second-order biquad IIR filter.<br><br>
///
/// This implementation uses the Direct Form II Transposed (DF2T) structure.
/// This minimizes floating-point rounding noise and requires only two internal state variables `w1` and `w2`.
/// The difference equation is:
///
/// ```math
/// y[n] = b{0} * x[n] + w{1}[n-1]
/// w{1}[n] = b{1} * x[n] -a{1} * y[n] + w{2}[n-1]
/// w{2}[n] = b{2} * x[n] -a{2} * y[n]
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
    T: Copy + ConstZero,
    R: Copy + Zero + One + ConstZero + ConstOne + MathConstants + Div<R, Output = R>,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<T, R> BiquadFilter<T, R>
where
    T: Copy + ConstZero,
    R: Copy + Zero + One + ConstZero + ConstOne + MathConstants + Div<R, Output = R>,
{
    pub const fn new() -> Self {
        Self::with_coefficients(BiquadFilterCoefficients::new())
    }

    pub const fn with_coefficients(coeffs: BiquadFilterCoefficients<R>) -> Self {
        Self {
            state: BiquadFilterState::new(),
            coeffs,
            weight: R::ONE,
            loop_time_seconds: R::ZERO,
            two_pi_loop_time_seconds: R::ZERO,
            q: R::ONE,
            one_over_2q: R::HALF,
        }
    }

    pub fn with_q(q: R) -> Self {
        let mut filter = Self::new();
        filter.set_q(q);
        filter
    }

    pub fn with_q_and_sample_rate(q: R, sample_rate_hz: R) -> Self {
        let mut filter = Self::new();
        filter.set_q(q);
        filter.set_sample_rate_hz(sample_rate_hz);
        filter
    }
}

impl<T, R> BiquadFilter<T, R>
where
    T: Copy,
    R: Copy + Zero + One + MathConstants + Div<R, Output = R>,
{
    pub fn set_q(&mut self, q: R) {
        self.q = q;
        self.one_over_2q = R::one() / ((R::one() + R::one()) * q); // cache value for faster setting of frequencies
    }

    pub fn set_loop_time(&mut self, loop_time_seconds: R) {
        self.loop_time_seconds = loop_time_seconds;
        self.two_pi_loop_time_seconds = (R::one() + R::one()) * R::PI * loop_time_seconds; // cache value for faster setting of frequencies
    }

    pub fn set_sample_rate_hz(&mut self, sample_rate_hz: R) {
        self.set_loop_time(R::one() / sample_rate_hz);
    }
}

/// NOTE: b2 == b0 for lowpass filter.
/// NOTE: b2 == b0 and a1 == b1 for notch filter.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct BiquadFilterCoefficients<R> {
    pub a1: R,
    pub a2: R,
    pub b0: R,
    pub b1: R,
    pub b2: R,
}

impl<T> Default for BiquadFilterCoefficients<T>
where
    T: Copy + ConstZero + ConstOne,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<R> BiquadFilterCoefficients<R>
where
    R: Copy + ConstZero + ConstOne,
{
    pub const fn new() -> Self {
        Self { a1: R::ZERO, a2: R::ZERO, b0: R::ONE, b1: R::ZERO, b2: R::ZERO }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BiquadFilterState<T> {
    pub w1: T,
    pub w2: T,
}

impl<T> BiquadFilterState<T>
where
    T: Copy + ConstZero,
{
    pub const fn new() -> Self {
        Self { w1: T::ZERO, w2: T::ZERO }
    }
}

impl<T> Default for BiquadFilterState<T>
where
    T: Copy + ConstZero,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<T, R> SignalFilter<T, R> for BiquadFilter<T, R>
where
    T: Copy + Zero + Sub<Output = T> + Mul<R, Output = T> + MulAdd<R, T, Output = T>,
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

    fn update(&mut self, input: T) -> T {
        // Uses Direct Form II (note: Direct Form I uses at least 9 operations: 5 multiplications, 4 additions).

        // 6 operations: 3 mul_adds, 2 multiplications, 1 subtractions
        // Calculate the current output sample: y[n] = b0 * x[n] + w1
        let output = input.mul_add(self.coeffs.b0, self.state.w1);

        // Calculate the next w1 state: w1 = b1 * x[n] - a1 * y[n] + w2
        self.state.w1 = input.mul_add(self.coeffs.b1, self.state.w2) - output * self.coeffs.a1;

        // Calculate the next w2 state: w2 = b2 * x[n] - a2 * y[n]
        self.state.w2 = input.mul_add(self.coeffs.b2, output * -self.coeffs.a2);
        output
    }
}

/// Biquad update notch.
///
/// For a notch filter a1 == b1 and b0 == b2, so this optimized form can be used.
impl<T, R> BiquadFilter<T, R>
where
    T: Copy + Zero + Add<Output = T> + Sub<Output = T> + Mul<R, Output = T> + MulAdd<R, T, Output = T>,
    R: Copy + Zero + One + Sub<Output = R> + Neg<Output = R>,
{
    pub fn update_notch(&mut self, input: T) -> T {
        /*
        // 6 operations: 2 mul_add, 2 multiplications, 2 addition/subtractions
        // Calculate output: y[n] = b0 * x[n] + w1
        let output = input.mul_add(self.coeffs.b0, self.state.w1);

        // Calculate w1 = b1 * x[n] - a1 * y[n] + w2
        // Refactored as w1 = (x[n] - y[n]) * a1 + w2
        self.state.w1 = (input - output).mul_add(self.coeffs.a1, self.state.w2);

        // Calculate w2 = b2 * x[n] - a2 * y[n]
        // Because b2 == b0, this expands to: w2 = b0 * x[n] - a2 * y[n]
        self.state.w2 = input * self.coeffs.b0 - output * self.coeffs.a2;
        */

        // 4 operations: 2 mul_add, 2 addition/subtractions, (negation not used, since compiler will generate a fused multiply-subtract instruction).
        let b0i = input * self.coeffs.b0;
        let output = b0i + self.state.w1;
        self.state.w1 = (input - output).mul_add(self.coeffs.a1, self.state.w2);
        self.state.w2 = output.mul_add(-self.coeffs.a2, b0i); // compiler automatically converts this to fused multiply-subtract

        output
    }
}

impl<T, R> BiquadFilter<T, R>
where
    T: Copy,
    R: Copy + Zero + One,
{
    pub fn set_weight(&mut self, weight: R) {
        self.weight = weight;
    }

    pub fn weight(&self) -> R {
        self.weight
    }

    pub fn set_coefficients(&mut self, coeffs: BiquadFilterCoefficients<R>) {
        self.coeffs = coeffs;
    }

    pub fn set_coefficients_and_weight(&mut self, coeffs: BiquadFilterCoefficients<R>, weight: R) {
        self.weight = weight;
        self.coeffs = coeffs;
    }

    /// Copy parameters from another Biquad filter.
    pub fn set_parameters_from(&mut self, other: &BiquadFilter<T, R>) {
        self.weight = other.weight;
        self.coeffs = other.coeffs;
    }

    pub fn calculate_omega(&self, frequency: R) -> R {
        frequency * self.two_pi_loop_time_seconds
    }

    pub fn q(&self) -> R {
        self.q
    }

    pub fn loop_time_seconds(&self) -> R {
        self.loop_time_seconds
    }

    pub fn state(self) -> BiquadFilterState<T> {
        self.state
    }
}

impl<T, R> BiquadFilter<T, R>
where
    T: Copy + Zero + Add<Output = T> + Sub<Output = T> + Mul<R, Output = T> + MulAdd<R, T, Output = T>,
    R: Copy
        + Zero
        + One
        + PartialOrd
        + Neg<Output = R>
        + MathConstants
        + TrigonometricMethods
        + SqrtMethods
        + Div<R, Output = R>
        + Sub<R, Output = R>,
{
    pub fn set_to_passthrough(&mut self) {
        self.coeffs.b0 = R::one();
        self.coeffs.b1 = R::zero();
        self.coeffs.b2 = R::zero();
        self.coeffs.a1 = R::zero();
        self.coeffs.a2 = R::zero();
        self.weight = R::one();
        self.reset();
    }

    pub fn update_weighted(&mut self, input: T) -> T {
        let output = self.update(input);
        // weight of 1.0 gives just output, weight of 0.0 gives just input
        (output - input).mul_add(self.weight, input)
    }

    pub fn update_notch_weighted(&mut self, input: T) -> T {
        let output = self.update_notch(input);
        // weight of 1.0 gives just output, weight of 0.0 gives just input
        (output - input).mul_add(self.weight, input)
    }

    pub fn init_low_pass(&mut self, frequency_hz: R, loop_time_seconds: R, q: R) {
        debug_assert!(q > R::zero());
        self.set_loop_time(loop_time_seconds);
        self.set_q(q);
        self.set_low_pass_frequency_assuming_q(frequency_hz);
        self.reset();
    }

    pub fn init_notch(&mut self, frequency_hz: R, loop_time_seconds: R, q: R) {
        debug_assert!(q > R::zero());
        self.set_loop_time(loop_time_seconds);
        self.set_q(q);
        self.set_notch_frequency_assuming_q(frequency_hz);
        self.reset();
    }

    /// Pre-charges the filter states based on the initial sample value.
    /// This drastically reduces startup transient ringing and settling time.
    pub fn pre_charge(&mut self, initial_value: T) {
        // For a DC input, steady state output equals input (unity gain).
        // Using: y[n] = b0*x + w1  =>  w1 = x * (1 - b0)
        self.state.w1 = initial_value * (R::one() - self.coeffs.b0);

        // Using: w2 = b2*x - a2*y  =>  Because x == y and b2 == b0:
        // w2 = x * (b0 - a2)
        self.state.w2 = initial_value * (self.coeffs.b0 - self.coeffs.a2);
    }

    //Note: weight must be in range [0, 1].
    pub fn calculate_low_pass_coefficients_assuming_q(&mut self, frequency_hz: R) -> BiquadFilterCoefficients<R> {
        let omega = frequency_hz * self.two_pi_loop_time_seconds;
        let (sin_omega, cos_omega) = omega.sin_cos();
        let alpha = sin_omega * self.one_over_2q;
        let a0_reciprocal = R::one() / (R::one() + alpha);

        let b1 = (R::one() - cos_omega) * a0_reciprocal;
        let b0 = self.coeffs.b1 * (R::one() / (R::one() + R::one()));
        BiquadFilterCoefficients {
            b0,
            b1,
            b2: b0,
            a1: -(R::one() + R::one()) * cos_omega * a0_reciprocal,
            a2: (R::one() - alpha) * a0_reciprocal,
        }
    }

    pub fn set_low_pass_frequency_weighted_assuming_q(&mut self, frequency_hz: R, weight: R) {
        self.weight = weight;
        self.coeffs = self.calculate_low_pass_coefficients_assuming_q(frequency_hz);
    }

    pub fn set_low_pass_frequency_assuming_q(&mut self, frequency_hz: R) {
        self.set_low_pass_frequency_weighted_assuming_q(frequency_hz, R::one());
    }

    pub fn calculate_notch_coefficients_from_sin_cos_assuming_q(
        &mut self,
        sin_omega: R,
        cos_omega: R,
    ) -> BiquadFilterCoefficients<R> {
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
    pub fn set_notch_frequency_weighted_from_sin_cos_assuming_q(&mut self, sin_omega: R, cos_omega: R, weight: R) {
        self.weight = weight;
        self.coeffs = self.calculate_notch_coefficients_from_sin_cos_assuming_q(sin_omega, cos_omega);
    }

    pub fn calculate_notch_coefficients_assuming_q(&mut self, frequency_hz: R) -> BiquadFilterCoefficients<R> {
        let omega = frequency_hz * self.two_pi_loop_time_seconds;
        let (sin_omega, cos_omega) = omega.sin_cos();
        self.calculate_notch_coefficients_from_sin_cos_assuming_q(sin_omega, cos_omega)
    }

    #[inline]
    pub fn set_notch_frequency_weighted_assuming_q(&mut self, frequency_hz: R, weight: R) {
        self.weight = weight;
        self.coeffs = self.calculate_notch_coefficients_assuming_q(frequency_hz);
    }

    #[inline]
    pub fn set_notch_frequency_assuming_q(&mut self, frequency_hz: R) {
        self.set_notch_frequency_weighted_assuming_q(frequency_hz, R::one());
    }

    #[inline]
    pub fn set_notch_frequency(&mut self, center_frequency_hz: R, lower_cutoff_frequency_hz: R) {
        self.set_q(Self::calculate_q(center_frequency_hz, lower_cutoff_frequency_hz));
        self.set_notch_frequency_assuming_q(center_frequency_hz);
    }

    pub fn calculate_q(center_frequency_hz: R, lower_cutoff_frequency_hz: R) -> R {
        center_frequency_hz * lower_cutoff_frequency_hz
            / (center_frequency_hz * center_frequency_hz - lower_cutoff_frequency_hz * lower_cutoff_frequency_hz)
    }

    pub fn set_q_from_frequencies(&mut self, center_frequency_hz: R, lower_cutoff_frequency_hz: R) {
        self.set_q(Self::calculate_q(center_frequency_hz, lower_cutoff_frequency_hz));
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

    #[allow(unused)]
    fn is_normal<T: Sized + Send + Sync + Unpin>() {}
    #[allow(unused)]
    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_full::<BiquadFilter<f32, f32>>();
        is_full::<BiquadFilterf32>();
        is_full::<BiquadFilterState<f32>>();
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
        use vqm::Vector3df32;
        let mut filter = BiquadFilterVector3df32::default();
        let mut state: BiquadFilterState<Vector3df32>;

        // test that filter with default settings performs no filtering
        let output = filter.update(Vector3df32 { x: 2.0, y: 3.0, z: 5.0 });
        assert_eq!(Vector3df32 { x: 2.0, y: 3.0, z: 5.0 }, output);
        state = filter.state();
        assert_eq!(0.0, state.w1.x);
        assert_eq!(0.0, state.w2.x);

        filter.reset();
        state = filter.state();
        assert_eq!(0.0, state.w1.x);
        assert_eq!(0.0, state.w2.x);
        assert_eq!(4.0, filter.update(Vector3df32 { x: 4.0, y: 0.0, z: 0.0 }).x);

        filter.set_coefficients_and_weight(
            BiquadFilterCoefficients { a1: 2.0, a2: 3.0, b0: 5.0, b1: 7.0, b2: 11.0 },
            13.0,
        );
        filter.set_to_passthrough();
        assert_eq!(1.0, filter.update(Vector3df32 { x: 1.0, y: 0.0, z: 0.0 }).x);
        assert_eq!(2.0, filter.update(Vector3df32 { x: 2.0, y: 0.0, z: 0.0 }).x);
        assert_eq!(1.0, filter.update_weighted(Vector3df32 { x: 1.0, y: 0.0, z: 0.0 }).x);
        assert_eq!(2.0, filter.update_weighted(Vector3df32 { x: 2.0, y: 0.0, z: 0.0 }).x);
    }
    #[test]
    fn test_notch_filter_attenuation_and_passthrough() {
        let sample_rate_hz: f32 = 1000.0; // 1 kHz sampling rate
        let notch_freq: f32 = 50.0; // 50 Hz powerline hum filter
        let q_factor: f32 = 10.0; // Narrow notch width

        // Initialize two identical filters to test different signals
        let mut notch_signal_filter = BiquadFilter::with_q_and_sample_rate(q_factor, sample_rate_hz);
        let mut pass_signal_filter = BiquadFilter::with_q_and_sample_rate(q_factor, sample_rate_hz);

        // Calculate filter coefficients
        let coeffs = notch_signal_filter.calculate_notch_coefficients_assuming_q(notch_freq);

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
            let t = (i as f32) / sample_rate_hz;

            // Signal A: Pure 50Hz sine wave (Targeted for suppression)
            let notch_input = (2.0 * f32::PI * notch_freq * t).sin();
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
        let sample_rate_hz: f32 = 1000.0; // 1 kHz sampling rate
        let notch_freq: f32 = 60.0; // 50 Hz powerline hum filter
        let q_factor: f32 = 1.0; // Narrow notch width

        let mut filter = BiquadFilter::with_q_and_sample_rate(q_factor, sample_rate_hz);

        // calculate the filter coefficients
        let coeffs = filter.calculate_notch_coefficients_assuming_q(notch_freq);

        // --- 1. Coefficient Verification Statements ---
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

        // --- 2. Iteration 1 State Verification ---
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

        // --- 3. Iteration 2 State Verification ---
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
