pub trait BiquadFilterMethods: Sized {
    /// Calculates the sine and cosine of an angle simultaneously.
    /// Returns a tuple containing `(sin(self), cos(self))`.
    #[must_use]
    fn sin_cos(self) -> (Self, Self);
}

// Use the hardware-linked math methods in Standard Library
#[cfg(feature = "std")]
impl BiquadFilterMethods for f32 {
    #[inline]
    fn sin_cos(self) -> (Self, Self) {
        self.sin_cos()
    }
}

#[cfg(feature = "std")]
impl BiquadFilterMethods for f64 {
    #[inline]
    fn sin_cos(self) -> (Self, Self) {
        self.sin_cos()
    }
}

#[cfg(all(not(feature = "std"), feature = "libm"))]
impl BiquadFilterMethods for f32 {
    #[inline]
    fn sin_cos(self) -> (Self, Self) {
        libm::sincosf(self)
    }
}

#[cfg(all(not(feature = "std"), feature = "libm"))]
impl BiquadFilterMethods for f64 {
    #[inline]
    fn sin_cos(self) -> (Self, Self) {
        libm::sincos(self)
    }
}

#[cfg(all(feature = "vqm", not(feature = "std"), not(feature = "libm")))]
use vqm::MathMethods;

#[cfg(all(feature = "vqm", not(feature = "std"), not(feature = "libm")))]
impl BiquadFilterMethods for f32 {
    #[inline]
    fn sin_cos(self) -> (Self, Self) {
        MathMethods::sin_cos(self)
    }
}

#[cfg(all(feature = "vqm", not(feature = "std"), not(feature = "libm")))]
impl BiquadFilterMethods for f64 {
    #[inline]
    fn sin_cos(self) -> (Self, Self) {
        MathMethods::sin_cos(self)
    }
}
