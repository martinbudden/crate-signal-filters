#![doc = include_str!("../README.md")]
//#![cfg_attr(feature = "simd", feature(portable_simd))]
// Conventions for generics used in this crate:
//    T: general type, eg f32, Vector3, Vector3f32, Quaternion etc
//    R: real number type ie f32 or f64
//    F: filter type, eg Pt1Filter, BiquadFilter etc
//#![doc(html_math_jax_enabled)]
#![no_std]
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]
#![deny(clippy::panic)]
//#![deny(missing_docs)]
#![deny(
    missing_copy_implementations,
    missing_debug_implementations,
    trivial_casts,
    trivial_numeric_casts,
    unused_must_use,
    unused_extern_crates,
    unused_import_braces,
    unused_qualifications,
    unused_results
)]
#![warn(unused_results)]
#![warn(clippy::pedantic)]
#![warn(clippy::doc_paragraphs_missing_punctuation)]

mod biquad_filter;
mod circular_buffer;
mod filters;
mod median_filter;
mod moving_average_filter;
mod pt_filters;
mod rolling_buffer;
mod slew_filter;

pub use biquad_filter::BiquadFilter;
pub use biquad_filter::{BiquadFilterVector2f32, BiquadFilterVector3f32, BiquadFilterVector4f32, BiquadFilterf32};
pub use biquad_filter::{BiquadFilterVector2f64, BiquadFilterVector3f64, BiquadFilterVector4f64, BiquadFilterf64};

pub use circular_buffer::CircularBuffer;

pub use filters::{SignalFilter, UpdateFilter};

pub use median_filter::{MedianFilter3, MedianFilter5};
pub use median_filter::{MedianFilter3f32, MedianFilter5f32};
pub use median_filter::{MedianFilter3f64, MedianFilter5f64};

pub use moving_average_filter::MovingAverageFilter;
pub use moving_average_filter::{
    MovingAverageFilterVector2f32, MovingAverageFilterVector3f32, MovingAverageFilterVector4f32, MovingAverageFilterf32,
};
pub use moving_average_filter::{
    MovingAverageFilterVector2f64, MovingAverageFilterVector3f64, MovingAverageFilterVector4f64, MovingAverageFilterf64,
};

pub use pt_filters::Pt1Filter;
pub use pt_filters::{Pt1FilterVector2f32, Pt1FilterVector3f32, Pt1FilterVector4f32, Pt1Filterf32};
pub use pt_filters::{Pt1FilterVector2f64, Pt1FilterVector3f64, Pt1FilterVector4f64, Pt1Filterf64};

pub use pt_filters::Pt2Filter;
pub use pt_filters::{Pt2FilterVector2f32, Pt2FilterVector3f32, Pt2FilterVector4f32, Pt2Filterf32};
pub use pt_filters::{Pt2FilterVector2f64, Pt2FilterVector3f64, Pt2FilterVector4f64, Pt2Filterf64};

pub use pt_filters::Pt3Filter;
pub use pt_filters::{Pt3FilterVector2f32, Pt3FilterVector3f32, Pt3FilterVector4f32, Pt3Filterf32};
pub use pt_filters::{Pt3FilterVector2f64, Pt3FilterVector3f64, Pt3FilterVector4f64, Pt3Filterf64};

pub use rolling_buffer::RollingBuffer;

pub use slew_filter::{LimitSlew, SlewRateLimiter, SlewRateLimiterf32, SlewRateLimiterf64};
