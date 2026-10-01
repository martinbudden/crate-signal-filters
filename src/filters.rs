/// Note the "filter" function is called "update" rather than "apply" or "filter".
/// This is because update implies the filter has internal state whereas
/// apply sometimes implies a "pure" mathematical function with no memory.
///
/// `filter.update()` reads better than `filter.filter()`
/// and it also avoids confusion with the filter function in the Iterator trait.
use vqm::{Vector2f32, Vector2f64, Vector3f32, Vector3f64, Vector4f32, Vector4f64};

#[allow(clippy::doc_paragraphs_missing_punctuation)]
/// Filter Definition trait.
/// `filter.reset()`
/// `filter.update(value)`
/// ```
///
/// use signal_filters::{Pt2Filterf32, SignalFilter};
///
/// let mut filter = Pt2Filterf32::new().with_k(0.25);
/// let mut value:f32 = 1.0;
///
/// value = filter.update(value);
///
/// assert_eq!(0.0625, value);
/// ```
pub trait SignalFilter<T, F> {
    fn reset(&mut self);
    fn reset_to_value(&mut self, value: T);
    fn update(&mut self, input: T) -> T;
}

// `T` is the type being filtered, so it might be an `f32` or a `Vector3f32`
// `R` is the type of the filter's internal constant. It is either `f32` or `f64`
// and should correspond to `T`.
//
// So:
// * If `T` is `f32`, `R` is `f32`.
//
//  * If `T` is `f64`, `R` is `f64`.
// * If `T` is `Vector3f32`, `R` is `f32`
// * If `T` is `Vector3f64`, `R` is `f64`

/// Adds `value.filter_using(&mut filter)` method call syntax to `SignalFilter`.
/// ```
/// use signal_filters::{Pt2Filterf32, UpdateFilter};
/// let mut filter = Pt2Filterf32::new().with_k(0.25);
/// let value: f32 = 1.0;
///
/// let value = value.filter_using(&mut filter);
///
/// assert_eq!(0.0625, value);
/// ```
pub trait UpdateFilter<T, R> {
    #[must_use]
    fn filter_using<F: SignalFilter<T, R>>(self, filter: &mut F) -> Self;
}

impl UpdateFilter<f32, f32> for f32 {
    fn filter_using<F: SignalFilter<f32, f32>>(self, filter: &mut F) -> Self {
        // self is f32, filter.update takes and returns f32
        filter.update(self)
    }
}

impl UpdateFilter<f64, f64> for f64 {
    fn filter_using<F: SignalFilter<f64, f64>>(self, filter: &mut F) -> Self {
        // self is f64, filter.update takes and returns f64
        filter.update(self)
    }
}

impl UpdateFilter<Vector2f32, f32> for Vector2f32 {
    fn filter_using<F: SignalFilter<Vector2f32, f32>>(self, filter: &mut F) -> Self {
        // self is Vector2f32, filter.update handles the whole vector at once
        filter.update(self)
    }
}

impl UpdateFilter<Vector2f64, f64> for Vector2f64 {
    fn filter_using<F: SignalFilter<Vector2f64, f64>>(self, filter: &mut F) -> Self {
        // self is Vector2f64, filter.update handles the whole vector at once
        filter.update(self)
    }
}

impl UpdateFilter<Vector3f32, f32> for Vector3f32 {
    fn filter_using<F: SignalFilter<Vector3f32, f32>>(self, filter: &mut F) -> Self {
        // self is Vector3f32, filter.update handles the whole vector at once
        filter.update(self)
    }
}

impl UpdateFilter<Vector3f64, f64> for Vector3f64 {
    fn filter_using<F: SignalFilter<Vector3f64, f64>>(self, filter: &mut F) -> Self {
        // self is Vector3f64, filter.update handles the whole vector at once
        filter.update(self)
    }
}

impl UpdateFilter<Vector4f32, f32> for Vector4f32 {
    fn filter_using<F: SignalFilter<Vector4f32, f32>>(self, filter: &mut F) -> Self {
        // self is Vector4f32, filter.update handles the whole vector at once
        filter.update(self)
    }
}

impl UpdateFilter<Vector4f64, f64> for Vector4f64 {
    fn filter_using<F: SignalFilter<Vector4f64, f64>>(self, filter: &mut F) -> Self {
        // self is Vector4f64, filter.update handles the whole vector at once
        filter.update(self)
    }
}
#[cfg(any(debug_assertions, test))]
mod tests {
    #[allow(unused)]
    use super::*;

    fn _is_normal<T: Sized + Send + Sync + Unpin>() {}
    fn _is_full<T: Sized + Send + Sync + Unpin + Clone + Copy + Default + PartialEq>() {}

    #[test]
    fn normal_types() {}

    #[test]
    fn default() {}
}
