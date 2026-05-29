# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

Releases of the form `0.1.n` do not adhere to [Semantic Versioning](https://semver.org/spec/v2.0.0.html),
that is each release may contain incompatible API changes.

Once the API has stabilized this project will adopt semantic versioning, the first release to do so will be `0.2.0`.

## [Unreleased]

### Added

### Changed

### Removed

### Deprecated

### Fixed

### Security

## [0.1.7] - 2026-05-29

### Added

- Examples to readme.

### Changed

- Updated to vqm version 0.1.10.
- Renamed `set_loop_time` to `set_sample_interval`.
- Use sample_interval rather than sample_rate in constructors.

### Removed

- `katex-header.html`.

## [0.1.6] - 2026-05-23

- `set_notch_frequency_weighted_from_sin_cos_assuming_q` to `BiquadFilter`.
- `set_cutoff_frequency_and_reset` to `PtFilter`s.

### Changed

- Updated to `vqm` version 0.1.8.

### Added

## [0.1.5] - 2026-05-18

### Added

- `.cargo/config.toml`.
- `reset_to_value` function to `SignalFilter`.
- `with_state_and_k` constructor to PtFilters.
- `set_k_safe` function to PtFilters.
- `set_cutoff_frequency_seamless` function for `Pt1Filter` and `Pt2Filter`.
- optional `serde` support for serializing `BiquadFilterCoefficients`.

### Changed

- Updated to `vqm` version 0.1.7.
- Improved handling of features in `Cargo.toml`.
- Made `k` and `state` PtFilter accessor functions public.
- Used `mul_add` in PtFilter `update` functions for speed and accuracy.
- Changed `BiquadFilter` to use Direct Form II Transposed (DF2T) structure.
- Optimized `BiquadFilter` `update` function.
- Optimized `BiquadFilter` `update_notch` function.

## [0.1.4] - 2026-05-16

### Changed

- Use vqm version 0.1.5
- `new` and other constructors to be `const`.
- Grouped `BiquadFilter` coefficients into a `struct`.
- Improved documentation.

## [0.1.3] - 2026-05-06

### Changed

- Use vqm version 0.1.3

## [0.1.2] - 2026-05-02

### Changed

- Use vqm version 0.1.2

## [0.1.1] - 2026-04-26

### Added

- This changelog.
- `MedianFilter5`.

### Changed

- Improved documentation.

## [0.1.0] - 2023-04-12

Initial release.
