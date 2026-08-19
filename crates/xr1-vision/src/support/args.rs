//! Command-line argument parsing.
//!
//! Step 1 finding #4: inside the orchestration files "CLI argument parsing, JSON
//! parsing, evidence storage, locking, and the adapter subprocess protocol are
//! not separated." This module is the argument-parsing half of that separation.
//!
//! Before this existed, `cli.rs`, `servo_loop.rs`, and `grasp_loop.rs` each
//! carried their own byte-identical copies of `option`, `optional_option`, and
//! `flag`, plus a `validate_command_args` that differed only in one error label.
//! Three copies of an argument parser is three places for a gate to drift.

/// A required option's value.
pub fn option(args: &[String], name: &str) -> Result<String, String> {
    optional_option(args, name)?.ok_or_else(|| format!("missing {name}"))
}

/// An optional option's value. Rejects a repeated option rather than silently
/// taking the first, so `--go --go` can never mean something different by luck.
pub fn optional_option(args: &[String], name: &str) -> Result<Option<String>, String> {
    let mut values = args.windows(2).filter(|pair| pair[0] == name);
    let value = values.next().map(|pair| pair[1].clone());
    if values.next().is_some() {
        return Err(format!("{name} may only be supplied once"));
    }
    Ok(value)
}

/// Whether a valueless flag is present.
pub fn flag(args: &[String], name: &str) -> bool {
    args.iter().any(|argument| argument == name)
}

/// Reject unknown arguments and options whose value is missing or is itself an
/// option. `command` labels the error for the caller ("servo-loop",
/// "grasp-loop"); `None` produces the generic message the top-level CLI uses.
pub fn validate_command_args(
    command: Option<&str>,
    args: &[String],
    options: &[&str],
    flags: &[&str],
) -> Result<(), String> {
    let mut index = 0;
    while index < args.len() {
        let argument = args[index].as_str();
        if flags.contains(&argument) {
            index += 1;
        } else if options.contains(&argument) {
            let Some(value) = args.get(index + 1) else {
                return Err(format!("missing value after {argument}"));
            };
            if value.starts_with("--") {
                return Err(format!("missing value after {argument}"));
            }
            index += 2;
        } else {
            return Err(match command {
                Some(command) => format!("unsupported {command} argument {argument:?}"),
                None => format!("unsupported argument {argument:?}"),
            });
        }
    }
    Ok(())
}

/// A bounded integer option. The bound is enforced here so a caller cannot
/// forget it.
pub fn usize_option(
    args: &[String],
    name: &str,
    default: usize,
    minimum: usize,
    maximum: usize,
) -> Result<usize, String> {
    let value = optional_option(args, name)?
        .map(|raw| {
            raw.parse::<usize>()
                .map_err(|_| format!("{name} must be an integer"))
        })
        .transpose()?
        .unwrap_or(default);
    if !(minimum..=maximum).contains(&value) {
        return Err(format!("{name} must be within [{minimum}, {maximum}]"));
    }
    Ok(value)
}

/// A bounded, finite float option. Non-finite input is rejected: a `NaN` bound
/// would make every downstream comparison silently false.
pub fn f64_option(
    args: &[String],
    name: &str,
    default: f64,
    minimum: f64,
    maximum: f64,
) -> Result<f64, String> {
    let value = optional_option(args, name)?
        .map(|raw| {
            raw.parse::<f64>()
                .map_err(|_| format!("{name} must be a number"))
        })
        .transpose()?
        .unwrap_or(default);
    if !value.is_finite() || !(minimum..=maximum).contains(&value) {
        return Err(format!(
            "{name} must be finite and within [{minimum}, {maximum}]"
        ));
    }
    Ok(value)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args(values: &[&str]) -> Vec<String> {
        values.iter().map(|value| value.to_string()).collect()
    }

    #[test]
    fn repeated_option_is_rejected() {
        let args = args(&["--proposal", "a.json", "--proposal", "b.json"]);
        assert!(optional_option(&args, "--proposal").is_err());
    }

    #[test]
    fn missing_required_option_is_rejected() {
        assert!(option(&args(&[]), "--calibration").is_err());
    }

    #[test]
    fn unknown_or_value_less_options_are_rejected() {
        assert!(validate_command_args(
            Some("servo-loop"),
            &args(&["--calibration", "c.json", "--go"]),
            &["--calibration"],
            &["--go"]
        )
        .is_ok());
        assert!(validate_command_args(None, &args(&["--oops"]), &[], &[]).is_err());
        assert!(
            validate_command_args(None, &args(&["--calibration"]), &["--calibration"], &[]).is_err()
        );
    }

    #[test]
    fn an_option_value_that_looks_like_an_option_is_rejected() {
        assert!(validate_command_args(
            None,
            &args(&["--calibration", "--go"]),
            &["--calibration"],
            &["--go"]
        )
        .is_err());
    }

    #[test]
    fn the_command_label_appears_in_the_error() {
        let error =
            validate_command_args(Some("grasp-loop"), &args(&["--oops"]), &[], &[]).unwrap_err();
        assert!(error.contains("grasp-loop"));
    }

    #[test]
    fn bounded_numeric_options_enforce_their_range_and_finiteness() {
        assert_eq!(usize_option(&args(&[]), "--steps", 20, 1, 40).unwrap(), 20);
        assert!(usize_option(&args(&["--steps", "99"]), "--steps", 20, 1, 40).is_err());
        assert!(usize_option(&args(&["--steps", "x"]), "--steps", 20, 1, 40).is_err());
        assert!(f64_option(&args(&["--t", "nan"]), "--t", 1.0, 0.0, 9.0).is_err());
        assert!(f64_option(&args(&["--t", "5.0"]), "--t", 1.0, 0.0, 9.0).is_ok());
    }
}
