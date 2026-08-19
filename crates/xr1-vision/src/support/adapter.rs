//! The Python-adapter subprocess protocol.
//!
//! Step 1 finding #1 observed that the harness's real interfaces were "Python
//! filenames, environment variables, and JSON passed over subprocess
//! boundaries", and finding #4 that this protocol was not separated from the
//! loops that speak it. This module is that protocol, in one place: an adapter
//! prints JSON on stdout, and the last JSON line is its report.
//!
//! Keeping it here means the parsing rules — last line wins, `ok` must be
//! literally `true`, a missing string field is an error — cannot drift between
//! the servo loop, the grasp loop, and the CLI.

/// The adapter's report is the **last** JSON line on stdout. Scanning from the
/// end means incidental logging before the report cannot be mistaken for it.
pub fn parse_last_json(output: &str, source: &str) -> Result<serde_json::Value, String> {
    output
        .lines()
        .rev()
        .find_map(|line| serde_json::from_str::<serde_json::Value>(line).ok())
        .ok_or_else(|| format!("{source} produced no JSON report: {}", output.trim()))
}

/// A required string field. Absent or non-string is an error, never a default —
/// a silently defaulted frame id would let a stale frame pass a freshness gate.
pub fn json_string<'a>(value: &'a serde_json::Value, name: &str) -> Result<&'a str, String> {
    value
        .get(name)
        .and_then(serde_json::Value::as_str)
        .ok_or_else(|| format!("JSON report is missing string field {name}"))
}

/// Require `"ok": true` exactly. A truthy-looking value (`1`, `"true"`) is not
/// accepted: the adapter contract is a JSON boolean.
pub fn require_ok(report: &serde_json::Value, source: &str) -> Result<(), String> {
    if report.get("ok") == Some(&serde_json::Value::Bool(true)) {
        Ok(())
    } else {
        Err(format!("{source} failed: {report}"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_last_json_line_is_the_report() {
        let output = "starting up\n{\"ok\":false}\n{\"ok\":true,\"frame_id\":\"f-2\"}\n";
        let report = parse_last_json(output, "adapter").unwrap();
        assert!(require_ok(&report, "adapter").is_ok());
        assert_eq!(json_string(&report, "frame_id").unwrap(), "f-2");
    }

    #[test]
    fn output_without_json_is_an_error() {
        assert!(parse_last_json("no json here", "adapter").is_err());
    }

    #[test]
    fn a_truthy_non_boolean_ok_is_not_accepted() {
        for raw in ["{\"ok\":1}", "{\"ok\":\"true\"}", "{}"] {
            let report: serde_json::Value = serde_json::from_str(raw).unwrap();
            assert!(require_ok(&report, "adapter").is_err(), "{raw} must not pass");
        }
    }

    #[test]
    fn a_missing_or_non_string_field_is_an_error() {
        let report: serde_json::Value = serde_json::from_str("{\"frame_id\":7}").unwrap();
        assert!(json_string(&report, "frame_id").is_err());
        assert!(json_string(&report, "absent").is_err());
    }
}
