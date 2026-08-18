pub fn sensor_age_ms(received_at_ns: u64, stamp_ns: u64) -> Option<u64> {
    received_at_ns.checked_sub(stamp_ns).map(|age| age / 1_000_000)
}
