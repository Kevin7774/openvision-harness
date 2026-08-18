uint8 STATE_UNKNOWN=0
uint8 STATE_CONNECTING=1
uint8 STATE_CONNECTED=2
uint8 STATE_DISCONNECTED=3
uint8 STATE_FAILED=4
uint8 STATE_CLOSED=5

uint8 state
string session_id
string peer_id
string purpose
string run_id
string resource_id
string[] media_tracks
string[] data_channels
uint64 steady_time_ns
string reason_code
