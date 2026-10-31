D.9.7B.2 — Discovery handshake fix (SIM ONLY)
- The pulse waits for exactly one /d97b2/summit0/request subscriber (up to 8 s) and verifies stability.
- Relay records requests_received / nonzero_requests_received in JSON.
- No changes to command speeds, timeout policy, or D8.
- Only run against stationary seed100 Gazebo with independent relay active, no D8 follower.
