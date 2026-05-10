# task68 SCAN_RSP FAIL-A evidence — 2026-05-10

Binary/worktree: `/tmp/ch32-scan-rsp-reply`
Commit: `7833c67 wip(ble): reply to scan requests in phase1 adv`
Refs: `bad/scan-rsp-fail-a-20260510-7833c67` branch + tag

Key verdict input:
- `task68_scanrsp_20260510_2309_connectloop20_sniff.pcap`
- `task68_scanrsp_20260510_2309_connectloop20_connect_loop.log`

Observed in connect-loop pcap:
- CONNECT_IND: 0
- ADV_IND: 1924
- SCAN_REQ targeting `12:87:65:43:21:C2`: 14
- SCAN_RSP from target: 0
- Result: §6.1 FAIL-A statistically significant per Lucy verdict gate.

Additional traffic-low setup runs:
- `task68_scanrsp_20260510_2200_scanrsp_sniff60.pcap`: 12 SCAN_REQ / 60s, 0 SCAN_RSP
- `task68_scanrsp_20260510_2204_scanrsp180_sniff180.pcap`: 19 SCAN_REQ / 180s, 0 SCAN_RSP
