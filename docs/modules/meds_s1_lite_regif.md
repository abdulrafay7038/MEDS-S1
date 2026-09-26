# `meds_s1_lite_regif`

| | |
|---|---|
| **Status** | COMPLETE — T-05 reference adapter |
| **Project** | T-05, consumed by M-01 · M-02 · M-03 · R-07 |
| **Spec** | INTERFACES.md §3 (I4), SPEC §24, ADR-0005 |
| **Source** | `rtl/peripherals/meds_s1_lite_regif.sv` |
| **Testbench** | `verif/unit/tb_meds_s1_lite_regif.sv`|

## Purpose

The AXI4-Lite slave handshake, written once. Every peripheral on the MEDS-S1 peripheral subtree
instantiates this and then implements only a register file — a combinational read decode and a
strobed write decode. Nobody in this repository writes AXI4-Lite handshaking twice.

It also absorbs the one consequence of a 64-bit peripheral bus (ADR-0005): a 32-bit peripheral sets
`REG_DW = 32`, never sees the upper half of the bus, and is byte-for-byte what it would have been on
a 32-bit bus. CLINT sets `REG_DW = 64` because `mtime` and `mtimecmp` must move in one access.

This is the most reused module in T-05. Build it first.

## Interface contract

### Bus side — I4, frozen

| Signal | Dir | Width | Meaning | Contract |
|---|---|---|---|---|
| `clk_i` | in | 1 | clock | single domain |
| `rst_ni` | in | 1 | reset | async assert, sync de-assert |
| `lite_req_i` | in | `lite_req_t` | master-driven channels | `valid` stable with payload until `ready` |
| `lite_rsp_o` | out | `lite_rsp_t` | slave-driven channels | no `ready` depends on its own `valid`; no `valid` depends on a `ready` (R-C10) |

`AW` and `W` must be accepted **independently and in either order**. A slave that requires `AW`
first deadlocks against a master that presents `W` first, which AXI4-Lite permits.

### Register-file side — what a peripheral implements

| Signal | Dir | Width | Meaning | Contract |
|---|---|---|---|---|
| `addr_o` | out | `ADDR_W` | **byte** offset in the window, aligned down to `REG_DW/8` | a peripheral decodes it against the literal offsets in its register-map table |
| `we_o` | out | 1 | write strobe | one cycle; never high with `re_o` |
| `re_o` | out | 1 | read strobe | one cycle; only needed for read-side-effect registers |
| `wdata_o` | out | `REG_DW` | write data, already shifted out of its bus lane | |
| `wstrb_o` | out | `REG_DW/8` | byte enables | a peripheral must honour these |
| `rdata_i` | in | `REG_DW` | read data | **combinational**, valid in the same cycle as `re_o` |
| `err_i` | in | 1 | "nothing is mapped at `addr_o`" | combinational from `addr_o`; becomes `SLVERR` |

**Backpressure:** responses are held until accepted, with the payload stable.
**Reset state:** all `valid` low, all `ready` low, no register access issued.
**Latency:** an `AW`/`W` pair executes on the clock edge after the later of its two channel
handshakes; an `AR` executes on the clock edge after its handshake. `B`/`R` becomes valid immediately
after that execution edge. The register-file strobe and read-data sampling occur in that execution
cycle.

**Throughput:** the register-file side executes at most one access per clock. With an accepting
master, alternating reads and writes can sustain one access per clock after the holding registers fill.
A stream of only reads or only writes executes every other clock because its one-entry response holder
is occupied until the following response handshake. When both directions are eligible, a one-bit
round-robin grant chooses the direction opposite the prior grant; therefore neither direction can
starve while its response channel can make progress.

**Known read-width limitation:** AXI4-Lite read requests in `lite_req_t` carry neither byte strobes nor
an access-size field. For `REG_DW = 32`, this adapter returns the lane selected by the address but
cannot distinguish a legal 32-bit read from an unsupported 64-bit read at that address. The upstream
width/alignment check must reject the latter under rule P3; write requests are distinguishable because
their strobes are present and are rejected here when they span both lanes.

## Parameters

| Parameter | Default | Legal range | Effect |
|---|---|---|---|
| `ADDR_W` | 16 | 3 … `LITE_AW` | window size in address bits; 64 KiB → 16, 4 MiB → 22. Comes from the region's `size` in `configs/*.yaml` |
| `REG_DW` | 32 | 32 or 64 | register-file width. Must be `LITE_DW` or `LITE_DW/2`; fail at elaboration otherwise |


## Verification status

| Layer | Status | Where |
|---|---|---|
| Lint | clean | `make lint` |
| Unit test | directed, backpressure, error, arbitration, and seeded-random sweeps at `REG_DW = 32` and `64` | `verif/unit/tb_meds_s1_lite_regif.sv` |
| Mutation | — | run it at the G1 review; see the testbench header |
