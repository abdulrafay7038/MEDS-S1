// Copyright 2026 Maktab-e-Digital Systems Lahore.
// Licensed under the Apache License, Version 2.0, see LICENSE file for details.
// SPDX-License-Identifier: Apache-2.0
//
// =============================================================================
// meds_s1_lite_regif : AXI4-Lite slave -> register-file adapter [COMPLETE]
//
// Shared AXI4-Lite adapter for peripheral register files. Independent AW, W,
// and AR buffers accept requests in either order. A round-robin arbiter serves
// one register access per cycle; registered B and R responses hold their
// payloads until accepted. Supports 32-bit and 64-bit registers on the I4 bus.
//
// Register-file contract (port list frozen):
//
//   we_o / re_o   one-cycle strobes, never both in the same cycle
//   addr_o        BYTE offset inside the window, aligned down to REG_DW/8, so a
//                 peripheral's decode is a case on the offsets in its own
//                 register-map table -- no shifting, no slot arithmetic
//   wdata_o       write data, already shifted out of its bus lane
//   wstrb_o       byte enables for that register
//   rdata_i       COMBINATIONAL read data, valid in the same cycle as re_o
//   err_i         combinational from addr_o: "nothing is mapped here"
//
// Reference: INTERFACES.md section 3 (I4), ADR-0005.
// Full contract: docs/modules/meds_s1_lite_regif.md.
// Testbench: verif/unit/tb_meds_s1_lite_regif.sv
// =============================================================================

module meds_s1_lite_regif
  import meds_s1_lite_pkg::*;
#(
  // Window size in address bits: 64 KiB -> 16, 4 MiB -> 22.  Comes from the
  // region's `size` in configs/*.yaml; never hand-written in two places.
  parameter int unsigned ADDR_W = 16,
  // Register-file width.  Must be LITE_DW (64) or LITE_DW/2 (32).
  parameter int unsigned REG_DW = 32
) (
  input  logic                clk_i,
  input  logic                rst_ni,

  // ---- I4 bus port.  FROZEN.  Do not add signals here. ----------------------
  input  lite_req_t           lite_req_i,
  output lite_rsp_t           lite_rsp_o,

  // ---- register-file port.  FROZEN.  This is the side a peripheral sees. ----
  output logic [ADDR_W-1:0]   addr_o,
  output logic                we_o,
  output logic                re_o,
  output logic [REG_DW-1:0]   wdata_o,
  output logic [REG_DW/8-1:0] wstrb_o,
  input  logic [REG_DW-1:0]   rdata_i,
  input  logic                err_i
);

  localparam int unsigned REG_SW         = REG_DW / 8;
  localparam int unsigned REG_ALIGN_BITS = $clog2(REG_SW);

  // Each AXI channel has exactly one place to wait.  In particular AW and W
  // have separate places, which is what makes W-before-AW safe.
  logic          aw_full_q, w_full_q, ar_full_q;
  // READY remains low until the first clock edge after reset release.
  logic          active_q;
  lite_addr_t    aw_addr_q, ar_addr_q;
  lite_data_t    wdata_q;
  lite_strb_t    wstrb_q;

  logic          b_full_q, r_full_q;
  lite_resp_t    bresp_q, rresp_q;
  lite_data_t    rdata_q;

  // A tie is resolved in the direction opposite the previous grant.  This is
  // only changed by a real register-file access, so a continuously queued read
  // and write alternate rather than allowing either to starve the other.
  logic          last_grant_write_q;

  logic          write_candidate, read_candidate;
  logic          execute_write, execute_read;
  logic          write_spans_lanes, write_error;
  logic [ADDR_W-1:0] access_addr;
  lite_data_t    read_bus_data;

  // These are parameter errors, rather than behaviours which can be decoded
  // meaningfully at run time.  Keep the range check next to the width check so
  // a bad window cannot create an invalid part-select below.
  if ((ADDR_W < 3) || (ADDR_W > LITE_AW)) begin : gen_bad_addr_width
    $error("meds_s1_lite_regif: ADDR_W (%0d) must be in [3, %0d]",
           ADDR_W, LITE_AW);
  end
  if ((REG_DW != LITE_DW) && (REG_DW != (LITE_DW / 2))) begin : gen_bad_reg_width
    $error("meds_s1_lite_regif: REG_DW (%0d) must be %0d or %0d",
           REG_DW, LITE_DW / 2, LITE_DW);
  end

  // A held response occupies its own channel until accepted.  No response
  // valid, payload, or request ready is combinationally dependent on a
  // response ready signal (R-C10).
  always_comb begin
    lite_rsp_o = LITE_RSP_IDLE;
    if (active_q) begin
      lite_rsp_o.aw_ready = !aw_full_q;
      lite_rsp_o.w_ready  = !w_full_q;
      lite_rsp_o.ar_ready = !ar_full_q;
    end
    lite_rsp_o.b_valid = b_full_q;
    lite_rsp_o.b.resp  = bresp_q;
    lite_rsp_o.r_valid = r_full_q;
    lite_rsp_o.r.resp  = rresp_q;
    lite_rsp_o.r.data  = rdata_q;
  end

  // Select at most one register-file access.  Address selection is deliberately
  // independent of err_i and rdata_i: a peripheral's combinational decode may
  // depend on addr_o, so mixing those paths would create a false combinational
  // loop in lint (and a very real integration trap for a future edit).
  always_comb begin
    write_candidate = aw_full_q && w_full_q && !b_full_q;
    read_candidate  = ar_full_q && !r_full_q;
    execute_write   = 1'b0;
    execute_read    = 1'b0;

    if (write_candidate && read_candidate) begin
      if (last_grant_write_q) begin
        execute_read = 1'b1;
      end else begin
        execute_write = 1'b1;
      end
    end else if (write_candidate) begin
      execute_write = 1'b1;
    end else if (read_candidate) begin
      execute_read = 1'b1;
    end
  end

  // The register-file address and write payload are live only while an access
  // is selected.  The address is an aligned byte offset, not a register index.
  always_comb begin
    access_addr = '0;
    if (execute_write) begin
      access_addr = aw_addr_q[ADDR_W-1:0];
    end else if (execute_read) begin
      access_addr = ar_addr_q[ADDR_W-1:0];
    end
    addr_o = access_addr;
    addr_o[REG_ALIGN_BITS-1:0] = '0;
    wdata_o = '0;
    wstrb_o = '0;
    if (execute_write) begin
      if (REG_DW == LITE_DW) begin
        wdata_o = wdata_q[REG_DW-1:0];
        wstrb_o = wstrb_q[REG_SW-1:0];
      end else if (aw_addr_q[REG_ALIGN_BITS]) begin
        wdata_o = wdata_q[LITE_DW-1 -: REG_DW];
        wstrb_o = wstrb_q[LITE_SW-1 -: REG_SW];
      end else begin
        wdata_o = wdata_q[REG_DW-1:0];
        wstrb_o = wstrb_q[REG_SW-1:0];
      end
    end
  end

  // For a 32-bit register file, the two halves of the 64-bit bus are both
  // structurally valid ranges.  A write selecting bytes in both is an
  // unsupported 8-byte access and must not reach the register file.
  always_comb begin
    write_spans_lanes = 1'b0;
    if (REG_DW == (LITE_DW / 2)) begin
      write_spans_lanes = (|wstrb_q[(LITE_SW / 2)-1:0]) &&
                           (|wstrb_q[LITE_SW-1:LITE_SW / 2]);
    end
    write_error = err_i || write_spans_lanes;
    we_o        = execute_write && !write_error;
    re_o        = execute_read;
  end

  // rdata_i is sampled into the registered R channel at the end of a read
  // access.  Keeping this in its own combinational block makes the peripheral
  // decode path one-way: addr_o -> rdata_i -> R payload.
  always_comb begin
    read_bus_data = '0;
    if (execute_read) begin
      if (REG_DW == LITE_DW) begin
        read_bus_data[REG_DW-1:0] = rdata_i;
      end else if (ar_addr_q[REG_ALIGN_BITS]) begin
        read_bus_data[LITE_DW-1 -: REG_DW] = rdata_i;
      end else begin
        read_bus_data[REG_DW-1:0] = rdata_i;
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      active_q            <= 1'b0;
      aw_full_q           <= 1'b0;
      w_full_q            <= 1'b0;
      ar_full_q           <= 1'b0;
      aw_addr_q           <= '0;
      ar_addr_q           <= '0;
      wdata_q             <= '0;
      wstrb_q             <= '0;
      b_full_q            <= 1'b0;
      r_full_q            <= 1'b0;
      bresp_q             <= RESP_OKAY;
      rresp_q             <= RESP_OKAY;
      rdata_q             <= '0;
      last_grant_write_q  <= 1'b0;
    end else begin
      active_q <= 1'b1;
      // Channel capture is deliberately independent: no channel waits for a
      // sibling channel before accepting its own transaction.
      if (lite_rsp_o.aw_ready && lite_req_i.aw_valid) begin
        aw_full_q <= 1'b1;
        aw_addr_q <= lite_req_i.aw.addr;
      end
      if (lite_rsp_o.w_ready && lite_req_i.w_valid) begin
        w_full_q <= 1'b1;
        wdata_q  <= lite_req_i.w.data;
        wstrb_q  <= lite_req_i.w.strb;
      end
      if (lite_rsp_o.ar_ready && lite_req_i.ar_valid) begin
        ar_full_q <= 1'b1;
        ar_addr_q <= lite_req_i.ar.addr;
      end

      if (b_full_q && lite_req_i.b_ready) begin
        b_full_q <= 1'b0;
      end
      if (r_full_q && lite_req_i.r_ready) begin
        r_full_q <= 1'b0;
      end

      if (execute_write) begin
        aw_full_q          <= 1'b0;
        w_full_q           <= 1'b0;
        b_full_q           <= 1'b1;
        bresp_q            <= write_error ? RESP_SLVERR : RESP_OKAY;
        last_grant_write_q <= 1'b1;
      end else if (execute_read) begin
        ar_full_q          <= 1'b0;
        r_full_q           <= 1'b1;
        rresp_q            <= err_i ? RESP_SLVERR : RESP_OKAY;
        rdata_q            <= read_bus_data;
        last_grant_write_q <= 1'b0;
      end
    end
  end

endmodule
