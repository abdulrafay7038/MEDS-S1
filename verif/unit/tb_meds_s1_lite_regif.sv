// Copyright 2026 Maktab-e-Digital Systems Lahore.
// Licensed under the Apache License, Version 2.0, see LICENSE file for details.
// SPDX-License-Identifier: Apache-2.0
//
// =============================================================================
// tb_meds_s1_lite_regif : unit testbench for the I4 bus adapter       [COMPLETE]
//
// Tests both supported register widths through one parameterised test case.
// The local register-file model is deliberately small: it only honours the
// adapter's strobes and reports an unmapped offset.  All response expectations
// come from the I4 contract, and the test case maintains a separate shadow
// model for its expected register contents.
// =============================================================================

module tb_meds_s1_lite_regif;

  logic clk;
  logic rst_n;

  logic        done_32, done_64;
  int unsigned checks_32, checks_64;
  int unsigned errors_32, errors_64;

  // Clock generator.  An `initial forever` rather than a bare `always`: the
  // coding standard has no bare-always form.
  initial begin
    clk = 1'b0;
    forever #5 clk = ~clk;
  end

  meds_s1_lite_regif_case #(
    .REG_DW (32)
  ) case_32 (
    .clk_i    (clk),
    .rst_ni   (rst_n),
    .done_o   (done_32),
    .checks_o (checks_32),
    .errors_o (errors_32)
  );

  meds_s1_lite_regif_case #(
    .REG_DW (64)
  ) case_64 (
    .clk_i    (clk),
    .rst_ni   (rst_n),
    .done_o   (done_64),
    .checks_o (checks_64),
    .errors_o (errors_64)
  );

  // A deadlocked handshake is a hang, not merely a bad datum.  Keep the
  // watchdog independent of the test-case timeouts so a new deadlock cannot
  // turn CI into an indefinitely running process.
  initial begin
    repeat (500000) @(posedge clk);
    $fatal(1, "tb_meds_s1_lite_regif: watchdog expired -- a handshake deadlocked");
  end

  initial begin
    rst_n = 1'b0;
    repeat (4) @(negedge clk);
    rst_n = 1'b1;

    wait (done_32 && done_64);
    #1;
    if ((errors_32 + errors_64) == 0) begin
      $display("=== PASS : %0d checks ===", checks_32 + checks_64);
      $finish;
    end else begin
      $display("=== FAIL : %0d errors of %0d checks ===",
               errors_32 + errors_64, checks_32 + checks_64);
      $fatal(1, "tb_meds_s1_lite_regif failed");
    end
  end

endmodule


// One copy of this case runs for each legal REG_DW.  Keeping the bus driver and
// test sequence here prevents a 64-bit configuration from becoming an
// unmaintained copy of the default testbench.
module meds_s1_lite_regif_case
  import meds_s1_lite_pkg::*;
#(
  parameter int unsigned ADDR_W = 16,
  parameter int unsigned REG_DW = 32
) (
  input  logic        clk_i,
  input  logic        rst_ni,
  output logic        done_o,
  output int unsigned checks_o,
  output int unsigned errors_o
);

  localparam int unsigned REG_SW         = REG_DW / 8;
  localparam int unsigned REG_BYTES      = REG_DW / 8;
  localparam int unsigned REG_ALIGN_BITS = $clog2(REG_BYTES);
  localparam int unsigned N_REGS         = 8;
  localparam int unsigned MAX_WAIT       = 32;

  lite_req_t req;
  lite_rsp_t rsp;
  logic [ADDR_W-1:0] addr;
  logic              we, re, err;
  logic [REG_DW-1:0] wdata, rdata;
  logic [REG_SW-1:0] wstrb;

  int unsigned checks;
  int unsigned errors;

  // This is the peripheral-side model.  It intentionally does not reuse the
  // adapter's lane-selection logic: it sees only the documented register-file
  // interface, exactly as a real peripheral does.
  logic [REG_DW-1:0] model  [0:N_REGS-1];
  logic [REG_DW-1:0] shadow [0:N_REGS-1];
  int unsigned       model_index;

  meds_s1_lite_regif #(
    .ADDR_W (ADDR_W),
    .REG_DW (REG_DW)
  ) dut (
    .clk_i      (clk_i),
    .rst_ni     (rst_ni),
    .lite_req_i (req),
    .lite_rsp_o (rsp),
    .addr_o     (addr),
    .we_o       (we),
    .re_o       (re),
    .wdata_o    (wdata),
    .wstrb_o    (wstrb),
    .rdata_i    (rdata),
    .err_i      (err)
  );

  always_comb begin
    model_index = addr >> REG_ALIGN_BITS;
    rdata       = '0;
    err         = 1'b1;
    if (model_index < N_REGS) begin
      rdata = model[model_index];
      err   = 1'b0;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int unsigned index = 0; index < N_REGS; index++) begin
        model[index] <= '0;
      end
    end else if (we && !err) begin
      for (int unsigned byte_index = 0; byte_index < REG_SW; byte_index++) begin
        if (wstrb[byte_index]) begin
          model[model_index][8*byte_index +: 8] <= wdata[8*byte_index +: 8];
        end
      end
    end
  end

  // Check the two register-file-port invariants directly, instead of relying
  // only on the model's register index to expose a violation.
  initial begin
    forever begin
      @(posedge clk_i);
      if (we || re) begin
        check1("register access never has we and re", we && re, 1'b0);
        check1("register access byte offset aligned",
               |addr[REG_ALIGN_BITS-1:0], 1'b0);
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Independent golden-model helpers
  // ---------------------------------------------------------------------------
  function automatic logic [ADDR_W-1:0] mapped_addr(input int unsigned index);
    logic [ADDR_W-1:0] result;
    begin
      result = index * REG_BYTES;
      return result;
    end
  endfunction

  function automatic logic [REG_DW-1:0] merge_strobes(
    input logic [REG_DW-1:0] old_value,
    input logic [REG_DW-1:0] new_value,
    input logic [REG_SW-1:0] strobes
  );
    logic [REG_DW-1:0] result;
    begin
      result = old_value;
      for (int unsigned byte_index = 0; byte_index < REG_SW; byte_index++) begin
        if (strobes[byte_index]) begin
          result[8*byte_index +: 8] = new_value[8*byte_index +: 8];
        end
      end
      return result;
    end
  endfunction

  // AXI4-Lite data and strobe positions are defined by the byte address.  For
  // a 32-bit register file bit 2 selects the lower or upper 32-bit bus lane;
  // a 64-bit register file occupies the whole bus word.
  function automatic lite_data_t bus_data_for_reg(
    input logic [ADDR_W-1:0] byte_addr,
    input logic [REG_DW-1:0] value
  );
    lite_data_t result;
    begin
      result = '0;
      if (REG_DW == LITE_DW) begin
        result[REG_DW-1:0] = value;
      end else if (byte_addr[REG_ALIGN_BITS]) begin
        result[LITE_DW-1 -: REG_DW] = value;
      end else begin
        result[REG_DW-1:0] = value;
      end
      return result;
    end
  endfunction

  function automatic lite_strb_t bus_strb_for_reg(
    input logic [ADDR_W-1:0] byte_addr,
    input logic [REG_SW-1:0] strobes
  );
    lite_strb_t result;
    begin
      result = '0;
      if (REG_DW == LITE_DW) begin
        result[REG_SW-1:0] = strobes;
      end else if (byte_addr[REG_ALIGN_BITS]) begin
        result[LITE_SW-1 -: REG_SW] = strobes;
      end else begin
        result[REG_SW-1:0] = strobes;
      end
      return result;
    end
  endfunction

  function automatic logic [REG_DW-1:0] random_reg_value;
    logic [LITE_DW-1:0] random_word;
    begin
      random_word = {$urandom, $urandom};
      return random_word[REG_DW-1:0];
    end
  endfunction

  function automatic logic [REG_SW-1:0] random_reg_strobes;
    logic [REG_SW-1:0] result;
    begin
      result = '0;
      for (int unsigned byte_index = 0; byte_index < REG_SW; byte_index++) begin
        result[byte_index] = $urandom_range(1, 0);
      end
      if (result == '0) begin
        result[0] = 1'b1;
      end
      return result;
    end
  endfunction

  // ---------------------------------------------------------------------------
  // Check helpers
  // ---------------------------------------------------------------------------
  task automatic check(input string name,
                       input lite_data_t got,
                       input lite_data_t exp);
    checks++;
    if (got !== exp) begin
      errors++;
      $display("  FAIL  REG_DW=%0d %-40s got %016h exp %016h",
               REG_DW, name, got, exp);
    end
  endtask

  task automatic check1(input string name, input logic got, input logic exp);
    checks++;
    if (got !== exp) begin
      errors++;
      $display("  FAIL  REG_DW=%0d %-40s got %b exp %b",
               REG_DW, name, got, exp);
    end
  endtask

  task automatic check_resp(input string name,
                            input lite_resp_t got,
                            input lite_resp_t exp);
    checks++;
    if (got !== exp) begin
      errors++;
      $display("  FAIL  REG_DW=%0d %-40s got %02b exp %02b",
               REG_DW, name, got, exp);
    end
  endtask

  // ---------------------------------------------------------------------------
  // AXI4-Lite driver.  Each request channel is retired on its own handshake.
  // The caller can skew AW and W, and can stall B/R after a response appears.
  // ---------------------------------------------------------------------------
  task automatic send_write_channels(
    input string name,
    input logic [ADDR_W-1:0] byte_addr,
    input lite_data_t bus_data,
    input lite_strb_t bus_strb,
    input int unsigned aw_delay,
    input int unsigned w_delay
  );
    bit aw_done, w_done;
    int unsigned cycles;
    begin
      aw_done = 1'b0;
      w_done  = 1'b0;
      cycles  = 0;
      req.b_ready = 1'b0;

      while (!(aw_done && w_done) && (cycles < MAX_WAIT)) begin
        @(negedge clk_i);
        if (aw_done) begin
          req.aw_valid = 1'b0;
        end else if (cycles >= aw_delay) begin
          req.aw.addr  = byte_addr;
          req.aw.prot  = '0;
          req.aw_valid = 1'b1;
        end else begin
          req.aw_valid = 1'b0;
        end

        if (w_done) begin
          req.w_valid = 1'b0;
        end else if (cycles >= w_delay) begin
          req.w.data  = bus_data;
          req.w.strb  = bus_strb;
          req.w_valid = 1'b1;
        end else begin
          req.w_valid = 1'b0;
        end

        @(posedge clk_i);
        if (req.aw_valid && rsp.aw_ready) begin
          aw_done = 1'b1;
        end
        if (req.w_valid && rsp.w_ready) begin
          w_done = 1'b1;
        end
        cycles++;
      end

      @(negedge clk_i);
      req.aw_valid = 1'b0;
      req.w_valid  = 1'b0;
      check1({name, " AW handshake"}, aw_done, 1'b1);
      check1({name, " W handshake"},  w_done,  1'b1);
    end
  endtask

  task automatic send_read_address(
    input string name,
    input logic [ADDR_W-1:0] byte_addr,
    input int unsigned ar_delay
  );
    bit ar_done;
    int unsigned cycles;
    begin
      ar_done = 1'b0;
      cycles  = 0;
      req.r_ready = 1'b0;

      while (!ar_done && (cycles < MAX_WAIT)) begin
        @(negedge clk_i);
        if (cycles >= ar_delay) begin
          req.ar.addr  = byte_addr;
          req.ar.prot  = '0;
          req.ar_valid = 1'b1;
        end else begin
          req.ar_valid = 1'b0;
        end

        @(posedge clk_i);
        if (req.ar_valid && rsp.ar_ready) begin
          ar_done = 1'b1;
        end
        cycles++;
      end

      @(negedge clk_i);
      req.ar_valid = 1'b0;
      check1({name, " AR handshake"}, ar_done, 1'b1);
    end
  endtask

  task automatic collect_b(
    input string name,
    input int unsigned stalled_cycles,
    output lite_resp_t response
  );
    lite_resp_t held_response;
    int unsigned waited;
    begin
      req.b_ready = 1'b0;
      waited = 0;
      while (!rsp.b_valid && (waited < MAX_WAIT)) begin
        @(negedge clk_i);
        waited++;
      end
      check1({name, " B valid despite b_ready=0"}, rsp.b_valid, 1'b1);
      held_response = rsp.b.resp;

      for (int unsigned cycle = 0; cycle < stalled_cycles; cycle++) begin
        @(posedge clk_i);
        #1;
        check1({name, " B valid held under backpressure"}, rsp.b_valid, 1'b1);
        check_resp({name, " B payload held under backpressure"},
                   rsp.b.resp, held_response);
      end

      @(negedge clk_i);
      req.b_ready = 1'b1;
      @(posedge clk_i);
      #1;
      @(negedge clk_i);
      req.b_ready = 1'b0;
      check1({name, " B retires after handshake"}, rsp.b_valid, 1'b0);
      response = held_response;
    end
  endtask

  task automatic collect_r(
    input string name,
    input int unsigned stalled_cycles,
    output lite_data_t data,
    output lite_resp_t response
  );
    lite_data_t held_data;
    lite_resp_t held_response;
    int unsigned waited;
    begin
      req.r_ready = 1'b0;
      waited = 0;
      while (!rsp.r_valid && (waited < MAX_WAIT)) begin
        @(negedge clk_i);
        waited++;
      end
      check1({name, " R valid despite r_ready=0"}, rsp.r_valid, 1'b1);
      held_data     = rsp.r.data;
      held_response = rsp.r.resp;

      for (int unsigned cycle = 0; cycle < stalled_cycles; cycle++) begin
        @(posedge clk_i);
        #1;
        check1({name, " R valid held under backpressure"}, rsp.r_valid, 1'b1);
        check_resp({name, " R response held under backpressure"},
                   rsp.r.resp, held_response);
        check({name, " R data held under backpressure"}, rsp.r.data, held_data);
      end

      @(negedge clk_i);
      req.r_ready = 1'b1;
      @(posedge clk_i);
      #1;
      @(negedge clk_i);
      req.r_ready = 1'b0;
      check1({name, " R retires after handshake"}, rsp.r_valid, 1'b0);
      data     = held_data;
      response = held_response;
    end
  endtask

  task automatic write_raw(
    input string name,
    input logic [ADDR_W-1:0] byte_addr,
    input lite_data_t bus_data,
    input lite_strb_t bus_strb,
    input int unsigned aw_delay,
    input int unsigned w_delay,
    input int unsigned b_stall,
    output lite_resp_t response
  );
    begin
      send_write_channels(name, byte_addr, bus_data, bus_strb, aw_delay, w_delay);
      collect_b(name, b_stall, response);
    end
  endtask

  task automatic read_raw(
    input string name,
    input logic [ADDR_W-1:0] byte_addr,
    input int unsigned ar_delay,
    input int unsigned r_stall,
    output lite_data_t data,
    output lite_resp_t response
  );
    begin
      send_read_address(name, byte_addr, ar_delay);
      collect_r(name, r_stall, data, response);
    end
  endtask

  task automatic write_mapped(
    input string name,
    input int unsigned index,
    input logic [REG_DW-1:0] value,
    input logic [REG_SW-1:0] strobes,
    input int unsigned aw_delay,
    input int unsigned w_delay,
    input int unsigned b_stall
  );
    lite_resp_t response;
    begin
      write_raw(name, mapped_addr(index),
                bus_data_for_reg(mapped_addr(index), value),
                bus_strb_for_reg(mapped_addr(index), strobes),
                aw_delay, w_delay, b_stall, response);
      check_resp({name, " response"}, response, RESP_OKAY);
      shadow[index] = merge_strobes(shadow[index], value, strobes);
    end
  endtask

  task automatic read_mapped(
    input string name,
    input int unsigned index,
    input int unsigned ar_delay,
    input int unsigned r_stall
  );
    lite_data_t data;
    lite_resp_t response;
    begin
      read_raw(name, mapped_addr(index), ar_delay, r_stall, data, response);
      check_resp({name, " response"}, response, RESP_OKAY);
      check({name, " data"}, data,
            bus_data_for_reg(mapped_addr(index), shadow[index]));
    end
  endtask

  // Queue an AW/W pair and an AR together.  They target different registers,
  // so either legal arbitration order has a single, unambiguous golden result.
  task automatic arbitration_pair(
    input string name,
    input int unsigned write_index,
    input logic [REG_DW-1:0] write_value,
    input int unsigned read_index
  );
    lite_resp_t b_response, r_response;
    lite_data_t r_data;
    begin
      req.b_ready = 1'b0;
      req.r_ready = 1'b0;
      @(negedge clk_i);
      req.aw.addr  = mapped_addr(write_index);
      req.aw.prot  = '0;
      req.aw_valid = 1'b1;
      req.w.data   = bus_data_for_reg(mapped_addr(write_index), write_value);
      req.w.strb   = bus_strb_for_reg(mapped_addr(write_index), {REG_SW{1'b1}});
      req.w_valid  = 1'b1;
      req.ar.addr  = mapped_addr(read_index);
      req.ar.prot  = '0;
      req.ar_valid = 1'b1;
      @(posedge clk_i);

      @(negedge clk_i);
      req.aw_valid = 1'b0;
      req.w_valid  = 1'b0;
      req.ar_valid = 1'b0;
      check1({name, " never asserts we and re together"}, we && re, 1'b0);
      check1({name, " grants one queued access"}, we || re, 1'b1);
      @(posedge clk_i);

      // Either response may have been granted first.  Both must make progress
      // while the other response remains backpressured.
      collect_b(name, 0, b_response);
      collect_r(name, 0, r_data, r_response);
      check_resp({name, " write response"}, b_response, RESP_OKAY);
      check_resp({name, " read response"}, r_response, RESP_OKAY);
      check({name, " read data"}, r_data,
            bus_data_for_reg(mapped_addr(read_index), shadow[read_index]));
      shadow[write_index] = merge_strobes(shadow[write_index], write_value,
                                           {REG_SW{1'b1}});
    end
  endtask

  // ---------------------------------------------------------------------------
  // Stimulus
  // ---------------------------------------------------------------------------
  initial begin : stimulus
    lite_resp_t response;
    lite_data_t data;
    logic [REG_DW-1:0] even_value, odd_value, partial_value, saved_value;
    logic [REG_SW-1:0] all_strobes, partial_strobes;

    req      = '0;
    done_o   = 1'b0;
    checks   = 0;
    errors   = 0;
    checks_o = 0;
    errors_o = 0;

    // Reset state is observable on the bus and must issue no register access.
    @(negedge clk_i);
    check1("b_valid low in reset",  rsp.b_valid,  1'b0);
    check1("r_valid low in reset",  rsp.r_valid,  1'b0);
    check1("aw_ready low in reset", rsp.aw_ready, 1'b0);
    check1("w_ready low in reset",  rsp.w_ready,  1'b0);
    check1("ar_ready low in reset", rsp.ar_ready, 1'b0);
    check1("we low in reset",       we,           1'b0);
    check1("re low in reset",       re,           1'b0);

    @(posedge rst_ni);
    @(negedge clk_i);
    for (int unsigned index = 0; index < N_REGS; index++) begin
      shadow[index] = '0;
    end
    void'($urandom(32'h4d45_4453 ^ REG_DW));
    all_strobes     = {REG_SW{1'b1}};
    partial_strobes = '0;
    partial_strobes[0]        = 1'b1;
    partial_strobes[REG_SW-1] = 1'b1;
    even_value    = 64'h0123_4567_89ab_cdef;
    odd_value     = 64'hfedc_ba98_7654_3210;
    partial_value = 64'h55aa_33cc_f00d_0f0f;

    // Basic accesses cover both bus lanes for REG_DW=32.  With REG_DW=64 the
    // same test covers two adjacent 64-bit register offsets.
    write_mapped("basic even write", 0, even_value, all_strobes, 0, 0, 0);
    read_mapped("basic even read",   0, 0, 0);
    write_mapped("basic odd write",  1, odd_value, all_strobes, 0, 0, 0);
    read_mapped("basic odd read",    1, 0, 0);

    // AXI4-Lite permits either ordering of AW and W.  The skew is deliberately
    // several cycles, so an implementation that waits for the other channel
    // cannot pass by accidentally sampling both in the same cycle.
    write_mapped("W before AW", 2, even_value, all_strobes, 4, 0, 0);
    read_mapped("W before AW readback", 2, 0, 0);
    write_mapped("AW before W", 3, odd_value, all_strobes, 0, 4, 0);
    read_mapped("AW before W readback", 3, 0, 0);

    // Strobes are part of the peripheral contract, not an optional detail.
    write_mapped("strobe seed", 4, even_value, all_strobes, 0, 0, 0);
    write_mapped("partial byte strobes", 4, partial_value, partial_strobes,
                 0, 0, 0);
    read_mapped("partial byte strobes readback", 4, 0, 0);

    // Both B and R must assert while their ready signal is low and preserve
    // their complete payload throughout a multi-cycle stall.
    write_mapped("B backpressure", 5, partial_value, all_strobes, 0, 0, 3);
    read_mapped("R backpressure", 5, 0, 3);

    // When an AW/W transaction and AR transaction are ready together, one is
    // granted in the first cycle and the other later; neither is lost.
    arbitration_pair("simultaneous read and write", 6, even_value, 7);
    arbitration_pair("fairness priority flips", 7, odd_value, 6);

    // Unmapped accesses use err_i and return SLVERR.  The known mapped register
    // is read afterward to make the write-error case observable at the device.
    saved_value = shadow[0];
    write_raw("unmapped write", mapped_addr(N_REGS),
              bus_data_for_reg(mapped_addr(N_REGS), partial_value),
              bus_strb_for_reg(mapped_addr(N_REGS), all_strobes),
              0, 0, 0, response);
    check_resp("unmapped write response", response, RESP_SLVERR);
    read_mapped("unmapped write leaves mapped state intact", 0, 0, 0);
    check("shadow stays intact after unmapped write",
          bus_data_for_reg(mapped_addr(0), shadow[0]),
          bus_data_for_reg(mapped_addr(0), saved_value));

    read_raw("unmapped read", mapped_addr(N_REGS), 0, 0, data, response);
    check_resp("unmapped read response", response, RESP_SLVERR);

    // A 32-bit peripheral cannot accept one write whose strobes claim both
    // 32-bit lanes.  A readback proves the failed transaction had no side
    // effect, rather than merely receiving an error response.
    if (REG_DW == (LITE_DW / 2)) begin
      saved_value = shadow[2];
      write_raw("cross-lane write", mapped_addr(2),
                64'hdeca_fbad_c001_d00d, {LITE_SW{1'b1}}, 0, 0, 0, response);
      check_resp("cross-lane write response", response, RESP_SLVERR);
      read_mapped("cross-lane write leaves register intact", 2, 0, 0);
      check("cross-lane golden state unchanged",
            bus_data_for_reg(mapped_addr(2), shadow[2]),
            bus_data_for_reg(mapped_addr(2), saved_value));
    end

    // Seeded random sweep against the independent shadow model.  Randomised
    // AW/W skew exercises channel capture repeatedly; reads and writes both
    // revisit odd/even register offsets under the two width configurations.
    for (int unsigned iteration = 0; iteration < 250; iteration++) begin
      int unsigned index;
      logic [REG_DW-1:0] value;
      logic [REG_SW-1:0] strobes;
      index   = $urandom_range(N_REGS-1, 0);
      value   = random_reg_value();
      strobes = random_reg_strobes();
      if ($urandom_range(1, 0)) begin
        write_mapped($sformatf("random write %0d", iteration), index, value,
                     strobes, $urandom_range(3, 0), $urandom_range(3, 0), 0);
      end else begin
        read_mapped($sformatf("random read %0d", iteration), index,
                    $urandom_range(2, 0), 0);
      end
    end

    checks_o = checks;
    errors_o = errors;
    done_o   = 1'b1;
  end

endmodule
