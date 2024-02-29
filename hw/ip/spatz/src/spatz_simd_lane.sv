// Copyright 2023 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Author: Matheus Cavalcante, ETH Zurich
//
// The SIMD lane calculates all simd operations given for a give distinct
// element width.

module spatz_simd_lane import spatz_pkg::*; import rvv_pkg::vew_e; #(
    parameter int  unsigned Width  = 8,
    // Derived parameters. Do not change!
    parameter type          data_t = logic [Width-1:0]
  ) (
    input  logic  clk_i,
    input  logic  rst_ni,
    // Operation Signals
    input  op_e   operation_i,
    input  logic  operation_valid_i,
    input  data_t op_s1_i,
    input  data_t op_s2_i,
    input  data_t op_d_i,
    input  logic  is_signed_i,
    input  logic  carry_i,
    input  vew_e  sew_i,
    input  fixpoint_rnd_e vxrm_i,
    // Result Output
    output data_t result_o, 
    output logic  fixedpoint_sat_o,
    output logic  result_valid_o,
    input  logic  result_ready_i
  );

  ////////////////
  // Multiplier //
  ////////////////

  logic                is_mult;
  logic  [2*Width-1:0] mult_result;
  data_t               mult_op1;
  data_t               mult_op2;

  // Multiplier
  always_comb begin: mult
    is_mult = operation_valid_i && (operation_i inside {VMACC, VNMSAC, VMADD, VNMSUB, VMUL, VMULH, VMULHU, VMULHSU, VSMUL});

    // Mute the multiplier
    mult_result = '0;
    if (is_mult)
      mult_result = $signed({mult_op1[Width-1] & is_signed_i & ~(operation_i == VMULHSU), mult_op1}) * $signed({mult_op2[Width-1] & is_signed_i, mult_op2});
  end: mult

  // Select multiplier operands
  always_comb begin : mult_operands
    mult_op1 = op_s1_i;
    mult_op2 = op_s2_i;
    if (operation_i inside {VMADD, VNMSUB}) begin
      mult_op1 = op_s1_i;
      mult_op2 = op_d_i;
    end
  end: mult_operands

  ////////////////////////
  // Adder / Subtractor //
  ////////////////////////

  logic  [Width:0] adder_result;
  logic  [Width:0] subtractor_result;
  data_t           simd_result;
  data_t           arith_op1;        // Subtrahend
  data_t           arith_op2;        // Minuend

  // Select arithmetic operands
  always_comb begin : arith_operands
    unique case (operation_i)
      VMACC,
      VNMSAC: begin
        arith_op1 = mult_result[Width-1:0];
        arith_op2 = op_d_i;
      end
      VMADD,
      VNMSUB: begin
        arith_op1 = mult_result[Width-1:0];
        arith_op2 = op_s2_i;
      end
      VRSUB: begin
        arith_op1 = op_s2_i;
        arith_op2 = op_s1_i;
      end
      default: begin
        arith_op1 = op_s1_i;
        arith_op2 = op_s2_i;
      end
    endcase // operation_i
  end       // arith_operands

  assign adder_result      = operation_valid_i ? $signed(arith_op2) + $signed(arith_op1) + carry_i : '0;
  assign subtractor_result = operation_valid_i ? $signed(arith_op2) - $signed(arith_op1) - carry_i : '0;

  // Fixed-point saturation
  logic add_overflow, sub_overflow;
  logic [$clog2(Width)-1:0] msb_indx;
  data_t add_sat_val, sub_sat_val;

  // Select sign bit index based on SEW
  if (Width == 64) begin
    always_comb begin
      unique case (sew_i)
        rvv_pkg::EW_64: msb_indx = 63;
        rvv_pkg::EW_32: msb_indx = 31;
        rvv_pkg::EW_16: msb_indx = 15;
        default       : msb_indx = 7;
      endcase
    end
  end else if (Width == 32) begin
    always_comb begin
      unique case (sew_i)
        rvv_pkg::EW_32: msb_indx = 31;
        rvv_pkg::EW_16: msb_indx = 15;
        default       : msb_indx = 7;
      endcase
    end
  end else if (Width == 16) begin
    always_comb begin
      unique case (sew_i)
        rvv_pkg::EW_16: msb_indx = 15;
        default       : msb_indx = 7;
      endcase
    end
  end else if (Width == 8) begin
    assign msb_indx = 7;
  end
      
  // Detect overflow and assign saturation value
  always_comb begin
    if (is_signed_i) begin
      // Signed overflow is detected using the sign of the result and the sign of the operands
      add_overflow = (arith_op1[Width-1] == arith_op2[Width-1]) && (adder_result[msb_indx] != arith_op1[Width-1]);
      sub_overflow = (arith_op1[Width-1] != arith_op2[Width-1]) && (subtractor_result[msb_indx] != arith_op2[Width-1]); // op2's sign was NOT "inverted"
    end else begin
      // Unsigned overflow is detected using the carry out
      add_overflow = adder_result[msb_indx+1];
      sub_overflow = subtractor_result[msb_indx+1];
    end

    // Default saturation values (unsigned)
    add_sat_val = {(Width){1'b1}};
    sub_sat_val = {(Width){1'b0}};
    if (is_signed_i) begin
      // Saturation is needed when the result overflows. Therefore, the sign of the saturation value is opposite to the sign of the result.
      if (adder_result[msb_indx]) begin
        add_sat_val[Width-2:0] = {(Width-1){1'b1}};
        add_sat_val[msb_indx]  = 1'b0; // Sign bit
      end
      else begin
        add_sat_val[Width-2:0] = {(Width-1){1'b0}};
        add_sat_val[msb_indx]  = 1'b1; // Sign bit
      end
      if (subtractor_result[msb_indx]) begin
        sub_sat_val[Width-2:0] = {(Width-1){1'b1}};
        sub_sat_val[msb_indx]  = 1'b0; // Sign bit
      end
      else begin
        sub_sat_val[Width-2:0] = {(Width-1){1'b0}};
        sub_sat_val[msb_indx]  = 1'b1; // Sign bit
      end
    end
  end

  // Fixed-point averaging and rounding
  data_t avg_add_result, avg_sub_result;
  always_comb begin
    automatic logic r_add, r_sub; // Rounding increment value
    // Do the shifting. The position of MSB depends on SEW value
    avg_add_result = adder_result[Width:1];
    avg_sub_result = subtractor_result[Width:1];
    if (is_signed_i) begin // Signed shift
      avg_add_result[msb_indx] = adder_result[msb_indx];
      avg_sub_result[msb_indx] = subtractor_result[msb_indx];
    end else begin
      avg_add_result[msb_indx] = adder_result[msb_indx+1];
      avg_sub_result[msb_indx] = subtractor_result[msb_indx+1];
    end
    // Do the rounding
    case(vxrm_i)
      RNU: begin
        r_add = adder_result[0];
        r_sub = subtractor_result[0];
      end
      RNE: begin
        r_add = &adder_result[1:0];
        r_sub = &subtractor_result[1:0];
      end
      RDN: begin
        r_add = 1'b0;
        r_sub = 1'b0;
      end
      ROD: begin 
        r_add = !adder_result[1] & (adder_result[0]!=0);
        r_sub = !subtractor_result[1] & (subtractor_result[0]!=0);
      end
      endcase
    avg_add_result += r_add;
    avg_sub_result += r_sub;
  end

  /////////////////////////////////////////////////////
  // Fixed-point fractional multiply with saturation //
  /////////////////////////////////////////////////////
  data_t vsmul_result;
  logic vsmul_sat;
  // The fowllowing 2 signals are used to select the appropriate bit range to do rounding for different SEW values
  logic [(Width-1)-2:0] rne_range; 
  logic [(Width-1)-1:0] rod_range;
  if (Width == 64) begin
    always_comb begin
      rne_range = { {(7*Width/8){1'b0}},mult_result[(Width/8-1)-2:0]};
      rod_range = { {(7*Width/8){1'b0}},mult_result[(Width/8-1)-1:0]};
      case (sew_i)
        rvv_pkg::EW_16: begin 
          rne_range[(Width/4-1)-2:0] = mult_result[(Width/4-1)-2:0];
          rod_range[(Width/4-1)-1:0] = mult_result[(Width/4-1)-1:0];
        end
        rvv_pkg::EW_32: begin 
          rne_range[(Width/2-1)-2:0] = mult_result[(Width/2-1)-2:0];
          rod_range[(Width/2-1)-1:0] = mult_result[(Width/2-1)-1:0];
        end
        rvv_pkg::EW_64: begin 
          rne_range[(Width-1)-2:0] = mult_result[(Width-1)-2:0];
          rod_range[(Width-1)-1:0] = mult_result[(Width-1)-1:0];
        end
      endcase
    end
  end else if (Width == 32) begin
    always_comb begin
      rne_range = {{(3*Width/4){1'b0}}, mult_result[(Width/4-1)-2:0]};
      rod_range = {{(3*Width/4){1'b0}}, mult_result[(Width/4-1)-1:0]};
      case (sew_i)
        rvv_pkg::EW_16: begin 
          rne_range[(Width/2-1)-2:0] = mult_result[(Width/2-1)-2:0];
          rod_range[(Width/2-1)-1:0] = mult_result[(Width/2-1)-1:0];
        end
        rvv_pkg::EW_32: begin
          rne_range = mult_result[(Width-1)-2:0];
          rod_range = mult_result[(Width-1)-1:0];
        end
      endcase
    end
  end else if (Width == 16) begin
    always_comb begin
      rne_range = {{(Width/2){1'b0}}, mult_result[(Width/2-1)-2:0]};
      rod_range = {{(Width/2){1'b0}}, mult_result[(Width/2-1)-1:0]};
      if (sew_i == rvv_pkg::EW_16) begin
          rne_range = mult_result[(Width-1)-2:0];
          rod_range = mult_result[(Width-1)-1:0];
      end
    end
  end else if (Width == 8) begin
    assign rne_range = mult_result[(Width-1)-2:0];
    assign rod_range = mult_result[(Width-1)-1:0];
  end

  always_comb begin
    automatic logic r; // Rounding increment value
    case(vxrm_i)
        RNU: r = mult_result[msb_indx-1];
        // RNE: r = mult_result[msb_indx-1] & (rne_range | mult_result[msb_indx]);
        RNE: r = mult_result[msb_indx-1] & (rne_range!=0 | mult_result[msb_indx]);
        RDN: r = 1'b0;
        // ROD: r = !mult_result[msb_indx] & (rod_range);
        ROD: r = !mult_result[msb_indx] & (rod_range!=0);
    endcase
    // Shift (signed only) and round
    vsmul_result = $signed(mult_result)>>>(msb_indx) + r;
    // Saturation: overflow happens when the shifted result have a different sign than the original result
    vsmul_sat = mult_result[2*msb_indx+1] ^ mult_result[2*msb_indx];
  end

  /////////////
  // Shifter //
  /////////////

  logic [$clog2(Width)-1:0] shift_amount;
  logic [Width-1:0]         shift_operand;
  if (Width >= 64) begin : gen_shift_operands_64
    always_comb begin
      unique case (sew_i)
        rvv_pkg::EW_64: begin
          shift_amount  = op_s1_i[5:0];
          shift_operand = op_s2_i;
        end
        rvv_pkg::EW_32: begin
          shift_amount = op_s1_i[4:0];
          if (operation_i == VSRA) shift_operand = $signed(op_s2_i[31:0]);
          else shift_operand                     = $unsigned(op_s2_i[31:0]);
        end
        rvv_pkg::EW_16: begin
          shift_amount = op_s1_i[3:0];
          if (operation_i == VSRA) shift_operand = $signed(op_s2_i[15:0]);
          else shift_operand                     = $unsigned(op_s2_i[15:0]);
        end
        default: begin
          shift_amount = op_s1_i[2:0];
          if (operation_i == VSRA) shift_operand = $signed(op_s2_i[7:0]);
          else shift_operand                     = $unsigned(op_s2_i[7:0]);
        end
      endcase
    end // always_comb
  end else if (Width >= 32) begin: gen_shift_operands_32
    always_comb begin
      unique case (sew_i)
        rvv_pkg::EW_32: begin
          shift_amount  = op_s1_i[4:0];
          shift_operand = op_s2_i[31:0];
        end
        rvv_pkg::EW_16: begin
          shift_amount = op_s1_i[3:0];
          if (operation_i == VSRA) shift_operand = $signed(op_s2_i[15:0]);
          else shift_operand                     = $unsigned(op_s2_i[15:0]);
        end
        default: begin
          shift_amount = op_s1_i[2:0];
          if (operation_i == VSRA) shift_operand = $signed(op_s2_i[7:0]);
          else shift_operand                     = $unsigned(op_s2_i[7:0]);
        end
      endcase
    end // always_comb
  end else if (Width >= 16) begin: gen_shift_operands_16
    always_comb begin
      unique case (sew_i)
        rvv_pkg::EW_16: begin
          shift_amount  = op_s1_i[3:0];
          shift_operand = op_s2_i[15:0];
        end
        default: begin
          shift_amount = op_s1_i[2:0];
          if (operation_i == VSRA) shift_operand = $signed(op_s2_i[7:0]);
          else shift_operand                     = $unsigned(op_s2_i[7:0]);
        end
      endcase
    end // shift_operands
  end else begin: gen_shift_operands_8
    always_comb begin
      shift_amount  = op_s1_i[2:0];
      shift_operand = op_s2_i[7:0];
    end
  end

  /////////////
  // Divider //
  /////////////

  logic         div_in_valid;
  logic         div_out_valid;
  logic  [31:0] div_op;
  data_t        div_result;

  always_comb begin: div_proc
    div_in_valid = operation_valid_i && (operation_i inside {VDIV, VDIVU, VREM, VREMU});

    unique case (operation_i)
      VDIV : div_op   = 32'b00000010000000000100000000110011;
      VDIVU: div_op   = 32'b00000010000000000101000000110011;
      VREM : div_op   = 32'b00000010000000000110000000110011;
      default: div_op = 32'b00000010000000000111000000110011;
    endcase
  end: div_proc

  spatz_serdiv #(
    .WIDTH  (Width),
    .IdWidth(1    )
  ) i_divider (
    .clk_i     (clk_i         ),
    .rst_ni    (rst_ni        ),
    .id_i      ('0            ),
    .op_a_i    (op_s2_i       ),
    .op_b_i    (op_s1_i       ),
    .operator_i(div_op        ),
    .in_vld_i  (div_in_valid  ),
    .in_rdy_o  (/* Unused */  ),
    .out_vld_o (div_out_valid ),
    .out_rdy_i (result_ready_i),
    .id_o      (/* Unused */  ),
    .res_o     (div_result    )
  );

  ////////////
  // Result //
  ////////////

  // Calculate arithmetic and logics and select correct result
  always_comb begin : simd
    simd_result    = '0;
    result_valid_o = 1'b0;
    fixedpoint_sat_o = 1'b0;
    if (operation_valid_i) begin
      // Valid result
      result_valid_o = 1'b1;
     
      unique case (operation_i)
        VADD, VMACC, VMADD, VADC         : simd_result = adder_result[Width-1:0];
        VSUB, VRSUB, VNMSAC, VNMSUB, VSBC: simd_result = subtractor_result[Width-1:0];
        VSADDU, VSADD                    : begin
          simd_result = add_overflow ? add_sat_val : adder_result[Width-1:0];
          fixedpoint_sat_o = add_overflow;
        end
        VSSUBU, VSSUB                    : begin
          simd_result = sub_overflow ? sub_sat_val : subtractor_result[Width-1:0];
          fixedpoint_sat_o = sub_overflow;
        end
        VAADDU, VAADD                    : simd_result = avg_add_result;
        VASUBU, VASUB                    : simd_result = avg_sub_result;
        VMIN, VMINU                      : simd_result = $signed({op_s1_i[Width-1] & is_signed_i, op_s1_i}) <= $signed({op_s2_i[Width-1] & is_signed_i, op_s2_i}) ? op_s1_i : op_s2_i;
        VMAX, VMAXU                      : simd_result = $signed({op_s1_i[Width-1] & is_signed_i, op_s1_i}) > $signed({op_s2_i[Width-1] & is_signed_i, op_s2_i}) ? op_s1_i : op_s2_i;
        VAND                             : simd_result = op_s1_i & op_s2_i;
        VOR                              : simd_result = op_s1_i | op_s2_i;
        VXOR                             : simd_result = op_s1_i ^ op_s2_i;
        VSLL                             : simd_result = shift_operand << shift_amount;
        VSRL                             : simd_result = shift_operand >> shift_amount;
        VSRA                             : simd_result = $signed(shift_operand) >>> shift_amount;
        // TODO: Change selection when SEW does not equal Width
        VMUL                             : simd_result = mult_result[Width-1:0];
        VMULH, VMULHU, VMULHSU           : begin
          simd_result = mult_result[2*Width-1:Width];
          for (int i = 0; i < $clog2(Width/8); i++)
            if (sew_i == rvv_pkg::vew_e'(i))
              simd_result = mult_result[8*(2**i) +: Width];
        end
        VSMUL                   : begin
          if (vsmul_sat) begin
            simd_result = '1;
            simd_result[msb_indx] = 1'b0; // Sign bit
          end else begin
            simd_result = vsmul_result;
          end
          fixedpoint_sat_o = vsmul_sat;
        end
        VMADC                   : simd_result = Width'(adder_result[Width]);
        VMSBC                   : simd_result = Width'(subtractor_result[Width]);
        VDIV, VDIVU, VREM, VREMU: begin
          simd_result    = div_result;
          result_valid_o = div_out_valid;
        end
        default: simd_result = '0;
      endcase // operation_i
    end
  end // simd

  assign result_o = simd_result;

endmodule : spatz_simd_lane
