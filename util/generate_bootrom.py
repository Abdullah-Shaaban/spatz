#!/usr/bin/env python3
#
# Copyright 2023 ETH Zurich and University of Bologna.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
#
# Author: Samuel Riedel, ETH Zurich

from string import Template
from math import log2
import argparse
import hjson
import os.path
import sys

from jsonref import JsonRef

parser = argparse.ArgumentParser(description="Convert binary file to verilog rom")
parser.add_argument(
    "filename", metavar="filename.bin", nargs=1, help="filename of input binary"
)
parser.add_argument(
    "--output", nargs="?", metavar="file", help="Name of output file", default=None
)
parser.add_argument(
    "--clustercfg",
    "-c",
    metavar="file",
    type=argparse.FileType("r"),
    required=True,
    help="A cluster configuration file",
)

args = parser.parse_args()
file = args.filename[0]

# check that file exists
if not os.path.isfile(file):
    sys.exit("File {} does not exist.".format(file))
filename = os.path.splitext(file)[0]

output = args.output
if output is None:
    output = filename

# Read HJSON description
cfg = []
with args.clustercfg as file:
    try:
        srcfull = file.read()
        cfg = hjson.loads(srcfull, use_decimal=True)
        cfg = JsonRef.replace_refs(cfg)
    except ValueError:
        raise SystemExit(sys.exc_info()[1])

DataWidth = int(cfg["cluster"]["dma_data_width"])
DataOffset = int(log2(DataWidth / 8))
if DataWidth < 32:
    sys.exit("DataWidth must be larger than 32")
if DataWidth % 32:
    sys.exit("DataWidth must be multiple of 32")

AddrWidth = int(cfg["cluster"]["addr_width"])

license = """\
// Copyright 2023 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Description: Automatically generated bootrom
//
// Generated by hardware/scripts/generate_bootrom.py

"""

module = """\
module $filename #(
  /* Automatically generated. DO NOT CHANGE! */
  parameter int unsigned DataWidth = $DataWidth,
  parameter int unsigned AddrWidth = $AddrWidth
) (
  input  logic                 clk_i,
  input  logic                 req_i,
  input  logic [AddrWidth-1:0] addr_i,
  output logic [DataWidth-1:0] rdata_o
);
  localparam int RomSize = $size;
  localparam int AddrBits = RomSize > 1 ? $$clog2(RomSize) : 1;

  const logic [RomSize-1:0][DataWidth-1:0] mem = {
$content
  };

  logic [AddrBits-1:0] addr_q;

  always_ff @(posedge clk_i) begin
    if (req_i) begin
      addr_q <= addr_i[AddrBits-1+$DataOffset:$DataOffset];
    end
  end

  // this prevents spurious Xes from propagating into
  // the speculative fetch stage of the core
  assign rdata_o = (addr_q < RomSize) ? mem[addr_q] : '0;
endmodule
"""

c_var = """\
// Auto-generated code

const int reset_vec_size = $size;

uint32_t reset_vec[reset_vec_size] = {
$content
};
"""


def read_bin():
    with open(filename + ".bin", "rb") as f:
        rom = bytes.hex(f.read())
        rom = list(map("".join, zip(rom[::2], rom[1::2])))
    # align to 64 bit
    align = (int((len(rom) + 7) / 8)) * 8
    for i in range(len(rom), align):
        rom.append("00")
    return rom


rom = read_bin()

""" Generate SystemVerilog bootcode for FPGA and ASIC
"""
with open(output, "w") as f:
    rom_str = ""
    ByteWidth = int(DataWidth / 8)
    # process in junks of DataWidth bit (DataWidth/8 byte)
    for i in reversed(range(int(len(rom) / ByteWidth))):
        rom_str += "    {}'h".format(DataWidth)
        for b in reversed(range(int(ByteWidth / 4))):
            rom_str += (
                "".join(rom[i * ByteWidth + b * 4 : i * ByteWidth + b * 4 + 4][::-1])
                + "_"
            )
            rom_str = rom_str[:-1]
        rom_str += ",\n"
    # remove the trailing comma
    rom_str = rom_str[:-2]
    f.write(license)
    s = Template(module)
    f.write(
        s.substitute(
            filename=os.path.basename(filename),
            size=int(len(rom) / ByteWidth),
            content=rom_str,
            DataWidth=DataWidth,
            DataOffset=DataOffset,
            AddrWidth=AddrWidth,
        )
    )
