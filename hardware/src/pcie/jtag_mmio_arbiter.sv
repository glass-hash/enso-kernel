`include "../constants.sv"
`include "pcie_consts.sv"

/*
 * This module arbitrates reads and writes from JTAG and MMIO to allow them to
 * share the same BRAM ports. PCIe requests have priority over JTAG.
 */

module jtag_mmio_arbiter (
    input logic pcie_clk,
    input logic jtag_clk,
    input logic pcie_reset_n,

    // pcie signals
    input  logic [PCIE_ADDR_WIDTH-1:0] pcie_address_0,
    input  logic                       pcie_write_0,
    input  logic                       pcie_read_0,
    output logic                       pcie_readdatavalid_0,
    output logic [511:0]               pcie_readdata_0,
    input  logic [511:0]               pcie_writedata_0,
    input  logic [63:0]                pcie_byteenable_0,

    // jtag signals
    input  logic [29:0] status_addr,
    input  logic        status_read,
    input  logic        status_write,
    input  logic [31:0] status_writedata,
    output logic [31:0] status_readdata,
    output logic        status_readdata_valid,

    // BRAM signals
    output logic [BRAM_TABLE_IDX_WIDTH-1:0] dsc_q_table_tails_addr,
    output logic [31:0]                     dsc_q_table_tails_wr_data,
    input  logic [31:0]                     dsc_q_table_tails_rd_data,
    output logic                            dsc_q_table_tails_rd_en,
    output logic                            dsc_q_table_tails_wr_en,
    output logic [BRAM_TABLE_IDX_WIDTH-1:0] dsc_q_table_heads_addr,
    output logic [31:0]                     dsc_q_table_heads_wr_data,
    input  logic [31:0]                     dsc_q_table_heads_rd_data,
    output logic                            dsc_q_table_heads_rd_en,
    output logic                            dsc_q_table_heads_wr_en,
    output logic [BRAM_TABLE_IDX_WIDTH-1:0] dsc_q_table_l_addrs_addr,
    output logic [31:0]                     dsc_q_table_l_addrs_wr_data,
    input  logic [31:0]                     dsc_q_table_l_addrs_rd_data,
    output logic                            dsc_q_table_l_addrs_rd_en,
    output logic                            dsc_q_table_l_addrs_wr_en,
    output logic [BRAM_TABLE_IDX_WIDTH-1:0] dsc_q_table_h_addrs_addr,
    output logic [31:0]                     dsc_q_table_h_addrs_wr_data,
    input  logic [31:0]                     dsc_q_table_h_addrs_rd_data,
    output logic                            dsc_q_table_h_addrs_rd_en,
    output logic                            dsc_q_table_h_addrs_wr_en,

    output logic [BRAM_TABLE_IDX_WIDTH-1:0] pkt_q_table_tails_addr,
    output logic [31:0]                     pkt_q_table_tails_wr_data,
    input  logic [31:0]                     pkt_q_table_tails_rd_data,
    output logic                            pkt_q_table_tails_rd_en,
    output logic                            pkt_q_table_tails_wr_en,
    output logic [BRAM_TABLE_IDX_WIDTH-1:0] pkt_q_table_heads_addr,
    output logic [31:0]                     pkt_q_table_heads_wr_data,
    input  logic [31:0]                     pkt_q_table_heads_rd_data,
    output logic                            pkt_q_table_heads_rd_en,
    output logic                            pkt_q_table_heads_wr_en,
    output logic [BRAM_TABLE_IDX_WIDTH-1:0] pkt_q_table_l_addrs_addr,
    output logic [31:0]                     pkt_q_table_l_addrs_wr_data,
    input  logic [31:0]                     pkt_q_table_l_addrs_rd_data,
    output logic                            pkt_q_table_l_addrs_rd_en,
    output logic                            pkt_q_table_l_addrs_wr_en,
    output logic [BRAM_TABLE_IDX_WIDTH-1:0] pkt_q_table_h_addrs_addr,
    output logic [31:0]                     pkt_q_table_h_addrs_wr_data,
    input  logic [31:0]                     pkt_q_table_h_addrs_rd_data,
    output logic                            pkt_q_table_h_addrs_rd_en,
    output logic                            pkt_q_table_h_addrs_wr_en,

    // extra control signals
    output logic [31:0] control_regs [NB_CONTROL_REGS]
);

queue_interface_t dsc_q_table;
queue_interface_t pkt_q_table;

// We can't expose a struct with both input and output signals, so we expose the
// individual signals and combine them here:
always_comb begin
    dsc_q_table_tails_addr = dsc_q_table.tails.addr;
    dsc_q_table_tails_wr_data = dsc_q_table.tails.wr_data;
    dsc_q_table.tails.rd_data = dsc_q_table_tails_rd_data;
    dsc_q_table_tails_rd_en = dsc_q_table.tails.rd_en;
    dsc_q_table_tails_wr_en = dsc_q_table.tails.wr_en;
    dsc_q_table_heads_addr = dsc_q_table.heads.addr;
    dsc_q_table_heads_wr_data = dsc_q_table.heads.wr_data;
    dsc_q_table.heads.rd_data = dsc_q_table_heads_rd_data;
    dsc_q_table_heads_rd_en = dsc_q_table.heads.rd_en;
    dsc_q_table_heads_wr_en = dsc_q_table.heads.wr_en;
    dsc_q_table_l_addrs_addr = dsc_q_table.l_addrs.addr;
    dsc_q_table_l_addrs_wr_data = dsc_q_table.l_addrs.wr_data;
    dsc_q_table.l_addrs.rd_data = dsc_q_table_l_addrs_rd_data;
    dsc_q_table_l_addrs_rd_en = dsc_q_table.l_addrs.rd_en;
    dsc_q_table_l_addrs_wr_en = dsc_q_table.l_addrs.wr_en;
    dsc_q_table_h_addrs_addr = dsc_q_table.h_addrs.addr;
    dsc_q_table_h_addrs_wr_data = dsc_q_table.h_addrs.wr_data;
    dsc_q_table.h_addrs.rd_data = dsc_q_table_h_addrs_rd_data;
    dsc_q_table_h_addrs_rd_en = dsc_q_table.h_addrs.rd_en;
    dsc_q_table_h_addrs_wr_en = dsc_q_table.h_addrs.wr_en;
    pkt_q_table_tails_addr = pkt_q_table.tails.addr;
    pkt_q_table_tails_wr_data = pkt_q_table.tails.wr_data;
    pkt_q_table.tails.rd_data = pkt_q_table_tails_rd_data;
    pkt_q_table_tails_rd_en = pkt_q_table.tails.rd_en;
    pkt_q_table_tails_wr_en = pkt_q_table.tails.wr_en;
    pkt_q_table_heads_addr = pkt_q_table.heads.addr;
    pkt_q_table_heads_wr_data = pkt_q_table.heads.wr_data;
    pkt_q_table.heads.rd_data = pkt_q_table_heads_rd_data;
    pkt_q_table_heads_rd_en = pkt_q_table.heads.rd_en;
    pkt_q_table_heads_wr_en = pkt_q_table.heads.wr_en;
    pkt_q_table_l_addrs_addr = pkt_q_table.l_addrs.addr;
    pkt_q_table_l_addrs_wr_data = pkt_q_table.l_addrs.wr_data;
    pkt_q_table.l_addrs.rd_data = pkt_q_table_l_addrs_rd_data;
    pkt_q_table_l_addrs_rd_en = pkt_q_table.l_addrs.rd_en;
    pkt_q_table_l_addrs_wr_en = pkt_q_table.l_addrs.wr_en;
    pkt_q_table_h_addrs_addr = pkt_q_table.h_addrs.addr;
    pkt_q_table_h_addrs_wr_data = pkt_q_table.h_addrs.wr_data;
    pkt_q_table.h_addrs.rd_data = pkt_q_table_h_addrs_rd_data;
    pkt_q_table_h_addrs_rd_en = pkt_q_table.h_addrs.rd_en;
    pkt_q_table_h_addrs_wr_en = pkt_q_table.h_addrs.wr_en;
end

// JTAG signals
logic [29:0]            status_addr_r;
logic                   status_read_r;
logic                   status_write_r;
logic [31:0]            status_writedata_r;
logic [STAT_AWIDTH-1:0] status_addr_sel_r;

logic q_table_wr;
logic q_table_rd_en_jtag;
logic q_table_wr_en_jtag;
logic q_table_rd_wr_en_pcie;
logic q_table_rd_wr_en_pcie_r;
logic q_table_data_en_pcie;
logic q_table_data_en_pcie_r;
logic q_table_jtag_rd_set;
logic q_table_jtag_wr_set;
logic q_table_jtag_wr_data_set;

logic [BRAM_TABLE_IDX_WIDTH-1:0] q_table_addr_jtag;
logic [BRAM_TABLE_IDX_WIDTH-1:0] q_table_addr_pcie;
logic [BRAM_TABLE_IDX_WIDTH-1:0] q_table_addr_jtag_pending;
logic [BRAM_TABLE_IDX_WIDTH-1:0] q_table_addr_pcie_rd_pending;

logic [PKT_Q_TABLE_TAILS_DWIDTH-1:0] q_table_rd_data_b;
logic [PKT_Q_TABLE_TAILS_DWIDTH-1:0] q_table_rd_data_b_jtag;

logic q_table_rd_data_b_pcie_ready;
logic q_table_rd_data_b_jtag_ready;
logic q_table_rd_data_b_ready_from_jtag;

logic q_table_rd_pending_from_jtag;
logic q_table_pcie_rd_set;

logic dsc_rd_en_r;
logic dsc_rd_en_r2;
logic pkt_rd_en_r;
logic pkt_rd_en_r2;

logic pcie_bram_rd;
logic pcie_bram_rd_r;
logic pcie_bram_rd_r2;

logic [31:0] c2f_head;
logic [31:0] c2f_tail;
logic [63:0] c2f_kmem_addr;
logic [63:0] c2f_head_addr;
assign c2f_head = 32'b0; // FIXME(sadok) remove when reenabling c2f path

logic [31:0] q_table_data;
logic [31:0] q_table_data_r;
logic [31:0] q_table_data_jtag;
logic [31:0] q_table_data_jtag_pending;

localparam NB_TABLES = 4; // We need 4 tables for every queue
localparam AWIDTH_NB_TABLES = $clog2(NB_TABLES);
localparam C2F_HEAD_OFFSET = (NB_TABLES + 1) * REG_SIZE;

logic [AWIDTH_NB_TABLES-1:0] q_table_jtag;
logic [AWIDTH_NB_TABLES-1:0] q_table_pcie;
logic [AWIDTH_NB_TABLES-1:0] q_table_jtag_pending;
logic [AWIDTH_NB_TABLES-1:0] last_rd_b_table;
logic [AWIDTH_NB_TABLES-1:0] last_rd_b_table_r;
logic [AWIDTH_NB_TABLES-1:0] last_rd_b_table_r2;

logic [31:0] pkt_q_table_pcie_out;
logic [31:0] pkt_q_table_pcie_out_r;

assign q_table_addr_pcie = pkt_q_table_pcie_out[0 +: BRAM_TABLE_IDX_WIDTH];
assign q_table_pcie = 
    pkt_q_table_pcie_out[BRAM_TABLE_IDX_WIDTH +: AWIDTH_NB_TABLES];
assign q_table_wr = pkt_q_table_pcie_out[31]; // 0 = rd, 1 = wr

logic [31:0] control_regs_r2 [NB_CONTROL_REGS];
logic [31:0] control_regs_r  [NB_CONTROL_REGS];

// Avoid short path/long path timing problem.
always @(posedge pcie_clk) begin
    integer i;
    for (i = 0; i < NB_CONTROL_REGS; i = i + 1) begin
        control_regs_r[i] <= control_regs_r2[i];
        control_regs[i] <= control_regs_r[i];
    end
end

// JTAG
always@(posedge jtag_clk) begin
    automatic logic [JTAG_ADDR_WIDTH-1:0] jtag_reg = 
        status_addr_r[JTAG_ADDR_WIDTH-1:0];

    status_addr_r       <= status_addr;
    status_addr_sel_r   <= status_addr[29:30-STAT_AWIDTH];

    status_read_r       <= status_read;
    status_write_r      <= status_write;
    status_writedata_r  <= status_writedata;

    status_readdata_valid <= 0;
    q_table_rd_en_jtag <= 0;
    q_table_wr_en_jtag <= 0;

    if (!pcie_reset_n) begin
        integer i;
        for (i = 0; i < NB_CONTROL_REGS; i = i + 1) begin
            control_regs_r2[i] <= 0;
        end
    end

    if (q_table_rd_data_b_ready_from_jtag && q_table_rd_pending_from_jtag) begin
        status_readdata <= q_table_rd_data_b_jtag;
        status_readdata_valid <= 1;
        q_table_rd_pending_from_jtag <= 0;
    end

    if (status_addr_sel_r == PCIE & status_read_r) begin
        if (jtag_reg < NB_CONTROL_REGS) begin
            status_readdata <= control_regs_r2[jtag_reg];
            status_readdata_valid <= 1;
        end else begin
            q_table_jtag <= {jtag_reg-NB_CONTROL_REGS}[AWIDTH_NB_TABLES-1:0];
            q_table_addr_jtag <=
                {jtag_reg - NB_CONTROL_REGS}[
                    AWIDTH_NB_TABLES +:BRAM_TABLE_IDX_WIDTH];
            q_table_rd_pending_from_jtag <= 1;
            q_table_rd_en_jtag <= 1;
        end
    end else if (status_addr_sel_r == PCIE & status_write_r) begin
        if (jtag_reg < NB_CONTROL_REGS) begin
            control_regs_r2[jtag_reg] <= status_writedata_r;
        end else begin
            q_table_jtag <= {jtag_reg-NB_CONTROL_REGS}[AWIDTH_NB_TABLES-1:0];
            q_table_data_jtag <= status_writedata_r;
            q_table_addr_jtag <=
                {jtag_reg - NB_CONTROL_REGS}[
                    AWIDTH_NB_TABLES +:BRAM_TABLE_IDX_WIDTH];
            q_table_wr_en_jtag <= 1;
        end
    end
end

logic cpu_reg_region;
logic cpu_reg_region_r1;
logic cpu_reg_region_r2;
assign cpu_reg_region = pcie_address_0[PCIE_ADDR_WIDTH-1:6] < MMIO_OFFSET;

logic pcie_reg_read;
assign pcie_reg_read = cpu_reg_region & pcie_read_0;

// We choose the right set of registers based on the page (the page's index LSB
// is at bit 12 of the memory address, the MSB depends on the number of queues
// we support). The first MAX_NB_FLOWS pages hold the packet queues and the next
// MAX_NB_APPS pages hold the descriptor queues.
logic [BRAM_TABLE_IDX_WIDTH-1:0] page_idx;
assign page_idx = pcie_address_0[12 +: BRAM_TABLE_IDX_WIDTH];

assign q_table_rd_data_b_pcie_ready = 
    (dsc_rd_en_r2 | pkt_rd_en_r2) & pcie_bram_rd_r2;

// We share a single BRAM port among: PCIe writes, PCIe reads and JTAG reads.
// We serve simultaneous requests following this priority. That way, we only
// serve PCIe reads when there are no PCIe writes and we only serve JTAG reads
// when there are no PCIe writes or reads.
always @(posedge pcie_clk) begin
    dsc_q_table.tails.wr_en <= 0;
    dsc_q_table.heads.wr_en <= 0;
    dsc_q_table.l_addrs.wr_en <= 0;
    dsc_q_table.h_addrs.wr_en <= 0;

    dsc_q_table.tails.rd_en <= 0;
    dsc_q_table.heads.rd_en <= 0;
    dsc_q_table.l_addrs.rd_en <= 0;
    dsc_q_table.h_addrs.rd_en <= 0;

    pkt_q_table.tails.wr_en <= 0;
    pkt_q_table.heads.wr_en <= 0;
    pkt_q_table.l_addrs.wr_en <= 0;
    pkt_q_table.h_addrs.wr_en <= 0;

    pkt_q_table.tails.rd_en <= 0;
    pkt_q_table.heads.rd_en <= 0;
    pkt_q_table.l_addrs.rd_en <= 0;
    pkt_q_table.h_addrs.rd_en <= 0;

    last_rd_b_table <= 0;

    // TODO(sadok) We assume PCIe reads only come after the previous completes.
    // If that is not true, we may need to keep a queue of requests.
    if (pcie_reg_read) begin
        q_table_addr_pcie_rd_pending <= page_idx;
        q_table_pcie_rd_set <= 1;
    end

    // We assign the JTAG read enable and the address to pending registers.
    // These are read opportunistically, when there is no operation from the
    // PCIe. This lets us share the same BRAM port for PCIe and JTAG
    if (q_table_rd_wr_en_pcie) begin
        q_table_jtag_pending <= q_table_pcie;
        if (q_table_wr) begin
            q_table_jtag_wr_set <= 1;
        end else begin
            q_table_jtag_rd_set <= 1;
        end
        q_table_addr_jtag_pending <= q_table_addr_pcie;
    end

    if (q_table_data_en_pcie) begin
        q_table_jtag_wr_data_set <= 1;
        q_table_data_jtag_pending <= q_table_data;
    end

    q_table_rd_data_b_jtag_ready <= 0;

    if (!pcie_reset_n) begin
        q_table_pcie_rd_set <= 0;
        q_table_jtag_wr_set <= 0;
        q_table_jtag_wr_data_set <= 0;
        q_table_jtag_rd_set <= 0;
        q_table_rd_data_b <= 0;
        c2f_kmem_addr <= 0;
    end else if (pcie_write_0) begin // PCIe write
        if (page_idx < MAX_NB_FLOWS) begin
            automatic logic [BRAM_TABLE_IDX_WIDTH-1:0] address = page_idx;
        
            // if (pcie_byteenable_0[0*REG_SIZE +:REG_SIZE] == {REG_SIZE{1'b1}}) begin
            //     pkt_q_table.tails.wr_data <= pcie_writedata_0[0*32 +: 32];
            //     pkt_q_table.tails.wr_en <= 1;
            //     pkt_q_table.tails.addr <= address;
            // end
            if (pcie_byteenable_0[1*REG_SIZE +:REG_SIZE] == {REG_SIZE{1'b1}}) begin
                pkt_q_table.heads.wr_data <= pcie_writedata_0[1*32 +: 32];
                pkt_q_table.heads.wr_en <= 1;
                pkt_q_table.heads.addr <= address;
            end
            if (pcie_byteenable_0[2*REG_SIZE +:REG_SIZE] == {REG_SIZE{1'b1}}) begin
                pkt_q_table.l_addrs.wr_data <= pcie_writedata_0[2*32 +: 32];
                pkt_q_table.l_addrs.wr_en <= 1;
                pkt_q_table.l_addrs.addr <= address;
            end
            if (pcie_byteenable_0[3*REG_SIZE +:REG_SIZE] == {REG_SIZE{1'b1}}) begin
                pkt_q_table.h_addrs.wr_data <= pcie_writedata_0[3*32 +: 32];
                pkt_q_table.h_addrs.wr_en <= 1;
                pkt_q_table.h_addrs.addr <= address;
            end
        end else begin
            automatic logic [BRAM_TABLE_IDX_WIDTH-1:0] address;
            address = page_idx - MAX_NB_FLOWS;

            // if (pcie_byteenable_0[0*REG_SIZE +:REG_SIZE] == {REG_SIZE{1'b1}}) begin
            //     dsc_q_table.tails.wr_data <= pcie_writedata_0[0*32 +: 32];
            //     dsc_q_table.tails.wr_en <= 1;
            //     dsc_q_table.tails.addr <= address;
            // end
            if (pcie_byteenable_0[1*REG_SIZE +:REG_SIZE] == {REG_SIZE{1'b1}}) begin
                dsc_q_table.heads.wr_data <= pcie_writedata_0[1*32 +: 32];
                dsc_q_table.heads.wr_en <= 1;
                dsc_q_table.heads.addr <= address;
            end
            if (pcie_byteenable_0[2*REG_SIZE +:REG_SIZE] == {REG_SIZE{1'b1}}) begin
                dsc_q_table.l_addrs.wr_data <= pcie_writedata_0[2*32 +: 32];
                dsc_q_table.l_addrs.wr_en <= 1;
                dsc_q_table.l_addrs.addr <= address;
            end
            if (pcie_byteenable_0[3*REG_SIZE +:REG_SIZE] == {REG_SIZE{1'b1}}) begin
                dsc_q_table.h_addrs.wr_data <= pcie_writedata_0[3*32 +: 32];
                dsc_q_table.h_addrs.wr_en <= 1;
                dsc_q_table.h_addrs.addr <= address;
            end
        end

        // TODO(sadok) implement CPU-> FPGA path
        // // CPU -> FPGA
        // // TODO(sadok) This assumes a single control queue. We also need to add
        // //             TX data queues eventually
        // if (pcie_byteenable_0[8*REG_SIZE +:REG_SIZE] == {REG_SIZE{1'b1}}) begin
        //     c2f_tail <= pcie_writedata_0[8*32 +: 32];
        // end
        // // if (pcie_byteenable_0[9*REG_SIZE +:REG_SIZE] == {REG_SIZE{1'b1}}) begin
        // //     c2f_head <= pcie_writedata_0[9*32 +: 32];
        // // end
        // if (pcie_byteenable_0[10*REG_SIZE +:REG_SIZE] == {REG_SIZE{1'b1}}) begin
        //     c2f_kmem_addr[31:0] <= pcie_writedata_0[10*32 +: 32];
        // end
        // if (pcie_byteenable_0[11*REG_SIZE +:REG_SIZE] == {REG_SIZE{1'b1}}) begin
        //     c2f_kmem_addr[63:32] <= pcie_writedata_0[11*32 +: 32];
        // end
    end else if (q_table_pcie_rd_set) begin // PCIe read
        q_table_pcie_rd_set <= 0;

        if (q_table_addr_pcie_rd_pending < MAX_NB_FLOWS) begin
            automatic logic [BRAM_TABLE_IDX_WIDTH-1:0] address = 
                q_table_addr_pcie_rd_pending;

            pkt_q_table.tails.rd_en <= 1;
            pkt_q_table.tails.addr <= address;

            pkt_q_table.heads.rd_en <= 1;
            pkt_q_table.heads.addr <= address;

            pkt_q_table.l_addrs.rd_en <= 1;
            pkt_q_table.l_addrs.addr <= address;

            pkt_q_table.h_addrs.rd_en <= 1;
            pkt_q_table.h_addrs.addr <= address;
        end else begin
            automatic logic [BRAM_TABLE_IDX_WIDTH-1:0] address = 
                q_table_addr_pcie_rd_pending - MAX_NB_FLOWS;

            dsc_q_table.tails.rd_en <= 1;
            dsc_q_table.tails.addr <= address;

            dsc_q_table.heads.rd_en <= 1;
            dsc_q_table.heads.addr <= address;

            dsc_q_table.l_addrs.rd_en <= 1;
            dsc_q_table.l_addrs.addr <= address;

            dsc_q_table.h_addrs.rd_en <= 1;
            dsc_q_table.h_addrs.addr <= address;
        end
    end else if (q_table_jtag_wr_set && q_table_jtag_wr_data_set) begin 
        // JTAG write
        q_table_jtag_wr_set <= 0;
        q_table_jtag_wr_data_set <= 0;
        if (q_table_addr_jtag_pending < MAX_NB_FLOWS) begin
            automatic logic [BRAM_TABLE_IDX_WIDTH-1:0] address = 
                q_table_addr_jtag_pending;
            case (q_table_jtag_pending)
                0: begin
                    pkt_q_table.tails.wr_data <= q_table_data_jtag_pending;
                    pkt_q_table.tails.wr_en <= 1;
                    pkt_q_table.tails.addr <= address;
                end
                1: begin
                    pkt_q_table.heads.wr_data <= q_table_data_jtag_pending;
                    pkt_q_table.heads.wr_en <= 1;
                    pkt_q_table.heads.addr <= address;
                end
                2: begin
                    pkt_q_table.l_addrs.wr_data <= q_table_data_jtag_pending;
                    pkt_q_table.l_addrs.wr_en <= 1;
                    pkt_q_table.l_addrs.addr <= address;
                end
                3: begin
                    pkt_q_table.h_addrs.wr_data <= q_table_data_jtag_pending;
                    pkt_q_table.h_addrs.wr_en <= 1;
                    pkt_q_table.h_addrs.addr <= address;
                end
            endcase
        end else begin
            automatic logic [BRAM_TABLE_IDX_WIDTH-1:0] address = 
                q_table_addr_jtag_pending - MAX_NB_FLOWS;
            case (q_table_jtag_pending)
                0: begin
                    dsc_q_table.tails.wr_data <= q_table_data_jtag_pending;
                    dsc_q_table.tails.wr_en <= 1;
                    dsc_q_table.tails.addr <= address;
                end
                1: begin
                    dsc_q_table.heads.wr_data <= q_table_data_jtag_pending;
                    dsc_q_table.heads.wr_en <= 1;
                    dsc_q_table.heads.addr <= address;
                end
                2: begin
                    dsc_q_table.l_addrs.wr_data <= q_table_data_jtag_pending;
                    dsc_q_table.l_addrs.wr_en <= 1;
                    dsc_q_table.l_addrs.addr <= address;
                end
                3: begin
                    dsc_q_table.h_addrs.wr_data <= q_table_data_jtag_pending;
                    dsc_q_table.h_addrs.wr_en <= 1;
                    dsc_q_table.h_addrs.addr <= address;
                end
            endcase
        end
    end else if (q_table_jtag_rd_set) begin // JTAG read
        q_table_jtag_rd_set <= 0;
        last_rd_b_table <= q_table_jtag_pending;

        if (q_table_addr_jtag_pending < MAX_NB_FLOWS) begin
            automatic logic [BRAM_TABLE_IDX_WIDTH-1:0] address = 
                q_table_addr_jtag_pending;
            case (q_table_jtag_pending)
                0: begin
                    pkt_q_table.tails.rd_en <= 1;
                    pkt_q_table.tails.addr <= address;
                end
                1: begin
                    pkt_q_table.heads.rd_en <= 1;
                    pkt_q_table.heads.addr <= address;
                end
                2: begin
                    pkt_q_table.l_addrs.rd_en <= 1;
                    pkt_q_table.l_addrs.addr <= address;
                end
                3: begin
                    pkt_q_table.h_addrs.rd_en <= 1;
                    pkt_q_table.h_addrs.addr <= address;
                end
            endcase
        end else begin
            automatic logic [BRAM_TABLE_IDX_WIDTH-1:0] address = 
                q_table_addr_jtag_pending - MAX_NB_FLOWS;
            case (q_table_jtag_pending)
                0: begin
                    dsc_q_table.tails.rd_en <= 1;
                    dsc_q_table.tails.addr <= address;
                end
                1: begin
                    dsc_q_table.heads.rd_en <= 1;
                    dsc_q_table.heads.addr <= address;
                end
                2: begin
                    dsc_q_table.l_addrs.rd_en <= 1;
                    dsc_q_table.l_addrs.addr <= address;
                end
                3: begin
                    dsc_q_table.h_addrs.rd_en <= 1;
                    dsc_q_table.h_addrs.addr <= address;
                end
            endcase
        end
        
    end

    last_rd_b_table_r <= last_rd_b_table;
    last_rd_b_table_r2 <= last_rd_b_table_r;

    dsc_rd_en_r <= dsc_q_table.tails.rd_en | dsc_q_table.heads.rd_en |
               dsc_q_table.l_addrs.rd_en | dsc_q_table.h_addrs.rd_en;

    pkt_rd_en_r <= pkt_q_table.tails.rd_en | pkt_q_table.heads.rd_en |
                pkt_q_table.l_addrs.rd_en | pkt_q_table.h_addrs.rd_en;

    dsc_rd_en_r2 <= dsc_rd_en_r;
    pkt_rd_en_r2 <= pkt_rd_en_r;

    // signals if this is a PCIe (1) or a JTAG (0) read
    pcie_bram_rd <= q_table_pcie_rd_set;
    pcie_bram_rd_r <= pcie_bram_rd;
    pcie_bram_rd_r2 <= pcie_bram_rd_r;

    // JTAG read is ready
    if (!pcie_bram_rd_r2) begin
        if (dsc_rd_en_r2) begin
            case (last_rd_b_table_r2)
                0: begin
                    q_table_rd_data_b <= dsc_q_table.tails.rd_data;
                end
                1: begin
                    q_table_rd_data_b <= dsc_q_table.heads.rd_data;
                end
                2: begin
                    q_table_rd_data_b <= dsc_q_table.l_addrs.rd_data;
                end
                3: begin
                    q_table_rd_data_b <= dsc_q_table.h_addrs.rd_data;
                end
            endcase
            q_table_rd_data_b_jtag_ready <= 1;
        end

        if (pkt_rd_en_r2) begin
            case (last_rd_b_table_r2)
                0: begin
                    q_table_rd_data_b <= pkt_q_table.tails.rd_data;
                end
                1: begin
                    q_table_rd_data_b <= pkt_q_table.heads.rd_data;
                end
                2: begin
                    q_table_rd_data_b <= pkt_q_table.l_addrs.rd_data;
                end
                3: begin
                    q_table_rd_data_b <= pkt_q_table.h_addrs.rd_data;
                end
            endcase
            q_table_rd_data_b_jtag_ready <= 1;
        end
    end
end


// PDU_BUFFER
// CPU side MMIO read MUX. If pkt_rd_en_r2, we return pkt queue info else, we
// return dsc queue info.
always @(posedge pcie_clk) begin
    if (!pcie_reset_n) begin
        pcie_readdata_0 <= 0;
        pcie_readdatavalid_0 <= 0;
    end else begin
        if (pkt_rd_en_r2) begin
            pcie_readdata_0 <= {
                256'h0, c2f_kmem_addr, c2f_head, c2f_tail,
                pkt_q_table.h_addrs.rd_data, pkt_q_table.l_addrs.rd_data,
                pkt_q_table.heads.rd_data, pkt_q_table.tails.rd_data
            };
        end else begin
            pcie_readdata_0 <= {
                256'h0, c2f_kmem_addr, c2f_head, c2f_tail,
                dsc_q_table.h_addrs.rd_data, dsc_q_table.l_addrs.rd_data,
                dsc_q_table.heads.rd_data, dsc_q_table.tails.rd_data
            };
        end
        
        pcie_readdatavalid_0 <= q_table_rd_data_b_pcie_ready;
    end
end

always @(posedge pcie_clk) begin
    // two cycle read delay
    cpu_reg_region_r1 <= cpu_reg_region;
    cpu_reg_region_r2 <= cpu_reg_region_r1;

    // jtag_to_pcie_fifo output pipeline
    q_table_rd_wr_en_pcie <= q_table_rd_wr_en_pcie_r;
    pkt_q_table_pcie_out <= pkt_q_table_pcie_out_r;
    q_table_data_en_pcie <= q_table_data_en_pcie_r;
    q_table_data <= q_table_data_r;
end

// PCIe and JTAG are in different clock domains, we use the following
// dual-clocked FIFOs to transfer data between the two
dc_fifo_reg_core  jtag_to_pcie_fifo (
    .wrclock               (jtag_clk),
    .wrreset_n             (pcie_reset_n),
    .rdclock               (pcie_clk),
    .rdreset_n             (pcie_reset_n),
    .avalonst_sink_valid   (q_table_rd_en_jtag | q_table_wr_en_jtag),
    .avalonst_sink_data    ({
        q_table_wr_en_jtag,
        {{32 - BRAM_TABLE_IDX_WIDTH - AWIDTH_NB_TABLES - 1}{1'b0}},
        q_table_jtag,
        q_table_addr_jtag
    }),
    .avalonst_source_valid (q_table_rd_wr_en_pcie_r),
    .avalonst_source_data  (pkt_q_table_pcie_out_r)
);

dc_fifo_reg_core  jtag_to_pcie_wr_data_fifo (
    .wrclock               (jtag_clk),
    .wrreset_n             (pcie_reset_n),
    .rdclock               (pcie_clk),
    .rdreset_n             (pcie_reset_n),
    .avalonst_sink_valid   (q_table_wr_en_jtag),
    .avalonst_sink_data    (q_table_data_jtag),
    .avalonst_source_valid (q_table_data_en_pcie_r),
    .avalonst_source_data  (q_table_data_r)
);

dc_fifo_reg_core  pcie_to_jtag_fifo (
    .wrclock               (pcie_clk),
    .wrreset_n             (pcie_reset_n),
    .rdclock               (jtag_clk),
    .rdreset_n             (pcie_reset_n),
    .avalonst_sink_valid   (q_table_rd_data_b_jtag_ready),
    .avalonst_sink_data    (q_table_rd_data_b),
    .avalonst_source_valid (q_table_rd_data_b_ready_from_jtag),
    .avalonst_source_data  (q_table_rd_data_b_jtag)
);

endmodule
