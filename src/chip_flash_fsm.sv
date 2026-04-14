module chip_flash_fsm #(
    parameter int unsigned PWR_WAIT_CYCLES = 1024,
    parameter int unsigned RST_WAIT_CYCLES = 1024,
    parameter int unsigned SPI_DIV         = 4,
    parameter bit          USE_4BYTE_ADDR      = 1'b1,
    parameter logic [31:0] FLASH_WEIGHT_BASE   = 32'h0000_0000,
    parameter logic [31:0] FLASH_THRESH_BASE   = 32'h0010_0000,

    parameter int unsigned NUM_CLASSES         = 4,
    parameter int unsigned GRID_SIZE           = 16,
    parameter int unsigned READOUT_BINS        = 8,
    parameter int unsigned WEIGHT_BITS         = 8,
    parameter int unsigned SCORE_BITS          = 36,

    parameter bit          THRESH_BIG_ENDIAN   = 1'b1
) (
    input  logic clk,
    input  logic rst_n,

    input  logic boot_req_i,
    input  logic reload_req_i,
    input  logic debug_req_i,

    input  logic spi_miso_i,
    output logic spi_cs_n_o,
    output logic spi_sck_o,
    output logic spi_mosi_o,

    output logic                                       weight_wr_valid_o,
    output logic [$clog2(NUM_CLASSES)-1:0]             weight_wr_class_o,
    output logic [$clog2(READOUT_BINS*GRID_SIZE*GRID_SIZE)-1:0] weight_wr_addr_o,
    output logic [WEIGHT_BITS-1:0]                     weight_wr_data_o,

    output logic                                       thresh_wr_valid_o,
    output logic [$clog2(2*NUM_CLASSES)-1:0]           thresh_wr_addr_o,
    output logic [SCORE_BITS-1:0]                      thresh_wr_data_o,

    output logic core_rst_o,

    output logic boot_done_o,
    output logic boot_fail_o,
    output logic [3:0] main_state_dbg_o,
    output logic [5:0] load_state_dbg_o,
    output logic [7:0] id_mfr_o,
    output logic [7:0] id_type_o,
    output logic [7:0] id_capacity_o
);

    localparam int unsigned FEATURE_COUNT = READOUT_BINS * GRID_SIZE * GRID_SIZE;
    localparam int unsigned WEIGHT_ADDR_W = $clog2(FEATURE_COUNT);
    localparam int unsigned THRESH_COUNT  = 2 * NUM_CLASSES;
    localparam int unsigned THRESH_ADDR_W = $clog2(THRESH_COUNT);
    localparam int unsigned THRESH_BYTES  = (SCORE_BITS + 7) / 8;
    localparam int unsigned SPI_DIV_W     = (SPI_DIV <= 1) ? 1 : $clog2(SPI_DIV);
    localparam int unsigned CLASS_W       = (NUM_CLASSES <= 1) ? 1 : $clog2(NUM_CLASSES);
    localparam logic [7:0] CMD_RSTEN = 8'h66;
    localparam logic [7:0] CMD_RST   = 8'h99;
    localparam logic [7:0] CMD_RDID  = 8'h9F;
    localparam logic [7:0] CMD_READ  = 8'h03;
    localparam logic [7:0] CMD_4READ = 8'h13;

    typedef enum logic [3:0] {
        ST_BOOT  = 4'd0,
        ST_LOAD  = 4'd1,
        ST_RUN   = 4'd2,
        ST_DEBUG = 4'd3
    } main_state_t;

    typedef enum logic [5:0] {
        LD_IDLE            = 6'd0,
        LD_WAIT_PWR        = 6'd1,
        LD_SEND_RSTEN      = 6'd2,
        LD_SEND_RST        = 6'd3,
        LD_WAIT_RESET_GAP  = 6'd4,
        LD_SEND_RDID       = 6'd5,
        LD_RDID_BYTES      = 6'd6,
        LD_CHECK_ID        = 6'd7,
        LD_W_OPEN          = 6'd8,
        LD_W_ADDR          = 6'd9,
        LD_W_DATA          = 6'd10,
        LD_W_WRITE         = 6'd11,
        LD_W_NEXT          = 6'd12,
        LD_W_CLOSE         = 6'd13,
        LD_T_OPEN          = 6'd14,
        LD_T_ADDR          = 6'd15,
        LD_T_DATA          = 6'd16,
        LD_T_WRITE         = 6'd17,
        LD_T_NEXT          = 6'd18,
        LD_DONE            = 6'd19,
        LD_FAIL            = 6'd20
    } load_state_t;

    main_state_t main_state;
    load_state_t load_state;
    logic [31:0] pwr_wait_cnt;
    logic [31:0] rst_wait_cnt;
    logic [1:0]  rdid_idx;

    logic [2:0]  addr_bytes_left;

    logic [31:0] weight_flash_addr;
    logic [31:0] thresh_flash_addr;

    logic [CLASS_W-1:0]       weight_class_idx;
    logic [WEIGHT_ADDR_W-1:0] weight_word_idx;

    logic [THRESH_ADDR_W-1:0] thresh_entry_idx;
    logic [$clog2(THRESH_BYTES+1)-1:0] thresh_byte_idx;
    logic [THRESH_BYTES*8-1:0] thresh_pack_reg;

    logic [7:0] tx_byte;
    logic [7:0] rx_byte;
    logic [7:0] tx_shift;
    logic [7:0] rx_shift;
    logic [2:0] bit_idx;
    logic [SPI_DIV_W-1:0] spi_div_cnt;
    logic spi_busy;
    logic spi_done;
    logic spi_start;
    logic spi_high_phase;

    assign main_state_dbg_o = main_state;
    assign load_state_dbg_o = load_state;
    logic [7:0] id_mfr_r;
    logic [7:0] id_type_r;
    logic [7:0] id_capacity_r;
    assign id_mfr_o      = id_mfr_r;
    assign id_type_o     = id_type_r;
    assign id_capacity_o = id_capacity_r;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            main_state        <= ST_BOOT;
            load_state        <= LD_IDLE;
            pwr_wait_cnt      <= '0;
            rst_wait_cnt      <= '0;
            rdid_idx          <= '0;
            addr_bytes_left   <= '0;
            weight_flash_addr <= FLASH_WEIGHT_BASE;
            thresh_flash_addr <= FLASH_THRESH_BASE;
            weight_class_idx  <= '0;
            weight_word_idx   <= '0;
            thresh_entry_idx  <= '0;
            thresh_byte_idx   <= '0;
            thresh_pack_reg   <= '0;

            spi_cs_n_o        <= 1'b1;
            spi_sck_o         <= 1'b0;
            spi_mosi_o        <= 1'b0;
            spi_busy          <= 1'b0;
            spi_done          <= 1'b0;
            spi_start         <= 1'b0;
            spi_high_phase    <= 1'b0;
            spi_div_cnt       <= '0;
            bit_idx           <= 3'd0;
            tx_byte           <= 8'h00;
            rx_byte           <= 8'h00;
            tx_shift          <= 8'h00;
            rx_shift          <= 8'h00;

            weight_wr_valid_o <= 1'b0;
            weight_wr_class_o <= '0;
            weight_wr_addr_o  <= '0;
            weight_wr_data_o  <= '0;
            thresh_wr_valid_o <= 1'b0;
            thresh_wr_addr_o  <= '0;
            thresh_wr_data_o  <= '0;

            core_rst_o        <= 1'b1;
            boot_done_o       <= 1'b0;
            boot_fail_o       <= 1'b0;

            id_mfr_r          <= 8'h00;
            id_type_r         <= 8'h00;
            id_capacity_r     <= 8'h00;
        end else begin
            spi_start         <= 1'b0;
            spi_done          <= 1'b0;
            weight_wr_valid_o <= 1'b0;
            thresh_wr_valid_o <= 1'b0;

            if (spi_start && !spi_busy) begin
                spi_busy       <= 1'b1;
                spi_high_phase <= 1'b0;
                spi_div_cnt    <= '0;
                bit_idx        <= 3'd7;
                tx_shift       <= tx_byte;
                rx_shift       <= 8'h00;
                spi_sck_o      <= 1'b0;
                spi_mosi_o     <= tx_byte[7];
            end else if (spi_busy) begin
                if (spi_div_cnt == SPI_DIV-1) begin
                    spi_div_cnt <= '0;

                    if (!spi_high_phase) begin
                        spi_sck_o      <= 1'b1;
                        rx_shift       <= {rx_shift[6:0], spi_miso_i};
                        spi_high_phase <= 1'b1;
                    end else begin
                        // Falling edge of SCK:
                        // - prepare the next MOSI bit
                        spi_sck_o      <= 1'b0;
                        spi_high_phase <= 1'b0;

                        if (bit_idx == 3'd0) begin
                            // Last bit of this byte just completed.
                            spi_busy <= 1'b0;
                            spi_done <= 1'b1;
                            rx_byte  <= {rx_shift[6:0], spi_miso_i};
                        end else begin
                            bit_idx    <= bit_idx - 3'd1;
                            tx_shift   <= {tx_shift[6:0], 1'b0};
                            spi_mosi_o <= tx_shift[6];
                        end
                    end
                end else begin
                    spi_div_cnt <= spi_div_cnt + 1'b1;
                end
            end

            unique case (main_state)
                ST_BOOT: begin
                    core_rst_o  <= 1'b1;
                    boot_done_o <= 1'b0;
                    boot_fail_o <= 1'b0;
                    spi_cs_n_o  <= 1'b1;

                    if (debug_req_i) begin
                        main_state <= ST_DEBUG;
                        load_state <= LD_IDLE;
                    end else if (boot_req_i || reload_req_i) begin
                        main_state        <= ST_LOAD;
                        load_state        <= LD_WAIT_PWR;
                        pwr_wait_cnt      <= '0;
                        rst_wait_cnt      <= '0;
                        weight_flash_addr <= FLASH_WEIGHT_BASE;
                        thresh_flash_addr <= FLASH_THRESH_BASE;
                        weight_class_idx  <= '0;
                        weight_word_idx   <= '0;
                        thresh_entry_idx  <= '0;
                        thresh_byte_idx   <= '0;
                        thresh_pack_reg   <= '0;
                        id_mfr_r          <= 8'h00;
                        id_type_r         <= 8'h00;
                        id_capacity_r     <= 8'h00;
                    end
                end

                ST_LOAD: begin
                    core_rst_o <= 1'b1;

                    if (debug_req_i) begin
                        main_state <= ST_DEBUG;
                        load_state <= LD_IDLE;
                        spi_cs_n_o <= 1'b1;
                    end else begin
                        unique case (load_state)
                            LD_IDLE: begin
                                load_state   <= LD_WAIT_PWR;
                                pwr_wait_cnt <= '0;
                            end

                            LD_WAIT_PWR: begin
                                // Keep flash deselected during initial wait.
                                spi_cs_n_o <= 1'b1;
                                if (pwr_wait_cnt == PWR_WAIT_CYCLES-1) begin
                                    load_state <= LD_SEND_RSTEN;
                                end else begin
                                    pwr_wait_cnt <= pwr_wait_cnt + 1'b1;
                                end
                            end

                            LD_SEND_RSTEN: begin
                                // Begin flash software reset sequence.
                                // Command 66h must be its own transaction.
                                spi_cs_n_o  <= 1'b0;
                                tx_byte     <= CMD_RSTEN;
                                spi_start   <= 1'b1;
                                if (spi_done) begin
                                    spi_cs_n_o <= 1'b1;
                                    load_state <= LD_SEND_RST;
                                end
                            end

                            LD_SEND_RST: begin
                                // Second half of software reset sequence: 99h.
                                spi_cs_n_o  <= 1'b0;
                                tx_byte     <= CMD_RST;
                                spi_start   <= 1'b1;
                                if (spi_done) begin
                                    spi_cs_n_o   <= 1'b1;
                                    rst_wait_cnt <= '0;
                                    load_state   <= LD_WAIT_RESET_GAP;
                                end
                            end

                            LD_WAIT_RESET_GAP: begin
                                if (rst_wait_cnt == RST_WAIT_CYCLES-1) begin
                                    load_state <= LD_SEND_RDID;
                                end else begin
                                    rst_wait_cnt <= rst_wait_cnt + 1'b1;
                                end
                            end

                            LD_SEND_RDID: begin
                                spi_cs_n_o <= 1'b0;
                                tx_byte    <= CMD_RDID;
                                spi_start  <= 1'b1;
                                if (spi_done) begin
                                    rdid_idx   <= 2'd0;
                                    tx_byte    <= 8'h00;
                                    spi_start  <= 1'b1;
                                    load_state <= LD_RDID_BYTES;
                                end
                            end

                            LD_RDID_BYTES: begin
                                if (spi_done) begin
                                    // Each received byte is one part of the ID.
                                    if (rdid_idx == 2'd0) begin
                                        id_mfr_r <= rx_byte;
                                        rdid_idx <= 2'd1;
                                        tx_byte  <= 8'h00;
                                        spi_start <= 1'b1;
                                    end else if (rdid_idx == 2'd1) begin
                                        id_type_r <= rx_byte;
                                        rdid_idx  <= 2'd2;
                                        tx_byte   <= 8'h00;
                                        spi_start <= 1'b1;
                                    end else begin
                                        id_capacity_r <= rx_byte;
                                        spi_cs_n_o <= 1'b1;
                                        load_state <= LD_CHECK_ID;
                                    end
                                end //combine with below
                            end

                            LD_CHECK_ID: begin
                                if ((id_mfr_r == 8'h00) || (id_mfr_r == 8'hFF)) begin
                                    load_state <= LD_FAIL;
                                end else begin
                                    weight_flash_addr <= FLASH_WEIGHT_BASE;
                                    weight_class_idx  <= '0;
                                    weight_word_idx   <= '0;
                                    load_state        <= LD_W_OPEN;
                                end
                            end

                            LD_W_OPEN: begin
                                spi_cs_n_o      <= 1'b0;
                                addr_bytes_left <= USE_4BYTE_ADDR ? 3'd4 : 3'd3;
                                tx_byte         <= USE_4BYTE_ADDR ? CMD_4READ : CMD_READ;
                                spi_start       <= 1'b1;
                                load_state      <= LD_W_ADDR;
                            end

                            LD_W_ADDR: begin
                                if (!spi_busy && !spi_done && addr_bytes_left != 0) begin
                                    // Launch the next address byte when the engine is idle.
                                    if (addr_bytes_left == 3'd4)
                                        tx_byte <= weight_flash_addr[31:24];
                                    else if (addr_bytes_left == 3'd3)
                                        tx_byte <= weight_flash_addr[23:16];
                                    else if (addr_bytes_left == 3'd2)
                                        tx_byte <= weight_flash_addr[15:8];
                                    else
                                        tx_byte <= weight_flash_addr[7:0];

                                    spi_start <= 1'b1;
                                end

                                if (spi_done) begin
                                    if (addr_bytes_left == 3'd1) begin
                                        // Address phase is complete.
                                        // Next transfer clocks out first data byte.
                                        load_state <= LD_W_DATA;
                                    end
                                    addr_bytes_left <= addr_bytes_left - 1'b1;
                                end
                            end

                            LD_W_DATA: begin
                                if (!spi_busy && !spi_done) begin
                                    tx_byte   <= 8'h00;
                                    spi_start <= 1'b1;
                                end

                                if (spi_done) begin
                                    weight_wr_data_o <= rx_byte[WEIGHT_BITS-1:0];
                                    load_state       <= LD_W_WRITE;
                                end
                            end

                            LD_W_WRITE: begin
                                weight_wr_valid_o <= 1'b1;
                                weight_wr_class_o <= weight_class_idx;
                                weight_wr_addr_o  <= weight_word_idx;
                                load_state        <= LD_W_NEXT;
                            end

                            LD_W_NEXT: begin
                                weight_flash_addr <= weight_flash_addr + 1'b1;
                                if (weight_word_idx == FEATURE_COUNT-1) begin
                                    weight_word_idx <= '0;

                                    if (weight_class_idx == NUM_CLASSES-1) begin
                                        // Finished all weights for all classes.
                                        load_state <= LD_W_CLOSE;
                                    end else begin
                                        weight_class_idx <= weight_class_idx + 1'b1;
                                        load_state       <= LD_W_DATA;
                                    end
                                end else begin
                                    weight_word_idx <= weight_word_idx + 1'b1;
                                    load_state      <= LD_W_DATA;
                                end
                            end

                            LD_W_CLOSE: begin
                                // End the weight-region read transaction.
                                spi_cs_n_o       <= 1'b1;
                                thresh_flash_addr <= FLASH_THRESH_BASE;
                                thresh_entry_idx  <= '0;
                                thresh_byte_idx   <= '0;
                                thresh_pack_reg   <= '0;
                                load_state        <= LD_T_OPEN;
                            end

                            LD_T_OPEN: begin
                                spi_cs_n_o      <= 1'b0;
                                addr_bytes_left <= USE_4BYTE_ADDR ? 3'd4 : 3'd3;
                                tx_byte    <= USE_4BYTE_ADDR ? CMD_4READ : CMD_READ;
                                spi_start  <= 1'b1;
                                load_state <= LD_T_ADDR;
                            end

                            LD_T_ADDR: begin
                                if (!spi_busy && !spi_done && addr_bytes_left != 0) begin
                                    if (addr_bytes_left == 3'd4)
                                        tx_byte <= thresh_flash_addr[31:24];
                                    else if (addr_bytes_left == 3'd3)
                                        tx_byte <= thresh_flash_addr[23:16];
                                    else if (addr_bytes_left == 3'd2)
                                        tx_byte <= thresh_flash_addr[15:8];
                                    else
                                        tx_byte <= thresh_flash_addr[7:0];

                                    spi_start <= 1'b1;
                                end

                                if (spi_done) begin
                                    if (addr_bytes_left == 3'd1) begin
                                        load_state <= LD_T_DATA;
                                    end
                                    addr_bytes_left <= addr_bytes_left - 1'b1;
                                end
                            end

                            LD_T_DATA: begin
                                if (!spi_busy && !spi_done) begin
                                    // Generate clocks to receive next threshold byte.
                                    tx_byte   <= 8'h00;
                                    spi_start <= 1'b1;
                                end

                                if (spi_done) begin
                                    if (THRESH_BIG_ENDIAN) begin
                                        thresh_pack_reg <= {thresh_pack_reg[(THRESH_BYTES*8)-9:0], rx_byte};
                                    end else begin
                                        thresh_pack_reg[(thresh_byte_idx*8) +: 8] <= rx_byte;
                                    end

                                    if (thresh_byte_idx == THRESH_BYTES-1) begin
                                        load_state <= LD_T_WRITE;
                                    end else begin
                                        thresh_byte_idx <= thresh_byte_idx + 1'b1;
                                    end
                                end
                            end

                            LD_T_WRITE: begin
                                thresh_wr_valid_o <= 1'b1;
                                thresh_wr_addr_o  <= thresh_entry_idx;

                                if (THRESH_BIG_ENDIAN)
                                    thresh_wr_data_o <= thresh_pack_reg[SCORE_BITS-1:0];
                                else
                                    thresh_wr_data_o <= thresh_pack_reg[SCORE_BITS-1:0];

                                load_state <= LD_T_NEXT;
                            end

                            LD_T_NEXT: begin
                                // Move to the next threshold entry.
                                thresh_flash_addr <= thresh_flash_addr + THRESH_BYTES;
                                thresh_pack_reg   <= '0;
                                thresh_byte_idx   <= '0;

                                if (thresh_entry_idx == THRESH_COUNT-1) begin
                                    spi_cs_n_o <= 1'b1;
                                    load_state <= LD_DONE;
                                end else begin
                                    thresh_entry_idx <= thresh_entry_idx + 1'b1;
                                    load_state       <= LD_T_DATA;
                                end //combine with below
                            end

                            LD_DONE: begin
                                boot_done_o <= 1'b1;
                                main_state  <= ST_RUN;
                                load_state  <= LD_IDLE;
                            end

                            LD_FAIL: begin
                                spi_cs_n_o  <= 1'b1;
                                boot_fail_o <= 1'b1;
                                main_state  <= ST_DEBUG;
                            end

                            default: begin
                                load_state <= LD_FAIL;
                            end
                        endcase
                    end
                end

                ST_RUN: begin
                    core_rst_o <= 1'b0;
                    spi_cs_n_o <= 1'b1;

                    if (debug_req_i) begin
                        main_state <= ST_DEBUG;
                        load_state <= LD_IDLE;
                    end else if (reload_req_i) begin
                        main_state        <= ST_LOAD;
                        load_state        <= LD_WAIT_PWR;
                        boot_done_o       <= 1'b0;
                        boot_fail_o       <= 1'b0;
                        pwr_wait_cnt      <= '0;
                        rst_wait_cnt      <= '0;
                        weight_flash_addr <= FLASH_WEIGHT_BASE;
                        thresh_flash_addr <= FLASH_THRESH_BASE;
                        weight_class_idx  <= '0;
                        weight_word_idx   <= '0;
                        thresh_entry_idx  <= '0;
                        thresh_byte_idx   <= '0;
                        thresh_pack_reg   <= '0;
                    end
                end

                ST_DEBUG: begin
                    core_rst_o <= 1'b1;
                    spi_cs_n_o <= 1'b1;

                    if (!debug_req_i && (boot_req_i || reload_req_i)) begin
                        main_state        <= ST_LOAD;
                        load_state        <= LD_WAIT_PWR;
                        boot_done_o       <= 1'b0;
                        boot_fail_o       <= 1'b0;
                        pwr_wait_cnt      <= '0;
                        rst_wait_cnt      <= '0;
                        weight_flash_addr <= FLASH_WEIGHT_BASE;
                        thresh_flash_addr <= FLASH_THRESH_BASE;
                        weight_class_idx  <= '0;
                        weight_word_idx   <= '0;
                        thresh_entry_idx  <= '0;
                        thresh_byte_idx   <= '0;
                        thresh_pack_reg   <= '0;
                    end
                end

                default: begin
                    main_state <= ST_BOOT;
                    load_state <= LD_IDLE;
                end
            endcase
        end
    end

endmodule

