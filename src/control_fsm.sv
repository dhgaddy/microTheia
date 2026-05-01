module control_fsm #(
    parameter int unsigned PWR_WAIT_CYCLES = 1024
) (
    input  logic clk,
    input  logic rst_n,
    input  logic boot_req_i,
    input  logic reload_req_i,
    input  logic debug_req_i,
    input  logic evt_reads_done,
    input  logic evt_ld_bypass,

    output logic evt_ld_en,
    output logic core_rst_o,
    output logic boot_done_o,
    output logic boot_fail_o,
    output logic [3:0] main_state_dbg_o,
    output logic [5:0] load_state_dbg_o
);

    typedef enum logic [3:0] {
        ST_BOOT  = 4'd0,
        ST_LOAD  = 4'd1,
        ST_RUN   = 4'd2,
        ST_DEBUG = 4'd3
    } main_state_t;

    typedef enum logic [5:0] {
        LD_IDLE      = 6'd0,
        LD_WAIT_PWR  = 6'd1,
        LD_OPEN      = 6'd2,
        LD_WAIT      = 6'd3 , 
        LD_DONE      = 6'd4,
        LD_FAIL      = 6'd5
    } load_state_t;

    main_state_t main_state;
    load_state_t load_state;

    logic [31:0] pwr_wait_cnt;

    assign main_state_dbg_o = main_state;
    assign load_state_dbg_o = load_state;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            main_state        <= ST_BOOT;
            load_state        <= LD_IDLE;
            pwr_wait_cnt      <= '0;
            core_rst_o        <= 1'b1;
            boot_done_o       <= 1'b0;
            boot_fail_o       <= 1'b0;
            evt_ld_en         <= 1'b0;

        end else begin
            // unique
            case (main_state)
                ST_BOOT: begin
                    core_rst_o  <= 1'b1;
                    boot_done_o <= 1'b0;
                    boot_fail_o <= 1'b0;

                    if (debug_req_i) begin
                        main_state <= ST_DEBUG;
                        load_state <= LD_IDLE;
                    end else if (boot_req_i || reload_req_i) begin
                        main_state        <= ST_LOAD;
                        load_state        <= LD_WAIT_PWR;
                        pwr_wait_cnt      <= '0;
                    end
                end

                ST_LOAD: begin
                    core_rst_o  <= 1'b1;
                    boot_done_o <= 1'b0;

                    if (debug_req_i) begin
                        main_state <= ST_DEBUG;
                        load_state <= LD_IDLE;
                        evt_ld_en  <= 1'b0;
                    end else begin
                        // unique
                        case (load_state)
                            LD_IDLE: begin
                                pwr_wait_cnt <= '0;
                                load_state   <= LD_WAIT_PWR;
                            end

                            LD_WAIT_PWR: begin
                                if (pwr_wait_cnt == PWR_WAIT_CYCLES-1) begin
                                    load_state <= LD_OPEN;
                                end else begin
                                    pwr_wait_cnt <= pwr_wait_cnt + 1'b1;
                                end
                            end

                            LD_OPEN: begin
                                evt_ld_en  <= 1'b1;
                                if (evt_ld_bypass)
                                    load_state <= LD_DONE;
                                else
                                    load_state <= LD_WAIT;
                            end

                            LD_WAIT: begin
                                if (evt_reads_done)
                                    load_state <= LD_DONE;
                            end

                            LD_DONE: begin
                                evt_ld_en   <= 1'b0;
                                boot_done_o <= 1'b1;
                                core_rst_o  <= 1'b0;
                                main_state  <= ST_RUN;
                                load_state  <= LD_IDLE;
                            end

                            LD_FAIL: begin
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
                    core_rst_o  <= 1'b0;
                    boot_done_o <= 1'b1;

                    if (debug_req_i) begin
                        main_state <= ST_DEBUG;
                        load_state <= LD_IDLE;
                        evt_ld_en  <= 1'b0;
                    end else if (reload_req_i) begin
                        main_state        <= ST_LOAD;
                        load_state        <= LD_WAIT_PWR;
                        boot_done_o       <= 1'b0;
                        boot_fail_o       <= 1'b0;
                        pwr_wait_cnt      <= '0;
                        evt_ld_en         <= 1'b0;
                    end
                end

                ST_DEBUG: begin
                    core_rst_o <= 1'b1;

                    if (!debug_req_i && (boot_req_i || reload_req_i)) begin
                        main_state        <= ST_LOAD;
                        load_state        <= LD_WAIT_PWR;
                        boot_done_o       <= 1'b0;
                        boot_fail_o       <= 1'b0;
                        pwr_wait_cnt      <= '0;
                        evt_ld_en         <= 1'b0;
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
